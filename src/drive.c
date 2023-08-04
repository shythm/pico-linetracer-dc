#include <stdlib.h>
#include <math.h>
#include "pico.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"

#include "drive.h"
#include "oled.h"
#include "switch.h"
#include "sensing.h"
#include "motor.h"
#include "mark.h"
#include "fs.h"

static uint32_t buzzer_timer = 0;

static void drive_buzzer_init(void) {
    gpio_init(DRIVE_BUZZER_GPIO);
    gpio_set_dir(DRIVE_BUZZER_GPIO, GPIO_OUT);
    gpio_pull_down(DRIVE_BUZZER_GPIO);

    gpio_put(DRIVE_BUZZER_GPIO, false);
}

static inline void drive_buzzer_update() {
    gpio_put(DRIVE_BUZZER_GPIO, time_us_32() < buzzer_timer);
}

static inline void drive_buzzer_out(uint32_t time_ms) {
    buzzer_timer = time_us_32() + (time_ms * 1000);
}

static inline bool _is_on_line(void) {
    return __builtin_popcount(sensing_ir_state & MARK_MASK_ALL);
}

#define DRIVE_LINE_OUT_STATE_IDLE 0x00
#define DRIVE_LINE_OUT_STATE_TRIG 0x01
#define DRIVE_LINE_OUT_STATE_EXIT 0x04

static bool drive_check_line_out(uint *state) {
    static uint32_t time;

    switch (*state) {
    case DRIVE_LINE_OUT_STATE_IDLE:
        if (!_is_on_line()) {
            *state = DRIVE_LINE_OUT_STATE_TRIG;
            time = time_us_32();
        }
        break;

    case DRIVE_LINE_OUT_STATE_TRIG:
        if (_is_on_line()) {
            *state = DRIVE_LINE_OUT_STATE_IDLE;
        } else {
            if (time_us_32() - time > DRIVE_LINE_OUT_TIME_US) {
                *state = DRIVE_LINE_OUT_STATE_EXIT;
            }
        }
        break;

    case DRIVE_LINE_OUT_STATE_EXIT:
        *state = DRIVE_LINE_OUT_STATE_IDLE;
        return true;
    }

    return false;
}

volatile static float v_command = 0.0f; // 지령 속도: 모터에 직접 인가되는 속도
volatile static float v_target = 0.0f; // 목표 속도: 가감속도 제어의 목표 속도
volatile static float accel = 4.0f; // 가속도
volatile static float decel = 6.0f; // 감속도
volatile static int curve_decel = 24000; // 커브 감속 (작을 수록 곡선에서 감속을 많이 한다)
volatile static float curve_coef = 0.00007f; // 곡률 계수

/**
 * @brief 모터 제어 시 호출되는 함수를 정의한다. 모터 제어를 시작할 때 이 함수를 전달한다.
 * 1. 가감속도 제어를 수행한다.
 * 2. position에 따른 곡선 감속 제어를 수행한다.
 * 3. 모터 좌우 속도를 position에 따라 제어한다.
 *
 * @param left 왼쪽 모터 지령 속도 포인터
 * @param right 오른쪽 모터 지령 속도 포인터
 */
static void drive_velocity_commander(int32_t *const left, int32_t *const right) {
    const static float dt_s = (float)MOTOR_CONTROL_INTERVAL_US / (1000 * 1000);

    // 가감속도 제어
    if (v_command < v_target) {
        v_command += accel * dt_s;
        if (v_command > v_target) { // limit
            v_command = v_target;
        }
    } else if (v_command > v_target) {
        v_command -= decel * dt_s;
        if (v_command < v_target) { // limit
            v_command = v_target;
        }
    }

    const int position = sensing_ir_position;

    // 곡선 감속
    float v_center = v_command / (1 + abs(position) / (float)curve_decel);

    // 좌우 모터 속도 결정
    float kp = curve_coef * position;
    float v_left = v_center * (1.f - kp);
    float v_right = v_center * (1.f + kp);

    // 좌우 모터 속도 인가
    *left -= MOTOR_TICK_PER_METER * v_left * dt_s;
    *right += MOTOR_TICK_PER_METER * v_right * dt_s;
}

/**
 * @brief 트레이서 주행에 필요한 상태 변수들을 초기화하고, 주행을 시작한다.
 */
static void drive_start(void) {
    v_command = 0.0f;
    v_target = 0.0f;
    motor_control_start(drive_velocity_commander);
}

/**
 * @brief 트레이서 주행을 중단한다.
 * 기본적으로 지령 속도가 0에 가까이 떨어질 때까지 기다린다. 이를 원치 않으면 force 매개변수를 참으로 둔다.
 *
 * @param force 주행 강제 중단 여부
 */
static void drive_stop(bool force) {
    if (force) {
        motor_control_stop();
        return;
    }

    v_target = 0.0f;
    while (v_command > 0.1f) {
        tight_loop_contents();
    }
    busy_wait_ms(100);
    motor_control_stop();
}

#define DRIVE_SET_PARAMETER(param, name, format, delta)                   \
    oled_clear();                                                         \
    for (;;) {                                                            \
        oled_printf("/0/gSet Parameter/1/w" name "/2:=" format, (param)); \
        enum switch_event_t sw = switch_read();                           \
        if (sw == SWITCH_EVENT_BOTH)                                      \
            break;                                                        \
        else if (sw == SWITCH_EVENT_LEFT)                                 \
            (param) -= (delta);                                           \
        else if (sw == SWITCH_EVENT_RIGHT)                                \
            (param) += (delta);                                           \
    }

/**
 * @brief 1차 주행
 */
void drive_first(void) {
    float v_default = 2.0f;

    DRIVE_SET_PARAMETER(v_default, "default velocity", "%1.2f", 0.1f);
    DRIVE_SET_PARAMETER(curve_coef, "curvature coefficient", "%1.6f", 0.00001f);
    DRIVE_SET_PARAMETER(curve_decel, "curve deceleration", "%5d", 1000);

    enum mark_t detected_mark[DRIVE_MARK_COUNT_MAX];
    uint detected_mark_count = 0;
    uint detected_tick[DRIVE_MARK_COUNT_MAX];

    struct mark_state_t mark_state =
        mark_init_state(MARK_MASK_LEFT_DEFAULT, MARK_MASK_RIGHT_DEFAULT);
    uint line_out_state = DRIVE_LINE_OUT_STATE_IDLE;
    uint mark_end_count = 0;

    drive_buzzer_init();
    oled_clear();
    sensing_start();
    drive_start();

    v_target = v_default;
    while (!drive_check_line_out(&line_out_state)) {

        drive_buzzer_update();
        enum mark_t mark = mark_update_state(&mark_state);

        if (mark) {
            detected_mark[detected_mark_count] = mark; // 마크 기록
            detected_tick[detected_mark_count] // 엔코더 값 기록
                = (motor_get_encoder_value(MOTOR_LEFT) + motor_get_encoder_value(MOTOR_RIGHT)) / 2;
            detected_mark_count++;

            if (mark != MARK_CROSS) {
                drive_buzzer_out(80);
            }

            if (mark == MARK_BOTH) { // 엔드 마크 증가
                mark_end_count++;
            }
        }

        if (mark_end_count == 2) {
            decel = powf(v_command, 2) / (2.0f * 0.25f);
            drive_stop(false);
            break;
        }
    }
    drive_stop(true);
    sensing_stop();

    oled_clear();
    if (mark_end_count == 2) {
        oled_printf("/0Drive finished/1by end mark.");
    } else {
        oled_printf("/0Drive finished");
    }

    // 마크 개수 확인
    uint left_count = 0, right_count = 0, cross_count = 0;
    for (int i = 0; i < detected_mark_count; i++) {
        if (detected_mark[i] == MARK_LEFT) {
            left_count++;
        } else if (detected_mark[i] == MARK_RIGHT) {
            right_count++;
        } else if (detected_mark[i] == MARK_CROSS) {
            cross_count++;
        }
    }
    oled_printf("/2/rL/w%3u /bR/w%3u /gC/w%3u", left_count, right_count, cross_count);

    // 마크 플래시 저장
    oled_printf("/3Do you want/4to /gsave/w mark?/5 (YES // NO)");
    enum switch_event_t sw = switch_wait_until_input();
    if (sw == SWITCH_EVENT_LEFT) {
        oled_printf("/6Saving ...");
        struct fs_data_t *fs = fs_get_data();
        fs->detected_mark_count = detected_mark_count;
        for (int i = 0; i < detected_mark_count; i++) {
            fs->detected_mark[i] = detected_mark[i];
            fs->detected_tick[i] = detected_tick[i];
        }
        fs_flush_data();
    }
}
