#include <stdlib.h>
#include <string.h>
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
#include "buzzer.h"

static inline bool _is_on_line(void) {
    return __builtin_popcount(sensing_ir_state & 0xFFFF);
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
volatile static int curve_decel = 16000; // 커브 감속 (작을 수록 곡선에서 감속을 많이 한다)
volatile static float curve_coef = 0.00008f; // 곡률 계수

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
    const int position_limited = sensing_ir_position_limited;

    // 곡선 감속
    float v_center = v_command / (1 + position_limited / (float)curve_decel);

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

void drive(const enum drive_t type) {
    static int mark_recover_enabled = 0;
    static float v_default = 3.0f;
    static float v_peak = 8.0f;
    static float fit_in = 0.18f;
    static float safe_distance = 0.2f;

    // 상수 설정
    DRIVE_SET_PARAMETER(v_default, "default velocity", "%1.2f", 0.1f);
    DRIVE_SET_PARAMETER(curve_coef, "curvature coefficient", "%1.6f", 0.00001f);
    DRIVE_SET_PARAMETER(curve_decel, "curve deceleration", "%5d", 1000);
    DRIVE_SET_PARAMETER(fit_in, "fit in", "%1.2f", 0.01f);
    if (type != DRIVE_FIRST) {
        DRIVE_SET_PARAMETER(v_peak, "peak velocity", "%1.2f", 0.1f);
        DRIVE_SET_PARAMETER(accel, "accel", "%1.2f", 1.0f);
        DRIVE_SET_PARAMETER(decel, "decel", "%1.2f", 1.0f);
        DRIVE_SET_PARAMETER(mark_recover_enabled, "mark recover", "%d", 1);
    }
    oled_clear();

    // [1차, n차 주행] 마크 및 거리 정보
    enum mark_t detected_mark[DRIVE_MARK_COUNT_MAX];
    uint detected_mark_count = 0;
    int32_t detected_tick[DRIVE_MARK_COUNT_MAX];
    int mark_index = 0;

    // [n차 주행] 직선 가속에 사용될 변수들
    enum mark_t previous_mark = MARK_BOTH;
    bool is_mark_valid = true;
    uint d_straight_end = 0;
    uint d_straight_start = 0;

    struct fs_data_t *const fs_data = fs_get_data();

    // [n차 주행] 플래시에서 마크와 거리 정보 불러오기
    if (type != DRIVE_FIRST) {
        detected_mark_count = fs_data->detected_mark_count;
        memcpy(detected_mark, fs_data->detected_mark, DRIVE_MARK_COUNT_MAX);
        memcpy(detected_tick, fs_data->detected_tick, DRIVE_MARK_COUNT_MAX);
    }

    // state machine 초기화
    struct mark_state_t mark_state = mark_init_state();
    uint line_out_state = DRIVE_LINE_OUT_STATE_IDLE;
    uint mark_end_count = 0;

    buzzer_init();
    sensing_start();
    drive_start();

    while (!drive_check_line_out(&line_out_state)) {
        buzzer_update();

        // mark state machine 업데이트
        mark_update_window(&mark_state, sensing_ir_position);
        const enum mark_t mark = mark_update_state(&mark_state);

        // 엔코더로부터 현재 위치 구하기
        const int32_t d_current =
            (abs(motor_get_encoder_value(MOTOR_LEFT)) + abs(motor_get_encoder_value(MOTOR_RIGHT))) / 2;

        if (type == DRIVE_FIRST) {
            /*
             * 1차 주행에서는, 기본 속도로 주행하고, 감지된 마크와 그 위치를 저장해둔다.
             */
            v_target = v_default;

            if (mark) {
                detected_mark[detected_mark_count] = mark; // 마크 기록
                detected_tick[detected_mark_count] = d_current; // 엔코더 값 기록
                detected_mark_count++;

                if (mark != MARK_CROSS) { // 크로스가 아닌 마크를 보면 부저를 울린다.
                    buzzer_out(80, false);
                }
            }
        } else {
            /*
             * n차 주행에서는, 똑같은 마크가 두 번 반복된다면 직선 구간이라고 판단하고,
             * 직선 구간의 시작 위치와, 종료 위치를 구해 놓는다.
             */
            if (mark && is_mark_valid && mark_index < detected_mark_count) {
                // 마크를 제대로 본 경우
                if (mark == detected_mark[mark_index]) {
                    // 이전에 본 마크와 현재 본 마크가 같다면 -> 직선 구간
                    if (mark == previous_mark && mark != MARK_CROSS) {
                        // 가속을 시작하는 위치 구하기
                        d_straight_start = d_current;
                        d_straight_start += safe_distance * MOTOR_TICK_PER_METER; // 안전 거리

                        // 가속을 끝내는 위치 구하기
                        int start = mark_index; // 직선 구간 시작 위치 인덱스
                        int end = mark_index + 1; // 직선 구간 종료 위치 인덱스
                        while (detected_mark[end] == MARK_CROSS && (end + 1) < detected_mark_count) {
                            end++; // 직선 구간 종료 위치 구하기: 크로스 구간은 건너 뛴다.
                        }
                        d_straight_end = d_current + (detected_tick[end] - detected_tick[start]);
                        d_straight_end -= safe_distance * MOTOR_TICK_PER_METER; // 안전 거리

                        // 직선 구간이 끝난 후 또 같은 마크가 나올 수 있으므로(이런 경우는 직선 구간이 아님) MARK_NONE으로 설정하여 예외 처리한다.
                        previous_mark = MARK_NONE;
                    } else {
                        previous_mark = mark;
                    }

                    mark_index++;
                    if (mark != MARK_CROSS) { // 크로스가 아닌 마크를 보면 부저를 울린다.
                        buzzer_out(80, false);
                    }
                }
                // 마크를 놓친 경우: 직선 가속 구간 설정 일단 중지
                else {
                    is_mark_valid = false;
                    previous_mark = MARK_NONE;
                    buzzer_out(1000, true);
                }
            }

            /*
             * 마크를 놓친 경우, 다음 크로스 마크가 올 때까지 기다린다(마크 복구).
             */
            if (mark_recover_enabled && mark == MARK_CROSS && !is_mark_valid) {
                is_mark_valid = true; // 일단 복구 성공했다고 가정

                while (detected_mark[++mark_index] != MARK_CROSS) { // 크로스 마크일 때까지 인덱스 증가
                    if (mark_index >= detected_mark_count) { // 마크 수를 넘겨 버리면 invalid
                        is_mark_valid = false;
                        break;
                    }
                }

                if (is_mark_valid) {
                    previous_mark = MARK_CROSS;
                    mark_index++;
                }
            }

            /*
             * 현재 라인트레이서가 가속 구간에 위치한다면, 가속을 진행한다.
             * 가속을 끝낼 때에는, 감속도에 의해 적절히 원래 속도로 되돌아 가야 한다.
             * 이를 위해 2as = (v' + v) * (v' - v) 식을 이용하여 현재 속도 v'에서 원래 속도 v로 되돌아 가기 위한 거리를 구한다.
             */
            const float decel_section = // Unit: m * (tick / m) -> tick
                ((v_command + v_default) * (v_command - v_default)) / (2.0f * decel) * MOTOR_TICK_PER_METER;

            if (d_straight_start < d_current && d_current < (d_straight_end - decel_section)) {
                v_target = v_peak; // 가속 구간: 최대 속도로 주행한다.
            } else {
                v_target = v_default; // 원래 속도로 되돌아 간다.
            }
        }

        if (mark) {
            if (mark == MARK_BOTH) { // 엔드 마크 증가
                mark_end_count++;
            }

            if (mark_end_count == 2) { // 엔드 마크를 두 번 봤으면 주행을 종료한다.
                decel = (v_command * v_command) / (2.0f * fit_in);
                drive_stop(false); // 속도가 0으로 떨어질 때까지 계속 주행한다.
                break;
            }
        }
    }
    drive_stop(true); // 모터 및 모터 컨트롤을 바로 중단한다.
    sensing_stop();

    oled_clear();
    if (mark_end_count == 2) {
        oled_printf("/0Drive finished/1by end mark.");
    } else {
        oled_printf("/0Drive finished");
    }

    if (type == DRIVE_FIRST) {
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
            fs_data->detected_mark_count = detected_mark_count;
            for (int i = 0; i < DRIVE_MARK_COUNT_MAX; i++) {
                if (i < detected_mark_count) {
                    fs_data->detected_mark[i] = detected_mark[i];
                    fs_data->detected_tick[i] = detected_tick[i];
                } else {
                    fs_data->detected_mark[i] = MARK_NONE;
                    fs_data->detected_tick[i] = 0;
                }
            }
            fs_flush_data();
        }
    }
}
