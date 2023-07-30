#include "drive.h"
#include <stdlib.h>

#include "pico.h"
#include "pico/types.h"
#include "motor.h"
#include "timer.h"
#include "oled.h"
#include "switch.h"

#include "sensing.h"
#include "config.h"

/**
 * @brief 마크의 종류를 나타내는 타입
 */
typedef uint mark_t;
#define MARK_NONE  (mark_t)0x00 // 0000: 마크를 보지 않음 (기본)
#define MARK_LEFT  (mark_t)0x01 // 0001: 왼쪽 마크를 봄
#define MARK_RIGHT (mark_t)0x02 // 0010: 오른쪽 마크를 봄
#define MARK_BOTH  (mark_t)0x03 // 0011: 왼쪽, 오른쪽 마크 둘 다 봄
#define MARK_CROSS (mark_t)0x04 // 0100: 크로스 영역을 지나침

/**
 * @brief 마크를 판단하는 State Machine의 현재 상태를 나타내는 타입
 */
typedef uint mark_state_t;
#define MARK_STATE_IDLE     (mark_state_t)0x00
#define MARK_STATE_CROSS    (mark_state_t)0x01
#define MARK_STATE_MARKER   (mark_state_t)0x04
#define MARK_STATE_DECISION (mark_state_t)0x08
#define MARK_STATE_ERROR    (mark_state_t)0x10

/**
 * @brief 센서 상태(sensor_state)로부터 어떤 마크인지 판단하기 위해 사용되는 비트 마스크를 저장하는 구조체
 *        16조 센서의 경우 마크를 판단하는 기준(비트 마스크)이 바뀔 수 있다.
 */
typedef struct {
    sensing_ir_state_t left, right, both, center, all;
} mark_mask_t;
#define MARK_MASK_DEFAULT_LEFT   (sensing_ir_state_t)0xF000 // 1111 0000 0000 0000
#define MARK_MASK_DEFAULT_RIGHT  (sensing_ir_state_t)0x000F // 0000 0000 0000 1111
#define MARK_MASK_DEFAULT_BOTH   (sensing_ir_state_t)0xF00F // 1111 0000 0000 1111
#define MARK_MASK_DEFAULT_CENTER (sensing_ir_state_t)0x0FF0 // 0000 1111 1111 0000
#define MARK_MASK_DEFAULT_ALL    (sensing_ir_state_t)0xFFFF // 1111 1111 1111 1111
static mark_mask_t mark_mask = {
    .left = MARK_MASK_DEFAULT_LEFT,
    .right = MARK_MASK_DEFAULT_RIGHT,
    .both = MARK_MASK_DEFAULT_BOTH,
    .center = MARK_MASK_DEFAULT_CENTER,
    .all = MARK_MASK_DEFAULT_ALL,
};

/**
 * @brief
 *
 * @param state
 * @return mark_t
 */
static mark_t mark_update_state(mark_state_t *state) {
    static sensing_ir_state_t accumulate;
    accumulate |= sensing_get_ir_state(); // 마크 판단을 위해 센서 상태를 누적한다

    bool is_line_6 = __builtin_popcount(mark_mask.center & sensing_get_ir_state()) >= 6;
    bool is_any = sensing_get_ir_state() & mark_mask.both;

    switch (*state) {
    case MARK_STATE_IDLE: // 일반 주행 중일 때
        accumulate = 0;

        if (is_line_6) {
            *state = MARK_STATE_CROSS;
        } else if (is_any) {
            *state = MARK_STATE_MARKER;
        }
        break;

    case MARK_STATE_CROSS: // 라인 교차 구간을 지나는 중일 때
        /**
         * is_line_4를 검사하는 이유? 먼저 나와있는 센서들이 크로스 구간을 빠져 나왔는지 판단하기 위해
         * accumulate를 검사하는 이유? 모든 센서들이 인식됐음은 곧 크로스 구간을 지나기 직전임을 의미한다.
         * is_any를 검사하는 이유? 처리 속도가 너무 빠른 나머지 뒤로 나와있는 센서들이 아직 크로스 위에 있을 수 있으므로 이 조건까지 검사한다.
         */
        if ((!is_line_6) && (accumulate == mark_mask.all) && (!is_any)) {
            *state = MARK_STATE_DECISION;
        }
        break;

    case MARK_STATE_MARKER: // 마커 인식 중일 때
        if (!is_any) {
            *state = MARK_STATE_DECISION;
        }
        break;

    case MARK_STATE_DECISION: // 판단 단계
        *state = MARK_STATE_IDLE;

        if (accumulate == mark_mask.all) {
            return MARK_CROSS;
        }
        bool is_left = accumulate & mark_mask.left;
        bool is_right = accumulate & mark_mask.right;
        if (is_left && is_right) {
            return MARK_BOTH;
        }
        if (is_left) {
            return MARK_LEFT;
        }
        if (is_right) {
            return MARK_RIGHT;
        }

        *state = MARK_STATE_ERROR;
        break;

    case MARK_STATE_ERROR: // 이 단계에 오면 무언가 잘못 된 것임
        *state = MARK_STATE_IDLE;
        break;
    }

    return MARK_NONE;
}

void mark_test(void) {
    mark_state_t mark_state = MARK_STATE_IDLE;

    oled_clear_all();
    while (!switch_read_wait_ms(1)) {
        mark_t mark = mark_update_state(&mark_state);

        switch (mark_state) {
        case MARK_STATE_IDLE:
            oled_printf("/0STATE: IDLE ");
            break;
        case MARK_STATE_CROSS:
            oled_printf("/0STATE: CROSS");
            break;
        case MARK_STATE_MARKER:
            oled_printf("/0STATE: MARK ");
            break;
        default:
            oled_printf("/0STATE: -----");
            break;
        }

        switch (mark) {
        case MARK_LEFT:
            oled_printf("/1MARK: LEFT  ");
            break;
        case MARK_RIGHT:
            oled_printf("/1MARK: RIGHT ");
            break;
        case MARK_BOTH:
            oled_printf("/1MARK: BOTH  ");
            break;
        case MARK_CROSS:
            oled_printf("/1MARK: CROSS ");
            break;
        }
    }
}

#define DRIVE_TIMER_SLOT        TIMER_SLOT_2
#define DRIVE_TIMER_INTERVAL_US 500

static volatile struct drive_control {
    float velocity_command[MOTOR_COUNT];
    float velocity_target[MOTOR_COUNT];
    float acceleration;
    float deceleration;
} _control = {
    .velocity_command = { 0.0f, 0.0f },
    .velocity_target = { 0.0f, 0.0f },
    .acceleration = 4.0f,
    .deceleration = 6.0f,
};

/**
 * @brief 모터의 가감속 제어를 수행한다. 일정 주기로 호출돼 가감속 제어를 구현한다.
 *
 * @param index 왼쪽 모터(MOTOR_DC_LEFT) 또는 오른쪽 모터(MOTOR_DC_RIGHT)
 */
static inline void drive_control_acceleration_dt(enum motor_index index) {
    static const float dt_s = (DRIVE_TIMER_INTERVAL_US * 0.001) * 0.001;

    float command = _control.velocity_command[index]; // 지령 속도 가져옴
    float target = _control.velocity_target[index]; // 외부로부터 지정된 목표 속도

    if (command < target) {
        command += _control.acceleration * dt_s;
        if (command > target) { // limit
            command = target;
        }
    } else if (command > target) {
        command -= _control.deceleration * dt_s;
        if (command < target) { // limit
            command = target;
        }
    }

    _control.velocity_command[index] = command; // 지령 속도 저장
}

/**
 * @brief 모터 제어 시 호출되는 함수를 정의한다. 모터 제어를 시작할 때 함수의 주소를 전달한다.
 *
 * @param left 왼쪽 모터 지령 속도 포인터
 * @param right 오른쪽 모터 지령 속도 포인터
 */
void drive_velocity_commander(int32_t *const left, int32_t *const right) {
    const static float dt_s = (float)MOTOR_CONTROL_INTERVAL_US / (1000 * 1000);

    *left += _control.velocity_command[MOTOR_LEFT] * dt_s;
    *right += _control.velocity_command[MOTOR_RIGHT] * dt_s;
}

/**
 * @brief 모터의 가감속 제어를 수행하는 타이머 IRQ Handler
 */
static void drive_control_acceleration_handler(void) {
    drive_control_acceleration_dt(MOTOR_LEFT);
    drive_control_acceleration_dt(MOTOR_RIGHT);
}

/**
 * @brief
 *
 * @param enabled
 */
static void drive_control_enabled(bool enabled) {
    if (enabled) {
        motor_control_start(drive_velocity_commander);
        timer_periodic_start(DRIVE_TIMER_SLOT, DRIVE_TIMER_INTERVAL_US, drive_control_acceleration_handler);
    } else {
        _control.velocity_target[MOTOR_LEFT] = 0.0f;
        _control.velocity_target[MOTOR_RIGHT] = 0.0f;

        for (;;) {
            float command_avg =
                (_control.velocity_command[MOTOR_LEFT] + _control.velocity_command[MOTOR_RIGHT]) / 2;

            if (command_avg < 0.1f) {
                break;
            }
        }

        _control.velocity_command[MOTOR_LEFT] = 0.0f;
        _control.velocity_command[MOTOR_RIGHT] = 0.0f;

        timer_periodic_stop(DRIVE_TIMER_SLOT);
        motor_control_stop();
    }
}

static inline bool drive_is_on_line(void) {
    return __builtin_popcount(sensing_get_ir_state() & mark_mask.all);
}

static inline void drive_set_velocity(enum motor_index index, float velocity) {
    _control.velocity_target[index] = velocity;
}

static inline float drive_get_velocity(void) {
    return (_control.velocity_command[MOTOR_LEFT] + _control.velocity_command[MOTOR_RIGHT]) / 2;
}

void drive_first(void) {
    static float velocity = 2.0f;

    static int curve_decel = 8000; // 커브 감속 (작을 수록 곡선에서 감속을 많이 한다)
    static float curve_coef = 0.00007f; // 곡률 계수

    oled_clear_all();
    for (;;) {
        oled_printf("/0velo:");
        oled_printf("/1%1.2f  ", velocity);

        uint sw = switch_read_wait_ms(100);
        if (sw == SWITCH_EVENT_BOTH)
            break;
        else if (sw == SWITCH_EVENT_LEFT)
            velocity -= 0.1;
        else if (sw == SWITCH_EVENT_RIGHT)
            velocity += 0.1;
    }

    oled_clear_all();
    for (;;) {
        oled_printf("/0curv:");
        oled_printf("/1%1.6f  ", curve_coef);

        uint sw = switch_read_wait_ms(100);
        if (sw == SWITCH_EVENT_BOTH)
            break;
        else if (sw == SWITCH_EVENT_LEFT)
            curve_coef -= 0.00001;
        else if (sw == SWITCH_EVENT_RIGHT)
            curve_coef += 0.00001;
    }

    oled_clear_all();
    for (;;) {
        oled_printf("/0decel:");
        oled_printf("/1%5d ", curve_decel);

        uint sw = switch_read_wait_ms(100);
        if (sw == SWITCH_EVENT_BOTH)
            break;
        else if (sw == SWITCH_EVENT_LEFT)
            curve_decel -= 1000;
        else if (sw == SWITCH_EVENT_RIGHT)
            curve_decel += 1000;
    }

    mark_state_t mark_state = MARK_STATE_IDLE;
    uint mark_end_count = 0;

    drive_control_enabled(true);
    while (drive_is_on_line()) {

        mark_t mark = mark_update_state(&mark_state);
        if (mark == MARK_BOTH) {
            mark_end_count++;
        }

        // 곡선 감속
        float velocity_center = velocity / (1 + abs(sensing_get_position()) / (float)curve_decel);

        // 좌우 모터 속도 조절
        float kp = curve_coef * sensing_get_position();
        drive_set_velocity(MOTOR_LEFT, velocity_center * (1.f - kp));
        drive_set_velocity(MOTOR_RIGHT, velocity_center * (1.f + kp));

        if (mark_end_count == 2) {
            // fit-in 2as = 나중속도^2 - 처음속도^2

            _control.deceleration = drive_get_velocity() * drive_get_velocity() / (2 * 0.3);
            drive_set_velocity(MOTOR_LEFT, 0.0f);
            drive_set_velocity(MOTOR_RIGHT, 0.0f);
            while (drive_get_velocity() > 0.1f) {
                tight_loop_contents();
            }
            break;
        }
    }

    sleep_ms(100);
    drive_control_enabled(false);
}
