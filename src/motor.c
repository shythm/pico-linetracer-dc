/**
 * @file motor.c
 * @author Seongho Lee (shythm@outlook.com)
 *
 * @brief DC 모터 제어를 위해
 * 1. 모터 관련 주변 장치 초기화, 2. 모터 위치 제어 코드를 서술한다.
 */
#include <stdlib.h>
#include "motor.h"
#include "sensing.h"
#include "timer.h"

#define PWM_LEFT_GPIO        (14)
#define PWM_RIGHT_GPIO       (15)
#define DIRECTION_LEFT_GPIO  (16)
#define DIRECTION_RIGHT_GPIO (17)
#define ENCODER_PIO          (pio0)
#define ENCODER_BASE_GPIO \
    { 10, 12 }
#define ENCODER_REVERSE \
    { true, true }

#define PI                 (3.141592f)
#define ENCODER_RESOLUTION (2048)
#define WHEEL_DIAMETER_M   (0.038f) // 바퀴의 지름(m)
#define GEAR_RATIO         (17.f / 69.f) // 모터(17) / 바퀴(69) 기어비

#define CONTROL_TIMER_SLOT  (TIMER_SLOT_1)
#define CONTROL_INTERVAL_US (500)
#define CONTROL_GAIN_P      (0.04f)
#define CONTROL_GAIN_D      (0.02f)

static const int dircomp[MOTOR_COUNT] = { -1, 1 }; // 모터 방향 보정 상수 (1 또는 -1)
static struct motor_pwm_t pwm[MOTOR_COUNT];
static struct motor_encoder_t encoder;
static struct motor_control_state_t control[MOTOR_COUNT];

/**
 * @brief 위치에 대한 목표값과 현재값의 차이(오차)를 계산하여 모터가 목표값까지 도달하기 위해 필요한 전압을 계산한다.
 */
static inline void motor_control_dt(const enum motor_index index) {
    static const float dt_s = (float)CONTROL_INTERVAL_US / (1000 * 1000);
    struct motor_control_state_t *const state = &control[index];

    /**
     * 매 주기마다 속도에 따른 목표 위치 값을 더해준다.
     * 이때 모터의 방향을 고려한다(한쪽 모터는 반대로 돌아야 하는데, 그러기 위해서는 그냥 위치를 반대 부호(음수)로 주면 된다).
     */
    static const float tick_per_meter = ENCODER_RESOLUTION / (WHEEL_DIAMETER_M * PI) / GEAR_RATIO; // 약 69,630
    state->target += dircomp[index] * tick_per_meter * (state->velocity * dt_s); // 2 m/s - 35 tick/interval

    // 엔코더를 이용해 현재 모터의 위치를 구한다.
    state->current = motor_encoder_get_value(&encoder, index);

    /**
     * 위치에 대한 목표값과 현재값의 차이(오차)를 계산 - 비례항을 위해
     * 도달하려는 위치(목표)와 멀어질 수록 더 큰 제어값을 전달하게 될 것.
     */
    const int32_t error = state->target - state->current;

    /**
     * 현재 오차와 이전 오차의 차이를 계산 - 미분항을 위해
     * Case 1: "현재 오차 > 이전 오차"인 경우, 양의 제어값으로 목표치에 더 빠르게 도달하도록 함.
     * Case 2: "현재 오차 < 이전 오차"인 경우, 음의 제어값으로 Overshoot 현상을 방지함.
     * Case 3: "현재 오차 = 이전 오차"인 경우, 아무런 제어를 하지 않을 것.
     */
    const float error_diff = (error - state->error) / dt_s;

    // 오차와 오차의 차이에 각각 비례 상수를 곱하고 이들의 합을 구해 최종적으로 모터에 인가될 전압을 계산한다.
    // const float voltage = state->gain_p * error + state->gain_d * error_diff;
    const float voltage = state->gain_p * error;

    // PWM duty ratio 계산 및 적용
    const float duty_ratio = voltage / sensing_get_supply_voltage();
    motor_pwm_set_direction(&pwm[index], duty_ratio > 0.f); // 모터 회전 방향 설정
    motor_pwm_set_duty_ratio(&pwm[index], duty_ratio); // PWM 출력

    // 다음 미분항 계산을 위해 오차를 저장해둔다.
    state->error = error;
}

static void motor_control_handler(void) {
    motor_control_dt(MOTOR_LEFT);
    motor_control_dt(MOTOR_RIGHT);
}

inline static void motor_reset_control_state(enum motor_index index) {
    struct motor_control_state_t *const _control = &control[index];

    _control->velocity = 0.0f;
    _control->gain_p = CONTROL_GAIN_P;
    _control->gain_d = CONTROL_GAIN_D;
    _control->current = motor_encoder_get_value(&encoder, index);
    _control->target = _control->current;
    _control->error = 0;
}

void motor_start(void) {
    motor_pwm_enabled(&pwm[MOTOR_LEFT], true);
    motor_pwm_enabled(&pwm[MOTOR_RIGHT], true);
    motor_reset_control_state(MOTOR_LEFT);
    motor_reset_control_state(MOTOR_RIGHT);

    timer_periodic_start(CONTROL_TIMER_SLOT, CONTROL_INTERVAL_US, motor_control_handler);
}

void motor_stop(void) {
    timer_periodic_stop(CONTROL_TIMER_SLOT);

    motor_pwm_enabled(&pwm[MOTOR_LEFT], false);
    motor_pwm_enabled(&pwm[MOTOR_RIGHT], false);
}

void motor_set_velocity(const enum motor_index index, float velocity) {
    control[index].velocity = velocity;
}

struct motor_control_state_t motor_get_control_state(const enum motor_index index) {
    return control[index];
}

void motor_init(void) {
    pwm[MOTOR_LEFT] = motor_pwm_init(PWM_LEFT_GPIO, DIRECTION_LEFT_GPIO);
    pwm[MOTOR_RIGHT] = motor_pwm_init(PWM_RIGHT_GPIO, DIRECTION_RIGHT_GPIO);

    uint encoder_gpios[MOTOR_COUNT] = ENCODER_BASE_GPIO;
    bool encoder_reverse[MOTOR_COUNT] = ENCODER_REVERSE;
    encoder = motor_encoder_init(ENCODER_PIO, encoder_gpios, encoder_reverse);
}

#include "switch.h"
#include "oled.h"
void motor_pwm_test(void) {
    float duty_ratio = 0;

    motor_pwm_enabled(&pwm[MOTOR_LEFT], true);
    motor_pwm_enabled(&pwm[MOTOR_RIGHT], true);

    for (;;) {
        uint sw = switch_read_wait_ms(100);
        struct motor_control_state_t csl = motor_get_control_state(MOTOR_LEFT);
        struct motor_control_state_t csr = motor_get_control_state(MOTOR_RIGHT);

        if (sw == SWITCH_EVENT_LEFT)
            duty_ratio -= 0.1f;
        else if (sw == SWITCH_EVENT_RIGHT)
            duty_ratio += 0.1f;
        else if (sw == SWITCH_EVENT_BOTH)
            break;

        motor_pwm_set_direction(&pwm[MOTOR_LEFT], duty_ratio > 0.0f);
        motor_pwm_set_duty_ratio(&pwm[MOTOR_LEFT], duty_ratio);
        motor_pwm_set_direction(&pwm[MOTOR_RIGHT], duty_ratio > 0.0f);
        motor_pwm_set_duty_ratio(&pwm[MOTOR_RIGHT], duty_ratio);

        oled_printf("/0PWM Test");
        oled_printf("/1duty ratio/2%1.2f", duty_ratio);
    }

    motor_pwm_enabled(&pwm[MOTOR_LEFT], false);
    motor_pwm_enabled(&pwm[MOTOR_RIGHT], false);
}