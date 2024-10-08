/**
 * @file motor.c
 * @author Seongho Lee (shythm@outlook.com)
 */

#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

#include "motor.h"
#include "sensing.h"
#include "timer.h"

/**
 * @brief 2개의 PIO(Programmable Input Ouput) 장치가 있다.
 * 이 장치를 이용하면 PIO 전용 명령어들을 이용하여 GPIO에 대해 간단한 연산들을 수행할 수 있는데,
 * 이는 CPU 자원을 이용하지 않고 PIO 장치에서 독립적으로 작동하기에 빠르고 효율적이다.
 * 우리는 하나의 PIO에 quadrature encoder 프로그램을 올리고 이를 이용하고자 한다.
 */
static const PIO encoder_pio = MOTOR_ENCODER_PIO;

static struct encoder_t {
    /**
     * @brief 엔코더의 한 쪽의 GPIO 번호.
     * 엔코더의 A상만 정의하는 이유는 PIO 프로그램에 의해 B상이 항상 A상 GPIO의 바로 다음 번호이기 때문이다.
     */
    const uint base_gpio;

    /**
     * @brief 엔코더 PIO의 state machine 번호.
     * 하나의 PIO에 총 4개의 state machine이 존재하는데,
     * 각 state machine은 동일한 PIO 프로그램에 대해 독립적인 상태를 가지는 인스턴스라고 보면 된다.
     */
    const uint sm;

    /**
     * @brief 엔코더 방향 보정 상수.
     * 모터의 회전 방향과 엔코더의 측정 방향이 반대일 수 있다.
     * 물리적인 축은 하나이고, 이에 종속된 두 기기(모터, 엔코더)가 방향을 서로 다르게 본다면 문제가 발생한다.
     * 가령, 모터는 1000 만큼 갔다고 생각했는데, 엔코더는 -1000 만큼 갔다고 인식하면 문제가 생기는 것.
     * 엔코더 값을 반환하는 함수를 호출할 때 이 값을 곱하여 보정한다.
     */
    const int32_t comp;

} encoder[MOTOR_COUNT] = {
    {
        .base_gpio = MOTOR_ENCODER_GPIO_LEFT_BASE,
        .sm = MOTOR_LEFT,
        .comp = MOTOR_ENCODER_COMP_LEFT,
    },
    {
        .base_gpio = MOTOR_ENCODER_GPIO_RIGHT_BASE,
        .sm = MOTOR_RIGHT,
        .comp = MOTOR_ENCODER_COMP_RIGHT,
    }
};

#include "quadrature_encoder.pio.h"

static inline void motor_encoder_init() {
    const uint instruction_offset = pio_add_program(encoder_pio, &quadrature_encoder_program);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        quadrature_encoder_program_init(
            encoder_pio, encoder[i].sm, instruction_offset, encoder[i].base_gpio, 0);
    }
}

int32_t motor_get_encoder_value(enum motor_index index) {
    int32_t value = quadrature_encoder_get_count(encoder_pio, encoder[index].sm);

    return encoder[index].comp * value;
}

static const uint pwm_slice_num = MOTOR_PWM_SLICE_NUM;
static const uint dir_gpio[MOTOR_COUNT] = {
    MOTOR_DIR_GPIO_LEFT, MOTOR_DIR_GPIO_RIGHT
};

static inline void motor_driver_init() {
    // PWM 및 시스템 클럭 정의, 이때 가청 주파수보다 높아야 귀에 거슬리는 소리가 나지 않는다.
    const uint32_t freq_pwm = 20000;
    const uint32_t freq_sys = clock_get_hz(clk_sys);

    /**
     * @brief PWM 주파수를 결정하기 위한 변수.
     * PWM의 카운터 레지스터가 계속 증가하면서 TOP에 도달하면 다시 초기화되는 형태로 동작한다.
     *
     * RP2040 datasheet 4.5.2.6절에 따르면 PWM 주파수는 다음과 같이 구한다.
     * freq_pwm = freq_sys / ( (TOP + 1) * (CSR_PH_CORRECT + 1) * ( DIV_INT + (DIV_FRAC / 16) ) )
     *
     * 1. CSR_PH_CORRECT는 카운터 레지스터가 TOP에 도달했을 때 0으로 떨어지는 것이 아니라 그대로 감소하는 설정을 말하며,
     *    우리는 이 기능을 사용하지 않기 때문에 0으로 둔다.
     * 2. TOP 레지스터 크기는 16비트로 기본적으로 65535 값을 가진다.
     * 3. DIV_INT 및 DIV_FRAC는 클럭을 나눌 때 사용하며, 1과 0으로 두어 사용하지 않는다.
     *
     * 위의 조건에 따라 TOP 레지스터를 좌항으로 두어 식을 정리하면,
     * TOP = freq_sys / freq_pwm - 1
     *
     * 예를 들어, freq_sys가 125,000,000Hz이고 freq_pwm이 20,000Hz이면 TOP은 6,250이 될 것이다.
     */
    const uint16_t top = freq_sys / freq_pwm - 1;

    // PWM 장치 기본 설정을 가져오고, PWM 주파수 설정
    pwm_config pwm_conf = pwm_get_default_config();
    pwm_config_set_wrap(&pwm_conf, top);

    // PWM 초기화
    pwm_init(pwm_slice_num, &pwm_conf, false);

    // (참고) RP2040에는 총 8개의 PWM slice가 존재하고, 각 slice마다 2개의 channel(GPIO)을 가지고 있다.
    // 양쪽 모터에 대해 PWM GPIO 설정
    gpio_set_function(MOTOR_PWM_GPIO_LEFT, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_PWM_GPIO_RIGHT, GPIO_FUNC_PWM);

    // DC 모터 드라이버의 direction 핀에 들어가는 GPIO를 초기화한다.
    // 모터 드라이버에 내장돼있는 H-Bridge 회로를 이용해 모터의 회전 방향을 변경하는데 사용된다.
    for (int i = 0; i < MOTOR_COUNT; i++) {
        gpio_init(dir_gpio[i]);
        gpio_set_dir(dir_gpio[i], GPIO_OUT);
        gpio_put(dir_gpio[i], false);
    }
}

void motor_pwm_enabled(const bool enabled) {
    pwm_set_chan_level(pwm_slice_num, MOTOR_LEFT, 0);
    pwm_set_chan_level(pwm_slice_num, MOTOR_RIGHT, 0);

    // PWM을 비활성화 할 때, counter 레지스터에 따라 해당 핀의 Low/High가 결정된다.
    // level을 0으로 두고, 해당 핀이 Low가 될 때까지 기다려준다. 약 1ms 정도.
    busy_wait_ms(1);
    pwm_set_enabled(pwm_slice_num, enabled);
}

void motor_set_pwm_duty_ratio(enum motor_index index, float duty_ratio) {
    const uint16_t level_max = pwm_hw->slice[pwm_slice_num].top;

    int level = abs(duty_ratio * level_max);
    if (level > level_max) { // 오버플로우 방지
        level = level_max;
    } else if (level < 0) { // 언더플로우 방지
        level = 0;
    }
    pwm_set_chan_level(pwm_slice_num, index, (uint16_t)level); // PWM 인가

    gpio_put(dir_gpio[index], duty_ratio > 0.f); // 방향 설정
}

static struct motor_control_state_t control_state[MOTOR_COUNT];

struct motor_control_state_t motor_get_control_state(enum motor_index index) {
    return control_state[index];
}

/**
 * @brief 위치에 대한 목표값과 현재값의 차이(오차)를 계산하여 모터가 목표값까지 도달하기 위해 필요한 전압을 계산한다.
 */
static inline void motor_control_dt(const enum motor_index index) {
    static const float dt_s = (float)MOTOR_CONTROL_INTERVAL_US / (1000 * 1000);
    struct motor_control_state_t *const state = &control_state[index];

    // 엔코더를 이용해 현재 모터의 위치를 구한다.
    state->current = motor_get_encoder_value(index);

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
    const float error_diff = error - state->error;

    // 오차와 오차의 차이에 각각 비례 상수를 곱하고 이들의 합을 구해 최종적으로 모터에 인가될 전압을 계산한다.
    const float voltage = state->gain_p * error + state->gain_d * error_diff;

    // PWM duty ratio 계산 및 적용
    const float duty_ratio = voltage / sensing_supply_voltage;
    motor_set_pwm_duty_ratio(index, duty_ratio); // PWM 출력

    // 다음 미분항 계산을 위해 오차를 저장해둔다.
    state->error = error;
}

// 모터 위치 PID 제어 시 목표량(Set Point, SP)을 결정하는 함수
static motor_target_updater_t target_updater = NULL;

static void motor_control_handler(void) {
    // 매 주기마다 목표값을 업데이트하기 위해 외부 함수를 호출한다.
    if (target_updater) {
        target_updater(
            &control_state[MOTOR_LEFT].target, &control_state[MOTOR_RIGHT].target);
    }

    // 매 주기마다 PID 제어를 실시한다.
    motor_control_dt(MOTOR_LEFT);
    motor_control_dt(MOTOR_RIGHT);
}

void motor_control_start(const motor_target_updater_t updater) {
    motor_pwm_enabled(true);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        struct motor_control_state_t *const state = &control_state[i];

        state->gain_p = MOTOR_CONTROL_GAIN_P;
        state->gain_d = MOTOR_CONTROL_GAIN_D;
        state->target = state->current = motor_get_encoder_value(i);
        state->error = 0;
    }

    target_updater = updater;
    timer_periodic_start(
        MOTOR_CONTROL_TIMER_SLOT, MOTOR_CONTROL_INTERVAL_US, motor_control_handler);
}

void motor_control_stop(void) {
    timer_periodic_stop(MOTOR_CONTROL_TIMER_SLOT);
    motor_pwm_enabled(false);
}

void motor_init(void) {
    motor_driver_init();
    motor_encoder_init();
}
