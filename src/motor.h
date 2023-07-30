/**
 * @file motor.h
 * @author Seongho Lee (shythm@outlook.com)
 *
 * @brief DC 모터 제어를 위한
 * 1. PWM 초기화/제어, 2. Encoder 초기화/제어 관련 기본 코드를 서술한다.
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdlib.h>
#include "pico/types.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

enum motor_index {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT,
};

/**
 * @brief 엔코더를 이용해 현재 모터의 위치를 반환한다.
 *
 * @param index 왼쪽 모터(MOTOR_LEFT) 또는 오른쪽 모터(MOTOR_RIGHT)
 * @return int32_t 현재 모터의 위치
 */
int32_t motor_get_encoder_value(enum motor_index index);

struct motor_pwm_t {
    uint gpio;
    uint direction_gpio;
    uint slice_num;
    uint channel;
};

static inline struct motor_pwm_t motor_pwm_init(const uint gpio, const uint direction_gpio) {
    // PWM 및 시스템 클럭 정의, 이때 가청 주파수보다 높아야 귀에 거슬리는 소리가 나지 않는다.
    const uint32_t freq_pwm = 20000;
    const uint32_t freq_sys = clock_get_hz(clk_sys);

    /**
     * RP2040 datasheet 4.5.2.6절에 따르면 PWM 주파수는 다음과 같이 구한다.
     * freq_pwm = freq_sys / ( (TOP + 1) * (CSR_PH_CORRECT + 1) * ( DIV_INT + (DIV_FRAC / 16) ) )
     *
     * 1. CSR_PH_CORRECT는 카운터 레지스터가 TOP에 도달했을 때 0으로 떨어지는 것이 아니라 그대로 감소하는 설정을 말하며,
     *    우리는 이 기능을 사용하지 않기 때문에 0으로 둔다.
     * 2. TOP 레지스터 크기는 16비트로 기본적으로 65535 값을 가진다.
     * 3. DIV_INT 및 DIV_FRAC는 클럭을 나눌 때 사용하며, 1과 0으로 두어 사용하지 않는다.
     *
     * 위의 조건에 따라 TOP 레지스터에 따른 식으로 정리하면,
     * TOP = freq_sys / freq_pwm - 1
     *
     * 예를 들어, freq_sys가 125,000,000Hz이고 freq_pwm이 20,000Hz이면 TOP은 6,250이다.
     */
    const float clkdiv = (float)(freq_sys / freq_pwm) / 65536 * 16;
    const uint16_t top = freq_sys / freq_pwm - 1;

    // pwm 장치 기본 설정을 가져오고, PWM 주파수 설정
    pwm_config pwm_conf = pwm_get_default_config();
    pwm_config_set_wrap(&pwm_conf, top);

    /**
     * RP2040에는 총 8개의 PWM slice가 존재하고, 각 slice마다 2개의 channel을 가지고 있어 총 16개의 PWM 신호를 만들 수 있다.
     * 각 GPIO마다 PWM slice 번호와 channel은 pwm_gpio_to_slice_num 함수와 pwm_gpio_to_channel 함수를 통해 구할 수 있다.
     */
    const uint slice_num = pwm_gpio_to_slice_num(gpio);
    const uint channel = pwm_gpio_to_channel(gpio);

    // DC 모터 드라이버에 들어갈 PWM 및 GPIO 초기화
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm_init(slice_num, &pwm_conf, false);

    /**
     * DC 모터 드라이버의 direction 핀에 들어가는 GPIO를 초기화한다.
     * 모터 드라이버에 내장돼있는 H-Bridge 회로를 이용해 모터의 회전 방향을 변경하는데 사용된다.
     */
    gpio_init(direction_gpio);
    gpio_set_dir(direction_gpio, GPIO_OUT);
    gpio_put(direction_gpio, false);

    // PWM 정보 반환
    struct motor_pwm_t ret = {
        .gpio = gpio,
        .direction_gpio = direction_gpio,
        .slice_num = slice_num,
        .channel = channel,
    };
    return ret;
}

static inline void motor_pwm_enabled(struct motor_pwm_t *pwm, const bool enabled) {
    pwm_set_chan_level(pwm->slice_num, pwm->channel, 0);
    pwm_set_enabled(pwm->slice_num, enabled);
}

static inline void motor_pwm_set_duty_ratio(struct motor_pwm_t *pwm, float duty_ratio) {
    const uint16_t level_max = pwm_hw->slice[pwm->slice_num].top;

    int level = abs(duty_ratio * level_max);
    if (level > level_max) { // 오버플로우 방지
        level = level_max;
    } else if (level < 0) { // 언더플로우 방지
        level = 0;
    }
    pwm_set_chan_level(pwm->slice_num, pwm->channel, (uint16_t)level);
}

static inline void motor_pwm_set_direction(struct motor_pwm_t *pwm, bool value) {
    gpio_put(pwm->direction_gpio, value);
}

struct motor_control_state_t {
    float velocity;
    float gain_p;
    float gain_d;
    int32_t error;
    int32_t target;
    int32_t current;
};

void motor_start(void);
void motor_stop(void);
void motor_set_velocity(const enum motor_index index, float velocity);
void motor_init(void);
struct motor_control_state_t motor_get_control_state(const enum motor_index index);
void motor_pwm_test(void);

#endif