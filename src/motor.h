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

struct motor_encoder_t {
    PIO pio;
    struct {
        uint base_gpio;
        uint sm;
        bool reverse;
    } where[MOTOR_COUNT];
};

#include "quadrature_encoder.pio.h"
static inline struct motor_encoder_t motor_encoder_init(const PIO pio, const uint gpio[], const bool reverse[]) {
    /**
     * RP2040에는 2개의 PIO(Programmable Input Ouput) 장치가 있다.
     * 이 장치를 이용하면 PIO 전용 명령어들을 이용하여 GPIO에 대해 간단한 연산들을 수행할 수 있는데,
     * 이는 CPU 자원을 이용하지 않고 PIO 장치에서 독립적으로 작동하기에 빠르고 효율적이다.
     * 우리는 하나의 PIO에 quadrature encoder 프로그램을 올리고 이를 이용하고자 한다.
     */
    const uint instruction_offset = pio_add_program(pio, &quadrature_encoder_program);

    /**
     * quadrature encoder PIO의 state machine 번호. 하나의 PIO에 총 4개의 state machine이 존재하는데,
     * 각 state machine은 동일한 PIO 프로그램에 대해 독립적인 상태를 가지는 인스턴스라고 보면 된다.
     */
    const uint left_sm = MOTOR_LEFT;
    const uint right_sm = MOTOR_RIGHT;

    /**
     * PIO 엔코더 프로그램을 초기화한다. 이때, 왼쪽 엔코더 A상 GPIO와 오른쪽 엔코더 A상 GPIO를 이용한다.
     * (!) 양쪽 엔코더의 A상만 정의하는 이유는 B상이 항상 A상 GPIO의 바로 다음 번호이기 때문이다.
     */
    quadrature_encoder_program_init(pio, left_sm, instruction_offset, gpio[MOTOR_LEFT], 0);
    quadrature_encoder_program_init(pio, right_sm, instruction_offset, gpio[MOTOR_RIGHT], 0);

    // encoder 정보 반환
    struct motor_encoder_t ret = {
        .pio = pio,
        .where = {
            {
                .base_gpio = gpio[MOTOR_LEFT],
                .sm = left_sm,
                .reverse = reverse[MOTOR_LEFT], // 모터의 회전 방향과 엔코더의 측정 방향이 반대일 수 있다.
            },
            {
                .base_gpio = gpio[MOTOR_RIGHT],
                .sm = right_sm,
                .reverse = reverse[MOTOR_RIGHT],
            },
        }
    };
    return ret;
}

static inline int32_t motor_encoder_get_value(struct motor_encoder_t *enc, enum motor_index index) {
    int32_t value = quadrature_encoder_get_count(enc->pio, enc->where[index].sm);

    /**
     * 모터의 회전 방향과 엔코더의 측정 방향이 반대일 수 있다.
     * 물리적인 축은 하나이고, 이에 종속된 두 기기(모터, 엔코더)가 방향을 서로 다르게 본다면 문제가 발생한다.
     * 가령, 모터는 1000 만큼 갔다고 생각했는데, 엔코더는 -1000 만큼 갔다고 인식하면 문제가 생기는 것.
     * 이에 따른 보정 절차를 수행하고 반환한다.
     */
    return enc->where[index].reverse ? -value : value;
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