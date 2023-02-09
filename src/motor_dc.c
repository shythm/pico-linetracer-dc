/**
 * @file motor_dc.c
 * @author Seongho Lee & Jinoo Li
 * @brief 1. DC 모터 드라이버에 PWM을 인가하여 모터를 구동하고,
 *        2. 엔코더를 통해 모터의 현재 속도를 가져오며,
 *        3. 속도에 대해 PID 제어하는 소스코드
 */

#define INCLUDE_MOTOR_DC_CONSTANTS
#include "motor_dc.h"

#include <stdlib.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "quadrature_encoder.pio.h"
#include "sensing.h"
#include "timer.h"

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define PI 3.141592f

#define GPIO_TO_SLICE_NUM(GPIO) ((GPIO >> 1u) & 7u)
#define GPIO_TO_CHANNEL(GPIO)   ((GPIO & 1u))

// 각 모터에 대해 필요한 상수들을 저장해 놓는다.
struct motor_dc_t {
    // DC 모터 드라이버의 PWM 핀에 들어가는 GPIO를 정의한다.
    const uint pwm_gpio;

    // RP2040에는 8개의 PWM slice가 있으며, 각 slice마다 2개의 channel을 가지고 있어 총 16개의 PWM 신호를 만들어낼 수 있다.
    // 각 GPIO 마다 PWM slice와 channel이 배정돼 있다.
    // PWM slice는 pwm_gpio_to_slice_num 함수를 통해 구할 수 있으며, channel은 pwm_gpio_to_channel 함수를 통해 구할 수 있다.
    // 우리는 해당 함수 호출을 매번 할 필요가 없도록 위의 함수를 매크로로 정의하여 각 모터에 들어가는 PWM의 slice num과 channel을 상수화한다.
    // (!) 이때, PWM GPIO가 서로 같은 PWM slice이어야 한다.
    //     그렇지 않다면, 다른 곳에서 사용하고 있는 PWM slice 설정을 덮어 씌울 수 있기 때문에 예기지 못한 오작동이 생길 수 있다.
    const uint slice_num, channel;

    // DC 모터 드라이버의 Direction 핀에 들어가는 GPIO를 정의한다.
    // 모터 드라이버에 내장돼있는 H-Bridge 회로를 이용해 모터의 회전 방향을 변경하는데 이용한다.
    const uint direction_gpio;

    // 좌우에 있는 DC 모터 드라이버의 Direction 핀에 같은 신호를 주게 되면, 서로 반대로 바퀴가 굴러갈 수 있다. 이를 보정하기 위한 상수이다.
    const int direction;

    // 엔코더의 A상 GPIO를 정의한다. A상만 정의하는 이유는 B상이 항상 A상의 GPIO 바로 다음 번호이기 때문이다.
    const uint encoder_base_gpio;

    // RP2040에는 2개의 PIO(Programmable Input Output) 장치가 있다.
    // 이 장치를 이용하면 사전 정의된 명령어 셋을 이용하여 GPIO에 대해 간단한 연산들을 수행할 수 있는데, 이는 CPU 자원을 사용하지 않고 독립적으로 작동하는 장치여서 빠르고 효율적이다.
    // 우리는 하나의 PIO에 quadrature encoder 프로그램을 올리고, PIO에 있는 총 4개의 state machine 중 모터당 하나씩 골라 프로그램을 수행한다.
    const uint encoder_pio_sm;
} motor_dc[MOTOR_DC_COUNT] = {
    {
        .pwm_gpio = MOTOR_DC_PWM_LEFT_GPIO,
        .slice_num = GPIO_TO_SLICE_NUM(MOTOR_DC_PWM_LEFT_GPIO),
        .channel = GPIO_TO_CHANNEL(MOTOR_DC_PWM_LEFT_GPIO),
        .direction_gpio = MOTOR_DC_DIRECTION_LEFT_GPIO,
        .direction = MOTOR_DC_DIRECTION_LEFT,
        .encoder_base_gpio = MOTOR_ENCODER_BASE_LEFT_GPIO,
        .encoder_pio_sm = MOTOR_DC_LEFT,
    },
    {
        .pwm_gpio = MOTOR_DC_PWM_RIGHT_GPIO,
        .slice_num = GPIO_TO_SLICE_NUM(MOTOR_DC_PWM_RIGHT_GPIO),
        .channel = GPIO_TO_CHANNEL(MOTOR_DC_PWM_RIGHT_GPIO),
        .direction_gpio = MOTOR_DC_DIRECTION_RIGHT_GPIO,
        .direction = MOTOR_DC_DIRECTION_RIGHT,
        .encoder_base_gpio = MOTOR_ENCODER_BASE_RIGHT_GPIO,
        .encoder_pio_sm = MOTOR_DC_RIGHT,
    },
};

void motor_dc_init(void) {
    const uint freq_sys = clock_get_hz(clk_sys); // PWM 주파수를 결정하기 위해 clk_sys 주파수를 구한다.

    // RP2040 datasheet 4.5.2.6절에 따르면 PWM 주파수는 다음과 같이 구한다.
    // f_pwm = f_sys / ( (TOP + 1) * (CSR_PH_CORRECT + 1) * ( DIV_INT + (DIV_FRAC / 16) ) )
    // * CSR_PH_CORRECT는 카운터 레지스터가 TOP에 도달했을 때 0으로 떨어지는 것이 아니라 그대로 감소하는 설정을 말하며,
    //   우리는 이 기능을 사용하지 않기 때문에 0으로 둔다.
    // * DIV_INT 및 DIV_FRAC는 클럭을 나눌 때 사용하며, 기본값인 DIV_INT = 1, DIV_FRAC = 0으로 놔둔다.
    // 위의 조건에 따라 TOP = ( f_sys / f_pwm ) - 1 식으로 나타낼 수 있다.
    // 예를 들어, f_sys가 125,000,000Hz이고 f_pwm이 20,000Hz이면 TOP은 6,249이며,
    // 한 클럭마다 PWM 카운터 레지스터의 값이 0에서부터 1씩 증가하다가, 6,249에 도달하고 나서 0으로 다시 떨어지게 된다.
    const uint top = (freq_sys / MOTOR_DC_PWM_FREQUENCY) - 1;

    // pwm 장치 기본 설정을 구한 뒤, TOP 레지스터 수정
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, top); // wrap == top

    // PIO에 엔코더 프로그램 주입
    uint offset = pio_add_program(MOTOR_ENCODER_PIO, &quadrature_encoder_program);

    for (int i = 0; i < MOTOR_DC_COUNT; i++) {
        // 모터 드라이버에 들어갈 PWM 및 GPIO 초기화
        gpio_set_function(motor_dc[i].pwm_gpio, GPIO_FUNC_PWM);
        pwm_init(motor_dc[i].slice_num, &config, false);

        // 모터의 회전 방향을 결정하는 GPIO 초기화
        gpio_init(motor_dc[i].direction_gpio);
        gpio_set_dir(motor_dc[i].direction_gpio, GPIO_OUT);

        // PIO 엔코더 프로그램 초기화
        quadrature_encoder_program_init(
            MOTOR_ENCODER_PIO, motor_dc[i].encoder_pio_sm, offset, motor_dc[i].encoder_base_gpio, 0);
    }
}

void motor_dc_pwm_enabled(enum motor_dc_index index, bool enabled) {
    uint slice_num = motor_dc[index].slice_num;
    uint channel = motor_dc[index].channel;

    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, enabled);
}

void motor_dc_input_voltage(enum motor_dc_index index, float input_voltage, float supply_voltage) {
    // PWM duty cycle 계산
    float duty_cycle = input_voltage / supply_voltage;

    // 모터 회전 방향 설정
    gpio_put(motor_dc[index].direction_gpio, duty_cycle > 0.f);

    // PWM 카운터 최댓값 구하기
    const uint pwm_top = pwm_hw->slice[motor_dc[index].slice_num].top;

    int level = MOTOR_DC_PWM_DEAD_ZONE + abs(pwm_top * duty_cycle); // PWM level 계산
    if (level > pwm_top) { // max limit(max)
        level = pwm_top;
    } else if (level < MOTOR_DC_PWM_DEAD_ZONE) { // min limit
        level = MOTOR_DC_PWM_DEAD_ZONE;
    }

    // PWM 출력
    pwm_set_chan_level(motor_dc[index].slice_num, motor_dc[index].channel, (uint16_t)level);
}

const float dt_us = MOTOR_DC_TIMER_INTERVAL_US; // 모터 제어시 인터럽트가 발생되는 주기(us)

struct motor_dc_pid_state_t {
    float error; // PID 제어 인터럽트 발생 당시 에러
    float error_sum; // 에러의 누적
    float velo_curr; // 현재 속도
    float velo_targ; // 목표 속도
} pid_state[MOTOR_DC_COUNT];

void motor_dc_set_velocity(enum motor_dc_index index, float velocity) {
    pid_state[index].velo_targ = velocity;
}

float motor_dc_get_velocity(enum motor_dc_index index) {
    return pid_state[index].velo_targ;
}

float motor_dc_get_current_velocity(enum motor_dc_index index) {
    return pid_state[index].velo_curr;
}

uint motor_dc_get_encoder_count(enum motor_dc_index index) {
    return quadrature_encoder_get_count(MOTOR_ENCODER_PIO, motor_dc[index].encoder_pio_sm);
}

/**
 * @brief 매 주기마다 호출되어 엔코더로 현재 속도를 갱신한다. 이때 pid_state 구조체 변수의 velo_curr 필드가 갱신된다.
 *
 * @param index MOTOR_DC_LEFT(왼쪽 모터) 또는 MOTOR_DC_RIGHT(오른쪽 모터)
 */
static inline void motor_dc_update_velocity(enum motor_dc_index index) {
    // 속도 구하는 함수를 작동시키는 인터럽트가 작동하는 주기(단위는 s)
    static const float interrupt_period_sec = dt_us / (1000 * 1000);
    // 엔코더 1틱당 몇 미터인지 나타내는 상수
    static const float tick_per_meter = PI * MOTOR_WHEEL_DIAMETER_M * MOTOR_GEAR_RATIO / MOTOR_ENCODER_RESOLUTION;
    // 속도를 구하기 위해 이전 틱 수를 저장하는 변수
    static int tick_prev[MOTOR_DC_COUNT];

    // 현재 엔코더에서 측정한 틱 수를 가져온다.
    int tick_curr = motor_dc_get_encoder_count(index);

    // "속도 = 거리 / 시간" 이므로 한 주기 동안 이동한 거리를 구하고 주기로 나눠 속도를 구한다.
    // 그리고 모터의 회전 방향에 맞는 상수(1 또는 -1)를 곱해준다.
    // 이때 모터의 회전 방향과 엔코더가 측정한 이동 방향과 반대일 수 있기 때문에 이에 따른 처리도 해준다.
    int sign = MOTOR_ENCODER_REVERSE ? -motor_dc[index].direction : motor_dc[index].direction;
    float velo = sign * (tick_curr - tick_prev[index]) * tick_per_meter / interrupt_period_sec;

    pid_state[index].velo_curr = velo; // 현재 상태에 속도 정보 업데이트

    // 다음 번에 이용하기 위해 현재 엔코더에서 측정한 틱 수를 저장해둔다.
    tick_prev[index] = tick_curr;
}

/**
 * @brief 매 주기마다 호출되어 모터 PID 제어를 한다.
 * 설정된 목표 속도로 추종하는 제어기이며, 엔코더로부터 가져온 현재 속도를 통해 에러를 구하여 피드백을 한다.
 *
 * @param index MOTOR_DC_LEFT(왼쪽 모터) 또는 MOTOR_DC_RIGHT(오른쪽 모터)
 */
inline static void motor_dc_control(enum motor_dc_index index) {
    // 모터에 대한 pre_error(이전 error값)을 저장한다. - 미분항을 구하기 위해
    float error_pre = pid_state[index].error;

    // 모터에 대한 error(현재 속도와 목표 속도와의 차)를 구한다. - 비례항을 구하기 위해
    float error = pid_state[index].velo_targ - pid_state[index].velo_curr;

    // 모터에 대한 error_sum(error의 누적)를 계산한다. - 적분항을 구하기 위해
    float error_sum = pid_state[index].error_sum;
    error_sum += error;

    // anti-windup: error의 누적에 제한을 건다.
    error_sum = MIN(error_sum, MOTOR_DC_ERROR_SUM_LIMIT);
    error_sum = MAX(error_sum, -MOTOR_DC_ERROR_SUM_LIMIT);

    // error를 통해 비례항(proportional term)을 구한다.
    float term_p = MOTOR_DC_GAIN_P * error;

    // error의 누적을 통해 적분항(integral term)을 구한다.
    float term_i = MOTOR_DC_GAIN_I * error_sum;

    // error의 미분(error - pre_error)을 통해 미분항(derivative term)을 구한다.
    // 이때 미분항의 노이즈를 줄이기위해 low pass filter을 거친다.
    // low pass filter: f(x) = x * c / (x + c) -> x == c일 때 아웃풋이 (x / 2)인 분수함수형의 low pass filter이다.
    // 논의점 1: low pass filter가 필요한가?
    // 논의점 2: d_term이 필요한가?
    // float error_d = error - error_pre;
    // float term_d = MOTOR_DC_GAIN_D * MOTOR_DC_LPF_CONST * error_d / (MOTOR_DC_LPF_CONST + error_d);

    // 비례항 적분항 미분항의 합을 전압으로 설정한다. 이를 모터에 인가한다.
    // 이때 dt_us를 곱하는데, 이는 dt에 대해 p, i, d term이 독립하도록 하기 위해서이다.
    float input_voltage = dt_us * (term_p + term_i /*+ term_d*/);
    motor_dc_input_voltage(index, input_voltage, motor_dc[index].direction * voltage);

    // 모터 PID 제어 상태를 저장한다.
    pid_state[index].error = error;
    pid_state[index].error_sum = error_sum;
}

/**
 * @brief DC 모터 제어를 위한 타이머 IRQ 핸들러
 */
static void motor_dc_control_handler(void) {
    motor_dc_update_velocity(MOTOR_DC_LEFT);
    motor_dc_control(MOTOR_DC_LEFT);
    motor_dc_update_velocity(MOTOR_DC_RIGHT);
    motor_dc_control(MOTOR_DC_RIGHT);
}

void motor_dc_control_enabled(bool enabled) {
    // 목표 속도 0으로 초기화
    motor_dc_set_velocity(MOTOR_DC_LEFT, 0.0f);
    motor_dc_set_velocity(MOTOR_DC_RIGHT, 0.0f);

    // 목표 속도가 0으로 떨어질 때까지 기다림
    if (!enabled) {
        sleep_ms(250);
    }

    // PWM 신호 (비)활성화
    motor_dc_pwm_enabled(MOTOR_DC_LEFT, enabled);
    motor_dc_pwm_enabled(MOTOR_DC_RIGHT, enabled);

    // DC 모터 제어 interrupt 시작/중지
    if (enabled) {
        timer_periodic_start(MOTOR_DC_TIMER_SLOT, dt_us, motor_dc_control_handler);
    } else {
        timer_periodic_stop(MOTOR_DC_TIMER_SLOT);
    }
}
