#include "motor_dc.h"

#include <stdlib.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "sensing.h"
#include "timer.h"

/**
 * @brief 속도에 대한 PID제어를 한다.
 */
void motor_dc_control(void);

typedef struct {
    const uint pwm_gpio;
    // RP2040에는 8개의 PWM slice가 있으며, 각 slice마다 2개의 channel을 가지고 있어 총 16개의 PWM 신호를
    // 만들어낼 수 있다. 각 GPIO 마다 PWM slice와 channel이 배정돼 있다.
    // PWM의 slice num은 pwm_gpio_to_slice_num 함수를 통해 구할 수 있으며, channel은 pwm_gpio_to_channel 함수를 통해 구할 수 있다.
    // 우리는 해당 함수 호출을 매번 할 필요가 없도록 위의 함수를 매크로로 정의하여 각 모터에 들어가는 PWM의 slice num과 channel을 상수화한다.
    const uint slice_num, channel;
    // DC 모터 드라이버의 Direction 핀에 들어가는 GPIO를 정의한다.
    // 모터 드라이버에 내장돼있는 H-Bridge 회로를 이용해 모터의 회전 방향을 변경하는데 이용한다.
    const uint direction_gpio;
} motor_dc_t;

#define GPIO_TO_SLICE_NUM(GPIO) ((GPIO >> 1u) & 7u)
#define GPIO_TO_CHANNEL(GPIO)   ((GPIO & 1u))

// 각 모터에 대한 PWM의 slice 번호와 channel 그리고 Direction GPIO를 저장해 놓는다.
// (!) 이때, PWM GPIO가 서로 같은 PWM slice이어야 한다.
//     그렇지 않다면, 다른 곳에서 사용하고 있는 PWM slice 설정을 덮어 씌울 수 있기 때문에 예기지 못한 오작동이 생길 수 있다.
motor_dc_t motor_dc[MOTOR_DC_COUNT] = {
    {
        .pwm_gpio = MOTOR_DC_PWM_LEFT_GPIO,
        .slice_num = GPIO_TO_SLICE_NUM(MOTOR_DC_PWM_LEFT_GPIO),
        .channel = GPIO_TO_CHANNEL(MOTOR_DC_PWM_LEFT_GPIO),
        .direction_gpio = MOTOR_DC_DIRECTION_LEFT_GPIO,
    },
    {
        .pwm_gpio = MOTOR_DC_PWM_RIGHT_GPIO,
        .slice_num = GPIO_TO_SLICE_NUM(MOTOR_DC_PWM_RIGHT_GPIO),
        .channel = GPIO_TO_CHANNEL(MOTOR_DC_PWM_RIGHT_GPIO),
        .direction_gpio = MOTOR_DC_DIRECTION_RIGHT_GPIO,
    },
};

void motor_dc_init(void) {
    const uint freq_sys = clock_get_hz(clk_sys);  // PWM 주파수를 결정하기 위해 clk_sys 주파수를 구한다.

    // RP2040 datasheet 4.5.2.6절에 따르면 PWM 주파수는 다음과 같이 구한다.
    // f_pwm = f_sys / ( (TOP + 1) * (CSR_PH_CORRECT + 1) * ( DIV_INT + (DIV_FRAC / 16) ) )
    // * CSR_PH_CORRECT는 카운터 레지스터가 TOP에 도달했을 때 0으로 떨어지는 것이 아니라 그대로 감소하는 설정을 말하며,
    //   우리는 이 기능을 사용하지 않기 때문에 0으로 둔다.
    // * DIV_INT 및 DIV_FRAC는 클럭을 나눌 때 사용하며, 기본값인 DIV_INT = 1, DIV_FRAC = 0으로 놔둔다.
    // 위의 조건에 따라 TOP = ( f_sys / f_pwm ) - 1 식으로 나타낼 수 있다.
    // 예를 들어, f_sys가 125,000,000Hz이고 f_pwm이 20,000Hz이면 TOP은 6,249이며,
    // 한 클럭마다 PWM 카운터 레지스터의 값이 0에서부터 1씩 증가하다가, 6,249에 도달하고 나서 0으로 다시 떨어지게 된다.
    const uint top = (freq_sys / MOTOR_DC_PWM_FREQUENCY) - 1;

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, top);  // wrap == top

    for (int i = 0; i < MOTOR_DC_COUNT; i++) {
        // 모터 드라이버에 들어갈 PWM 및 GPIO 초기화
        gpio_set_function(motor_dc[i].pwm_gpio, GPIO_FUNC_PWM);
        pwm_init(motor_dc[i].slice_num, &config, false);
        // 모터의 회전 방향을 결정하는 GPIO 초기화
        gpio_init(motor_dc[i].direction_gpio);
        gpio_set_dir(motor_dc[i].direction_gpio, GPIO_OUT);
    }
}

void motor_dc_set_enabled(enum motor_index index, bool enabled) {
    uint slice_num = motor_dc[index].slice_num;
    uint channel = motor_dc[index].channel;

    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, enabled);
}

void motor_dc_input_voltage(enum motor_index index, float input_voltage, float supply_voltage) {
    // PWM duty cycle 계산
    float duty_cycle = input_voltage / supply_voltage;

    // 모터 회전 방향 설정
    gpio_put(motor_dc[index].direction_gpio, duty_cycle > 0.f);

    // PWM 카운터 최댓값 구하기
    const uint pwm_top = pwm_hw->slice[motor_dc[index].slice_num].top;

    int level = MOTOR_DC_PWM_DEAD_ZONE + abs(pwm_top * duty_cycle); // PWM level 계산
    if (level > pwm_top) {  // max limit(max)
        level = pwm_top;
    } else if (level < MOTOR_DC_PWM_DEAD_ZONE) { // min limit
        level = MOTOR_DC_PWM_DEAD_ZONE;
    }

    // PWM 출력
    pwm_set_chan_level(motor_dc[index].slice_num, motor_dc[index].channel, (uint16_t)level);
}

#define PI 3.141592f
const float dt_us = 500;                                                                    // 모터 제어시 인터럽트가 발생하는 주기(us)
const float interrupt_period_sec = dt_us / (1000 * 1000);                                   // 속도 구하는 함수를 작동시키는 인터럽트가 작동하는 주기(단위는 s).
const float wheel_diameter_m = 0.038;                                                       // 바퀴의 지름(m)
const float gear_ratio = 17.f / 69.f;                                                       // 모터(17)/바퀴(69) 기어비
const float velo_param = PI * wheel_diameter_m * gear_ratio / 2048 / interrupt_period_sec;  // 속도를 구하기 위한 계수를 계산해놓는다.

float cur_velo[MOTOR_DC_COUNT];  // 각 모터의 현재 속도 (m/s)

void update_velocity_from_encoder(void) {
    static int pre_tick[2];
    int cur_tick[2];

    // 현재 엔코더에서 측정한 틱 수를 가져온다.
    cur_tick[MOTOR_DC_LEFT] = get_encoder_count(MOTOR_DC_LEFT);
    cur_tick[MOTOR_DC_RIGHT] = get_encoder_count(MOTOR_DC_RIGHT);

    // dt_us(us)전에 측정한 틱 수를 현재 틱 수에서 뺀 수에 velo_param을 곱하여 현재 속도를 구한다.
    // 여기서 왼쪽 모터의 속도를 구하는 데 (현재 틱수-이전 틱수)에 -를 붙인다. 
    // 그 이유는 실제 전진하는 회전 방향과 엔코더의 틱이 올라가는 회전 방향이 반대이기 때문이다
    cur_velo[MOTOR_DC_LEFT] = -(cur_tick[MOTOR_DC_LEFT] - pre_tick[MOTOR_DC_LEFT]) * velo_param;
    cur_velo[MOTOR_DC_RIGHT] = (cur_tick[MOTOR_DC_RIGHT] - pre_tick[MOTOR_DC_RIGHT]) * velo_param;

    // 현재 엔코더에서 측정한 틱 수를 다음 속도 측정을 위해 저장한다.
    pre_tick[MOTOR_DC_LEFT] = cur_tick[MOTOR_DC_LEFT];
    pre_tick[MOTOR_DC_RIGHT] = cur_tick[MOTOR_DC_RIGHT];
}

float tar_velo[MOTOR_DC_COUNT]; // 각 모터에 대한 목표 속도(m/s)
float target_velocity;          // 목표 속도 (두 모터의 목표 평균 속도)

void set_target_velocity(float target_vel) {
    target_velocity = target_vel;
}

float get_target_velocity(void) {
    return target_velocity;
}

const float k_p = 0.005f;               // P 비례상수
const float k_i = 0.005f;               // I 비례상수
const float k_d = 0.003f;               // D 비례상수
const float limit_error_sum = 10.f;     // error_sum을 제한하는 상수
const float lpf_const = 0.1f;           // low pass filter의 감도를 설정하는 상수
float error_sum[MOTOR_DC_COUNT] = {0,}; // 에러의 누적을 저장하는 변수
float error[MOTOR_DC_COUNT];            // 에러를 저장하는 변수

void motor_dc_control_enabled(bool enabled) {
    if (enabled) {
        set_target_velocity(0);                            // 목표 속도 0으로 초기화.
        motor_dc_set_enabled(MOTOR_DC_LEFT, true);         // 왼쪽 모터 드라이버에 PWM 신호 인가
        motor_dc_set_enabled(MOTOR_DC_RIGHT, true);        // 오른쪽 모터 드라이버에 PWM 신호 인가
        timer_periodic_start(1, dt_us, motor_dc_control);  // dc모터 제어 interrupt 시작
    } else {
        set_target_velocity(0);                      // 목표 속도 0으로 설정
        sleep_ms(250);                               // 목표 속도에 도달하는 시간을 기다려준다.
        motor_dc_set_enabled(MOTOR_DC_LEFT, false);  // 왼쪽 모터 드라이버에 PWM 신호 인가 종료
        motor_dc_set_enabled(MOTOR_DC_RIGHT, false); // 오른쪽 모터 드라이버에 PWM 신호 인가 종료
        timer_periodic_stop(1);                      // dc모터 제어 interrupt 중지
    }
}

void motor_dc_control(void) {
    float p_term[MOTOR_DC_COUNT], i_term[MOTOR_DC_COUNT], d_term[MOTOR_DC_COUNT];  // 각각 비례항, 적분항, 미분항
    float d_error[MOTOR_DC_COUNT];   // 에러의 미분값(derivative of error)을 저장하는 변수
    float pre_error[MOTOR_DC_COUNT]; // 에러의 미분값을 구하기 위해 dt_us(us)전의 에러를 저장하는 변수
    float pid[MOTOR_DC_COUNT];       // 비례항 적분항 미분항의 합에 비례하는 최종적으로 인가할 전압을 저장하는 변수

    // 각 모터에 대한 현재 속도를 측정한다.
    update_velocity_from_encoder();

    // 목표 속도에 따른 각 모터의 실제 목표 속도를 설정한다.
    // !!!!!아직 센서 파트가 만들어져있지 않으므로 완성된 것은 아님!!!!!!
    tar_velo[MOTOR_DC_LEFT] = tar_velo[MOTOR_DC_RIGHT] = target_velocity;

    // 각 모터에 대한 pre_error(이전 error값)을 저장한다. - 비례항을 위해
    pre_error[MOTOR_DC_LEFT] = error[MOTOR_DC_LEFT];
    pre_error[MOTOR_DC_RIGHT] = error[MOTOR_DC_RIGHT];

    // 각 모터에 대한 error(현재 속도와 목표 속도와의 차)를 구한다.
    error[MOTOR_DC_LEFT] = tar_velo[MOTOR_DC_LEFT] - cur_velo[MOTOR_DC_LEFT];
    error[MOTOR_DC_RIGHT] = tar_velo[MOTOR_DC_RIGHT] - cur_velo[MOTOR_DC_RIGHT];

    // 각 모터에 대한 error_sum(error의 누적)를 계산한다. - 적분항을 위해.
    error_sum[MOTOR_DC_LEFT] += error[MOTOR_DC_LEFT];
    error_sum[MOTOR_DC_RIGHT] += error[MOTOR_DC_RIGHT];

    // anti-windup : error의 누적에 제한을 건다.
    // 이때 제한값은 limit_error_sum이다.
    if (error_sum[MOTOR_DC_LEFT] > limit_error_sum)
        error_sum[MOTOR_DC_LEFT] = limit_error_sum;
    else if (error_sum[MOTOR_DC_LEFT] < -limit_error_sum)
        error_sum[MOTOR_DC_LEFT] = -limit_error_sum;
    if (error_sum[MOTOR_DC_RIGHT] > limit_error_sum)
        error_sum[MOTOR_DC_RIGHT] = limit_error_sum;
    else if (error_sum[MOTOR_DC_RIGHT] < -limit_error_sum)
        error_sum[MOTOR_DC_RIGHT] = -limit_error_sum;

    // error를 통해 p_term(proportional term, 비례항)를 구한다.
    p_term[MOTOR_DC_LEFT] = k_p * error[MOTOR_DC_LEFT];
    p_term[MOTOR_DC_RIGHT] = k_p * error[MOTOR_DC_RIGHT];

    // error의 누적을 통해 i_term(integral term, 적분항)를 구한다.
    i_term[MOTOR_DC_LEFT] = k_i * error_sum[MOTOR_DC_LEFT];
    i_term[MOTOR_DC_RIGHT] = k_i * error_sum[MOTOR_DC_RIGHT];

    // error의 미분(error - pre_error)을 통해 d_term(derivative term, 미분항)를 구한다.
    // 이때 미분항의 노이즈를 줄이기위해 low pass filter을 거친다.
    // low pass filter : f(x) = x*c/(x+c) -> x == c일 때 아웃풋이 x/2인 분수함수형의 low pass filter이다.

    // 논의점 1 low pass filter가 필요한가?
    // 논의점 2 d_term이 필요한가? 
    d_error[MOTOR_DC_LEFT] = error[MOTOR_DC_LEFT] - pre_error[MOTOR_DC_LEFT];
    d_error[MOTOR_DC_RIGHT] = error[MOTOR_DC_RIGHT] - pre_error[MOTOR_DC_RIGHT];
    d_term[MOTOR_DC_LEFT] = k_d * lpf_const * d_error[MOTOR_DC_LEFT] / (lpf_const + d_error[MOTOR_DC_LEFT]);
    d_term[MOTOR_DC_RIGHT] = k_d * lpf_const * d_error[MOTOR_DC_RIGHT] / (lpf_const + d_error[MOTOR_DC_RIGHT]);

    // 비례항 적분항 미분항의 합을 전압으로 설정한다. 이를 각 모터에 인가한다.
    // 이때 dt_us를 곱하는데, 이는 dt에 대해 p,i,d term이 독립하도록 하기 위해서이다.
    pid[MOTOR_DC_LEFT] = dt_us * (p_term[MOTOR_DC_LEFT] + i_term[MOTOR_DC_LEFT] + d_term[MOTOR_DC_LEFT]);
    pid[MOTOR_DC_RIGHT] = dt_us * (p_term[MOTOR_DC_RIGHT] + i_term[MOTOR_DC_RIGHT] + d_term[MOTOR_DC_RIGHT]);
    motor_dc_input_voltage(MOTOR_DC_LEFT, pid[MOTOR_DC_LEFT], voltage);
    motor_dc_input_voltage(MOTOR_DC_RIGHT, pid[MOTOR_DC_RIGHT], -voltage);
    // 마지막줄 함수에서 오른쪽 모터에 -를 붙여 전압을 반대로 준다.
    // 만약 이것(-부호)이 없을 때 원하는 회전 방향과 반대의 전압을 인가하기 때문이다.
}
