#include "motor_dc.h"

#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define PWM_LEVEL_MAX 0xffff
#define PWM_LEVEL_MIN 0x0000

const uint motor_dc_pwm[2] = {
  MOTOR_DC_PWM_LEFT_GPIO,
  MOTOR_DC_PWM_RIGHT_GPIO,
};
const uint motor_dc_direction[2] = {
  MOTOR_DC_DIRECTION_LEFT_GPIO,
  MOTOR_DC_DIRECTION_RIGHT_GPIO,
};

/*
 * LMD18200에 입력 PWM 신호를 넣을 때, duty cycle의 시간이 너무 짧을 경우
 * 해당 신호가 무시되는 현상이 발생한다. 거의 대부분의 트랜지스터가 그렇다고 하며,
 * 이를 보정하기 위해 짧은 신호가 무시되지 않는 최소한의 duty cycle를 정의한다.
 */
#define LMD18200_DEAD_ZONE  0
uint motor_pwm_slice[2];

void motor_dc_init(void) {
  // 모터 드라이버에 들어갈 PWM을 위해 해당 핀들을 GPIO_FUNC_PWM 기능으로 설정한다.
  gpio_set_function(MOTOR_DC_PWM_LEFT_GPIO,  GPIO_FUNC_PWM);
  gpio_set_function(MOTOR_DC_PWM_RIGHT_GPIO, GPIO_FUNC_PWM);

  // PWM 설정
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 1.f);
  
  for (int i = 0; i < 2; i++) {
    // PWM 출력 GPIO가 어떤 PWM slice인지 구한다. 이 slice 번호를 이용해 PWM을 제어한다.
    motor_pwm_slice[i] = pwm_gpio_to_slice_num(motor_dc_pwm[i]);
    // PWM 초기화
    pwm_init(motor_dc_pwm[i], &config, false);
    // 모터 드라이버 방향을 결정하는 GPIO 초기화
    gpio_init(motor_dc_direction[i]);
    gpio_set_dir(motor_dc_direction[i], GPIO_OUT);
  }
}

void motor_dc_set_enabled(uint motor, bool enabled) {
  pwm_set_gpio_level(motor_dc_pwm[motor], PWM_LEVEL_MIN);
  pwm_set_enabled(motor_pwm_slice[motor], enabled);
}

void motor_dc_input_voltage(uint motor, float input_voltage, float supply_voltage) {
  // PWM duty cycle 계산
  float duty_cycle = input_voltage / supply_voltage;

  // 모터 회전 방향 설정
  gpio_put(motor_dc_direction[motor], duty_cycle > 0.f);

  // PWM level 계산
  int level = LMD18200_DEAD_ZONE + abs(duty_cycle * PWM_LEVEL_MAX);
  if (level > PWM_LEVEL_MAX) {
    level = PWM_LEVEL_MAX;
  } else if (level < PWM_LEVEL_MIN) {
    level = PWM_LEVEL_MIN;
  }

  // PWM 출력
  pwm_set_gpio_level(motor_dc_pwm[motor], (uint16_t)level);
}