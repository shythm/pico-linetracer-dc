#ifndef __MOTOR_DC_H_
#define __MOTOR_DC_H_

#define MOTOR_DC_PWM_LEFT_GPIO          14
#define MOTOR_DC_PWM_RIGHT_GPIO         15
#define MOTOR_DC_DIRECTION_LEFT_GPIO    16
#define MOTOR_DC_DIRECTION_RIGHT_GPIO   17

// PWM 주파수가 가청 주파수 영역대면, 모터에서 귀에 거슬리는 고주파음이 들리게 된다.
// 따라서 가청 주파수에 해당하지 않는 20khz 정도를 PWM 주파수로 사용한다.
#define MOTOR_DC_PWM_FREQUENCY  20000

// 모터 드라이버에 입력 PWM 신호를 넣을 때, duty cycle이 너무 짧을 경우 해당 신호가 무시되는 현상이 발생한다.
// 이는 트랜지스터의 전형적인 특징이며, 이를 보정하기 위해 신호가 인식되는 최소한의 duty cycle을 정의한다.
// TODO: 오실로스코프로 LMD18200 모터드라이버의 dead zone을 측정하자.
#define MOTOR_DC_PWM_DEAD_ZONE  0

enum motor_index {
    MOTOR_DC_LEFT = 0,
    MOTOR_DC_RIGHT,
    MOTOR_DC_COUNT
};

#include "pico/types.h"

/**
 * @brief DC 모터 드라이버 구동을 위해 PWM의 주기 설정 및 초기화를 진행하고, 
 *        DC 모터의 회전 방향을 제어하는 Direction GPIO를 초기화한다.
 */
void motor_dc_init(void);

/**
 * @brief DC 모터 드라이버에 PWM 신호 생성 여부를 결정한다.
 * 
 * @param index 왼쪽 모터(MOTOR_DC_LEFT) 또는 오른쪽 모터(MOTOR_DC_RIGHT)
 * @param enabled PWM 신호 생성 여부
 */
void motor_dc_set_enabled(enum motor_index index, bool enabled);

/**
 * @brief DC 모터에 전압을 인가한다.
 * 
 * @param index 왼쪽 모터(MOTOR_DC_LEFT) 또는 오른쪽 모터(MOTOR_DC_RIGHT)
 * @param input_voltage 모터에 인가할 전압
 * @param supply_voltage 모터 드라이버에 공급되는 전압
 */
void motor_dc_input_voltage(enum motor_index index, float input_voltage, float supply_voltage);

#endif