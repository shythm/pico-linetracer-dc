#ifndef __MOTOR_DC_H_
#define __MOTOR_DC_H_

#define MOTOR_DC_PWM_LEFT_GPIO        14
#define MOTOR_DC_PWM_RIGHT_GPIO       15
#define MOTOR_DC_DIRECTION_LEFT_GPIO  16
#define MOTOR_DC_DIRECTION_RIGHT_GPIO 17

// PWM 주파수가 가청 주파수 영역대면, 모터에서 귀에 거슬리는 고주파음이 들리게 된다.
// 따라서 가청 주파수에 해당하지 않는 20khz 정도를 PWM 주파수로 사용한다.
#define MOTOR_DC_PWM_FREQUENCY 20000

// 모터 드라이버에 입력 PWM 신호를 넣을 때, duty cycle이 너무 짧을 경우 해당 신호가 무시되는 현상이 발생한다.
// 이는 트랜지스터의 전형적인 특징이며, 이를 보정하기 위해 신호가 인식되는 최소한의 duty cycle을 정의한다.
#define MOTOR_DC_PWM_DEAD_ZONE 40

#define K_PROPORTIONAL 1.1
#define K_INTEGRAL     1.1
#define K_DERIVATIVE   1.1

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

extern float cur_velo[MOTOR_DC_COUNT]; // 각 모터의 현재 속도 (m/s)
extern float error_sum[MOTOR_DC_COUNT]; // 에러의 누적을 저장하는 변수

/**
 * @brief 현재 속도를 업데이트하는 함수. 인터럽트를 사용해 주기적으로 실행해야 제대로 작동한다.
 */
void update_velocity_from_encoder(void);

/**
 * @brief 목표 속도를 설정한다.
 *
 * @param target_vel 설정할 목표 속도(m/s)
 */
void set_target_velocity(float target_vel);

/**
 * @brief 설정한 목표 속도를 가져온다.
 */
float get_target_velocity(void);

/**
 * @brief dc모터 컨트롤을 하기 전에 관련 파라미터를 초기화한다.
 *
 * @param enabled 0 : dc모터 제어 종료 / 1 : dc모터 제어 시작
 */
void motor_dc_control_enabled(bool enabled);

#endif