/**
 * @file motor_dc.h
 * @author Seongho Lee & Jinoo Li
 */

#ifndef __MOTOR_DC_H_
#define __MOTOR_DC_H_

#include "pico/types.h"
#include "hardware/pio.h"
#include "timer.h"

#ifdef INCLUDE_MOTOR_DC_CONSTANTS

/**
 * DC 모터 관련 상수들
 */

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

/**
 * 엔코더 관련 상수들
 */

#define MOTOR_ENCODER_PIO             pio0
#define MOTOR_ENCODER_BASE_LEFT_GPIO  10 // 왼쪽 모터 엔코터 A(베이스)핀 (B핀은 항상 그 다음 GPIO이다)
#define MOTOR_ENCODER_BASE_RIGHT_GPIO 12 // 오른쪽 모터 엔코더 A(베이스)핀 (B핀은 항상 그 다음 GPIO이다)
#define MOTOR_ENCODER_RESOLUTION      2048
#define MOTOR_WHEEL_DIAMETER_M        0.038f // 바퀴의 지름(m)
#define MOTOR_GEAR_RATIO              17.f / 69.f // 모터(17) / 바퀴(69) 기어비

/**
 * DC 모터 및 엔코더 방향 보정 상수들
 */

#define MOTOR_DC_DIRECTION_LEFT  1
#define MOTOR_DC_DIRECTION_RIGHT -1
#define MOTOR_ENCODER_REVERSE    true

/**
 * DC 모터 PID 제어 관련 상수들
 */

#define MOTOR_DC_GAIN_P            0.01f
#define MOTOR_DC_GAIN_I            0.01f
#define MOTOR_DC_ERROR_SUM_LIMIT   10.0f
#define MOTOR_DC_TIMER_SLOT        TIMER_SLOT_1
#define MOTOR_DC_TIMER_INTERVAL_US 500

#endif

enum motor_dc_index {
    MOTOR_DC_LEFT = 0,
    MOTOR_DC_RIGHT,
    MOTOR_DC_COUNT
};

/**
 * @brief DC 모터 드라이버 구동을 위해 PWM의 주기 설정 및 초기화를 진행하고,
 *        DC 모터의 회전 방향을 제어하는 Direction GPIO를 초기화한다.
 */
void motor_dc_init(void);

/**
 * @brief DC 모터 드라이버에 PWM 신호 생성 여부를 결정한다.
 *
 * @param index MOTOR_DC_LEFT(왼쪽 모터) 또는 MOTOR_DC_RIGHT(오른쪽 모터)
 * @param enabled PWM 신호 생성 여부
 */
void motor_dc_pwm_enabled(enum motor_dc_index index, bool enabled);

/**
 * @brief DC 모터에 전압을 인가한다.
 *
 * @param index MOTOR_DC_LEFT(왼쪽 모터) 또는 MOTOR_DC_RIGHT(오른쪽 모터)
 * @param input_voltage 모터에 인가할 전압
 * @param supply_voltage 모터 드라이버에 공급되는 전압
 */
void motor_dc_input_voltage(enum motor_dc_index index, float input_voltage, float supply_voltage);

/**
 * @brief 모터의 속도를 설정한다.
 *
 * @param index MOTOR_DC_LEFT(왼쪽 모터) 또는 MOTOR_DC_RIGHT(오른쪽 모터)
 * @param velocity 설정할 모터의 속도 (m/s)
 */
void motor_dc_set_velocity(enum motor_dc_index index, float velocity);

/**
 * @brief 설정한 모터의 속도를 가져온다.
 *
 * @param index MOTOR_DC_LEFT(왼쪽 모터) 또는 MOTOR_DC_RIGHT(오른쪽 모터)
 * @return 설정된 모터의 속도
 */
float motor_dc_get_velocity(enum motor_dc_index index);

/**
 * @brief 엔코더로부터 구한 모터의 속도를 가져온다.
 *
 * @param index MOTOR_DC_LEFT(왼쪽 모터) 또는 MOTOR_DC_RIGHT(오른쪽 모터)
 * @return 현재 모터의 속도
 */
float motor_dc_get_current_velocity(enum motor_dc_index index);

/**
 * @brief 엔코더의 현재 틱 수를 가져온다.
 *
 * @param index MOTOR_DC_LEFT(왼쪽 모터) 또는 MOTOR_DC_RIGHT(오른쪽 모터)
 * @return 틱 수
 */
uint motor_dc_get_encoder_count(enum motor_dc_index index);

/**
 * @brief DC 모터 제어 활성화 여부에 따른 초기화 작업을 수행한다.
 *
 * @param enabled 0(DC 모터 제어 종료) 또는 1(DC 모터 제어 시작)
 */
void motor_dc_control_enabled(bool enabled);

#endif