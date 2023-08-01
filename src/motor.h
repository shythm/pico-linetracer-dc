/**
 * @file motor.h
 * @author Seongho Lee (shythm@outlook.com)
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "pico/types.h"
#include "config.h"

enum motor_index {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT,
};

/**
 * @brief 엔코더를 이용해 현재 모터의 위치를 반환한다.
 *
 * @param index MOTOR_LEFT(왼쪽 모터) 또는 MOTOR_RIGHT(오른쪽 모터)
 * @return int32_t 현재 모터의 위치
 */
int32_t motor_get_encoder_value(enum motor_index index);

/**
 * @brief 모터 PWM 활성화 여부를 결정한다.
 *
 * @param enabled true(활성화) 또는 false(비활성화)
 */
void motor_pwm_enabled(const bool enabled);

/**
 * @brief 모터 PWM의 듀티 사이클을 설정한다. 그리고 모터의 회전 방향도 설정한다.
 *
 * @param index MOTOR_LEFT(왼쪽 모터) 또는 MOTOR_RIGHT(오른쪽 모터)
 * @param duty_ratio -1에서 1사이의 실수
 */
void motor_set_pwm_duty_ratio(enum motor_index index, float duty_ratio);

struct motor_control_state_t {
    float gain_p;
    float gain_d;
    int32_t error;
    int32_t current;
    int32_t target;
};

/**
 * @brief 모터의 현재 제어 상태를 반환한다. 에러값, 목표값, 현재값 등을 확인할 수 있다.
 *
 * @param index MOTOR_LEFT(왼쪽 모터) 또는 MOTOR_RIGHT(오른쪽 모터)
 * @return struct motor_control_state_t 모터 제어 상태 구조체
 */
struct motor_control_state_t motor_get_control_state(enum motor_index index);

/**
 * @brief 모터 위치 PID 제어 시 목표량(Set Point, SP, Target)을 설정하는 함수 포인터 타입
 *
 * [예시]
 *   void target_updater(int32_t *const left, int32_t *const right) {
 *     *left = *left + 1;
 *     *right = *right + 2;
 *   }
 */
typedef void (*motor_target_updater_t)(int32_t *const left, int32_t *const right);

/**
 * @brief 모터 위치 PID 제어를 시작한다. 두 모터에 대해
 * 1. PWM 장치를 활성화하고,
 * 2. 모터 위치 PID 제어 관련 상태 변수들을 초기화하고,
 * 3. 목표량(위치) 갱신 함수를 등록하고,
 * 4. PID 제어 타이머를 시작한다.
 *
 * @param updater 모터 위치 PID 제어 시 목표량을 결정하는 함수
 */
void motor_control_start(const motor_target_updater_t updater);

/**
 * @brief 모터 위치 PID 제어를 종료한다. 두 모터에 대해
 * 1. PID 제어 타이머를 중지시킨다.
 * 2. PWM 장치를 비활성화한다.
 */
void motor_control_stop(void);

/**
 * @brief 모터 동작을 위해 모터 PWM 및 엔코더 장치를 초기화한다.
 */
void motor_init(void);

#endif