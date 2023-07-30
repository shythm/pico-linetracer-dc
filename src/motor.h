/**
 * @file motor.h
 * @author Seongho Lee (shythm@outlook.com)
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "pico/types.h"

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
 * @param index MOTOR_LEFT(왼쪽 모터) 또는 MOTOR_RIGHT(오른쪽 모터)
 * @param enabled true(활성화) 또는 false(비활성화)
 */
void motor_pwm_enabled(enum motor_index index, const bool enabled);

/**
 * @brief 모터 PWM의 듀티 사이클을 설정한다. 그리고 모터의 회전 방향도 설정한다.
 *
 * @param index MOTOR_LEFT(왼쪽 모터) 또는 MOTOR_RIGHT(오른쪽 모터)
 * @param duty_ratio -1에서 1사이의 실수
 */
void motor_set_pwm_duty_ratio(enum motor_index index, float duty_ratio);

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

#endif