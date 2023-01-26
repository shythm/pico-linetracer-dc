/**
 * @file switch.h
 * @author Joonho Gwon (Modified by Seongho Lee)
 * @brief STM32F411 Nucleo Board에서 사용하면 Switch 라이브러리를 Raspberry Pi Pico에 맞게 포팅 및 수정한 소스코드
 * @date 2023-01-26
 */

#ifndef __SWITCH_H_
#define __SWITCH_H_

// 왼쪽 스위치 GPIO 번호
#define SWITCH_LEFT_GPIO  0
// 오른쪽 스위치 GPIO 번호
#define SWITCH_RIGHT_GPIO 1

enum switch_event {
    SWITCH_EVENT_NONE  = 0x00,  // 아무 스위치도 눌리지 않았을 때
    SWITCH_EVENT_LEFT  = 0x01,  // 왼쪽 스위치가 눌렸을 때
    SWITCH_EVENT_RIGHT = 0x02,  // 오른쪽 스위치가 눌렸을 때
    SWITCH_EVENT_BOTH  = 0x03,  // 양쪽 스위치가 모두 눌렸을 때
};

typedef enum switch_event switch_event_t;

// Bouncing 방지를 위해 클릭 후 잠시 대기하는 시간(ms)
#define SWITCH_TIME_SHORT  50
// 스위치 클릭 이벤트 발생 주기(ms)
#define SWITCH_TIME_LONG   250

#include "pico/types.h"

/**
 * @brief 스위치를 사용할 수 있도록 스위치 관련 GPIO 및 변수들을 내부적으로 초기화하는 함수.
 * 
 */
void switch_init(void);

/**
 * @brief 스위치 입력을 받아 스위치가 어떻게 눌렸는지 반환하는 함수.
 * 
 * @return switch_event_t 스위치의 눌림 이벤트
 */
switch_event_t switch_read(void);

/**
 * @brief 스위치 입력을 특정 시간동안 기다리는 함수.
 * 
 * 스위치 읽기 함수(switch_read)가 차지하는 CPU 점유 시간이 다른 함수에 비해 상대적으로 짧을 때, 스위치를 눌러도 인식이 안되는 현상이 발생한다.
 * 왜냐하면, 상대적으로 CPU 점유 시간이 긴 함수로 인해 스위치를 인식할 시간을 많이 갖지 못하기 때문이다.
 * 이런 경우에 이 함수를 사용해 인위적으로 스위치 입력 함수가 긴 CPU 점유 시간을 갖도록 하면 스위치 씹힘 문제를 해결할 수 있다.
 * 
 * @param ms 기다릴 시간 (단위 microsecond)
 * @return switch_event_t 스위치의 눌림 이벤트
 */
switch_event_t switch_read_wait_ms(uint ms);

#endif
