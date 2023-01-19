/*
 ******************************************************************************
 * file         switch.c
 * author       Joonho Gwon
 * modified by  Seongho Lee (2023.01.19.)
 * brief        STM32F411 Nucleo Board에서 사용하던 Switch 라이브러리를
 *              Raspberry Pi Pico에 맞게 포팅 및 수정한 소스코드
 ******************************************************************************
 */

#ifndef __SWITCH_H_
#define __SWITCH_H_

#define SWITCH_LEFT_GPIO  0
#define SWITCH_RIGHT_GPIO 1

#define SWITCH_EVENT_NONE   0x00  // 00
#define SWITCH_EVENT_LEFT   0x01  // 01
#define SWITCH_EVENT_RIGHT  0x02  // 10
#define SWITCH_EVENT_BOTH   0x03  // 11

#define SWITCH_TIME_SHORT  50    // Bouncing 방지를 위해 클릭 후 잠시 대기하는 시간(ms)
#define SWITCH_TIME_LONG   250   // 스위치 클릭 이벤트 발생 주기(ms)

#include "pico/types.h"

void switch_init(void);
uint switch_read(void);

/*
 ******************************************************************************
 * User Settings
 *
 * SWITCH_LEFT_GPIO : left switch GPIO number of pico
 * SWITCH_RIGHT_GPIO : right swicth GPIO number of pico
 ******************************************************************************
 */

#endif /* INC_CUSTOM_SWITCH_H_ */
