/**
 * @file timer.c
 * @author Seongho Lee
 * @brief Raspberry Pi Pico의 Alarm을 이용하여 주기적으로 인터럽트를 발생시킬 수 있는 기능을 제공하는 라이브러리
 */
#ifndef _TIMER_H_
#define _TIMER_H_

#include "pico/types.h"

enum timer_slot_index {
    TIMER_SLOT_0 = 0,
    TIMER_SLOT_1,
    TIMER_SLOT_2,
    TIMER_SLOT_3,
    TIMER_SLOT_COUNT,
};

/*! \brief 다른 코드의 진행 여부에 상관 없이 일정한 주기로 특정 함수를 실행할 수 있도록 도와주는 함수
 * \param index 타이머 슬롯 (TIMER_SLOT_0, TIMER_SLOT_1, ..., TIMER_SLOT_3)
 * \param interval 타이머 주기 (단위: microsecond(us))
 * \param handler 주기적으로 실행할 함수(핸들러)
 */
void timer_periodic_start(enum timer_slot_index index, uint interval, void (*handler)(void));

/*! \brief 주기적 타이머 멈춤 함수
 * \param index 타이머 슬롯 (TIMER_SLOT_0, TIMER_SLOT_1, ..., TIMER_SLOT_3)
 */
void timer_periodic_stop(enum timer_slot_index index);

#endif