/**
 * @file timer.c
 * @author Seongho Lee
 * @brief Raspberry Pi Pico의 Alarm 기능을 이용하여 주기적으로 인터럽트를 발생시킬 수 있는 기능을 제공하는 라이브러리
 */
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

/*
 * Raspberry Pi Pico에는 지정된 주기마다 인터럽트를 발생시키는 기능이 "없다".
 * 하지만 Timer 내부에 Alarm이라는 기능이 있으며, 이 기능을 이용해서 Timer의 값이
 * 지정된 값에 도달할 때 인터럽트를 발생시킬 수 있다. 아래는 Alarm을 적절히 사용하여
 * 일정 주기마다 인터럽트를 발생시키는 기능을 구현한다.
 * 
 * 참고로, Raspberry Pi Pico에는 Timer가 하나이고 여기에 총 4개의 Alarm이 존재한다.
 */

// 각 인터럽트의 주기들
uint timer_periodic_intervals[4];

// 일정 주기에 도달했을 때 수행되는 함수들
static void (*timer_periodic_handlers[4])(void);

/*
 * Timer의 Alarm 값을 [(현재 Timer 값) + (interval)]로 설정하여 (interval)가 지난 후
 * Alarm 기능이 동작하도록 설정한다. 즉, 인터럽트를 발생시킨다. 한편, 일정 주기로 인터럽트를
 * 발생시키기 위해서는 해당 함수를 인터럽트 함수가 끝날 때마다 실행시켜주면 된다.
 */
inline static void timer_set_alarm(uint num, uint interval) {
    timer_hw->alarm[num] = timer_hw->timerawl + interval;
}

/*
 * Timer의 각 Alarm에 대한 인터럽트 핸들러를 정의하는 매크로
 * 1. Alarm IRQ를 초기화 한다.
 * 2. Alarm에 등록된 함수를 수행한다.
 * 3. Alarm IRQ를 설정한다.
 * 4. 다음 주기에 Alarm이 수행되도록 설정한다.
 */
#define TIMER_IRQ_HANDLER(NUM)                               \
    static void timer_irq_handler_##NUM(void) {              \
        timer_set_alarm(NUM, timer_periodic_intervals[NUM]); \
        hw_clear_bits(&timer_hw->intr, 1u << NUM);           \
        timer_periodic_handlers[NUM]();                      \
        hw_set_bits(&timer_hw->inte, 1u << NUM);             \
}

// 총 4개의 Alarm IRQ 핸들러 등록
TIMER_IRQ_HANDLER(0);
TIMER_IRQ_HANDLER(1);
TIMER_IRQ_HANDLER(2);
TIMER_IRQ_HANDLER(3);
static const void const (*timer_irq_handlers[4])(void) = {
    timer_irq_handler_0,
    timer_irq_handler_1,
    timer_irq_handler_2,
    timer_irq_handler_3,
};

// 총 4개의 Alarm IRQ
static const uint const alarm_irqs[4] = {
    TIMER_IRQ_0,
    TIMER_IRQ_1,
    TIMER_IRQ_2,
    TIMER_IRQ_3,
};

void timer_periodic_start(uint num, uint interval, void (*handler)(void)) {
    // 알람 num에 대한 인터럽트 활성화
    hw_set_bits(&timer_hw->inte, 1u << num);

    // 알람 num에 대한 IRQ 핸들러 추가
    timer_periodic_handlers[num] = handler;
    irq_set_exclusive_handler(num, timer_irq_handlers[num]);

    // IRQ 활성화
    irq_set_enabled(alarm_irqs[num], true);

    // Alarm 값 설정
    timer_periodic_intervals[num] = interval;
    timer_set_alarm(num, interval);
}

void timer_periodic_stop(uint num) {
    hw_set_bits(&timer_hw->armed, 1u << num);
    irq_set_enabled(alarm_irqs[num], false);
    hw_clear_bits(&timer_hw->intr, 1u << num);
}