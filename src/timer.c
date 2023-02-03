/**
 * @file timer.c
 * @author Seongho Lee
 * @brief Raspberry Pi Pico의 Alarm을 이용하여 주기적으로 인터럽트를 발생시킬 수 있는 기능을 제공하는 라이브러리
 *
 * Raspberry Pi Pico에는 지정된 주기마다 인터럽트를 발생시키는 기능이 "없다".
 * 하지만 Timer 내부에 Alarm이라는 기능을 이용하여 Timer의 값이 정된 값에 도달할 때 인터럽트를 발생시킬 수 있다.
 * Alarm은 "1us마다 1씩 증가하는 Timer 카운터"가 지정된 값(ALARM 레지스터)에 도달하면 IRQ를 발생시키는 기능을 제공한다.
 * 여기서 Timer는 하나이고, 이 Timer에 4개의 Alarm이 존재한다. 그래서 주기가 다른 각 4개의 IRQ를 생성할 수 있다.
 * 아래의 소스코드들은 Alarm을 적절히 사용하여 일정 주기마다 인터럽트를 발생시키는 기능을 구현한다.
 */
#include "timer.h"

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

// 각 인터럽트의 주기들
uint timer_periodic_intervals[TIMER_SLOT_COUNT];

// 일정 주기에 도달했을 때 수행되는 함수들
static void (*timer_periodic_handlers[TIMER_SLOT_COUNT])(void);

/*
 * Timer의 Alarm 값을 [(현재 Timer 값) + (interval)]로 설정하여 (interval)가
 * 지난 후 Alarm 기능이 동작하도록 설정한다. 즉, 인터럽트를 발생시킨다. 한편,
 * 일정 주기로 인터럽트를 발생시키기 위해서는 해당 함수를 인터럽트 함수가 끝날
 * 때마다 실행시켜주면 된다.
 */
inline static void timer_set_alarm(enum timer_slot_index index, uint interval) {
    timer_hw->alarm[index] = timer_hw->timerawl + interval;
}

/*
 * Timer의 각 Alarm에 대한 인터럽트 핸들러를 정의하는 매크로
 * 1. 다음 주기에 Alarm이 수행될 수 있도록 ALRAM 레지스터를 설정한다.
 * 2. Alarm IRQ를 초기화 한다.
 * 3. Alarm에 등록된 함수를 수행한다.
 * 4. Alarm IRQ를 설정한다.
 */
#define TIMER_IRQ_HANDLER(SLOT)                                \
    static void timer_irq_handler_##SLOT(void) {               \
        timer_set_alarm(SLOT, timer_periodic_intervals[SLOT]); \
        hw_clear_bits(&timer_hw->intr, 1u << SLOT);            \
        timer_periodic_handlers[SLOT]();                       \
        hw_set_bits(&timer_hw->inte, 1u << SLOT);              \
    }

// 총 4개의 Alarm IRQ 핸들러 등록
TIMER_IRQ_HANDLER(0);
TIMER_IRQ_HANDLER(1);
TIMER_IRQ_HANDLER(2);
TIMER_IRQ_HANDLER(3);
static const void const (*timer_irq_handlers[TIMER_SLOT_COUNT])(void) = {
    timer_irq_handler_0,
    timer_irq_handler_1,
    timer_irq_handler_2,
    timer_irq_handler_3,
};

// 총 4개의 Alarm IRQ
static const uint const alarm_irqs[TIMER_SLOT_COUNT] = {
    TIMER_IRQ_0,
    TIMER_IRQ_1,
    TIMER_IRQ_2,
    TIMER_IRQ_3,
};

void timer_periodic_start(enum timer_slot_index index, uint interval, void (*handler)(void)) {
    // 알람 슬롯에 대한 인터럽트 활성화
    hw_set_bits(&timer_hw->inte, 1u << index);

    // 알람 슬롯에 대한 IRQ 핸들러 추가
    timer_periodic_handlers[index] = handler;
    irq_set_exclusive_handler(index, timer_irq_handlers[index]);

    // IRQ 활성화
    irq_set_enabled(alarm_irqs[index], true);

    // Alarm 값 설정
    timer_periodic_intervals[index] = interval;
    timer_set_alarm(index, interval);
}

void timer_periodic_stop(enum timer_slot_index index) {
    hw_set_bits(&timer_hw->armed, 1u << index);
    irq_set_enabled(alarm_irqs[index], false);
    hw_clear_bits(&timer_hw->intr, 1u << index);
}