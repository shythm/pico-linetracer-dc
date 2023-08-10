#ifndef _BUZZER_H_
#define _BUZZER_H_

#include "pico/types.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define BUZZER_GPIO (22)

static uint32_t buzzer_timer = 0;

/**
 * @brief 부저를 초기화한다.
 */
static void buzzer_init(void) {
    gpio_init(BUZZER_GPIO);
    gpio_set_dir(BUZZER_GPIO, GPIO_OUT);
    gpio_pull_down(BUZZER_GPIO);

    gpio_put(BUZZER_GPIO, false);
}

/**
 * @brief 지정된 소리 발생 시간 동안 부저에 소리를 발생시킨다.
 */
static inline void buzzer_update() {
    gpio_put(BUZZER_GPIO, time_us_32() < buzzer_timer);
}

/**
 * @brief 부저 소리 발생 시간을 설정한다.
 *
 * @param time_ms 시간(ms)
 * @param overwrite true(어떠한 경우에도 시간을 덮어 씌운다) 또는 false(소리 발생 중이면 덮어 씌우지 않는다)
 */
static inline void buzzer_out(uint32_t time_ms, bool overwrite) {
    uint32_t timer = time_us_32() + (time_ms * 1000);

    if (overwrite || timer > buzzer_timer) {
        buzzer_timer = timer;
    }
}

#endif