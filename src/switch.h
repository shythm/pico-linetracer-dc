/**
 * @file switch.h
 */

#ifndef _SWITCH_H_
#define _SWITCH_H_

#include "pico.h"
#include "config.h"

enum switch_event_t {
    SWITCH_EVENT_NONE = 0x00, // 아무 스위치도 눌리지 않았을 때
    SWITCH_EVENT_LEFT = 0x01, // 왼쪽 스위치가 눌렸을 때
    SWITCH_EVENT_RIGHT = 0x02, // 오른쪽 스위치가 눌렸을 때
    SWITCH_EVENT_BOTH = 0x03, // 양쪽 스위치가 모두 눌렸을 때
};

/**
 * @brief 스위치를 사용할 수 있도록 스위치 관련 GPIO 및 변수들을 내부적으로 초기화하는 함수.
 */
void switch_init(void);

/**
 * @brief 스위치 입력을 받아 스위치가 어떻게 눌렸는지 반환하는 함수.
 *
 * @return switch_event_t 스위치의 눌림 이벤트
 */
enum switch_event_t switch_read(void);

/**
 * @brief 스위치에 입력이 들어올 때까지 기다리는 함수.
 *
 * @return enum switch_event_t 스위치의 눌림 이벤트
 */
inline static enum switch_event_t switch_wait_until_input(void) {
    enum switch_event_t sw;

    while (!(sw = switch_read())) {
        tight_loop_contents();
    }

    return sw;
}

#endif
