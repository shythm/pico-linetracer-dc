#ifndef _MARK_H_
#define _MARK_H_

#include "pico/types.h"
#include "sensing.h"
#include "config.h"

/**
 * @brief 마크의 종류를 나타내는 타입
 */
enum mark_t {
    MARK_NONE = 0, // 마크를 보지 않음 (기본)
    MARK_LEFT, // 왼쪽 마크
    MARK_RIGHT, // 오른쪽 마크
    MARK_BOTH, // 양쪽 마크
    MARK_CROSS, // 크로스 마크
};

struct mark_state_t {
    /**
     * @brief 센서 상태로부터 어떤 마크인지 판단하기 위해 사용되는 비트 마스크
     * 16조 센서의 경우 마크를 판단하는 기준(비트 마스크)이 바뀔 수 있다.
     */
    sensing_ir_state_t left, right, both, center;
    sensing_ir_state_t accumulate;
    int32_t encoder;
    int motor;
    uint state;
};

/**
 * @brief 초기화된 마크 상태를 반환한다.

 * @return struct mark_state_t
 */
struct mark_state_t mark_init_state(void);

/**
 * @brief 포지션에 따라 왼쪽 그리고 오른쪽 마크를 보는 기준을 변경한다 -> Mark Window 기능
 *
 * @param state struct mark_state_t
 * @param position 현재 센서의 위치
 */
void mark_update_window(struct mark_state_t *const state, const float position);

/**
 * @brief 마크를 판단하는 state machine을 수행한다.
 * 마크가 결정되지 않으면 MARK_NONE이, 마크가 결정되면 그에 따른 마크의 종류를 반환한다.
 *
 * @param mark_state state 변수
 * @return enum mark_t
 */
enum mark_t mark_update_state(struct mark_state_t *mark_state);

/**
 * @brief 마크 state machine의 동작을 실시간으로 확인해볼 수 있는 테스트 함수.
 */
void mark_live_test(void);

#endif