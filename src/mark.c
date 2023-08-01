#include "mark.h"
#include "switch.h"
#include "oled.h"

#define MARK_STATE_IDLE     0x01
#define MARK_STATE_CROSS    0x02
#define MARK_STATE_MARKER   0x04
#define MARK_STATE_DECISION 0x08

struct mark_state_t mark_init_state(sensing_ir_state_t left, sensing_ir_state_t right) {
    struct mark_state_t ret;

    ret.state = MARK_STATE_IDLE;
    ret.left = left;
    ret.right = right;
    ret.both = left | right;
    ret.center = ~left & ~right;

    return ret;
}

enum mark_t mark_update_state(struct mark_state_t *mark_state) {
    static sensing_ir_state_t accumulate;
    const sensing_ir_state_t ir_state = sensing_ir_state;

    accumulate |= ir_state; // 마크 판단을 위해 센서 상태를 누적한다

    bool is_line_6 = __builtin_popcount(mark_state->center & ir_state) >= 6;
    bool is_any = ir_state & mark_state->both;

    switch (mark_state->state) {
    case MARK_STATE_IDLE: // 일반 주행 중일 때
        accumulate = 0;

        if (is_line_6) {
            mark_state->state = MARK_STATE_CROSS;
        } else if (is_any) {
            mark_state->state = MARK_STATE_MARKER;
        }
        break;

    case MARK_STATE_CROSS: // 라인 교차 구간을 지나는 중일 때
        /**
         * accumulate를 검사하는 이유?
         *  -> 모든 센서들이 인식됐음은 곧 크로스 구간을 지나기 직전임을 의미한다.
         * is_any를 검사하는 이유?
         *  -> 처리 속도가 너무 빠른 나머지 뒤로 나와있는 센서들이 아직 크로스 위에 있을 수 있으므로 이 조건까지 검사한다.
         */
        if ((accumulate == MARK_MASK_ALL) && (!is_any)) {
            mark_state->state = MARK_STATE_DECISION;
        }
        break;

    case MARK_STATE_MARKER: // 마커 인식 중일 때
        if (!is_any) {
            mark_state->state = MARK_STATE_DECISION;
        }
        break;

    case MARK_STATE_DECISION: // 판단 단계
        mark_state->state = MARK_STATE_IDLE;

        if (accumulate == MARK_MASK_ALL) {
            return MARK_CROSS;
        }
        bool is_left = accumulate & mark_state->left;
        bool is_right = accumulate & mark_state->right;
        if (is_left && is_right) {
            return MARK_BOTH;
        }
        if (is_left) {
            return MARK_LEFT;
        }
        if (is_right) {
            return MARK_RIGHT;
        }
        break;
    }

    return MARK_NONE;
}

void mark_live_test(void) {
    sensing_start();

    struct mark_state_t mark_state =
        mark_init_state(MARK_MASK_LEFT_DEFAULT, MARK_MASK_RIGHT_DEFAULT);

    oled_clear();
    oled_printf("/0Mark Live Test");
    while (!switch_read()) {
        enum mark_t mark = mark_update_state(&mark_state);

        switch (mark_state.state) {
        case MARK_STATE_IDLE:
            oled_printf("/0STATE: IDLE     ");
            break;
        case MARK_STATE_CROSS:
            oled_printf("/0STATE: CROSS    ");
            break;
        case MARK_STATE_MARKER:
            oled_printf("/0STATE: MARK     ");
            break;
        default:
            oled_printf("/0STATE: -----    ");
            break;
        }

        switch (mark) {
        case MARK_LEFT:
            oled_printf("/1MARK: LEFT      ");
            break;
        case MARK_RIGHT:
            oled_printf("/1MARK: RIGHT     ");
            break;
        case MARK_BOTH:
            oled_printf("/1MARK: BOTH      ");
            break;
        case MARK_CROSS:
            oled_printf("/1MARK: CROSS     ");
            break;
        }
    }

    sensing_stop();
}
