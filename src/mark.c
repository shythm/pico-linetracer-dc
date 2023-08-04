#include <string.h>

#include "mark.h"
#include "switch.h"
#include "oled.h"
#include "sensing.h"

#define MARK_STATE_IDLE     0x01
#define MARK_STATE_CROSS    0x02
#define MARK_STATE_MARKER   0x04
#define MARK_STATE_DECISION 0x08

static const sensing_ir_state_t MARK_STATE_LEFT[SENSING_IR_COUNT] = {
    0x0000, 0x0000, 0x0000, 0x0000, 0x8000, 0xC000, 0xE000, 0xF000, //
    0x7800, 0x3C00, 0x1E00, 0x0F00, 0x0780, 0x03C0, 0x01E0, 0x00F0, //
};

static const sensing_ir_state_t MARK_STATE_RIGHT[SENSING_IR_COUNT] = {
    0x0780, 0x03C0, 0x01E0, 0x00F0, 0x0078, 0x003C, 0x001E, 0x000F, //
    0x0007, 0x0003, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, //
};

// static const sensing_ir_state_t MARK_STATE_CENTER[SENSING_IR_COUNT] = {
//     0x1000, 0x0800, 0x8400, 0x4200, 0x2100, 0x1080, 0x0840, 0x0420, //
//     0x0210, 0x0108, 0x0084, 0x0042, 0x0021, 0x0010, 0x0008, 0x0004, //
// };

static const sensing_ir_state_t MARK_STATE_CENTER[SENSING_IR_COUNT] = {
    0xF800, 0xFC00, 0xFE00, 0xFF00, 0x7F80, 0x3FC0, 0x1FE0, 0x0FF0, //
    0x07F8, 0x03FC, 0x01FE, 0x00FF, 0x007F, 0x003F, 0x001F, 0x000F, //
};

struct mark_state_t mark_init_state(sensing_ir_state_t left, sensing_ir_state_t right) {
    struct mark_state_t ret;

    ret.state = MARK_STATE_IDLE;
    ret.left = MARK_STATE_LEFT[7];
    ret.right = MARK_STATE_RIGHT[7];
    ret.both = ret.left | ret.right;
    ret.center = MARK_STATE_CENTER[7];

    return ret;
}

void mark_update_window(struct mark_state_t *const state, const float position) {
    // 현재 position을 통해 16조 어느 IR 센서 위에 트레이서가 위치하는지 구한다. (0: 가장 왼쪽, 15: 가장 오른쪽)
    int where = (position + 30000) / 4000;
    state->left = MARK_STATE_LEFT[where];
    state->right = MARK_STATE_RIGHT[where];
    state->both = state->left | state->right;
    state->center = MARK_STATE_CENTER[where];
}

enum mark_t mark_update_state(struct mark_state_t *mark_state) {
    static sensing_ir_state_t accumulate;
    const sensing_ir_state_t ir_state = sensing_ir_state;

    accumulate |= ir_state; // 마크 판단을 위해 센서 상태를 누적한다

    bool is_any = ir_state & mark_state->both;

    switch (mark_state->state) {
    case MARK_STATE_IDLE: // 일반 주행 중일 때
        accumulate = 0;

        if (__builtin_popcount(mark_state->center & ir_state) >= 6) {
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
        if (__builtin_popcount(mark_state->center & accumulate) >= 6) {
            mark_state->state = MARK_STATE_CROSS;
            break;
        }
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
        mark_update_window(&mark_state, sensing_ir_position);
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

        char output[SENSING_IR_COUNT * 3 + 1];
        for (int i = 0; i < SENSING_IR_COUNT; i++) {
            sensing_ir_state_t pop = 1 << (0xf - i);

            if (mark_state.left & pop) {
                strncpy(output + 3 * i, "/r1", 3);
            } else if (mark_state.right & pop) {
                strncpy(output + 3 * i, "/b1", 3);
            } else if (mark_state.center & pop) {
                strncpy(output + 3 * i, "/y1", 3);
            } else {
                strncpy(output + 3 * i, "/w0", 3);
            }
        }
        oled_printf("/6%s", output);
    }

    sensing_stop();
}
