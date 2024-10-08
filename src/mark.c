#include <stdlib.h>
#include <string.h>

#include "mark.h"
#include "switch.h"
#include "oled.h"
#include "sensing.h"
#include "motor.h"

#define MARK_STATE_READY        0x01
#define MARK_STATE_ACCUMULATION 0x02
#define MARK_STATE_DECISION     0x04

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

struct mark_state_t mark_init_state(void) {
    struct mark_state_t ret;

    ret.state = MARK_STATE_READY;
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

enum mark_t mark_update_state(struct mark_state_t *state) {
    const sensing_ir_state_t ir_state = sensing_ir_state;
    bool is_any = ir_state & state->both;
    bool is_line_6 = __builtin_popcount(ir_state & state->center) >= 6;

    switch (state->state) {
    case MARK_STATE_READY: // 일반 주행 중일 때

        /*
         * 마커 센서가 어느 하나라도 잡혔다 -> 왼쪽, 오른쪽, 크로스, 엔드 가능성
         * 라인 센서가 많이 잡혔다 -> 크로스 가능성, 곡선 구간이어서 라인 센서가 많이 잡힐 수 있음
         */
        if (is_any || is_line_6) {
            state->state = MARK_STATE_ACCUMULATION;
            state->accumulate = ir_state;
            state->motor = ir_state & state->left ? MOTOR_LEFT : MOTOR_RIGHT;
            state->encoder = abs(motor_get_encoder_value(state->motor)) + MARK_LENGTH_TICK;
        }
        break;

    case MARK_STATE_ACCUMULATION:

        state->accumulate |= ir_state; // 마크 판단을 위해 센서 상태를 누적한다.

        /*
         * 트레이서가 마크의 길이(약 2cm)를 지난 후에 상태 전이를 결정한다.
         * -> 트레이서가 뜨거나 노이즈로 인해 마크를 잠시 동안 못 볼 수 있는 경우를 방지한다.
         */
        if (state->encoder < abs(motor_get_encoder_value(state->motor))) {

            /*
             * 트레이서가 마크를 지난 것 같음에도 불구하고 마크가 잡힌다면,
             * 아직 완전히 그 구간을 지나지 않은 것으로 판단하여, 다시 누적 상태로 돌아간다. (또 마크의 길이 만큼 더 누적할 것)
             *
             * 다만, 이 순간에도 트레이서가 뜨거나 노이즈로 인해 마크가 있음에도 불구하고 못 볼 수 있을 것이다.
             * 어쩔 수 없는 경우로, 죄우 마크를 매번 검사하는 것 보다는 잘못 볼 확률을 낮출 수 있기에 이 방식을 채택한다.
             */
            if (is_any || is_line_6) {
                state->encoder = abs(motor_get_encoder_value(state->motor)) +
                                 MARK_LENGTH_TICK / 2;
            } else {
                state->state = MARK_STATE_DECISION;
            }
        }
        break;

    case MARK_STATE_DECISION:

        state->state = MARK_STATE_READY;

        // if (__builtin_popcount(state->accumulate) >= 12) {
        if (state->accumulate == 0xFFFF) {
            return MARK_CROSS;
        }
        bool is_left = state->accumulate & state->left;
        bool is_right = state->accumulate & state->right;
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

    struct mark_state_t mark_state = mark_init_state();

    oled_clear();
    oled_printf("/0Mark Live Test");
    while (!switch_read()) {
        mark_update_window(&mark_state, sensing_ir_position);
        enum mark_t mark = mark_update_state(&mark_state);

        switch (mark_state.state) {
        case MARK_STATE_READY:
            oled_printf("/0STATE: READY    ");
            break;
        case MARK_STATE_ACCUMULATION:
            oled_printf("/0STATE: ACCUM    ");
            break;
        case MARK_STATE_DECISION:
            oled_printf("/0STATE: DECISION ");
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
