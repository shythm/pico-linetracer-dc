#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "drive.h"
#include "fs.h"
#include "mark.h"
#include "motor.h"
#include "oled.h"
#include "sensing.h"
#include "switch.h"
#include "timer.h"
#include "buzzer.h"

#define UPDATE_PARAMETER(sw, param, delta) \
    if (sw == SWITCH_EVENT_BOTH)           \
        break;                             \
    else if (sw == SWITCH_EVENT_LEFT)      \
        (param) -= (delta);                \
    else if (sw == SWITCH_EVENT_RIGHT)     \
        (param) += (delta);

void test_motor_pwm(void) {
    float duty_ratio = 0;

    oled_clear();
    motor_pwm_enabled(true);
    for (;;) {
        uint sw = switch_read();
        UPDATE_PARAMETER(sw, duty_ratio, 0.1f);

        motor_set_pwm_duty_ratio(MOTOR_LEFT, duty_ratio);
        motor_set_pwm_duty_ratio(MOTOR_RIGHT, duty_ratio);

        oled_printf("/0PWM Test");
        oled_printf("/1duty ratio/2%1.2f", duty_ratio);
    }
    motor_pwm_enabled(false);
}

static int32_t _position_left = 0;
static int32_t _position_right = 0;

void _position_commander(int32_t *const left, int32_t *const right) {
    *left = _position_left;
    *right = _position_right;
}

void test_motor_control(void) {
    _position_left = motor_get_encoder_value(MOTOR_LEFT);
    _position_right = motor_get_encoder_value(MOTOR_RIGHT);

    oled_clear();
    sensing_start();
    motor_control_start(_position_commander);
    for (;;) {
        uint sw = switch_read();

        if (sw == SWITCH_EVENT_LEFT)
            _position_left += 50;
        else if (sw == SWITCH_EVENT_RIGHT)
            _position_right += 50;
        else if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0Motor Ctrl Test");
        oled_printf("/1position left /2%10d", _position_left);
        oled_printf("/3position right/4%10d", _position_right);
    }
    motor_control_stop();
    sensing_stop();
}

static void _display_ir(volatile const int *const arr) {
#define SEL(X, IDX) ((uint8_t)((*((X) + (IDX))) & 0xFF))
    oled_printf("/0/g 4 5 6 7 8 9 A B");
    oled_printf("/1%02x%02x%02x%02x%02x%02x%02x%02x",
                SEL(arr, 0x4), SEL(arr, 0x5), SEL(arr, 0x6), SEL(arr, 0x7),
                SEL(arr, 0x8), SEL(arr, 0x9), SEL(arr, 0xA), SEL(arr, 0xB));
    oled_printf("/2/g3 /w%02x        %02x/g C", SEL(arr, 0x3), SEL(arr, 0xC));
    oled_printf("/3/g2 /w%02x        %02x/g D", SEL(arr, 0x2), SEL(arr, 0xD));
    oled_printf("/4/g1 /w%02x        %02x/g E", SEL(arr, 0x1), SEL(arr, 0xE));
    oled_printf("/5/g0 /w%02x        %02x/g F", SEL(arr, 0x0), SEL(arr, 0xF));
#undef SEL
}

void test_ir_normalized(void) {
    sensing_start();
    oled_clear();
    while (!switch_read()) {
        _display_ir(sensing_ir_normalized);
    }
    sensing_stop();
}

void test_ir_state(void) {
    enum switch_event_t sw;
    char state_str[17] = "0000000000000000";

    sensing_start();
    oled_clear();
    for (;;) {
        for (int i = 0; i < SENSING_IR_COUNT; i++) {
            state_str[i] = (sensing_ir_state & (0x8000 >> i)) ? '1' : '0';
        }
        oled_printf("/0%s", state_str);
        oled_printf("/1threshold: %1.2f", sensing_ir_threshold);

        sw = switch_read();
        UPDATE_PARAMETER(sw, sensing_ir_threshold, 0.02f);
    }
    sensing_stop();

    // flash 저장
    oled_clear();
    oled_printf("/0Do you want/1to /gsave/2/wthreshold?/4(YES // NO)");
    sw = switch_wait_until_input();
    if (sw == SWITCH_EVENT_LEFT) {
        struct fs_data_t *fs = fs_get_data();
        fs->sensing_ir_threshold = sensing_ir_threshold;
        fs_flush_data();
    }
}

void test_ir_position(void) {
    sensing_start();
    oled_clear();
    while (!switch_read()) {
        oled_printf("/0position: %6d", sensing_ir_position);
        oled_printf("/1/rlimitpos: %6d", sensing_ir_position_limited);
    }
    sensing_stop();
}

void test_buzzer(void) {
    buzzer_init();

    oled_clear();
    oled_printf("/0Buzzer Test");
    oled_printf("/1(1000ms // 100ms)");
    for (;;) {
        buzzer_update();
        uint sw = switch_read();

        if (sw == SWITCH_EVENT_LEFT)
            buzzer_out(1000, false);
        else if (sw == SWITCH_EVENT_RIGHT)
            buzzer_out(100, true);
        else if (sw == SWITCH_EVENT_BOTH)
            break;
    }

    buzzer_out(0, true);
}

void print_saved_map(void) {
    const struct fs_data_t *const fs_data = fs_get_data();
    int index = 0;

    oled_clear();
    for (;;) {
        oled_printf("/0/gmark");
        switch (fs_data->detected_mark[index]) {
        case MARK_LEFT:
            oled_printf("/1/wleft");
            break;
        case MARK_RIGHT:
            oled_printf("/1/wright");
            break;
        case MARK_BOTH:
            oled_printf("/1/wboth");
            break;
        case MARK_CROSS:
            oled_printf("/1/wcorss");
            break;
        }

        oled_printf("/2/gposition/3/w%d", fs_data->detected_tick[index]);
        oled_printf("/6%d//%d", index, fs_data->detected_mark_count - 1);

        uint sw = switch_wait_until_input();
        UPDATE_PARAMETER(sw, index, 1);
        oled_clear();
    }
}

static void calibration(void) {
    enum switch_event_t sw;
    int temp[SENSING_IR_COUNT] = {
        0,
    };

    sensing_start();

    // blackmax 구하기
    oled_clear();
    oled_printf("/6    /bblackmax    ");
    while (!switch_read()) {
        _display_ir(temp);
        for (int i = 0; i < SENSING_IR_COUNT; i++) {
            if (temp[i] < sensing_ir_raw[i]) {
                temp[i] = sensing_ir_raw[i];
            }
        }
    }

    // bias(blackmax) 설정
    for (int i = 0; i < SENSING_IR_COUNT; i++) {
        sensing_ir_bias[i] = temp[i];
    }

    // whitemax 구하기
    oled_clear();
    oled_printf("/6    /rwhitemax    ");
    while (!switch_read()) {
        _display_ir(temp);
        for (int i = 0; i < SENSING_IR_COUNT; i++) {
            if (temp[i] < sensing_ir_raw[i]) {
                temp[i] = sensing_ir_raw[i];
            }
        }
    }

    // range(whitemax - blackmax) 설정
    for (int i = 0; i < SENSING_IR_COUNT; i++) {
        sensing_ir_range[i] = temp[i] - sensing_ir_bias[i];
    }

    sensing_stop();

    // 정규화 테스트
    oled_clear();
    oled_printf("/0Do you want/1to /gwatch/2/wcalib value?/4(YES // NO)");
    sw = switch_wait_until_input();
    if (sw == SWITCH_EVENT_LEFT) {
        test_ir_normalized();
    }

    // flash 저장
    oled_clear();
    oled_printf("/0Do you want/1to /gsave/2/wcalib value?/4(YES // NO)");
    sw = switch_wait_until_input();
    if (sw == SWITCH_EVENT_LEFT) {
        oled_printf("/6Saving ...");
        struct fs_data_t *fs = fs_get_data();
        for (int i = 0; i < SENSING_IR_COUNT; i++) {
            fs->sensing_ir_bias[i] = sensing_ir_bias[i];
            fs->sensing_ir_range[i] = sensing_ir_range[i];
        }
        fs_flush_data();
    }
}

void do_format_flash(void) {
    oled_clear();

    oled_printf("/0Do you /rreally/1/wwant to /rformat/2/wthe flash?/3(YES // NO)");
    enum switch_event_t sw = switch_wait_until_input();
    if (sw == SWITCH_EVENT_LEFT) {
        bool result = fs_format();
        oled_printf("/6%s", result ? "/rFailed." : "/gSuccess!");
        switch_wait_until_input();
    }
}

static const struct menu_t {
    char *name;
    void (*func)(void);
} menu[] = {
    { "Calibration", calibration },
    { "IR Sensor Test", test_ir_normalized },
    { "IR State Test", test_ir_state },
    { "IR Position Test", test_ir_position },
    { "Mark Live Test", mark_live_test },
    { "Motor PWM Test", test_motor_pwm },
    { "Motor Pos Test", test_motor_control },
    { "Flash Format", do_format_flash },
    { "Print Saved Map", print_saved_map },
    { "Buzzer Test", test_buzzer },
    { "First Drive", drive_first },
    { "Second Drive", drive_second },
};

int main(void) {
    stdio_init_all();
    fs_init();

    switch_init();
    oled_init();
    sensing_init();
    motor_init();

    const uint menu_index_max = sizeof(menu) / sizeof(struct menu_t) - 1;
    int menu_index = 0;

    for (;;) {
        oled_clear();
        oled_printf("/0/gMain Menu");
        oled_printf("/1%s", menu[menu_index].name);

        enum switch_event_t sw = switch_wait_until_input();
        if (sw == SWITCH_EVENT_BOTH)
            menu[menu_index].func();
        if (sw == SWITCH_EVENT_LEFT)
            menu_index--;
        if (sw == SWITCH_EVENT_RIGHT)
            menu_index++;

        if (menu_index < 0)
            menu_index = menu_index_max;
        if (menu_index > menu_index_max)
            menu_index = 0;
    }

    return 0;
}
