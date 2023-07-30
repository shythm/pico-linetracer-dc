#include <stdio.h>
#include "pico/stdlib.h"

#include "switch.h"
#include "sensing.h"
#include "timer.h"
#include "motor.h"
#include "oled.h"
#include "drive.h"

#include "hardware/clocks.h"

#define MENU_NUM 9

void calibrate(void);
void cal_test(void);
void state_test(void);
void set_threshold(void);
void position_test(void);

void motor_test_position_control(void) {
    motor_start();
    float tv = 0.0f;

    for (;;) {
        uint sw = switch_read_wait_ms(100);
        struct motor_control_state_t csl = motor_get_control_state(MOTOR_LEFT);
        struct motor_control_state_t csr = motor_get_control_state(MOTOR_RIGHT);

        if (sw == SWITCH_EVENT_LEFT)
            tv -= 0.1f;
        else if (sw == SWITCH_EVENT_RIGHT)
            tv += 0.1f;
        else if (sw == SWITCH_EVENT_BOTH)
            break;

        motor_set_velocity(MOTOR_LEFT, tv);
        motor_set_velocity(MOTOR_RIGHT, tv);

        oled_printf("/0MotorPosTest");
        oled_printf("/1tv: %1.2f", tv);
        oled_printf("/2l_error/3%+10d", csl.error);
        oled_printf("/4r_error/5%+10d", csr.error);
    }

    motor_stop();
}

void (*menu_fp[MENU_NUM])(void) = {
    motor_test_position_control,
    motor_pwm_test,
    calibrate,
    cal_test,
    set_threshold,
    state_test,
    position_test,
    mark_test,
    drive_first,
};

char menu_name[MENU_NUM][16] = {
    "MotorPosTest",
    "PWM Test",
    "Calibration",
    "Calib Test",
    "Set Thresh",
    "State Test",
    "Pos Test",
    "Mark Test",
    "First Drive",
};

void menu_select(void) {
    uint menu_index = 0;

    for (;;) {
        uint sw = switch_read_wait_ms(100);

        oled_printf("/0   -MENU-   ");
        oled_printf("/1%s", menu_name[menu_index]);

        if (sw == SWITCH_EVENT_BOTH) {
            oled_clear_all();
            menu_fp[menu_index]();
            oled_clear_all();
        } else if (sw == SWITCH_EVENT_RIGHT) {
            oled_printf("/1                ");
            menu_index = (menu_index + 1) % MENU_NUM;
        } else if (sw == SWITCH_EVENT_LEFT) {
            oled_printf("/1                ");
            menu_index = (MENU_NUM + menu_index - 1) % MENU_NUM;
        }
    }
}

int main(void) {
    stdio_init_all();

    switch_init();
    sensing_init();
    motor_init();
    oled_init();
    oled_clear_all();

    sensing_set_enabled(true); // voltage sensing, ir sensing 수행
    menu_select();
    return 0;
}

void calibrate(void) {
    uint sw;
    uint maximum[16] = {
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0
    };

    oled_printf("/0 Black Max  ");
    for (;;) {
        sw = switch_read_wait_ms(100);
        if (sw == SWITCH_EVENT_BOTH)
            break;
        for (int i = 0; i < 16; i++) {
            if (maximum[i] < sensing_get_ir_raw(i))
                maximum[i] = sensing_get_ir_raw(i);
        }
        oled_printf("/1%2x %2x %2x %2x", (uint8_t)maximum[0], (uint8_t)maximum[1], (uint8_t)maximum[2], (uint8_t)maximum[3]);
        oled_printf("/2%2x %2x %2x %2x", (uint8_t)maximum[4], (uint8_t)maximum[5], (uint8_t)maximum[6], (uint8_t)maximum[7]);
        oled_printf("/3%2x %2x %2x %2x", (uint8_t)maximum[8], (uint8_t)maximum[9], (uint8_t)maximum[10], (uint8_t)maximum[11]);
        oled_printf("/4%2x %2x %2x %2x", (uint8_t)maximum[12], (uint8_t)maximum[13], (uint8_t)maximum[14], (uint8_t)maximum[15]);
    }
    for (int i = 0; i < 16; i++)
        sensing_ir_bias[i] = maximum[i];

    oled_clear_all();
    oled_printf("/0 White Max  ");
    for (;;) {
        sw = switch_read_wait_ms(100);
        if (sw == SWITCH_EVENT_BOTH)
            break;
        for (int i = 0; i < 16; i++) {
            if (maximum[i] < sensing_get_ir_raw(i))
                maximum[i] = sensing_get_ir_raw(i);
        }
        oled_printf("/1%2x %2x %2x %2x", (uint8_t)maximum[0], (uint8_t)maximum[1], (uint8_t)maximum[2], (uint8_t)maximum[3]);
        oled_printf("/2%2x %2x %2x %2x", (uint8_t)maximum[4], (uint8_t)maximum[5], (uint8_t)maximum[6], (uint8_t)maximum[7]);
        oled_printf("/3%2x %2x %2x %2x", (uint8_t)maximum[8], (uint8_t)maximum[9], (uint8_t)maximum[10], (uint8_t)maximum[11]);
        oled_printf("/4%2x %2x %2x %2x", (uint8_t)maximum[12], (uint8_t)maximum[13], (uint8_t)maximum[14], (uint8_t)maximum[15]);
    }
    for (int i = 0; i < 16; i++)
        sensing_ir_range[i] = maximum[i] - sensing_ir_bias[i];
}

void cal_test(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0%2x %2x %2x %2x", (uint8_t)sensing_get_ir_normalized(0), (uint8_t)sensing_get_ir_normalized(1), (uint8_t)sensing_get_ir_normalized(2), (uint8_t)sensing_get_ir_normalized(3));
        oled_printf("/1%2x %2x %2x %2x", (uint8_t)sensing_get_ir_normalized(4), (uint8_t)sensing_get_ir_normalized(5), (uint8_t)sensing_get_ir_normalized(6), (uint8_t)sensing_get_ir_normalized(7));
        oled_printf("/2%2x %2x %2x %2x", (uint8_t)sensing_get_ir_normalized(8), (uint8_t)sensing_get_ir_normalized(9), (uint8_t)sensing_get_ir_normalized(10), (uint8_t)sensing_get_ir_normalized(11));
        oled_printf("/3%2x %2x %2x %2x", (uint8_t)sensing_get_ir_normalized(12), (uint8_t)sensing_get_ir_normalized(13), (uint8_t)sensing_get_ir_normalized(14), (uint8_t)sensing_get_ir_normalized(15));
    }
}

void set_threshold(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_LEFT)
            sensing_ir_threshold -= 0.02f;
        else if (sw == SWITCH_EVENT_RIGHT)
            sensing_ir_threshold += 0.02f;
        else if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0    Set    ");
        oled_printf("/1 Threshold ");
        oled_printf("/2    %1.2f", sensing_ir_threshold);
    }
}

void state_test(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0%1x%1x%1x%1x%1x%1x%1x%1x%1x%1x%1x%1x", ((uint16_t)sensing_get_ir_state() >> 13) & 1, ((uint16_t)sensing_get_ir_state() >> 12) & 1, ((uint16_t)sensing_get_ir_state() >> 11) & 1, ((uint16_t)sensing_get_ir_state() >> 10) & 1, ((uint16_t)sensing_get_ir_state() >> 9) & 1, ((uint16_t)sensing_get_ir_state() >> 8) & 1, ((uint16_t)sensing_get_ir_state() >> 7) & 1, ((uint16_t)sensing_get_ir_state() >> 6) & 1, ((uint16_t)sensing_get_ir_state() >> 5) & 1, ((uint16_t)sensing_get_ir_state() >> 4) & 1, ((uint16_t)sensing_get_ir_state() >> 3) & 1, ((uint16_t)sensing_get_ir_state() >> 2) & 1);
        oled_printf("/1%1x          %1x", ((uint16_t)sensing_get_ir_state() >> 14) & 1, ((uint16_t)sensing_get_ir_state() >> 1) & 1);
        oled_printf("/2%1x          %1x", ((uint16_t)sensing_get_ir_state() >> 15) & 1, ((uint16_t)sensing_get_ir_state() >> 0) & 1);
    }
}

void position_test(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0  Pos Test  ");
        oled_printf("/1   %5d   ", sensing_get_position());
    }
}