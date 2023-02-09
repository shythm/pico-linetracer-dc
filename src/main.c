#include <stdio.h>
#include "pico/stdlib.h"

#include "switch.h"
#include "sensing.h"
#include "timer.h"
#include "motor_dc.h"
#include "oled.h"

#include "hardware/clocks.h"

#define MENU_NUM 7

void motor_test(void);
void sensor_test(void);
void calibrate(void);
void cal_test(void);
void state_test(void);
void set_threshold(void);
void position_test(void);

void (*menu_fp[MENU_NUM])(void) = {
    motor_test,
    sensor_test,
    calibrate,
    cal_test,
    set_threshold,
    state_test,
    position_test
};

char menu_name[MENU_NUM][16] = {
    "Motor Test",
    "Sensor Test",
    "Calibration",
    "Calib Test",
    "Set Thresh",
    "State Test",
    "Pos Test"
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

void sensing_handler(void) {
    sensing_voltage();
    sensing_infrared();
}

int main(void) {
    stdio_init_all();

    switch_init();
    sensing_init();
    motor_dc_init();
    oled_init();
    oled_clear_all();
    timer_periodic_start(2, 500, sensing_handler);

    menu_select();
    return 0;
}

void motor_test(void) {
    float tv = 0.f;
    motor_dc_control_enabled(true);

    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_LEFT) {
            tv -= 0.1f;
        } else if (sw == SWITCH_EVENT_RIGHT) {
            tv += 0.1f;
        } else if (sw == SWITCH_EVENT_BOTH) {
            motor_dc_control_enabled(false);
            break;
        }
        motor_dc_set_velocity(MOTOR_DC_LEFT, tv);
        motor_dc_set_velocity(MOTOR_DC_RIGHT, tv);
        oled_printf("/0tv: %2.2f", tv);
        oled_printf("/1spdl:%2.4f", motor_dc_get_current_velocity(MOTOR_DC_LEFT));
        oled_printf("/2spdr:%2.4f", motor_dc_get_current_velocity(MOTOR_DC_RIGHT));
    }
}

void sensor_test(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0%2x %2x %2x %2x", (uint8_t)sensor_raw[0], (uint8_t)sensor_raw[1], (uint8_t)sensor_raw[2], (uint8_t)sensor_raw[3]);
        oled_printf("/1%2x %2x %2x %2x", (uint8_t)sensor_raw[4], (uint8_t)sensor_raw[5], (uint8_t)sensor_raw[6], (uint8_t)sensor_raw[7]);
        oled_printf("/2%2x %2x %2x %2x", (uint8_t)sensor_raw[8], (uint8_t)sensor_raw[9], (uint8_t)sensor_raw[10], (uint8_t)sensor_raw[11]);
        oled_printf("/3%2x %2x %2x %2x", (uint8_t)sensor_raw[12], (uint8_t)sensor_raw[13], (uint8_t)sensor_raw[14], (uint8_t)sensor_raw[15]);
    }
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
            if (maximum[i] < sensor_raw[i])
                maximum[i] = sensor_raw[i];
        }
        oled_printf("/1%2x %2x %2x %2x", (uint8_t)maximum[0], (uint8_t)maximum[1], (uint8_t)maximum[2], (uint8_t)maximum[3]);
        oled_printf("/2%2x %2x %2x %2x", (uint8_t)maximum[4], (uint8_t)maximum[5], (uint8_t)maximum[6], (uint8_t)maximum[7]);
        oled_printf("/3%2x %2x %2x %2x", (uint8_t)maximum[8], (uint8_t)maximum[9], (uint8_t)maximum[10], (uint8_t)maximum[11]);
        oled_printf("/4%2x %2x %2x %2x", (uint8_t)maximum[12], (uint8_t)maximum[13], (uint8_t)maximum[14], (uint8_t)maximum[15]);
    }
    for (int i = 0; i < 16; i++)
        sensor_coef_bias[i] = maximum[i];

    oled_clear_all();
    oled_printf("/0 White Max  ");
    for (;;) {
        sw = switch_read_wait_ms(100);
        if (sw == SWITCH_EVENT_BOTH)
            break;
        for (int i = 0; i < 16; i++) {
            if (maximum[i] < sensor_raw[i])
                maximum[i] = sensor_raw[i];
        }
        oled_printf("/1%2x %2x %2x %2x", (uint8_t)maximum[0], (uint8_t)maximum[1], (uint8_t)maximum[2], (uint8_t)maximum[3]);
        oled_printf("/2%2x %2x %2x %2x", (uint8_t)maximum[4], (uint8_t)maximum[5], (uint8_t)maximum[6], (uint8_t)maximum[7]);
        oled_printf("/3%2x %2x %2x %2x", (uint8_t)maximum[8], (uint8_t)maximum[9], (uint8_t)maximum[10], (uint8_t)maximum[11]);
        oled_printf("/4%2x %2x %2x %2x", (uint8_t)maximum[12], (uint8_t)maximum[13], (uint8_t)maximum[14], (uint8_t)maximum[15]);
    }
    for (int i = 0; i < 16; i++)
        sensor_coef_range[i] = maximum[i] - sensor_coef_bias[i];
}

void cal_test(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0%2x %2x %2x %2x", (uint8_t)sensor_normalized[0], (uint8_t)sensor_normalized[1], (uint8_t)sensor_normalized[2], (uint8_t)sensor_normalized[3]);
        oled_printf("/1%2x %2x %2x %2x", (uint8_t)sensor_normalized[4], (uint8_t)sensor_normalized[5], (uint8_t)sensor_normalized[6], (uint8_t)sensor_normalized[7]);
        oled_printf("/2%2x %2x %2x %2x", (uint8_t)sensor_normalized[8], (uint8_t)sensor_normalized[9], (uint8_t)sensor_normalized[10], (uint8_t)sensor_normalized[11]);
        oled_printf("/3%2x %2x %2x %2x", (uint8_t)sensor_normalized[12], (uint8_t)sensor_normalized[13], (uint8_t)sensor_normalized[14], (uint8_t)sensor_normalized[15]);
    }
}

void set_threshold(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_LEFT)
            sensor_threshold -= 0.02f;
        else if (sw == SWITCH_EVENT_RIGHT)
            sensor_threshold += 0.02f;
        else if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0    Set    ");
        oled_printf("/1 Threshold ");
        oled_printf("/2    %1.2f", sensor_threshold);
    }
}

void state_test(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0%1x%1x%1x%1x%1x%1x%1x%1x%1x%1x%1x%1x", ((uint16_t)sensor_state >> 2) & 1, ((uint16_t)sensor_state >> 3) & 1, ((uint16_t)sensor_state >> 4) & 1, ((uint16_t)sensor_state >> 5) & 1, ((uint16_t)sensor_state >> 6) & 1, ((uint16_t)sensor_state >> 7) & 1, ((uint16_t)sensor_state >> 8) & 1, ((uint16_t)sensor_state >> 9) & 1, ((uint16_t)sensor_state >> 10) & 1, ((uint16_t)sensor_state >> 11) & 1, ((uint16_t)sensor_state >> 12) & 1, ((uint16_t)sensor_state >> 13) & 1);
        oled_printf("/1%1x          %1x", ((uint16_t)sensor_state >> 1) & 1, ((uint16_t)sensor_state >> 14) & 1);
        oled_printf("/2%1x          %1x", ((uint16_t)sensor_state >> 0) & 1, ((uint16_t)sensor_state >> 15) & 1);
    }
}

void position_test(void) {
    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_BOTH)
            break;

        oled_printf("/0  Pos Test  ");
        oled_printf("/1   %5d", sensor_position);
    }
}