#include <stdio.h>
#include "pico/stdlib.h"

#include "switch.h"
#include "sensing.h"
#include "timer.h"
#include "motor_dc.h"
#include "oled.h"

#include "hardware/clocks.h"

void sensing_handler(void) {
    sensing_voltage();
}

int main(void) {
    stdio_init_all();

    switch_init();
    sensing_init();
    motor_dc_init();
    oled_init();
    oled_clear_all();

    timer_periodic_start(2, 500, sensing_handler);

    float tv = 0.f;
    bool motor_enabled = false;

    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_LEFT) {
            tv -= 0.1f;
        } else if (sw == SWITCH_EVENT_RIGHT) {
            tv += 0.1f;
        } else if (sw == SWITCH_EVENT_BOTH) {
            motor_enabled = !motor_enabled;

            motor_dc_control_enabled(motor_enabled);
            oled_clear_all();
        }
        set_target_velocity(tv);

        if (motor_enabled) {
            oled_printf("/0tv: %2.2f", tv);
            oled_printf("/1spdl:%2.4f", cur_velo[0]);
            oled_printf("/2spdr:%2.4f", cur_velo[1]);
            oled_printf("/3ersl:%2.1f", error_sum[0]);
            oled_printf("/4ersr:%2.1f", error_sum[1]);
        } else {
            oled_printf("/0motor disabl/1ced.");
            oled_printf("/2%ld", clock_get_hz(clk_sys));
        }
    }

    return 0;
}
