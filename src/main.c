#include <stdio.h>
#include "pico/stdlib.h"

#include "switch.h"
#include "sensing.h"
#include "timer.h"
#include "motor_dc.h"
#include "oled.h"

void sensing_handler(void) {
  sensing_voltage();
}


int main(void) {
  stdio_init_all();
  
  switch_init();
  sensing_init();
  motor_dc_init();
  motor_control_init();
  oled_init();
  oled_clear_all();

  timer_periodic_start(2, 500, sensing_handler);

  float input_voltage = 0.f;
  bool motor_enabled = false;

  for (;;) {
    uint sw = switch_read_wait_ms(100);

    if (sw == SWITCH_EVENT_LEFT) {
      input_voltage -= 1.f;
    } else if (sw == SWITCH_EVENT_RIGHT) {
      input_voltage += 1.f;
    } else if (sw == SWITCH_EVENT_BOTH) {
      motor_enabled = !motor_enabled;
      motor_dc_set_enabled(MOTOR_DC_LEFT, motor_enabled);
      motor_dc_set_enabled(MOTOR_DC_RIGHT, motor_enabled);
      oled_clear_all();
    }

    if (motor_enabled) {
      motor_dc_input_voltage(MOTOR_DC_LEFT,  input_voltage, voltage);
      motor_dc_input_voltage(MOTOR_DC_RIGHT, input_voltage, voltage);
      oled_printf("/0Vmo: %2.1f", input_voltage);
      oled_printf("/1Vin: %2.1f", voltage);
      oled_printf("/2spd0:%2.4f", cur_velo[0]);
      oled_printf("/3spd1:%2.4f", cur_velo[1]);
    } else {
      oled_printf("/0motor disabl/1ced.");
    }
  }

  return 0;
}
