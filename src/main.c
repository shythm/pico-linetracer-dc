#include <stdio.h>
#include "pico/stdlib.h"

#include "switch.h"
#include "sensing.h"
#include "timer.h"
#include "motor_dc.h"

void sensing_handler(void) {
  sensing_voltage();
}

int main(void) {
  stdio_init_all();
  switch_init();
  sensing_init();
  motor_dc_init();

  timer_periodic_start(2, 500, sensing_handler);

  float input_voltage = 0.f;
  bool motor_enabled = false;

  for (;;) {
    uint sw = switch_read();

    if (sw == SWITCH_EVENT_LEFT) {
      input_voltage -= 1.f;
    } else if (sw == SWITCH_EVENT_RIGHT) {
      input_voltage += 1.f;
    } else if (sw == SWITCH_EVENT_BOTH) {
      motor_enabled = !motor_enabled;
      motor_dc_set_enabled(MOTOR_DC_LEFT, motor_enabled);
      motor_dc_set_enabled(MOTOR_DC_RIGHT, motor_enabled);
    }

    if (motor_enabled) {
      motor_dc_input_voltage(MOTOR_DC_LEFT,  input_voltage, voltage);
      motor_dc_input_voltage(MOTOR_DC_RIGHT, input_voltage, voltage);
      printf("input_voltage: %2.1f, current_voltage: %2.1f \r\n", input_voltage, voltage);
    } else {
      printf("motor disabled \r\n");
    }
  }

  return 0;
}
