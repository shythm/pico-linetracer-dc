#include <stdio.h>
#include "pico/stdlib.h"

#include "sensing.h"
#include "timer.h"
#include "motor_dc.h"

void sensing_handler(void) {
  sensing_voltage();
}

int main(void) {
  stdio_init_all();
  sensing_init();
  motor_dc_init();

  timer_periodic_start(2, 500, sensing_handler);

  float input_voltage_left = -12.0f;
  float input_voltage_right = 12.0f;
  motor_dc_set_enabled(MOTOR_DC_LEFT, true);
  motor_dc_set_enabled(MOTOR_DC_RIGHT, true);

  for (;;) {
    motor_dc_input_voltage(MOTOR_DC_LEFT,  input_voltage_left, voltage);
    motor_dc_input_voltage(MOTOR_DC_RIGHT, input_voltage_right, voltage);
  }

  return 0;
}