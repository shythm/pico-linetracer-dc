#include <stdio.h>
#include "pico/stdlib.h"

#include "sensing.h"
#include "timer.h"

int i = 0;

void sensing_handler(void) {
  sensing_voltage();
}

int main(void) {
  stdio_init_all();
  sensing_init();

  timer_periodic_start(2, 500000, sensing_handler);

  for (;;) {
    printf("Encoder counter left: %d, right: %d \n", get_encoder_count(0), get_encoder_count(1));
    printf("Voltage: %f \n", voltage);
    sleep_ms(100);
  }

  return 0;
}