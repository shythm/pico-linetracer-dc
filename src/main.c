#include <stdio.h>
#include "pico/stdlib.h"

#include "sensing.h"
#include "timer.h"

int i = 0;

void sensing_handler(void) {
  sensing_voltage();
  printf("%d - voltage: %f \r\n", i, voltage);
  i++;
  if (i == 16) {
    timer_periodic_stop(2);
  }
}

int main(void) {
  stdio_init_all();
  sensing_init();

  timer_periodic_start(2, 500000, sensing_handler);

  for (;;) {
    printf("Main loop \n", voltage);
    sleep_ms(1000);
  }

  return 0;
}