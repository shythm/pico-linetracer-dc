#include <stdio.h>

#include "pico/stdlib.h"

#include "sensing.h"

int main(void) {
    stdio_init_all();

    sensing_init();

    for (;;) {
        printf("voltage: %f \r\n", sensing_voltage());
        sleep_ms(1000);
    }

    return 0;
}