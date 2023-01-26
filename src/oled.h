#ifndef __PICO_SSD1331_SPI_H_
#define __PICO_SSD1331_SPI_H_

#include "pico/types.h"
#include "hardware/spi.h"

#define OLED_SPI_INSTANCE   spi0
#define OLED_SPI_BUADRATE   8 * 1000 * 1000

#define OLED_SCL_GPIO       2
#define OLED_SDA_GPIO       3
#define OLED_DC_GPIO        4
#define OLED_CS_GPIO        5

#define OLED_PRINT_BUFFER   256

void oled_init(void);
void oled_set_enabled(bool enabled);
void oled_clear_all(void);
void oled_printf(const char *format, ...);

#endif