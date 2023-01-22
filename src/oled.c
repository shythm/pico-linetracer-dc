/**
 * @file oled.c
 * @author Seongho Lee (shythm@outlook.com)
 * @brief oled driver(ssd1331 module) and utilities for pico linetracer
 */
#include "oled.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"

enum oled_command {
    OLED_CMD_SETCOLUMN      = 0x15,    // Set column address
    OLED_CMD_DRAWLINE       = 0x21,    // Draw line
    OLED_CMD_DRAWRECT       = 0x22,    // Draw rectangle
    OLED_CMD_CLEAR          = 0x25,    // Clear window
    OLED_CMD_FILL           = 0x26,    // Fill enable/disable
    OLED_CMD_SETROW         = 0x75,    // Set row adress
    OLED_CMD_CONTRASTA      = 0x81,    // Set contrast for color A
    OLED_CMD_CONTRASTB      = 0x82,    // Set contrast for color B
    OLED_CMD_CONTRASTC      = 0x83,    // Set contrast for color C
    OLED_CMD_MASTERCURRENT  = 0x87,    // Master current control
    OLED_CMD_PRECHARGEA     = 0x8A,    // Set second pre-charge speed for color A
    OLED_CMD_PRECHARGEB     = 0x8B,    // Set second pre-charge speed for color B
    OLED_CMD_PRECHARGEC     = 0x8C,    // Set second pre-charge speed for color C
    OLED_CMD_SETREMAP       = 0xA0,    // Set re-map & data format
    OLED_CMD_STARTLINE      = 0xA1,    // Set display start line
    OLED_CMD_DISPLAYOFFSET  = 0xA2,    // Set display offset(Set vertical offset by Com)
    OLED_CMD_NORMALDISPLAY  = 0xA4,    // Set display to normal mode
    OLED_CMD_DISPLAYALLON   = 0xA5,    // Set entire display ON
    OLED_CMD_DISPLAYALLOFF  = 0xA6,    // Set entire display OFF
    OLED_CMD_INVERTDISPLAY  = 0xA7,    // Invert display
    OLED_CMD_SETMULTIPLEX   = 0xA8,    // Set multiplex ratio
    OLED_CMD_SETMASTER      = 0xAD,    // Set master configuration
    OLED_CMD_DISPLAYOFF     = 0xAE,    // Display OFF (sleep mode)
    OLED_CMD_DISPLAYON      = 0xAF,    // Normal Brightness Display ON
    OLED_CMD_POWERMODE      = 0xB0,    // Power save mode
    OLED_CMD_PRECHARGE      = 0xB1,    // Phase 1 and 2 period adjustment
    OLED_CMD_CLOCKDIV       = 0xB3,    // Set display clock divide ratio/oscillator frequency
    OLED_CMD_PRECHARGELEVEL = 0xBB,    // Set pre-charge voltage
    OLED_CMD_VCOMH          = 0xBE,    // Set Vcomh voltge
};

static inline void oled_write_command(const uint8_t *command, size_t length) {
    gpio_put(OLED_CS_GPIO, 0);
    gpio_put(OLED_DC_GPIO, 0);  // 0 is command mode

    spi_write_blocking(OLED_SPI_INSTANCE, command, length);

    gpio_put(OLED_CS_GPIO, 1);
}

static inline void oled_write_data(const uint8_t *data, size_t length) {
    gpio_put(OLED_CS_GPIO, 0);
    gpio_put(OLED_DC_GPIO, 1);  // 1 is data mode

    spi_write_blocking(OLED_SPI_INSTANCE, data, length);

    gpio_put(OLED_CS_GPIO, 1);
}

void oled_init(void) {
    // SPI 기능을 초기화한다.
    spi_init(OLED_SPI_INSTANCE, OLED_SPI_BUADRATE);
    gpio_set_function(OLED_SCL_GPIO, GPIO_FUNC_SPI); // SPI 클럭 신호
    gpio_set_function(OLED_SDA_GPIO, GPIO_FUNC_SPI); // SPI TX 신호 (RX 신호는 따로 사용하지 않는다.)

    // D/C GPIO를 초기화한다. High 신호일 때 Data 모드로 동작하고, Low 신호일 때 Command 모드로 동작한다.
    gpio_init(OLED_DC_GPIO);
    gpio_set_dir(OLED_DC_GPIO, GPIO_OUT);

    // CS GPIO를 초기화한다. CS는 Chip Select의 약자로 SPI에서 슬레이브를 선택할 때 사용하는 핀이다.
    // Low 신호일 때 SPI 신호가 수용되며, High 신호일 때는 무시된다.
    gpio_init(OLED_CS_GPIO);
    gpio_set_dir(OLED_CS_GPIO, GPIO_OUT);
    gpio_put(OLED_CS_GPIO, 1);

    // 우리가 사용하는 SSD1331 OLED 모듈의 초기화 명령어들이다.
    // 아래의 사이트에서 해당 명령어를 참조했으며, SSD1331 데이터시트 8장과 9절을 참조하면 더욱 자세한 내용을 알 수 있다.
    // https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
    const uint8_t init_cmd[] = {
        OLED_CMD_DISPLAYOFF,
        OLED_CMD_SETREMAP, 0x72,
        OLED_CMD_STARTLINE, 0x00,
        OLED_CMD_DISPLAYOFFSET, 0x00,
        OLED_CMD_NORMALDISPLAY,
        OLED_CMD_SETMULTIPLEX, 0x3F,
        OLED_CMD_SETMASTER, 0x8E,
        OLED_CMD_POWERMODE, 0x0B,
        OLED_CMD_PRECHARGE, 0x31,
        OLED_CMD_CLOCKDIV, 0xF0,
        OLED_CMD_PRECHARGEA, 0x64,
        OLED_CMD_PRECHARGEB, 0x78,
        OLED_CMD_PRECHARGEC, 0x64,
        OLED_CMD_PRECHARGELEVEL, 0x3A,
        OLED_CMD_VCOMH, 0x3E,
        OLED_CMD_MASTERCURRENT, 0x06,
        OLED_CMD_CONTRASTA, 0x91,
        OLED_CMD_CONTRASTB, 0x50,
        OLED_CMD_CONTRASTC, 0x7D,
        OLED_CMD_DISPLAYON
    };

    oled_write_command(init_cmd, sizeof(init_cmd));
}

void oled_set_enabled(bool enabled) {
    const uint8_t cmd = enabled ? OLED_CMD_DISPLAYON : OLED_CMD_DISPLAYOFF;
    oled_write_command(&cmd, 1);
}

void oled_clear_all(void) {
    const uint8_t cmd[] = {
        OLED_CMD_CLEAR,
        0x00,   // Column Address of Start
        0x00,   // Row Address of Start
        0x5F,   // Column Address of End (Col 95)
        0x40,   // Row Address of End (Row 63)
    };

    oled_write_command(cmd, sizeof(cmd));
}