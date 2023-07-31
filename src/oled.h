#ifndef __PICO_SSD1331_SPI_H_
#define __PICO_SSD1331_SPI_H_

#include "pico/types.h"

/**
 * @brief OLED 화면을 모두 검은색으로 채우는 함수.
 */
void oled_clear(void);

/**
 * @brief SSD1331 OLED를 초기화하는 함수.
 */
void oled_init(void);

/*
 * 우리는 일상적으로 24비트 컬러를 사용한다. 각각 다음과 같이 나타낼 수 있다.
 *           0  1  2  3  4  5  6  7
 *   Red:   R0 R1 R2 R3 R4 R5 R6 R7
 *   Green: G0 G1 G2 G3 G4 G5 G6 G7
 *   Blue:  B0 B1 B2 B3 B4 B5 B6 B7
 *
 * 하지만 OLED의 GDDRAM은 16비트만 사용하고, 실질적으로 각각의 색의 상위 비트 몇 개만 반영된다.
 * 이 중 빨강은 R3~R7, 총 5비트만, 초록은 G2~G7, 총 6비트, 파랑은 B3~B7, 총 5비트를 사용한다.
 *
 *   Bit:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
 *   Map: B3 B4 B5 B6 B7 G2 G3 G4 G5 G6 G7 B3 B4 B5 B6 B7
 * 위의 비트맵 구조에 맞게 아래와 같은 색상 코드를 정의한다.
 */
enum oled_colors {
    OLED_COLOR_RED = 0xF800,
    OLED_COLOR_GREEN = 0x07E0,
    OLED_COLOR_BLUE = 0x001F,

    OLED_COLOR_RED_DARK = 0x7800,
    OLED_COLOR_GREEN_DARK = 0x03E0,
    OLED_COLOR_BLUE_DARK = 0x000F,

    OLED_COLOR_YELLOW = OLED_COLOR_RED | OLED_COLOR_GREEN,
    OLED_COLOR_CYAN = OLED_COLOR_GREEN | OLED_COLOR_BLUE,
    OLED_COLOR_MAGENTA = OLED_COLOR_RED | OLED_COLOR_BLUE,

    OLED_COLOR_ORANGE = OLED_COLOR_RED | OLED_COLOR_GREEN_DARK,
    OLED_COLOR_MINT = OLED_COLOR_GREEN | OLED_COLOR_BLUE_DARK,
    OLED_COLOR_ROSE = OLED_COLOR_RED | OLED_COLOR_BLUE_DARK,

    OLED_COLOR_LIME = OLED_COLOR_RED_DARK | OLED_COLOR_GREEN,
    OLED_COLOR_SEA = OLED_COLOR_GREEN_DARK | OLED_COLOR_BLUE,
    OLED_COLOR_VIOLET = OLED_COLOR_RED_DARK | OLED_COLOR_BLUE,

    OLED_COLOR_WHITE = OLED_COLOR_RED | OLED_COLOR_GREEN | OLED_COLOR_BLUE,
    OLED_COLOR_GRAY = OLED_COLOR_RED_DARK | OLED_COLOR_GREEN_DARK | OLED_COLOR_BLUE_DARK,
    OLED_COLOR_DARK = 0x0000,
};

typedef enum oled_colors oled_color_t;

/**
 * @brief 지정된 위치에 문자 하나를 출력하는 함수.
 *
 * @param c 출력할 문자
 * @param color 문자의 색상 (oled_colors 열거형 참고)
 * @param x 출력할 가로 위치
 * @param y 출력할 세로 위치
 * @return int 0 또는 -1
 *         0: 정상적으로 처리가 완료됨, -1: 문자의 출력 범위를 넘어 처리가 제대로 완료되지 않음
 */
int oled_putchar(char c, oled_color_t color, uint8_t x, uint8_t y);

/**
 * @brief 서식을 포함한 문자열을 OLED에 출력하는 함수.
 *
 * @param format 서식 문자열
 * @param ...
 */
void oled_printf(const char *format, ...);

#endif