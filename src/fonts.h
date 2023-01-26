#ifndef __FONTS_H_
#define __FONTS_H_

#include "pico/types.h"

#define FONT_START  32
#define FONT_LENGTH 96
#define FONT_WIDTH  8
#define FONT_HEIGHT 8

extern const uint8_t fonts[FONT_LENGTH][FONT_WIDTH];

#endif