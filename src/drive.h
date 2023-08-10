#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "config.h"

enum drive_t {
    DRIVE_FIRST,
    DRIVE_SECOND,
};

void drive(const enum drive_t type);

static inline void drive_first() {
    drive(DRIVE_FIRST);
}

static inline void drive_second() {
    drive(DRIVE_SECOND);
}

#endif