#include "drive.h"

#include "pico.h"
#include "motor_dc.h"
#include "timer.h"
#include "oled.h"
#include "sensing.h"
#include "switch.h"

#define DRIVE_TIMER_SLOT        TIMER_SLOT_2
#define DRIVE_TIMER_INTERVAL_US 500

volatile float accel = 6.f; // 가속도(m/s^2)
volatile float k = 0.0001f;

volatile float velocity_command[MOTOR_DC_COUNT];
volatile float velocity_target = 0.0f;

void drive_control_velocity(enum motor_dc_index index, float velocity) {
    static const float dt_s = (DRIVE_TIMER_INTERVAL_US * 0.001) * 0.001;
    volatile float *command = &velocity_command[index];

    if (*command < velocity) {
        *command += accel * dt_s;
        if (*command > velocity)
            *command = velocity;
    } else if (*command > velocity) {
        *command -= accel * dt_s;
        if (*command < velocity)
            *command = velocity;
    }

    motor_dc_set_velocity(index, *command);
}

/**
 * @brief 지정된 가속도와 라인의 포지션에 따라 양쪽 모터의 속도를 조절한다.
 * @brief  인터럽트를 사용해 주기적으로 작동해야한다.
 */
void drive_control_velocity_handler(void) {
    drive_control_velocity(MOTOR_DC_LEFT, velocity_target * (1.f + k * sensor_position));
    drive_control_velocity(MOTOR_DC_RIGHT, velocity_target * (1.f - k * sensor_position));
}

void drive_control_velocity_enabled(bool enabled) {
    if (enabled) {
        velocity_command[MOTOR_DC_LEFT] = 0.0f;
        velocity_command[MOTOR_DC_RIGHT] = 0.0f;
        motor_dc_control_enabled(true);
        timer_periodic_start(DRIVE_TIMER_SLOT, DRIVE_TIMER_INTERVAL_US, drive_control_velocity_handler);
    } else {
        velocity_target = 0.0f;
        while ((velocity_command[MOTOR_DC_LEFT] + velocity_command[MOTOR_DC_RIGHT]) / 2 > 0.1f) {
            tight_loop_contents();
        }
        timer_periodic_stop(DRIVE_TIMER_SLOT);
        motor_dc_control_enabled(false);
    }
}

void drive_first(void) {
    oled_printf("/0  RUN!!!!  ");

    velocity_target = 2.0f;
    for (;;) {
        oled_printf("/1   %5d   ", sensor_position);
        oled_printf("/2 velo : %1.2f  ", velocity_target);

        uint sw = switch_read_wait_ms(100);
        if (sw == SWITCH_EVENT_BOTH)
            break;
        else if (sw == SWITCH_EVENT_LEFT)
            velocity_target -= 0.1;
        else if (sw == SWITCH_EVENT_RIGHT)
            velocity_target += 0.1;
    }

    drive_control_velocity_enabled(true);

    for (;;) {
        uint sw = switch_read_wait_ms(100);

        if (sw == SWITCH_EVENT_BOTH)
            break;
    }

    drive_control_velocity_enabled(false);
}