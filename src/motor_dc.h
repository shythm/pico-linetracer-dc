#ifndef __MOTOR_DC_H_
#define __MOTOR_DC_H

#define MOTOR_DC_PWM_LEFT_GPIO         14
#define MOTOR_DC_PWM_RIGHT_GPIO        15
#define MOTOR_DC_DIRECTION_LEFT_GPIO   16
#define MOTOR_DC_DIRECTION_RIGHT_GPIO  17

#define MOTOR_DC_LEFT   0
#define MOTOR_DC_RIGHT  1

#include "pico/types.h"

void motor_dc_init(void);
void motor_dc_set_enabled(uint motor, bool enabled);
void motor_dc_input_voltage(uint motor, float input_voltage, float supply_voltage);

#endif