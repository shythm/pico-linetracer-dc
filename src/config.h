#ifndef _CONFIG_H_
#define _CONFIG_H_

/* COMMON BEGIN */

#define PI (3.141592f)
/* COMMON END */

/* DC MOTOR BEGIN */

#define MOTOR_PWM_SLICE_NUM  (7)
#define MOTOR_PWM_GPIO_LEFT  (14)
#define MOTOR_PWM_GPIO_RIGHT (15)
#define MOTOR_DIR_GPIO_LEFT  (16)
#define MOTOR_DIR_GPIO_RIGHT (17)

#define MOTOR_ENCODER_PIO             (pio0)
#define MOTOR_ENCODER_GPIO_LEFT_BASE  (10)
#define MOTOR_ENCODER_GPIO_RIGHT_BASE (12)
#define MOTOR_ENCODER_COMP_LEFT       (-1)
#define MOTOR_ENCODER_COMP_RIGHT      (-1)
#define ENCODER_RESOLUTION            (2048)
#define WHEEL_DIAMETER_M              (0.038f) // 바퀴의 지름(m)
#define GEAR_RATIO                    (17.f / 69.f) // 모터(17) / 바퀴(69) 기어비

#define CONTROL_TIMER_SLOT  (TIMER_SLOT_1)
#define CONTROL_INTERVAL_US (500)
#define CONTROL_GAIN_P      (0.04f)
#define CONTROL_GAIN_D      (0.02f)

/* DC MOTOR END */

#endif