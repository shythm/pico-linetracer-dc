#ifndef _CONFIG_H_
#define _CONFIG_H_

/* COMMON BEGIN */

#define PI (3.141592f)
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/* COMMON END */

/* SWITCH BEGIN */

#define SWITCH_GPIO_LEFT  (0)
#define SWITCH_GPIO_RIGHT (1)

/** @brief Bouncing 방지를 위해 클릭 후 잠시 대기하는 시간(us) */
#define SWITCH_TIME_SHORT ((50) * (1000))
/** @brief 스위치 클릭 이벤트 발생 주기(us) */
#define SWITCH_TIME_LONG ((200) * (1000))

/* SWITCH END */

/* OLED BEGIN */

#define OLED_SPI          (spi0)
#define OLED_SPI_BUADRATE ((8) * (1000) * (1000))
#define OLED_SPI_GPIO_SCL (2)
#define OLED_SPI_GPIO_SDA (3)

#define OLED_GPIO_DC (4)
#define OLED_GPIO_CS (5)

#define OLED_PRINT_BUFFER (256)

/* OLED END */

/* SENSING BEGIN */

#define SENSING_IR_COUNT             (16)
#define SENSING_IR_MUX_GPIO_SEL0     (6)
#define SENSING_IR_MUX_GPIO_SEL1     (7)
#define SENSING_IR_MUX_GPIO_SEL2     (8)
#define SENSING_IR_MUX_GPIO_OUT      (9)
#define SENSING_IR_THRESHOLD_DEFAULT (0.2f)

#define SENSING_IR_MUX_GPIO_IN_A (27)
#define SENSING_IR_MUX_GPIO_IN_B (28)

/**
 * @brief ADC로부터 얻은 데이터를 실제 전압으로 바꿔주는 계산식
 */
#define SENSING_EXPR_RAW_TO_VOLTAGE(X) (((3.3f) / (4096.0f) * (21.0f) / (1.0f)) * (X))
#define SENSING_VOLTAGE_GPIO           (26)

#define SENSING_TIMER_SLOT        (TIMER_SLOT_0)
#define SENSING_TIMER_INTERVAL_US (500)

/* SENSING END */

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
#define MOTOR_ENCODER_RESOLUTION      (2048)
#define MOTOR_WHEEL_DIAMETER_M        (0.038f) // 바퀴의 지름(m)
#define MOTOR_GEAR_RATIO              (17.f / 69.f) // 모터(17) / 바퀴(69) 기어비

#define MOTOR_CONTROL_TIMER_SLOT  (TIMER_SLOT_1)
#define MOTOR_CONTROL_INTERVAL_US (500)
#define MOTOR_CONTROL_GAIN_P      (0.2f)
#define MOTOR_CONTROL_GAIN_D      (0.1f)

/**
 * @brief 바퀴 지름 1미터 당 엔코더 몇 틱인지에 대한 상수
 */
#define MOTOR_TICK_PER_METER \
    ((float)(MOTOR_ENCODER_RESOLUTION) / (MOTOR_WHEEL_DIAMETER_M * PI) / (MOTOR_GEAR_RATIO))

/* DC MOTOR END */

/* MARK BEGIN */

#define MARK_LENGTH_TICK ((MOTOR_TICK_PER_METER) * (0.02))

/* MARK END */

/* DRIVE BEGIN */

#define DRIVE_MARK_COUNT_MAX   (400)
#define DRIVE_LINE_OUT_TIME_US ((200) * (1000))

// #define DRIVE_ENABLE_MARK_RECOVER

/* DRIVE END */

/* FLASH BEGIN */

#define FLASH_LOAD_DEFAULT (true)

/* FLASH END */

#endif