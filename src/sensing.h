#ifndef __SENSING_H_
#define __SENSING_H_

#include "pico/types.h"

#define SENSING_VOLTAGE_GPIO 26

#define SENSING_IR_MUX_SEL0_GPIO 6
#define SENSING_IR_MUX_SEL1_GPIO 7
#define SENSING_IR_MUX_SEL2_GPIO 8
#define SENSING_IR_IN_MUXA_GPIO  27
#define SENSING_IR_IN_MUXB_GPIO  28
#define SENSING_IR_OUT_MUX_GPIO  9

// ADC로부터 얻은 데이터를 실제 전압으로 바꿔주는 계산식
#define SENSING_EXPR_RAW_TO_VOLTAGE(X) ((3.3f / 4096.0f * 21.0f / 1.0f) * (X))

void sensing_init(void);

extern volatile float voltage; // 감지된 배터리 전압
void sensing_voltage(void);

extern volatile int sensor_raw[16];
extern volatile int sensor_coef_bias[16]; // 블랙 맥스(=편향값)
extern volatile int sensor_coef_range[16]; // 센서값 범위(화이트 맥스 - 블랙 맥스)
extern volatile int sensor_normalized[16];
extern volatile uint16_t sensor_state; // 센서가 흰색을 보면 1 아니면 0임을 저장한다.
extern volatile float sensor_threshold; // 노멀라이즈된 센서값을 스테이트로 바꾸기 위한 임계값.(0~ff)
extern volatile int sensor_position; // 라인이 감지되는 포지션
extern volatile int zero_point; // 3차를 위한 영점. 0보다 크면 센서 중앙을 기준으로 오른쪽으로 position의 영점이 변환된다.
void sensing_infrared(void);

#endif