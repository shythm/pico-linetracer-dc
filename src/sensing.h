#ifndef __SENSING_H_
#define __SENSING_H_

#define SENSING_VOLTAGE_GPIO  26

#define SENSING_IR_MUX_SEL0_GPIO  6
#define SENSING_IR_MUX_SEL1_GPIO  7
#define SENSING_IR_MUX_SEL2_GPIO  8
#define SENSING_IR_IN_MUXA_GPIO   27
#define SENSING_IR_IN_MUXB_GPIO   28
#define SENSING_IR_OUT_MUX_GPIO   9

// ADC로부터 얻은 데이터를 실제 전압으로 바꿔주는 계산식
#define SENSING_EXPR_RAW_TO_VOLTAGE(X) ((3.3f / 4096.0f * 21.0f / 1.0f) * (X))

extern volatile float voltage;

void sensing_init(void);
void sensing_voltage(void);

#endif