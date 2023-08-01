#ifndef _SENSING_H_
#define _SENSING_H_

#include "pico/types.h"
#include "config.h"

/**
 * @brief [READ ONLY] 현재 메인보드에 인가되는 전압
 */
extern volatile float sensing_supply_voltage;

/**
 * @brief IR 센서 값의 하한선 (blackmax)
 */
extern volatile int sensing_ir_bias[SENSING_IR_COUNT];

/**
 * @brief IR 센서 값의 범위 (whitemax - blackmax)
 */
extern volatile int sensing_ir_range[SENSING_IR_COUNT];

/**
 * @brief [READ ONLY] ADC 과정을 통해 얻은 IR 센서 값의 배열
 */
extern volatile int sensing_ir_raw[SENSING_IR_COUNT];

/**
 * @brief [READ ONLY] 정규화된 IR 센서 값의 배열
 */
extern volatile int sensing_ir_normalized[SENSING_IR_COUNT];

/**
 * @brief IR 센서 state를 판단할 역치
 */
extern volatile float sensing_ir_threshold;

/**
 * @brief IR 센서 state를 저장할 타입.
 * 16조 센서의 경우에 총 16개의 비트가 필요하므로 16비트 자료형(uint16_t)을 사용한다.
 */
typedef uint16_t sensing_ir_state_t;

/**
 * @brief [READ ONLY] IR 센서들 중 어떤 센서가 흰색 부분을 감지하는지를 나타내는 상태.
 * 총 n개의 센서에 대해, k(1 <= k <= n)번째 센서 상태가 (n - k)번째 비트에 저장되어 있음.
 */
extern volatile sensing_ir_state_t sensing_ir_state;

/**
 * @brief [READ ONLY] 현재 라인트레이서의 센서 보드가 라인 어디에 위치해있는지에 대한 지표.
 * 만약 센서 보드가 라인의 중심에 있다면 값은 0이 될 것이고, 센서 보드의 왼쪽에 라인이 위치하면 값은 음수가 되고 갈 수록 작아질 것이다.
 * 이에 반해 오른쪽에 라인이 위치하면 값은 양수가 되고 갈수록 커질 것이다.
 */
extern volatile int sensing_ir_position;

/**
 * @brief 전압 및 IR 센서의 정보를 ADC로부터 가져오고 가공할 준비를 하는 초기화 함수
 */
void sensing_init(void);

/**
 * @brief 전압 및 IR 센서 측정 시작
 */
void sensing_start(void);

/**
 * @brief 전압 및 IR 센서 측정 중지
 */
void sensing_stop(void);

#endif