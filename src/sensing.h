#ifndef __SENSING_H_
#define __SENSING_H_

#include "pico/types.h"

/**
 * @brief 전압 및 IR 센서의 정보를 ADC로부터 가져오고 가공할 준비를 하는 초기화 함수
 */
void sensing_init(void);

extern volatile float _sensing_voltage;

/**
 * @brief 현재 메인보드에 인가되는 전압을 가져옵니다.
 * @return 전압(V)
 */
static inline float sensing_get_supply_voltage(void) {
    return _sensing_voltage;
}

#define SENSING_IR_COUNT 16

extern volatile int _sensing_ir_raw[SENSING_IR_COUNT];

/**
 * @brief ADC 과정을 통해 얻은 IR 센서의 원본 값을 가져옵니다.
 * @param index IR 센서 인덱스
 * @return IR 센서 원본 값
 */
static inline int sensing_get_ir_raw(int index) {
    return _sensing_ir_raw[index];
}

extern volatile int sensing_ir_bias[SENSING_IR_COUNT]; // IR 센서의 최솟값 (blackmax)
extern volatile int sensing_ir_range[SENSING_IR_COUNT]; // IR 센서가 가질 수 있는 범위 (whitemax - blackmax)
extern volatile int _sensing_ir_normalized[SENSING_IR_COUNT]; // 정규화된 IR 센서의 값

/**
 * @brief 정규화된 IR 센서의 값을 가져옵니다.
 *        이때, `sensing_ir_bias`와 `sensing_ir_range`를 조절하는 캘리브레이션이 선행되지 않으면 원본 그대로의 값이 출력됩니다.
 * @param index IR 센서 인덱스
 * @return 정규화된 IR 센서 값
 */
static inline int sensing_get_ir_normalized(int index) {
    return _sensing_ir_normalized[index];
}

/**
 * @brief 센서 state를 저장할 타입. 16조 센서의 경우에 총 16개의 비트가 필요하므로 16비트 자료형(uint16_t)을 사용한다.
 */
typedef uint16_t sensing_ir_state_t;

extern volatile float sensing_ir_threshold; // 정규화된 센서 값을 State로 바꾸기 위한 임계값
extern volatile sensing_ir_state_t _sensing_ir_state;

/**
 * @brief IR 센서들 중 어떤 센서가 흰색 부분을 감지하는지를 나타내는 상태를 가져옵니다.
 * @return 총 n개의 센서에 대해, k(1 <= k <= n)번째 센서 상태가 (n - k)번째 비트에 저장된 변수
 */
static inline sensing_ir_state_t sensing_get_ir_state(void) {
    return _sensing_ir_state;
}

extern volatile int _sensing_position;

/**
 * @brief 현재 라인트레이서의 센서 보드가 라인 어디에 위치해있는지에 대한 지표를 가져옵니다.
 *        만약 센서 보드가 라인의 중심에 있다면 값은 0이 될 것이고, 센서 보드의 왼쪽에 라인이 위치하면 값은 음수가 되고 갈 수록 작아질 것이다.
 *        이에 반해 오른쪽에 라인이 위치하면 값은 양수가 되고 갈 수록 커질 것이다.
 * @return 현재 라인의 위치
 */
static inline int sensing_get_position(void) {
    return _sensing_position;
}

/**
 * @brief 전압 및 IR 센서의 정보를 계속해서 최신화할 지 결정합니다.
 * @param enabled 1(감지 시작) 또는 0(감지 종료)
 */
void sensing_set_enabled(bool enabled);

#endif