#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

#include "sensing.h"
#include "timer.h"
#include "fs.h"

void sensing_init(void) {
    // ADC Block 초기화
    adc_init();

    // 전압 측정, IR 수광 센서 측정을 위해 해당 GPIO를 ADC 기능으로 초기화
    adc_gpio_init(SENSING_VOLTAGE_GPIO);
    adc_gpio_init(SENSING_IR_MUX_GPIO_IN_A);
    adc_gpio_init(SENSING_IR_MUX_GPIO_IN_B);

    // IR 센서부 GPIO 초기화
    gpio_init(SENSING_IR_MUX_GPIO_SEL0);
    gpio_set_dir(SENSING_IR_MUX_GPIO_SEL0, GPIO_OUT);
    gpio_init(SENSING_IR_MUX_GPIO_SEL1);
    gpio_set_dir(SENSING_IR_MUX_GPIO_SEL1, GPIO_OUT);
    gpio_init(SENSING_IR_MUX_GPIO_SEL2);
    gpio_set_dir(SENSING_IR_MUX_GPIO_SEL2, GPIO_OUT);
    gpio_init(SENSING_IR_MUX_GPIO_OUT);
    gpio_set_dir(SENSING_IR_MUX_GPIO_OUT, GPIO_OUT);

#if FLASH_LOAD_DEFAULT
    struct fs_data_t *fs = fs_get_data();
    sensing_ir_threshold = fs->sensing_ir_threshold;

    for (int i = 0; i < SENSING_IR_COUNT; i++) {
        sensing_ir_bias[i] = fs->sensing_ir_bias[i];
        sensing_ir_range[i] = fs->sensing_ir_range[i];
    }
#endif
}

/**
 * @brief Raspberry Pi Pico에 존재하는 총 3개의 ADC 채널에 대해 아날로그 디지털 변환을 실시한다.
 *
 * @param channel 0(GPIO 26), 1(GPIO 27), 2(GPIO 28) 중 하나의 값
 * @return 해당 채널에서 아날로그-디지털 변환한 데이터
 */
static uint sensing_get_adc_data(uint channel) {
    uint data[3];
    uint status;

    /*
     * 1. 앞으로 있을 인터럽트를 잠깐 중단시킨다.
     *    (ADC 도중에 다른 인터럽트로 인해 또 다른 ADC가 수행될 수 있어 잠깐 중단시킨다.)
     *    (참고로 RP2040의 ADC는 하나 뿐이고, 여기에 여러 channel이 물려있다.)
     * 2. ADC 채널을 선택한다.
     *    (매번 계속 해주는 이유는 ADC 변환이 끝나고 다른 인터럽트로 인해 채널이 변경될 수 있기 때문이다.)
     * 3. 아날로그 값을 읽는다.
     * 4. 인터럽트를 다시 복원한다.
     *
     * < 참고 사항 >
     * 위의 과정을 세 번 반복하는 이유
     *  -> Median Filter를 구현에 필요하다.
     * 반복문(for)을 사용하지 않는 이유
     *  -> 반복문 속에 있는 분기문 및 여러 연산자로 인해 생기는 클럭 소모를 없애기 위해서이다.
     */

    status = save_and_disable_interrupts();
    adc_select_input(channel);
    data[0] = adc_read();
    restore_interrupts(status);

    status = save_and_disable_interrupts();
    adc_select_input(channel);
    data[1] = adc_read();
    restore_interrupts(status);

    status = save_and_disable_interrupts();
    adc_select_input(channel);
    data[2] = adc_read();
    restore_interrupts(status);

    /*
     * 위에서 구한 총 3개의 ADC 결과를 오름차순으로 정렬한다.
     * 아래의 알고리즘은 선택 정렬의 일종이다.
     */

    uint temp;
    if (data[0] < data[1]) {
        temp = data[0];
        data[0] = data[1];
        data[1] = temp;
    }
    if (data[0] < data[2]) {
        temp = data[0];
        data[0] = data[2];
        data[2] = temp;
    }
    if (data[1] < data[2]) {
        temp = data[1];
        data[1] = data[2];
        data[2] = temp;
    }

    return data[1];
}

#define GET_ADC_CHANNEL(GPIO_PIN) ((GPIO_PIN) - (26))

volatile float sensing_supply_voltage;

/**
 * @brief 전압 센싱을 한 후 실제 전압으로 바꾸어 전역 변수에 저장(갱신)한다.
 */
static inline void sensing_update_voltage(void) {
    sensing_supply_voltage = SENSING_EXPR_RAW_TO_VOLTAGE(
        sensing_get_adc_data(GET_ADC_CHANNEL(SENSING_VOLTAGE_GPIO)));
}

volatile int sensing_ir_bias[SENSING_IR_COUNT];
volatile int sensing_ir_range[SENSING_IR_COUNT] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //
};
volatile int sensing_ir_raw[SENSING_IR_COUNT];
volatile int sensing_ir_normalized[SENSING_IR_COUNT];
volatile float sensing_ir_threshold = SENSING_IR_THRESHOLD_DEFAULT;
volatile sensing_ir_state_t sensing_ir_state;
volatile int sensing_ir_position;
volatile int sensing_ir_position_limited;

/**
 * @brief IR 센서가 받아들일 수 있는 최소의 값과 범위를 통해 IR 센서의 값을 0에서 255 사이의 값으로 정규화합니다.
 *
 * @param raw 원본 값
 * @param bias IR 센서의 편향(blackmax)
 * @param range IR 센서의 범위(whitemax - blackmax)
 * @return 정규화된 값
 */
static inline int sensing_normalize_ir(int raw, int bias, int range) {
    int norm = 0xff * (raw - bias) / range;
    norm = MIN(norm, 0xff);
    norm = MAX(norm, 0x00);

    return norm;
}

/**
 * @brief 현재 라인이 감지되는 위치를 의미하는 position 값을 계산한다.
 *        내부적으로 window 기능이 구현돼 position 계산에 필요한 센서만 가변적으로 이용할 수 있도록 했다.
 */
static inline void sensing_calc_position(void) {
    static const int weight[SENSING_IR_COUNT] = {
        -30000, -26000, -22000, -18000, -14000, -10000, -6000, -2000, //
        2000, 6000, 10000, 14000, 18000, 22000, 26000, 30000, //
    };
    const int position = sensing_ir_position;

    /**
     * 현재 position과 가중치를 통해 16조 어느 IR 센서 위에 트레이서가 위치하는지 구한다.
     * 가장 왼쪽의 position이 -30000이고, 각 센서 사이의 값 차이는 4000이므로 index = (position + 30000) / 4000
     *
     * 한편, 위의 식을 통해 window 기능을 구현하는데, window는 현재 위치한 IR 센서의 일부 주변 IR 센서만을 position 구하는데
     * 활용하여 주행 중 마크 표시로 인해 position이 튀는 것을 방지하는 기법을 말한다.
     */
    int window = (position + 30000) / 4000;
    static const int window_size_half = 2;
    int window_start = MAX(window - window_size_half + 1, 0);
    int window_end = MIN(window + window_size_half, SENSING_IR_COUNT - 1);

    int sum_weighted = 0;
    int sum = 0;
    /**
     * 중심을 0이라고 가정하고, 각 센서 값과 가중치를 곱하여 더한 값을 센서 값의 합으로 나누어 가중 평균을 구한다.
     * 결국, 센서 보드의 왼쪽에 라인이 위치하면 position 값은 음수가 되고 갈 수록 작아질 것이다.
     * 이에 반해 오른쪽에 라인이 위치하면 position 값은 양수가 되고 갈 수록 커질 것이다.
     */
    for (int i = window_start; i <= window_end; i++) {
        sum_weighted += sensing_ir_normalized[i] * weight[i];
        sum += sensing_ir_normalized[i];
    }

    if (sum == 0) {
        sensing_ir_position = 0;
    } else {
        sensing_ir_position = sum_weighted / sum;
    }
}

static float limiter_delta_in = 20.0f;
static float limiter_delta_out = 10.0f;

/**
 * @brief 곡선 감속 시, 진입할 때는 빠르게 감속하지만, 빠져나올 떄에 서서히 원래 속도로 돌아오게 만들기 위해 아래의 기능을 추가한다.
 */
static inline void sensing_calc_position_limited(void) {
    const int position = abs(sensing_ir_position);

    if (sensing_ir_position_limited < position) {
        // 곡선에 진입할 때, limiter_delta_in 만큼 증가시키면서 position에 대한 accel 구현
        sensing_ir_position_limited += limiter_delta_in;

        if (sensing_ir_position_limited > position) {
            sensing_ir_position_limited = position;
        }
    } else {
        // 곡선에서 나올 때, limiter_delta_out 만큼 감소시키면서 position에 대한 decel 구현
        sensing_ir_position_limited -= limiter_delta_out;

        if (sensing_ir_position_limited < position) {
            sensing_ir_position_limited = position;
        }
    }
}

/**
 * @brief IR 센서 데이터를 ADC 과정을 통해 구하고, 적절하게 가공한다.
 */
static inline void sensing_update_ir(void) {
    static const uint sel0 = (1 << SENSING_IR_MUX_GPIO_SEL0),
                      sel1 = (1 << SENSING_IR_MUX_GPIO_SEL1),
                      sel2 = (1 << SENSING_IR_MUX_GPIO_SEL2);

    static const uint ir_order[8] = {
        sel2 | sel1 | sel0, // 111
        sel2 | sel1 | 0x00, // 110
        sel2 | 0x00 | sel0, // 101
        sel2 | 0x00 | 0x00, // 100
        0x00 | sel1 | sel0, // 011
        0x00 | sel1 | 0x00, // 010
        0x00 | 0x00 | sel0, // 001
        0x00 | 0x00 | 0x00, // 000
    };
    static const uint ir_mask = sel0 | sel1 | sel2;

    static uint i = 0;

    // MUX를 이용하여 IR 발광 및 수광 센서 선택
    gpio_clr_mask(ir_mask);
    gpio_set_mask(ir_order[i]);

    gpio_put(SENSING_IR_MUX_GPIO_OUT, 1); // IR 발광센서 켜기
    busy_wait_us_32(8); // 발광 센서 안정화
    // 두 개의 MUX로부터 ADC 값을 가져옴
    int raw_l = sensing_get_adc_data(GET_ADC_CHANNEL(SENSING_IR_MUX_GPIO_IN_A)) >> 4;
    int raw_r = sensing_get_adc_data(GET_ADC_CHANNEL(SENSING_IR_MUX_GPIO_IN_B)) >> 4;
    gpio_put(SENSING_IR_MUX_GPIO_OUT, 0); // IR 발광센서 끄기

    // raw
    sensing_ir_raw[i] = raw_l;
    sensing_ir_raw[i + 8] = raw_r;

    // normalization
    sensing_ir_normalized[i] = sensing_normalize_ir(
        sensing_ir_raw[i], sensing_ir_bias[i], sensing_ir_range[i]);
    sensing_ir_normalized[i + 8] = sensing_normalize_ir(
        sensing_ir_raw[i + 8], sensing_ir_bias[i + 8], sensing_ir_range[i + 8]);

    // state
    sensing_ir_state &= ~((1 << (0x7 - i)) | (1 << (0xf - i)));
    sensing_ir_state |= (sensing_ir_normalized[i] > (sensing_ir_threshold * 0xff)) << (0xf - i);
    sensing_ir_state |= (sensing_ir_normalized[i + 8] > (sensing_ir_threshold * 0xff)) << (0x7 - i);

    // position
    sensing_calc_position();

    // limited position
    sensing_calc_position_limited();

    i = (i + 1) & 0x07;
}

/**
 * @brief `sensing.c` 전용 타이머 IRQ 핸들러
 */
static void sensing_handler(void) {
    sensing_update_voltage();
    sensing_update_ir();
}

void sensing_start(void) {
    timer_periodic_start(SENSING_TIMER_SLOT, SENSING_TIMER_INTERVAL_US, sensing_handler);
}

void sensing_stop(void) {
    timer_periodic_stop(SENSING_TIMER_SLOT);
}
