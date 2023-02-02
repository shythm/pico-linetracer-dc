#include "sensing.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/pio.h"

#include "quadrature_encoder.pio.h"

const PIO encoder_pio[2] = { pio0, pio1 };
uint encoder_pio_offset[2];
uint encoder_pio_sm[2];
uint encoder_pio_pin[2] = {
    SENSING_ENCODER_LAB_GPIO,
    SENSING_ENCODER_RAB_GPIO
};

void sensing_init(void) {
    // ADC Block 초기화
    adc_init();

    // 전압 측정, IR 수광 센서 측정을 위해 해당 GPIO를 ADC 기능으로 초기화
    adc_gpio_init(SENSING_VOLTAGE_GPIO);
    adc_gpio_init(SENSING_IR_IN_MUXA_GPIO);
    adc_gpio_init(SENSING_IR_IN_MUXB_GPIO);

    // IR 센서부 GPIO 초기화
    gpio_init(SENSING_IR_MUX_SEL0_GPIO);
    gpio_set_dir(SENSING_IR_MUX_SEL0_GPIO, GPIO_OUT);
    gpio_init(SENSING_IR_MUX_SEL1_GPIO);
    gpio_set_dir(SENSING_IR_MUX_SEL1_GPIO, GPIO_OUT);
    gpio_init(SENSING_IR_MUX_SEL2_GPIO);
    gpio_set_dir(SENSING_IR_MUX_SEL2_GPIO, GPIO_OUT);
    gpio_init(SENSING_IR_OUT_MUX_GPIO);
    gpio_set_dir(SENSING_IR_OUT_MUX_GPIO, GPIO_OUT);

    // 엔코더 초기화
    for (uint i = 0; i < 2; i++) {
        encoder_pio_offset[i] = pio_add_program(
            encoder_pio[i],
            &quadrature_encoder_program);
        quadrature_encoder_program_init(
            encoder_pio[i],
            encoder_pio_sm[i],
            encoder_pio_offset[i],
            encoder_pio_pin[i],
            0);
    }
}

#define GET_ADC_CHANNEL(GPIO_PIN) ((GPIO_PIN) - (26))

static uint get_adc_data(uint channel) {
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

volatile float voltage;

void sensing_voltage(void) {
    voltage = SENSING_EXPR_RAW_TO_VOLTAGE(
        get_adc_data(GET_ADC_CHANNEL(SENSING_VOLTAGE_GPIO)));
}

#define IMS0 (1 << SENSING_IR_MUX_SEL0_GPIO)
#define IMS1 (1 << SENSING_IR_MUX_SEL1_GPIO)
#define IMS2 (1 << SENSING_IR_MUX_SEL2_GPIO)

static const uint sensor_order[8] = {
    0x00 | 0x00 | 0x00, // 000
    0x00 | 0x00 | IMS0, // 001
    0x00 | IMS1 | 0x00, // 010
    0x00 | IMS1 | IMS0, // 011
    IMS2 | 0x00 | 0x00, // 100
    IMS2 | 0x00 | IMS0, // 101
    IMS2 | IMS1 | 0x00, // 110
    IMS2 | IMS1 | IMS0, // 111
};
static const uint sensor_mask = IMS0 | IMS1 | IMS2;

volatile uint sensor_raw[16]; // ADC로부터 구한 IR 센서의 값
volatile uint sensor_coef_bias[16]; // IR 센서의 최솟값
volatile uint sensor_coef_range[16] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //
}; // IR 센서가 가질 수 있는 범위
volatile uint sensor_normalized[16]; // 정규화된 IR 센서의 값, 이 값들을 주로 이용

void sensing_infrared(void) {
    static uint i = 0;
    uint raw_l, raw_r;
    uint raw_norm_l, raw_norm_r;

    // MUX를 이용하여 IR 발광 및 수광 센서 선택
    gpio_clr_mask(sensor_mask);
    gpio_set_mask(sensor_order[i]);

    gpio_put(SENSING_IR_OUT_MUX_GPIO, 1); // IR 발광센서 켜기
    // 두 개의 MUX로부터 ADC 값을 가져옴
    raw_l = get_adc_data(GET_ADC_CHANNEL(SENSING_IR_IN_MUXA_GPIO)) << 4;
    raw_r = get_adc_data(GET_ADC_CHANNEL(SENSING_IR_IN_MUXB_GPIO)) << 4;
    gpio_put(SENSING_IR_OUT_MUX_GPIO, 0); // IR 발광센서 끄기

    // 센서 값 저장
    sensor_raw[i] = raw_l;
    sensor_raw[i + 8] = raw_r;

    // 센서 정규화 수행 및 저장
    sensor_normalized[i] = 0xff * (raw_l - sensor_coef_bias[i]) / sensor_coef_range[i];
    sensor_normalized[i + 8] = 0xff * (raw_r - sensor_coef_bias[i + 8]) / sensor_coef_range[i + 8];

    i = (i + 1) & 0x07;
}

inline uint get_encoder_count(uint num) {
    return quadrature_encoder_get_count(
        encoder_pio[num],
        encoder_pio_sm[num]);
}