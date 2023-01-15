#include "sensing.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/sync.h"

#define GET_ADC_CHANNEL(GPIO_PIN) ((GPIO_PIN) - (26))

void sensing_init(void) {
  // ADC Block 초기화
  adc_init();

  // 전압 측정, IR 수광 센서 측정을 위해 해당 GPIO를 ADC 기능으로 초기화
  adc_gpio_init(SENSING_VOLTAGE_GPIO);
  adc_gpio_init(SENSING_IR_IN_MUXA_GPIO);
  adc_gpio_init(SENSING_IR_IN_MUXB_GPIO);

  // IR 센서부 GPIO 초기화
  gpio_init(SENSING_IR_MUX_SEL0_GPIO);
  gpio_init(SENSING_IR_MUX_SEL1_GPIO);
  gpio_init(SENSING_IR_MUX_SEL2_GPIO);
  gpio_init(SENSING_IR_OUT_MUX_GPIO);
  gpio_set_dir_out_masked(
    1 << SENSING_IR_MUX_SEL0_GPIO |
    1 << SENSING_IR_MUX_SEL1_GPIO |
    1 << SENSING_IR_MUX_SEL2_GPIO |
    1 << SENSING_IR_OUT_MUX_GPIO
  );
}

static uint16_t get_adc_data(uint channel) {  
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

inline float sensing_voltage(void) {
  return SENSING_EXPR_RAW_TO_VOLTAGE(
    get_adc_data(GET_ADC_CHANNEL(SENSING_VOLTAGE_GPIO))
  );
}

inline uint8_t sensing_ir_receiver(uint num) {

}