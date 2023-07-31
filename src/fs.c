#include "fs.h"

#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"

/**
 * @brief flash의 size를 정의한다.
 * RP2040의 2MB on-board flash memory 중 절반인 1MB를 flash 영역으로 사용한다.
 * flash에 쓸 수 있는 최소 크기인 FLASH_PAGE_SIZE를 토대로 flash 크기를 정의한다.
 */
#define FLASH_DATA_SIZE ((size_t)((FLASH_PAGE_SIZE) * (4)))

/**
 * @brief flash의 offset을 정의한다.
 * code 영역을 침범하면 안되기 때문에, code 영역보다 offset이 커야 한다.
 * RP2040에는 2MB on-board flash memory를 가지고 있으며, 절반을 나누어 그 이상을 offset으로 한다.
 */
#define FLASH_DATA_OFFSET ((1024) * (1024))

union flash_memory {
    struct flash_data_t data;
    uint8_t memory[FLASH_DATA_SIZE];
} *const buffer = (union flash_memory *)(XIP_BASE + FLASH_DATA_OFFSET);

void fs_put_data(struct flash_data_t *data) {
    // flash에 값을 쓰는 과정을 안정적으로 수행하기 위해 잠시 인터럽트를 중지한다.
    uint status = save_and_disable_interrupts();

    // flash 메모리에 쓰기 앞서 그 공간을 지운다.
    flash_range_erase(FLASH_DATA_OFFSET, FLASH_DATA_SIZE);

    // flash 메모리에 내용을 쓴다.
    memcpy(buffer->memory, data, sizeof(struct flash_data_t));
    flash_range_program(FLASH_DATA_OFFSET, buffer->memory, FLASH_DATA_SIZE);

    // 다시 인터럽트를 활성화한다.
    restore_interrupts(status);
}

struct flash_data_t *fs_get_data(void) {
    return &(buffer->data);
}
