#include "fs.h"

#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"

/**
 * @brief flash의 size를 4KB로 설정한다.
 * flash에 쓸 수 있는 최소 크기인 FLASH_PAGE_SIZE(256 bytes)와 맞아 떨어지며,
 * flash를 지울 수 있는 최소 크기인 FLASH_SECTOR_SIZE(4096 bytes)와 맞아 떨어진다.
 */
#define FLASH_DATA_SIZE ((4) * (1024))

/**
 * @brief flash의 offset을 정의한다.
 * code 영역을 침범하면 안되기 때문에, code 영역보다 offset이 커야 한다.
 * RP2040에는 2MB on-board flash memory를 가지고 있으며, 절반을 나누어 그 이상을 offset으로 한다.
 */
#define FLASH_DATA_OFFSET ((1024) * (1024))

union flash_memory {
    uint8_t memory[FLASH_DATA_SIZE];
    struct fs_data_t data;
};

static union flash_memory *flash_target = (union flash_memory *)(XIP_BASE + FLASH_DATA_OFFSET);
static union flash_memory buffer;

void fs_init(void) {
    memcpy(&buffer, flash_target, FLASH_DATA_SIZE);
}

int fs_format(void) {
    struct fs_data_t *data = &buffer.data;

    data->sensing_ir_threshold = SENSING_IR_THRESHOLD_DEFAULT;
    for (int i = 0; i < SENSING_IR_COUNT; i++) {
        data->sensing_ir_bias[i] = 0;
        data->sensing_ir_range[i] = 0xff;
    }

    data->detected_mark_count = 0;
    for (int i = 0; i < DRIVE_MARK_COUNT_MAX; i++) {
        data->detected_mark[i] = MARK_NONE;
        data->detected_tick[i] = 0;
    }

    return fs_flush_data();
}

int fs_flush_data(void) {
    // flash에 값을 쓰는 과정을 안정적으로 수행하기 위해 잠시 인터럽트를 중지한다.
    uint status = save_and_disable_interrupts();
    int ret = 0;

    // flash 메모리에 쓰기 앞서 그 공간을 지운다.
    flash_range_erase(FLASH_DATA_OFFSET, FLASH_SECTOR_SIZE);

    // flash 메모리에 내용을 쓴다.
    flash_range_program(FLASH_DATA_OFFSET, (uint8_t *)&buffer, FLASH_DATA_SIZE);

    // flash 메모리 검증
    for (int i = 0; i < FLASH_DATA_SIZE; i++) {
        if (flash_target->memory[i] != buffer.memory[i]) {
            ret = 1;
            break;
        }
    }

    // 다시 인터럽트를 활성화한다.
    restore_interrupts(status);

    return ret;
}

struct fs_data_t *fs_get_data(void) {
    return &buffer.data;
}
