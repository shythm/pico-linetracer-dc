#ifndef _FS_H_
#define _FS_H_

#include "config.h"
#include "sensing.h"

struct fs_data_t {
    int sensing_ir_bias[SENSING_IR_COUNT];
    int sensing_ir_range[SENSING_IR_COUNT];
    float sensing_ir_threshold;
    uint detected_mark[DRIVE_MARK_COUNT_MAX];
    uint detected_mark_count;
    uint detected_tick[DRIVE_MARK_COUNT_MAX];
};

/**
 * @brief flash를 장치 사용을 준비한다.
 */
void fs_init(void);

/**
 * @brief filesystem을 포맷한다.
 *
 * @return 0: flash에 쓰기 성공, 1: flash에 쓰기 실패
 */
int fs_format(void);

/**
 * @brief fs_get_data 함수로 가져온 buffer를 flash에 저장한다.
 *
 * @return 0: flash에 쓰기 성공, 1: flash에 쓰기 실패
 */
int fs_flush_data(void);

/**
 * @brief 아직 flash에 저장되지 않은 buffer의 주소를 반환한다.
 *
 * @return struct fs_data_t*
 */
struct fs_data_t *fs_get_data(void);

#endif