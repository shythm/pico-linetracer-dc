#ifndef _FS_H_
#define _FS_H_

#include "config.h"
#include "mark.h"

struct fs_data_t {
    int sensing_ir_bias[SENSING_IR_COUNT];
    int sensing_ir_range[SENSING_IR_COUNT];
    float sensing_ir_threshold;
    enum mark_t detected_mark[DRIVE_MARK_COUNT_MAX];
    uint detected_mark_count;
    int32_t detected_tick[DRIVE_MARK_COUNT_MAX];
};

/**
 * @brief flash 장치 사용을 준비한다.
 */
void fs_init(void);

/**
 * @brief flash를 기본 값으로 초기화(포맷)한다.
 *
 * @return 0: flash에 쓰기 성공, 1: flash에 쓰기 실패
 */
int fs_format(void);

/**
 * @brief buffer를 flash에 저장한다.
 *
 * @return 0: flash에 쓰기 성공, 1: flash에 쓰기 실패
 */
int fs_flush_data(void);

/**
 * @brief flash의 데이터를 복사하여 보관한 buffer의 주소를 반환한다.
 * 참고로 `fs_init` 함수가 호출된 후에 buffer의 값이 유효해진다.
 * 값을 변경한 후에는 꼭 `fs_flush_data` 함수를 호출하여 buffer에 있는 값을 flash에 작성하도록 한다.
 *
 * @return struct fs_data_t *
 */
struct fs_data_t *fs_get_data(void);

#endif