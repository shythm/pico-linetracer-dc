#ifndef _FS_H_
#define _FS_H_

#include "sensing.h"

struct flash_data_t {
    int sensing_ir_bias[SENSING_IR_COUNT];
    int sensing_ir_range[SENSING_IR_COUNT];
    float sensing_ir_threshold;
    float drive_velocity_target;
    int drive_curve_decel;
    float drive_curve_coef;
};

void fs_put_data(struct flash_data_t *data);
struct flash_data_t *fs_get_data(void);

#endif