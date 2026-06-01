#ifndef __POSITIONING_H
#define __POSITIONING_H

#include "common.h"

typedef struct {
    float x;        /* 全局 X 坐标，单位 m */
    float y;        /* 全局 Y 坐标，单位 m */
    float angle;    /* 航向角，单位 rad */
    bool  valid;
    uint32_t timestamp;
} pos_data_t;

/* OPS9 协议：
 * 每帧 28 字节，数据区为小端 IEEE-754 float。
 * [0x0D][0x0A]
 * [航向角_deg][俯仰角_deg][横滚角_deg][X_mm][Y_mm][航向角速度_dps]
 * [0x0A][0x0D]
 * 波特率：115200，帧率：200 fps。
 */
#define OPS9_HEADER1        0x0D
#define OPS9_HEADER2        0x0A
#define OPS9_TAIL1          0x0A
#define OPS9_TAIL2          0x0D
#define OPS9_FRAME_LEN      28
#define OPS9_BAUD_RATE      115200

void positioning_init(void);
void positioning_process(void);
bool positioning_is_valid(void);

extern pos_data_t pos_current;

#endif /* __POSITIONING_H */
