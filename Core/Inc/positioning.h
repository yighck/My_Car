#ifndef __POSITIONING_H
#define __POSITIONING_H

#include "common.h"

/* ========== OPS9 Position Data ========== */
typedef struct {
    float x;        /* Global X (m)     */
    float y;        /* Global Y (m)     */
    float angle;    /* Heading (rad)    */
    bool  valid;    /* Data valid flag  */
    uint32_t timestamp;
} pos_data_t;

/* ========== OPS9 协议 (说明书) ========== */
/*
 * 28字节, 小端序:
 * [0x0D][0x0A][X(4B float,mm)][Y(4B float,mm)][航向角(4B float,deg)][速度(4B float)][0x0A][0x0D]
 * 帧头: 0x0D 0x0A, 帧尾: 0x0A 0x0D
 * 帧率: 200fps, 波特率: 115200
 */
#define OPS9_HEADER1        0x0D
#define OPS9_HEADER2        0x0A
#define OPS9_TAIL1          0x0A
#define OPS9_TAIL2          0x0D
#define OPS9_FRAME_LEN      28
#define OPS9_BAUD_RATE      115200

/* ========== Functions ========== */
void positioning_init(void);
void positioning_process(void);
bool positioning_is_valid(void);

/* ========== Data ========== */
extern pos_data_t pos_current;

#endif /* __POSITIONING_H */
