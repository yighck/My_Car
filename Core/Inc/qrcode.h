#ifndef __QRCODE_H
#define __QRCODE_H

#include "common.h"

/* ========== 二维码任务数据 ========== */
typedef struct {
    uint8_t batch1_colors[3];    /* 第一批颜色顺序 (1-6) */
    uint8_t batch1_positions[3]; /* 第一批放置位置 (1-3) */
    uint8_t batch2_colors[3];    /* 第二批颜色顺序 */
    uint8_t batch2_positions[3]; /* 第二批放置位置 (1-3) */
    bool    valid;
} qr_task_t;

/* ========== 函数声明 ========== */
void qr_code_init(void);
void qr_code_process(void);
bool qr_code_available(void);

/* ========== 数据 ========== */
extern qr_task_t qr_task;

#endif /* __QRCODE_H */
