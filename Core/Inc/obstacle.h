#ifndef __OBSTACLE_H
#define __OBSTACLE_H

#include "common.h"

/* ========== 避障参数 ========== */
#define OBSTACLE_SAFE_DIST_MM   150     /* 安全距离阈值 (mm) */

/* ========== 协议定义 (根据实际传感器修改) ========== */
/*
 * 默认协议: [0x59][0x59][DIST_L][DIST_H][...][CHECKSUM]
 * 9字节帧, 距离为低字节在前的 16-bit (mm)
 *
 * 如果你的传感器协议不同, 修改 obstacle_process() 中的解析逻辑
 */

/* ========== 函数声明 ========== */
void obstacle_init(void);
void obstacle_process(void);            /* 从 uart6_rx_buf 解析距离数据 */
bool obstacle_detected(void);           /* 前方是否有障碍 */
uint16_t obstacle_get_distance(void);   /* 当前距离 (mm), 0=无效 */

#endif /* __OBSTACLE_H */
