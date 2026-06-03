#ifndef __OBSTACLE_H
#define __OBSTACLE_H

#include "common.h"

/* ========== 避障参数 ========== */
#define OBSTACLE_SAFE_DIST_MM   150     /* 安全距离阈值 (mm) */

/* ========== 传感器: STP-23L 激光测距 (USART6, 230400 baud) ==========
 * 196字节帧, 4x0xAA帧头, 12个测量点取平均, 校验和=字节4~193之和
 * 详见 obstacle.c 中的协议说明
 */

/* ========== 函数声明 ========== */
void obstacle_init(void);
void obstacle_process(void);            /* 从 uart6_rx_buf 解析距离数据 */
bool obstacle_detected(void);           /* 前方是否有障碍 */
uint16_t obstacle_get_distance(void);   /* 当前距离 (mm), 0=无效 */

#endif /* __OBSTACLE_H */
