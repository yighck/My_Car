#ifndef __VISION_H
#define __VISION_H

#include "common.h"

/* ========== 视觉数据类型 ========== */
typedef enum {
    VISION_TYPE_BARCODE,    /* 条码扫描结果 */
    VISION_TYPE_POSITION,   /* 视觉定位目标位置 */
    VISION_TYPE_RECOGNIZE   /* 物料识别 */
} vision_type_t;

typedef struct {
    vision_type_t type;
    uint16_t      barcode_id;   /* 解码后的条码编号 */
    float         offset_x;     /* 相机中心 X 偏移 (m) */
    float         offset_y;     /* 相机中心 Y 偏移 (m) */
    bool          detected;     /* 目标检测标志 */
    uint32_t      timestamp;
} vision_data_t;

/* ========== 通信协议 ========== */
/*
 * Linux 视觉端通过 USART1 发送结构化数据:
 * [0xCC][TYPE][DATA...][0x33]
 *
 * 条码响应:    [0xCC][0x01][ID_H][ID_L][CHECK][0x33]
 * 位置响应:    [0xCC][0x02][X(4)][Y(4)][FOUND(1)][CHECK][0x33]
 * 识别响应:    [0xCC][0x03][ID_H][ID_L][X(4)][Y(4)][CHECK][0x33]
 *
 * 发送到 Linux: [0xDD][CMD][PARAM...][CHECK][0x33]
 *   CMD 0x01: 扫描条码
 *   CMD 0x02: 查找目标位置
 *   CMD 0x03: 识别物料
 *
 * TODO: 请与你的 Linux 视觉程序核对协议
 */
#define VISION_RX_HEADER  0xCC
#define VISION_TX_HEADER  0xDD
#define VISION_TAIL       0x33

/* ========== 函数声明 ========== */
void vision_init(void);
void vision_process(void);
void vision_request_barcode(void);
void vision_request_position(void);
void vision_request_recognize(void);
bool vision_data_available(void);

/* ========== 数据 ========== */
extern vision_data_t vision_current;

#endif /* __VISION_H */
