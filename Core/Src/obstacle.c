#include "obstacle.h"

/*
 * 避障模块 - STP-23L 激光测距传感器 (USART6, 230400 baud)
 *
 * 帧格式 (196 字节):
 *   [0xAA][0xAA][0xAA][0xAA]  帧头 (4字节)
 *   [0x00]                     设备地址
 *   [0x02]                     功能码 (0x02=测距数据)
 *   [0x00][0x00]               偏移量
 *   [LEN_L][LEN_H]             数据长度
 *   [180字节数据]              12个测量点 x 15字节/点
 *   [4字节时间戳]              小端序
 *   [1字节校验和]              字节4~193之和
 *
 * 每个测量点 (15字节):
 *   [DIST_L][DIST_H]  距离 (mm, uint16_t, 小端序)
 *   [NOISE_L][NOISE_H] 噪声
 *   [PEAK 4字节]       信号强度
 *   [CONFIDENCE]       置信度 (0-100)
 *   [INTG 4字节]       积分计数
 *   [REFTOF 2字节]     温度漂移补偿
 */

/* ========== 协议参数 ========== */
#define STP_HEADER_BYTE     0xAA
#define STP_FRAME_LEN       196
#define STP_DATA_OFFSET     10      /* 数据区起始偏移 */
#define STP_POINTS_PER_FRAME 12     /* 每帧 12 个测量点 */
#define STP_POINT_SIZE      15      /* 每个测量点 15 字节 */
#define STP_CHECKSUM_OFFSET 194     /* 校验和位于第 194 字节 */
#define STP_MAX_DISTANCE    4000    /* 最大有效距离 (mm) */

/* ========== 内部变量 ========== */
static uint16_t distance_mm = 0;    /* 当前距离 (mm) */
static uint32_t last_update_tick = 0;
static bool frame_valid = false;

/* ========== 帧解析 ========== */
static bool stp_parse_frame(uint8_t *frame)
{
    /* 校验帧头: 4 个 0xAA */
    if (frame[0] != STP_HEADER_BYTE || frame[1] != STP_HEADER_BYTE ||
        frame[2] != STP_HEADER_BYTE || frame[3] != STP_HEADER_BYTE)
        return false;

    /* 校验功能码: 0x02 = 测距数据 */
    if (frame[5] != 0x02)
        return false;

    /* 校验和: 字节 4~193 之和 == 字节 194 */
    uint8_t crc = 0;
    for (int i = 4; i < STP_CHECKSUM_OFFSET; i++) {
        crc += frame[i];
    }
    if (crc != frame[STP_CHECKSUM_OFFSET])
        return false;

    /* 提取 12 个测量点的距离, 取平均值 (跳过 distance=0 的无效点) */
    uint32_t sum = 0;
    uint8_t valid_count = 0;

    for (int i = 0; i < STP_POINTS_PER_FRAME; i++) {
        int offset = STP_DATA_OFFSET + i * STP_POINT_SIZE;
        uint16_t dist = (uint16_t)frame[offset] | ((uint16_t)frame[offset + 1] << 8);

        if (dist > 0 && dist <= STP_MAX_DISTANCE) {
            sum += dist;
            valid_count++;
        }
    }

    if (valid_count == 0)
        return false;

    distance_mm = (uint16_t)(sum / valid_count);
    last_update_tick = HAL_GetTick();
    frame_valid = true;
    return true;
}

/* ========== 帧搜索状态机 ========== */
static void stp_search_frame(void)
{
    /*
     * STP-23L 帧头是 4 个连续的 0xAA。
     * 用计数器跟踪已匹配的帧头字节数, 收集满 196 字节后解析。
     */
    static uint8_t frame_buf[STP_FRAME_LEN];
    static uint16_t frame_idx = 0;
    static uint8_t header_count = 0;  /* 已匹配的帧头字节数 */

    while (!ring_buf_empty(&uart6_rx_buf)) {
        uint8_t byte = ring_buf_get(&uart6_rx_buf);

        if (header_count < 4) {
            /* 还在匹配帧头 */
            if (byte == STP_HEADER_BYTE) {
                frame_buf[header_count] = byte;
                header_count++;
            } else {
                /* 帧头匹配失败, 重头开始 */
                header_count = 0;
                /* 当前字节可能是新帧头的第一个 0xAA */
                if (byte == STP_HEADER_BYTE) {
                    frame_buf[0] = byte;
                    header_count = 1;
                }
            }
        } else {
            /* 帧头已匹配, 继续收集后续字节 */
            frame_buf[frame_idx + 4] = byte;  /* 前 4 字节是帧头 */
            frame_idx++;

            if (frame_idx >= STP_FRAME_LEN - 4) {
                /* 收集满一帧, 尝试解析 */
                stp_parse_frame(frame_buf);
                frame_idx = 0;
                header_count = 0;
            }
        }
    }
}

/* ========== 公共接口 ========== */

void obstacle_init(void)
{
    distance_mm = 0;
    last_update_tick = 0;
    frame_valid = false;
    /* USART6 已在 main.c 中以 230400 baud 初始化, 无需额外操作 */
}

void obstacle_process(void)
{
    /* 数据超时: 500ms 内无新帧则清除 */
    if (frame_valid && (HAL_GetTick() - last_update_tick > 500)) {
        distance_mm = 0;
        frame_valid = false;
    }

    /* 从环形缓冲区搜索并解析 STP-23L 帧 */
    stp_search_frame();
}

bool obstacle_detected(void)
{
    if (!frame_valid) return false;
    return distance_mm < OBSTACLE_SAFE_DIST_MM;
}

uint16_t obstacle_get_distance(void)
{
    return distance_mm;
}
