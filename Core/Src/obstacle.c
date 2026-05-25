#include "obstacle.h"

/*
 * 避障模块 - USART6 UART 读取测距传感器
 *
 * 默认协议 (TFmini / 兼容格式, 9字节帧):
 *   [0x59][0x59][DIST_L][DIST_H][DIST_H2][DIST_L2][QUAL_L][QUAL_H][CHECKSUM]
 *   - 帧头: 0x59 0x59
 *   - 距离: bytes[2] + bytes[3]<<8 (单位 mm)
 *   - 校验: 前8字节之和 & 0xFF
 *
 * 如果你的传感器协议不同, 只需修改 obstacle_parse_frame() 函数。
 */

/* ========== 协议参数 ========== */
#define OBS_FRAME_LEN       9
#define OBS_HEADER1         0x59
#define OBS_HEADER2         0x59

/* ========== 内部变量 ========== */
static uint16_t distance_mm = 0;
static uint32_t last_update_tick = 0;
static bool frame_valid = false;

/* ========== 帧解析 (根据传感器修改此函数) ========== */
static bool obstacle_parse_frame(uint8_t *frame)
{
    /* 校验头 */
    if (frame[0] != OBS_HEADER1 || frame[1] != OBS_HEADER2)
        return false;

    /* 校验和: 前8字节之和 & 0xFF == 第9字节 */
    uint8_t sum = 0;
    for (int i = 0; i < OBS_FRAME_LEN - 1; i++) {
        sum += frame[i];
    }
    if (sum != frame[OBS_FRAME_LEN - 1])
        return false;

    /* 提取距离 (mm), 小端序 */
    uint16_t dist = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);

    /* 有效性: 0 和 >4000 视为无效 */
    if (dist == 0 || dist > 4000)
        return false;

    distance_mm = dist;
    last_update_tick = HAL_GetTick();
    frame_valid = true;
    return true;
}

/* ========== 公共接口 ========== */

void obstacle_init(void)
{
    distance_mm = 0;
    last_update_tick = 0;
    frame_valid = false;
    /* USART6 已在 main.c 中初始化, 无需额外操作 */
}

void obstacle_process(void)
{
    /* 数据超时: 500ms 内无新数据则清除 */
    if (frame_valid && (HAL_GetTick() - last_update_tick > 500)) {
        distance_mm = 0;
        frame_valid = false;
    }

    /* 从环形缓冲区逐字节读取, 搜索帧头 */
    static uint8_t frame_buf[OBS_FRAME_LEN];
    static uint8_t frame_idx = 0;
    static bool header1_found = false;

    while (!ring_buf_empty(&uart6_rx_buf)) {
        uint8_t byte = ring_buf_get(&uart6_rx_buf);

        if (!header1_found) {
            if (byte == OBS_HEADER1) {
                frame_buf[0] = byte;
                frame_idx = 1;
                header1_found = true;
            }
        } else {
            if (frame_idx == 1 && byte != OBS_HEADER2) {
                header1_found = false;
                frame_idx = 0;
                /* 当前字节可能是新帧头 */
                if (byte == OBS_HEADER1) {
                    frame_buf[0] = byte;
                    frame_idx = 1;
                    header1_found = true;
                }
                continue;
            }

            frame_buf[frame_idx++] = byte;
            if (frame_idx >= OBS_FRAME_LEN) {
                obstacle_parse_frame(frame_buf);
                frame_idx = 0;
                header1_found = false;
            }
        }
    }
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
