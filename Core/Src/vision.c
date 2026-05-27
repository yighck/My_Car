#include "vision.h"

vision_data_t vision_current = {0};

void vision_init(void)
{
    vision_current.type = VISION_TYPE_BARCODE;
    vision_current.detected = false;
}

/* 解析一个完整的视觉响应帧 */
static bool vision_parse_frame(uint8_t *frame, uint16_t len)
{
    if (len < 4) return false;
    if (frame[0] != VISION_RX_HEADER) return false;

    vision_current.type = (vision_type_t)frame[1];

    switch (vision_current.type) {
    case VISION_TYPE_BARCODE:
        if (len >= 6) {
            vision_current.barcode_id = ((uint16_t)frame[2] << 8) | frame[3];
            vision_current.detected = true;
            vision_current.timestamp = HAL_GetTick();
            return true;
        }
        break;

    case VISION_TYPE_POSITION:
        if (len >= 12) {
            memcpy(&vision_current.offset_x, &frame[2], 4);
            memcpy(&vision_current.offset_y, &frame[6], 4);
            vision_current.detected = (frame[10] != 0);
            vision_current.timestamp = HAL_GetTick();
            return true;
        }
        break;

    case VISION_TYPE_RECOGNIZE:
        if (len >= 14) {
            vision_current.barcode_id = ((uint16_t)frame[2] << 8) | frame[3];
            memcpy(&vision_current.offset_x, &frame[4], 4);
            memcpy(&vision_current.offset_y, &frame[8], 4);
            vision_current.detected = true;
            vision_current.timestamp = HAL_GetTick();
            return true;
        }
        break;
    }

    return false;
}

/* 从 UART1 环形缓冲区解析视觉数据 */
void vision_process(void)
{
    static uint8_t frame_buf[32];
    static uint16_t frame_idx = 0;
    static bool header_found = false;

    while (!ring_buf_empty(&uart1_rx_buf)) {
        uint8_t byte = ring_buf_get(&uart1_rx_buf);

        if (!header_found) {
            if (byte == VISION_RX_HEADER) {
                frame_buf[0] = byte;
                frame_idx = 1;
                header_found = true;
            }
        } else {
            frame_buf[frame_idx++] = byte;
            if (byte == VISION_TAIL && frame_idx >= 4) {
                vision_parse_frame(frame_buf, frame_idx);
                frame_idx = 0;
                header_found = false;
            }
            if (frame_idx >= sizeof(frame_buf)) {
                frame_idx = 0;
                header_found = false;
            }
        }
    }
}

/* 向 Linux 视觉端发送指令 */
static void vision_send_cmd(uint8_t cmd, uint8_t *data, uint8_t data_len)
{
    uint8_t frame[16];
    uint8_t idx = 0;

    frame[idx++] = VISION_TX_HEADER;
    frame[idx++] = cmd;

    for (uint8_t i = 0; i < data_len && idx < sizeof(frame) - 2; i++) {
        frame[idx++] = data[i];
    }

    /* 校验和: CMD + DATA 的异或值 */
    uint8_t check = cmd;
    for (uint8_t i = 0; i < data_len; i++) {
        check ^= data[i];
    }
    frame[idx++] = check;
    frame[idx++] = VISION_TAIL;

    HAL_UART_Transmit(&huart1, frame, idx, 20);
}

void vision_request_barcode(void)
{
    vision_send_cmd(0x01, NULL, 0);
    vision_current.detected = false;
}

void vision_request_position(void)
{
    vision_send_cmd(0x02, NULL, 0);
    vision_current.detected = false;
}

void vision_request_recognize(void)
{
    vision_send_cmd(0x03, NULL, 0);
    vision_current.detected = false;
}

bool vision_data_available(void)
{
    return vision_current.detected && (HAL_GetTick() - vision_current.timestamp < 2000);
}
