#include "positioning.h"

pos_data_t pos_current = {0};

void positioning_init(void)
{
    pos_current.valid = false;
}

static bool positioning_parse_frame(uint8_t *frame)
{
    if (frame[0] != OPS9_HEADER1 || frame[1] != OPS9_HEADER2) return false;
    if (frame[26] != OPS9_TAIL1 || frame[27] != OPS9_TAIL2) return false;

    /* OPS9 载荷顺序:
     * 航向角, 俯仰角, 横滚角, X, Y, 航向角速度 */
    float angle, x, y;
    memcpy(&angle, &frame[2],  4);
    memcpy(&x,     &frame[14], 4);
    memcpy(&y,     &frame[18], 4);

    if (x < -20000.0f || x > 20000.0f) return false;
    if (y < -20000.0f || y > 20000.0f) return false;
    if (angle < -360.0f || angle > 360.0f) return false;

    pos_current.x = x / 1000.0f;
    pos_current.y = y / 1000.0f;
    pos_current.angle = angle * M_PI / 180.0f;
    pos_current.valid = true;
    pos_current.timestamp = HAL_GetTick();

    return true;
}

void positioning_process(void)
{
    static uint8_t frame_buf[OPS9_FRAME_LEN];
    static uint16_t frame_idx = 0;
    static bool header1_found = false;

    while (!ring_buf_empty(&uart3_rx_buf)) {
        uint8_t byte = ring_buf_get(&uart3_rx_buf);

        if (!header1_found) {
            if (byte == OPS9_HEADER1) {
                frame_buf[0] = byte;
                frame_idx = 1;
                header1_found = true;
            }
        } else {
            if (frame_idx == 1 && byte != OPS9_HEADER2) {
                header1_found = false;
                frame_idx = 0;

                if (byte == OPS9_HEADER1) {
                    frame_buf[0] = byte;
                    frame_idx = 1;
                    header1_found = true;
                }
                continue;
            }

            frame_buf[frame_idx++] = byte;
            if (frame_idx >= OPS9_FRAME_LEN) {
                positioning_parse_frame(frame_buf);
                frame_idx = 0;
                header1_found = false;
            }
        }
    }
}

bool positioning_is_valid(void)
{
    return pos_current.valid && (HAL_GetTick() - pos_current.timestamp < 500);
}
