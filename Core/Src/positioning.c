#include "positioning.h"

pos_data_t pos_current = {0};

void positioning_init(void)
{
    pos_current.valid = false;
}

/* Verify checksum: SUM of bytes[2..17], 16-bit */
static uint16_t positioning_checksum(uint8_t *data)
{
    uint16_t sum = 0;
    for (int i = 2; i < 18; i++) {
        sum += data[i];
    }
    return sum;
}

/* Parse a complete 20-byte OPS9 frame */
static bool positioning_parse_frame(uint8_t *frame)
{
    /* Verify header */
    if (frame[0] != OPS9_HEADER1 || frame[1] != OPS9_HEADER2) return false;

    /* Verify checksum (bytes 18-19) */
    uint16_t calc_sum = positioning_checksum(frame);
    uint16_t rx_sum = (uint16_t)frame[18] | ((uint16_t)frame[19] << 8);
    if (calc_sum != rx_sum) return false;

    /* Extract position data (little-endian IEEE 754) */
    float x, y, angle;
    memcpy(&x,     &frame[2],  4);
    memcpy(&y,     &frame[6],  4);
    memcpy(&angle, &frame[10], 4);

    /* Validate range (raw mm values from OPS9) */
    if (x < -20000.0f || x > 20000.0f) return false;
    if (y < -20000.0f || y > 20000.0f) return false;
    if (angle < -360.0f || angle > 360.0f) return false;

    pos_current.x = x / 1000.0f;           /* mm -> m */
    pos_current.y = y / 1000.0f;           /* mm -> m */
    pos_current.angle = angle * M_PI / 180.0f;  /* degree -> rad */
    pos_current.valid = true;
    pos_current.timestamp = HAL_GetTick();

    return true;
}

/* Process incoming OPS9 data from UART3 ring buffer */
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
                /* Second byte not 0x00, restart */
                header1_found = false;
                frame_idx = 0;
                /* Check if this byte is a new header */
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
