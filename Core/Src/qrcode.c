#include "qrcode.h"

qr_task_t qr_task = {0};

void qr_code_init(void)
{
    memset(&qr_task, 0, sizeof(qr_task));
}

/* 解析 "156+123+516+231" 格式的任务码 */
static bool qr_parse_task(const char *buf, uint16_t len)
{
    /* 预期格式: "DDD+DDD+DDD+DDD" = 15字符 */
    if (len < 15) return false;

    /* 验证分隔符 */
    if (buf[3] != '+' || buf[7] != '+' || buf[11] != '+') return false;

    qr_task_t task;

    /* 第一批颜色 */
    task.batch1_colors[0] = buf[0] - '0';
    task.batch1_colors[1] = buf[1] - '0';
    task.batch1_colors[2] = buf[2] - '0';

    /* 第一批放置位置 */
    task.batch1_positions[0] = buf[4] - '0';
    task.batch1_positions[1] = buf[5] - '0';
    task.batch1_positions[2] = buf[6] - '0';

    /* 第二批颜色 */
    task.batch2_colors[0] = buf[8] - '0';
    task.batch2_colors[1] = buf[9] - '0';
    task.batch2_colors[2] = buf[10] - '0';

    /* 第二批放置位置 */
    task.batch2_positions[0] = buf[12] - '0';
    task.batch2_positions[1] = buf[13] - '0';
    task.batch2_positions[2] = buf[14] - '0';

    /* 验证数字范围 (颜色1-6, 位置1-3) */
    for (int i = 0; i < 3; i++) {
        if (task.batch1_colors[i] < 1 || task.batch1_colors[i] > 6) return false;
        if (task.batch2_colors[i] < 1 || task.batch2_colors[i] > 6) return false;
        if (task.batch1_positions[i] < 1 || task.batch1_positions[i] > 3) return false;
        if (task.batch2_positions[i] < 1 || task.batch2_positions[i] > 3) return false;
    }

    task.valid = true;
    qr_task = task;
    return true;
}

void qr_code_process(void)
{
    if (qr_task.valid) return;  /* 已解析过 */

    static char line_buf[32];
    static uint16_t line_idx = 0;

    while (!ring_buf_empty(&uart5_rx_buf)) {
        char ch = (char)ring_buf_get(&uart5_rx_buf);

        if (ch == '\n' || ch == '\r') {
            if (line_idx > 0) {
                line_buf[line_idx] = '\0';
                qr_parse_task(line_buf, line_idx);
                line_idx = 0;
            }
        } else if (line_idx < sizeof(line_buf) - 1) {
            line_buf[line_idx++] = ch;
        }
    }
}

bool qr_code_available(void)
{
    return qr_task.valid;
}
