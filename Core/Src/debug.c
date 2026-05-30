#include "debug.h"

/* 调试日志模块 - 通过 USART6 TX 输出 (UART全双工, 避障传感器只用RX) */

static log_level_t current_level = LOG_LEVEL_INFO;

void debug_init(void)
{
    current_level = LOG_LEVEL_INFO;
}

void debug_set_level(log_level_t level)
{
    current_level = level;
}

void debug_log(log_level_t level, const char *tag, const char *fmt, ...)
{
    if (level > current_level) return;

    static char buf[128];
    int pos = 0;

    /* 前缀: [时间戳][级别][标签] */
    static const char level_char[] = {'E', 'W', 'I', 'D'};
    pos += snprintf(buf + pos, sizeof(buf) - pos, "[%lu][%c][%s] ",
                    (unsigned long)HAL_GetTick(), level_char[level], tag);

    /* 格式化用户消息 */
    va_list args;
    va_start(args, fmt);
    pos += vsnprintf(buf + pos, sizeof(buf) - pos, fmt, args);
    va_end(args);

    /* 添加换行 */
    if (pos < (int)sizeof(buf) - 2) {
        buf[pos++] = '\r';
        buf[pos++] = '\n';
    }

    /* 通过 USART6 发送 (短超时, 避免阻塞主循环过久) */
    HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)pos, 10);
}
