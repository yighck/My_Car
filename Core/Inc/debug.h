#ifndef __DEBUG_H
#define __DEBUG_H

#include "common.h"
#include <stdarg.h>
#include <stdio.h>

/* ========== 日志级别 ========== */
typedef enum {
    LOG_LEVEL_ERROR = 0,    /* 仅错误 */
    LOG_LEVEL_WARN,         /* 警告 + 错误 */
    LOG_LEVEL_INFO,         /* 信息 + 警告 + 错误 */
    LOG_LEVEL_DEBUG,        /* 全部 */
} log_level_t;

/* ========== 函数声明 ========== */
void debug_init(void);
void debug_set_level(log_level_t level);
void debug_log(log_level_t level, const char *tag, const char *fmt, ...);

/* ========== 便捷宏 ========== */
#define LOG_ERR(tag, fmt, ...)   debug_log(LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)
#define LOG_WARN(tag, fmt, ...)  debug_log(LOG_LEVEL_WARN,  tag, fmt, ##__VA_ARGS__)
#define LOG_INFO(tag, fmt, ...)  debug_log(LOG_LEVEL_INFO,  tag, fmt, ##__VA_ARGS__)
#define LOG_DBG(tag, fmt, ...)   debug_log(LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)

#endif /* __DEBUG_H */
