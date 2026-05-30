#ifndef __ERROR_CODE_H
#define __ERROR_CODE_H

#include <stdint.h>

/* ========== 错误码定义 ========== */
typedef enum {
    ERR_NONE = 0,               /* 无错误 */
    ERR_NAV_TIMEOUT,            /* 导航超时: 未在规定时间内到达目标 */
    ERR_NAV_POS_LOST,           /* 定位丢失: OPS9 数据中断 */
    ERR_VISION_TIMEOUT,         /* 视觉超时: 视觉系统无响应 */
    ERR_VISION_MAX_RETRIES,     /* 视觉重试上限: 超过最大重试次数 */
    ERR_QR_TIMEOUT,             /* 二维码超时: 未在规定时间内扫描到QR码 */
    ERR_STATE_TIMEOUT,          /* 状态超时: 通用状态超时 */
    ERR_OBSTACLE_STUCK,         /* 障碍卡死: 障碍物持续阻塞 */
    ERR_UART_FAULT,             /* UART故障: 通信错误 */
    ERR_MOTOR_FAULT,            /* 电机故障 (预留) */
} error_code_t;

/* ========== 恢复策略 ========== */
typedef enum {
    RECOVERY_RETRY,             /* 重试当前状态 */
    RECOVERY_SKIP_VISION,       /* 跳过视觉对齐, 盲取/放 */
    RECOVERY_ABORT_TASK,        /* 终止任务, 进入 TASK_DONE */
    RECOVERY_RESTART_STATE,     /* 重新进入当前状态 */
} recovery_action_t;

#endif /* __ERROR_CODE_H */
