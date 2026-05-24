#ifndef __TASK_H
#define __TASK_H

#include "common.h"
#include "chassis.h"
#include "actuator.h"
#include "positioning.h"
#include "vision.h"
#include "navigation.h"
#include "qrcode.h"

/* ========== 任务状态机 ========== */
typedef enum {
    TASK_INIT,
    TASK_NAV_QR,            /* 导航到二维码板搜索区域 */
    TASK_WAIT_QR,

    /* 第一批: 原料区取料→放托盘→运到粗加工区→放粗加工→搬到暂存区 */
    TASK_B1_PICK_1,         /* 原料区取第1个 */
    TASK_B1_TRAY_1,         /* 放到托盘 */
    TASK_B1_PICK_2,         /* 原料区取第2个 */
    TASK_B1_TRAY_2,         /* 放到托盘 */
    TASK_B1_PICK_3,         /* 原料区取第3个 */
    TASK_B1_TRAY_3,         /* 放到托盘 */
    TASK_B1_PLACE_R1,       /* 粗加工区放第1个 */
    TASK_B1_PLACE_R2,       /* 粗加工区放第2个 */
    TASK_B1_PLACE_R3,       /* 粗加工区放第3个 */
    TASK_B1_MOVE_1,         /* 粗加工区搬到暂存区第1个 */
    TASK_B1_MOVE_2,         /* 粗加工区搬到暂存区第2个 */
    TASK_B1_MOVE_3,         /* 粗加工区搬到暂存区第3个 */

    /* 第二批: 同上流程, 码垛到暂存区 */
    TASK_B2_PICK_1,
    TASK_B2_TRAY_1,
    TASK_B2_PICK_2,
    TASK_B2_TRAY_2,
    TASK_B2_PICK_3,
    TASK_B2_TRAY_3,
    TASK_B2_PLACE_R1,
    TASK_B2_PLACE_R2,
    TASK_B2_PLACE_R3,
    TASK_B2_RETURN_RAW,     /* 第二批放完粗加工后返回原料区 */
    TASK_B2_MOVE_1,
    TASK_B2_MOVE_2,
    TASK_B2_MOVE_3,

    /* 返回 */
    TASK_RETURN_HOME,
    TASK_DONE
} task_state_t;

/* ========== 场地坐标 (米) ========== */
/* 坐标系: 场地左下角为原点, x向右, y向上 */

/* 启停区 (抽签选择, 取消注释对应的一组) */
#if 0  /* 启停区1 */
#define START_X             2.250f
#define START_Y             2.250f
#else  /* 启停区2 */
#define START_X             2.250f
#define START_Y             0.150f
#endif
#define START_ANGLE         M_PI    /* 朝x负方向 */

/* 二维码板搜索区域中心 (待公布后填入) */
#define QR_SEARCH_X         0.400f  /* TODO: 填入二维码板搜索区域中心 X */
#define QR_SEARCH_Y         1.200f  /* TODO: 填入二维码板搜索区域中心 Y */
#define QR_SEARCH_ANGLE     M_PI    /* TODO: 面向二维码板的角度 (rad) */

/* 原料区搜索区域中心 (待公布后填入) */
#define RAW_MATERIAL_X      1.200f  /* TODO: 填入原料区搜索区域中心 X */
#define RAW_MATERIAL_Y      2.400f  /* TODO: 填入原料区搜索区域中心 Y */
#define RAW_MATERIAL_FACE   0.0f    /* TODO: 面向原料区的角度 (rad) */

/* 粗加工区 (3个圆环位置, 按任务码编号1-3) */
#define ROUGH_1_X           1.050f
#define ROUGH_1_Y           0.075f
#define ROUGH_2_X           1.200f
#define ROUGH_2_Y           0.075f
#define ROUGH_3_X           1.350f
#define ROUGH_3_Y           0.075f
#define ROUGH_FACE          0.0f    /* TODO: 粗加工区放置朝向 (rad), 需实测 */

/* 暂存区 (3个圆环位置, 按任务码编号1-3) */
#define TEMP_1_X            0.075f
#define TEMP_1_Y            1.350f
#define TEMP_2_X            0.075f
#define TEMP_2_Y            1.200f
#define TEMP_3_X            0.075f
#define TEMP_3_Y            1.050f
#define TEMP_FACE           0.0f    /* TODO: 暂存区放置朝向 (rad), 需实测 */

/* 码垛: 第二批暂存区下降补偿 (秒) */
#define STACK_EXTRA_MS      500

/* ========== 颜色编号定义 ========== */
#define COLOR_RED           1
#define COLOR_YELLOW        2
#define COLOR_BLUE          3
#define COLOR_GREEN         4
#define COLOR_BLACK         5
#define COLOR_LIGHT_BLUE    6

/* ========== 任务统计 ========== */
typedef struct {
    uint8_t pick_attempts;      /* 总取料尝试次数 */
    uint8_t pick_successes;     /* 成功取料次数 */
    uint8_t place_successes;    /* 成功放置次数 */
    uint8_t vision_timeouts;    /* 视觉超时次数 */
} task_stats_t;

extern task_stats_t task_stats;

/* ========== 函数声明 ========== */
void task_init(void);
void task_update(void);
task_state_t task_get_state(void);

#endif /* __TASK_H */
