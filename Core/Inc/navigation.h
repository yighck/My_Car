#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "common.h"
#include "positioning.h"
#include "obstacle.h"

/* ========== PID 参数 ========== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} pid_t;

/* ========== 导航状态 ========== */
typedef enum {
    NAV_IDLE,
    NAV_RUNNING,
    NAV_REACHED,
    NAV_PAUSED       /* 避障暂停 */
} nav_state_t;

/* ========== 导航参数 ========== */
#define NAV_POS_TOLERANCE     0.02f   /* 位置容差 (m) */
#define NAV_ANGLE_TOLERANCE   0.05f   /* 角度容差 (rad) */
#define NAV_MAX_LINEAR_SPEED  0.3f    /* 最大线速度 (m/s) */
#define NAV_MAX_ANGULAR_SPEED 1.5f    /* 最大角速度 (rad/s) */

/* ========== 路径点 ========== */
typedef struct {
    float x;
    float y;
    float heading;  /* 目标朝向 (rad), NAN = 自动面向目标 */
} waypoint_t;

#define NAV_MAX_PATH_POINTS  16

/* ========== 函数声明 ========== */
void navigation_init(void);
void nav_goto(float target_x, float target_y);
void nav_goto_heading(float target_x, float target_y, float target_angle);
void nav_turn_to(float target_angle);
void nav_update(void);
nav_state_t nav_get_state(void);
void nav_stop(void);
void nav_pause(void);    /* 避障暂停 */
void nav_resume(void);   /* 避障恢复 */

/* 路径点导航 */
void nav_follow_path(const waypoint_t *path, uint8_t count);

/* ========== PID 工具函数 ========== */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);
float pid_compute(pid_t *pid, float error, float dt);

/* ========== 数据 ========== */
extern pid_t pid_linear;
extern pid_t pid_angular;
extern nav_state_t nav_state;

#endif /* __NAVIGATION_H */
