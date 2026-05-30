#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "common.h"
#include "positioning.h"
#include "obstacle.h"
#include "error_code.h"

/* ========== PID 参数 ========== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float prev_derivative;      /* 上一次微分值 (用于滤波) */
    float deriv_filter_alpha;   /* 微分低通滤波系数 (0~1, 越小滤波越强) */
    float output_min;
    float output_max;
    float integral_max;         /* 积分限幅 (抗饱和) */
} pid_t;

/* ========== 速度规划 ========== */
typedef struct {
    float current_speed;        /* 当前输出速度 */
    float target_speed;         /* 目标速度 (来自PID) */
    float max_speed;            /* 最大允许速度 */
    float accel;                /* 加速度限制 (units/s^2) */
} vel_profile_t;

/* ========== 导航参数 ========== */
#define NAV_POS_TOLERANCE     0.02f   /* 位置容差 (m) */
#define NAV_ANGLE_TOLERANCE   0.05f   /* 角度容差 (rad) */
#define NAV_MAX_LINEAR_SPEED  0.3f    /* 最大线速度 (m/s) */
#define NAV_MAX_ANGULAR_SPEED 1.5f    /* 最大角速度 (rad/s) */
#define NAV_TIMEOUT_MS        10000U  /* 导航超时 (ms) */
#define NAV_POS_LOST_TIMEOUT  1000U   /* 定位丢失超时 (ms) */
#define NAV_ACCEL             0.5f    /* 线加速度 (m/s^2) */
#define NAV_ANGULAR_ACCEL     2.0f    /* 角加速度 (rad/s^2) */
#define NAV_DERIV_FILTER_ALPHA 0.3f   /* 微分滤波系数 */
#define NAV_ROTATE_FIRST_THRESH 0.5f  /* 先旋转阈值 (rad, ~28°) */

/* ========== 导航状态 ========== */
typedef enum {
    NAV_IDLE,
    NAV_RUNNING,
    NAV_REACHED,
    NAV_PAUSED,      /* 避障暂停 */
    NAV_FAILED       /* 导航失败 (超时/定位丢失) */
} nav_state_t;

/* ========== 导航参数 ========== */
#define NAV_POS_TOLERANCE     0.02f   /* 位置容差 (m) */
#define NAV_ANGLE_TOLERANCE   0.05f   /* 角度容差 (rad) */
#define NAV_MAX_LINEAR_SPEED  0.3f    /* 最大线速度 (m/s) */
#define NAV_MAX_ANGULAR_SPEED 1.5f    /* 最大角速度 (rad/s) */
#define NAV_TIMEOUT_MS        10000U  /* 导航超时 (ms) */
#define NAV_POS_LOST_TIMEOUT  1000U   /* 定位丢失超时 (ms) */

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

/* 错误查询 */
error_code_t nav_get_error(void);

/* ========== PID 工具函数 ========== */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);
float pid_compute(pid_t *pid, float error, float dt);

/* ========== 数据 ========== */
extern pid_t pid_linear;
extern pid_t pid_angular;
extern nav_state_t nav_state;

#endif /* __NAVIGATION_H */
