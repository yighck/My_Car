#include "navigation.h"
#include "chassis.h"

pid_t pid_linear  = {0};
pid_t pid_angular = {0};
nav_state_t nav_state = NAV_IDLE;

static float target_x = 0;
static float target_y = 0;
static float target_angle = 0;
static bool  heading_control = false;
static uint32_t last_update_tick = 0;

/* 路径跟随状态 */
static const waypoint_t *path_points = NULL;
static uint8_t path_count = 0;
static uint8_t path_index = 0;

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_min = out_min;
    pid->output_max = out_max;
}

float pid_compute(pid_t *pid, float error, float dt)
{
    if (dt <= 0) return 0;

    pid->integral += error * dt;
    /* 积分限幅 */
    if (pid->integral > pid->output_max) pid->integral = pid->output_max;
    if (pid->integral < pid->output_min) pid->integral = pid->output_min;

    float derivative = (error - pid->prev_error) / dt;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    pid->prev_error = error;

    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void navigation_init(void)
{
    /* 线速度PID: 位置误差 -> 速度 */
    pid_init(&pid_linear, 2.0f, 0.01f, 0.5f, -NAV_MAX_LINEAR_SPEED, NAV_MAX_LINEAR_SPEED);

    /* 角速度PID: 角度误差 -> 角速度 */
    pid_init(&pid_angular, 3.0f, 0.02f, 0.8f, -NAV_MAX_ANGULAR_SPEED, NAV_MAX_ANGULAR_SPEED);

    nav_state = NAV_IDLE;
    heading_control = false;
    last_update_tick = 0;
}

void nav_goto(float x, float y)
{
    target_x = x;
    target_y = y;
    heading_control = false;
    nav_state = NAV_RUNNING;
    pid_linear.integral = 0;
    pid_linear.prev_error = 0;
    pid_angular.integral = 0;
    pid_angular.prev_error = 0;
    last_update_tick = HAL_GetTick();
}

void nav_goto_heading(float x, float y, float angle)
{
    target_x = x;
    target_y = y;
    target_angle = angle;
    heading_control = true;
    nav_state = NAV_RUNNING;
    pid_linear.integral = 0;
    pid_linear.prev_error = 0;
    pid_angular.integral = 0;
    pid_angular.prev_error = 0;
    last_update_tick = HAL_GetTick();
}

void nav_turn_to(float angle)
{
    if (positioning_is_valid()) {
        pos_data_t pos = pos_current;
        target_x = pos.x;
        target_y = pos.y;
    }
    target_angle = angle;
    heading_control = true;
    nav_state = NAV_RUNNING;
    pid_linear.integral = 0;
    pid_linear.prev_error = 0;
    pid_angular.integral = 0;
    pid_angular.prev_error = 0;
    last_update_tick = HAL_GetTick();
}

/* 导航主更新函数 - 在主循环中周期调用 */
void nav_update(void)
{
    if (nav_state != NAV_RUNNING) return;
    if (!positioning_is_valid()) return;

    uint32_t now = HAL_GetTick();
    float dt = (now - last_update_tick) / 1000.0f;
    if (dt < 0.01f) return;  /* 限制100Hz更新频率 */
    last_update_tick = now;

    pos_data_t pos = pos_current;

    /* 计算全局坐标系下的位置误差 */
    float dx = target_x - pos.x;
    float dy = target_y - pos.y;
    float dist = sqrtf(dx * dx + dy * dy);

    /* 全局坐标系下的线速度 */
    float vx_global = 0, vy_global = 0;
    if (dist > NAV_POS_TOLERANCE) {
        float linear_speed = pid_compute(&pid_linear, dist, dt);
        /* 归一化方向并施加速度 */
        vx_global = (dx / dist) * linear_speed;
        vy_global = (dy / dist) * linear_speed;
    }

    /* 角速度 */
    float wz = 0;
    if (heading_control) {
        float angle_err = normalize_angle(target_angle - pos.angle);
        if (fabsf(angle_err) > NAV_ANGLE_TOLERANCE) {
            wz = pid_compute(&pid_angular, angle_err, dt);
        }
    } else if (dist > NAV_POS_TOLERANCE) {
        /* 自动面向目标 */
        float desired_angle = atan2f(dy, dx);
        float angle_err = normalize_angle(desired_angle - pos.angle);
        wz = pid_compute(&pid_angular, angle_err, dt);
    }

    /* 全局速度转换到机器人坐标系 */
    float cos_a = cosf(pos.angle);
    float sin_a = sinf(pos.angle);
    float vx_robot =  cos_a * vx_global + sin_a * vy_global;
    float vy_robot = -sin_a * vx_global + cos_a * vy_global;

    /* 判断是否到达 */
    bool reached = (dist < NAV_POS_TOLERANCE);
    if (heading_control) {
        reached = reached && (fabsf(normalize_angle(target_angle - pos.angle)) < NAV_ANGLE_TOLERANCE);
    } else {
        reached = reached && (fabsf(wz) < 0.1f);
    }

    if (reached) {
        /* 检查是否还有下一个路径点 */
        if (path_points != NULL && path_index + 1 < path_count) {
            path_index++;
            waypoint_t wp = path_points[path_index];
            target_x = wp.x;
            target_y = wp.y;
            if (isnan(wp.heading)) {
                heading_control = false;
            } else {
                target_angle = wp.heading;
                heading_control = true;
            }
            pid_linear.integral = 0;
            pid_linear.prev_error = 0;
            pid_angular.integral = 0;
            pid_angular.prev_error = 0;
            last_update_tick = now;
            /* 不停车 - 继续导航到下一个路径点 */
            chassis_set_velocity(0, 0, 0);
        } else {
            path_points = NULL;
            chassis_stop();
            nav_state = NAV_REACHED;
        }
    } else {
        chassis_set_velocity(vx_robot, vy_robot, wz);
    }
}

nav_state_t nav_get_state(void)
{
    return nav_state;
}

void nav_stop(void)
{
    chassis_stop();
    nav_state = NAV_IDLE;
    path_points = NULL;
}

void nav_follow_path(const waypoint_t *path, uint8_t count)
{
    if (path == NULL || count == 0) return;

    path_points = path;
    path_count = count;
    path_index = 0;

    /* 开始导航到第一个路径点 */
    waypoint_t wp = path[0];
    target_x = wp.x;
    target_y = wp.y;
    if (isnan(wp.heading)) {
        heading_control = false;
    } else {
        target_angle = wp.heading;
        heading_control = true;
    }

    nav_state = NAV_RUNNING;
    pid_linear.integral = 0;
    pid_linear.prev_error = 0;
    pid_angular.integral = 0;
    pid_angular.prev_error = 0;
    last_update_tick = HAL_GetTick();
}
