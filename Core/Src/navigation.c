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

/* 导航超时和定位丢失检测 */
static uint32_t nav_start_tick = 0;
static uint32_t nav_pos_lost_tick = 0;
static error_code_t nav_error = ERR_NONE;

/* 速度规划器 */
static vel_profile_t linear_profile = {0};
static vel_profile_t angular_profile = {0};

/* 路径跟随状态 */
static const waypoint_t *path_points = NULL;
static uint8_t path_count = 0;
static uint8_t path_index = 0;

/* ========== 重置PID/规划器状态 ========== */
static void reset_pid_state(void)
{
    pid_linear.integral = 0;
    pid_linear.prev_error = 0;
    pid_linear.prev_derivative = 0;
    pid_angular.integral = 0;
    pid_angular.prev_error = 0;
    pid_angular.prev_derivative = 0;
    linear_profile.current_speed = 0;
    angular_profile.current_speed = 0;
}

/* ========== PID 初始化 ========== */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_derivative = 0;
    pid->deriv_filter_alpha = NAV_DERIV_FILTER_ALPHA;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_max = out_max * 0.5f;  /* 积分限幅 = 输出范围的50% */
}

/* ========== PID 计算 (带微分滤波 + 抗积分饱和) ========== */
float pid_compute(pid_t *pid, float error, float dt)
{
    if (dt <= 0) return 0;

    /* P 项 */
    float p_term = pid->kp * error;

    /* I 项 (抗积分饱和: 独立限幅) */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    /* D 项 (一阶低通滤波) */
    float raw_derivative = (error - pid->prev_error) / dt;
    float filtered_derivative = pid->deriv_filter_alpha * raw_derivative
                              + (1.0f - pid->deriv_filter_alpha) * pid->prev_derivative;
    pid->prev_derivative = filtered_derivative;
    pid->prev_error = error;
    float d_term = pid->kd * filtered_derivative;

    /* 输出限幅 */
    float output = p_term + i_term + d_term;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

/* ========== 速度规划 (梯形加减速) ========== */
static float vel_profile_update(vel_profile_t *prof, float target, float dt)
{
    prof->target_speed = target;
    float max_delta = prof->accel * dt;

    float delta = prof->target_speed - prof->current_speed;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    prof->current_speed += delta;

    if (prof->current_speed >  prof->max_speed) prof->current_speed =  prof->max_speed;
    if (prof->current_speed < -prof->max_speed) prof->current_speed = -prof->max_speed;

    return prof->current_speed;
}

/* ========== 导航初始化 ========== */
void navigation_init(void)
{
    /* 线速度PID: Kp降低(速度规划已限变化率), Kd降低(已滤波) */
    pid_init(&pid_linear, 1.5f, 0.01f, 0.3f, -NAV_MAX_LINEAR_SPEED, NAV_MAX_LINEAR_SPEED);

    /* 角速度PID: 同理 */
    pid_init(&pid_angular, 2.0f, 0.02f, 0.5f, -NAV_MAX_ANGULAR_SPEED, NAV_MAX_ANGULAR_SPEED);

    /* 速度规划器初始化 */
    linear_profile.current_speed = 0;
    linear_profile.max_speed = NAV_MAX_LINEAR_SPEED;
    linear_profile.accel = NAV_ACCEL;
    angular_profile.current_speed = 0;
    angular_profile.max_speed = NAV_MAX_ANGULAR_SPEED;
    angular_profile.accel = NAV_ANGULAR_ACCEL;

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
    nav_error = ERR_NONE;
    nav_pos_lost_tick = 0;
    reset_pid_state();
    last_update_tick = HAL_GetTick();
    nav_start_tick = HAL_GetTick();
}

void nav_goto_heading(float x, float y, float angle)
{
    target_x = x;
    target_y = y;
    target_angle = angle;
    heading_control = true;
    nav_state = NAV_RUNNING;
    nav_error = ERR_NONE;
    nav_pos_lost_tick = 0;
    reset_pid_state();
    last_update_tick = HAL_GetTick();
    nav_start_tick = HAL_GetTick();
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
    nav_error = ERR_NONE;
    nav_pos_lost_tick = 0;
    reset_pid_state();
    last_update_tick = HAL_GetTick();
    nav_start_tick = HAL_GetTick();
}

/* 导航主更新函数 - 在主循环中周期调用 */
void nav_update(void)
{
    obstacle_process();

    if (nav_state == NAV_PAUSED) {
        if (!obstacle_detected()) {
            nav_resume();
        }
        return;
    }
    if (nav_state != NAV_RUNNING) return;

    /* 导航超时检查 */
    if ((HAL_GetTick() - nav_start_tick) > NAV_TIMEOUT_MS) {
        chassis_stop();
        nav_state = NAV_FAILED;
        nav_error = ERR_NAV_TIMEOUT;
        return;
    }

    /* 定位数据丢失检查 */
    if (!positioning_is_valid()) {
        if (nav_pos_lost_tick == 0) {
            nav_pos_lost_tick = HAL_GetTick();
        } else if (HAL_GetTick() - nav_pos_lost_tick > NAV_POS_LOST_TIMEOUT) {
            chassis_stop();
            nav_state = NAV_FAILED;
            nav_error = ERR_NAV_POS_LOST;
            return;
        }
        return;  /* 等待定位恢复 (在容忍时间内) */
    } else {
        nav_pos_lost_tick = 0;  /* 定位有效, 重置丢失计时 */
    }

    /* 运动中: 检查前方障碍 */
    if (obstacle_detected()) {
        nav_pause();
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = (now - last_update_tick) / 1000.0f;
    if (dt < 0.01f) return;  /* 限制100Hz更新频率 */
    last_update_tick = now;

    pos_data_t pos = pos_current;

    /* 计算全局坐标系下的位置误差 */
    float dx = target_x - pos.x;
    float dy = target_y - pos.y;
    float dist = sqrtf(dx * dx + dy * dy);

    /* 计算航向误差 */
    float angle_err = 0;
    if (heading_control) {
        angle_err = normalize_angle(target_angle - pos.angle);
    } else if (dist > NAV_POS_TOLERANCE) {
        float desired_angle = atan2f(dy, dx);
        angle_err = normalize_angle(desired_angle - pos.angle);
    }

    /* 策略: 大角度偏差时先原地旋转, 不平移 */
    bool rotate_first = (fabsf(angle_err) > NAV_ROTATE_FIRST_THRESH);

    /* 全局坐标系下的线速度 (带速度规划) */
    float vx_global = 0, vy_global = 0;
    if (!rotate_first && dist > NAV_POS_TOLERANCE) {
        float linear_speed = pid_compute(&pid_linear, dist, dt);
        linear_speed = vel_profile_update(&linear_profile, linear_speed, dt);
        vx_global = (dx / dist) * linear_speed;
        vy_global = (dy / dist) * linear_speed;
    } else if (rotate_first) {
        /* 旋转期间清零线速度规划器 */
        linear_profile.current_speed = 0;
    }

    /* 角速度 (带速度规划) */
    float wz = 0;
    if (fabsf(angle_err) > NAV_ANGLE_TOLERANCE) {
        wz = pid_compute(&pid_angular, angle_err, dt);
        wz = vel_profile_update(&angular_profile, wz, dt);
    }

    /* 全局速度转换到机器人坐标系 */
    float cos_a = cosf(pos.angle);
    float sin_a = sinf(pos.angle);
    float vx_robot =  cos_a * vx_global + sin_a * vy_global;
    float vy_robot = -sin_a * vx_global + cos_a * vy_global;

    /* 判断是否到达 (位置+角度+速度均满足) */
    float current_speed = sqrtf(chassis_velocity.vx * chassis_velocity.vx +
                                chassis_velocity.vy * chassis_velocity.vy);
    bool reached = (dist < NAV_POS_TOLERANCE) && (current_speed < 0.05f);
    if (heading_control) {
        reached = reached && (fabsf(angle_err) < NAV_ANGLE_TOLERANCE);
        reached = reached && (fabsf(chassis_velocity.wz) < 0.1f);
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
            reset_pid_state();
            last_update_tick = now;
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

void nav_pause(void)
{
    if (nav_state == NAV_RUNNING) {
        chassis_stop();
        nav_state = NAV_PAUSED;
    }
}

void nav_resume(void)
{
    if (nav_state == NAV_PAUSED) {
        last_update_tick = HAL_GetTick();
        nav_state = NAV_RUNNING;
    }
}

void nav_stop(void)
{
    chassis_stop();
    nav_state = NAV_IDLE;
    nav_error = ERR_NONE;
    path_points = NULL;
}

error_code_t nav_get_error(void)
{
    return nav_error;
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
    nav_error = ERR_NONE;
    nav_pos_lost_tick = 0;
    reset_pid_state();
    last_update_tick = HAL_GetTick();
    nav_start_tick = HAL_GetTick();
}
