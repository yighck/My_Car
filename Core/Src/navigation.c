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
    /* Anti-windup */
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
    /* Linear PID: position error -> velocity */
    pid_init(&pid_linear, 2.0f, 0.01f, 0.5f, -NAV_MAX_LINEAR_SPEED, NAV_MAX_LINEAR_SPEED);

    /* Angular PID: angle error -> angular velocity */
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
    target_angle = angle;
    heading_control = true;
    nav_state = NAV_RUNNING;
    pid_angular.integral = 0;
    pid_angular.prev_error = 0;
    last_update_tick = HAL_GetTick();
}

/* Main navigation update - call periodically from main loop */
void nav_update(void)
{
    if (nav_state != NAV_RUNNING) return;
    if (!positioning_is_valid()) return;

    uint32_t now = HAL_GetTick();
    float dt = (now - last_update_tick) / 1000.0f;
    if (dt < 0.01f) return;  /* Limit to 100Hz */
    last_update_tick = now;

    pos_data_t pos = pos_current;

    /* Calculate position error in global frame */
    float dx = target_x - pos.x;
    float dy = target_y - pos.y;
    float dist = sqrtf(dx * dx + dy * dy);

    /* Linear velocity in global frame */
    float vx_global = 0, vy_global = 0;
    if (dist > NAV_POS_TOLERANCE) {
        float linear_speed = pid_compute(&pid_linear, dist, dt);
        /* Normalize direction and apply speed */
        vx_global = (dx / dist) * linear_speed;
        vy_global = (dy / dist) * linear_speed;
    }

    /* Angular velocity */
    float wz = 0;
    if (heading_control) {
        float angle_err = normalize_angle(target_angle - pos.angle);
        if (fabsf(angle_err) > NAV_ANGLE_TOLERANCE) {
            wz = pid_compute(&pid_angular, angle_err, dt);
        }
    } else if (dist > NAV_POS_TOLERANCE) {
        /* Auto-face toward target */
        float desired_angle = atan2f(dy, dx);
        float angle_err = normalize_angle(desired_angle - pos.angle);
        wz = pid_compute(&pid_angular, angle_err, dt);
    }

    /* Transform global velocity to robot frame */
    float cos_a = cosf(pos.angle);
    float sin_a = sinf(pos.angle);
    float vx_robot =  cos_a * vx_global + sin_a * vy_global;
    float vy_robot = -sin_a * vx_global + cos_a * vy_global;

    /* Check if reached */
    bool reached = (dist < NAV_POS_TOLERANCE);
    if (heading_control) {
        reached = reached && (fabsf(normalize_angle(target_angle - pos.angle)) < NAV_ANGLE_TOLERANCE);
    } else {
        reached = reached && (fabsf(wz) < 0.1f);
    }

    if (reached) {
        chassis_stop();
        nav_state = NAV_REACHED;
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
}
