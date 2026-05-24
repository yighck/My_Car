#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "common.h"
#include "positioning.h"

/* ========== PID Parameters ========== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} pid_t;

/* ========== Navigation State ========== */
typedef enum {
    NAV_IDLE,
    NAV_RUNNING,
    NAV_REACHED
} nav_state_t;

/* ========== Navigation Parameters ========== */
#define NAV_POS_TOLERANCE     0.02f   /* Position tolerance (m) */
#define NAV_ANGLE_TOLERANCE   0.05f   /* Angle tolerance (rad) */
#define NAV_MAX_LINEAR_SPEED  0.3f    /* Max linear speed (m/s) */
#define NAV_MAX_ANGULAR_SPEED 1.5f    /* Max angular speed (rad/s) */

/* ========== Functions ========== */
void navigation_init(void);
void nav_goto(float target_x, float target_y);
void nav_goto_heading(float target_x, float target_y, float target_angle);
void nav_turn_to(float target_angle);
void nav_update(void);
nav_state_t nav_get_state(void);
void nav_stop(void);

/* ========== PID Utilities ========== */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);
float pid_compute(pid_t *pid, float error, float dt);

/* ========== Data ========== */
extern pid_t pid_linear;
extern pid_t pid_angular;
extern nav_state_t nav_state;

#endif /* __NAVIGATION_H */
