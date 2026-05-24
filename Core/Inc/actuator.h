#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "common.h"

/* ========== Servo Angle Defines (degrees) ========== */
/* Gimbal servo (TIM3_CH3 - PB0) */
#define GIMBAL_ANGLE_MIN      0
#define GIMBAL_ANGLE_MAX      180
#define GIMBAL_ANGLE_CENTER   90

/* Material tray servo (TIM3_CH4 - PB1) */
#define TRAY_ANGLE_OPEN       90
#define TRAY_ANGLE_CLOSE      0

/* Gripper servo (TIM8_CH4 - PC9) */
#define GRIPPER_ANGLE_OPEN    60
#define GRIPPER_ANGLE_CLOSE   0

/* ========== Lift Stepper Motor (UART4, Emm_V5.0) ========== */
#define LIFT_MOTOR_ADDR       0x05   /* Lift motor address */
#define LIFT_SPEED_UP         500    /* 50.0 RPM up */
#define LIFT_SPEED_DOWN       -500   /* 50.0 RPM down */
#define LIFT_ACCEL            10     /* Acceleration */

/* ========== Servo Control ========== */
void actuator_init(void);
void servo_set_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void gimbal_set_angle(float angle);
void tray_set_angle(float angle);
void gripper_set_angle(float angle);

/* ========== Gripper Convenience ========== */
void gripper_open(void);
void gripper_close(void);

/* ========== Tray Convenience ========== */
void tray_open(void);
void tray_close(void);

/* ========== Lift Stepper Motor (Emm_V5.0 on UART4) ========== */
void lift_move_up(void);
void lift_move_down(void);
void lift_stop(void);
void lift_set_speed(int16_t speed_rpm10);

#endif /* __ACTUATOR_H */
