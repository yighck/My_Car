#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "common.h"

/* ========== Servo Angle Defines (degrees) ========== */
/* Material tray servo (TIM3_CH4 - PB1) */
#define TRAY_ANGLE_OPEN       90
#define TRAY_ANGLE_CLOSE      0

/* Gripper rotate servo (TIM3_CH3 - PC8) */
#define GRIPPER_ROTATE_CENTER  90
#define GRIPPER_ROTATE_LEFT    0
#define GRIPPER_ROTATE_RIGHT   180

/* Gripper open/close servo (TIM8_CH4 - PC9) */
#define GRIPPER_ANGLE_OPEN    60
#define GRIPPER_ANGLE_CLOSE   0

/* ========== 升降步进电机 (USART2总线, Emm_V5.0) ========== */
#define LIFT_MOTOR_ADDR       0x05   /* Lift motor address */
#define LIFT_SPEED_UP         50     /* 50 RPM up */
#define LIFT_SPEED_DOWN       -50    /* 50 RPM down */
#define LIFT_ACCEL            10     /* Acceleration */

/* ========== Servo Control ========== */
void actuator_init(void);
void servo_set_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void tray_set_angle(float angle);
void gripper_rotate_set_angle(float angle);
void gripper_set_angle(float angle);

/* ========== Gripper Convenience ========== */
void gripper_open(void);
void gripper_close(void);
void gripper_rotate_center(void);
void gripper_rotate_left(void);
void gripper_rotate_right(void);

/* ========== Tray Convenience ========== */
void tray_open(void);
void tray_close(void);

/* ========== 升降步进电机 (Emm_V5.0, USART2总线) ========== */
void lift_move_up(void);
void lift_move_down(void);
void lift_stop(void);
void lift_set_speed(int16_t speed_rpm);

#endif /* __ACTUATOR_H */
