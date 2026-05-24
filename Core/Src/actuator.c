#include "actuator.h"
#include "chassis.h"  /* emm_crc16, emm_send_speed, emm_stop */

void actuator_init(void)
{
    /* Start PWM on TIM3 CH3 (gripper rotate) and CH4 (tray) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* Start PWM on TIM8 CH4 (gripper open/close) */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* Set default positions */
    gripper_rotate_set_angle(GRIPPER_ROTATE_CENTER);
    tray_set_angle(TRAY_ANGLE_CLOSE);
    gripper_set_angle(GRIPPER_ANGLE_OPEN);
}

/* ========== Servo Control ========== */
void servo_set_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle)
{
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    __HAL_TIM_SET_COMPARE(htim, channel, ANGLE_TO_PULSE(angle));
}

void tray_set_angle(float angle)           { servo_set_angle(&htim3, TIM_CHANNEL_4, angle); }
void gripper_rotate_set_angle(float angle) { servo_set_angle(&htim3, TIM_CHANNEL_3, angle); }
void gripper_set_angle(float angle)        { servo_set_angle(&htim8, TIM_CHANNEL_4, angle); }

void gripper_open(void)          { gripper_set_angle(GRIPPER_ANGLE_OPEN); }
void gripper_close(void)         { gripper_set_angle(GRIPPER_ANGLE_CLOSE); }
void gripper_rotate_center(void) { gripper_rotate_set_angle(GRIPPER_ROTATE_CENTER); }
void gripper_rotate_left(void)   { gripper_rotate_set_angle(GRIPPER_ROTATE_LEFT); }
void gripper_rotate_right(void)  { gripper_rotate_set_angle(GRIPPER_ROTATE_RIGHT); }
void tray_open(void)             { tray_set_angle(TRAY_ANGLE_OPEN); }
void tray_close(void)            { tray_set_angle(TRAY_ANGLE_CLOSE); }

/* ========== 升降步进电机 (Emm_V5.0, USART2总线) ========== */
void lift_set_speed(int16_t speed_rpm)
{
    emm_send_speed(&huart2, LIFT_MOTOR_ADDR, speed_rpm, LIFT_ACCEL);
}

void lift_move_up(void)
{
    lift_set_speed(LIFT_SPEED_UP);
}

void lift_move_down(void)
{
    lift_set_speed(LIFT_SPEED_DOWN);
}

void lift_stop(void)
{
    emm_stop(&huart2, LIFT_MOTOR_ADDR, true);
}
