#include "actuator.h"
#include "chassis.h"  /* emm_crc16, emm_send_speed, emm_stop */

void actuator_init(void)
{
    /* 启动 TIM3 CH3 (夹爪旋转) 和 CH4 (托盘) 的 PWM */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* 启动 TIM8 CH4 (夹爪开合) 的 PWM */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* 设置默认位置 */
    gripper_rotate_set_angle(GRIPPER_ROTATE_CENTER);
    tray_set_angle(TRAY_SLOT_0);
    gripper_set_angle(GRIPPER_ANGLE_OPEN);
}

/* ========== 舵机控制函数 ========== */
void servo_set_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle)
{
    if (angle < 0.0f) angle = 0.0f;
    if (angle > (float)SERVO_MAX_ANGLE) angle = (float)SERVO_MAX_ANGLE;
    __HAL_TIM_SET_COMPARE(htim, channel, ANGLE_TO_PULSE(angle));
}

void tray_set_angle(float angle)           { servo_set_angle(&htim3, TIM_CHANNEL_4, angle); }
void gripper_rotate_set_angle(float angle) { servo_set_angle(&htim3, TIM_CHANNEL_3, angle); }
void gripper_set_angle(float angle)        { servo_set_angle(&htim8, TIM_CHANNEL_4, angle); }

void gripper_open(void)          { gripper_set_angle(GRIPPER_ANGLE_OPEN); }
void gripper_close(void)         { gripper_set_angle(GRIPPER_ANGLE_CLOSE); }
void gripper_rotate_center(void) { gripper_rotate_set_angle(GRIPPER_ROTATE_CENTER); }
void gripper_rotate_to_tray(void) { gripper_rotate_set_angle(GRIPPER_ROTATE_TRAY); }

void tray_rotate_to_slot(uint8_t slot)
{
    static const float tray_angles[] = {TRAY_SLOT_0, TRAY_SLOT_1, TRAY_SLOT_2};
    if (slot > 2) slot = 2;
    tray_set_angle(tray_angles[slot]);
}

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
