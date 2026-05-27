#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "common.h"

/* ========== 舵机角度定义 ========== */
/* TODO: 以下角度均为初始值, 需根据实际舵机安装位置实测校准 */

/* 物料托盘舵机 (TIM3_CH4 - PB1) - 旋转托盘, 3个槽位各120° */
#define TRAY_SLOT_0           0     /* TODO: 槽位0角度, 需实测 */
#define TRAY_SLOT_1           120   /* TODO: 槽位1角度 (槽位0 + 120°) */
#define TRAY_SLOT_2           240   /* TODO: 槽位2角度 (槽位0 + 240°) */

/* 夹爪旋转舵机 (TIM3_CH3 - PC8) */
#define GRIPPER_ROTATE_CENTER  90   /* TODO: 夹爪朝前(取料/放料) */
#define GRIPPER_ROTATE_TRAY   90   /* TODO: 夹爪朝向托盘的固定角度, 需实测 */

/* 夹爪开合舵机 (TIM8_CH4 - PC9) */
#define GRIPPER_ANGLE_OPEN    60    /* TODO: 张开角度, 需实测确保能夹住物料 */
#define GRIPPER_ANGLE_CLOSE   0     /* TODO: 闭合角度, 需实测 */

/* ========== 升降步进电机 (USART2总线, Emm_V5.0) ========== */
#define LIFT_MOTOR_ADDR       0x05   /* 升降电机地址 */
#define LIFT_SPEED_UP         100    /* 100 RPM 上升 */
#define LIFT_SPEED_DOWN       -100   /* 100 RPM 下降 */
#define LIFT_ACCEL            10     /* 加速度 */

/* ========== 舵机控制函数 ========== */
void actuator_init(void);
void servo_set_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void tray_set_angle(float angle);
void gripper_rotate_set_angle(float angle);
void gripper_set_angle(float angle);

/* ========== 夹爪便捷函数 ========== */
void gripper_open(void);
void gripper_close(void);
void gripper_rotate_center(void);
void gripper_rotate_to_tray(void);  /* 夹爪转向托盘 */

/* ========== 托盘便捷函数 ========== */
void tray_rotate_to_slot(uint8_t slot);  /* 旋转托盘到指定槽位 (0/1/2) */

/* ========== 升降步进电机 (Emm_V5.0, USART2总线) ========== */
void lift_move_up(void);
void lift_move_down(void);
void lift_stop(void);
void lift_set_speed(int16_t speed_rpm);

#endif /* __ACTUATOR_H */
