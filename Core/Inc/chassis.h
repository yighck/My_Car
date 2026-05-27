#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "common.h"

/* ========== 底盘参数 ========== */
#define CHASSIS_WHEEL_TRACK   0.205f  /* 轮距 (m) - 左右轮距 */
#define CHASSIS_WHEEL_BASE    0.180f  /* 轴距 (m) - 前后轴距 */
#define CHASSIS_WHEEL_RADIUS  0.0325f /* 轮半径 (m) */

/* ========== 电机地址 (Emm_V5.0, 串联在 USART2 总线上) ========== */
#define MOTOR_ADDR_FL  0x01  /* 左前 */
#define MOTOR_ADDR_FR  0x02  /* 右前 */
#define MOTOR_ADDR_RL  0x03  /* 左后 */
#define MOTOR_ADDR_RR  0x04  /* 右后 */

/* Emm_V5.0 手册默认使用固定的 0x6B 校验字节。
 * 仅当所有驱动器已配置为 Modbus 校验时才设为 1。 */
#define EMM_USE_MODBUS_CRC  0

/* ========== 速度结构体 ========== */
typedef struct {
    float vx;    /* 前进 (m/s)  */
    float vy;    /* 左移 (m/s) */
    float wz;    /* 逆时针旋转 (rad/s)    */
} velocity_t;

/* ========== 函数声明 ========== */
void chassis_init(void);
void chassis_set_velocity(float vx, float vy, float wz);
void chassis_stop(void);
void chassis_stop_all(void);

/* Emm_V5.0 协议辅助函数 */
void emm_send_speed(UART_HandleTypeDef *huart, uint8_t addr, int16_t speed_rpm, uint8_t accel);
void emm_stop(UART_HandleTypeDef *huart, uint8_t addr, bool lock);
uint16_t emm_crc16(uint8_t *data, uint8_t len);

/* ========== 状态数据 ========== */
extern velocity_t chassis_velocity;

#endif /* __CHASSIS_H */
