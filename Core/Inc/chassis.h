#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "common.h"

/* ========== Chassis Parameters ========== */
#define CHASSIS_WHEEL_TRACK   0.205f  /* Wheel track (m) - left to right */
#define CHASSIS_WHEEL_BASE    0.180f  /* Wheel base (m)  - front to back */
#define CHASSIS_WHEEL_RADIUS  0.0325f /* Wheel radius (m) */

/* ========== Motor Addresses (Emm_V5.0, daisy-chained on USART2) ========== */
#define MOTOR_ADDR_FL  0x01  /* Front-Left  */
#define MOTOR_ADDR_FR  0x02  /* Front-Right */
#define MOTOR_ADDR_RL  0x03  /* Rear-Left   */
#define MOTOR_ADDR_RR  0x04  /* Rear-Right  */

/* Emm_V5.0 manual default is a fixed 0x6B checksum byte.
 * Set to 1 only if every driver has been configured for Modbus checksum. */
#define EMM_USE_MODBUS_CRC  0

/* ========== Velocity Structure ========== */
typedef struct {
    float vx;    /* Forward (m/s)  */
    float vy;    /* Leftward (m/s) */
    float wz;    /* CCW (rad/s)    */
} velocity_t;

/* ========== Functions ========== */
void chassis_init(void);
void chassis_set_velocity(float vx, float vy, float wz);
void chassis_stop(void);
void chassis_stop_all(void);

/* Emm_V5.0 protocol helpers */
void emm_send_speed(UART_HandleTypeDef *huart, uint8_t addr, int16_t speed_rpm, uint8_t accel);
void emm_stop(UART_HandleTypeDef *huart, uint8_t addr, bool lock);
uint16_t emm_crc16(uint8_t *data, uint8_t len);

/* ========== State ========== */
extern velocity_t chassis_velocity;

#endif /* __CHASSIS_H */
