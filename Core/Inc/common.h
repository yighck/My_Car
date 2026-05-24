#ifndef __COMMON_H
#define __COMMON_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ========== Hardware Handles (defined in main.c) ========== */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* ========== Timer Channel Defines ========== */
#define SERVO_TIM_PRESCALER   63    /* 64MHz/(63+1)=1MHz tick */
#define SERVO_TIM_PERIOD      19999 /* 1MHz/(19999+1)=50Hz   */

#define SERVO_MIN_PULSE       500   /* 0.5ms = 0 degree   */
#define SERVO_MAX_PULSE       2500  /* 2.5ms = 180 degree */
#define SERVO_CENTER_PULSE    1500  /* 1.5ms = 90 degree  */

/* ========== Servo Angle Macros ========== */
#define ANGLE_TO_PULSE(angle) \
    (SERVO_MIN_PULSE + (uint16_t)((angle) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180.0f))

/* ========== UART Ring Buffer Size ========== */
#define UART_RX_BUF_SIZE  256

/* ========== UART Ring Buffer Structure ========== */
typedef struct {
    uint8_t  data[UART_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} uart_ring_buf_t;

/* ========== Global UART Receive Buffers ========== */
extern uart_ring_buf_t uart1_rx_buf;  /* Vision    */
extern uart_ring_buf_t uart2_rx_buf;  /* Chassis   */
extern uart_ring_buf_t uart3_rx_buf;  /* OPS9      */
extern uart_ring_buf_t uart4_rx_buf;  /* Lift      */
extern uart_ring_buf_t uart5_rx_buf;  /* Screen/QR */

/* ========== Ring Buffer Operations ========== */
static inline void ring_buf_init(uart_ring_buf_t *buf)
{
    buf->head = 0;
    buf->tail = 0;
}

static inline bool ring_buf_empty(uart_ring_buf_t *buf)
{
    return buf->head == buf->tail;
}

static inline void ring_buf_put(uart_ring_buf_t *buf, uint8_t byte)
{
    uint16_t next = (buf->head + 1) % UART_RX_BUF_SIZE;
    if (next != buf->tail) {
        buf->data[buf->head] = byte;
        buf->head = next;
    }
}

static inline uint8_t ring_buf_get(uart_ring_buf_t *buf)
{
    uint8_t byte = buf->data[buf->tail];
    buf->tail = (buf->tail + 1) % UART_RX_BUF_SIZE;
    return byte;
}

static inline uint16_t ring_buf_count(uart_ring_buf_t *buf)
{
    return (buf->head - buf->tail + UART_RX_BUF_SIZE) % UART_RX_BUF_SIZE;
}

/* ========== Utility Functions ========== */
static inline float clampf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static inline float normalize_angle(float angle)
{
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

#endif /* __COMMON_H */
