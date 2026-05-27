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

/* ========== 硬件句柄 (在 main.c 中定义) ========== */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* ========== 定时器通道定义 ========== */
/* TIM3 时钟 84MHz, TIM8 时钟 168MHz (当前时钟树配置) */
#define SERVO_TIM3_PRESCALER  83
#define SERVO_TIM8_PRESCALER  167
#define SERVO_TIM_PERIOD      19999 /* 1MHz/(19999+1)=50Hz */

#define SERVO_MIN_PULSE       500   /* 0.5ms = 0°   */
#define SERVO_MAX_PULSE       2500  /* 2.5ms = 180° */
#define SERVO_CENTER_PULSE    1500  /* 1.5ms = 90°  */
#define SERVO_MAX_ANGLE       270   /* 支持270°舵机 */

/* ========== 舵机角度宏 ========== */
#define ANGLE_TO_PULSE(angle) \
    (SERVO_MIN_PULSE + (uint16_t)((angle) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / (float)SERVO_MAX_ANGLE))

/* ========== 串口接收缓冲区大小 ========== */
#define UART_RX_BUF_SIZE  256

/* ========== 串口接收缓冲区结构体 ========== */
typedef struct {
    uint8_t  data[UART_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} uart_ring_buf_t;

/* ========== 全局串口接收缓冲区 ========== */
extern uart_ring_buf_t uart1_rx_buf;  /* USART1: Linux视觉通信 */
extern uart_ring_buf_t uart2_rx_buf;  /* USART2: 底盘电机+升降电机 */
extern uart_ring_buf_t uart3_rx_buf;  /* USART3: OPS9定位 */
extern uart_ring_buf_t uart4_rx_buf;  /* UART4:  串口屏 */
extern uart_ring_buf_t uart5_rx_buf;  /* UART5:  二维码模块 */
extern uart_ring_buf_t uart6_rx_buf;  /* USART6: 调试串口 */

/* ========== 环形缓冲区操作 ========== */
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

/* ========== 工具函数 ========== */
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
