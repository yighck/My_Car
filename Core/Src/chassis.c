#include "chassis.h"

velocity_t chassis_velocity = {0};

/* ========== Modbus CRC-16 校验 ========== */
uint16_t emm_crc16(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ========== 发送速度指令到 Emm_V5.0 (0xF6) ==========
 * addr:      电机地址 (0x00-0xFF)
 * speed_rpm: 带符号转速 (RPM), 正=顺时针, 负=逆时针
 * accel:     加速度, 单位 1 step/s² (0=立即)
 */
void emm_send_speed(UART_HandleTypeDef *huart, uint8_t addr, int16_t speed_rpm, uint8_t accel)
{
#if EMM_USE_MODBUS_CRC
    uint8_t frame[9];
#else
    uint8_t frame[8];
#endif
    uint16_t speed_abs = (speed_rpm < 0) ? (uint16_t)(-speed_rpm) : (uint16_t)speed_rpm;

    frame[0] = addr;
    frame[1] = 0xF6;           /* 速度控制功能码 */
    frame[2] = (speed_rpm < 0) ? 0x01 : 0x00;   /* 00=顺时针, 01=逆时针 */
    frame[3] = (uint8_t)((speed_abs >> 8) & 0xFF);
    frame[4] = (uint8_t)(speed_abs & 0xFF);
    frame[5] = accel;                           /* 加速度 */
    frame[6] = 0x00;           /* 同步标志 (0=立即执行) */

#if EMM_USE_MODBUS_CRC
    uint16_t crc = emm_crc16(frame, 7);
    frame[7] = (uint8_t)(crc & 0xFF);        /* CRC 低字节 */
    frame[8] = (uint8_t)((crc >> 8) & 0xFF); /* CRC 高字节 */

    HAL_UART_Transmit(huart, frame, 9, 20);
#else
    frame[7] = 0x6B;
    HAL_UART_Transmit(huart, frame, 8, 20);
#endif
}

/* ========== 停止电机 (0xFE) ==========
 * addr: 电机地址
 * lock: true=锁定转子, false=释放转子
 */
void emm_stop(UART_HandleTypeDef *huart, uint8_t addr, bool lock)
{
#if EMM_USE_MODBUS_CRC
    uint8_t frame[6];
#else
    uint8_t frame[5];
#endif
    frame[0] = addr;
    frame[1] = 0xFE;  /* 停止指令 */
    frame[2] = 0x98;  /* 停止子指令 */
    frame[3] = 0x00;  /* 同步标志 (0=立即执行) */

#if EMM_USE_MODBUS_CRC
    uint16_t crc = emm_crc16(frame, 4);
    frame[4] = (uint8_t)(crc & 0xFF);
    frame[5] = (uint8_t)((crc >> 8) & 0xFF);

    HAL_UART_Transmit(huart, frame, 6, 10);
#else
    frame[4] = 0x6B;
    HAL_UART_Transmit(huart, frame, 5, 10);
#endif

    (void)lock;
}

/* 将 m/s 转换为 Emm_V5.0 的 RPM 指令值 */
static int16_t speed_to_cmd(float speed_ms)
{
    /* v = 2*pi*R*RPM/60  =>  RPM = v*60/(2*pi*R) */
    float rpm = speed_ms * 60.0f / (2.0f * M_PI * CHASSIS_WHEEL_RADIUS);
    int16_t cmd = (int16_t)rpm;
    if (cmd > 300) cmd = 300;
    if (cmd < -300) cmd = -300;
    return cmd;
}

/* ========== 麦卡纳姆轮运动学解算 - 设置底盘速度 ========== */
void chassis_set_velocity(float vx, float vy, float wz)
{
    float L = CHASSIS_WHEEL_TRACK;
    float B = CHASSIS_WHEEL_BASE;

    /* 麦卡纳姆轮逆运动学 */
    float v_fl = vx - vy - (L + B) * wz;
    float v_fr = vx + vy + (L + B) * wz;
    float v_rl = vx + vy - (L + B) * wz;
    float v_rr = vx - vy + (L + B) * wz;

    /* 转换为电机指令并发送 */
    int16_t s_fl = speed_to_cmd(v_fl);
    int16_t s_fr = speed_to_cmd(v_fr);
    int16_t s_rl = speed_to_cmd(v_rl);
    int16_t s_rr = speed_to_cmd(v_rr);

    /* 向各电机发送指令 (加速度=10) */
    emm_send_speed(&huart2, MOTOR_ADDR_FL, s_fl, 10);
    emm_send_speed(&huart2, MOTOR_ADDR_FR, s_fr, 10);
    emm_send_speed(&huart2, MOTOR_ADDR_RL, s_rl, 10);
    emm_send_speed(&huart2, MOTOR_ADDR_RR, s_rr, 10);

    chassis_velocity.vx = vx;
    chassis_velocity.vy = vy;
    chassis_velocity.wz = wz;
}

void chassis_stop(void)
{
    chassis_velocity.vx = 0;
    chassis_velocity.vy = 0;
    chassis_velocity.wz = 0;
    emm_stop(&huart2, 0x00, true);  /* 广播停止, 锁定转子 */
}

void chassis_stop_all(void)
{
    emm_stop(&huart2, MOTOR_ADDR_FL, false);
    emm_stop(&huart2, MOTOR_ADDR_FR, false);
    emm_stop(&huart2, MOTOR_ADDR_RL, false);
    emm_stop(&huart2, MOTOR_ADDR_RR, false);
}

void chassis_init(void)
{
    chassis_stop();
}
