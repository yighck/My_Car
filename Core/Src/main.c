/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "chassis.h"
#include "actuator.h"
#include "positioning.h"
#include "vision.h"
#include "navigation.h"
#include "qrcode.h"
#include "task.h"
#include "obstacle.h"
#include <stdio.h>

/* 底盘运动测试模式
 * 用途: 脱离任务流程, 通过蓝牙单独调试底盘运动
 * 使用: 取消下面这行的注释, 重新编译烧录, 手机蓝牙串口连接后发指令
 * 关闭: 测试完毕后注释回去, 重新编译即可恢复完整任务流程
 * 指令: V vx vy wz (速度) / N x y (导航) / S (停车) / ? (查询)
 */
// #define CHASSIS_TEST_MODE

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uart_ring_buf_t uart1_rx_buf;  /* USART1: Linux视觉通信 */
uart_ring_buf_t uart2_rx_buf;  /* USART2: 底盘电机+升降电机 */
uart_ring_buf_t uart3_rx_buf;  /* USART3: OPS9定位 */
uart_ring_buf_t uart4_rx_buf;  /* UART4:  串口屏 */
uart_ring_buf_t uart5_rx_buf;  /* UART5:  二维码模块 */
uart_ring_buf_t uart6_rx_buf;  /* USART6: 调试串口 */

/* 每个串口的单字节接收缓冲区 */
uint8_t uart1_rx_byte;
uint8_t uart2_rx_byte;
uint8_t uart3_rx_byte;
uint8_t uart4_rx_byte;
uint8_t uart5_rx_byte;
uint8_t uart6_rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 使能串口中断 */
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
  HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);

  /* 初始化环形缓冲区 */
  ring_buf_init(&uart1_rx_buf);
  ring_buf_init(&uart2_rx_buf);
  ring_buf_init(&uart3_rx_buf);
  ring_buf_init(&uart4_rx_buf);
  ring_buf_init(&uart5_rx_buf);
  ring_buf_init(&uart6_rx_buf);

  /* 启动串口接收中断 (每次1字节) */
  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
  HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
  HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);

  /* 初始化各模块 */
  chassis_init();
  actuator_init();
  positioning_init();
  vision_init();
  obstacle_init();
  navigation_init();
  qr_code_init();
  task_init();

  /* 启动延时 - 等待传感器稳定 */
  HAL_Delay(500);

#ifdef CHASSIS_TEST_MODE
  /* 测试模式启动提示: 上电后通过蓝牙发送帮助信息 */
  {
      const char *banner =
          "\r\n===== 底盘测试模式 (蓝牙) =====\r\n"
          "V vx vy wz  速度(m/s,rad/s)\r\n"
          "N x y       导航到坐标(m)\r\n"
          "S           停车\r\n"
          "?           查询状态\r\n"
          "================================\r\n";
      HAL_UART_Transmit(&huart6, (uint8_t *)banner, strlen(banner), 100);
  }
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef CHASSIS_TEST_MODE
    chassis_test_process();
#else
    task_update();
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;    /* 84MHz/(83+1)=1MHz → 1μs分辨率 */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;    /* 1MHz/(19999+1)=50Hz 舵机标准频率 */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;   /* 168MHz/(167+1)=1MHz → 1μs分辨率 */
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19999;    /* 1MHz/(19999+1)=50Hz 舵机标准频率 */
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef CHASSIS_TEST_MODE
/* ========== 底盘运动测试模式 (蓝牙 USART6) ==========
 *
 * 脱离任务流程, 通过蓝牙单独调试底盘运动。
 * 其他模块 (舵机/升降/视觉等) 不需要蓝牙调试, 不在此模式中。
 *
 * 使用步骤:
 *   1. 取消 main.c 顶部 #define CHASSIS_TEST_MODE 的注释
 *   2. 编译烧录, 上电后蓝牙自动发送帮助菜单
 *   3. 手机蓝牙串口助手连接, 发送指令测试底盘
 *   4. 测试完毕后注释回 #define, 重新编译恢复任务流程
 *
 * 支持的指令 (以换行符结尾):
 *   V vx vy wz  -- 直接速度控制 (持续运动, 直到发 S 停车)
 *       vx: 前进(+)/后退(-), 单位 m/s, 范围约 +/-0.3
 *       vy: 左移(+)/右移(-), 单位 m/s
 *       wz: 逆时针(+)/顺时针(-), 单位 rad/s
 *   N x y       -- 导航到场地坐标 (自动 PID, 到达后停下)
 *   S           -- 立即停车
 *   ?           -- 查询位置和导航状态
 */

/* 行缓冲区: 逐字节接收蓝牙数据, 遇到换行符解析整行 */
static char test_line_buf[64];
static uint8_t test_line_idx = 0;

/* 通过 USART6 发送字符串到蓝牙 */
static void test_send(const char *str)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), 100);
}

/* 解析并执行一条完整的指令行 */
static void test_handle_line(char *line)
{
    char msg[96];
    char cmd = line[0];

    if (cmd == 'V' || cmd == 'v') {
        float vx = 0, vy = 0, wz = 0;
        if (sscanf(line + 1, "%f %f %f", &vx, &vy, &wz) >= 1) {
            chassis_set_velocity(vx, vy, wz);
            snprintf(msg, sizeof(msg), "OK V %.2f %.2f %.2f\r\n", vx, vy, wz);
        } else {
            snprintf(msg, sizeof(msg), "ERR 格式: V vx vy wz\r\n");
        }
    } else if (cmd == 'N' || cmd == 'n') {
        float x = 0, y = 0;
        if (sscanf(line + 1, "%f %f", &x, &y) == 2) {
            nav_goto(x, y);
            snprintf(msg, sizeof(msg), "OK N %.3f %.3f\r\n", x, y);
        } else {
            snprintf(msg, sizeof(msg), "ERR 格式: N x y\r\n");
        }
    } else if (cmd == 'S' || cmd == 's') {
        nav_stop(); chassis_stop();
        snprintf(msg, sizeof(msg), "OK 停车\r\n");
    } else if (cmd == '?') {
        pos_data_t p = pos_current;
        nav_state_t ns = nav_get_state();
        const char *st = (ns==NAV_IDLE)?"IDLE":(ns==NAV_RUNNING)?"RUNNING":
                         (ns==NAV_REACHED)?"REACHED":(ns==NAV_PAUSED)?"PAUSED":
                         (ns==NAV_FAILED)?"FAILED":"?";
        snprintf(msg, sizeof(msg), "POS %.3f %.3f %.1fdeg %s | NAV %s\r\n",
                 p.x, p.y, p.angle*180.0f/M_PI, p.valid?"OK":"LOST", st);
    } else {
        /* 未知指令: 打印帮助 */
        snprintf(msg, sizeof(msg),
                 "ERR 未知. 用法:\r\n"
                 "  V vx vy wz  速度控制\r\n"
                 "  N x y       导航到坐标\r\n"
                 "  S           停车\r\n"
                 "  ?           查询状态\r\n");
    }
    test_send(msg);
}

/* 主循环调用: 从蓝牙缓冲区逐字节读取, 拼行后解析 */
static void chassis_test_process(void)
{
    /* 从环形缓冲区逐字节读取, 遇换行触发解析 */
    while (!ring_buf_empty(&uart6_rx_buf)) {
        char ch = (char)ring_buf_get(&uart6_rx_buf);
        if (ch == '\n' || ch == '\r') {
            if (test_line_idx > 0) {
                test_line_buf[test_line_idx] = '\0';
                test_handle_line(test_line_buf);
                test_line_idx = 0;
            }
        } else if (test_line_idx < sizeof(test_line_buf) - 1) {
            test_line_buf[test_line_idx++] = ch;
        }
    }

    /* N 指令启动的导航需要周期调用 nav_update() 才能推进 */
    nav_update();
}
#endif /* CHASSIS_TEST_MODE */

/* 串口接收完成回调 - 将字节存入环形缓冲区并重新启动接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ring_buf_put(&uart1_rx_buf, uart1_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    else if (huart->Instance == USART2) {
        ring_buf_put(&uart2_rx_buf, uart2_rx_byte);
        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
    }
    else if (huart->Instance == USART3) {
        ring_buf_put(&uart3_rx_buf, uart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
    else if (huart->Instance == UART4) {
        ring_buf_put(&uart4_rx_buf, uart4_rx_byte);
        HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
    }
    else if (huart->Instance == UART5) {
        ring_buf_put(&uart5_rx_buf, uart5_rx_byte);
        HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
    }
    else if (huart->Instance == USART6) {
        ring_buf_put(&uart6_rx_buf, uart6_rx_byte);
        HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);
    }
}

/* UART错误回调 - 通信出错时重新启动接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    else if (huart->Instance == USART2) HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
    else if (huart->Instance == USART3) HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    else if (huart->Instance == UART4) HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
    else if (huart->Instance == UART5) HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
    else if (huart->Instance == USART6) HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 可在此添加自定义错误处理 */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 可在此添加断言失败的错误处理 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
