# 智能物流小车 - 项目文档

## 项目概述
2027年第十届中国大学生工创大赛"智能+工程创新"赛项 - 智能物流小车。
自主完成二维码扫描、原料区取料、粗加工区放置、暂存区码垛的完整物流流程。

## 硬件平台
- **MCU**: STM32F405RGTx (Cortex-M4, 168MHz, 1024K Flash, 192K RAM)
- **底盘**: 4个麦轮 + 4个Emm_V5.0步进驱动 (USART2总线, 地址0x01-0x04)
- **升降**: 1个Emm_V5.0步进电机 (USART2总线, 地址0x05)
- **舵机**: 3个270°舵机 (TIM3 CH3/CH4, TIM8 CH4)
- **定位**: OPS9视觉定位系统 (USART3, 28字节帧, 200fps)
- **视觉**: Linux视觉计算机 (USART1, 自定义二进制协议)
- **二维码**: 扫码模块 (UART5, 文本格式)
- **避障**: TFmini激光雷达 (USART6, 9字节帧)
- **串口屏**: HMI显示屏 (UART4)

## 构建方法
```bash
cmake --preset Debug
cmake --build build/Debug
```
工具链: arm-none-eabi-gcc, CMake 3.22+, STM32CubeMX 6.17.0

## 软件架构
```
main.c          → 初始化 + 主循环
  └─ task_update()
       ├─ nav_update()         ← 导航PID + 避障
       ├─ positioning_process()← OPS9数据解析
       ├─ vision_process()     ← 视觉协议解析
       ├─ qr_code_process()   ← 二维码解析
       └─ task state machine  ← 30+状态的任务流程
```

## UART分配
| UART   | 用途         | 波特率 |
|--------|-------------|--------|
| USART1 | Linux视觉    | 115200 |
| USART2 | 底盘+升降电机 | 115200 |
| USART3 | OPS9定位     | 115200 |
| UART4  | 串口屏       | 115200 |
| UART5  | 二维码模块    | 115200 |
| USART6 | 避障+调试日志 | 115200 |

## 状态机流程
```
INIT → NAV_QR → WAIT_QR
  → B1_PICK_1..3 → B1_PLACE_R1..3 → B1_MOVE_1..3
  → B2_PICK_1..3 → B2_PLACE_R1..3 → B2_RETURN_RAW → B2_MOVE_1..3
  → RETURN_HOME → DONE
  ↘ TASK_ERROR (任何状态超时/故障)
```

## 编码规范
- C11标准, HAL库, CubeMX生成的代码不修改USER CODE区域
- 内部函数用static, 全局变量通过extern在头文件声明
- UART接收使用环形缓冲区 (ISR写入, 主循环读取)
- 舵机角度通过 `ANGLE_TO_PULSE()` 宏转换

## 待标定项目 (TODO)
- [ ] 舵机角度: 托盘槽位0/1/2, 夹爪旋转中心/托盘位, 夹爪开/合
- [ ] 场地坐标: QR搜索区中心, 原料区中心, 各区域朝向角
- [ ] 视觉协议: 与Linux端核对通信格式
- [ ] 启停区选择: 比赛当天抽签决定
