# 双轴云台稳定与视觉追踪系统

基于 STM32F103 的双轴云台控制系统，支持惯性稳定、视觉目标追踪和陀螺仪辅助追踪三种工作模式。搭配 K230 视觉模块实现实时目标检测与跟踪。

## 项目概览

- **主控芯片**: STM32F103xB (Cortex-M3, 72MHz)
- **姿态传感器**: MPU6050 (六轴陀螺仪+加速度计，支持 DMP 姿态解算)
- **显示模块**: OLED 显示屏 (I2C)
- **执行机构**: 双步进电机 (PWM 驱动，TIM2/TIM3)
- **视觉模块**: K230 (Kendryte) — 矩形检测，通过 UART 发送目标坐标
- **控制算法**: PID 控制 (增量式/位置式)
- **开发环境**: STM32CubeMX + Keil MDK-ARM

## 目录结构

```
├── Core/                          # 应用层核心代码
│   ├── Inc/                       # 头文件
│   │   ├── main.h                 # 主程序头文件 (引脚定义)
│   │   ├── gpio.h / dma.h / tim.h / usart.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   └── stm32f1xx_it.h
│   └── Src/                       # 源文件
│       ├── main.c                 # 主程序 (含三种模式逻辑)
│       ├── gpio.c / dma.c / tim.c / usart.c
│       ├── stm32f1xx_it.c         # 中断服务函数
│       ├── stm32f1xx_hal_msp.c    # HAL MSP 配置
│       └── system_stm32f1xx.c     # 系统时钟初始化
│
├── Drivers/                       # 驱动层
│   ├── CMSIS/                     # ARM Cortex 微控制器软件接口标准
│   ├── STM32F1xx_HAL_Driver/      # STM32 HAL 库
│   ├── MPU6050/                   # MPU6050 驱动 + DMP 姿态解算
│   │   ├── mpu6050.c/h            # MPU6050 基础驱动
│   │   ├── inv_mpu.c/h            # InvenSense DMP 库
│   │   └── IIC.c/h                # 软件 I2C 驱动
│   ├── OLED/                      # OLED 显示屏驱动
│   │   ├── OLED.c/h               # OLED 驱动
│   │   └── OLEDFONT.h             # 字库
│   ├── pid/                       # PID 控制器
│   │   ├── pid.c/h                # 增量式/位置式 PID
│   ├── IMU/                       # IMU 相关
│   └── icm42688/                  # ICM-42688 驱动 (备用)
│
├── Hardware/                      # 硬件设计文件
│   ├── PCB_Schematic/
│   │   ├── 原理图设计.pdf          # 电路原理图
│   │   ├── 步进电机驱动板PCB制板文件.zip
│   │   ├── 步进电机驱动板bom清单.xlsx
│   │   └── 步进电机驱动板bom坐标清单.xlsx
│   └── 云台部件.zip                # 云台机械结构
│
├── K230detect/                    # K230 视觉检测代码
│   └── 矩形检测2.py               # 矩形目标检测 + UART 坐标发送
│
├── MDK-ARM/                       # Keil MDK 工程文件
├── new_1_2026.ioc                 # STM32CubeMX 工程配置文件
└── README.md
```

## 硬件连接

### 步进电机接口

| 引脚 | 功能 | 说明 |
|------|------|------|
| PB12 | EN_2 | 电机使能 |
| PB13 | dir_2 | 电机2方向 (yaw 轴) |
| PB14 | dir_1 | 电机1方向 (pitch 轴) |
| PB15 | EN_1 | 电机使能 |
| PA6 | TIM3_CH1 | 电机2 PWM 脉冲 |
| PA0 | TIM2_CH1 | 电机1 PWM 脉冲 |

### 按键接口

| 引脚 | 功能 |
|------|------|
| PA10 | 模式1选择 (button1) |
| PA11 | 模式2选择 (button2) |
| PA15 | 模式3选择 (button3) |
| PA9  | 停止/返回 (外部中断) |

### 通信接口

| 外设 | 引脚 | 用途 |
|------|------|------|
| UART2 | PA2(TX) / PA3(RX) | 与 K230 视觉模块通信 |
| UART3 | PB10(TX) / PB11(RX) | 预留串口 |
| I2C1 | PB6(SCL) / PB7(SDA) | MPU6050 通信 |

## 三种工作模式

### MODE 1 — 惯性稳定模式

使用 MPU6050 DMP 姿态解算获取偏航角 (yaw)，通过增量式 PID 控制步进电机，实现云台偏航轴的自稳控制。

- **传感器**: MPU6050 DMP (pitch, roll, yaw)
- **控制轴**: Yaw 轴 (电机2)
- **PID 参数**: P=24.0, I=0.01, D=0.2
- **死区**: ±2°

### MODE 2 — 视觉追踪模式

接收 K230 视觉模块通过 UART2 发送的目标坐标 `(x, y)`，使用两个独立的增量式 PID 控制器分别控制 yaw 轴和 pitch 轴，将目标锁定在画面中心 (120, 120)。

- **数据源**: K230 UART 坐标 (`PCEN x,y`)
- **控制轴**: Yaw + Pitch (双轴)
- **死区**: 中心 ±2 像素
- **PID 参数**: P=4.0/5.0, I=0, D=0.0002

### MODE 3 — 陀螺仪辅助追踪模式

结合 MPU6050 陀螺仪 Z 轴角速度补偿与视觉坐标 PID 控制，提高追踪响应速度和稳定性。

- **数据源**: 视觉坐标 + 陀螺仪 Z 轴补偿
- **补偿系数**: 0.01
- **PID 参数**: P=4.0, I=0, D=0.0001

## 通信协议 (K230 → STM32)

K230 视觉模块检测到矩形目标后，通过 UART2 (115200bps, 8N1) 发送坐标数据：

```
格式: PCEN xxx,yyy
示例: PCEN 120,085
```

| 字段 | 说明 |
|------|------|
| `PCEN` | 帧头标识 (4字节) |
| `xxx` | 目标中心 X 坐标 (0-224, 3位) |
| `,` | 分隔符 |
| `yyy` | 目标中心 Y 坐标 (0-224, 3位) |

## 快速开始

### 1. 硬件准备

- STM32F103 最小系统板 (如 STM32F103C8T6)
- MPU6050 六轴传感器模块
- 双步进电机驱动板 (见 `Hardware/PCB_Schematic/`)
- OLED 显示屏 (I2C, 128x64, SSD1306)
- 云台机械结构 (见 `Hardware/云台部件.zip`)
- K230 视觉模块 (可选，用于 MODE 2/3)

### 2. 软件编译

1. 使用 STM32CubeMX 打开 `new_1_2026.ioc` 工程配置文件
2. 生成 Keil MDK 工程代码
3. 使用 Keil MDK-ARM 打开 `MDK-ARM/new_1_2026.uvprojx`
4. 编译并下载到 STM32F103

### 3. 运行

1. 上电后 OLED 显示 `waiting`，MPU6050 正在初始化
2. 进入模式选择界面：
   ```
   MODE CHOOSE
   MODE1
   MODE2
   MODE3
   ```
3. 按下对应按键选择工作模式：
   - **PA10**: MODE 1 (惯性稳定)
   - **PA11**: MODE 2 (视觉追踪)
   - **PA15**: MODE 3 (陀螺仪辅助追踪)
4. 按下 **PA9** 停止当前模式，返回模式选择界面

### 4. K230 视觉追踪配置

1. 将 `矩形检测2.py` 部署到 K230 开发板
2. 连接 K230 UART2 (TX→RX, RX→TX, GND→GND) 到 STM32 UART2
3. 确保矩形目标在 K230 摄像头视野内
4. STM32 选择 MODE 2 或 MODE 3 开始目标追踪

## PID 控制说明

系统实现了两种 PID 控制方式：

```c
typedef enum {
    POSITION, // 位置式 PID
    DELTA     // 增量式 PID
} PID_Mode;

typedef struct {
    PID_Mode mode;
    float kp, ki, kd;     // PID 系数
    float max_out;        // 输出限幅
    float max_iout;       // 积分限幅
    float Pout, Iout, Dout;
    float out;
    // ...
} PID;
```

- **增量式 (DELTA)**: 输出控制增量，适用于步进电机速度环控制
- **位置式 (POSITION)**: 输出绝对控制量，适用于位置环控制

## 步进电机控制原理

通过改变 TIM2/TIM3 的 ARR 寄存器值实现 PWM 频率调节，从而控制步进电机转速：

```c
speed = init_num / speed_in;
if (speed < 500) TIMx->ARR = 200;   // 最低速限制
else             TIMx->ARR = speed;  // 正常调速
```

- PWM 占空比固定 (pulse_width=5)，频率可调
- 方向控制通过 GPIO (PB13/PB14) 电平切换
- 使能控制通过 GPIO (PB12/PB15)

## 技术栈

| 组件 | 技术 |
|------|------|
| MCU | STM32F103xB, ARM Cortex-M3 |
| HAL | STM32F1xx HAL Driver |
| IDE | STM32CubeMX, Keil MDK-ARM |
| 传感器驱动 | MPU6050 + InvenSense DMP |
| 显示驱动 | SSD1306 OLED (I2C) |
| 控制算法 | 增量式/位置式 PID |
| 视觉方案 | K230 + Python (矩形检测) |
| 硬件设计 | Altium Designer / 立创EDA |

## 版本历史

| 版本 | 提交 | 说明 |
|------|------|------|
| v1.0 | `3e37c20` | 初始版本，基础云台控制框架 |
| v1.1 | `c963ce8` | 添加硬件部分 (原理图/PCB/BOM) |
| v1.2 | `5d9b2f8` | PCB 设计文件完善 |
| v1.3 | `7ec6dd7` | 添加 K230 视觉检测模块支持 |

## 许可证

本项目仅供学习和研究使用。

