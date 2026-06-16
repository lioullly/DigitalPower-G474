---
name: g474-hardware-reference
description: STM32G474 核心板完整硬件参考。包含 GPIO 引脚分配、ADC/DMA/HRTIM 配置、中断优先级、采样标定常数。适用于 DC/AC 逆变、AC/DC PFC、DC/DC 变换器等所有基于此核心板的电力电子项目。
---

# G474 Core Board Hardware Reference

## 平台概述

| 参数 | 值 |
|---|---|
| MCU | STM32G474RBT6 (Cortex-M4F, 170MHz, FPU+DSP) |
| 封装 | LQFP64 |
| 系统时钟 | HSI→PLL: 16M×20/2 = **160MHz** |
| 供电 | 3.3V, Scale1 Boost |
| Flash 延迟 | 4 wait states |

---

## GPIO 引脚总表

### 数字输入

| 标签 | 端口 | 引脚 | 用途 |
|------|------|------|------|
| KEY2 | GPIOC | PC0 | 按键 |
| KEY3 | GPIOC | PC4 | 按键 |
| user | GPIOC | PC5 | 启停按键 / 故障清除 |
| KEY4 | GPIOC | PC10 | 按键 |
| KEY1 | GPIOB | PB13 | 按键 |

### 数字输出

| 标签 | 端口 | 引脚 | 初始状态 | 用途 |
|------|------|------|----------|------|
| Green | GPIOB | PB11 | RESET | 运行心跳指示 |
| key_relay | GPIOB | PB12 | RESET | 继电器控制 |
| Red | GPIOA | PA15 | RESET | 故障指示 |

### HRTIM PWM 输出 (AF13)

| 通道 | 端口 | 引脚 | 半桥 |
|------|------|------|------|
| HRTIM_CHA1 | GPIOA | PA8 | A相上管 |
| HRTIM_CHA2 | GPIOA | PA9 | A相下管 |
| HRTIM_CHB1 | GPIOA | PA10 | B相上管 |
| HRTIM_CHB2 | GPIOA | PA11 | B相下管 |
| HRTIM_CHF1 | GPIOC | PC6 | F相上管 |
| HRTIM_CHF2 | GPIOC | PC7 | F相下管 |

### 模拟输入 (ADC)

| 信号 | 端口 | 引脚 | ADC | 通道 | 模式 |
|------|------|------|-----|------|------|
| il1_p (电流L1+) | GPIOC | PC1 | ADC1 | IN7 | 差分 |
| il1_n (电流L1-) | GPIOC | PC2 | ADC1 | IN8 | 差分 |
| il2_p (电流L2+) | GPIOA | PA2 | ADC1 | IN3 | 差分 |
| il2_n (电流L2-) | GPIOA | PA3 | ADC1 | IN4 | 差分 |
| Udc / Uab | GPIOA | PA0 | ADC2 | IN1 | 单端 |
| Uac / Udc | GPIOA | PA1 | ADC2 | IN2 | 单端 |
| Ubc | GPIOA | PA6 | ADC2 | IN3 | 单端 |
| Uba | GPIOA | PA7 | ADC2 | IN4 | 单端 |
| il3_p (电流L3+) | GPIOB | PB14 | ADC4 | IN4 | 差分 |
| il3_n (电流L3-) | GPIOB | PB15 | ADC4 | IN5 | 差分 |

### 通信/调试

| 功能 | 端口 | 引脚 | AF |
|------|------|------|-----|
| USART1_TX | GPIOB | PB6 | AF7 |
| USART1_RX | GPIOB | PB7 | AF7 |
| I2C3_SCL | GPIOC | PC8 | AF8 |
| I2C3_SDA | GPIOC | PC9 | AF8 |
| DAC1_OUT1 | GPIOA | PA4 | Analog |

### 未使用的引脚（可重新分配）

| 端口 | 引脚 |
|------|------|
| PA5, PA12, PA13, PA14 | SWD 占用 (PA13=SWDIO, PA14=SWCLK) |
| PB0, PB1, PB2, PB3, PB4, PB5, PB8, PB9, PB10 | 空闲 |
| PC3, PC11, PC12, PC13, PC14, PC15 | 空闲 |
| PD2 | 空闲 |

---

## ADC 详细配置

### ADC1 — 注入组 (差分电流采样)

| 参数 | 值 |
|------|-----|
| 时钟 | SYSCLK/4 = 40MHz |
| 分辨率 | 12-bit, 右对齐 |
| 触发源 | HRTIM_TRG2, 上升沿 |
| 注入通道数 | 2 |
| 采样时间 | 24.5 cycles |

| 注入次序 | 通道 | 引脚 | 信号 | 差分对 |
|----------|------|------|------|--------|
| Rank 1 | CH7 (IN7) | PC1 | il1_p | IN7-IN8 (PC1-PC2) |
| Rank 2 | CH3 (IN3) | PA2 | il2_p | IN3-IN4 (PA2-PA3) |

**中断:** ADC1_2_IRQn, 优先级 0,0 (最高)
**ADC值范围:** ±2048 (差分, 16-bit有符号右移3位≈12-bit差分)
**偏移量:** ≈2050 (实测, 对应 1.65V 零电流)

### ADC2 — 规则组+DMA (单端电压采样)

| 参数 | 值 |
|------|-----|
| 时钟 | SYSCLK/4 = 40MHz |
| 分辨率 | 12-bit, 右对齐 |
| 模式 | 连续扫描, 软件启动 |
| DMA | DMA1_Channel1, 循环, 半字 |
| 通道数 | 4 |

| 次序 | 通道 | 引脚 | 当前用途 | 采样时间 |
|------|------|------|----------|----------|
| Rank 1 | CH1 (IN1) | PA0 | Uab 电压 | 47.5 cycles |
| Rank 2 | CH2 (IN2) | PA1 | Udc 电压 | 24.5 cycles |
| Rank 3 | CH3 (IN3) | PA6 | 备用电压 | 24.5 cycles |
| Rank 4 | CH4 (IN4) | PA7 | 备用电压 | 24.5 cycles |

**DMA 缓冲区:** `uint16_t adc2_voltage_buffer[4]`

### ADC4 — 注入组 (差分电流采样)

| 参数 | 值 |
|------|-----|
| 时钟 | SYSCLK/4 = 40MHz |
| 分辨率 | 12-bit, 右对齐 |
| 触发源 | HRTIM_TRG2, 上升沿 |
| 注入通道数 | 1 |

| 注入次序 | 通道 | 引脚 | 信号 | 差分对 |
|----------|------|------|------|--------|
| Rank 1 | CH4 (IN4) | PB14 | il3_p | IN4-IN5 (PB14-PB15) |

**中断:** ADC4_IRQn, 优先级 0,0

### 触发时序

```
HRTIM Master Period (50kHz)
  └→ HRTIM_TRG2 ─→ ADC1 注入启动 + ADC4 注入启动 (同步)
                   ADC1 中断 (HAL_ADCEx_InjectedConvCpltCallback)
                   ADC4 中断 (同回调)
```

- ADC1+ADC4 注入触发频率 = **25kHz** (HRTIM_TRG2 postscaler=1, ÷2)
- ADC2 DMA 连续循环, 不受触发控制

---

## HRTIM 配置

### 全局

| 参数 | 值 |
|------|-----|
| 系统时钟 | 160MHz (f_HRTIM = 160MHz×2 = 320MHz, 复用16×) |
| Period | 51200 |
| PWM 频率 | 160M×16 / 51200 = **50kHz** |
| 计数模式 | 中心对齐 (Up-Down) |
| DLL 校准 | Rate 3, 超时 500ms |

### Master Timer

| 参数 | 值 |
|------|-----|
| CMP1 值 | 25600 (50% 周期) |
| ADC Trigger 2 | Master Period, postscaler=1 |

### Timer A / B (Leg A, B)

| 参数 | A | B |
|------|---|---|
| CMP1 初始值 | 25600 | 25600 |
| 更新触发 | Timer A | Timer A |
| 复位触发 | Master Period | Master Period |
| 死区上升沿 | 50 | 50 |
| 死区下降沿 | 50 | 50 |
| 死区预分频 | DIV1 | DIV1 |

### Timer F (Leg F)

| 参数 | 值 |
|------|-----|
| CMP1 初始值 | 25600 |
| 更新触发 | Timer A |
| 复位触发 | Master Period |
| 死区上升沿 | **80** |
| 死区下降沿 | **80** |
| 死区预分频 | DIV1 |

### 输出配置

| 输出 | 引脚 | Set 源 | Reset 源 | 空闲电平 |
|------|------|---------|----------|----------|
| TA1 | PA8 | Master Period | Timer CMP1 | Inactive |
| TA2 | PA9 | NONE | NONE | Inactive |
| TB1 | PA10 | Master Period | Timer CMP1 | Inactive |
| TB2 | PA11 | NONE | NONE | Inactive |
| TF1 | PC6 | Master Period | Timer CMP1 | Inactive |
| TF2 | PC7 | NONE | NONE | Inactive |

> **注意:** TA2/TB2/TF2 的 Set/Reset 源均为 NONE，由软件通过 CMP1 控制。实际上 TA2/TB2/TF2 的互补输出由 HRTIM 硬件死区自动生成。

---

## DMA 通道分配

| DMA | 通道 | 请求源 | 方向 | 数据宽度 | 模式 | 用途 |
|-----|------|--------|------|----------|------|------|
| DMA1 | Ch1 | ADC2 | P→M | 16-bit | 循环 | ADC2 电压 DMA |
| DMA2 | Ch1 | USART1_TX | M→P | 8-bit | 普通 | VOFA 遥测 |
| DMA2 | Ch2 | I2C3_TX | M→P | 8-bit | 普通 | OLED (未用) |
| DMA2 | Ch4 | I2C3_RX | P→M | 8-bit | 普通 | OLED (未用) |

---

## 中断优先级

| IRQn | 优先级 (preempt,sub) | 用途 |
|------|----------------------|------|
| ADC1_2_IRQn | **0,0** | ADC1注入回调 + ADC2 |
| ADC4_IRQn | **0,0** | ADC4注入回调 |
| DMA1_Ch1_IRQn | 0,0 | ADC2 DMA |
| I2C3_ER_IRQn | 0,0 | I2C3 错误 |
| DMA2_Ch1_IRQn | 0,0 | USART1 TX DMA |
| TIM6_DAC_IRQn | 1,0 | DAC1 |
| HRTIM1_Master_IRQn | **4,0** | 调度器 Tick (50kHz) |
| USART1_IRQn | 8,0 | USART1 TX 完成 |
| I2C3_EV_IRQn | 9,0 | I2C3 事件 |
| DMA2_Ch2_IRQn | 9,0 | I2C3 TX DMA |
| DMA2_Ch4_IRQn | 9,0 | I2C3 RX DMA |

> ADC 和调度器中断优先级都较高 (0-4)，I2C/USART 较低 (8-9)，确保控制环不被通信阻塞。

---

## USART1

| 参数 | 值 |
|------|-----|
| 波特率 | **500,000** (460800 实际) |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 模式 | TX only (无 RX) |
| 时钟源 | PCLK2 = 160MHz |
| FIFO | 禁用 |

---

## I2C3 (SSD1306 OLED)

| 参数 | 值 |
|------|-----|
| 地址 | 0x3C (7-bit) |
| 时序 | 0x10B30F25 |
| 快速模式+ | 启用 |
| 时钟源 | PCLK1 = 160MHz |

---

## DAC1

| 参数 | 值 |
|------|-----|
| 通道 | CH1 (PA4) |
| 触发 | 无 (软件写入) |
| 输出缓冲 | 启用 |

---

## 采样标定常数

```c
// 电压 (分压比 4.7k/200k)
VOLTAGE_GAIN  = 4.7f / 200.0f         = 0.0235
VOLTAGE_CONST = 3.3f / (0.0235 * 4095) ≈ 34.25 V/count

// 电流 (分流 0.02Ω, 运放增益 8.2×, 偏置 1.65V)
CURRENT_CONST = 3.3f / (0.02 * 8.2 * 2048) ≈ 9.82 A/count
// ADC 偏移 (1.65V 零电流): ≈ 2050 counts
```

---

## 新建项目时的硬件资源汇总

| 资源 | 可用数量 | 已用 | 备注 |
|------|----------|------|------|
| HRTIM 输出 | 6路 (3对互补) | 6路 | A/B/F 三相全桥 |
| ADC 差分通道 | 3对 | 3对 | ADC1×2 + ADC4×1 |
| ADC 单端通道 | 4路 | 4路 | ADC2 DMA 循环 |
| DMA 通道 | 4个 | 4个 | 1×ADC + 1×USART + 2×I2C |
| USART | 1路 | USART1 TX | 可加 RX |
| I2C | 1路 | I2C3 | SSD1306 OLED |
| DAC | 1路 | DAC1 CH1 | 调试输出 |
| 空闲 GPIO | ~18个 | — | 见上表 |
| TIM (通用定时器) | 全部空闲 | — | 可用作 PWM/编码器/捕获 |
| OPAMP | 全部空闲 | — | 内置运放 |
| COMP | 全部空闲 | — | 模拟比较器 |
