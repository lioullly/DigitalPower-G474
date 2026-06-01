# CLAUDE.md

## 项目概述

STM32G474 单相电力电子平台，同一套硬件支持逆变（DC→AC）和 PFC（AC→DC 图腾柱 Boost）两种模式。
Keil MDK-ARM 编译，STM32CubeMX 生成 HAL 初始化代码。

## 技术栈

- **MCU:** STM32G474xx (Cortex-M4F, FPU + DSP)
- **HAL:** STM32G4xx HAL Driver
- **DSP:** ARM CMSIS-DSP (`arm_sin_f32` / `arm_cos_f32` / `arm_rms_f32`)
- **工具链:** Keil MDK-ARM (`.uvprojx`)
- **生成器:** STM32CubeMX (`.ioc`)
- **显示:** SSD1306 OLED via I2C3（未编译）

## 编译

- 用 Keil 打开 `MDK-ARM/inverter.uvprojx` 编译下载
- CubeMX 重新生成后，**只在 `USER CODE BEGIN/END` 区间内保留用户代码**，其余会被覆盖

## 架构

```
HRTIM Master CMP1 @ 50kHz (20μs)
  └─ ISR → Scheduler_Tick()               // 递减 remaining_us

ADC1 Injected ISR (HRTIM TRG2 @ 10kHz)
  └─ 读 IL1 原始值 → 生成 g_sin_wt → DAC 输出 → 设 data_ready

ADC2 Regular + DMA (连续循环)
  └─ 4 通道电压 [Uab, Udc, Uac, Ubc]，后台自动刷新

main loop
  └─ Scheduler_Dispatch()
       ├─ Task_ADC_Fetch           @ 50kHz  // 读 ADC 值转 SI 单位
       ├─ Task_PLL_1P_Process      @ 1kHz   // 单相 PLL
       ├─ Task_Control_PFC         @ 10kHz  // PFC 图腾柱控制
       ├─ Task_Protect_1P          @ 10kHz  // 软硬件保护
       ├─ Task_Button_Scan         @ 100Hz  // 按键 + LED
       └─ Task_VOFA_1P             @ 200Hz  // JustFloat 遥测
```

### 当前模式：PFC（图腾柱 Boost）

- **外环:** PI 电压环，每 20ms 更新电流幅值 i_mag，目标 UREF=40V DC
- **内环:** PR 电流环，频率实时跟踪 PLL 输出的 g_freq_est
- **参考:** `i_ref = i_mag × sin(g_wt × 2π)`，PLL 锁定后的纯净正弦
- **调制:** 图腾柱单腿切换——正半周 leg A 开关、负半周 leg B 开关，杜绝直通
- **安全:** PLL 未锁定时不输出、母线过压立即关断、软启动缓升

### 逆变模式（`Task_Control_PI_PR_Loop`，当前未注册）

- **外环:** PI 电压环控 Uab RMS
- **内环:** PR 电流环，参考 `i_mag × 1.414 × g_sin_wt`
- **调制:** 双极性互补 PWM，腿 A/B 对称

两种模式通过 `user_tasks.c` 中注册哪个控制任务来切换。

## 关键外设配置

| 外设 | 用途 | 关键参数 |
|------|------|---------|
| HRTIM1 | 3 相 PWM + 调度器时基 | Period=51200, 中心对齐增减, 死区 30 ticks, Master CMP1 触发调度器 |
| ADC1 (Injected) | IL1 电流采样 | 差分模式, 64x 过采样, RightShift 2, HRTIM_TRG2 触发 @ 10kHz |
| ADC2 (Regular+DMA) | 4 路电压采样 | 单端扫描, DMA 循环缓冲区 4×uint16 |
| ADC4 (Injected) | IL3 电流（未用） | 差分模式, 裸板会卡死 |
| USART1 | VOFA 遥测 | DMA 发送, 460800 baud, JustFloat 二进制协议 |
| DAC1 | 调试波形输出 | 输出 g_sin_wt 到 PA4 |

## ADC 采样流程

1. HRTIM Master CMP1 事件产生 HRTIM_TRG2 → 触发 ADC1/ADC4 注入组
2. `HAL_ADCEx_InjectedConvCpltCallback()`:
   - 读 `g_il1 = ADC_Raw - 2050`（2050 为零电流偏置）
   - 用积分器累加相位 `g_wt`，生成 `g_sin_wt = arm_sin_f32(g_wt × 2π)`
   - 输出 DAC，置 `g_adc_data_ready = 1`
3. `Task_ADC_Fetch()`: 等 `g_adc_data_ready`，读 ADC2 DMA buffer 转 SI 单位
4. 每 200 个样本（一个 50Hz 周期）算一次 RMS

## U_line / I_line 索引

```c
U_line[0] = Uab   (ADC2 ch0, 有偏置 2036)
U_line[1] = Udc   (ADC2 ch1, 无偏置)
U_line[2] = Uac   (ADC2 ch2)
U_line[3] = Ubc   (ADC2 ch3)

I_line[0] = IL1   (ADC1 Injected, 电流传感器)
I_line[1] = IL2   (预留)
I_line[2] = IL3   (ADC4, 未用)
```

## 硬件参数 (hardware_def.h)

| 宏 | 值 | 含义 |
|----|-----|------|
| `VOLTAGE_GAIN` | 4.7/200 | 电压采样分压比 |
| `VOLTAGE_CONST` | 3.3/(GAIN×4095) | 电压 ADC→V 系数 |
| `UAB_VOLTAGE_CONST` | =VOLTAGE_CONST | Uab 同分压比 |
| `R` | 0.020Ω | 采样电阻 |
| `CURRENT_GAIN` | 8.2 | AMC1301 + 运放增益 |
| `CURRENT_CONST` | 3.3/(R×GAIN×2048) | 电流 ADC→A 系数（差分 ±2048） |
| `L` | 1mH | 滤波电感 |
| `Fsw` | 50Hz | 基波频率（非 PWM 频率） |
| `DC_OV` | 80V | DC 母线过压阈值 |
| `DC_UV` | 0V | DC 母线欠压阈值（0=禁用） |

| `IL1_OC` | 10A | 瞬时过流阈值 |
| `UREF` | 40V | PFC 母线电压参考 |
| `I_MAG_DEFAULT` | 1A | 默认电流幅值 |
| `I_MAG_MAX` | 5A | PI 输出电流上限 |

## 调度器

- **时基:** HRTIM Master CMP1 @ 50kHz → tick=20μs
- **Scheduler_AddTask(func, hz, repeat):** 第二个参数是频率(Hz)，内部 `period_us = 1e6/hz`
- **协作式:** 每个任务后 `tim_delay_us(2)` 让出 CPU 给 ISR
- **tim_delay_us():** 用 DWT 周期计数器（`SystemCoreClock=160MHz`）
- **ISR 内调用:** `Scheduler_Tick()` 在 `HRTIM1_Master_IRQHandler` 中，优先级 7

## NVIC 优先级（PreemptPriority Group 4，数字越小越高）

| 中断 | 优先级 | 说明 |
|------|--------|------|
| HRTIM1_Master | 7 | 调度器 tick，最高 |
| USART1 | 8 | 低于 HRTIM，不能抢占 tick |
| ADC1_2 | 0 | CubeMX 默认 |
| DMA 相关 | 0 | CubeMX 默认 |

## CubeMX 重新生成后检查清单

每次 `Device Configuration Tool` 重新生成代码后，以下设置会被覆盖：

1. **HRTIM PostScaler:** `hrtim.c` 里 `PostScaler=0` → 在 `user_Init()` 末尾加 `HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 4)`
2. **HRTIM Master IRQ:** CubeMX 不生成 `MDIER` → 加 `HRTIM1->sMasterRegs.MDIER |= HRTIM_MDIER_MCMP1IE`
3. **USART1 IRQ 优先级:** CubeMX 默认 0 → 改为 8（`HAL_NVIC_SetPriority(USART1_IRQn, 8, 0)`）
4. **USART1 RXNE 禁用:** RX 浮空 → `__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE)`
5. **DMA1_Channel1 ISR 禁用:** ADC2 DMA 不需要 ISR → `HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn)`
6. **ADC2 EOC 中断禁用:** `__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_EOC)`

## 文件结构

```
Core/                           — CubeMX 生成，只在 USER CODE BEGIN/END 内编辑
├── Src/main.c                  — 入口：HAL_Init → SystemClock_Config → MX_*_Init → user_Init → user_Loop
├── Src/stm32g4xx_it.c          — 中断向量：HRTIM Master ISR 调 Scheduler_Tick()
├── Src/hrtim.c                 — HRTIM 配置：Period=51200, ADC Trigger @ PERIOD
├── Src/adc.c                   — ADC1/2/4 配置
├── Src/usart.c                 — USART1 TX DMA 循环模式
├── Src/dma.c                   — DMA 通道配置
└── Inc/*.h                     — 对应头文件、main.h 引脚宏

user/                           — 用户代码，CubeMX 不触及
├── user.c / user.h             — 全局变量、user_Init、Task_Button_Scan
├── task_adc.c / task_adc.h     — ADC 硬件初始化、Task_ADC_Fetch、ADC ISR 回调
├── user_tasks.c                — 调度器注册（当前 PFC 模式）
├── hardware_def.h              — 所有可调参数集中定义
├── pi_pr_ctrl.c / .h           — PR 控制器（双二阶带通）、PI 控制器、积分器、Clarke/Park 变换
├── timebase_scheduler.c / .h   — μs 级协作式调度器 + tim_delay_us(DWT)

├── vofa.c / .h                 — VOFA JustFloat 遥测（Task_VOFA_1P/3P/Wave）
├── terminal.c / .h             — 串口调试终端
├── singlephase/
│   ├── task_control_pfc.c / .h — PFC 图腾柱控制（当前使用）
│   ├── task_control_1p.c / .h  — 逆变 PI+PR 双环控制（备用）
│   ├── task_pll_1p.c / .h      — 单相 PLL（延时法虚拟正交）
│   ├── task_pwm_1p.c / .h      — HRTIM 比较值更新
│   ├── task_protect_1p.c / .h  — 过压/欠压/过流/AC过压 保护
│   └── task_display_1p.c / .h  — OLED 显示（未编译）
└── threephase/                 — 三相扩展（未编译）
```

## PLL (task_pll_1p.c)

- **正交生成:** 5 点延时缓冲 → 1kHz × 5 = 5ms = T/4 @ 50Hz → 真正 90°
- **鉴相:** `v_q = (v_beta × cos - v_alpha × sin) / v_mag`，PI 控 v_q→0
- **输出:** `g_wt ∈ [0, 1)`（归一化相位）、`g_freq_est`（频率估计 Hz）
- **锁定:** v_mag > 10V 持续 200 个周期后 `g_pll_locked = 1`
- **参数:** `PLL_Kp=60`, `PLL_Ki=2`, `PLL_Ts=0.001`
- **绝对不用 SOGI**

## 5 类保护 (task_protect_1p.c)

| 故障码 | 检测条件 | 去抖 |
|--------|---------|------|
| `FAULT_OV` | `U_line[1]`(Udc) > `DC_OV` | 10 次连续 |
| `FAULT_UV` | `U_line[1]`(Udc) < `DC_UV` | 10 次连续 |
| `FAULT_OC` | `|I_line[0]|` > `IL1_OC` | 10 次连续 |
触发任一保护 → `Run_Flag = 0` → 封锁 PWM 输出。

## 编码规范

- **语言:** C（非 C++）
- **编码:** GBK，注释用中文
- **命名:** 函数 CamelCase（`f32_PR_Calculate`），变量 snake_case（`g_pll_locked`），结构体 `_TypeDef` 后缀
- **CubeMX 文件:** 只在 `USER CODE BEGIN/END` 区间内编辑
- **用户文件:** 放在 `user/` 目录，Keil 手动添加，CubeMX 不管理
- **硬编码:** 禁止——所有参数进 `hardware_def.h`
- **浮点:** 用 `float` 不用 `double`，三角函数用 CMSIS-DSP

## 行为准则

1. **先想再写** — 不确定就问，有多种理解就列出来
2. **最小改动** — 不改不相关的代码、注释、格式
3. **简单优先** — 不引入未要求的抽象、灵活性、错误处理
4. **安全第一** — PFC 涉及高压，改控制代码前想清楚会不会炸
