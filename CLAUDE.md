# CLAUDE.md

## 项目概述

STM32G474 单相电力电子平台，同一套硬件支持 PFC 整流（AC→DC 图腾柱 Boost）和并网逆变（DC→AC）两种模式。
CMake + Ninja + arm-none-eabi-gcc 编译，STM32CubeMX 生成 HAL 初始化代码。

## 技术栈

- **MCU:** STM32G474xx (Cortex-M4F, FPU + DSP)
- **HAL:** STM32G4xx HAL Driver
- **DSP:** ARM CMSIS-DSP (`arm_sin_f32` / `arm_cos_f32` / `arm_rms_f32` / `arm_sin_q15` / `arm_cos_q15`)
- **工具链:** CMake + Ninja + arm-none-eabi-gcc 13.3
- **生成器:** STM32CubeMX (`.ioc`)
- **显示:** SSD1306 OLED via I2C3（未编译进固件）
- **遥测:** VOFA JustFloat 二进制协议 via USART1 DMA @ 460800 baud

## 编译

```bash
cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug ..
ninja
```

- 新增 `.c` 文件后需删 `CMakeCache.txt` 重跑 cmake（CMakeLists.txt 用 `GLOB` 收集源文件，不自动感知新文件）
- CubeMX 重新生成后，**只在 `USER CODE BEGIN/END` 区间内保留用户代码**，其余会被覆盖
- 用户代码放在 `user/` 目录，CubeMX 不管理

## 架构

```
HRTIM Master CMP1 @ 50kHz (20μs)
  └─ ISR → Scheduler_Tick()               // 递减调度器 remaining_us

ADC1 Injected ISR (HRTIM TRG2 @ 10kHz)
  └─ 读 IL1 原始值 → 积分器更新相位 → arm_sin_f32 生成 g_sin_wt → DAC 输出 → 置 data_ready

ADC2 Regular + DMA (连续循环)
  └─ 4 通道电压 [Uab, Udc, Uac, Ubc]，后台自动刷新，无 ISR

main loop
  └─ Scheduler_Dispatch()
       ├─ Task_ADC_Fetch           @ 10kHz  // 读 ADC 值转 SI 单位 + 200点滑动 RMS
       ├─ Task_Protect_1P          @ 10kHz  // 过压/欠压/过流保护
       ├─ Task_Control_PFC         @ 10kHz  // PFC 图腾柱控制（当前激活）
       ├─ Task_Button_Scan         @ 100Hz  // 按键 + LED + 相位调节
       └─ Task_VOFA_1P             @ 5kHz   // JustFloat 遥测 (8通道)
```

### 当前模式：PFC（图腾柱 Boost）— task_control_pfc.c

- **激活逻辑:** 带迟滞的电压判断——启动需 `g_uab_rms > 5V && Udc > Uab_rms×1.2`，保持需 `g_uab_rms > 3V && Udc > Uab_rms×0.8`
- **电压外环:** PI 控制器，10:1 降采样（10kHz→1kHz），目标 `PFC_UREF=40V`，输出限幅 `[0.05, PFC_IREF_MAX]`
- **电流内环:** PR 控制器（双二阶带通），`Kp=4, Kr=10, f0=50Hz, BW=10Hz, TH=±10`
- **参考生成:** Hilbert 变换产生正交 α-β 对 → `cos(φ)·v_α/v_mag + sin(φ)·v_β/v_mag` 移相 → × iref×1.414 得峰值参考
- **调制:** 双极性 SPWM（`_pwm_bipolar`），`m = (v_α - v_ctrl) / Udc`，中心对齐互补输出
- **软启动:** iref 从 0.05A 起步，电压环逐步提升

### 并网逆变模式 — task_control_grid.c

- **激活逻辑:** 电网 RMS 检测——启动需 `g_uab_rms > 20V`，保持需 `>16V`
- **同步:** Hilbert 变换 → `atan2f(v_β, v_α)` 得瞬时相位 → 差分得频率 → 一阶 LPF 滤波得 `g_freq_est`
- **功率控制:** `i_ref = i_mag × [v_α/v_mag + tan(φ)×v_β/v_mag]`，φ 为功角（0°=纯有功）
- **电流内环:** PR 控制器，`Kp=0.5, Kr=5, TH=±60`
- **电流方向:** `i_fb = -I_line[0]`（逆变与 PFC 方向相反）
- **调制:** 双极性 SPWM via `g_duty_a/b` + `Task_PWM_1P_Update()`
- 当前未在调度器中注册（PFC 模式激活中）

### 离网逆变模式 — task_control_1p.c（备用）

- 状态机：STOP → STARTUP → OFF_GRID / GRID_TIED
- 离网：PI 电压外环控 `OFFGRID_UREF=20V` RMS + PR 电流内环
- 并网：PR 电流环单独运行（旧架构，无 Hilbert）
- 当前未在调度器中注册

## 希尔伯特变换 (pi_pr_ctrl.c)

一阶全通滤波器，在 `f0=50Hz` 处产生精确 90° 相移，输出正交 α-β 对：
```c
Hilbert_TypeDef hilbert;
f32_Hilbert_Init(&hilbert, 50.0f, 10000.0f);  // f0=50Hz, Fs=10kHz
f32_Hilbert_Calculate(&hilbert, input, &alpha, &beta);
// alpha = input (同相), beta = Hilbert(input) (滞后90°)
```
- 系数: `k = (1-tan(π·f0/Fs)) / (1+tan(π·f0/Fs))`，用 `arm_sin_f32/arm_cos_f32` 计算 tan
- PFC 和 Grid 任务各自初始化自己的 Hilbert 实例

## U_line / I_line 索引

```c
U_line[0] = Uab   (ADC2 ch0, 偏置 2036, × UAB_VOLTAGE_CONST)
U_line[1] = Udc   (ADC2 ch1, 无偏置, × VOLTAGE_CONST)
U_line[2] = Uac   (ADC2 ch2, 未使用)
U_line[3] = Ubc   (ADC2 ch3, 未使用)

I_line[0] = IL1   (ADC1 Injected, 差分电流传感器, 偏置 2050)
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
| `K_CONST` | 2π×Fsw×L | 电感阻抗 |
| `DC_OV` | 60V | DC 母线过压阈值 |
| `DC_UV` | 0V | DC 母线欠压阈值（0=禁用） |
| `IL1_OC` | 9A | 瞬时过流阈值 |

| `OFFGRID_UREF` | 20V | 离网 AC 输出 RMS 电压参考 |
| `I_MAG_DEFAULT` | 1A | 默认电流幅值 |
| `I_MAG_MAX` | 10A | PI 输出电流上限 |

| `PFC_UREF` | 40V | PFC 母线电压参考 |
| `PFC_IREF_MAX` | 5.66A | PFC 最大 RMS 电流（peak≈8A） |
| `PFC_PHASE_DEG_DEFAULT` | 0° | PFC 初始移相角 |

| `GRID_PHI_DEG_DEFAULT` | 0° | 并网初始功角（0=纯有功） |
| `GRID_I_MAG_DEFAULT` | 1A | 并网初始有功峰值电流 |
| `GRID_IREF_MAX` | 5A | 并网最大峰值电流 |

## 关键外设配置

| 外设 | 用途 | 关键参数 |
|------|------|---------|
| HRTIM1 | 单相 PWM + 调度器时基 | Period=51200, 中心对齐增减, 死区 30 ticks, Master CMP1 触发调度器 |
| ADC1 (Injected) | IL1 电流采样 | 差分模式, 64x 过采样, RightShift 2, HRTIM_TRG2 触发 @ 10kHz |
| ADC2 (Regular+DMA) | 4 路电压采样 | 单端扫描, DMA 循环缓冲区 4×uint16, 无 DMA ISR |
| ADC4 (Injected) | IL3 电流（已禁用） | 差分模式, 裸板会卡死调度器 |
| USART1 | VOFA 遥测 | DMA 发送 (Normal 模式, ping-pong), 460800 baud, JustFloat 二进制协议 |
| DAC1 | 调试波形输出 | 输出 `g_sin_wt×2047+2048` 到 PA4 |

## ADC 采样流程

1. HRTIM Master CMP1 事件产生 HRTIM_TRG2 → 触发 ADC1/ADC4 注入组
2. `HAL_ADCEx_InjectedConvCpltCallback()`:
   - 读 `g_il1 = ADC_Raw - 2050`（2050 为零电流偏置）
   - 积分器累加相位，`g_sin_wt = arm_sin_f32(phase × 2π)`
   - 输出 DAC：`g_sin_wt × 2047 + 2048`
   - 置 `g_adc_data_ready = 1`
3. `Task_ADC_Fetch()`: 等 `g_adc_data_ready`，读 ADC2 DMA buffer 转 SI 单位
4. 每 200 个样本（一个 50Hz 周期）用 `arm_rms_f32` 算一次 RMS

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

## 按键与 LED

| GPIO | 功能 |
|------|------|
| PC5 (user_Pin) | Run_Flag 切换（短按启停），故障时清除故障码 |
| PB13 (KEY1_Pin) | 相位减：短按 `-ph_step°`，长按 (>500ms) 切换粗调 1°/细调 0.1° |
| PC0 (KEY2_Pin) | 相位加：短按 `+ph_step°`，长按切换步长 |
| Green LED | 心跳闪烁 @ 50Hz (Task_Button_Scan 内翻转) |
| Red LED | 运行指示 (激活时亮) |

- KEY1/KEY2 同时修改 `g_pfc_phase_deg` 和 `g_grid_phi_deg`（两个任务互斥，只有激活的那个生效）
- 相位值自动 wrap 到 [0, 360)

## 保护 (task_protect.c)

| 故障码 | 检测条件 | 去抖 |
|--------|---------|------|
| `FAULT_OV` | `Udc > DC_OV` (60V) | 10 次连续 |
| `FAULT_UV` | `Udc < DC_UV` (0V, 禁用) | 10 次连续 |
| `FAULT_OC` | `|I_line[0]| > IL1_OC` (9A) | 10 次连续 |

- 启动后 200ms 内跳过 OC 检测（母线电容充电 blanking）
- Run_Flag 上升沿自动清除故障码
- 触发任一保护 → `Run_Flag = 0` → 控制任务封锁 PWM 输出

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
├── Src/usart.c                 — USART1 配置
├── Src/dma.c                   — DMA 通道配置
└── Inc/*.h                     — 对应头文件、main.h 引脚宏

user/                           — 用户代码，CubeMX 不触及
├── user.c / user.h             — 全局变量、user_Init、Task_Button_Scan
├── task_adc.c / task_adc.h     — ADC 硬件初始化、Task_ADC_Fetch、ADC ISR 回调
├── user_tasks.c / .h           — 调度器注册（当前 PFC 模式）
├── hardware_def.h              — 所有可调参数集中定义
├── pi_pr_ctrl.c / .h           — PR/PI 控制器、积分器、Clarke/Park 变换、Hilbert 变换
├── timebase_scheduler.c / .h   — μs 级协作式调度器 + tim_delay_us(DWT)
├── vofa.c / .h                 — VOFA JustFloat 遥测（Task_VOFA_1P/3P，ping-pong DMA）
├── terminal.c / .h             — 串口调试终端
├── singlephase/
│   ├── task_control_pfc.c / .h — PFC 图腾柱控制（当前使用）
│   ├── task_control_grid.c / .h— 并网逆变器（Hilbert 同步 + 有功/无功）
│   ├── task_control_1p.c / .h  — 离网/并网逆变 PI+PR 双环（备用，旧架构）
│   ├── task_pwm_1p.c / .h      — HRTIM 比较值更新（双极性/单极性/倍频）
│   ├── task_protect.c / .h  — 过压/欠压/过流保护
│   └── task_display_1p.c / .h  — OLED 显示（未编译）
└── threephase/                 — 三相扩展（未编译）
```

## 编码规范

- **语言:** C（非 C++）
- **编码:** GBK，注释用中文
- **命名:** 函数 CamelCase（`f32_PR_Calculate`），变量 snake_case（`g_pfc_phase_deg`），结构体 `_TypeDef` 后缀
- **CubeMX 文件:** 只在 `USER CODE BEGIN/END` 区间内编辑
- **用户文件:** 放在 `user/` 目录，CubeMX 不管理
- **硬编码:** 禁止——所有可调参数进 `hardware_def.h`
- **浮点:** 用 `float` 不用 `double`，三角函数用 CMSIS-DSP（`arm_sin_f32` 等），禁止 `tanf`（会链接 libm 增大 4KB）

## 行为准则

1. **先想再写** — 不确定就问，有多种理解就列出来
2. **最小改动** — 不改不相关的代码、注释、格式
3. **简单优先** — 不引入未要求的抽象、灵活性、错误处理
4. **安全第一** — PFC 涉及高压，改控制代码前想清楚会不会炸管
