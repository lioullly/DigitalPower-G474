# CLAUDE.md

## Project Overview

STM32G474xx 单相逆变器，CMake + GCC ARM 工具链。支持离网（开环调试 / PI+PR 双环）和并网（PLL+PR 电流环）三种模式，
在 `user_tasks.c` 注释切换。

## Tech Stack

- **MCU:** STM32G474xx (Cortex-M4F, FPU + DSP)
- **HAL:** STM32G4xx HAL Driver
- **DSP:** ARM CMSIS-DSP (arm_math.h, arm_sin_f32 / arm_cos_f32 / arm_rms_f32)
- **Build:** CMake + arm-none-eabi-gcc
- **Display:** SSD1306 OLED via I2C3 (optional)

## Build

```
cmake --preset default
cmake --build build
```

## Architecture

```
TIM6 (1MHz / 1μs)
  └─ ISR → Scheduler_Tick()                // 递减计数器，极轻量

ADC1/ADC4 ISR (HRTIM TRG2 @ 50kHz)
  └─ 读原始值 → 转 SI 单位 → 设 data_ready    // 不跑控制计算

main loop
  └─ Scheduler_Dispatch()
       ├─ Task_Control_*     @10kHz (period=100us)
       ├─ Task_Button_Scan   @100Hz
       └─ Task_VOFA          @50Hz

每任务后 tim_delay_us(2) 让出 CPU 给 ISR。
```

### 三种控制任务（编译时三选一）

| 任务 | 内容 | 用途 |
|------|------|------|
| `Task_Control_Debug` | 固定 m=0.8 + 软起动 | 开环调试 |
| `Task_Control_OffGrid` | PI 电压环(AC RMS) + PR 电流环 | 离网双环 |
| `Task_Control_GridTied` | PLL + PR 电流环 | 并网电流源 |

### 控制参数 (hardware_def.h)

| 宏 | 值 | 含义 |
|----|-----|------|
| `UREF` | 32.0f | AC 输出 RMS 电压参考 (V) |
| `I_MAG_DEFAULT` | 0.5f | 默认电流幅值 (A) |
| `OFFGRID_MOD_INDEX` | 0.8f | 离网调试固定调制比 |
| `UDC_OV` / `UDC_UV` | 60 / 10 | DC 母线过/欠压阈值 (V) |
| `IL1_OC` | 5.0f | 瞬时过流阈值 (A) |
| `UAC_OV_RATIO` | 1.5f | AC 过压: \|Uab\| > UREF × 1.5 |

### 调度器时基

TIM6 @ 1MHz (PSC=15, ARR=9)。所有 `_ms` 已改为 `_us`。
TASK_YIELD_US = 2（任务间让出 2μs）。

## File Map

```
user/
├── user.c / user.h           — 全局变量、三个控制任务、Button_Scan、ADC ISR、TIM6 ISR
├── user_tasks.c              — 调度器注册（控制任务三选一）
├── hardware_def.h            — 所有可调参数（保护阈值、参考值、系数）
├── pi_pr_ctrl.c / .h         — PI/PR 控制器、Clarke/Park 变换、ABC↔DQ
├── timebase_scheduler.c / .h — μs 级协作式调度器 + tim_delay_us
├── task.c / task.h           — Task 结构体定义
├── terminal.c / .h           — 串口调试终端
├── vofa.c / .h               — VOFA 遥测输出
├── singlephase/
│   ├── task_control_1p.c / .h  — PR 电流环 + PI 电压环函数
│   ├── task_pll_1p.c / .h      — 单相 PLL（延时法虚拟 β）
│   ├── task_pwm_1p.c / .h      — HRTIM 比较值更新
│   ├── task_protect_1p.c / .h  — 过压/欠压/过流/AC过压保护
│   └── task_display_1p.c / .h  — OLED 显示（未编译）
└── threephase/               — 三相扩展（未编译，extern 声明已同步）
Core/                         — CubeMX 生成，只在 USER CODE BEGIN/END 内编辑
Hardware/ssd1306/             — OLED 驱动
```

## Key Design Decisions

- **控制不入 ISR**：ADC 回调只读原始值，控制计算在调度器 Task 里跑
- **PI 控 AC RMS**：用 `arm_rms_f32` 算滑窗 RMS（200 样本 = 一个 50Hz 周期），PI 控 AC 输出电压
- **前馈解耦**：PR 输出 + Uab 前馈，再除以 Udc 得到调制比
- **保护不收 ADC 锁**：`Task_Protect_1P` 在控制任务入口每次都跑，不依赖 `g_adc_data_ready`
- **PLL 延时 5 点**：1kHz PLL 频率 × 5 = 5ms = T/4 @ 50Hz = 真正 90° 正交
- **I_mag 为 extern**：全局变量，PI 和 PR 共享，离网时 PI 写、PR 读

## Coding Conventions

- 只在 `USER CODE BEGIN/END` 区间编辑 CubeMX 生成的文件
- `hardware_def.h` 集中所有可调参数，不散落硬编码值
- 新增变量/函数注意同步更新 `threephase/` 下的 `extern` 声明
- GBK 编码，注释用中文
