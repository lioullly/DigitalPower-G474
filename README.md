# DigitalPower-G474

STM32G474 单相电力电子数字电源平台，同一套硬件实现三种模式。

## 硬件

| 参数 | 值 |
|---|---|
| MCU | STM32G474RBT6 (Cortex-M4F 170MHz, FPU + DSP) |
| 拓扑 | 单相全桥 + LC 滤波 |
| 开关频率 | 50kHz (中心对齐双极性 SPWM) |
| 死区 | HRTIM 硬件死区 |
| 滤波电感 | 1mH |
| 电流采样 | 0.02Ω 分流 + 8.2× 运放 → ADC1/4 差分注入 |
| 电压采样 | 4.7/200 分压 → ADC2 规则通道 DMA |
| DC 母线耐压 | 80V |
| 显示 | SSD1306 OLED (I2C3, 可选) |
| 调试 | USART1 VOFA 上位机 @ 460800bps |

## 三种工作模式

| 模式 | 源文件 | 说明 |
|---|---|---|
| **离网逆变** | `task_control_1p.c` | DC→AC 独立供电，电压 PI 外环 + PR 电流内环 + 电压前馈 |
| **并网逆变** | `task_control_grid.c` | DC→AC 馈网，Hilbert 锁相 + PR 电流环，支持有功/无功调节 |
| **图腾柱 PFC** | `task_control_pfc.c` | AC→DC 整流升压，电压 PI 外环 + Hilbert 锁相 + PR 电流内环 |

模式切换：修改 `user/user_tasks.c` 中 `g_control_isr` 的函数指针。

## 控制架构

```
ADC ISR @ 25kHz (HRTIM_TRG2)
  ├── 电流/电压采样 + SI 转换
  ├── 相位积分器 → sin(wt)
  ├── 滑动窗口 RMS (1000 点, 40ms)
  ├── 保护检测 (OV/OC)
  ├── 控制环路 (PR 电流环 25kHz + PI 电压环 200Hz)
  └── VOFA 遥测捕获

主循环 (后台)
  ├── Scheduler_Dispatch (Tick 50kHz via HRTIM Master IRQ)
  ├── Task_Button_Scan (100Hz)
  └── Task_Display (20Hz)
```

## 编译

```bash
# 配置
cmake -B build/Debug -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug

# 编译
cmake --build build/Debug
```

**注意：** `CMakeLists.txt` 用 `GLOB` 收集 `user/` 源文件。新增 `.c` 后需删 `build/CMakeCache.txt` 重跑 cmake。

## VOFA 遥测 (Ch0–Ch9)

| 通道 | 数据 | 单位 |
|---|---|---|
| Ch0 | I_line 瞬时电流 | A |
| Ch1 | g_irms RMS 电流 | A |
| Ch2 | Uab 瞬时电压 | V |
| Ch3 | g_uab_rms RMS 电压 | V |
| Ch4 | Udc 母线电压 | V |
| Ch5 | g_fault_code 故障码 | — |
| Ch6 | g_dbg_vctrl PR 输出 | V |
| Ch7 | g_dbg_err 电流误差 | A |
| Ch8 | g_cpu_usage CPU 占用 | % |
| Ch9 | g_isr_us ISR 耗时 | μs |

VOFA 上位机选择 JustFloat 协议，44 字节/帧。

## 按键操作

| 按键 | 功能 |
|---|---|
| KEY1 | 相位角 - (短按细调 0.1°, 长按切换粗调 1°) |
| KEY2 | 相位角 + (同上) |
| USER (PC5) | Run_Flag 启停 / 长按清除故障 |

## 目录结构

```
├── Core/           CubeMX 生成 HAL 代码
├── Drivers/        CMSIS + HAL 库
├── Hardware/       SSD1306 OLED 驱动
├── user/           用户代码
│   ├── singlephase/  控制任务 (离网/并网/PFC/保护/PWM)
│   ├── vofa.c        VOFA 遥测
│   ├── timebase_scheduler.c  协作式调度器
│   └── hardware_def.h        硬件参数配置
├── cmake/          工具链 + CubeMX CMake 生成
└── inverter.ioc    CubeMX 项目文件
```

## 关键参数 (hardware_def.h)

| 参数 | 值 | 说明 |
|---|---|---|
| DC_OV | 80V | DC 母线过压保护 |
| IL1_OC | 9A | 过流瞬时保护 |
| PR_CTRL_CLAMP | 20V | PR 输出限幅 |
| OFFGRID_UREF | 32V RMS | 离网输出电压 |
| PFC_UREF | 40V | PFC 母线电压目标 |
| GRID_I_MAG_DEFAULT | 1A RMS | 并网初始电流 |
