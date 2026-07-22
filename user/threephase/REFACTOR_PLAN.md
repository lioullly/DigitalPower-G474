# 三相代码重构技术方案

## 1. 现状分析

### 1.1 当前三相代码问题清单

对比单相 (PFC/Grid) 和 Buck DCDC 的成熟架构，现有三相代码存在以下结构性缺陷：

| # | 问题 | 严重度 | 说明 |
|---|------|--------|------|
| 1 | **未接入 hook 体系** | 致命 | 有自己的 `UserTasks_Init()`，不走 `g_control_isr`/`g_adc_preproc`/`g_vofa_fn`/`g_display_fn` 函数指针，与主调度器割裂 |
| 2 | **保护完全缺失** | 致命 | 不调用 `Task_Protect_Run()`，不设置 `g_protect_mask`，过流/过压无保护 |
| 3 | **调度器架构不同** | 严重 | 用旧架构 `Scheduler_Init(1)` + `Scheduler_AddTask` 把控制放在 main loop，单相已经改为 ADC ISR @ 10kHz 零抖动 |
| 4 | **U_line 索引不一致** | 严重 | 三相 `U_line[0]=Udc, U_line[1]=Uab`，单相 `U_line[0]=Uab, U_line[1]=Udc`，共用 `task_protect.c` 会误判 |
| 5 | **无 VOFA 遥测** | 严重 | 完全没有 `vofa_capture_3p()`，调试无法看波形 |
| 6 | **PLL 使用全局变量** | 中等 | `g_wt`, `g_pll_locked` 是全局变量，而单相 PFC/Grid 的 Hilbert 是 `static` 局部变量 |
| 7 | **PR 控制器 extern 悬空** | 中等 | `extern PR_TypeDef Current_PR_Loop_alpha` 依赖外部初始化，违反封装原则 |
| 8 | **无软启动** | 中等 | 无电流/电压 ramp，直接跳变到目标值 |
| 9 | **无激活迟滞** | 中等 | 单相用带迟滞的电压判断（启动/保持阈值不同），三相 PLL 仅简单 `v_mag < 10` |
| 10 | **继电器在 PLL 中控制** | 中等 | 单相模式在控制任务的 rising edge 控制继电器，三相在 `Task_PLL_Process()` 内 |
| 11 | **PWM 无法停止输出** | 中等 | `Run_Flag=0` 时只跳过比较值更新，不调用 `HAL_HRTIM_WaveformOutputStop` |
| 12 | **hardware_def_3p.h 为空** | 低 | 所有参数硬编码在 `.c` 中 |
| 13 | **控制架构混用 α-β PR + PI 电压环** | 设计 | 当前是 α-β 静止坐标系 PR 电流环 + PI 电压外环，是合理选择；但缺 SVPWM，用 SPWM+中点注入替代 |

---

## 2. 单相/DCDC 成熟模式总结

### 2.1 函数指针 hook 体系

```
user_tasks.c → UserTasks_Init()
  ├─ g_adc_preproc  = adc_preproc_1p/buck/3p  ← ADC ISR @ 50kHz
  ├─ g_control_isr  = Task_Control_PFC/Grid/Buck ← ADC ISR @ 10kHz
  ├─ g_vofa_fn      = vofa_capture_1p/buck/3p   ← ADC ISR @ 10kHz
  └─ g_display_fn   = Task_Display_1P/Buck/3P    ← scheduler @ 5Hz
```

### 2.2 控制任务状态机模式

每个控制任务都遵循完全相同的结构：

```c
void Task_Control_XXX(void) {
    static uint8_t  prev_active = 0;
    static <controller_state...>;

    // === Phase 1: Run_Flag=0 停机 ===
    if (!Run_Flag) {
        stop_pwm();
        reset_peripherals();
        prev_active = 0;
        return;
    }

    // === Phase 2: 激活条件判断 (带迟滞) ===
    uint8_t active;
    if (prev_active)  active = <保持条件>;
    else              active = <启动条件>;
    uint8_t rising = (active && !prev_active);

    if (!active && prev_active)  shutdown_pwm();
    prev_active = active;
    if (!active) return;

    // === Phase 3: 上升沿一次性初始化 ===
    if (rising) {
        start_pwm();
        init_controllers();    // PI/PR/Hilbert/Notch
        set_protect_mask();    // g_protect_mask / g_protect_ac_mask
        set_hooks();           // g_display_fn / g_vofa_fn
        relay_on();
    }

    // === Phase 4: 正常控制循环 @ 10kHz ===
    <read_adc> → <control_law> → <update_pwm>
}
```

### 2.3 关键设计决策

- **PFC**: 电压外环 PI (1kHz 降采样) + 电流内环 PR + Hilbert 移相 + 100Hz 陷波
- **Grid**: 纯 PR 电流环 + Hilbert 电网同步 + 频率估算 (atan2f 相位差分 + LPF)
- **Buck**: 电压外环 PI + 电流内环 PI(P+I 分离) + 前馈
- **保护**: mask 驱动的公共保护框架 (`g_protect_mask` + `g_protect_ac_mask`)

---

## 3. 三相并网逆变器重构方案

### 3.1 文件结构 (重构后)

```
user/threephase/
├── hardware_def_3p.h       # 三相参数集中定义
├── adc_preproc_3p.c        # ADC 预处理 @ 50kHz (替代 task_adc.c)
├── task_control_3p.c       # 三相并网控制 @ 10kHz (替代 task_control.c)
├── task_control_3p.h
├── task_pwm_3p.c           # SVPWM 三相调制 (替代 task_pwm.c)
├── task_pwm_3p.h
├── vofa_3p.c               # VOFA 遥测
├── task_display_3p.c       # OLED 显示 (可选)
├── task_display_3p.h
└── REFACTOR_PLAN.md        # 本文档
```

删除的文件（合并/替代）:
- `task_pll.c` / `task_pll.h` → PLL 逻辑内嵌到 `task_control_3p.c`
- `task_adc.c` / `task_adc.h` → 替换为 `adc_preproc_3p.c`
- `user_tasks.c` / `user_tasks.h` → 不再需要，三相通过 hook 接入主 `user_tasks.c`

### 3.2 U_line / I_line 索引统一

**必须与单相保持一致**（`task_protect.c` 的 mask 定义依赖此映射）:

```c
// 三相 ADC 通道映射 (hardware_def_3p.h)
#define ADC2_BUF_UDC  0    // ADC2 IN1 → buffer[0] = 直流母线电压
#define ADC2_BUF_UAB  1    // ADC2 IN2 → buffer[1] = 线电压 Uab
#define ADC2_BUF_UAC  2    // ADC2 IN3 → buffer[2] = 线电压 Uac
#define ADC2_BUF_UBC  3    // ADC2 IN4 → buffer[3] = 线电压 Ubc

// U_line 索引 (与单相一致)
// U_line[0] = Uab (AC, 偏置 2036)
// U_line[1] = Udc (DC, 无偏置)
// U_line[2] = Uac (AC, 偏置 2036)
// U_line[3] = Ubc (AC, 偏置 2036)

// I_line 索引
// I_line[0] = IL1 (ADC1 injected)
// I_line[1] = IL2 (ADC4 injected, 预留)
// I_line[2] = IL3 (ADC4 injected, 预留)
```

### 3.3 控制架构

采用 **α-β 静止坐标系 PR 电流环 + SRF-PLL 锁相 + PI 电压外环**：

```
                    ┌──────────┐     ┌──────────┐
  Udc_ref ──→[PI]──→│ I_mag    │────→│ i_ref_α  │
                    │ × cos(wt)│     │ i_ref_β  │
  g_wt ─────────────→│ × sin(wt)│     └────┬─────┘
                    └──────────┘          │
                                          ↓
  Uab, Ubc ──→[Clarke]──→ v_g_α, v_g_β ──(+)──→[PR_α]──→ v_ref_α ──→[iClarke]──→[SVPWM]
                         i_α, i_β ←──[Clarke]── IL1,IL2,IL3
                                            ──→[PR_β]──→ v_ref_β

  Uab, Ubc ──→[Clarke]──→[SRF-PLL]──→ g_wt, freq_est, locked
```

**控制律:**
- **电压外环 (1kHz)**: PI 控制 Udc → I_mag (限幅 [0, I_MAX])
- **电流内环 (10kHz)**: PR 控制器 (α-β 各一个)，前馈电网电压
  - `i_ref_α = I_mag × cos(ωt + φ)`  (φ 为功角，0=纯有功)
  - `i_ref_β = I_mag × sin(ωt + φ)`
  - `v_ref_α = PR(i_err_α) + v_g_α`
  - `v_ref_β = PR(i_err_β) + v_g_β`
- **调制**: SVPWM (电压利用率比 SPWM 高 15.5%)

### 3.4 SRF-PLL 设计

内嵌到控制任务中，替代独立的 `task_pll.c`：

```c
// PLL 状态 (static 局部变量)
static float pll_vq_integ = 0.0f;
static float pll_freq_est = 50.0f;
static float pll_wt_accum = 0.0f;
static uint16_t pll_lock_cnt = 0;
static uint8_t pll_locked = 0;

// 参数宏 (hardware_def_3p.h)
#define PLL_KP        60.0f    // 锁相环比例增益
#define PLL_KI         2.0f    // 锁相环积分增益
#define PLL_LOCK_CNT  200      // 锁定计数器阈值 (200ms @ 1kHz)
#define PLL_VMAG_MIN   10.0f   // 最小电网电压幅值 [V]
```

**关键改进**: PLL 在电压外环降采样处运行 (1kHz)，而非 10kHz。三相 SRF-PLL 比 Hilbert PLL 更适合三相系统（天然 α-β 可用）。

### 3.5 激活条件 (带迟滞)

```c
// 并网启动条件 (hardware_def_3p.h)
#define GRID3P_VMAG_START  20.0f   // 电网电压幅值启动阈值 [V]
#define GRID3P_VMAG_HOLD   16.0f   // 电网电压幅值保持阈值 [V]
#define GRID3P_UDC_MIN     50.0f   // 母线电压最低启动 [V]

// 启动: v_mag > 20V && Udc > 50V && PLL_locked
// 保持: v_mag > 16V && Udc > 40V && PLL_locked
```

### 3.6 软启动设计

```c
// 软启动参数 (hardware_def_3p.h)
#define GRID3P_IRAMP_SLEW  0.0001f  // 电流 ramp 步长 [A/step] → 1A/s @ 10kHz
#define GRID3P_I_START      0.05f   // 初始电流幅值 [A]
#define GRID3P_I_MAX        5.0f    // 最大 RMS 电流 [A]

// 在 rising edge 后逐步 ramp:
if (i_mag < I_MAG_DEFAULT)
    i_mag += GRID3P_IRAMP_SLEW;
```

### 3.7 保护配置

```c
// Rising edge 时设置:
g_protect_mask    = PROT_IL1_OC | PROT_IL2_OC | PROT_IL3_OC   // 三相过流
                  | PROT_ADC2_R1_OV | PROT_ADC2_R3_OV | PROT_ADC2_R4_OV  // 交流过压
                  | PROT_ADC2_R2_OV | PROT_ADC2_R2_UV;  // 母线过压/欠压
g_protect_ac_mask = PROT_ADC2_R1_AC | PROT_ADC2_R3_AC | PROT_ADC2_R4_AC; // 三相交流
```

### 3.8 VOFA 通道定义

```c
// 10 通道 JustFloat:
// Ch0: I_line[0] (IL1 瞬时)
// Ch1: I_line[1] (IL2 瞬时)
// Ch2: I_line[2] (IL3 瞬时)
// Ch3: U_line[0] (Uab 瞬时)
// Ch4: U_line[1] (Udc 母线)
// Ch5: g_fault_code
// Ch6: pll_freq_est (电网频率)
// Ch7: g_wt (锁相角度, 0~1)
// Ch8: g_cpu_usage
// Ch9: g_isr_khz
```

### 3.9 主 user_tasks.c 接入

```c
// 在 UserTasks_Init() 中切换为三相模式:
g_adc_preproc = adc_preproc_3p;
g_display_fn  = Task_Display_3P;    // 或 NULL
g_vofa_fn     = vofa_capture_3p;
g_control_isr = Task_Control_3P_Grid;
```

---

## 4.实施步骤

### Phase 1: 基础设施 (1 文件)
1. **`hardware_def_3p.h`** — 填入所有三相参数宏

### Phase 2: ADC 预处理 (1 文件)
2. **`adc_preproc_3p.c`** — ADC 预处理 @ 50kHz:
   - 三相电压偏置校正 (Uab/Uac/Ubc: -2036, Udc: -0)
   - 三相电流从 `g_il1/g_il2/g_il3` 转 SI
   - 滑动窗口 RMS 计算 (Uab, IL1/2/3, 1000点 @ 50kHz)

### Phase 3: 控制核心 (1 文件)
3. **`task_control_3p.c`** — 三相并网逆变控制 @ 10kHz:
   - 完整状态机 (Run_Flag → active hysteresis → rising init → loop)
   - SRF-PLL 内嵌 (替代 task_pll.c)
   - 电压外环 PI @ 1kHz (10:1 降采样)
   - 电流内环 PR @ 10kHz (α-β 各一个)
   - 电网电压前馈
   - 软启动电流 ramp
   - RMS 计算 (arm_rms_f32)

### Phase 4: PWM 调制 (1 文件)
4. **`task_pwm_3p.c`** — SVPWM 三相调制:
   - 提供 `_pwm_svpwm(v_alpha, v_beta, udc)` 函数
   - 内部调用 `__HAL_HRTIM_SETCOMPARE` 更新 TA/TB/TF
   - HRTIM 启停封装

### Phase 5: VOFA + Display (2 文件)
5. **`vofa_3p.c`** — VOFA JustFloat 遥测
6. **`task_display_3p.c`** — OLED 显示 (可选)

### Phase 6: 集成
7. 修改根 `user_tasks.c` → `UserTasks_Init()` 添加三相注释块
8. 修改根 `user.h` → 添加 `adc_preproc_3p()`, `Task_Control_3P_Grid()`, `vofa_capture_3p()` 声明
9. 删除旧文件 (或标记 deprecated):
   - `threephase/task_adc.c/.h`
   - `threephase/task_pll.c/.h`
   - `threephase/task_pwm.c/.h` (替换)
   - `threephase/task_control.c/.h` (替换)
   - `threephase/user_tasks.c/.h` (不再需要)

---

## 5. 与单相/DCDC 的一致性检查

| 特性 | 单相 PFC | 单相 Grid | Buck DCDC | 三相 Grid (重构后) |
|------|----------|-----------|-----------|-------------------|
| hook 体系 | ✅ g_control_isr | ✅ | ✅ | ✅ |
| 状态机模式 | ✅ | ✅ | ✅ | ✅ |
| 激活迟滞 | ✅ | ✅ | ✅ | ✅ |
| 保护 mask | ✅ | ✅ | ✅ | ✅ (三相扩展) |
| 软启动 | ✅ (iref) | ✅ (i_mag) | ✅ (vref) | ✅ (i_mag) |
| VOFA | ✅ | ✅ | ✅ | ✅ |
| 控制器 | PI+PR+Hilbert | PR+Hilbert | PI+PI | PI+PR×2+SRF-PLL |
| 调制 | 双极性 SPWM | 双极性 SPWM | Buck PWM | SVPWM |
| 电网同步 | Hilbert(α-β) | Hilbert + atan2f | N/A | SRF-PLL (Clarke→dq) |

---

## 6. 风险评估

| 风险 | 缓解措施 |
|------|---------|
| U_line 索引变更影响保护逻辑 | `task_protect.c` 的 mask 是按位定义的，只要 `adc_preproc_3p` 正确映射 U_line 即可；保护逻辑本身不依赖索引含义 |
| SRF-PLL 参数需实物整定 | 沿用原 `task_pll.c` 的 PLL_Kp/PLL_Ki/LOCK_CNT 值，已在 G474 上验证过 |
| SVPWM 需 HRTIM 三路互补输出 | 确认硬件 Timer A/B/F 均已配置为互补+死区；原 `task_pwm.c` 已用这三个 timer |
| 三相电流传感器未校准 | `hardware_def_3p.h` 中三相电流常数独立可配 |
| Flash/ROM 占用增加 | PR 双份 + SVPWM 计算量约 200 条 FPU 指令，G474 的 170MHz Cortex-M4F 完全可承受 @ 10kHz |
