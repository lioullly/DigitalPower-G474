#include "task_control_3p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_3p.h"
#include "task_pwm_3p.h"
#include "task_protect.h"

// ============================================================
// 三相离网逆变器: 2×PI 电压环 + 3×PR 电流环
//
// 电压外环 (1kHz, 10:1 降采样):
//   PI_Uab: Uab RMS → I_a (A 相电流幅值)
//   PI_Ubc: Ubc RMS → I_c (C 相电流幅值)
//
// 电流内环 (10kHz, 每相独立 PR):
//   i_ref_a = I_a × sin(wt)
//   i_ref_c = I_c × sin(wt + 120°)
//   i_ref_b = -(i_ref_a + i_ref_c)   ← 三线系统自动满足 KCL
//
//   PR_a(IL1), PR_b(IL2), PR_c(IL3) → v_a, v_b, v_c
//
// 调制: SVPWM (min-max 共模注入)
// ============================================================

void Task_Control_3P_OffGrid(void)
{
    static uint8_t   prev_active = 0;
    static uint16_t  v_cnt = 0;
    static PI_TypeDef pi_ab, pi_bc;
    static PR_TypeDef pr_a, pr_b, pr_c;
    static float     I_a   = 0.0f;   // A 相电流幅值 (PI_Uab 输出)
    static float     I_c   = 0.0f;   // C 相电流幅值 (PI_Ubc 输出)
    static float     i_lim = GRID3P_I_START;  // 软启动上限 (逐步放开)

    // ============================================================
    // Phase 1: Run_Flag=0 → 停机
    // ============================================================
    if (!Run_Flag) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        if (prev_active) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
            HAL_HRTIM_WaveformCounterStop(&hhrtim1,
                HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B |
                HRTIM_TIMERID_TIMER_F);
        }
        prev_active = 0;
        v_cnt = 0;
        I_a = 0.0f; I_c = 0.0f;
        i_lim = GRID3P_I_START;
        return;
    }

    // ============================================================
    // Phase 2: 激活条件判断 (带迟滞)
    // ============================================================
    uint8_t active;
    if (prev_active) {
        active = (U_line[1] > GRID3P_UDC_HOLD);
    } else {
        active = (U_line[1] > GRID3P_UDC_START);
    }
    uint8_t rising = (active && !prev_active);

    if (!active && prev_active) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
            HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_HRTIM_WaveformCounterStop(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B |
            HRTIM_TIMERID_TIMER_F);
    }
    prev_active = active;
    if (!active) return;

    // ============================================================
    // Phase 3: 上升沿一次性初始化
    // ============================================================
    if (rising) {
        // --- 功率硬件 ---
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);

        // --- 3 个 PR 电流环 (参数相同, 实例独立) ---
        f32_PR_Init(&pr_a, GRID3P_PR_KP, GRID3P_PR_KR,
                    GRID3P_PR_F0, GRID3P_PR_BW, 10000.0f,
                    (int16_t)PR_CTRL_CLAMP, -(int16_t)PR_CTRL_CLAMP);
        f32_PR_Init(&pr_b, GRID3P_PR_KP, GRID3P_PR_KR,
                    GRID3P_PR_F0, GRID3P_PR_BW, 10000.0f,
                    (int16_t)PR_CTRL_CLAMP, -(int16_t)PR_CTRL_CLAMP);
        f32_PR_Init(&pr_c, GRID3P_PR_KP, GRID3P_PR_KR,
                    GRID3P_PR_F0, GRID3P_PR_BW, 10000.0f,
                    (int16_t)PR_CTRL_CLAMP, -(int16_t)PR_CTRL_CLAMP);

        // --- 2 个电压 PI (线电压 RMS 控制) ---
        // Ts=0.001s (1kHz), 输出限幅 [0, I_MAX]
        f32_PI_Init(&pi_ab, 0.001f, GRID3P_V_KP, GRID3P_V_KI,
                    (int16_t)GRID3P_IMAG_MAX, 0);
        f32_PI_Init(&pi_bc, 0.001f, GRID3P_V_KP, GRID3P_V_KI,
                    (int16_t)GRID3P_IMAG_MAX, 0);

        // --- 软启动状态 ---
        i_lim = GRID3P_I_START;
        I_a   = GRID3P_I_START;
        I_c   = GRID3P_I_START;
        v_cnt = 0;

        // --- 启动三路 HRTIM ---
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
            HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B |
            HRTIM_TIMERID_TIMER_F);

        // --- 保护: 三相过流 + 四路过压 + 母线欠压 ---
        g_protect_mask    = PROT_IL1_OC | PROT_IL2_OC | PROT_IL3_OC
                          | PROT_ADC2_R1_OV | PROT_ADC2_R2_OV
                          | PROT_ADC2_R3_OV | PROT_ADC2_R4_OV
                          | PROT_ADC2_R2_UV;
    }

    // ============================================================
    // Phase 4a: 电压外环 @ 1kHz (10:1 降采样)
    // ============================================================
    if (++v_cnt >= GRID3P_V_DECIMATE) {
        v_cnt = 0;

        // 软启动 ramp: 逐步提高电流上限
        if (i_lim < GRID3P_IMAG_MAX)
            i_lim += GRID3P_I_SLEW;
        if (i_lim > GRID3P_IMAG_MAX)
            i_lim = GRID3P_IMAG_MAX;

        // PI_Uab: 控制 Uab RMS → A 相电流幅值
        float tmp_a = f32_PI_Calculate(&pi_ab, GRID3P_UREF, g_uab_rms);
        if (tmp_a > i_lim) { tmp_a = i_lim; pi_ab.y1 = i_lim; }
        if (tmp_a < 0.0f)  { tmp_a = 0.0f;  pi_ab.y1 = 0.0f; }
        I_a = tmp_a;

        // PI_Ubc: 控制 Ubc RMS → C 相电流幅值
        float tmp_c = f32_PI_Calculate(&pi_bc, GRID3P_UREF, g_ubc_rms);
        if (tmp_c > i_lim) { tmp_c = i_lim; pi_bc.y1 = i_lim; }
        if (tmp_c < 0.0f)  { tmp_c = 0.0f;  pi_bc.y1 = 0.0f; }
        I_c = tmp_c;
    }

    // ============================================================
    // Phase 4b: 电流参考生成 + PR 电流内环 @ 10kHz
    //
    // 控制自由度映射:
    //   I_a (PI_Uab 输出) → A 相电流幅值
    //   I_c (PI_Ubc 输出) → C 相电流幅值
    //   B 相由 KCL 推导: i_ref_b = -(i_ref_a + i_ref_c)
    //
    // 物理含义:
    //   I_a↑ → Uab↑ (A 相电流增大 → A 相电压增大 → Uab=Va-Vb 增大)
    //   I_c↑ → Ubc↑ (C 相电流增大 → C 相电压减小 → Ubc=Vb-Vc 增大)
    // ============================================================

    // 本地相位累加器 (50Hz @ 10kHz, 不依赖 Sine_Phase_Integrator)
    // 离网模式: 逆变器自己生成频率基准, 无需锁电网
    static float phase_3p = 0.0f;   // 归一化 [0, 1)
    phase_3p += 0.005f;             // 50Hz × 0.0001s = 0.005/step
    if (phase_3p >= 1.0f) phase_3p -= 1.0f;

    float wt = phase_3p * 2.0f * PI;

    float sin_a = arm_sin_f32(wt);                         // sin(0°) → A 相
    float sin_c = arm_sin_f32(wt + 2.094395102f);          // sin(+120°) → C 相
    // B 相不作 sin 模板: i_ref_b = -(i_ref_a + i_ref_c) 由 KCL 推导

    // 电流参考值 (瞬时值 = 幅值 × 正弦模板)
    float i_ref_a = I_a * sin_a;
    float i_ref_c = I_c * sin_c;
    float i_ref_b = -(i_ref_a + i_ref_c);                  // KCL: ia+ib+ic=0

    // 峰值限幅 (防止瞬时值超出硬件能力)
    float i_pk = GRID3P_IREF_PK;
    if (i_ref_a >  i_pk) i_ref_a =  i_pk;
    if (i_ref_a < -i_pk) i_ref_a = -i_pk;
    if (i_ref_b >  i_pk) i_ref_b =  i_pk;
    if (i_ref_b < -i_pk) i_ref_b = -i_pk;
    if (i_ref_c >  i_pk) i_ref_c =  i_pk;
    if (i_ref_c < -i_pk) i_ref_c = -i_pk;

    // --- 三个独立 PR 电流环 ---
    float i_fb_a = I_line[0];
    float i_fb_b = I_line[1];
    float i_fb_c = I_line[2];

    // 逆变器电流方向: 桥臂流出为正, 所以反馈取负 (与单相 Grid 模式一致)
    float v_a = f32_PR_Calculate(&pr_a, i_ref_a + i_fb_a);  // err = ref - (-fb) = ref + fb
    float v_b = f32_PR_Calculate(&pr_b, i_ref_b + i_fb_b);
    float v_c = f32_PR_Calculate(&pr_c, i_ref_c + i_fb_c);

    // --- SVPWM ---
    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;

    g_dbg_err   = i_ref_a + i_fb_a;  // A 相电流误差
    g_dbg_vctrl = v_a;               // A 相 PR 输出

    _svpwm_3p(v_a, v_b, v_c, udc);
}

// ============================================================
// SVPWM 开环调试: 固定调制比 M=0.3, 50Hz 旋转矢量
// 用法: g_control_isr = Task_Debug_SVPWM;
// 示波器测 TA/TB/TF 输出 → 应看到三相正弦 PWM，相位互差 120°
// ============================================================
void Task_Debug_SVPWM(void)
{
    static uint8_t started = 0;

    if (!Run_Flag) {
        if (started) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
            started = 0;
        }
        return;
    }

    if (!started) {
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
            HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B |
            HRTIM_TIMERID_TIMER_F);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);

        // 最小保护: 只开三相过流, 不设电压保护 (开环无反馈)
        g_protect_mask    = PROT_IL1_OC | PROT_IL2_OC | PROT_IL3_OC;

        started = 1;
    }

    // 本地相位累加器 (50Hz @ 10kHz)
    static float phase = 0.0f;
    phase += 0.005f;                          // 50Hz / 10000Hz
    if (phase >= 1.0f) phase -= 1.0f;

    float wt = phase * 2.0f * PI;

    // 三相正弦参考 (调制比 M=0.3, 用 Udc 估算或固定值)
    float udc = U_line[1];
    if (udc < 1.0f) udc = 20.0f;             // Udc 未上电时用默认值
    float v_mag = 0.3f * udc;                 // M=0.3

    float v_a = v_mag * arm_sin_f32(wt);
    float v_b = v_mag * arm_sin_f32(wt - 2.094395102f);
    float v_c = v_mag * arm_sin_f32(wt + 2.094395102f);

    _svpwm_3p(v_a, v_b, v_c, udc);
}
