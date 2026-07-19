#include "task_protect.h"
#include "user.h"
#include "hardware_def.h"

uint8_t  g_fault_code     = FAULT_NONE;
uint16_t g_protect_mask    = 0;
uint16_t g_protect_ac_mask = 0;

#define FAULT_DELAY  10
#define BLANK_MS     200
#define BLANK_CNT    (BLANK_MS * 10)  // 10kHz → 10 counts/ms

// 去抖计数器: [0]=IL1 [1]=IL2 [2]=IL3 [3]=Uab [4]=Udc [5]=Uac [6]=Ubc
static uint16_t cnt[7] = {0};

// ISR-safe: called from ADC ISR @ 10kHz, mask 驱动的公共保护
void Task_Protect_Run(void)
{
    // Run_Flag 上升沿清除已锁存的故障码和计数器
    static uint8_t prev_run = 0;
    uint8_t rising = (Run_Flag && !prev_run);
    prev_run = Run_Flag;

    if (rising) {
        g_fault_code = FAULT_NONE;
        for (int i = 0; i < 7; i++) cnt[i] = 0;
    }

    // 启动 blanking: 跳过母线电容充电期间的 OC 检测
    static uint16_t blank_cnt = BLANK_CNT;
    if (rising) blank_cnt = 0;
    if (blank_cnt < BLANK_CNT) { blank_cnt++; goto skip_oc; }

    if (g_fault_code != FAULT_NONE)
        return;

    // --- IL1 过流 ---
    if (g_protect_mask & PROT_IL1_OC) {
        float i_abs = I_line[0];
        if (i_abs < 0.0f) i_abs = -i_abs;
        if (i_abs > IL1_OC) {
            if (++cnt[0] >= 3)           // OC 用 3 次去抖 (0.3ms)
                g_fault_code = FAULT_OC_IL1;
        } else cnt[0] = 0;
    }

    // --- IL2 / IL3 过流 (预留) ---
    if (g_protect_mask & PROT_IL2_OC) {
        float i_abs = I_line[1];
        if (i_abs < 0.0f) i_abs = -i_abs;
        if (i_abs > IL1_OC) {
            if (++cnt[1] >= 3)
                g_fault_code = FAULT_OC_IL2;
        } else cnt[1] = 0;
    }
    if (g_protect_mask & PROT_IL3_OC) {
        float i_abs = I_line[2];
        if (i_abs < 0.0f) i_abs = -i_abs;
        if (i_abs > IL1_OC) {
            if (++cnt[2] >= 3)
                g_fault_code = FAULT_OC_IL3;
        } else cnt[2] = 0;
    }

skip_oc:
    if (g_fault_code != FAULT_NONE) { Run_Flag = 0; return; }

    // --- Uab 过压 (U_line[0]) ---
    if (g_protect_mask & PROT_ADC2_R1_OV) {
        float v = U_line[0];
        if (g_protect_ac_mask & PROT_ADC2_R1_AC) { if (v < 0.0f) v = -v; }
        if (v > DC_OV) {
            if (++cnt[3] >= FAULT_DELAY)
                g_fault_code = FAULT_OV;
        } else cnt[3] = 0;
    }

    // --- Udc 过压 (U_line[1], 母线) ---
    if (g_protect_mask & PROT_ADC2_R2_OV) {
        float v = U_line[1];
        if (g_protect_ac_mask & PROT_ADC2_R2_AC) { if (v < 0.0f) v = -v; }
        if (v > DC_OV) {
            if (++cnt[4] >= FAULT_DELAY)
                g_fault_code = FAULT_OV;
        } else cnt[4] = 0;
    }

    // --- Uac 过压 (U_line[2]) ---
    if (g_protect_mask & PROT_ADC2_R3_OV) {
        float v = U_line[2];
        if (g_protect_ac_mask & PROT_ADC2_R3_AC) { if (v < 0.0f) v = -v; }
        if (v > DC_OV) {
            if (++cnt[5] >= FAULT_DELAY)
                g_fault_code = FAULT_OV;
        } else cnt[5] = 0;
    }

    // --- Ubc 过压 (U_line[3]) ---
    if (g_protect_mask & PROT_ADC2_R4_OV) {
        float v = U_line[3];
        if (g_protect_ac_mask & PROT_ADC2_R4_AC) { if (v < 0.0f) v = -v; }
        if (v > DC_OV) {
            if (++cnt[6] >= FAULT_DELAY)
                g_fault_code = FAULT_OV;
        } else cnt[6] = 0;
    }

    // --- Udc 欠压 (U_line[1], 母线) ---
    if (g_protect_mask & PROT_ADC2_R2_UV) {
        if (U_line[1] < DC_UV) {
            if (++cnt[4] >= FAULT_DELAY)
                g_fault_code = FAULT_UV;
        } else cnt[4] = 0;
    }

    if (g_fault_code != FAULT_NONE)
        Run_Flag = 0;
}
