#include "task_protect.h"
#include "user.h"
#include "hardware_def.h"

uint8_t  g_fault_code  = FAULT_NONE;
uint16_t g_protect_mask = 0;

#define FAULT_DELAY  10
#define BLANK_MS     10    // 10ms enough for pre-charge inrush, PFC switches actively
#define BLANK_CNT    (BLANK_MS * 20)  // 20kHz → 20 counts/ms

// 去抖计数器: [0]=IL1_OC [1]=Uac_OV [2]=Udc_OV [3]=Udc_UV
static uint16_t cnt[4] = {0};

// ISR-safe: called from ADC ISR @ 20kHz, mask 驱动的公共保护
void Task_Protect_Run(void)
{
    static uint8_t prev_run = 0;
    uint8_t rising = (Run_Flag && !prev_run);
    prev_run = Run_Flag;

    if (rising) {
        g_fault_code = FAULT_NONE;
        for (int i = 0; i < 4; i++) cnt[i] = 0;
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
            if (++cnt[0] >= 3)           // OC 用 3 次去抖
                g_fault_code = FAULT_OC_IL1;
        } else cnt[0] = 0;
    }

skip_oc:
    if (g_fault_code != FAULT_NONE) { Run_Flag = 0; return; }

    // --- Uac 过压 (U_line[0], AC) ---
    if (g_protect_mask & PROT_UAC_OV) {
        float v = U_line[0];
        if (v < 0.0f) v = -v;
        if (v > DC_OV) {
            if (++cnt[1] >= FAULT_DELAY)
                g_fault_code = FAULT_OV;
        } else cnt[1] = 0;
    }

    // --- Udc 过压 (U_line[1], DC bus) ---
    if (g_protect_mask & PROT_UDC_OV) {
        float v = U_line[1];
        if (v < 0.0f) v = -v;
        if (v > DC_OV) {
            if (++cnt[2] >= FAULT_DELAY)
                g_fault_code = FAULT_OV;
        } else cnt[2] = 0;
    }

    // --- Udc 欠压 (U_line[1], DC bus) ---
    if (g_protect_mask & PROT_UDC_UV) {
        if (U_line[1] < DC_UV) {
            if (++cnt[3] >= FAULT_DELAY)
                g_fault_code = FAULT_UV;
        } else cnt[3] = 0;
    }

    if (g_fault_code != FAULT_NONE)
        Run_Flag = 0;
}
