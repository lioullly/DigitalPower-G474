#include "task_protect_1p.h"
#include "user.h"
#include "hardware_def.h"
#include "task_pll_1p.h"

uint8_t g_fault_code = FAULT_NONE;

#define FAULT_DELAY  10

#define BLANK_MS  200
#define BLANK_CNT (BLANK_MS * 10)  // 10kHz → 10 counts/ms

static uint16_t ov_cnt = 0;
static uint16_t uv_cnt = 0;
static uint16_t oc_cnt = 0;

void Task_Protect_1P(void)
{
    // clear latched fault on Run_Flag rising edge
    static uint8_t prev_run = 0;
    uint8_t rising = (Run_Flag && !prev_run);
    prev_run = Run_Flag;

    if (rising) {
        g_fault_code = FAULT_NONE;
        ov_cnt = uv_cnt = oc_cnt = 0;
    }

    // startup blanking: skip OC while DC bus capacitor charges
    static uint16_t blank_cnt = BLANK_CNT;
    if (rising) blank_cnt = 0;
    if (blank_cnt < BLANK_CNT) { blank_cnt++; goto skip_oc; }

    if (g_fault_code != FAULT_NONE)
        return;

    // DC bus: U_line[1] = Udc
    if (U_line[1] > DC_OV)
    {
        if (++ov_cnt >= FAULT_DELAY)
            g_fault_code = FAULT_OV;
    }
    else ov_cnt = 0;

    if (U_line[1] < DC_UV)
    {
        if (++uv_cnt >= FAULT_DELAY)
            g_fault_code = FAULT_UV;
    }
    else uv_cnt = 0;

    if (g_irms > IL1_OC_RMS)
    {
        if (++oc_cnt >= FAULT_DELAY)
            g_fault_code = FAULT_OC;
    }
    else oc_cnt = 0;

skip_oc:

    if (g_fault_code != FAULT_NONE)
    {
        Run_Flag = 0;
    }
}
