#include "task_protect_1p.h"
#include "user.h"
#include "hardware_def.h"
#include "task_pll_1p.h"

uint8_t g_fault_code = FAULT_NONE;

#define FAULT_DELAY  10

static uint16_t ov_cnt = 0;
static uint16_t uv_cnt = 0;
static uint16_t oc_cnt = 0;
static uint16_t ac_ov_cnt = 0;

void Task_Protect_1P(void)
{
    if (g_fault_code != FAULT_NONE)
        return;

    if (U_line[0] > UDC_OV)
    {
        if (++ov_cnt >= FAULT_DELAY)
            g_fault_code = FAULT_OV;
    }
    else ov_cnt = 0;

    if (U_line[0] < UDC_UV)
    {
        if (++uv_cnt >= FAULT_DELAY)
            g_fault_code = FAULT_UV;
    }
    else uv_cnt = 0;

    float i_abs = I_line[0];
    if (i_abs < 0.0f) i_abs = -i_abs;
    if (i_abs > IL1_OC)
    {
        if (++oc_cnt >= FAULT_DELAY)
            g_fault_code = FAULT_OC;
    }
    else oc_cnt = 0;

    float uab_abs = U_line[1];
    if (uab_abs < 0.0f) uab_abs = -uab_abs;
    if (uab_abs > UREF * UAC_OV_RATIO)
    {
        if (++ac_ov_cnt >= FAULT_DELAY)
            g_fault_code = FAULT_AC_OV;
    }
    else ac_ov_cnt = 0;

    if (g_fault_code != FAULT_NONE)
    {
        Run_Flag = 0;
    }
}
