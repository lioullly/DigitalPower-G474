#include "task_protect_1p.h"
#include "user.h"
#include "hardware_def.h"
// #include "task_pll_1p.h" — removed (unused)
uint8_t g_fault_code = FAULT_NONE;

#define FAULT_DELAY  10

#define BLANK_MS  200
#define BLANK_CNT (BLANK_MS * 10)  // 10kHz → 10 counts/ms

static uint16_t ov_cnt = 0;
static uint16_t uv_cnt = 0;
static uint16_t oc_cnt = 0;

// ISR-safe: called from ADC ISR @ 10kHz, no scheduler dependency
void Task_Protect_1P_Run(void)
{
    // clear latched fault on Run_Flag rising edge
    static uint8_t prev_run = 0;
    uint8_t rising = (Run_Flag && !prev_run);
    prev_run = Run_Flag;

    if (rising) {
        g_fault_code = FAULT_NONE;
        ov_cnt = uv_cnt = oc_cnt = 0;
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
    }

    // startup blanking: skip OC while DC bus capacitor charges
    static uint16_t blank_cnt = BLANK_CNT;
    if (rising) blank_cnt = 0;
    if (blank_cnt < BLANK_CNT) { blank_cnt++; goto skip_oc; }

    // --- fault blink: Red LED ~2Hz when latched ---
    if (g_fault_code != FAULT_NONE) {
        static uint16_t blink_cnt = 0;
        if (++blink_cnt >= 12500) {        // 25000 / 12500 = 2Hz
            blink_cnt = 0;
            HAL_GPIO_TogglePin(Red_GPIO_Port, Red_Pin);
        }
        return;
    }

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

    float i_abs = I_line[0];
    if (i_abs < 0.0f) i_abs = -i_abs;
    if (i_abs > IL1_OC)
    {
        if (++oc_cnt >= 3)
            g_fault_code = FAULT_OC;
    }
    else oc_cnt = 0;

skip_oc:

    if (g_fault_code != FAULT_NONE)
    {
        Run_Flag = 0;
    }
}
