#include "task_pll_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"

float g_wt = 0.0f;
uint8_t g_pll_locked = 0;

static float delay_buf[5] = {0};
static uint8_t delay_idx = 0;

static float vq_integ   = 0.0f;
float g_freq_est = 50.0f;
static float wt_accum   = 0.0f;
static uint16_t lock_cnt = 0;

#define PLL_Kp    5.0f
#define PLL_Ki   50.0f
#define PLL_Ts   0.001f
#define LOCK_CNT 200

#define FREQ_NOM  50.0f
#define WT_MAX    1.0f

void Task_PLL_1P_Process(void)
{
    float v_alpha = U_line[0];  // Uab, not Udc

    float v_beta = delay_buf[delay_idx];

    delay_buf[delay_idx] = v_alpha;
    delay_idx = (delay_idx + 1) % 5;

    float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);

    if (v_mag < 10.0f)
    {
        lock_cnt = 0;
        g_pll_locked = 0;
    }
    else
    {
        float cos_wt = arm_cos_f32(wt_accum * 2.0f * PI);
        float sin_wt = arm_sin_f32(wt_accum * 2.0f * PI);

        float v_q = (v_beta * cos_wt - v_alpha * sin_wt) / v_mag;

        vq_integ += PLL_Ki * PLL_Ts * v_q;
        float freq_err = PLL_Kp * v_q + vq_integ;

        g_freq_est = FREQ_NOM + freq_err;

        wt_accum += g_freq_est * PLL_Ts;
        if (wt_accum >= WT_MAX) wt_accum -= WT_MAX;
        if (wt_accum <  0.0f)   wt_accum += WT_MAX;

        g_wt = wt_accum;

        if (lock_cnt < LOCK_CNT)
            lock_cnt++;
        else
            g_pll_locked = 1;
    }

    if (Run_Flag && g_pll_locked)
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
}
