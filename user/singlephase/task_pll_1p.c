/**
 * ATAN-PLL — single-phase grid synchronization
 *
 * States:  INIT → SOFT → RUN
 *
 * External API:
 *   g_wt          — estimated grid phase [0, 1) cycles
 *   g_freq_est    — estimated grid frequency [Hz]
 *   g_pll_locked  — 1 = PLL locked (RUN state)
 */

#include "task_pll_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "arm_math.h"
#include <math.h>

/* ================================================================
 *  Global outputs
 * ================================================================ */
float   g_wt         = 0.0f;
uint8_t g_pll_locked = 0;
float   g_freq_est   = 50.0f;

/* ================================================================
 *  Quadrature — 5-point delay ring (90° @ 50Hz, 1kHz)
 * ================================================================ */
static float   delay_buf[5] = {0};
static uint8_t delay_idx    = 0;

/* ================================================================
 *  PI controller (uses f32_PI_Calculate from pi_pr_ctrl.c)
 *  Note: Ki_cont = Ki_disc / Ts → 150 / 0.001 = 150000
 *        so discrete I-step = Ki_cont * Ts = 150
 * ================================================================ */
static PI_TypeDef PLL_PI;

/* ================================================================
 *  State machine
 * ================================================================ */
typedef enum { PLL_INIT, PLL_SOFT, PLL_RUN } PllState;
static PllState  pll_state = PLL_INIT;
static uint16_t  soft_cnt  = 0;     // soft-start ramp counter

/* ================================================================
 *  Tunables
 * ================================================================ */
#define PLL_Kp         2.0f         // proportional gain
#define PLL_Ki_DISC    20.0f         // discrete integral gain (per sample)
#define PLL_Ki_CONT  (PLL_Ki_DISC / PLL_Ts)  // continuous Ki = 20000
#define PLL_Ts         0.001f        // sample time [s] (1kHz)
#define PLL_I_MAX      3.0f          // integrator clamp [Hz]
#define PLL_F_MIN     45.0f          // frequency hard-limit low  [Hz]
#define PLL_F_MAX     55.0f          // frequency hard-limit high [Hz]
#define PLL_F_NOM     50.0f          // nominal grid frequency [Hz]
#define PLL_VTH        5.0f          // voltage magnitude threshold [V]
#define PLL_SOFT_MS   80             // soft-start duration [ms]
#define PLL_SOFT_CNT  (PLL_SOFT_MS)  // samples @ 1kHz
#define PLL_LOCK_MS  200             // lock debounce [ms]
#define PLL_LOCK_CNT  (PLL_LOCK_MS)  // samples @ 1kHz

/*  Task_PLL_1P_Process — call @ 1kHz*/
void Task_PLL_1P_Process(void)
{
    /* ---- αβ pair ---- */
    static float uab_filt = 0.0f;
    uab_filt   = uab_filt * 0.5f + U_line[0] * 0.5f;  // ~80Hz LPF, 50Hz attenuation ~15%
    float v_a  = uab_filt;
    float v_b  = delay_buf[delay_idx];
    delay_buf[delay_idx] = v_a;
    delay_idx = (delay_idx + 1) % 5;

    /* ---- grid present? ---- */
    float v_mag;
    arm_sqrt_f32(v_a * v_a + v_b * v_b, &v_mag);

    // hysteresis: enter on <3V, exit on >5V
    static uint16_t run_cnt = 0;
    static uint8_t  grid_ok = 0;
    if (v_mag > PLL_VTH)  grid_ok = 1;
    if (v_mag < 3.0f)     grid_ok = 0;

    if (!grid_ok) {
        pll_state = PLL_INIT;
        run_cnt   = 0;
        return;
    }
    run_cnt++;

    /* ---- measured phase ---- */
    float phase_raw = atan2f(v_b, v_a) / (2.0f * PI);
    if (phase_raw < 0.0f) phase_raw += 1.0f;

    /* ---- phase error, wrapped + low-pass filtered ---- */
    float pe_raw = phase_raw - g_wt;
    if (pe_raw >  0.5f) pe_raw -= 1.0f;
    if (pe_raw < -0.5f) pe_raw += 1.0f;

    static float pe_filt = 0.0f;
    pe_filt = pe_filt * 0.9f + pe_raw * 0.1f;   // ~16Hz LPF on phase error
    float pe = pe_filt;

    /* ================================================================
     *  State machine
     * ================================================================ */
    switch (pll_state) {

    case PLL_INIT:
        g_wt      = phase_raw;
        soft_cnt  = 0;
        f32_PI_Init(&PLL_PI, PLL_Ts, 0.0f, 0.0f, 0.5f, -0.5f);   // init once, zero gains
        pll_state = PLL_SOFT;
        __attribute__((fallthrough));

    case PLL_SOFT: {
        float ramp = (float)(soft_cnt + 1) / (float)PLL_SOFT_CNT;
        if (ramp > 1.0f) ramp = 1.0f;

        float kp_s = PLL_Kp * ramp;
        float ki_c = (PLL_Ki_DISC * ramp) / PLL_Ts;                // discrete→continuous
        float imax = (PLL_I_MAX * ramp > 0.5f) ? PLL_I_MAX * ramp : 0.5f;

        // update gains only — integrator state (y1) preserved
        PLL_PI.Kp = kp_s;
        PLL_PI.Ki = ki_c;
        PLL_PI.filter_B1 =  kp_s + PLL_Ts * ki_c / 2.0f;
        PLL_PI.filter_B2 = -kp_s + PLL_Ts * ki_c / 2.0f;
        PLL_PI.TH =  imax;
        PLL_PI.TL = -imax;

        g_freq_est = PLL_F_NOM + f32_PI_Calculate(&PLL_PI, pe, 0.0f);
        if (g_freq_est > PLL_F_MAX) g_freq_est = PLL_F_MAX;
        if (g_freq_est < PLL_F_MIN) g_freq_est = PLL_F_MIN;

        g_wt += g_freq_est * PLL_Ts;
        if (g_wt >= 1.0f) g_wt -= 1.0f;
        if (g_wt <  0.0f) g_wt += 1.0f;

        if (++soft_cnt >= PLL_SOFT_CNT) {
            pll_state = PLL_RUN;
        }
        return;
    }

    case PLL_RUN: {
        g_freq_est = PLL_F_NOM + f32_PI_Calculate(&PLL_PI, pe, 0.0f);
        if (g_freq_est > PLL_F_MAX) g_freq_est = PLL_F_MAX;
        if (g_freq_est < PLL_F_MIN) g_freq_est = PLL_F_MIN;

        g_wt += g_freq_est * PLL_Ts;
        if (g_wt >= 1.0f) g_wt -= 1.0f;
        if (g_wt <  0.0f) g_wt += 1.0f;

        g_pll_locked = (run_cnt >= PLL_LOCK_CNT) ? 1 : 0;
        return;
    }
    }
}
