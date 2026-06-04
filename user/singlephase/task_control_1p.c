#include "task_control_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_pwm_1p.h"
#include "task_pll_1p.h"

extern PI_TypeDef Voltage_PI_Loop;
extern PR_TypeDef Current_PR_Loop_alpha;
extern float g_duty_a, g_duty_b, g_duty_c;
extern uint8_t g_fault_code;

typedef enum {
    STATE_STOP = 0,
    STATE_STARTUP,      // detecting grid, initializing
    STATE_OFF_GRID,     // free-running voltage source
    STATE_GRID_TIED,    // PLL-locked current source
    STATE_FAULT,        // protection shutdown
} CtrlState;

void Task_Control_PI_PR_Loop(void)
{
    static CtrlState state = STATE_STOP;
    static uint8_t prev_run = 0;
    uint8_t run_rising = (Run_Flag && !prev_run);
    uint8_t run_falling = (!Run_Flag && prev_run);
    prev_run = Run_Flag;

    // ---- state transitions ----
    switch (state) {

    case STATE_STOP:
        if (run_rising) {
            state = STATE_STARTUP;
        }
        break;

    case STATE_STARTUP:
        // startup complete → enter operating mode
        // (one-shot init is done below, then immediately switch)
        break;

    case STATE_OFF_GRID:
    case STATE_GRID_TIED:
        if (run_falling) {
            state = STATE_STOP;
        }
        if (g_fault_code != 0) {
            state = STATE_FAULT;
        }
        break;

    case STATE_FAULT:
        if (run_falling || g_fault_code == 0) {
            state = STATE_STOP;
        }
        break;
    }

    // ---- state actions ----
    static float  i_mag    = I_MAG_DEFAULT;
    static uint16_t v_dec  = 0;
    static uint8_t grid_mode = 0;
    static uint8_t counters_started = 0;

    // start TA/TB/TF counters exactly once (never stop, avoids HAL restart bugs)
    if (!counters_started) {
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_F);
        counters_started = 1;
    }

    switch (state) {

    case STATE_STOP:
        // only stop outputs on the transition (not every cycle)
        if (run_falling) {
            g_duty_a = g_duty_b = g_duty_c = 0.5f;
            Task_PWM_1P_Update();
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        }
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        i_mag = I_MAG_DEFAULT;
        v_dec = 0;
        grid_mode = 0;
        return;

    case STATE_STARTUP:
        // detect grid BEFORE HRTIM output starts
        grid_mode = (g_uab_rms > 20.0f) ? 1 : 0;
        if (grid_mode) {
            // Grid-tied: PR current-loop only (no PI), i_mag set externally
            f32_PR_Init(&Current_PR_Loop_alpha, 0.5f, 5.0f, 50.0f, 10.0f, 10000.0f, 60.0f, -60.0f);
            i_mag = 1.0f;  // default grid current amplitude
        } else {
            f32_PI_Init(&Voltage_PI_Loop, 0.02f, 0.5f, 5.0f, (int16_t)DC_OV, 0);  // Vpeak output
            f32_PR_Init(&Current_PR_Loop_alpha, 2.0f, 20.0f, 50.0f, 10.0f, 10000.0f, 60.0f, -60.0f);
        }
        v_dec = 0;
        if (!grid_mode) i_mag = UREF * 1.414f;  // pre-charge Vpeak for faster startup
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);

        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
            HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        state = grid_mode ? STATE_GRID_TIED : STATE_OFF_GRID;
        return;

    case STATE_FAULT:
        Run_Flag = 0;
        state = STATE_STOP;
        return;

    default:
        break;
    }

    // ============================================================
    //  running states: OFF_GRID or GRID_TIED (common control loop)
    // ============================================================

    HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);

    // --- voltage outer loop (off-grid only), every 20ms ---
    if (state != STATE_GRID_TIED) {
        if (++v_dec >= 200) {
            v_dec = 0;
            i_mag = f32_PI_Calculate(&Voltage_PI_Loop, UREF, g_uab_rms);
        }
    }

    float v_ref, i_err = 0.0f, v_ctrl = 0.0f;
    if (state == STATE_GRID_TIED) {
        // Grid-tied: PR current control only, i_mag set by user/serial
        float i_ref = i_mag * arm_cos_f32(g_wt * 2.0f * PI);
        float i_fb  = -I_line[0];
        i_err  = i_ref - i_fb;
        v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);
        v_ref  = U_line[0] - v_ctrl;
    } else {
        // Off-grid: voltage-mode — PI output directly sets Vpeak
        v_ref = i_mag * g_sin_wt;
    }

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;
    // clamp v_ref to avoid overmodulation before computing m
    if (v_ref >  0.95f * udc) v_ref =  0.95f * udc;
    if (v_ref < -0.95f * udc) v_ref = -0.95f * udc;
    float m = v_ref / udc;

    g_dbg_err   = i_err;
    g_dbg_vctrl = v_ctrl;
    g_dbg_m     = m;

    // --- SPWM ---
    g_duty_a = 0.5f + 0.5f * m;
    g_duty_b = 0.5f - 0.5f * m;

    if (g_duty_a > 0.95f) g_duty_a = 0.95f;
    if (g_duty_b > 0.95f) g_duty_b = 0.95f;
    if (g_duty_a < 0.05f) g_duty_a = 0.05f;
    if (g_duty_b < 0.05f) g_duty_b = 0.05f;
    g_duty_c = 0.5f;
    Task_PWM_1P_Update();
}
