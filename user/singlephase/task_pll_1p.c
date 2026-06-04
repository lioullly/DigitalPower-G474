#include "task_pll_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"

// --- SRF-PLL state ---
float g_wt = 0.0f;             // estimated grid phase [0, 1) cycles
uint8_t g_pll_locked = 0;      // 1 = PLL locked to grid
float g_freq_est = 50.0f;      // estimated grid frequency [Hz]

// ---- delayed-signal quadrature generator (90° at 50Hz, 1kHz) ----
static float delay_buf[5] = {0};     // 5-sample ring buffer = 5ms
static uint8_t delay_idx = 0;

// ---- PI loop filter for frequency correction ----
static float pi_integ = 0.0f;        // PI integrator accumulator

// ---- phase accumulator ----
static float   phase_accum = 0.0f;   // PLL phase [0, 1) cycles
static uint16_t lock_cnt  = 0;       // consecutive valid samples

// ---- tunables ----
#define PLL_Kp     4.0f           // proportional gain
#define PLL_Ki    80.0f           // integral gain
#define PLL_Ts     0.001f         // sample time = 1ms (1kHz)
#define LOCK_CNT   200            // lock after 200×1ms = 200ms
#define FREQ_NOM   50.0f          // nominal grid frequency [Hz]
#define PHASE_WRAP 1.0f           // phase wraps at 1.0 cycles

void Task_PLL_1P_Process(void)
{
    // ---- stationary αβ pair ----
    float v_alpha = U_line[0];  // Uab — AC instantaneous voltage
    float v_beta  = delay_buf[delay_idx];      // v_alpha delayed 5ms = 90° @ 50Hz
    delay_buf[delay_idx] = v_alpha;            // push new sample into ring buffer
    delay_idx = (delay_idx + 1) % 5;

    // ---- voltage magnitude ----
    float v_mag;
    arm_sqrt_f32(v_alpha * v_alpha + v_beta * v_beta, &v_mag);

    if (v_mag < 10.0f) {
        // ---- grid lost ----
        lock_cnt  = 0;
        g_pll_locked = 0;
        pi_integ  = 0.0f;
    } else {
        // ---- SRF Park transform: αβ → dq ----
        float cos_pll = arm_cos_f32(phase_accum * 2.0f * PI);
        float sin_pll = arm_sin_f32(phase_accum * 2.0f * PI);

        // v_q = v_β·cos(θ_pll) - v_α·sin(θ_pll) = sin(θ_grid - θ_pll) ≈ Δθ
        float v_q = (v_beta * cos_pll - v_alpha * sin_pll) / v_mag;

        // ---- PI frequency corrector ----
        pi_integ += PLL_Ki * PLL_Ts * v_q;
        float freq_corr = PLL_Kp * v_q + pi_integ;

        g_freq_est = FREQ_NOM + freq_corr;

        // ---- phase integrator (VCO) ----
        phase_accum += g_freq_est * PLL_Ts;
        if (phase_accum >= PHASE_WRAP) phase_accum -= PHASE_WRAP;
        if (phase_accum <  0.0f)      phase_accum += PHASE_WRAP;

        g_wt = phase_accum;

        // ---- lock detection ----
        if (lock_cnt < LOCK_CNT)
            lock_cnt++;
        else
            g_pll_locked = 1;
    }

    // relay controlled by Task_Control_PI_PR_Loop state machine
}
