#include "task_pwm_1p.h"
#include "hrtim.h"
#include "user.h"

#define PWM_PERIOD  64000U
#define PWM_DZ      0.02f    // dead zone around m=0 (hysteresis)

extern float g_duty_a, g_duty_b;

// center-aligned bipolar SPWM: m -> [cmp_a, cmp_b]
// m = v_ref/Udc ∈ [-1, 1];  KREF = m * (period/2)
// cmp_a = KREF + period/2 = (m+1) * period/2
// cmp_b = -KREF + period/2 = (1-m) * period/2
// output: v_bridge = (cmp_a - cmp_b) / period * Udc = m * Udc
void _pwm_bipolar(float m)
{
    if (m >  0.95f) m =  0.95f;
    if (m < -0.95f) m = -0.95f;

    uint32_t half = PWM_PERIOD >> 1;
    uint32_t cmp_a = (uint32_t)( m * (float)half + (float)half);
    uint32_t cmp_b = (uint32_t)(-m * (float)half + (float)half);

    if (cmp_a > PWM_PERIOD) cmp_a = PWM_PERIOD;
    if (cmp_b > PWM_PERIOD) cmp_b = PWM_PERIOD;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
}

// unipolar frequency-doubling PWM: one leg active, the other held at 0
// m > 0 → A switches, B=0.  m < 0 → B switches, A=0.  ripple @ 2×f_sw
void _pwm_unipolar(float m)
{
    float abs_m = (m > 0.0f) ? m : -m;
    if (abs_m > 0.95f) abs_m = 0.95f;

    // ZC hysteresis: prevent noise toggling near m=0
    //   0 → +1: m > +DZ      0 → -1: m < -DZ
    //  +1 →  0: m < +DZ/2   +1 → -1: m < -DZ   (direct flip, large transient)
    //  -1 →  0: m > -DZ/2   -1 → +1: m > +DZ   (direct flip)
    static int8_t leg = 0;
    if (leg == 0) {
        if      (m >  PWM_DZ) leg =  1;
        else if (m < -PWM_DZ) leg = -1;
    } else if (leg == 1) {
        if      (m < -PWM_DZ)       leg = -1;
        else if (m <  PWM_DZ * 0.5f) leg =  0;
    } else { // leg == -1
        if      (m > PWM_DZ)        leg =  1;
        else if (m > -PWM_DZ * 0.5f) leg =  0;
    }

    // Up mode: duty=1-CMP1/PERIOD.  PERIOD-1 avoids SET+RESET collision at CNT=PERIOD
    uint32_t GND = PWM_PERIOD - 1;
    if (leg == 0) {
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, GND);
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, GND);
    } else if (leg == 1) {
        uint32_t cmp = (uint32_t)((1.0f - abs_m) * (float)PWM_PERIOD);
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp);
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, GND);
    } else {
        uint32_t cmp = (uint32_t)((1.0f - abs_m) * (float)PWM_PERIOD);
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, GND);
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp);
    }
}

// hybrid PWM: hard-switch bipolar ↔ unipolar, narrow hysteresis
// |m| crosses 0.10 entering → unipolar; falls below 0.09 → bipolar
// No blend — single-sample transition, LC filter absorbs the step
void _pwm_hybrid(float m)
{
    if (m >  0.95f) m =  0.95f;
    if (m < -0.95f) m = -0.95f;

    float abs_m = (m > 0.0f) ? m : -m;
    uint32_t half = PWM_PERIOD >> 1;
    uint32_t GND  = PWM_PERIOD - 1;

    // --- mode with narrow hysteresis ---
    static uint8_t unipolar = 0;
    if (unipolar) {
        if (abs_m < 0.09f) unipolar = 0;
    } else {
        if (abs_m > 0.10f) unipolar = 1;
    }

    if (!unipolar) {
        // --- bipolar (Up mode, cmp swapped) ---
        uint32_t cmp_a = (uint32_t)(-m * (float)half + (float)half);
        uint32_t cmp_b = (uint32_t)( m * (float)half + (float)half);
        if (cmp_a > PWM_PERIOD) cmp_a = PWM_PERIOD;
        if (cmp_b > PWM_PERIOD) cmp_b = PWM_PERIOD;
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
    } else {
        // --- unipolar (Up mode, leg state machine) ---
        static int8_t leg = 0;
        if (leg == 0) {
            if      (m >  PWM_DZ) leg =  1;
            else if (m < -PWM_DZ) leg = -1;
        } else if (leg == 1) {
            if      (m < -PWM_DZ)       leg = -1;
            else if (m <  PWM_DZ * 0.5f) leg =  0;
        } else {
            if      (m > PWM_DZ)        leg =  1;
            else if (m > -PWM_DZ * 0.5f) leg =  0;
        }

        if (leg == 0) {
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, GND);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, GND);
        } else if (leg == 1) {
            uint32_t cmp = (uint32_t)((1.0f - abs_m) * (float)PWM_PERIOD);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, GND);
        } else {
            uint32_t cmp = (uint32_t)((1.0f - abs_m) * (float)PWM_PERIOD);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, GND);
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp);
        }
    }
}
