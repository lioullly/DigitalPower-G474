#include "task_pwm_1p.h"
#include "hrtim.h"
#include "user.h"

#define PWM_PERIOD  51200U
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
    static int8_t leg = 0;
    if (leg == 0) {
        if      (m >  PWM_DZ) leg =  1;
        else if (m < -PWM_DZ) leg = -1;
    } else if (leg == 1 && m < PWM_DZ * 0.5f) {
        leg = 0;
    } else if (leg == -1 && m > -PWM_DZ * 0.5f) {
        leg = 0;
    }

    if (leg == 0) {
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, 0);
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, 0);
    } else if (leg == 1) {
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1,
            (uint32_t)(abs_m * (float)PWM_PERIOD));
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, 0);
    } else {
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, 0);
        __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1,
            (uint32_t)(abs_m * (float)PWM_PERIOD));
    }
}

// unipolar frequency-doubling PWM — for PFC (task_control_pfc.c)
// m ∈ [-1, 1] → leg A/B alternate, output ripple @ 2×f_sw
void Task_PWM_1P_UniUpdate(float m)
{
    if (!Run_Flag) return;

    float abs_m = (m > 0.0f) ? m : -m;
    if (abs_m > 0.95f) abs_m = 0.95f;

    // ZC hysteresis: prevent noise toggling near m=0
    static int8_t leg = 0;
    if (leg == 0) {
        if      (m >  PWM_DZ) leg =  1;
        else if (m < -PWM_DZ) leg = -1;
    } else if (leg == 1 && m < PWM_DZ * 0.5f) {
        leg = 0;
    } else if (leg == -1 && m > -PWM_DZ * 0.5f) {
        leg = 0;
    }

    uint32_t cmp_a, cmp_b;
    if (leg == 0) {
        cmp_a = cmp_b = 0;
    } else if (leg == 1) {
        cmp_a = (uint32_t)(abs_m * (float)PWM_PERIOD);
        cmp_b = 0;
    } else {
        cmp_a = 0;
        cmp_b = (uint32_t)(abs_m * (float)PWM_PERIOD);
    }

    if (cmp_a > PWM_PERIOD) cmp_a = PWM_PERIOD;
    if (cmp_b > PWM_PERIOD) cmp_b = PWM_PERIOD;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
}
