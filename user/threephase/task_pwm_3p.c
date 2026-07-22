#include "task_pwm_3p.h"
#include "hrtim.h"
#include "hardware_def_3p.h"

#define PWM_PERIOD  51200U

// ============================================================
// SVPWM 三相中心对齐调制 (min-max 共模注入)
//
// v_a/b/c: 三相电压指令 [V] (相电压, 相对于虚拟中点)
// udc:    直流母线电压 [V]
//
// 输出: TA_CMP1, TB_CMP1, TF_CMP1 (中心对齐, 互补+死区由硬件处理)
//
// 原理:
//   v_offset = -(v_max + v_min) / 2     ← 注入零序分量
//   duty_x = (v_x + v_offset) / udc + 0.5
//
// 等效于 SVPWM, 电压利用率比纯 SPWM 高 15.5%
// ============================================================
void _svpwm_3p(float v_a, float v_b, float v_c, float udc)
{
    // --- 找最大/最小值 ---
    float v_max = v_a;
    if (v_b > v_max) v_max = v_b;
    if (v_c > v_max) v_max = v_c;

    float v_min = v_a;
    if (v_b < v_min) v_min = v_b;
    if (v_c < v_min) v_min = v_c;

    // --- 共模注入 (零序分量) ---
    float v_offset = -(v_max + v_min) * 0.5f;

    // --- 计算占空比 [0, 1] ---
    float duty_a = (v_a + v_offset) / udc + 0.5f;
    float duty_b = (v_b + v_offset) / udc + 0.5f;
    float duty_c = (v_c + v_offset) / udc + 0.5f;

    // --- 限幅 ---
    if (duty_a > GRID3P_M_MAX) duty_a = GRID3P_M_MAX;
    if (duty_a < GRID3P_M_MIN) duty_a = GRID3P_M_MIN;
    if (duty_b > GRID3P_M_MAX) duty_b = GRID3P_M_MAX;
    if (duty_b < GRID3P_M_MIN) duty_b = GRID3P_M_MIN;
    if (duty_c > GRID3P_M_MAX) duty_c = GRID3P_M_MAX;
    if (duty_c < GRID3P_M_MIN) duty_c = GRID3P_M_MIN;

    // --- HRTIM 比较值 ---
    uint32_t cmp_a = (uint32_t)(duty_a * (float)PWM_PERIOD);
    uint32_t cmp_b = (uint32_t)(duty_b * (float)PWM_PERIOD);
    uint32_t cmp_c = (uint32_t)(duty_c * (float)PWM_PERIOD);

    if (cmp_a > PWM_PERIOD) cmp_a = PWM_PERIOD;
    if (cmp_b > PWM_PERIOD) cmp_b = PWM_PERIOD;
    if (cmp_c > PWM_PERIOD) cmp_c = PWM_PERIOD;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, cmp_c);
}
