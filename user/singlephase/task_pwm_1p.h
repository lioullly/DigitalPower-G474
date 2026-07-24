#ifndef __TASK_PWM_1P_H
#define __TASK_PWM_1P_H

#include "main.h"

void _pwm_bipolar(float m);
void _pwm_unipolar(float m);  // 单极性倍频, m∈[-1,1]
void _pwm_hybrid(float m);   // |m|>0.12→单极性, |m|<0.08→双极性, 中间迟滞

#endif
