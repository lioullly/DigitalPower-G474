#ifndef __TASK_PLL_1P_H
#define __TASK_PLL_1P_H

#include "main.h"

void Task_PLL_1P_Process(void);

extern float g_wt;
extern uint8_t g_pll_locked;
extern float g_freq_est;

#endif
