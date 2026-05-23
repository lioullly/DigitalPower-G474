#include "task.h"

void Task_Init(Task* t, TaskFunc func, uint32_t period_us, uint8_t repeat)
{
    if (!t) return;
    t->func = func;
    t->period_us = (period_us == 0) ? 1 : period_us;
    t->remaining_us = t->period_us;
    t->repeat = repeat;
    t->enabled = 1;
}

void Task_Enable(Task* t)
{
    if (!t) return;
    t->enabled = 1;
}

void Task_Disable(Task* t)
{
    if (!t) return;
    t->enabled = 0;
}
