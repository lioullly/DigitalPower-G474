#include "task.h"

void Task_Init(Task* t, TaskFunc func, uint32_t period_ms, uint8_t repeat)
{
    if (!t) return;
    t->func = func;
    t->period_ms = (period_ms == 0) ? 1 : period_ms;
    t->remaining_ms = t->period_ms;
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
