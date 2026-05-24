#include "timebase_scheduler.h"
#include <string.h>
#include "main.h"   // for TIM6

static Task tasks[SCHED_MAX_TASKS];
static uint32_t g_tick_us = 1;

#define TIM6_WRAP  100  // ARR=99 → counter wraps at 100

void tim_delay_us(uint32_t us)
{
    if (us == 0) return;
    volatile uint32_t *cnt = &TIM6->CNT;
    uint32_t ticks = us * 10;
    uint32_t start = *cnt;
    uint32_t elapsed = 0;
    uint32_t prev = start;
    while (elapsed < ticks) {
        uint32_t now = *cnt;
        if (now >= prev) elapsed += now - prev;
        else elapsed += (TIM6_WRAP - prev) + now;
        prev = now;
    }
}

void Scheduler_Init(uint32_t tick_us)
{
    g_tick_us = (tick_us == 0) ? 1 : tick_us;
    memset(tasks, 0, sizeof(tasks));
}

int Scheduler_AddTask(TaskFunc func, uint32_t period_us, uint8_t repeat)
{
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        if (tasks[i].func == 0) {
            tasks[i].func = func;
            tasks[i].period_us = (period_us == 0) ? g_tick_us : period_us;
            tasks[i].remaining_us = tasks[i].period_us;
            tasks[i].repeat = repeat;
            tasks[i].enabled = 1;
            return i;
        }
    }
    return -1;
}

void Scheduler_RemoveTask(int id)
{
    if (id < 0 || id >= SCHED_MAX_TASKS) return;
    tasks[id].func = 0;
    tasks[id].enabled = 0;
}

void Scheduler_Tick(void)
{
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        if (tasks[i].func && tasks[i].enabled) {
            if (tasks[i].remaining_us <= g_tick_us) {
                tasks[i].remaining_us = 0;
            } else {
                tasks[i].remaining_us -= g_tick_us;
            }
        }
    }
}

void Scheduler_Dispatch(void)
{
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        if (tasks[i].func && tasks[i].enabled && tasks[i].remaining_us == 0) {
            TaskFunc f = tasks[i].func;
            if (!tasks[i].repeat) {
                tasks[i].enabled = 0;
                tasks[i].func = 0;
            } else {
                tasks[i].remaining_us = tasks[i].period_us;
            }
            f();
            tim_delay_us(TASK_YIELD_US);  // yield CPU to ISRs
        }
    }
}
