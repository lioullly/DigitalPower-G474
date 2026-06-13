#include "timebase_scheduler.h"
#include <string.h>
#include "main.h"

static Task tasks[SCHED_MAX_TASKS];
static uint32_t g_tick_us = 1;

volatile float g_cpu_usage = 0.0f;
volatile float g_isr_us    = 0.0f;

void tim_delay_us(uint32_t us)
{
    if (us == 0) return;
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

void Scheduler_Init(uint32_t tick_us)
{
    g_tick_us = (tick_us == 0) ? 1 : tick_us;
    memset(tasks, 0, sizeof(tasks));
}

int Scheduler_AddTask(TaskFunc func, uint32_t hz, uint8_t repeat)
{
    uint32_t period_us = (hz > 0) ? (1000000U / hz) : g_tick_us;
    if (period_us < g_tick_us) period_us = g_tick_us;
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        if (tasks[i].func == 0) {
            tasks[i].func = func;
            tasks[i].period_us = period_us;
            tasks[i].remaining_us = period_us;
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
    static uint32_t idle_ticks  = 0;
    static uint32_t total_ticks = 0;

    uint32_t start = DWT->CYCCNT;
    uint8_t ran_any = 0;

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
            tim_delay_us(TASK_YIELD_US);
            ran_any = 1;
        }
    }

    uint32_t elapsed = DWT->CYCCNT - start;
    total_ticks += elapsed;
    if (!ran_any)
        idle_ticks += elapsed;

    // Update CPU usage every ~100ms
    if (total_ticks >= SystemCoreClock / 10) {
        g_cpu_usage = 100.0f * (1.0f - (float)idle_ticks / (float)total_ticks);
        idle_ticks  = 0;
        total_ticks = 0;
    }
}
