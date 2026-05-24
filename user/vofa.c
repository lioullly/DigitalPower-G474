#include "vofa.h"
#include "user.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern float g_freq_est;

static char vofa_buf[256];

// Ch0:Udc  Ch1:IL1  Ch2:Uab  Ch3:Urms  Ch4:I_mag  Ch5:Freq  Ch6:DutyA  Ch7:Fault
void Task_VOFA_1P(void)
{
    extern uint8_t g_fault_code;
    // Test: integer-only, no float formatting
    int len = snprintf(vofa_buf, sizeof(vofa_buf),
        "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
        (int)U_line[0], (int)I_line[0], (int)U_line[1],
        (int)g_uab_rms, (int)I_mag, (int)g_freq_est,
        (int)(g_duty_a * 1000), (int)g_fault_code);
    HAL_UART_Transmit(&huart1, (uint8_t*)vofa_buf, len, 1);
}

void Task_VOFA_3P(void)
{
    int len = snprintf(vofa_buf, sizeof(vofa_buf),
        "%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\r\n",
        (double)U_line[0], (double)I_line[0], (double)I_line[1],
        (double)I_line[2], (double)U_line[1], (double)U_line[2],
        (double)U_line[3], (double)g_freq_est);
    HAL_UART_Transmit(&huart1, (uint8_t*)vofa_buf, len, 1);
}
