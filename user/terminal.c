#include "terminal.h"
#include "user.h"
#include "usart.h"
#include <stdio.h>

// ---- printf output redirect to USART1 (GCC/newlib) ----
#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
    return len;
}
#endif

// ---- Externs from other modules ----
extern uint8_t g_pll_locked;
extern float   g_freq_est;
extern uint8_t g_fault_code;

void Task_Terminal(void)
{
    printf("\r\n=== INVERTER STATUS ===\r\n");
    printf("Run:%d  Fault:%d  Lock:%d  Freq:%.1fHz  wt:%.3f\r\n",
           Run_Flag, g_fault_code, g_pll_locked, g_freq_est, (double)g_wt);
    printf("--- Current (raw / A) ---\r\n");
    printf("IL1:%ld(%.3fA) IL2:%ld(%.3fA) IL3:%ld(%.3fA)\r\n",
           g_il1, (double)I_line[0], g_il2, (double)I_line[1], g_il3, (double)I_line[2]);
    printf("Iref_a:%.3fA  Iref_b:%.3fA\r\n",
           (double)Iref_alpha, (double)Iref_beta);
    printf("--- Voltage (V) ---\r\n");
    printf("Udc:%.1fV  Uab:%.1fV  Uac:%.1fV  Ubc:%.1fV\r\n",
           (double)U_line[0], (double)U_line[1], (double)U_line[2], (double)U_line[3]);
    printf("Uref:%.1fV\r\n", (double)OFFGRID_UREF);
    printf("--- PWM Duty ---\r\n");
    printf("Da:%.3f  Db:%.3f  Dc:%.3f\r\n",
           (double)g_duty_a, (double)g_duty_b, (double)g_duty_c);
}
