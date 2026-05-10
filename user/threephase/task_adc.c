#include "task_adc.h"
#include "user.h"
#include "hardware_def.h"

extern volatile uint8_t  g_adc_data_ready;

void Task_ADC_Process(void)
{
    if (!g_adc_data_ready)
        return;
    g_adc_data_ready = 0;

    I_line[0] = (float)g_il1 * current_const;
    I_line[1] = (float)g_il2 * current_const;
    I_line[2] = (float)g_il3 * current_const;

    uint16_t udc_raw = adc2_voltage_buffer[0];
    uint16_t uab_raw = adc2_voltage_buffer[1];
    uint16_t uac_raw = adc2_voltage_buffer[2];
    uint16_t ubc_raw = adc2_voltage_buffer[3];

    U_line[0] = (float)(udc_raw - 2048) * U_coefficient;
    U_line[1] = (float)(uab_raw - 2048) * U_coefficient;
    U_line[2] = (float)(uac_raw - 2048) * U_coefficient;
    U_line[3] = (float)(ubc_raw - 2048) * U_coefficient;
}
