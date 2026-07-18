#include "user.h"
#include "hardware_def_dcdc.h"

// Buck ADC 预处理 @ 50kHz (ADC ISR 中调用)
// 电压全部 DC 无偏置, 供保护和 VOFA 使用
void adc_preproc_buck(void)
{
    U_line[0] = (float)(adc2_voltage_buffer[BUCK_ADC_VIN_BUF]  - 0) * U_coefficient;
    U_line[1] = (float)(adc2_voltage_buffer[BUCK_ADC_VOUT_BUF] - 0) * U_coefficient;
    U_line[2] = U_line[0];  // Vin 副本, 供保护 PROT_ADC2_R3_OV
    U_line[3] = U_line[1];  // Vout 副本, 供保护 PROT_ADC2_R4_OV
}
