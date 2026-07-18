#include "vofa.h"
#include "user.h"
#include "task_protect.h"
#include "timebase_scheduler.h"
#include "hardware_def_dcdc.h"

// Buck VOFA: 10 通道, JustFloat 协议
// Ch0: Vin   Ch1: Vout   Ch2: IL   Ch3: Iref   Ch4: duty%
// Ch5: fault  Ch6: Vref   Ch7: unused  Ch8: CPU%  Ch9: ISR kHz
void vofa_capture_buck(void)
{
    float data[10];
    data[0] = U_line[2];               // Vin
    data[1] = U_line[3];               // Vout
    data[2] = I_line[0];               // IL
    data[3] = g_dbg_err;               // Iref
    data[4] = g_dbg_vctrl * 100.0f;    // duty %
    data[5] = (float)g_fault_code;     // fault
    data[6] = g_dbg_vctrl;             // duty (raw)
    data[7] = g_cpu_usage;
    data[8] = g_isr_khz;
    data[9] = 0.0f;
    vofa_send_frame(data);
}
