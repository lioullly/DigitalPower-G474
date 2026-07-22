#include "vofa.h"
#include "user.h"
#include "task_protect.h"
#include "timebase_scheduler.h"
#include "hardware_def_3p.h"

// 三相 VOFA: 10 通道 JustFloat 协议
// Ch0: IL1 瞬时     Ch1: IL2 瞬时     Ch2: IL3 瞬时
// Ch3: Uab 瞬时     Ch4: Ubc 瞬时     Ch5: Udc 母线
// Ch6: Uab RMS      Ch7: Ubc RMS      Ch8: fault
// Ch9: CPU% / ISR kHz (交替)
void vofa_capture_3p(void)
{
    float data[10];
    data[0] = I_line[0];          // IL1 瞬时
    data[1] = I_line[1];          // IL2 瞬时
    data[2] = I_line[2];          // IL3 瞬时
    data[3] = U_line[0];          // Uab 瞬时
    data[4] = U_line[3];          // Ubc 瞬时
    data[5] = U_line[1];          // Udc 母线
    data[6] = g_uab_rms;          // Uab RMS
    data[7] = g_ubc_rms;          // Ubc RMS
    data[8] = (float)g_fault_code;
    data[9] = g_cpu_usage;

    vofa_send_frame(data);
}
