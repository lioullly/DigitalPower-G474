#include "vofa.h"
#include "user.h"
#include "task_protect.h"
#include "timebase_scheduler.h"
#include "hardware_def_1p.h"

// 单相 VOFA: 10 通道 JustFloat 协议
// Ch0: IL1 瞬时   Ch1: IL1 RMS   Ch2: Uab 瞬时   Ch3: Uab RMS
// Ch4: Udc 母线   Ch5: fault     Ch6: v_ctrl     Ch7: i_err
// Ch8: CPU%       Ch9: ISR kHz
void vofa_capture_1p(void)
{
    float data[10];
    data[0] = I_line[0];
    data[1] = g_irms;
    data[2] = U_line[0];
    data[3] = g_uab_rms;
    data[4] = U_line[1];
    data[5] = (float)g_fault_code;
    data[6] = g_dbg_vctrl;
    data[7] = g_dbg_err;
    data[8] = g_cpu_usage;
    data[9] = g_pfc_phase_deg;  // debug: 看按键是否改变了相位
    vofa_send_frame(data);
}
