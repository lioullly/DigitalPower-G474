#include "vofa.h"
#include "user.h"
#include "task_protect.h"
#include "timebase_scheduler.h"
#include "hardware_def_1p.h"

// 单相 VOFA: 完整环路信号链
//  电压外环             电流内环              调制
// Ch0:Uac  Ch1:Udc   Ch2:Iref_rms  Ch3:Iref_inst  Ch4:IL1  Ch5:Ierr  Ch6:Vctrl  Ch7:m  Ch8:ISR_kHz  Ch9:Fault
void vofa_capture_1p(void)
{
    float data[10] = {0};
    data[0] = U_line[0];              // Uac (V) — 电压环输入
    data[1] = U_line[1];              // Udc (V) — 电压环反馈
    data[2] = g_dbg_iref;             // iref (Arms) — 电压环输出
    data[3] = g_dbg_iref_inst;        // i_ref_inst (A) — 电流环给定
    data[4] = I_line[0];              // IL1 (A) — 电流环反馈
    data[5] = g_dbg_err;              // i_err (A) — 电流环误差
    data[6] = g_dbg_vctrl;            // v_ctrl (V) — PR 输出
    data[7] = g_dbg_m;                // m — 调制比
    data[8] = (float)(g_il1);   // IL1 raw ADC (g_il1 = raw - 2050)
    data[9] = (float)g_fault_code;    // 故障码
    vofa_send_frame(data);
}
