#ifndef __TASK_PROTECT_H
#define __TASK_PROTECT_H

#include "main.h"

// --- 故障码 ---
#define FAULT_NONE      0
#define FAULT_OC_IL1    1   // I_line[0] 过流
#define FAULT_OV        13  // 过压
#define FAULT_UV        14  // 欠压

// --- 保护使能 mask (g_protect_mask) ---
#define PROT_IL1_OC   (1<<0)  // IL1 过流
#define PROT_UAC_OV   (1<<1)  // Uac 过压 (U_line[0], AC)
#define PROT_UDC_OV   (1<<2)  // Udc 过压 (U_line[1], DC bus)
#define PROT_UDC_UV   (1<<3)  // Udc 欠压 (U_line[1], DC bus)

extern uint16_t g_protect_mask;
extern uint8_t  g_fault_code;

void Task_Protect_Run(void);

#endif
