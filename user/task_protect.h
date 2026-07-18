#ifndef __TASK_PROTECT_H
#define __TASK_PROTECT_H

#include "main.h"

// --- 故障码 ---
// 公共过流 (0-3)
#define FAULT_NONE      0
#define FAULT_OC_IL1    1   // I_line[0] 过流
#define FAULT_OC_IL2    2   // I_line[1] 过流 (预留)
#define FAULT_OC_IL3    3   // I_line[2] 过流 (预留)
// DCDC 电压 (5-6)
#define FAULT_DCDC_OV   5   // DCDC 过压
#define FAULT_DCDC_UV   6   // DCDC 欠压
// 单相母线电压 (13-14)
#define FAULT_OV        13  // DC 母线过压
#define FAULT_UV        14  // DC 母线欠压
// 三相电压 (33-36)
#define FAULT_3P_OV_A   33  // A 相过压
#define FAULT_3P_OV_B   34  // B 相过压
#define FAULT_3P_OV_C   35  // C 相过压
#define FAULT_3P_OV_BUS 36  // 三相母线过压

// --- 保护使能 mask (g_protect_mask) ---
#define PROT_IL1_OC   (1<<0)  // I_line[0] 过流
#define PROT_IL2_OC   (1<<1)  // I_line[1] 过流 (预留)
#define PROT_IL3_OC   (1<<2)  // I_line[2] 过流 (预留)
#define PROT_ADC2_R1_OV   (1<<3)  // U_line[0] 过压
#define PROT_ADC2_R2_OV   (1<<4)  // U_line[1] 过压 (母线)
#define PROT_ADC2_R3_OV   (1<<5)  // U_line[2] 过压
#define PROT_ADC2_R4_OV   (1<<6)  // U_line[3] 过压
#define PROT_ADC2_R2_UV   (1<<7)  // U_line[1] 欠压 (母线)

// AC/DC 标记: 对应位=1 表示交流 (OV 检查时取绝对值), =0 表示直流
#define PROT_ADC2_R1_AC   (1<<3)   // U_line[0] 为交流
#define PROT_ADC2_R2_AC   (1<<4)   // U_line[1] 交流标记 (通常不设)
#define PROT_ADC2_R3_AC   (1<<5)   // U_line[2] 为交流
#define PROT_ADC2_R4_AC   (1<<6)   // U_line[3] 为交流

extern uint16_t g_protect_mask;
extern uint16_t g_protect_ac_mask;
extern uint8_t  g_fault_code;

void Task_Protect_Run(void);

#endif
