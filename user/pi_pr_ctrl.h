#ifndef __PI_PR_CTRL_H
#define __PI_PR_CTRL_H

#define ARM_MATH_CM4
#include "arm_math.h"

void Clarke_Transform(float ia, float ib, float ic, float *alpha, float *beta);
void Inverse_Clarke_Transform(float alpha, float beta, float *va, float *vb, float *vc);

typedef struct
{
	float x0, x1;
	float Ts;
	float Ki;
	float y0, y1;
	float Integral_MAX;
} Integral_TypeDef;

typedef struct
{
	float x0, x1;
	float Kp, Ki ,Ts;
	float y0, y1;
	float filter_B1, filter_B2;
	float TH;
	float TL;
} PI_TypeDef;

// PR 控制器结构体（使用二阶带通/双二阶近似共振）
typedef struct
{
    float x0, x1, x2;    // 输入状态
    float y0, y1, y2;    // 输出状态
    float Kp;            // P项系数
    float Kr;            // PR（谐振）增益
    float f0;            // 共振中心频率（Hz）
    float BW;            // 带宽（Hz）
    float Fs;            // 采样频率（Hz）
    // 归一化后的双二阶系数
    float b0, b1, b2;
    float a1, a2;        // a0已经归一化为1
    int16_t TH;          // 输出上限
    int16_t TL;          // 输出下限
} PR_TypeDef;

static inline float MyFmod(float _X, float _Y)
{
    return _X - (int16_t)(_X / _Y) * _Y;
}

void f32_Integral_Init(Integral_TypeDef *I_pamer, float _Ts, float Intergral_MAX); //浮点积分初始化
float f32_Integral_Calculate(Integral_TypeDef *I_pamer, float _X); //浮点积分运算

void f32_PI_Init(PI_TypeDef *PI_Pamer, float _Ts, float _Kp, float _Ki, float _TH, float _TL);
float f32_PI_Calculate(PI_TypeDef *PI_Pamer, float REF, float Sample);

// PR 控制器接口
void f32_PR_Init(PR_TypeDef *PR_Pamer, float _Kp, float _Kr, float _f0, float _BW, float _Fs, int16_t _TH, int16_t _TL);
float f32_PR_Calculate(PR_TypeDef *PR_Pamer, float ERR);

void ABC_to_DQ(float DQ[2], float ABC[3], float _wt);
void DQ_to_ABC(float DQ[2], float ABC[3], float _wt);
void Cal_Sin_Cos(float _theta[3], int16_t _Sin[3], int16_t _Cos[3]);

// 希尔伯特变换 — 单相→正交(α-β)对
// 一阶全通滤波器，在 f0 处产生精确 90° 相移
typedef struct
{
    float k;         // 全通滤波器系数
    float x1;        // 上一拍输入
    float y1;        // 上一拍输出
    float f0;        // 设计频率 [Hz]
    float Fs;        // 采样频率 [Hz]
} Hilbert_TypeDef;

void f32_Hilbert_Init(Hilbert_TypeDef *H, float _f0, float _Fs);
void f32_Hilbert_Calculate(Hilbert_TypeDef *H, float input, float *alpha, float *beta);

// 2阶陷波器 — 滤除 f0 频率分量（用于 DC bus 100Hz 纹波抑制）
typedef struct
{
    float b0, b1, b2;    // 分子系数
    float a1, a2;        // 分母系数 (a0 归一化为1)
    float x1, x2;        // 输入状态
    float y1, y2;        // 输出状态
} Notch_TypeDef;

void f32_Notch_Init(Notch_TypeDef *N, float f0, float Q, float Fs);
float f32_Notch_Calculate(Notch_TypeDef *N, float input);

// Type-II 补偿器 (PI + 高频极点), 复用 PR 结构体
void f32_Type2_Init(PR_TypeDef *C, float fz, float fp, float gain, float Fs);
float f32_Type2_Calculate(PR_TypeDef *C, float error);

#endif
