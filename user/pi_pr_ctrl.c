#include "PI_PR_CTRL.h"
#include "arm_math.h"

#define COEFF_ALPHA_BETA  0.6666666666666666f   // 2/3
#define COEFF_SQRT3_HALF  0.8660254037844386f   // √3/2

// 标准 Clarke 变换（支持不平衡负载）
void Clarke_Transform(float ia, float ib, float ic, float *alpha, float *beta)
{
    *alpha = COEFF_ALPHA_BETA * (ia - 0.5f * ib - 0.5f * ic);
    *beta  = COEFF_ALPHA_BETA * (COEFF_SQRT3_HALF * ib - COEFF_SQRT3_HALF * ic);
}

// 标准逆 Clarke 变换
void Inverse_Clarke_Transform(float alpha, float beta, float *va, float *vb, float *vc)
{
    *va = alpha;
    *vb = -0.5f * alpha + COEFF_SQRT3_HALF * beta;
    *vc = -0.5f * alpha - COEFF_SQRT3_HALF * beta;
}

void Cal_Sin_Cos(float _theta[3], int16_t _Sin[3], int16_t _Cos[3])
{
	int16_t theta_Q15[3];	// 定义一个整型的输入数组，用于存放三个角度值的Q15格式
	
	theta_Q15[0] = _theta[0] * 0x8000;		// 将三个浮点的角度值转换为Q15格式
	theta_Q15[1] = _theta[1] * 0x8000;
	theta_Q15[2] = _theta[2] * 0x8000;
	
	_Sin[0] = arm_sin_q15(theta_Q15[0]);	// 计算三相正弦值
	_Sin[1] = arm_sin_q15(theta_Q15[1]);
	_Sin[2] = arm_sin_q15(theta_Q15[2]);
	
	_Cos[0] = arm_cos_q15(theta_Q15[0]);	// 计算三相正弦值
	_Cos[1] = arm_cos_q15(theta_Q15[1]);
	_Cos[2] = arm_cos_q15(theta_Q15[2]);
}


// ABC到DQ变换，输入wt的范围为归一化的[0~1]
void ABC_to_DQ(float DQ[2], float ABC[3], float _wt) // 定义一个函数，用于进行ABC到DQ变换，参数为DQ分量数组、ABC分量数组、归一化角度wt
{
	float theta[3];		// 定义一个浮点型的数组，用于存放三个角度值
	int16_t Sin[3];		// 定义一个整型的输出数组，用于存放三个正弦值的Q15格式
	int16_t Cos[3];		// 定义一个整型的输出数组，用于存放三个余弦值的Q15格式

	theta[0] = _wt;							 	// 将归一化角度wt赋值给数组的第一个元素
	theta[1] = MyFmod(_wt + 0.666667f, 1.0f);	// 将归一化角度wt加上2/3，对1取模，赋值给数组的第二个元素
	theta[2] = MyFmod(_wt + 0.333333f, 1.0f);	// 将归一化角度wt加上1/3，对1取模，赋值给数组的第三个元素
	
	Cal_Sin_Cos(theta, Sin, Cos);	// 通过FPU计算正弦余弦结果
	
	DQ[0] = (float)0.6667f * (Sin[0] * ABC[0] + Sin[1] * ABC[1] + Sin[2] * ABC[2]) / 0x8000; // 根据ABC到DQ变换公式计算D分量，使用FPU输出数组中的余弦值，最后除以0x8000，转换为浮点型
	DQ[1] = (float)0.6667f * (Cos[0] * ABC[0] + Cos[1] * ABC[1] + Cos[2] * ABC[2]) / 0x8000; // 根据ABC到DQ变换公式计算Q分量，使用FPU输出数组中的正弦值，最后除以0x8000，转换为浮点型
}

// DQ到ABC变换，输入wt的范围为归一化的[0~1]
void DQ_to_ABC(float DQ[2], float ABC[3], float _wt) // 定义一个函数，用于进行ABC到DQ变换，参数为DQ分量数组、ABC分量数组、归一化角度wt
{
	float theta[3];		// 定义一个浮点型的数组，用于存放三个角度值
	int16_t Sin[3];		// 定义一个整型的输出数组，用于存放三个正弦值的Q15格式
	int16_t Cos[3];		// 定义一个整型的输出数组，用于存放三个余弦值的Q15格式
	
	theta[0] = _wt;							 	// 将归一化角度wt赋值给数组的第一个元素
	theta[1] = MyFmod(_wt + 0.666667f, 1.0f);	// 将归一化角度wt加上2/3，对1取模，赋值给数组的第二个元素
	theta[2] = MyFmod(_wt + 0.333333f, 1.0f);	// 将归一化角度wt加上1/3，对1取模，赋值给数组的第三个元素
	
	Cal_Sin_Cos(theta, Sin, Cos);	// 通过FPU计算正弦余弦结果
	
	ABC[0] = (float)(Sin[0] * DQ[0] + Cos[0] * DQ[1]) / 0x8000;		// 根据DQ到ABC变换公式计算A分量，使用FPU输出数组中的余弦值，最后除以0x8000，转换为浮点型
	ABC[1] = (float)(Sin[1] * DQ[0] + Cos[1] * DQ[1]) / 0x8000;		// 根据DQ到ABC变换公式计算B分量，使用FPU输出数组中的余弦值，最后除以0x8000，转换为浮点型
	ABC[2] = (float)(Sin[2] * DQ[0] + Cos[2] * DQ[1]) / 0x8000;		// 根据DQ到ABC变换公式计算C分量，使用FPU输出数组中的余弦值，最后除以0x8000，转换为浮点型
}

// ----------------- PR 控制器实现 -----------------
void f32_PR_Init(PR_TypeDef *PR_Pamer, float _Kp, float _Kr, float _f0, float _BW, float _Fs, int16_t _TH, int16_t _TL)
{
    PR_Pamer->Kp = _Kp;
    PR_Pamer->Kr = _Kr;
    PR_Pamer->f0 = _f0;
    PR_Pamer->BW = _BW;
    PR_Pamer->Fs = _Fs;
    PR_Pamer->x0 = PR_Pamer->x1 = PR_Pamer->x2 = 0.0f;
    PR_Pamer->y0 = PR_Pamer->y1 = PR_Pamer->y2 = 0.0f;
    PR_Pamer->TH = _TH;
    PR_Pamer->TL = _TL;

    float w0 = 2.0f * (float)PI * _f0 / _Fs;
    float Q = _f0 / _BW;
    if (Q <= 0.0f) Q = 1.0f;
    float alpha = arm_sin_f32(w0) / (2.0f * Q);

    float b0 = alpha;
    float b1 = 0.0f;
    float b2 = -alpha;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * arm_cos_f32(w0);
    float a2 = 1.0f - alpha;

    PR_Pamer->b0 = b0 / a0;
    PR_Pamer->b1 = b1 / a0;
    PR_Pamer->b2 = b2 / a0;
    PR_Pamer->a1 = a1 / a0;
    PR_Pamer->a2 = a2 / a0;
}

float f32_PR_Calculate(PR_TypeDef *PR_Pamer, float ERR)
{
    float x0 = ERR;
    float y0 = PR_Pamer->b0 * x0 + PR_Pamer->b1 * PR_Pamer->x1 + PR_Pamer->b2 * PR_Pamer->x2 - PR_Pamer->a1 * PR_Pamer->y1 - PR_Pamer->a2 * PR_Pamer->y2;

    PR_Pamer->x2 = PR_Pamer->x1;
    PR_Pamer->x1 = x0;
    PR_Pamer->y2 = PR_Pamer->y1;
    PR_Pamer->y1 = y0;

    float u = PR_Pamer->Kp * ERR + PR_Pamer->Kr * y0;
    if (u >= PR_Pamer->TH) u = PR_Pamer->TH;
    if (u <= PR_Pamer->TL) u = PR_Pamer->TL;

    return u;
}


// 定义一个函数，用于初始化积分器，参数为积分器结构体指针和采样时间、积分上限
void f32_Integral_Init(Integral_TypeDef *I_pamer, float _Ts, float Intergral_MAX)
{
	I_pamer->Ts = _Ts;					   // 将采样时间赋值给结构体中的Ts变量
	I_pamer->Ki = _Ts / 2;				   // 计算积分系数，赋值给结构体中的Ki变量
	I_pamer->Integral_MAX = Intergral_MAX; // 将积分上限赋值给结构体中的Integral_MAX变量
}

// 定义一个函数，用于计算积分器的输出，参数为积分器结构体指针和输入值
float f32_Integral_Calculate(Integral_TypeDef *I_pamer, float _X)
{
	float I_tmp;
	I_pamer->x1 = I_pamer->x0; // 将上一次的输入值赋值给结构体中的x1变量
	// I_pamer->x0 = _X + 314.16f;//积分前馈
	I_pamer->x0 = _X;												 // 将当前的输入值赋值给结构体中的x0变量
	I_tmp = I_pamer->Ki * (I_pamer->x0 + I_pamer->x1) + I_pamer->y1; // 根据积分系数和输入值计算临时的输出值
	I_pamer->y0 = MyFmod(I_tmp, I_pamer->Integral_MAX);				 // 将临时的输出值对积分上限取模，得到最终的输出值
	I_pamer->y1 = I_pamer->y0;										 // 将输出值赋值给结构体中的y1变量，用于下一次计算
	return I_pamer->y0;												 // 返回输出值
}

// 定义一个函数，用于初始化PI控制器，参数为PI结构体指针和采样时间、比例系数、积分系数、输出上下限
void f32_PI_Init(PI_TypeDef *PI_Pamer, float _Ts, float _Kp, float _Ki, int16_t _TH, int16_t _TL)
{
	PI_Pamer->Ts = _Ts;							// 将采样时间赋值给结构体中的Ts变量
	PI_Pamer->Kp = _Kp;							// 将比例系数赋值给结构体中的Kp变量
	PI_Pamer->Ki = _Ki;							// 将积分系数赋值给结构体中的Ki变量
	PI_Pamer->filter_B1 = _Kp + _Ts * _Ki / 2;	// 计算滤波器参数B1的值，表示PI控制器中的一个系数
	PI_Pamer->filter_B2 = -_Kp + _Ts * _Ki / 2; // 计算滤波器参数B2的值，表示PI控制器中的一个系数
	PI_Pamer->TH = _TH;							// 将输出上限赋值给结构体中的TH变量
	PI_Pamer->TL = _TL;							// 将输出下限赋值给结构体中的TL变量
}

// 定义一个函数，用于计算PI控制器的输出，参数为PI结构体指针和参考值、采样值
float f32_PI_Calculate(PI_TypeDef *PI_Pamer, float REF, float Sample)
{
	PI_Pamer->x1 = PI_Pamer->x0;   // 将上一次的误差赋值给结构体中的x1变量
	PI_Pamer->x0 = (REF - Sample); // 将当前的误差赋值给结构体中的x0变量

	float tmp_co;
	// tmp_co = PI_Pamer->y1 + PI_Pamer->Kp*(PI_Pamer->x0-PI_Pamer->x1) + PI_Pamer->Ts*PI_Pamer->Ki*(PI_Pamer->x0+PI_Pamer->x1)/2;
	tmp_co = PI_Pamer->y1 + PI_Pamer->filter_B1 * PI_Pamer->x0 + PI_Pamer->filter_B2 * PI_Pamer->x1; // 根据滤波器参数和误差计算临时的输出值
	if (tmp_co >= PI_Pamer->TH)																		 // 如果临时的输出值大于等于输出上限
		PI_Pamer->y0 = PI_Pamer->TH;																 // 将输出值设为输出上限
	else if (tmp_co <= PI_Pamer->TL)																 // 如果临时的输出值小于等于输出下限
		PI_Pamer->y0 = PI_Pamer->TL;																 // 将输出值设为输出下限
	else																							 // 否则
		PI_Pamer->y0 = tmp_co;																		 // 将输出值设为临时的输出值

	PI_Pamer->y1 = PI_Pamer->y0; // 将输出值赋值给结构体中的y1变量，用于下一次计算
	return PI_Pamer->y0;		 // 返回输出值
}
