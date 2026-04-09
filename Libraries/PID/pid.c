/**
*@简介：PID库
*@作者：高旭
*/


#include "pid.h"

#include <math.h>

#include "delay.h"
//
// @简介：对PID控制器进行初始化
// @参数 Kp - 比例系数
// @参数 Ki - 积分系数
// @参数 Kd - 微分系数
//
void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd )
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->SP = 0.0f;

	PID->t_k_1 = 0;
	PID->err_k_1 = 0.0f;
	PID->err_int_k_1 = 0.0f;

	PID->COd_1 = 0.0f;
	PID->alpha = 0.2f; // 默认滤波系数，越大越平滑，响应越慢
	PID->UpperLimit = +3.4e+38f;
	PID->LowerLimit = -3.4e+38f;
}
void PID_Set_Alpha(PID_TypeDef *PID, float alpha) {
	PID->alpha = alpha;
}
//
// @简介：设置PID控制器门限（pid控制的是加在电机两端的电压）
// @参数 Upper - 上限值（输出电压的最大值）
// @参数 Lower - 下限值
//
void PID_LimitConfig(PID_TypeDef *PID, float Upper, float Lower)
{
	PID->UpperLimit = Upper;
	PID->LowerLimit = Lower;
}

//
// @简介：改变设定值SP
// @参数 SP - 新的设定值
//
void PID_ChangeSP(PID_TypeDef *PID, float SP)
{
	PID->SP = SP;
}



//
// @简介：执行一次PID运算
// @参数 FB - 反馈的值，也就是传感器采回的值
// @返回值：PID控制器计算的结果
//
float PID_Compute(PID_TypeDef *PID, float FB)
{
	float err = PID->SP - FB;

	uint64_t t_k = gx_GetUs();

	float deltaT = (t_k - PID->t_k_1)* 1.0e-6f;
	if(deltaT < 1.0e-6f) deltaT = 1.0e-6f; // 至少保证有 1us 的步长

	//首次运行时忽略积分项和微分项
	float err_dev = 0.0f;
	float err_int = 0.0f;


	if(PID->t_k_1 != 0)
	{
		err_dev = (err - PID->err_k_1) / deltaT;
		//这是定速控制的，不需要积分分离
		err_int = PID->err_int_k_1 + (err + PID->err_k_1) * deltaT * 0.5f;

		// //定位置控制时，需要加入积分分离
		//阈值的大小，先用pd控制器测几次稳态，误差最大值设为阈值
		// if ( fabsf(err) < 积分分离阈值) {
		// 	err_int = PID->err_int_k_1 + (err + PID->err_k_1) * deltaT * 0.5f;
		// }else {
		// 	err_int = 0;
		// }
	}

	float COp = PID->Kp * err;
	float COi = PID->Ki * err_int;
	float COd = (1 - PID->alpha) * (PID->Kd * err_dev) + PID->alpha * PID->COd_1;
	float CO = COp + COi + COd;

	// 更新
	PID->COd_1 = COd;
	PID->t_k_1 = t_k;
	PID->err_k_1 = err;
	PID->err_int_k_1 = err_int;

	// 输出限幅
	if(CO > PID->UpperLimit) CO = PID->UpperLimit;
	if(CO < PID->LowerLimit) CO = PID->LowerLimit;

	// 积分限幅
	if(PID->err_int_k_1 > PID->UpperLimit) PID->err_int_k_1 = PID->UpperLimit;
	if(PID->err_int_k_1 < PID->LowerLimit) PID->err_int_k_1 = PID->LowerLimit;

	// //输出限幅
	// //当误差小于2的时候不输出（p，d项太大了，一点误差都会让它走很多）
	// if (fabsf(err) <= 1.5f) {
	// 	CO = 0.0f;
	// }

	return CO;
}

//
// @简介：对PID控制器进行复位
//
void PID_Reset(PID_TypeDef *PID)
{
	PID->t_k_1 = 0;
	PID->err_k_1 = 0.0f;
	PID->err_int_k_1 = 0.0f;
	PID->COd_1 = 0.0f;

}