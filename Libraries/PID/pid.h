/*
* pid.h
 *
 *  Created on: 2025年5月7日
 *      Author: gaoxu
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct
{
    float Kp; // 比例系数
    float Ki; // 积分项的系数
    float Kd; // 微分项的系数
    float SP; // 用户的设定值

    uint64_t t_k_1; // t[k-1]，上次运行PID的时间
    float err_k_1; // err[k-1]，上次运行PID时的误差
    float err_int_k_1; // err_int[k-1]，上次运行的积分值

    // --- 新增滤波相关变量 ---
    float COd_1;   // 存储上一次滤波后的微分项结果
    float alpha;        // 滤波系数 (取值范围 0.0 到 1.0)
    // -----------------------


    float UpperLimit; // 上限
    float LowerLimit; // 下限
}PID_TypeDef;

void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void PID_ChangeSP(PID_TypeDef *PID, float SP);
float PID_Compute(PID_TypeDef *PID, float FB);
void PID_LimitConfig(PID_TypeDef *PID, float Upper, float Lower);
void PID_Reset(PID_TypeDef *PID);
void PID_Set_Alpha(PID_TypeDef *PID, float alpha);



#endif /* INC_PID_H_ */
