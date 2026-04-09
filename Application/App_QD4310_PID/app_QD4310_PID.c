//
// Created by gaoxu on 2026/4/5.
//

#include "app_QD4310_PID.h"
#include <math.h>
#include "delay.h"
#include "irqHandlers.h"
#include "task.h"
#include "QD4310.h"
#include "vision_protocol.h"


extern QD4310_t YawMotor;
extern QD4310_t PitchMotor;

uint8_t valid = 0;
float Kp_pitch = 0;
float Ki_pitch = 0;
float Kd_pitch = 0;

float Kp_yaw = 0;
float Ki_yaw = 0;
float Kd_yaw = 0;

volatile float laser_current_yaw = 0;//激光笔现在的水平坐标
volatile float laser_current_yaw_target = 0;//设定的水平目标
volatile float laser_current_pitch = 0;//现在的竖直位置
volatile float laser_current_pitch_target = 0;//设定的竖直位置

static PID_TypeDef pid_motor_pitch; // pitch电机调速系统的PID控制器
static PID_TypeDef pid_motor_yaw; // yaw调速系统的PID控制器

void QD4310_PID_Reset(void) {
    PID_Reset(&pid_motor_pitch);
    PID_Reset(&pid_motor_yaw);
}

void QD4310_PID_Init(void) {
    CAN_InterfaceInit(); //CAN初始化
    QD4310_Init(&YawMotor, &hcan1, 0x00); // 初始化云台yaw电机
    QD4310_Init(&PitchMotor, &hcan1, 0x01);//初始化pitch轴
    HAL_Delay(100);
    // QD4310_Enable(&YawMotor);// 使能电机
    // QD4310_Enable(&PitchMotor);
    HAL_Delay(20);

    Kp_yaw = 0.2;
    Ki_yaw = 0;
    Kd_yaw = 0.001;

    Kp_pitch = 0.17;
    Kd_pitch = 0;
    Kd_pitch = 0;

    PID_Init(&pid_motor_pitch, Kp_pitch, Ki_pitch, Kd_pitch);
    PID_LimitConfig(&pid_motor_pitch, +30.0f, -30.0f);
    PID_Init(&pid_motor_yaw, Kp_yaw,Ki_yaw , Kd_pitch);
    PID_LimitConfig(&pid_motor_yaw, +30.0f, -30.0f);
}
void QD4310_PID_Pro(void) {
    PERIODIC_START(QD4310PID, 2)
    QD4310_Vaild_Update();
    QD4310_PID_Update_yaw(&pid_motor_yaw);//先更新yaw轴和pitch轴的数据
    QD4310_PID_Update_pitch(&pid_motor_pitch);
    //再进行pid计算
    if (valid == 0) {
        //如果没识别到矩形框，视为无效。在此时将电机速度设为0并且复位pid，防止之前的积分项影响下一次目标识别
        QD4310_SetSpeed(&YawMotor,0);
        QD4310_SetSpeed(&PitchMotor,0);
        PID_Reset(&pid_motor_yaw);
        PID_Reset(&pid_motor_pitch);
    }else {
        float yaw_speed = PID_Compute_YAW(&pid_motor_yaw, laser_current_yaw);//这个的pid加上了输出限幅
        QD4310_SetSpeed(&YawMotor,yaw_speed);

        float pitch_speed = PID_Compute(&pid_motor_pitch, laser_current_pitch);
        QD4310_SetSpeed(&PitchMotor,pitch_speed);
    }
    PERIODIC_END
}



void QD4310_Vaild_Update(void) {
    __disable_irq();
    valid = g_vision.valid;//有效值，0代表相机未识别到图形
    __enable_irq();
}
void QD4310_PID_Update_yaw(PID_TypeDef *PID) {
    __disable_irq();
    laser_current_yaw = g_vision.dx;
    __enable_irq();
    laser_current_yaw_target = 0.0f;//视觉发送的坐标

    PID->SP = laser_current_yaw_target;
}
void QD4310_PID_Update_pitch(PID_TypeDef *PID) {
    __disable_irq();
    laser_current_pitch = g_vision.dy;
    __enable_irq();

    laser_current_pitch_target = 0.0f;

    PID->SP = laser_current_pitch_target;
}

 //
// @简介：执行一次PID运算
// @参数 FB - 反馈的值，也就是传感器采回的值
// @返回值：PID控制器计算的结果
//
float PID_Compute_YAW(PID_TypeDef *PID, float FB)
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

    //当误差小于2的时候不输出（p，d项太大了，一点误差都会让它走很多）
    if (fabsf(err) <= 1.5f) {
        CO = 0.0f;
    }

    return CO;
}



