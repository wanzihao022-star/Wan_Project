/**
*@简介：用TIM11计时器来实现微妙计时
*@作者：AI
*/

#include "delay.h"


/*
 *
 *函数前必须有HAL_TIM_Base_Start_IT(&htimx);的启动，必须是中断模式的启动
必须要有周期是1MHz的定时器中断，也就是每1us触发一次中断，才能保证us级的计时精度
预分频为167（最高主频减1），ARR为65535

*/
static volatile uint64_t g_us = 0;//全局us计时


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM11)
    {
        g_us += 0x10000;   // 16位定时器溢出补偿(0x10000 = 65536 =2的16次方，0~65535一共65536个数)
    }
}

uint64_t gx_GetUs(void)
{
    uint64_t base;
    uint32_t cnt;

    __disable_irq();

    base = g_us;
    cnt  = __HAL_TIM_GET_COUNTER(&htim11);

    // ★关键：若此刻已经溢出但中断还没执行
    if (__HAL_TIM_GET_FLAG(&htim11, TIM_FLAG_UPDATE))
    {
        base += 0x10000;
        cnt  = __HAL_TIM_GET_COUNTER(&htim11); // 重新读一次 CNT
    }

    __enable_irq();

    return base + cnt;
}


