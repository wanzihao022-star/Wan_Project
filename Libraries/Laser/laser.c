/**
*@简介：激光笔的简单控制
*@作者：万梓豪
*/
#include "laser.h"
#include "tim.h"

void Laser_Init() {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    Laser_Off();
}
//最高亮度工作
void Laser_On() {
  TIM3-> CCR3 = LASER_PWM_MAX;
}

void Laser_Off() {
    TIM3-> CCR3 = LASER_PWM_MIN;
}

void Laser_SetBrightness(uint16_t percent) {
    if (percent > 100) percent = 100;

    // 直接映射：0->0, 100->1000
    // 公式：CCR = percent * 10
    TIM3->CCR3 = (uint16_t)(percent * 10);
}

