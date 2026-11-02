#ifndef __TIME_H
#define __TIME_H

#include "./SYSTEM/sys/sys.h"

#define TIM1_Period  (1400 - 1)    //定时器1重装载值
#define TIM1_PSC    2       //定时器1分频
#define TIM1_DIE_ARE    0  //死区时间
#define ADC_SAMPLE_POINT    50  //ADC采样点设置

/**
 * @brief 中心对齐模式PWM频率公式
 * F_pwm = F_clk / ((PSC+1) × 2 × (ARR+1))
 * F_clk = 84000000 Hz
 */
 
extern TIM_HandleTypeDef g_tim6_handle; //定时器6句柄
extern TIM_HandleTypeDef g_tim1_handle; //定时器1句柄

void time1_init(uint32_t pre, uint32_t pod); //定时器1初始化
void time6_init(uint32_t pre, uint32_t pod); //定时器6初始化

#endif
