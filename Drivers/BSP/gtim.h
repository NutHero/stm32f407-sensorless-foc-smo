#ifndef __GTIM_H
#define __GTIM_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/ADC/adc.h"

#define TIM8_Period  2500    //定时器重装载值

void gtim_init(uint16_t pre, uint16_t pod);
extern TIM_HandleTypeDef g_tim1_npwm_chy_handle;     /* 定时器1句柄 */

#endif

