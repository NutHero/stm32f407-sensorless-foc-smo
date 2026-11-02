#ifndef __SPWM_H
#define __SPWM_H

#include "./SYSTEM/sys/sys.h"

void set_spwm(float uq, float ud, float angle, uint16_t pwm_period, uint16_t ccrs[3]); //SPWM算法
void set_spwm_q15(int16_t uq, int16_t ud, int16_t angle, uint16_t pwm_period, uint16_t ccrs[3]); //SPWM算法，Q15格式实现

#endif
