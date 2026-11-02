#ifndef __SVPWM_H
#define __SVPWM_H

#include "./SYSTEM/sys/sys.h"

void set_svpwm_q15(int16_t ualpha, int16_t ubeta, uint16_t pwm_period, uint16_t ccrs[3]); //SVPWM算法，Q15定点化实现
//void set_svpwm(float ualpha, float ubeta, uint16_t pwm_period, uint16_t ccrs[3]); //SVPWM算法

#endif
