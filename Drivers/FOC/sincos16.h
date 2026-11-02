#ifndef __SINCOS16_H
#define __SINCOS16_H

#include "./SYSTEM/sys/sys.h"

#define fToQ15(x)    ((float)x * 32767)  //浮点数转Q15定点数
    
typedef struct
{
  int16_t hCos;
  int16_t hSin;
} Trig_Components;

Trig_Components MCM_Trig_Functions(  int16_t hAngle );
float q15_to_float(int16_t q15_data);
int16_t float_to_q15(float f);

#endif
