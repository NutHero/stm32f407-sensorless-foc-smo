#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "bldc_control.h"
#include "quen.h"

extern ADC_HandleTypeDef g_adc_injected_handle;
extern uint16_t adc_value[3]; 

void adc_injected_init(void);

#endif 





