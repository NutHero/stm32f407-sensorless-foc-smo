#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"

/* LED端口定义 */
#define LED_GPIO_PORT   GPIOC
#define LED_GPIO_PIN    GPIO_PIN_13

#define LED_CLOCK_ENABLE()  do{__HAL_RCC_GPIOC_CLK_ENABLE();}while(0) //时钟使能

#define LED0(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED0 = RED */

/* LED取反定义 */
#define LED0_TOGGLE()    do{ HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN); }while(0)    

void led_init(void);                                                                            /* 初始化 */

#endif
