#ifndef __GPIO_H
#define __GPIO_H

#include "./SYSTEM/sys/sys.h"

/* 调试引脚端口定义 */
#define DEBUG_PORT  GPIOB
#define DEBUG_PIN   GPIO_PIN_4
#define DEBUG_CLOCK_ENABLE()  do{__HAL_RCC_GPIOB_CLK_ENABLE();}while(0) //时钟使能

/* 调试引脚 */ 
#define DeBUG(x) do{ x ? \
                      HAL_GPIO_WritePin(DEBUG_PORT, DEBUG_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(DEBUG_PORT, DEBUG_PIN, GPIO_PIN_RESET); \
                  }while(0) 

/* 电机2使能端口定义 */
#define MOTOR2_EN_PORT  GPIOC
#define MOTOR2_EN_PIN   GPIO_PIN_9
#define MOTOR2_CLOCK_ENABLE()  do{__HAL_RCC_GPIOC_CLK_ENABLE();}while(0) //时钟使能

/* 电机1使能端口定义 */
#define MOTOR1_EN_PORT  GPIOA
#define MOTOR1_EN_PIN   GPIO_PIN_11
#define MOTOR1_CLOCK_ENABLE()  do{__HAL_RCC_GPIOA_CLK_ENABLE();}while(0) //时钟使能

/* 电机1使能 */
#define M1_EN(x) do{ x ? \
                      HAL_GPIO_WritePin(MOTOR1_EN_PORT, MOTOR1_EN_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(MOTOR1_EN_PORT, MOTOR1_EN_PIN, GPIO_PIN_RESET); \
                  }while(0)   
/* 电机2使能 */
#define M2_EN(x) do{ x ? \
                      HAL_GPIO_WritePin(MOTOR2_EN_PORT, MOTOR2_EN_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(MOTOR2_EN_PORT, MOTOR2_EN_PIN, GPIO_PIN_RESET); \
                  }while(0) 

void gpio_init(void); //初始化部分GPIO

#endif
