#ifndef _IIC_H
#define _IIC_H

#include "./SYSTEM/sys/sys.h"

/* IIC端口定义 */
#define SDA(x)   do{ x ? \
                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); \
                  }while(0)      /* SDA翻转 */

#define SCL(x)   do{ x ? \
                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); \
                  }while(0)      /* SCL翻转 */
#define RSDA    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) /* 读SDA引脚电平 */
                  
void iic_write_byte(uint8_t dat);//写字节
uint8_t iic_read_byte(void);//读字节
void iic_stop(void);//发送停止信号
void iic_start(void);//发送开始信号
void iic_ack(void);//发送应答信号
void iic_nack(void);//发送非应答信号
uint8_t iic_wait_ack(void);//等待应答
void iic_init(void); /* 软件IIC初始化 */
              
static void iic_delay(void);//延时
                  
#endif

