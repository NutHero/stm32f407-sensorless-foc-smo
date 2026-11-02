#ifndef __UART_H
#define __UART_H

#include "./SYSTEM/sys/sys.h"




#define UART6_BUFFER_SIZE    100 //串口可以接收的数据包尺寸。单位：字节
#define USART_TX_GPIO_PORT                  GPIOC
#define USART_TX_GPIO_PIN                   GPIO_PIN_6
#define USART_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define USART_RX_GPIO_PORT                  GPIOC
#define USART_RX_GPIO_PIN                   GPIO_PIN_7
#define USART_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define USART_UX                            USART6
#define USART_UX_IRQn                       USART6_IRQn
#define USART_UX_IRQHandler                 USART6_IRQHandler
#define USART_UX_CLK_ENABLE()               do{ __HAL_RCC_USART6_CLK_ENABLE(); }while(0)  /* USART1 时钟使能 */
#define GPIO_ALTERNATE  GPIO_AF8_USART6

extern uint8_t g_uart6_rxbuffer[UART6_BUFFER_SIZE];//接收电脑发送来的数据
extern uint8_t g_uart6_rx_flag;//串口接收数据完成标志位,接收完成后 变量 置 1
extern uint8_t g_uart6_rx_dat_temp;//临时保存串口接收的数据
extern UART_HandleTypeDef g_uart6_handle;//定义句柄结构体变量

void usart6_init(uint32_t baduratr);//串口初始化

/* ---------------- 串口3（PB10=TX，PB11=RX） ---------------- */
#define UART3_BUFFER_SIZE                   100
#define USART3_TX_GPIO_PORT                 GPIOB
#define USART3_TX_GPIO_PIN                  GPIO_PIN_10
#define USART3_TX_GPIO_CLK_ENABLE()         do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)
#define USART3_RX_GPIO_PORT                 GPIOB
#define USART3_RX_GPIO_PIN                  GPIO_PIN_11
#define USART3_RX_GPIO_CLK_ENABLE()         do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)
#define USART3_UX                           USART3
#define USART3_UX_IRQn                      USART3_IRQn
#define USART3_UX_IRQHandler                USART3_IRQHandler
#define USART3_UX_CLK_ENABLE()              do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)
#define USART3_GPIO_ALTERNATE               GPIO_AF7_USART3

extern UART_HandleTypeDef g_uart3_handle;                   //串口3句柄
extern uint8_t g_uart3_rxbuffer[UART3_BUFFER_SIZE];         //串口3接收缓冲
extern volatile uint16_t g_uart3_rx_len;                    //本次接收长度
extern volatile uint8_t  g_uart3_rx_flag;                   //接收完成标志

void usart3_init(uint32_t baudrate);                        //串口3初始化
void usart3_printf(const char *fmt, ...);                   //串口3格式化发送

#endif
