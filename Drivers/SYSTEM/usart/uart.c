#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/uart.h"
#include "stdio.h"
#include <stdarg.h>
#include <string.h>

uint8_t g_uart6_rxbuffer[UART6_BUFFER_SIZE];//储存串口接收的数据
uint8_t g_uart6_rx_flag;//串口接收数据完成标志位,接收完成后 变量 置 1
uint8_t g_uart6_rx_dat_temp;//临时保存串口接收的数据
UART_HandleTypeDef g_uart6_handle;//定义结构体变量

/* 如果使用os,则包括下面的头文件即可. */
#if SYS_SUPPORT_OS
#include "includes.h" /* os 使用 */
#endif

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* MDK下需要重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART_UX->SR & 0X40) == 0);     /* 等待上一个字符发送完成 */

    USART_UX->DR = (uint8_t)ch;             /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif

// 函数功能：串口6初始化
// 参数 baduratr：串口波特率
void usart6_init(uint32_t baduratr)
{
    g_uart6_handle.Instance = USART_UX;//串口寄存器基地址
    g_uart6_handle.Init.BaudRate = baduratr;//波特率设置
    g_uart6_handle.Init.WordLength = UART_WORDLENGTH_8B;//每一帧数据设定为8位
    g_uart6_handle.Init.StopBits = UART_STOPBITS_1;//一个停止位
    g_uart6_handle.Init.Parity = UART_PARITY_NONE; // 无奇偶校验位 
    g_uart6_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE; //无硬件流控(监控数据的传输是否完成）   
    g_uart6_handle.Init.Mode = UART_MODE_TX_RX; //收发模式:可以接收数据和发送数据 
    /* APB2=84MHz：16倍过采样最高约 5.25M，8倍过采样最高约 10.5M */
    if(baduratr > 5250000U)
    {
        g_uart6_handle.Init.OverSampling = UART_OVERSAMPLING_8;
    }
    else
    {
        g_uart6_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    }
    HAL_UART_Init(&g_uart6_handle);//串口初始化，并使能串口   
    
    HAL_UART_Receive_IT(&g_uart6_handle, &g_uart6_rx_dat_temp, 1);//开启串口接收中断
} 

// 函数功能：时钟初始化，引脚初始化，中断开启
/**
 * @brief  串口时钟/GPIO/中断初始化
 * @param  huart 串口句柄
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct = {0};

    if(huart->Instance == USART_UX) //USART6
    {
        USART_UX_CLK_ENABLE();
        USART_TX_GPIO_CLK_ENABLE();
        USART_RX_GPIO_CLK_ENABLE();

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_ALTERNATE;
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = USART_RX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_ALTERNATE;
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);

        HAL_NVIC_SetPriority(USART_UX_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);
    }
    else if(huart->Instance == USART3_UX) //USART3
    {
        USART3_UX_CLK_ENABLE();
        USART3_TX_GPIO_CLK_ENABLE();
        USART3_RX_GPIO_CLK_ENABLE();

        gpio_init_struct.Pin = USART3_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = USART3_GPIO_ALTERNATE;
        HAL_GPIO_Init(USART3_TX_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = USART3_RX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = USART3_GPIO_ALTERNATE;
        HAL_GPIO_Init(USART3_RX_GPIO_PORT, &gpio_init_struct);

        HAL_NVIC_SetPriority(USART3_UX_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART3_UX_IRQn);
    }
}

void USART_UX_IRQHandler(void)//串口中断服务函数
{
    static uint16_t i = 0;//数据数组下标
    static uint8_t g_uart6_rx_sta = 0;//串口状态：0等待包头，1接收数据，2接收完成等待包尾
    HAL_UART_IRQHandler(&g_uart6_handle);//串口公共处理函数,调用CallBack函数,并清除串口中断标志,并接收串口发送到来的数据
    if(g_uart6_rx_dat_temp == 'S')//验证指令是不是包头
    {
        g_uart6_rx_sta = 1;//状态置1开始接收数据
    }
    else if(g_uart6_rx_dat_temp == 'E')//验证指令是不是包尾
    {
        g_uart6_rx_sta = 2;//状态置2开始准备下一轮接收
    }
    if(g_uart6_rx_sta == 1)//接收数据
    {
        g_uart6_rxbuffer[i] = g_uart6_rx_dat_temp;//储存数据
        i++;
    }
    else if(g_uart6_rx_sta == 2)//处理包尾
    {
        g_uart6_rxbuffer[i] = g_uart6_rx_dat_temp;//储存数据,储存包尾
        g_uart6_rx_flag = 1;//变量置1代表数据接收完成
        g_uart6_rx_sta = 0;//重新开始下一轮数据接收
        i = 0;
    }
    if(g_uart6_rx_flag==1)//接收完数据后根据接收到的指令单片机指向对应的程序
    {
        //g_uart6_rx_flag=0;//清除串口接收完成标志
    }
    HAL_UART_Receive_IT(&g_uart6_handle, &g_uart6_rx_dat_temp, 1);//重新开启串口接收中断    
}



/* ---------------- 串口3（PB10=TX，PB11=RX） ---------------- */
UART_HandleTypeDef g_uart3_handle;                          //串口3句柄
uint8_t g_uart3_rxbuffer[UART3_BUFFER_SIZE];                //串口3接收缓冲
volatile uint16_t g_uart3_rx_len = 0;                       //本次接收长度
volatile uint8_t  g_uart3_rx_flag = 0;                      //接收完成标志（接收完置1）

/**
 * @brief  串口3初始化（PB10=TX，PB11=RX），开启空闲中断接收
 * @param  baudrate 波特率
 */
void usart3_init(uint32_t baudrate)
{
    g_uart3_handle.Instance = USART3_UX;
    g_uart3_handle.Init.BaudRate = baudrate;
    g_uart3_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart3_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart3_handle.Init.Parity = UART_PARITY_NONE;
    g_uart3_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart3_handle.Init.Mode = UART_MODE_TX_RX;
        /* APB1=42MHz：16倍过采样最高约 2.625M，8倍过采样最高约 5.25M */
    if(baudrate > 2600000U)
    {
        g_uart3_handle.Init.OverSampling = UART_OVERSAMPLING_8;
    }
    else
    {
        g_uart3_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    }
    HAL_UART_Init(&g_uart3_handle);

    /* 开启空闲中断接收：一帧数据接收完成后进入 HAL_UARTEx_RxEventCallback */
    HAL_UARTEx_ReceiveToIdle_IT(&g_uart3_handle, g_uart3_rxbuffer, UART3_BUFFER_SIZE);
}

/**
 * @brief  串口3中断服务函数
 */
void USART3_UX_IRQHandler(void)
{
    HAL_UART_IRQHandler(&g_uart3_handle);
}

/**
 * @brief  串口3空闲中断接收完成回调：置接收完成标志并重新开启接收
 * @param  huart 串口句柄
 * @param  Size 本次接收到的字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3_UX)
    {
        g_uart3_rx_len = Size;
        if(Size < UART3_BUFFER_SIZE)
        {
            g_uart3_rxbuffer[Size] = '\0';
        }
        g_uart3_rx_flag = 1; //接收完成标志置1
        HAL_UARTEx_ReceiveToIdle_IT(&g_uart3_handle, g_uart3_rxbuffer, UART3_BUFFER_SIZE);
    }
}

/**
 * @brief  串口3格式化发送（类似 printf）
 * @param  fmt 格式化字符串
 */
void usart3_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    HAL_UART_Transmit(&g_uart3_handle, (uint8_t *)buf, (uint16_t)strlen(buf), HAL_MAX_DELAY);
}
