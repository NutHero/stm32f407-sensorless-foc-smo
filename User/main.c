#include "main.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/uart.h"
#include "led.h"
#include "spi.h"
#include "adc.h"
#include "time.h"
#include "myiic.h"
#include "gpio.h"
#include "sram.h"
#include "bldc_control.h"
#include "smo.h"
#include "task_process.h"
#include "decodeVofa+.h"

/* 测试用数组, 起始地址为: SRAM_BASE_ADDR */
#if (__ARMCC_VERSION >= 6010050)
uint32_t g_test_buffer[250000] __attribute__((section(".bss.ARM.__at_0x68000000")));
#else
uint32_t g_test_buffer[250000] __attribute__((at(SRAM_BASE_ADDR)));
#endif
uint32_t ts;

uint16_t temp;

int main(void)
{
    HAL_Init();                                 /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);         /* 设置时钟,168Mhz */
    delay_init(168);                            /* 延时初始化 */
    usart6_init(1000000);                        /* 串口6初始化，波特率：115200 */
    usart3_init(1000000);                        /* 串口3初始化，波特率115200 */
    led_init();                                 /* 初始化LED */
    gpio_init();                                /* 初始化GPIO */
	sram_init();								/* SRAM初始化 */
	iic_init();                                 /* IIC初始化 */
    spi3_init();								/* SPI初始化 */
    time1_init(TIM1_PSC, TIM1_Period);          /* 定时器1初始化 */
    time6_init(3, 21000-1);                     /* 定时器6初始化，周期：1ms */
    adc_injected_init();                        /* ADC初始化 */
	printf("BLDC platform test\r\n");
	usart3_printf("uart3 test\n");
    pid_init_q15(); //PID初始化
	trap_init_q15(&m1_planner, M1_Speed_LIMIT, 200); //梯形规划器1初始化
	encode_initial();	//编码器初始化
	BLDC_Smo_Init();	//滑膜观测器初始化
	
	task_scheduler_stop();	//关闭系统任务调度
	BldcInit(&motor1);	//电机1参数初始化
	
	sys_sta.current_process = PROC_GET_OFFSET; //设置第一个要执行的任务
	task_scheduler_start();	//开启系统任务调度
	while(motor1.initial_flag == 0)	//等待电机初始化完成
	{
		HAL_Delay(10);
	}
	HAL_Delay(1000);
	
	// BLDC_MODE_SPEED_CONTROL		BLDC_MODE_POS_CONTROL
	MotorSetMode(&motor1, BLDC_MODE_CLOSE_LOOP);	//设置电机模式
	MotorSetSpeed(&motor1, 1500);	//设置速度
	//MotorSetAngle(&motor1, 300);
	MotorEnable(&motor1, 1);	//打开电机
	
    while(1)
    {
//		if(g_uart3_rx_flag == 1)
//		{
//			usart3_printf("USART3 RX: %s\r\n", (char *)g_uart3_rxbuffer);
//			g_uart3_rx_flag = 0;
//		}
		temp++;
		if(temp % 500 == 0)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
		printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
			smo.we_mea, smo.we_est,
			smo.theta_mea, smo.theta_est,
			smo.h1, smo.h2,
			smo.theta_err,
			smo.sum_pll,
			smo.Ealpha_esc, smo.Ebeta_esc,
			smo.theta_mea
		);
		
//		printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
//				smo.Ialpha_mea, smo.Ialpha_esc, smo.Ialpha_err,
//				smo.Ibeta_mea, smo.Ibeta_esc, smo.Ibeta_err,
//				smo.h1, smo.h2
//			);
		
//		usart3_printf("%d,%d\n", motor1.mea_speed, motor1.mea_iq);
//		VOFA_SEND(&g_uart3_handle, 2, motor1.mea_speed, motor1.mea_iq);
//		printf("%d,%d,%d,%d,%d,%d\n", motor1.mea_speed, motor1.mea_angle, motor1.mea_ia, motor1.mea_ib, motor1.mea_ic, motor1.mea_iq);
//		HAL_Delay(1);
    }
}

