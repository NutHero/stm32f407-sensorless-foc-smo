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

/* 测试用给定转速(RPM)：
   300 —— 验证"能不能切进无感闭环"。相电阻修正后切换门槛降到约 116rpm，
            300rpm 一定能在 0.25s 左右切进去(启动斜坡 900rad/s^2)。
   50  —— 验证低速 I/f 段(设定 50rpm 就只转 50rpm，电流 2A，不再"转一下停一下")。
            这个转速下第 13 列恒为 1(I/f 段)，是设计如此，不是"切不进闭环"。
   注意: 纯反电动势观测器的真实物理下限在 80rpm 附近(反电动势 0.12V，与死区压降/
   采样零漂同量级)。再往下的闭环位置必须靠高频注入(见对话里的方案)，I/f 段则永远可用。 */
#define TEST_SPEED_RPM      1000

/* 自动换向测试(ms): 0 = 关闭。设成 2000 就每 2 秒正反转切换一次，专门验证换向。
   看第 5 列(I/f 磁场频率, 带符号)必须是从 + 连续减到 0 再变 -，中间不能出现
   "+w 直接跳到 -w" 的台阶；出现台阶就是又变回反接制动(抖动/概率换向)。 */
#define TEST_REVERSE_MS     0

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
    adc_regular_init();                         /* 普通组ADC: 温度/母线电流/母线电压 */
	printf("BLDC platform test\r\n");
	usart3_printf("uart3 test\n");
    pid_init_q15(); //PID初始化
	trap_init_q15(&m1_planner, M1_Speed_LIMIT, 200); //梯形规划器1初始化
	encode_initial();	//编码器初始化
	BLDC_Smo_Init();	//滑膜观测器初始化

	/* 电流环中断必须是最高优先级。
	   time.c / adc.c 里 TIM6(1ms) 是 1、ADC 是 2，也就是 1ms 的进程状态机
	   (命令解析、堵转检测、printf)会把 100us 的电流环顶歪几十微秒。
	   高速时这几十微秒就是十几度的换相角抖动，表现为电流变大、噪声变大。 */
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);

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
	MotorSetMode(&motor1, BLDC_MODE_SPEED_LOOP);	//设置电机模式
	MotorSetSpeed(&motor1, TEST_SPEED_RPM);	//设置速度(改 TEST_SPEED_RPM 即可)
	//MotorSetAngle(&motor1, 300);
	MotorEnable(&motor1, 1);	//打开电机

    while(1)
    {
		temp++;
		if(temp % 500 == 0)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
#if (TEST_REVERSE_MS > 0)
		/* 自动换向测试: 每 TEST_REVERSE_MS 毫秒把给定取反 */
		{
			static uint32_t t_rev = 0;
			static int8_t   rev_sgn = 1;

			if((HAL_GetTick() - t_rev) >= TEST_REVERSE_MS)
			{
				t_rev = HAL_GetTick();
				rev_sgn = (int8_t)(-rev_sgn);
				MotorSetSpeed(&motor1, (int32_t)rev_sgn * TEST_SPEED_RPM);
			}
		}
#endif
		/* 无感/I/f 调试通道(串口6, 1Mbps, 逗号分隔)：
		   1 编码器转速(未接编码器时恒0)   2 估计转速(RPM, I/f 段=磁场频率)
		   3 速度给定(斜坡后)             4 q轴电压(Q15)          5 I/f磁场频率(x10 rad/s, 带符号)
		   6 切入标志(1=已切无感, 0=I/f)   7 估计角-强拖角(x10度)  8 反电动势幅值(x100 V)
		   9 I/f电流环积分项(mV)          10 q轴电流(mA)          11 估计电角速度(rad/s)
		   12 d轴电流(mA)                 13 I/f段标志(1=I/f)     14 I/f转矩电流给定(mA)
		   15 相电阻辨识值(m欧)
		   看这次新增的一列:
		     第 15 列 = 定角阶段辨识出来的相电阻(定角时 E=0, R=V/I)。正常应在 200~260 m欧。
		              它直接决定观测器模型准不准: 以前写死 400 m欧, 2A 就多算 0.4V 假反电动势,
		              低速根本锁不住。若这一列长期停在 400 或 0, 说明辨识没跑起来
		              (定角被跳过 / 电流太小)。
		   第 8 列 Eamp 现在可以当真值看了: 50rpm 应≈0.08V, 100rpm≈0.15V, 300rpm≈0.46V。
		   第 10 列 iq 应贴住第 14 列(误差 <200mA), 否则电流环还有前馈残差。
		   换向时盯第 5 列(带符号频率)和第 2 列(带符号转速): 必须一起连续过零, 不能跳变。 */
		printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			(int)motor1.mea_speed, 
			(int)motor1.est_speed,
			(int)motor1._speed_pid->SetPoint,
			(int)motor1.out_vq,
			(int)(smo.ol_we * 10.0f),
			(int)motor1.sensorless_flag,
			(int)(smo.dth_deg * 10.0f),
			(int)(smo.Eamp * 100.0f),
			(int)(smo.if_i * 1000.0f),
			(int)motor1.mea_iq,
			(int)smo.we_est,
			(int)motor1.mea_id,
			(int)motor1.if_flag,
			(int)(smo.if_iref * 1000.0f),
			(int)(smo.R_est * 1000.0f),
			(int)(smo.theta_est * 1000.0f), 
			(int)(smo.theta_mea * 1000.0f),
			(int)(power_board.bus_current), 
			(int)(power_board.bus_voltage * 10.0f), 
			(int)(power_board.temperature * 10.0f)
		);

//		/* 另一组：αβ 电流误差(查观测器模型是否准) */
//		printf("%f,%f,%f,%f,%f,%f\n",
//				smo.Ialpha_mea, smo.Ialpha_esc, smo.Ialpha_err,
//				smo.Ibeta_mea, smo.Ibeta_esc, smo.Ibeta_err);

//		usart3_printf("%d,%d\n", motor1.mea_speed, motor1.mea_iq);
//		VOFA_SEND(&g_uart3_handle, 2, motor1.mea_speed, motor1.mea_iq);
    }
}
