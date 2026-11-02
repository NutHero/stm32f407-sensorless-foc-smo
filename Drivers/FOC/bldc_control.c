#include "bldc_control.h"
#include "task_process.h"

static int32_t SpeedSin(int32_t speed_pp, float add_pre);
static void PosCurrentLoop(dev_motor *pmotor);
static void SpeedLoop(dev_motor *pmotor);
static void CurrentLoop(dev_motor *pmotor);
static void PosSpeedLoop(dev_motor *pmotor);
static void PosLoop(dev_motor *pmotor);
static void CloseLoopTest(dev_motor *pmotor, int32_t target_vq);
static void ControlSpeed(dev_motor *pmotor);
static void ControlPosition(dev_motor *pmotor);
static void OpenLoop(dev_motor *pmotor); //开环强拖

/****** 电机1 ******/
dev_motor motor1; //电机1结构体变量

/**
 * @brief  电机控制初始化，绑定编码器、定时器、滤波器、PID 与规划器等句柄
 * @note   需在时钟、编码器、定时器与 ADC 初始化完成之后调用
 * @param  pmotor 电机结构体句柄指针
 */
void BldcInit(dev_motor *pmotor)
{   
    pmotor->_encoder = &m1_encoder; //先将电机绑定编码器
    pmotor->_timer = &g_tim1_handle; //电机绑定定时器
    pmotor->_speed_lpf = &m1_lpf_speed; //电机绑定滤波器
    pmotor->_iq_lpf = &m1_lpf_iq; //电机绑定滤波器
    pmotor->_id_lpf = &m1_lpf_id; //电机绑定滤波器
	pmotor->_speed_pid = &m1_speed_pid;
	pmotor->_iq_pid = &m1_iq_pid;
	pmotor->_id_pid = &m1_id_pid;
	pmotor->_pos_pid = &m1_pos_pid;
	pmotor->_trap_planner = &m1_planner;
	pmotor->_speed_ramp_lpff = &m1_lpff_speed_limit;
    pmotor->target_angle = 0; 
	pmotor->target_speed = 0;
    pmotor->sta = 0;
    pmotor->compensate_ia = -0; 
    pmotor->compensate_ib = -0; 
    pmotor->compensate_ic = -0;  
    pmotor->offset_angle = -1; 
    pmotor->openLoop_fre = OPEN_LOOP_COUNT; 
	pmotor->open_vq = OPEN_LOOP_VOL;
	pmotor->speed_limit = M1_Speed_LIMIT;
	pmotor->current_limit = M1_Current_LIMIT;
    pmotor->pole = M1_POLE_PAIRS;
    pmotor->mode = M1_Control_Mode; 
}

/**
 * @brief  电机使能/失能
 * @param  pmotor 电机结构体句柄指针
 * @param  en 使能标志：1 启动，0 停止
 */
void MotorEnable(dev_motor *pmotor, uint8_t en)
{
    if(en)
    {
        HAL_TIM_PWM_Start(&g_tim1_handle, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&g_tim1_handle, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&g_tim1_handle, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&g_tim1_handle, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&g_tim1_handle, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&g_tim1_handle, TIM_CHANNEL_3);
        pmotor->sta = 1;
    }
    else
    {
        if(pmotor == &motor1)
        {
            pid_clear_q15(&m1_speed_pid);
            pid_clear_q15(&m1_iq_pid);
            pid_clear_q15(&m1_pos_pid);
            pid_clear_q15(&m1_id_pid);
        }
        Smo_Startup_Reset();   /* 停机时复位无感启动状态 */
        pmotor->out_vq = 0;
        pmotor->_timer->Instance->CCR1 = 0;
        pmotor->_timer->Instance->CCR2 = 0;
        pmotor->_timer->Instance->CCR3 = 0;
        HAL_TIM_PWM_Stop(&g_tim1_handle, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&g_tim1_handle, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&g_tim1_handle, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&g_tim1_handle, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&g_tim1_handle, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&g_tim1_handle, TIM_CHANNEL_3);
        pmotor->sta = 0;
    }
}	

/**
 * @brief  设置电机控制模式
 * @param  pmotor 电机结构体句柄指针
 * @param  mode 控制模式（BLDC_MODE_*）
 */
void MotorSetMode(dev_motor *pmotor, Motor_mode mode)
{
    pmotor->mode = mode;
}	

/**
 * @brief  设置电机目标速度
 * @param  pmotor 电机结构体句柄指针
 * @param  target_speed 目标速度，单位 RPM
 */
void MotorSetSpeed(dev_motor *pmotor, int32_t target_speed)
{
    pmotor->target_speed = target_speed;
} 

/**
 * @brief  设置电机目标角度
 * @param  pmotor 电机结构体句柄指针
 * @param  target_angle 目标角度，单位度
 */
void MotorSetAngle(dev_motor *pmotor, int32_t target_angle)
{
    pmotor->target_angle = target_angle;
} 

/**
 * @brief  产生正弦速度给定信号，用于开环速度爬升测试
 * @param  speed_pp 速度峰值
 * @param  add_pre 正弦相位增量
 * @return 当前正弦速度给定
 */
static int32_t SpeedSin(int32_t speed_pp, float add_pre)
{
    static float theta = 0.0;
    int32_t out;
    theta += add_pre;
    if(theta >= 6.28)
    {
        theta = 0;
    }
    out = speed_pp * sin(theta);
    return out;
}

/**
 * @brief  在调试引脚输出一个约 350ns 的脉冲，用于示波器触发
 */
void PulseTrig(void)
{
    DeBUG(1);
    //大约延时350ns
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    DeBUG(0);
}

/**
 * @brief  电机高频控制入口，在 ADC 注入转换完成中断中调用
 * @note   完成编码器/电流状态更新与偏置校准，并按当前模式执行控制环
 * @param  pmotor 电机结构体句柄指针
 */
void MotorHandler(dev_motor *pmotor)
{
	if(pmotor == &motor1)
	{
		//   PulseTrig();
		pmotor->control_pre++;
		if(pmotor->resetAngle_flag == 1)
		{
			if(pmotor->control_pre % M1_GETSTA_ANGLE == 0)
			{
				__disable_irq();
				UpdateMotorSta(pmotor, M1_GETSTA_ANGLE * 50);
				GetIqId(pmotor);
				__enable_irq();
			}
		}
		if(pmotor->offset_current_flag == 0)
		{
			if(GetOffsetCurrent(pmotor))
			{
				pmotor->offset_current_flag = 1;
			}
		}
		else if(pmotor->sta == 1 && pmotor->resetAngle_flag == 1)
		{
			if(pmotor->control_pre % M1_Control_FRE == 0)
			{
				switch(pmotor->mode)
				{                
					case BLDC_MODE_OPEN_LOOP:
					{
						pmotor->out_vq =pmotor->open_vq;
						OpenLoop(pmotor);
						break;
					}
					case BLDC_MODE_CLOSE_LOOP:
					{
						pmotor->out_vq = pmotor->open_vq;   /* 启动阶段: 固定电压强拖(切无感后也先沿用) */
						pmotor->out_vd = 0;

						/* 选择换相角: 开环强拖角 或 SMO 估计角 */
						if(Smo_Startup())                   /* 返回1: 已检测到反电动势, 切入无感 */
						{
							/* 无感: 用估计的电角度换相(Q15)。
							   注意: smo.theta_est 是电角度, 电机轴机械角 = theta_est / M1_POLE_PAIRS */
							pmotor->motor_axle_angle = Smo_ElecToQ15(smo.theta_est);
						}
						else
						{
							/* 开环强拖: 用内部推进的电角度换相 */
							pmotor->motor_axle_angle = Smo_ElecToQ15(smo.ol_theta);
						}
						Smo_Control(100);                   /* 观测器使用上面设置好的换相角 */
						SetPhaseVol(pmotor);
						break;
					}
					case BLDC_MODE_SPEED_LOOP:
					{
						Smo_Control(100);	//滑膜控制
						SpeedLoop(pmotor);
						break;
					}
					case BLDC_MODE_CURRENT_LOOP:
					{
						CurrentLoop(pmotor);
						break;
					}
					case BLDC_MODE_POS_LOOP:
					{
						PosLoop(pmotor);
						break;
					}
					case BLDC_MODE_POS_SPEED_LOOP:
					{
						PosSpeedLoop(pmotor);
						break;
					}
					case BLDC_MODE_SPEED_CONTROL:
					{
						ControlSpeed(pmotor);
						break;
					}
					case BLDC_MODE_POS_CURRENT_LOOP:
					{
						PosCurrentLoop(pmotor);
						break;
					}
					case BLDC_MODE_POS_CONTROL:
					{
						ControlPosition(pmotor);
						break;
					}
					case BLDC_MODE_ERROR:
					default: break;
				}
			}
		}	
	}
}

/**
 * @brief  位置+电流环控制
 * @param  pmotor 电机结构体句柄指针
 */
static void PosCurrentLoop(dev_motor *pmotor)
{
	if(pmotor->target_angle != pmotor->target_angle_pre)
	{
		pmotor->target_angle_pre = pmotor->target_angle;
		trap_set_target_q15(pmotor->_trap_planner, pmotor->target_angle);
	}   
	trap_update_q15(pmotor->_trap_planner, 45);                        
	pmotor->_pos_pid->SetPoint = trap_get_pos_q15(pmotor->_trap_planner);
	pmotor->_iq_pid->SetPoint = pid_cpos_q15(pmotor->_pos_pid, pmotor->mea_angle); 
	_constrain(pmotor->_iq_pid->SetPoint, -pmotor->current_limit, pmotor->current_limit);            	  	
	pmotor->out_vq = pid_cpos_q15(pmotor->_iq_pid, pmotor->mea_iq);
	SetPhaseVol(pmotor);  	
}

/**
 * @brief  速度环控制，输出 q 轴电压
 * @param  pmotor 电机结构体句柄指针
 */
static void SpeedLoop(dev_motor *pmotor)
{
	pmotor->_speed_pid->SetPoint = (int32_t)LowPass_Filter((float)pmotor->target_speed, pmotor->_speed_ramp_lpff);
	pmotor->out_vq = pid_cpos_q15(pmotor->_speed_pid, pmotor->mea_speed);
	if(pmotor->_speed_pid->SetPoint == 0)
	{
		pid_clear_q15(pmotor->_iq_pid);
		pid_clear_q15(pmotor->_id_pid);
		pid_clear_q15(pmotor->_speed_pid);
		pid_clear_q15(pmotor->_pos_pid);
	}
	SetPhaseVol(pmotor);
}

/**
 * @brief  电流环控制，输出 q 轴电压
 * @param  pmotor 电机结构体句柄指针
 */
static void CurrentLoop(dev_motor *pmotor)
{
//	pmotor->_id_pid->SetPoint = pmotor->target_id;
	pmotor->_iq_pid->SetPoint = pmotor->target_iq;
	pmotor->out_vq = pid_cpos_q15(pmotor->_iq_pid, pmotor->mea_iq);
	//pmotor->ud = pid_cpos_q15(pmotor->_id_pid, pmotor->id);
	SetPhaseVol(pmotor);
}

/**
 * @brief  位置+速度环控制
 * @param  pmotor 电机结构体句柄指针
 */
static void PosSpeedLoop(dev_motor *pmotor)
{
	if(pmotor->target_angle != pmotor->target_angle_pre)
	{
		pmotor->target_angle_pre = pmotor->target_angle;
		trap_set_target_q15(pmotor->_trap_planner, pmotor->target_angle);
	}   
	trap_update_q15(pmotor->_trap_planner, 45);                        
	pmotor->_pos_pid->SetPoint = trap_get_pos_q15(pmotor->_trap_planner);
	pmotor->_speed_pid->SetPoint = pid_cpos_q15(pmotor->_pos_pid, pmotor->mea_angle); 
	_constrain(pmotor->_speed_pid->SetPoint, -pmotor->speed_limit, pmotor->speed_limit);            	
	pmotor->out_vq = pid_cpos_q15(pmotor->_speed_pid, pmotor->mea_speed);
	SetPhaseVol(pmotor);  
}

/**
 * @brief  纯位置环控制
 * @param  pmotor 电机结构体句柄指针
 */
static void PosLoop(dev_motor *pmotor)
{
	if(pmotor->target_angle != pmotor->target_angle_pre)
	{
		pmotor->target_angle_pre = pmotor->target_angle;
		trap_set_target_q15(pmotor->_trap_planner, pmotor->target_angle);
	}   
	trap_update_q15(pmotor->_trap_planner, 45);                        
	pmotor->_pos_pid->SetPoint = trap_get_pos_q15(pmotor->_trap_planner);
	pmotor->out_vq = pid_cpos_q15(pmotor->_pos_pid, pmotor->mea_angle);          	
	SetPhaseVol(pmotor);  
}

/**
 * @brief  开环电压闭环测试，直接给定 q 轴电压
 * @param  pmotor 电机结构体句柄指针
 * @param  target_vq 给定的 q 轴电压（Q15）
 */
static void CloseLoopTest(dev_motor *pmotor, int32_t target_vq)
{
	pmotor->out_vq = target_vq;
	SetPhaseVol(pmotor);
}

/**
 * @brief  速度+电流环控制（速度模式）
 * @param  pmotor 电机结构体句柄指针
 */
static void ControlSpeed(dev_motor *pmotor)
{
	pmotor->_speed_pid->SetPoint = (int32_t)LowPass_Filter((float)pmotor->target_speed, pmotor->_speed_ramp_lpff);
	pmotor->_iq_pid->SetPoint = pid_cpos_q15(pmotor->_speed_pid, pmotor->mea_speed);
	_constrain(pmotor->_iq_pid->SetPoint, -pmotor->current_limit, pmotor->current_limit);
	pmotor->out_vq = pid_cpos_q15(pmotor->_iq_pid, pmotor->mea_iq);
	if(pmotor->_speed_pid->SetPoint == 0)
	{
		pid_clear_q15(pmotor->_iq_pid);
		pid_clear_q15(pmotor->_id_pid);
		pid_clear_q15(pmotor->_speed_pid);
		pid_clear_q15(pmotor->_pos_pid);
	}
	SetPhaseVol(pmotor);
}

/**
 * @brief  位置+速度+电流环控制（位置模式）
 * @param  pmotor 电机结构体句柄指针
 */
static void ControlPosition(dev_motor *pmotor)
{
	if(pmotor->target_angle != pmotor->target_angle_pre)
	{
		pmotor->target_angle_pre = pmotor->target_angle;
		trap_set_target_q15(pmotor->_trap_planner, pmotor->target_angle);
	}   
	trap_update_q15(pmotor->_trap_planner, 45);                        
	pmotor->_pos_pid->SetPoint = trap_get_pos_q15(pmotor->_trap_planner);
	pmotor->_speed_pid->SetPoint = pid_cpos_q15(pmotor->_pos_pid, pmotor->mea_angle); 
	_constrain(pmotor->_speed_pid->SetPoint, -pmotor->speed_limit, pmotor->speed_limit); 
	pmotor->_iq_pid->SetPoint = pid_cpos_q15(pmotor->_speed_pid, pmotor->mea_speed);
	_constrain(pmotor->_iq_pid->SetPoint, -pmotor->current_limit, pmotor->current_limit);                   
	pmotor->out_vq = pid_cpos_q15(pmotor->_iq_pid, pmotor->mea_iq);
	SetPhaseVol(pmotor);  
}

/**
 * @brief  开环强拖，按给定电压与递增电角度驱动电机
 * @note   仅用于测试，产生旋转磁场
 * @param  pmotor 电机结构体句柄指针
 */
void OpenLoop(dev_motor *pmotor)
{
    RevPark rpark;
    uint16_t duty_phase[3];
    static int32_t temp1 = 0, temp2 = 0;
	if(pmotor == &motor1)
    {
        if(pmotor->out_vq > 0)
        {
            temp1 += pmotor->openLoop_fre;
            if(temp1 > 32767)temp1 = -32768;
        }
        else
        {
            temp1 -= pmotor->openLoop_fre;
            if(temp1 < -32768)temp1 = 32767;
        } 
        #if defined (SVPWM)
            rpark = RevPark_Transform15(pmotor->out_vq, pmotor->out_vd, temp1);
            set_svpwm_q15(rpark.ualpha, rpark.ubeta, TIM1_Period, duty_phase); 
        #elif defined (SPWM)
            set_spwm_q15(pmotor->out_vq, pmotor->out_vd, temp1, TIM1_Period, duty_phase);
        #else
            #error "没有选择PWM调制模式：SVPWM/SPWM"
        #endif
    }   
    //printf("%d,%d,%d\n", duty_phase[0], duty_phase[1], duty_phase[2]);
    pmotor->_timer->Instance->CCR1 = duty_phase[0];
    pmotor->_timer->Instance->CCR2 = duty_phase[1];
    pmotor->_timer->Instance->CCR3 = duty_phase[2]; 
}

/**
 * @brief  获取电机状态信息，包括电机速度，输出轴位置，电机旋转的总角度，圈数，方向，并更新到
           输入结构体。
 * @note   同时会调用编码器更新函数，以一个固定的频率被调用，interval_us 变量用于计算电机速度
 * @param  pmotor  电机结构体句柄指针
 * @param  interval_us 调用频率(us)
 */
void UpdateMotorSta(dev_motor *pmotor, uint32_t interval_us)
{
	int16_t temp1;
    dev_encoder *pencoder_t = pmotor->_encoder;
    updat_encoder_sta(pencoder_t);
    pmotor->motor_axle_angle = GetMotorPos(pmotor); 
    pmotor->mea_angle = pencoder_t->total_pulse * ENCODER_RATIO;
	temp1 = pmotor->mea_angle % 360;
	if(temp1 < 0)temp1 = 360 + temp1; //角度为负时将其转为正数
	pmotor->physic_angle = temp1;
    pmotor->mea_speed = GetMotorSpeed(pmotor, interval_us);
    pmotor->cir = GetMotorCir(pmotor);
}

/**
 * @brief  获取电机旋转圈数
 * @param  pmotor 电机结构体句柄指针
 * @return 电机旋转圈数
 */
int32_t GetMotorCir(dev_motor *pmotor)
{
    pmotor->cir = pmotor->_encoder->over_cnt;
    return pmotor->cir;
}

#define FACTOR3 60000000.0f //用于计算速度：= 60 * 1000000
const uint32_t factor2 = ENCODER_FACTOR; //输出轴转一圈编码器脉冲数
const float factor4 = FACTOR3 / factor2;

/**
 * @brief  计算电机转速（RPM）
 * @note   以固定频率调用，interval_us 用于根据编码器增量计算速度
 * @param  pmotor 电机结构体句柄指针
 * @param  interval_us 调用频率(us)
 * @return 电机转速，单位 RPM
 */
int32_t GetMotorSpeed(dev_motor *pmotor, uint32_t interval_us)
{
    if(interval_us == 0) interval_us = 1;
    pmotor->mea_speed = (float)pmotor->_encoder->pulse_diff / interval_us * factor4; // 计算速度
    pmotor->mea_speed = LowPass_Filter_Q15(pmotor->mea_speed, pmotor->_speed_lpf); //滤波

    /* 当速度异常时，过滤掉这个数据 */
    if(pmotor->mea_speed > (SPEED_MAX * 10) || pmotor->mea_speed < -(SPEED_MAX * 10))
    {
        pmotor->mea_speed = pmotor->_speed_lpf->Yn;
    }
    
    /* 编码器超过一定时间没有脉冲，则认为电机静止，将电机速度置0 */
    if(pmotor->_encoder->change_flag == 0)
    {
        pmotor->static_time++; //+100us
        if(pmotor->static_time > 10 * 500) //500ms
        {
            pmotor->mea_speed = 0;
        }
    }
    else
    {
        pmotor->static_time = 0;
    }
    return pmotor->mea_speed;
}

const uint32_t factor5 = ENCODER_FACTOR;

/**
 * @brief  将编码器机械角映射到 Q15 电角度
 * @note   输入 0~ENCODER_FACTOR，输出 -32768~32767
 * @param  a 输入机械角
 * @return 映射后的 Q15 值
 */
static int32_t AToB(int32_t a)
{
    if (a < 0) a = 0;
    else if (a > factor5) a = factor5;    
    return (a <= factor5 / 2) ? (a << 2) : ((a << 2) - 65536);
}

/**
 * @brief  计算电机电角度位置
 * @param  pmotor 电机结构体句柄指针
 * @return 电角度位置（Q15：-32768~32767）
 */
int32_t GetMotorPos(dev_motor *pmotor)
{
    int32_t ele_angle,angle;
    ele_angle = GetElectricity(pmotor->_encoder->value, pmotor->pole);
	pmotor->ele_angle = ele_angle;
    angle = NormalizeAngle(ele_angle);
    return AToB(angle);
}

/**
 * @brief  角度归一化到 0~ENCODER_FACTOR
 * @param  angle 输入角度
 * @return 归一化后的角度
 */
int32_t NormalizeAngle(int32_t angle)
{
    angle = angle % factor5;
    return angle < 0 ? angle + factor5 : angle;
}

/**
 * @brief  计算机械角对应的电角度
 * @note   电角度 = 机械角 × 极对数
 * @param  angle 机械角
 * @param  pole 极对数
 * @return 电角度
 */
int32_t GetElectricity(int32_t angle, uint8_t pole)
{
    return angle * pole;
}

/**
 * @brief  获取电机当前转动方向
 * @param  pmotor 电机结构体句柄指针
 * @return 方向：1 正转，-1 反转，0 静止
 */
int8_t GetMotorDir(dev_motor *pmotor)
{
    return pmotor->_encoder->dir;
}

/**
 * @brief  电角度零位校准
 * @note   以递增的 d 轴电压将转子拉到已知位置，记录编码器偏置
 * @param  pmotor 电机结构体句柄指针
 * @param  interval_ms 调用间隔(ms)
 * @return 校准完成返回 1，未完成返回 0
 */
int8_t ResetAngle(dev_motor *pmotor, uint32_t interval_ms)
{
    static uint16_t i[1] = {0};
    uint16_t ud_max = GPS_MAX_VOL; 
    const uint8_t voltage_ramp = GPS_VOL_RAMP; 
    if(pmotor == &motor1)
    {
        if(i[0]++ < (GPS_TIME / interval_ms))   //等待电机转到A相
        {
            
            if(pmotor->out_vd < ud_max)
            {
                pmotor->out_vd += (voltage_ramp * interval_ms); //逐渐增加d轴电压
            }
            else
            {
                pmotor->out_vd = ud_max;
            }
            pmotor->motor_axle_angle = 0;
            pmotor->out_vq = 0;
            SetPhaseVol(pmotor);  //设置为0角度
            return 0;
        }
        else //校准完成
        {
            pmotor->_encoder->offset_value = MT6701_GetRawData();
            pmotor->out_vq = 0;
            pmotor->out_vd = 0;
            SetPhaseVol(pmotor);
            i[0] = 0;
            return 1;
        }
 
    }
}

/**
 * @brief  堵转检测
 * @note   速度低于阈值且电流高于阈值并持续一定时间判定为堵转
 * @param  pmotor 电机结构体句柄指针
 * @return 堵转返回 1，未堵转返回 0
 */
int8_t StallCheck(dev_motor *pmotor)
{
    if(abs(pmotor->mea_speed) < MOTOR_STALL_SPEED_THRESHOLD_RPM &&  //速度小于阈值
            abs(pmotor->mea_iq) > MOTOR_STALL_CURRENT_THRESHOLD_MA //且电流过大
        )
    {
        pmotor->stall_time++; //开始堵转计时
        if(pmotor->stall_time > MOTOR_STALL_TIME_THRESHOLD_MS) //超时则认为电机堵转
        {
            return 1;
        }
    }
    else
    {
        pmotor->stall_time = 0;
    }
    return 0;
}


/**
 * @brief  根据 q/d 轴电压与电角度计算并输出三相 PWM
 * @param  pmotor 电机结构体句柄指针
 */
void SetPhaseVol(dev_motor *pmotor)
{      
    RevPark rpark;
    uint16_t duty_phase[3];
    
    #if defined( SVPWM )
        rpark = RevPark_Transform15(pmotor->out_vq, pmotor->out_vd, pmotor->motor_axle_angle);
        set_svpwm_q15(rpark.ualpha, rpark.ubeta, TIM1_Period, duty_phase); 
    #elif defined (SPWM)
        set_spwm_q15(pmotor->out_vq, pmotor->out_vd, pmotor->motor_axle_angle, TIM1_Period, duty_phase);
    #else
        #error "没有选择PWM调制模式：SVPWM/SPWM"
    #endif
    pmotor->_timer->Instance->CCR1 = duty_phase[0];
    pmotor->_timer->Instance->CCR2 = duty_phase[1];
    pmotor->_timer->Instance->CCR3 = duty_phase[2];  
    //printf("%d,%d,%d\n", duty_phase[0], duty_phase[1], duty_phase[2]);
}

/**
 * @brief  清零编码器累计值、圈数、角度与电角度
 * @param  pmotor 电机结构体句柄指针
 */
void ClearEncoder(dev_motor *pmotor)
{
    pmotor->_encoder->over_cnt = 0;
    pmotor->_encoder->total_pulse = 0;
    pmotor->_encoder->over_pre_cnt = 0;
    pmotor->_encoder->pre_value = 0;
    pmotor->_encoder->value = 0;
    pmotor->mea_angle = 0;
    pmotor->cir = 0;
    pmotor->motor_axle_angle = 0;
}

/**
 * @brief  静态电流零偏校准
 * @note   电机静止时对三相电流 ADC 原始值取平均作为零偏
 * @param  pmotor 电机结构体句柄指针
 * @return 校准完成返回 1，未完成返回 0
 */
int8_t GetOffsetCurrent(dev_motor *pmotor)
{
    static uint16_t i[1] = {0};
    static uint32_t sum[3] = {0};
    if(pmotor == &motor1)
    {
        sum[0] += adc_value[0];
        sum[1] += adc_value[1];
        sum[2] += adc_value[2];
        i[0]++;
        if(i[0] >= 1024)
        {
            pmotor->offset_ia = sum[0] >> 10;
            pmotor->offset_ib = sum[1] >> 10;
            pmotor->offset_ic = sum[2] >> 10;
            sum[0] = 0;
            sum[1] = 0;
            sum[2] = 0;
            i[0] = 0;
            return 1;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

const float factor = (ADC_RATIO / (VOL_GAIN * SAMP_R)) * 1000; //AD原始值转电流系数，x1000将电流单位转为mA

/**
 * @brief  计算三相电流，单位为 mA
 * @param  pmotor 电机结构体句柄指针
 */
void GetPhaseCurrent(dev_motor *pmotor)
{
    if(pmotor == &motor1)
    {
        pmotor->mea_ia = ((int32_t)adc_value[0] - pmotor->offset_ia) * factor + pmotor->compensate_ia;
        pmotor->mea_ib = ((int32_t)adc_value[1] - pmotor->offset_ib) * factor + pmotor->compensate_ib;
        pmotor->mea_ic = ((int32_t)adc_value[2] - pmotor->offset_ic) * factor + pmotor->compensate_ic;
//		pmotor->ic = -(pmotor->ia + pmotor->ib);
        pmotor->mea_ia = -pmotor->mea_ia;
        pmotor->mea_ib = -pmotor->mea_ib;
        pmotor->mea_ic = -pmotor->mea_ic;
    }
}

/**
 * @brief  通过 Clarke/Park 变换计算 q 轴与 d 轴电流并滤波
 * @param  pmotor 电机结构体句柄指针
 */
void GetIqId(dev_motor *pmotor)
{
    Clarke clark;
    Park park;
    GetPhaseCurrent(pmotor);
    clark = Clarke_Transform15(pmotor->mea_ia, pmotor->mea_ib); 
    park = Park_Transform15(clark.ialpha, clark.ibeta, pmotor->motor_axle_angle); 
    pmotor->mea_iq = park.iq; 
    pmotor->mea_id = park.id;
    
    /* 电流滤波 */
    pmotor->mea_iq = LowPass_Filter_Q15(pmotor->mea_iq, pmotor->_iq_lpf); 
    pmotor->mea_id = LowPass_Filter_Q15(pmotor->mea_id, pmotor->_id_lpf); 
}
