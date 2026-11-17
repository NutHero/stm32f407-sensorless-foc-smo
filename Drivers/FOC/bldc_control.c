#include "bldc_control.h"
#include "task_process.h"

static int32_t SpeedSin(int32_t speed_pp, float add_pre);
static void PosCurrentLoop(dev_motor *pmotor);
static void SpeedLoop(dev_motor *pmotor);
static void SensorlessSpeedLoop(dev_motor *pmotor, int32_t target);
static void SensorlessSpeedLoop_Reset(void);
static void SensorlessSpeedLoop_Engage(dev_motor *pmotor, int32_t vq_hold);
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
	pmotor->est_speed = 0;
	pmotor->sensorless_flag = 0;
	pmotor->if_flag = 0;
	pmotor->relock_time = 0;
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
        SensorlessSpeedLoop_Reset();     /* 清无感速度环积分/反馈状态 */
        pmotor->sensorless_flag = 0;
        pmotor->if_flag = 0;
        pmotor->relock_time = 0;
        pmotor->est_speed = 0;
        pmotor->_speed_ramp_lpff->Yn = 0;   /* 清速度给定爬升初值，下次启动从0开始爬升 */
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
/* 有效转速绝对值: 编码器没接/没脉冲时 mea_speed 恒为 0, 不能拿它做停机、换向、
   失锁的判据(否则高速给定 0 会直接撤压, 换向也会在满速时硬反拖)。*/
static int32_t MotorSpeedAbs(dev_motor *pmotor)
{
    int32_t a = pmotor->mea_speed;
    int32_t b = pmotor->est_speed;
    if(a < 0) a = -a;
    if(b < 0) b = -b;
    return (a > b) ? a : b;
}

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

						/* 测试模式: 频率固定按 400rpm 爬升, 等价于原来"强拖到 300rad/s"的行为 */
						Smo_Startup_SetRef(400);
						/* 选择换相角: 开环强拖角 或 SMO 估计角 */
						if(Smo_Startup())                   /* 返回1: 已检测到反电动势, 切入无感 */
						{
							/* 无感: 用估计的电角度换相(Q15)。
							   注意: Smo_CommTheta() 是电角度, 电机轴机械角 = 该角 / M1_POLE_PAIRS */
							pmotor->motor_axle_angle = Smo_ElecToQ15(Smo_CommTheta());
						}
						else
						{
							/* 开环强拖: 用内部推进的电角度换相(已回退 90 度, 见 Smo_IfTheta) */
							pmotor->motor_axle_angle = Smo_ElecToQ15(Smo_IfTheta());
						}
						Smo_Control(100);                   /* 观测器使用上面设置好的换相角 */
						SetPhaseVol(pmotor);
						break;
					}
					case BLDC_MODE_SPEED_LOOP:
					{
						uint8_t ready;
						int8_t  dir_req;        /* 目标转向: 1 正转, -1 反转 */
						int32_t target_cmd;     /* 本周期速度环给定(换向制动时为 0) */
						int32_t spd_abs;        /* 给定幅值 [RPM] */
						int32_t vq_hold;        /* 切入无感瞬间的电压, 用于无扰接管 */
						
						/* 1) 目标转向: 由速度给定符号决定; 给定为 0 时保持当前锁定方向 */
						if(pmotor->target_speed > 0)      dir_req = 1;
						else if(pmotor->target_speed < 0) dir_req = -1;
						else                              dir_req = smo.dir;

						target_cmd = pmotor->target_speed;
						spd_abs = (target_cmd >= 0) ? target_cmd : -target_cmd;
						
						/* 1b) 给定为 0: 只做停转, 不启动。
						     停转策略(三段, 都不再有电压阶跃):
						       - 无感段还带着转速: 交给无感速度环(target=0)制动, 降到
						         SPEED_STOP_RPM 以下再按固定斜率把 Vq 撤到 0;
						       - 低速 I/f 段: 频率按斜坡降到 0, 磁场停住 -> 转子跟着停住,
						         停稳后撤压(不再靠 PI 把转速"伺服"到 0, 那样只会抖);
						       - 已经停稳: 直接撤压复位。 */
						if(target_cmd == 0)
						{
							uint8_t cut_now = 0;

							if(pmotor->sensorless_flag == 1)
							{
								if(MotorSpeedAbs(pmotor) < SPEED_STOP_RPM)
								{
									if(pmotor->out_vq != 0)
									{
										/* 柔性撤压: 仍用观测器换相, Vq 按斜率降到 0,
										   避免停机瞬间绕组上出现电流阶跃 */
										int32_t v = pmotor->out_vq;
										if(v >  SPEED_STOP_VQ_STEP)      v -= SPEED_STOP_VQ_STEP;
										else if(v < -SPEED_STOP_VQ_STEP) v += SPEED_STOP_VQ_STEP;
										else                             v  = 0;
										pmotor->out_vq = (int16_t)v;
										pmotor->out_vd = 0;
										pmotor->motor_axle_angle = Smo_ElecToQ15(Smo_CommTheta());
										Smo_Control(100);
										SetPhaseVol(pmotor);
										break;
									}
									cut_now = 1;
								}
							}
							else
							{
								/* I/f 段: 频率降到 0 且转子停稳才撤压 */
								if(fabsf(smo.ol_we) <= SMO_IF_WE_STOP &&
								   MotorSpeedAbs(pmotor) <= SMO_IF_STOP_RPM)
								{
									cut_now = 1;
								}
							}

							if(cut_now)
							{
								pmotor->out_vq = 0;
								pmotor->out_vd = 0;
								pmotor->est_speed = 0;
								pmotor->sensorless_flag = 0;
								pmotor->if_flag = 0;
								pmotor->relock_time = 0;
								pmotor->_speed_pid->SetPoint = 0;
								pmotor->_speed_ramp_lpff->Yn = 0.0f;
								SensorlessSpeedLoop_Reset();        /* 清无感速度环积分/反馈 */
								Smo_Startup_Reset();                /* 复位启动状态, 下次给速度重新起拖 */
								SetPhaseVol(pmotor);                /* Vq=Vd=0: 三相占空比50%, 绕组无电流 */
								break;
							}
						}
						
						/* 2) 转向与锁定方向不一致: 需要换向。
						     无感段还带着转速时: 先把给定压到 0 让速度环制动, 降到
						     SPEED_REVERSE_RPM 以下再交回 I/f 段。
						     交回之后(以及本来就在 I/f 段): 这里【绝不直接翻 smo.dir】。
						     翻 dir 等于把磁场角速度从 +w 阶跃成 -w, 转差 2w, 那是异步机的
						     反接制动: 转矩方向随转子位置乱跳、电流环饱和, 表现就是"抖一会,
						     有时才换得过去"。正确做法是把【带符号】的目标转速交给 I/f 频率
						     斜坡(见本函数末尾的 Smo_Startup_SetRef(target_speed)), 让它把
						     频率先减到 0、过零之后再反向加速; 转向由 Smo_Startup 在过零点翻。 */
						if(dir_req != smo.dir)
						{
							if(pmotor->sensorless_flag == 1 &&
							   MotorSpeedAbs(pmotor) > SPEED_REVERSE_RPM)
							{
								target_cmd = 0;                      /* 制动: 速度环把电机刹到低速 */
							}
							else
							{
								if(pmotor->sensorless_flag == 1)     /* 只在交接的那一拍做一次 */
								{
									pmotor->sensorless_flag = 0;
									pmotor->if_flag = 1;
									pmotor->relock_time = 0;
									pmotor->_speed_ramp_lpff->Yn = 0.0f;
									pmotor->_speed_pid->SetPoint = 0;
									SensorlessSpeedLoop_Reset();
									Smo_Startup_SetRef(pmotor->target_speed);
									Smo_IfEnter();
								}
								/* 换向期间速度环不出力: 减速->过零->反向全部由 I/f 频率斜坡
								   完成。等 smo.dir 翻过来, dir_req == smo.dir, 这里自然不再进 */
								target_cmd = 0;
							}
						}

						/* 3) 无感段 -> 低速 I/f 段的交接(滞环下沿):
						      给定降到 SMO_IF_RPM_MAX 以下就交回 I/f。不重启、不定角,
						      转速严格按给定走, 观测器锁不住也不影响运行 —— 这一条
						      直接消灭了低速"切进来 -> 失锁 -> 重拖 -> 再切"的往复。
						      交接前必须等实际转速也降到 I/f 段附近: 转子还在高速转的时候
						      把磁场切成低速旋转, 等于瞬间失步反拖。 */
						if(pmotor->sensorless_flag == 1 && target_cmd != 0 &&
						   spd_abs <= (int32_t)SMO_IF_RPM_MAX &&
						   MotorSpeedAbs(pmotor) <= (int32_t)(SMO_IF_RPM_MAX * 1.5f))
						{
							pmotor->sensorless_flag = 0;
							pmotor->if_flag = 1;
							pmotor->relock_time = 0;
							SensorlessSpeedLoop_Reset();
							Smo_Startup_SetRef(target_cmd);
							Smo_IfEnter();
						}

						/* 4) 无感段失锁判定: 估计转速掉到 0 附近 或 转向与目标相反
						      (观测器没锁住, 或锁到了 180 度反相点) -> 交回 I/f 段。
						      换向制动期间不做失锁判定, 否则会被拉回原方向;
						      另外要求实际转速也已经低下来: 转子还在高速转的时候交回 I/f,
						      只会被拖成"反拖刹车"。 */
						if(pmotor->sensorless_flag == 1 && pmotor->target_speed != 0 &&
						   dir_req == smo.dir &&
						   MotorSpeedAbs(pmotor) < SPEED_REVERSE_RPM &&
						   (abs(pmotor->est_speed) < SPEED_RELOCK_RPM ||
						    (pmotor->est_speed > 0) != (pmotor->target_speed > 0)))
						{
							if(++pmotor->relock_time > SPEED_RELOCK_TIME)
							{
								pmotor->relock_time = 0;
								pmotor->sensorless_flag = 0;
								pmotor->if_flag = 1;
								SensorlessSpeedLoop_Reset();
								Smo_Startup_SetRef(target_cmd);
								Smo_IfEnter();                      /* 接续, 不再从头重拖 */
							}
						}
						else
						{
							pmotor->relock_time = 0;
						}
						/* 5) 无感段 / 低速 I/f 段 */
						if(pmotor->sensorless_flag == 1)
						{
							/* 无感: 用估计的电角度换相(Q15)。
							   Smo_CommTheta() 补偿了观测器滞后与换相延时, 高速段转矩角
							   不会跑到接近 90 度, 切大速度时不抖 */
							pmotor->if_flag = 0;
							pmotor->motor_axle_angle = Smo_ElecToQ15(Smo_CommTheta());
							Smo_Control(100);                   /* 观测器使用上面设置好的换相角 */
#if SPEED_LOOP_USE_SENSORLESS
							SensorlessSpeedLoop(pmotor, target_cmd);  /* 无感速度环生成Vq并输出PWM */
#else
							SpeedLoop(pmotor);                  /* 原速度环(编码器反馈) */
#endif
						}
						else
						{
							/* 低速 I/f 段(启动 / 低速 / 失锁恢复):
							   定角阶段用固定小电压, 斜坡阶段用 iq 电流闭环;
							   换相角 = 强拖角回退 90 度(见 Smo_IfTheta), 与无感段同一定义。 */
							pmotor->if_flag = 1;
							/* 频率给定必须是【带符号的目标转速】, 不能用 target_cmd:
							   换向期间 target_cmd 被压成 0(不给速度环出力), 若拿它当频率给定,
							   斜坡就停在 0 上永远换不过去。目标符号交给斜坡, 由它连续过零。 */
							Smo_Startup_SetRef(pmotor->target_speed);
							ready = Smo_Startup();              /* 推进强拖角, 并做切换判据 */

							/* 反向锁定保护: 估计转向与目标【明显】相反时不切入(否则换相角
							   相差 180 度, 一接管就反转); 估计转速过零或小幅抖动不算 */
							if(ready &&
							   ((pmotor->target_speed > 0 && Smo_GetSpeedRpm() < -SPEED_RELOCK_RPM) ||
							    (pmotor->target_speed < 0 && Smo_GetSpeedRpm() >  SPEED_RELOCK_RPM)))
							{
								ready = 0;
							}

							if(ready)
							{
								/* 观测器已锁定(转速 > 164rpm 且相位对齐): 无扰接管。
								   给定和反馈都从当前转速接续; 电压直接沿用 I/f 段最后那一拍真正
								   加过的 Vq —— 两个段的换相角定义现在是一致的(都是"磁场角"),
								   I/f 段的电流也在 q 轴(转矩轴)上, 所以电压、电流、转矩全部连续,
								   切换时不会"冲一下"也不会掉速。
								   只有 I/f 段电压异常偏小(0 或明显低于反电动势)时才用
								   Smo_SwitchVqQ15() 兜底(反电动势 + 0.6A), 保证切过去之后还有转矩。 */
								pmotor->sensorless_flag = 1;
								pmotor->if_flag = 0;
								pmotor->relock_time = 0;
								pmotor->_speed_ramp_lpff->Yn = (float)Smo_GetSpeedRpm();
								pmotor->_speed_pid->SetPoint = (int32_t)pmotor->_speed_ramp_lpff->Yn;
								vq_hold = pmotor->out_vq;
								{
									/* 兜底电压按【幅值】比较, 不能直接比大小: 反转时两个量都是
									   负的, "负得更多"的才是幅值更大的那个。直接写
									   if(vq_hold < vq_min) 会把 -1700 换成 -780, 切换瞬间
									   把电压砍掉一半 -> 转矩掉一半 -> 掉速 -> 判失锁 ->
									   退回 I/f, 反转永远进不了闭环。 */
									int32_t vq_min = Smo_SwitchVqQ15(pmotor->_speed_pid->SetPoint);
									if(smo.dir >= 0) { if(vq_hold < vq_min) vq_hold = vq_min; }
									else             { if(vq_hold > vq_min) vq_hold = vq_min; }
								}
								SensorlessSpeedLoop_Engage(pmotor, vq_hold);
								pmotor->motor_axle_angle = Smo_ElecToQ15(Smo_CommTheta());
								Smo_Control(100);
								SensorlessSpeedLoop(pmotor, target_cmd);
							}
							else if(smo.align_cnt > 0)
							{
								/* 定角: 磁场钉住 + 固定小电压, 只做位置对准, 不产生转矩 */
								int32_t v = Smo_StartupVqQ15();
								pmotor->out_vq = (int16_t)((smo.dir >= 0) ? v : -v);
								pmotor->out_vd = 0;
								pmotor->est_speed = 0;
								pmotor->motor_axle_angle = Smo_ElecToQ15(Smo_IfTheta());
								Smo_Control(100);
								SetPhaseVol(pmotor);
							}
							else
							{
								/* I/f 运行: iq 电流闭环给电压, 频率按斜坡跟随给定 */
								pmotor->out_vq = (int16_t)Smo_IfRun(100, pmotor->mea_iq_raw);
								pmotor->out_vd = 0;
								/* 同步拖动: 转子转速就等于给定频率, 所以低速段拿这个值当
								   "实际转速"用(编码器和观测器在这个区间都不可靠) */
								pmotor->est_speed = Smo_IfSpeedRpm();
								pmotor->motor_axle_angle = Smo_ElecToQ15(Smo_IfTheta());
								Smo_Control(100);
								SetPhaseVol(pmotor);
							}
						}
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

/* ===== 无感速度环(电压型PI) =====
 * 独立于上面的 SpeedLoop，只在 BLDC_MODE_SPEED_LOOP 切入无感后调用。
 * 本模式没有电流内环，PI 直接输出 Q15 电压，所以增益单位必须是 "Q15电压/RPM"，
 * 与 BLDC_MODE_SPEED_CONTROL 里按 "输出=电流给定(mA)" 整定的 M1_KP_SPEED/M1_KI_SPEED
 * 不通用。若直接套用后者(0.1/0.1)，折算到电压上前馈之外只剩 ±3264(积分器输出上限)，
 * 而比例项小到几乎为零，积分会在正负限幅之间来回撞，表现为电机乱转、切过去后卡住高频抖动。
 *
 * 输出电压 = 反电动势前馈(按给定转速) + 电压型PI(补电阻压降/负载/前馈误差)
 *            + dq解耦(Vd = -we*L*Iq)
 * 高速段能否跑起来完全取决于"前馈 + 调节量"够不够反电动势和 R*Iq，
 * 所以调节量限幅不能太小（见 bldc_control.h 里 SPEED_PI_LIMIT 的说明）。
 */
static float spd_i  = 0.0f;     /* 速度环积分项, 单位: Q15电压 */
static float spd_fb = 0.0f;     /* 估计转速反馈(滤波后), 单位: RPM */

/**
 * @brief  清无感速度环内部状态(停机时调用)
 */
static void SensorlessSpeedLoop_Reset(void)
{
	spd_i  = 0.0f;
	spd_fb = 0.0f;
}

/**
 * @brief  切入无感瞬间做无扰接管
 * @note   把积分项预置成 "当前强拖电压 - 反电动势前馈 - 比例项"，使切换瞬间
 *         Vq 严格等于原强拖电压 vq_hold，电压与转矩都不跳变。
 *         调用前必须先把 SetPoint 和速度斜坡初值设成当前估计转速。
 * @param  pmotor  电机结构体句柄指针
 * @param  vq_hold 切换前的强拖电压(Q15)
 */
static void SensorlessSpeedLoop_Engage(dev_motor *pmotor, int32_t vq_hold)
{
	float err;

	spd_fb = (float)Smo_GetSpeedRpm();
	err = (float)pmotor->_speed_pid->SetPoint - spd_fb;
	spd_i = (float)vq_hold
	      - (float)Smo_BemfFfQ15(pmotor->_speed_pid->SetPoint)
	      - SPEED_KP * err;
	_constrain(spd_i, -(float)SPEED_PI_LIMIT, (float)SPEED_PI_LIMIT);
}

/**
 * @brief  无感速度环控制，输出 q 轴电压(电压型PI)
 * @note   换相角(电角度)由调用者用 SMO 估计角给定，本函数只负责生成 Vq/Vd:
 *         Vq = 反电动势前馈(按给定转速) + 电压型PI
 *         比例项提供阻尼，积分项消稳态误差；两者合成后的调节量限制在 SPEED_PI_LIMIT
 *         以内，避免积分无限积累(过流另有转矩电流软限幅)。
 * @param  pmotor 电机结构体句柄指针
 * @param  target 本拍速度给定(RPM)，换向制动时调用者传 0
 */
static void SensorlessSpeedLoop(dev_motor *pmotor, int32_t target)
{
	float ref, err, pi_out, lim;
	int32_t ff, vq;

	/* 1) 反馈: 观测器估计转速, 一阶滤波抑制 PLL 在低速时的抖动 */
	pmotor->est_speed = Smo_GetSpeedRpm();
	spd_fb += SPEED_FB_LPF_A * ((float)pmotor->est_speed - spd_fb);

	/* 2) 给定: 按斜率爬升(时间常数约 0.5s), 防止给定突变导致电流冲击。
	   target 由调用者传入: 正常时是 target_speed, 换向制动时是 0。 */
	pmotor->_speed_pid->SetPoint = (int32_t)LowPass_Filter((float)target, pmotor->_speed_ramp_lpff);
	ref = (float)pmotor->_speed_pid->SetPoint;

	/* 3) 反电动势前馈: 按给定转速算, 给定为0时前馈归零, 停机才能把电压降下来 */
	ff = Smo_BemfFfQ15(pmotor->_speed_pid->SetPoint);

	/* 4) 调节量限幅: 随给定转速成比例收缩(见 bldc_control.h 中 SPEED_PI_FULL_RPM 的说明)。
	   给定满速时就是 SPEED_PI_LIMIT, 给定降到 0 时限幅也收缩, 电压随给定一起
	   平滑退掉, 不会出现"给定0但PI反向出力"的反冲。
	   但收缩不能没有下限: 给定 50rpm 时正比收缩只剩 562(0.24V), 顶不住相电阻压降
	   和齿槽转矩, 转速一掉就判失锁 -> 退回强拖 -> 再失锁。所以给 SPEED_PI_LIMIT_MIN 兜底。 */
	lim = (float)SPEED_PI_LIMIT;
	{
		float sc = fabsf(ref) / SPEED_PI_FULL_RPM;
		if(sc < 1.0f) lim *= sc;
		if(lim < (float)SPEED_PI_LIMIT_MIN) lim = (float)SPEED_PI_LIMIT_MIN;
	}

	/* 5) 电压型PI; 输出饱和时停止积分(抗饱和) */
	err = ref - spd_fb;
	pi_out = SPEED_KP * err + spd_i;
	if(!((pi_out >  lim && err > 0.0f) ||
	     (pi_out < -lim && err < 0.0f)))
	{
		spd_i += SPEED_KI * err * SPEED_CTRL_TS;
	}

	/* 6) 转矩电流软限幅: 本模式没有电流内环, iq 超限时只能把电压调节量往回拉。
	   相当于给电压型PI加了一个很慢的电流外环, 防止失步瞬间把电流顶到采样饱和。
	   iq 是换相角下的 q 轴电流(已按转向折算), 反转时 iq 本身是负的。 */
	{
		float iq_dir = (float)smo.dir * (float)pmotor->mea_iq;
		if(iq_dir > (float)pmotor->current_limit)
		{
			float over = iq_dir - (float)pmotor->current_limit;
			spd_i -= SPEED_IQ_LIM_K * over * SPEED_CTRL_TS * (float)smo.dir;
		}
	}
	_constrain(spd_i, -lim, lim);
	pi_out = SPEED_KP * err + spd_i;
	_constrain(pi_out, -lim, lim);

	/* 7) 前馈 + 调节量 -> Vq */
	vq = ff + (int32_t)pi_out;
	_constrain(vq, -SPEED_VQ_MAX, SPEED_VQ_MAX);
	pmotor->out_vq = (int16_t)vq;

	/* 8) d 轴只保留解耦项: Vd = -we*L*Iq。高速时这一项有几伏,
	   不补的话电流矢量会偏到 d 轴, 同样的转矩要多花电流 */
#if SPEED_VD_COMP
	pmotor->out_vd = (int16_t)Smo_VdDecoupleQ15((int32_t)pmotor->mea_iq);
#else
	pmotor->out_vd = 0;
#endif

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
    {
        int32_t pos_now = GetMotorPos(pmotor);   /* 顺带刷新 ele_angle, 供调试观察 */
        /* 无感/低速I/f 模式下换相角由观测器(或 I/f 频率积分)给: 这里若被编码器值覆盖,
           紧接着的 GetIqId 就会用错误的角度做 Park 变换, iq/id 变成假值(编码器失效时
           更严重, 角度是冻结的)。转矩电流软限幅、换相角自校准、I/f 电流环都依赖 iq/id,
           所以必须区分模式。*/
        if(pmotor->sensorless_flag == 0 && pmotor->if_flag == 0) pmotor->motor_axle_angle = pos_now;
    }
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
            /* 零位必须在"还顶着最大定位电压"时取, 释放过程中转子会开始回弹, 再取就不准了 */
            if(pmotor->out_vd >= (int16_t)ud_max)
            {
                pmotor->_encoder->offset_value = MT6701_GetRawData();
            }
            pmotor->out_vq = 0;
            /* d轴定位电压按斜率释放: 原来从 GPS_MAX_VOL 一把清 0, 转子会被从定位点
               "弹回"磁阻死点, 表现为"初始化完成后抖一下才停下来"。 */
            if(pmotor->out_vd > (int16_t)(GPS_VOL_DOWN * (int32_t)interval_ms))
            {
                pmotor->out_vd -= (int16_t)(GPS_VOL_DOWN * (int32_t)interval_ms);
                pmotor->motor_axle_angle = 0;
                SetPhaseVol(pmotor);
                return 0;
            }
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
    /* 转速判据优先用观测器估计值: 编码器没脉冲时 mea_speed 恒为 0(实测就是这样),
       只看它会让堵转检测在电机正常运转时误判, 2 秒后直接把电机关掉。 */
    int32_t spd = abs(pmotor->mea_speed);
    if(pmotor->sensorless_flag == 1 && abs(pmotor->est_speed) > spd)
    {
        spd = abs(pmotor->est_speed);
    }
    /* 低速 I/f 段: 转速就是给定频率, 而 mea_speed 在这个区间要么是 0(编码器没脉冲)
       要么被量化成 36rpm 的台阶(14 位编码器 100us 采一次), 拿它判堵转必然误动作:
       50rpm/1.5A 正常拖动会被判成"转速<5 且 电流>1A", 2 秒后关电机 —— 这也是
       "转一下停一下"的来源之一。所以这里直接用 I/f 的给定转速。
       真正的堵转由电流软限幅/过流保护兜底。*/
    if(pmotor->if_flag == 1)
    {
        int32_t c = Smo_IfSpeedRpm();
        if(c < 0) c = -c;
        if(c > spd) spd = c;
    }
    if(spd < MOTOR_STALL_SPEED_THRESHOLD_RPM &&
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

    /* 未滤波值: 低速 I/f 电流环用它。滤波后的 iq 截止频率只有 24Hz, 对 64Hz 的
       电流环来说相位滞后太大, 会把闭环带宽拖死。 */
    pmotor->mea_iq_raw = park.iq;
    pmotor->mea_id_raw = park.id;
    
    /* 电流滤波 */
    pmotor->mea_iq = LowPass_Filter_Q15(pmotor->mea_iq, pmotor->_iq_lpf); 
    pmotor->mea_id = LowPass_Filter_Q15(pmotor->mea_id, pmotor->_id_lpf); 
}
