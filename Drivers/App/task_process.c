#include "task_process.h"
#include "decodeVofa+.h"

sys_sta_ sys_sta;

float bus_i; //母线电流
float bus_v; //母线电压


/** 
 * @brief 开启系统任务调度 
 * @return 无
 */
void task_scheduler_start(void)
{
    HAL_TIM_Base_Start_IT(&g_tim6_handle);   
}

/**
 * @brief  关闭系统任务调度   
 * @return 无
 */
void task_scheduler_stop(void)
{
    HAL_TIM_Base_Stop_IT(&g_tim6_handle);
}

/**
 * @brief 定时器中断回调函数
 * @retval 无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        sys_process_run(&motor1);
    }
}

/**
 * @brief 运行任务管理
 * @note 1ms周期调用
 * @return 无
 */
void sys_process_run(dev_motor *pmotor)
{
    sys_sta.tick++;
    if(sys_sta.tick % 10 == 0) //系统任务处理周期，10ms
    {
        switch(sys_sta.current_process)
        {
            case PROC_GET_OFFSET:
            {
                pmotor->resetAngle_flag = 0;
                __disable_irq();
                if(pmotor->offset_current_flag == 1)
                {
                    MotorEnable(pmotor, 1); //打开电机用于之后电机校准角度
                    pmotor->sta = 0;//电机状态置0，避免电机被其他中断控制
                    sys_sta.current_process = PROC_RESET_ANGLE;
                }
                __enable_irq();
                break;
            }
            case PROC_RESET_ANGLE:
            {
                if(pmotor->_encoder->offset_value > 0) //手动设置了编码器偏置值时，跳过电机校准步骤
                {
                    pmotor->resetAngle_flag = 1;
                    MotorEnable(pmotor, 0);
					UpdateMotorSta(pmotor, 0);
                    /* 电机为位置控制时，将目标角度设置为当前角度，避免电机启动时抖动 */
                    if(pmotor->mode == BLDC_MODE_POS_LOOP || pmotor->mode == BLDC_MODE_POS_SPEED_LOOP || 
                        pmotor->mode == BLDC_MODE_POS_CURRENT_LOOP || pmotor->mode == BLDC_MODE_POS_CONTROL)
                    {
                        pmotor->target_angle = pmotor->mea_angle;
						pmotor->target_angle_pre = pmotor->target_angle;
						pmotor->_trap_planner->current_pos_q = pmotor->mea_angle;
                    }
					pmotor->initial_flag = 1;
                    sys_sta.current_process = PROC_WAIT_CMD;
                }
                else
                {
                    if(pmotor->resetAngle_flag == 0)
                    {
                        if(ResetAngle(pmotor, 10))
                        {
                            pmotor->resetAngle_flag = 1;
                        }
                    }
                    if(pmotor->resetAngle_flag)
                    {
                        MotorEnable(pmotor, 0);
                        
                        /* 电机为位置控制时，将目标角度设置为当前角度，避免电机启动时抖动 */
                        if(pmotor->mode == BLDC_MODE_POS_LOOP || pmotor->mode == BLDC_MODE_POS_SPEED_LOOP || 
                            pmotor->mode == BLDC_MODE_POS_CURRENT_LOOP || pmotor->mode == BLDC_MODE_POS_CONTROL)
                        {
                            pmotor->target_angle = pmotor->mea_angle;
                        }
						pmotor->initial_flag = 1;
                        sys_sta.current_process = PROC_WAIT_CMD;
                    }
                }
                break;
            }
            case PROC_WAIT_CMD:
            {
                __disable_irq();
                if(pmotor->mode == BLDC_MODE_SPEED_LOOP || pmotor->mode == BLDC_MODE_SPEED_CONTROL) //在速度模式下
                {                         
                    if(pmotor->sta) //且电机是开启
                    {
                        if(StallCheck(pmotor)) //执行堵转检测
                        {
                           sys_sta.current_process = PROC_CLOSE_MOTOR;
                        }    
                        if(GetMotorDir(pmotor) != pmotor->dir_pre) //电机方向发生改变时清除PID误差可以加快电机正反转
                        {
                            pid_clear_q15(pmotor->_speed_pid);
                            pid_clear_q15(pmotor->_iq_pid);
                            pmotor->dir_pre = GetMotorDir(pmotor);
                        }                        
                    }
                }
                __enable_irq();
                
                decode_vofa_command(); //解析VOFA+下发命令
                
                if(pmotor->sta == 1)
                {
                    sys_sta.current_process = PROC_OPEN_MOTOR;
                }
                else if(pmotor->sta == 0)
                {
                    sys_sta.current_process = PROC_CLOSE_MOTOR;
                }
                break;
            }
            case PROC_OPEN_MOTOR:
            {
                if(pmotor->sta == 1)
                {
                    MotorEnable(pmotor, 1);
                }
                sys_sta.current_process = PROC_WAIT_CMD;
                break;
            }
            case PROC_CLOSE_MOTOR:
            {
                if(pmotor->sta == 0)
                {
                    MotorEnable(pmotor, 0);
                }
                sys_sta.current_process = PROC_WAIT_CMD;
                break;
            }
            case PROC_STORE_ANGLE:
            {
                sys_sta.current_process = PROC_WAIT_CMD;
                break;
            }
            case PROC_GET_ANGLE:
            {
                sys_sta.current_process = PROC_WAIT_CMD;
                break;
            }
            case PROC_GET_BUS_CUR:
            {
                sys_sta.current_process = PROC_WAIT_CMD;
                break;
            }
			case PROC_GET_BUS_VOL:
			{
				sys_sta.current_process = PROC_WAIT_CMD;
				break;
			}
			case PROC_GET_TEMP:
			{
				sys_sta.current_process = PROC_WAIT_CMD;
				break;
			}
            case PROC_ERROR://8
            default: break;
        }    
    }   
}
