#include "pid.h"


/****** 电机1 ******/
PID_Q15_TypeDef m1_speed_pid; //速度环PID结构体句柄
PID_Q15_TypeDef m1_iq_pid; //电流环Q轴PID结构体句柄
PID_Q15_TypeDef m1_id_pid; //电流环D轴PID结构体句柄
PID_Q15_TypeDef m1_pos_pid; //位置环PID结构体句柄

/**
  * @brief PID初始化，Q15格式
  * @return 无
  */
void pid_init_q15(void)
{
    /***** 电机1 ******/
    /* 速度环参数初始化 */
    m1_speed_pid.SetPoint = 0;
    m1_speed_pid.ActualValue = 0;
    m1_speed_pid.SumError = 0;
    m1_speed_pid.Proportion = (uint16_t)((float)M1_KP_SPEED * Q_Format);  //比列项
    m1_speed_pid.Integral = (uint16_t)((float)M1_KI_SPEED * Q_Format);     //积分项
    m1_speed_pid.Derivative = (uint16_t)((float)M1_KD_SPEED * Q_Format);   //微分项
    m1_speed_pid.Error = 0;
    m1_speed_pid.LastError = 0;
    m1_speed_pid.PreError = 0;
    
    /* 电流环Q轴参数初始化 */
    m1_iq_pid.SetPoint = 0;
    m1_iq_pid.ActualValue = 0;
    m1_iq_pid.SumError = 0;
    m1_iq_pid.Proportion = (uint16_t)((float)M1_KP_IQ * Q_Format);  //比列项
    m1_iq_pid.Integral = (uint16_t)((float)M1_KI_IQ * Q_Format);     //积分项
    m1_iq_pid.Derivative = (uint16_t)((float)M1_KD_IQ * Q_Format);   //微分项
    m1_iq_pid.Error = 0;
    m1_iq_pid.LastError = 0;
    m1_iq_pid.PreError = 0;
    
    /* 电流环D轴参数初始化 */
    m1_id_pid.SetPoint = 0;
    m1_id_pid.ActualValue = 0;
    m1_id_pid.SumError = 0;
    m1_id_pid.Proportion = (uint16_t)((float)M1_KP_ID * Q_Format);  //比列项
    m1_id_pid.Integral = (uint16_t)((float)M1_KI_ID * Q_Format);     //积分项
    m1_id_pid.Derivative = (uint16_t)((float)M1_KD_ID * Q_Format);   //微分项
    m1_id_pid.Error = 0;
    m1_id_pid.LastError = 0;
    m1_id_pid.PreError = 0;
    
    /* 位置环参数初始化 */
    m1_pos_pid.SetPoint = 0;
    m1_pos_pid.ActualValue = 0;
    m1_pos_pid.SumError = 0;
    m1_pos_pid.Proportion = (uint16_t)((float)M1_KP_POS * Q_Format);  //比列项
    m1_pos_pid.Integral = (uint16_t)((float)M1_KI_POS * Q_Format);     //积分项
    m1_pos_pid.Derivative = (uint16_t)((float)M1_KD_POS * Q_Format);   //微分项
    m1_pos_pid.Error = 0;
    m1_pos_pid.LastError = 0; 
    m1_pos_pid.PreError = 0;		
}

/**
  * @brief 位置式PID，Q15格式实现
  * @param PID PID结构体句柄指针
  * @param feedback 反馈值，Q15格式
  * @return PID输出，Q15格式
  */
int16_t pid_cpos_q15(PID_Q15_TypeDef *PID, int32_t feedback)
{
    int32_t p_term, i_term, d_term, output, last_sumerror;
    int64_t sum; 
  
    PID->Error = PID->SetPoint - feedback; // 计算误差  
    last_sumerror = PID->SumError;
    sum = (int64_t)PID->SumError + ((int32_t)PID->Error << 16); // 累计误差
    
    /* 积分限幅 */
    if(sum > INT32_MAX) 
    {
        sum = INT32_MAX;
    }
    else if(sum < -INT32_MAX) 
    {
        sum = -INT32_MAX;   
    }
    PID->SumError = sum;  
    
    // 比例项：Q12 * Q15 = Q25，右移12位→Q15
    p_term = ((int32_t)PID->Proportion * PID->Error) >> PID_Q_Format;
    PID->Psum = p_term;
    
    // 积分项：Q12 * Q31 = Q43，右移28位→Q15
    i_term = (int32_t)(((int64_t)PID->Integral * PID->SumError) >> (16 + PID_Q_Format));
    PID->Isum = i_term;
    
    if(PID->Derivative > 0)
    {
        // 微分项：Q12 * (Q15-Q15) = Q12 * Q15 = Q27，右移12位→Q15
        d_term = ((int32_t)PID->Derivative * ((int32_t)PID->Error - PID->LastError)) >> PID_Q_Format;
        output = p_term + i_term + d_term;    // PID控制
    }
    else
    {
        output = p_term + i_term; //PI控制
    }

    /* 输出限幅 */
    if(output > 32767)
    {
        PID->SumError = last_sumerror; //使用上一次累计误差
        output = 32767;
    }
    else if(output < -32768) 
    {
        PID->SumError = last_sumerror;
        output = -32768; 
    }
    
    PID->LastError = PID->Error;
    PID->ActualValue = (int16_t)output;   
    
    return (int16_t)output;
}

/**
 * @brief PID参数清零,Q15格式PID
 * @param PID PID结构体句柄指针
 */
void pid_clear_q15(PID_Q15_TypeDef*PID)
{
    PID->ActualValue = 0;
    PID->SumError = 0;
    PID->Error = 0;
    PID->LastError = 0;
    PID->PreError = 0;
}

