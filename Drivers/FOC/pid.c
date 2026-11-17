#include "pid.h"


/****** 电机1 ******/
PID_Q15_TypeDef m1_speed_pid; //速度环PID结构体句柄
PID_Q15_TypeDef m1_iq_pid; //电流环Q轴PID结构体句柄
PID_Q15_TypeDef m1_id_pid; //电流环D轴PID结构体句柄
PID_Q15_TypeDef m1_pos_pid; //位置环PID结构体句柄

/**
  * @brief  由积分项输出限幅(IOutMax)换算出累计误差的限幅(SumMax)
  * @note   积分项 i_term = (Integral * SumError) >> 26，要让 i_term 不超过 IOutMax，
  *         就要 SumError <= (IOutMax << 26) / Integral。
  *         这里只在初始化/改参数时算一次，避免在每个控制周期里做 64 位除法。
  * @param  PID PID结构体句柄指针
  * @return 无
  */
static void pid_update_summax(PID_Q15_TypeDef *PID)
{
    int64_t s;

    if(PID->Integral > 0 && PID->IOutMax > 0)
    {
        s = ((int64_t)PID->IOutMax << (16 + PID_Q_Format)) / (int64_t)PID->Integral;
        if(s > (int64_t)INT32_MAX) s = (int64_t)INT32_MAX;
        PID->SumMax = (int32_t)s;
    }
    else
    {
        PID->SumMax = INT32_MAX;
    }
    PID->SumMaxI = PID->Integral;
}

/**
  * @brief  单独设置积分项输出限幅
  * @param  PID PID结构体句柄指针
  * @param  i_max 积分项最大输出，Q15格式
  * @return 无
  */
void pid_set_iout_max_q15(PID_Q15_TypeDef *PID, int32_t i_max)
{
    if(i_max < 0) i_max = -i_max;
    PID->IOutMax = i_max;
    pid_update_summax(PID);
}

/**
  * @brief 初始化一个PID结构体
  * @param  PID PID结构体句柄指针
  * @param  kp 比例系数
  * @param  ki 积分系数
  * @param  kd 微分系数
  * @return 无
  */
static void pid_init_one(PID_Q15_TypeDef *PID, float kp, float ki, float kd)
{
    PID->SetPoint = 0;
    PID->ActualValue = 0;
    PID->SumError = 0;
    PID->Proportion = (int16_t)(kp * (float)Q_Format);
    PID->Integral = (int16_t)(ki * (float)Q_Format);
    PID->Derivative = (int16_t)(kd * (float)Q_Format);
    PID->Psum = 0;
    PID->Isum = 0;
    PID->Error = 0;
    PID->LastError = 0;
    PID->PreError = 0;
    /* 默认积分项输出上限 = Integral*32，与旧版"积分器打满"的行为一致。
       想让积分出力更大(或更小)时，用 pid_set_iout_max_q15() 单独改。 */
    PID->IOutMax = (int32_t)PID->Integral * 32;
    pid_update_summax(PID);
}

/**
  * @brief PID初始化，Q15格式
  * @return 无
  */
void pid_init_q15(void)
{
    /***** 电机1 ******/
    pid_init_one(&m1_speed_pid, M1_KP_SPEED, M1_KI_SPEED, M1_KD_SPEED);
    pid_init_one(&m1_iq_pid,    M1_KP_IQ,    M1_KI_IQ,    M1_KD_IQ);
    pid_init_one(&m1_id_pid,    M1_KP_ID,    M1_KI_ID,    M1_KD_ID);
    pid_init_one(&m1_pos_pid,   M1_KP_POS,   M1_KI_POS,   M1_KD_POS);
}

/**
  * @brief 位置式PID，Q15格式实现
  * @param PID PID结构体句柄指针
  * @param feedback 反馈值
  * @return PID输出，Q15格式
  * @note  相对旧版的改动：
  *        1. 累计误差改为 64 位累加，旧版 (int32_t)Error<<16 在 |Error|>32767 时会溢出；
  *        2. 积分项有独立的输出限幅 IOutMax，不会再吃满整个输出范围；
  *        3. 抗饱和按方向判断，只有"同向饱和"才停止积分，反向时允许积分退出来。
  */
int16_t pid_cpos_q15(PID_Q15_TypeDef *PID, int32_t feedback)
{
    int32_t p_term, i_term, d_term, output, last_sumerror;
    int64_t sum;

    /* 上位机(VOFA+)可能在线改了积分系数，这里发现不一致就重算限幅 */
    if(PID->SumMaxI != PID->Integral) pid_update_summax(PID);

    PID->Error = PID->SetPoint - feedback; // 计算误差
    last_sumerror = PID->SumError;

    /* 累计误差（64 位） + 限幅 */
    sum = (int64_t)PID->SumError + ((int64_t)PID->Error << 16);
    if(sum > (int64_t)PID->SumMax)  sum = (int64_t)PID->SumMax;
    if(sum < -(int64_t)PID->SumMax) sum = -(int64_t)PID->SumMax;
    PID->SumError = (int32_t)sum;

    // 比例项：Q10 * Q15 = Q25，右移10位→Q15
    p_term = ((int32_t)PID->Proportion * PID->Error) >> PID_Q_Format;
    PID->Psum = p_term;

    // 积分项：Q10 * Q31 = Q41，右移26位→Q15
    i_term = (int32_t)(((int64_t)PID->Integral * (int64_t)PID->SumError) >> (16 + PID_Q_Format));
    PID->Isum = i_term;

    if(PID->Derivative > 0)
    {
        // 微分项：Q10 * (Q15-Q15) = Q25，右移10位→Q15
        d_term = ((int32_t)PID->Derivative * (PID->Error - PID->LastError)) >> PID_Q_Format;
        output = p_term + i_term + d_term;    // PID控制
    }
    else
    {
        output = p_term + i_term; // PI控制
    }

    /* 输出限幅 + 抗积分饱和：只有误差与饱和同方向时才回退累计误差，
       误差反向时保留累积，让积分能顺着退出来(否则退饱和会明显变慢) */
    if(output > 32767)
    {
        output = 32767;
        if(PID->Error > 0) PID->SumError = last_sumerror;
    }
    else if(output < -32768)
    {
        output = -32768;
        if(PID->Error < 0) PID->SumError = last_sumerror;
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
    PID->Psum = 0;
    PID->Isum = 0;
}

/**
  * @brief 预装积分器，使下一次 pid_cpos_q15 的积分项输出约等于 i_out(Q15)
  * @note  用于PI控制器的无扰切换: 切换瞬间让PI输出直接接续到目标值，避免输出跳变。
  * @param PID PID结构体句柄指针
  * @param i_out 期望的积分项输出，Q15格式
  * @return 无
  */
void pid_preload_q15(PID_Q15_TypeDef *PID, int32_t i_out)
{
    int32_t i_max;
    int64_t sum;

    if(PID->Integral <= 0)
    {
        PID->SumError = 0;
        return;
    }

    i_max = PID->IOutMax;              /* 积分器输出上限 */
    if(i_out >  i_max) i_out =  i_max;
    if(i_out < -i_max) i_out = -i_max;

    sum = ((int64_t)i_out << (16 + PID_Q_Format)) / (int64_t)PID->Integral;
    if(sum >  (int64_t)PID->SumMax) sum =  (int64_t)PID->SumMax;
    if(sum < -(int64_t)PID->SumMax) sum = -(int64_t)PID->SumMax;
    PID->SumError = (int32_t)sum;
    PID->LastError = PID->Error;        /* 防止预装后微分项冲击 */
}
