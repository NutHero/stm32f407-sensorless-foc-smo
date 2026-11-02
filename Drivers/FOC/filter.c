#include "filter.h"


Lpf15 m1_lpf_speed = {0, fToQ15(0.62)}; //速度低通滤波结构体变量
Lpf15 m1_lpf_iq = {0, fToQ15(0.015)}; //Q轴电流低通滤波结构体变量
Lpf15 m1_lpf_id = {0, fToQ15(0.1)}; //D轴电流低通滤波结构体变量
Lpf m1_lpff_speed_limit = {0, 0.0002}; //速度爬升斜率。速度模式下用来限制目标速度变化速率，防止Uq突变太快。使用浮点型提高精度
Lpf m1_lpff_pos_limit = {0, 0.0001}; //位置加速斜率。位置模式下用来限制目标位置变化速率，防止Uq突变太快
Lpf15 busI_lpf = {0, fToQ15(0.1)}; //母线电流低通滤波器结构体变量

/**
 * @brief 一阶IIR低通滤波器，定点实现
 * @param X 当前输入，Q15格式
 * @param Plpf 指向低通滤波结构体变量
 * @return 当前输出，Q15格式
 */
int32_t LowPass_Filter_Q15(int32_t X, Lpf15 *Plpf)
{
    int32_t temp2,temp1,out;    
    temp1 = (int32_t)Plpf->Yn * (32767 - Plpf->q); // Yn*(1-q)
    temp2 = (int32_t)X * Plpf->q;                 // X*q
    out = temp1 + temp2;
    out += (int32_t)1 << 14;  // 加上0.5（Q15）进行四舍五入
    out >>= 15;// 右移15位，回Q15格式
    Plpf->Yn = out;
    return (int32_t)out;
}

/**
 * @brief 一阶IIR低通滤波器
 * @param X 当前输入
 * @param Y_prev 上一次输出
 * @param q 滤波系数
 * @return 当前输出
 */
float LowPass_Filter(float X, Lpf *Plpf)
{
    float temp;
    temp = Plpf->Yn * (1 - Plpf->q) + X * Plpf->q;
    Plpf->Yn = temp; 
    return temp; //输出按四舍五入计算
}

