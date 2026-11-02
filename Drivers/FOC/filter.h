#ifndef __FILTER_H
#define __FILTER_H

#include "./SYSTEM/sys/sys.h"
#include "sincos16.h"


/* 一阶低通滤波结构体 */
typedef struct{
    int32_t Yn; //旧值
    int16_t q; //滤波系数
}Lpf15;

/* 一阶低通滤波结构体 */
typedef struct{
    float Yn; //旧值
    float q; //滤波系数
}Lpf;

/**
 * @brief 一阶低通滤波器
 * @param Yn_1 储存滤波后的数据&新输入数据
 * @param Xn 旧值
 * @param a 滤波系数
 */
#define FirstOrderRC_LPF(Yn_1, Xn, a) Yn_1 = (1 - a) * Yn_1 + a * Xn; 

///*一阶低通滤波器，Q15格式 */
//#define FirstOrderRC_LPF15(Yn_1, Xn, a) Yn_1 = (((((32768 - a) * (int32_t)Yn_1) >> 15) + (((int32_t)a * Xn) >> 15) + (1 << 14)) >> 15); 

/* Q15格式一阶低通滤波器 */
#define FirstOrderRC_LPF15(Yn_1, Xn, a) \
    do { \
        int32_t temp = (int32_t)(32768 - a) * Yn_1; \
        temp += (int32_t)a * Xn; \
        Yn_1 = (temp + (1 << 14)) >> 15; \
    } while(0)
    
extern Lpf15 m1_lpf_speed; //速度低通滤波结构体变量
extern Lpf15 m1_lpf_iq; //Q轴电流低通滤波结构体变量
extern Lpf15 m1_lpf_id; //D轴电流低通滤波结构体变量
extern Lpf m1_lpff_speed_limit ; //速度爬升斜率。速度模式下用来限制目标速度变化速率，防止Uq突变太快。使用浮点型提高精度
extern Lpf m1_lpff_pos_limit;
extern Lpf15 busI_lpf;

int32_t LowPass_Filter_Q15(int32_t X, Lpf15*Plpf); //一阶低通滤波器定点实现
float LowPass_Filter(float X, Lpf *Plpf);
    
#endif
