#ifndef __PID_H
#define __PID_H

#include "./SYSTEM/sys/sys.h"

#define PID_Q_Format    (int32_t)10      //PID系数Q格式，Q10
#define Q_Format    ((int32_t)2 << (PID_Q_Format - 1)) //系数

/******* 电机1 *******/
/* 速度环系数 */
#define M1_KP_SPEED    0.1   //速度环比例项系数，Max=31
#define M1_KI_SPEED    0.1     //速度环积分项系数
#define M1_KD_SPEED    0       //速度环微分项系数
/* 位置环系数 */
#define M1_KP_POS       1.5        //位置环比例项系数
#define M1_KI_POS       0           //位置环积分项系数
#define M1_KD_POS       0       //位置环微分项系数
/* 电流环Q系数 */
#define M1_KP_IQ        0.1       //电流环Iq比例项系数
#define M1_KI_IQ        0.05        //电流环Iq积分项系数
#define M1_KD_IQ        0           //电流环Iq微分项系数
/* 电流环D系数 */
#define M1_KP_ID        0.5       //电流环Id比例项系数
#define M1_KI_ID        0.3       //电流环Id积分项系数
#define M1_KD_ID        0           //电流环Id微分项系数

/* PID结构体定义 */
typedef struct {
    int32_t SetPoint;        // 目标值，Q15格式
    int16_t ActualValue;     // 期望值，Q15格式
    int32_t SumError;        // 累计偏差，Q31格式（防止溢出）
    int32_t SumMax;          // 累计偏差限幅，由 IOutMax 换算得到（初始化时算好，避免每拍做除法）
    int16_t SumMaxI;         // 算 SumMax 时用的积分系数，用于发现上位机在线改了 I 后自动重算
    int16_t Proportion;      // 比例系数P，Q12格式，Max=8
    int16_t Integral;        // 积分系数I，Q12格式，
    int16_t Derivative;      // 微分系数D，Q12格式
    int32_t IOutMax;         // 积分项输出限幅，Q15格式。积分项最大只能出力到这里，防止积分吃满整个输出
    int32_t Psum;	//测试用
    int32_t Isum;	//测试用
    int32_t Error;          // error[1]，Q15格式
    int32_t LastError;      // error[-1]，Q15格式
    int32_t PreError;       // error[-2]，Q15格式
} PID_Q15_TypeDef;

/******* 电机1 ******/
extern PID_Q15_TypeDef m1_speed_pid; //速度环PID结构体句柄
extern PID_Q15_TypeDef m1_iq_pid; //电流环Q轴PID结构体句柄
extern PID_Q15_TypeDef m1_id_pid; //电流环D轴PID结构体句柄
extern PID_Q15_TypeDef m1_pos_pid; //位置环PID结构体句柄

int16_t pid_cpos_q15(PID_Q15_TypeDef *PID, int32_t feedback);//位置式PID，Q15格式
void pid_init_q15(void);//位置式PID初始化，Q15格式
void pid_clear_q15(PID_Q15_TypeDef*PID); //清零PID参数，Q15格式
void pid_preload_q15(PID_Q15_TypeDef *PID, int32_t i_out); //预装积分器，使积分项输出=i_out(Q15)，用于无扰切换
void pid_set_iout_max_q15(PID_Q15_TypeDef *PID, int32_t i_max); //单独设置积分项输出限幅，Q15

#endif
