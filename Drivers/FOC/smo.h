#ifndef __SMO_H
#define __SMO_H

#include "./SYSTEM/sys/sys.h"
#include "bldc_control.h"


typedef struct
{
	float Ealpha_esc,Ebeta_esc; //alpha,beta轴反电动势
	float Ialpha_esc, Ibeta_esc;	//alpha,beta轴估计电流，单位：A
	float Ialpha_esc_pre, Ibeta_esc_pre;	//alpha,beta轴估计电流旧值，单位：A
	float Ialpha_err, Ibeta_err;	//alpha,beta轴电流误差，单位：A
	float ia,ib,ic;	//A,B,C三相实际电流，单位：A
	float Ualpha,Ubeta;	//alpha,beta轴注入电压
	float Ialpha_mea, Ibeta_mea;	//alpha,beta轴实际电流，单位：A
	float Valpha_esc,Vbeta_esc; //alpha,beta轴估计电压
	float we_est; //电角速度估计值，单位：rad/s
	float we_mea;	//实际电角速度，单位：rad/s
	float theta_err;	//角度误差值，单位：rad
	float theta_err_limit;	//限制角度误差范围
	float sum_pll;	//锁相环积分项
	float ki_pll;	//锁相环积分系数
	float kp_pll;	//锁相环比列系数
	float theta_mea;	//电角度测量值，单位：rad。范围：0~2π
	float theta_est;	//电角度估计值，单位：rad。范围：0~2π
	float h,h1, h2;	//滑膜增益h
	/* 无感启动 */
	uint8_t start_state;	/* 0 = 开环强拖, 1 = 已切入无感 */
	float   ol_theta;		/* 开环强拖电角度 [rad] */
}Smo;
extern Smo smo;

void BLDC_Smo_Init(void);
void Smo_Control(uint32_t interver_us);
uint8_t Smo_Startup(void);			/* 返回1表示检测到反电动势, 可切无感 */
int16_t Smo_ElecToQ15(float theta_rad);	/* 电角度[rad] -> Q15换相角(motor_axle_angle) */
void Smo_Startup_Reset(void);		/* 复位启动状态 */

#endif
