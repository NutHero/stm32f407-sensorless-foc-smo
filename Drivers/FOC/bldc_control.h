#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#include "./SYSTEM/sys/sys.h"
#include "sensor.h"
#include "filter.h"
#include "pid.h"
#include "trap_planner.h"  
#include "clarke_park.h"
#include "svpwm.h"
#include "spwm.h"
#include "time.h"
#include "gpio.h"
#include "adc.h"
#include <stdlib.h>
#include "smo.h"

/**
 * @brief 范围限制
 * @param amt 要判断的数据
 * @param low 范围最低值
 * @param heigh 范围最高值
 */
#define _constrain(amt, low, high) amt = ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

/* 电机控制模式 */
typedef enum{
    BLDC_MODE_ERROR = 0,                   //错误
    BLDC_MODE_OPEN_LOOP = 1,               //开环强拖，仅测试
    BLDC_MODE_CLOSE_LOOP = 2,              //角度闭环，仅测试
    BLDC_MODE_SPEED_LOOP = 3,              //速度环，仅测试
    BLDC_MODE_CURRENT_LOOP = 4,            //电流环，仅测试
    BLDC_MODE_POS_LOOP = 5,                //位置环，仅测试
    BLDC_MODE_POS_SPEED_LOOP = 6,           //位置+速度环，仅测试
    BLDC_MODE_POS_CURRENT_LOOP = 7,         //位置+电流环，仅测试
    BLDC_MODE_SPEED_CONTROL = 8,       //速度+电流环，速度模式
    BLDC_MODE_POS_CONTROL = 9,              //位置+速度+电流环，位置模式
	BLDC_MODE_TORQUE_CONTROL = 10,		//电流力矩模式
}Motor_mode;

/* 调制模式选择 */
#define SVPWM   
//#define SPWM
                     
/* 电机参数 */
#define SPEED_MAX     8300         //电机最大速度，单位：转/分
#define M1_POWER_VOL   24           //额定电压，单位：V
#define M1_KV_VALUE			360		//电机KV值
#define M1_MAX_CURRENT     23000       //额定电流，单位：1mA
#define M1_POLE_PAIRS  		7          //电机极对数

#define OPEN_LOOP_COUNT		200		//开环角度自增量，范围：0~10000
#define OPEN_LOOP_VOL		4000		//开环电压；范围：-32768~32767。负值时电机反转
#define CLOSELOOP_VOL		OPEN_LOOP_VOL	//单角度闭环电压；范围：-32768~32767。负值时电机反转

/* 电机1限制 */
#define M1_Control_Mode	 		BLDC_MODE_SPEED_LOOP	//控制模式
#define M1_POS_DIE_ARE          1           //位置环死区，单位：1°
#define M1_IQ_DIE_ARE           20        //电流环Iq允许误差
#define M1_ID_DIE_ARE           20        //电流环Id允许误差
#define M1_Current_LIMIT     	5000       //电流限制，单位：mA
#define M1_Speed_LIMIT          5000       //速度限制，单位：RPM
#define M1_GETSTA_ANGLE         2         //角度计算频率,单位：50us
#define M1_Control_FRE          2         //电机控制频率。

/* 堵转判断条件：速度低于阈值且电流高于阈值 */
#define MOTOR_STALL_SPEED_THRESHOLD_RPM     5           // 速度阈值，低于此值认为可能堵转
#define MOTOR_STALL_CURRENT_THRESHOLD_MA    1000        // 电流阈值，高于此值认为负载过大
#define MOTOR_STALL_TIME_THRESHOLD_MS       200        // 堵转持续时间阈值(2秒)，单位：10ms

#define CIR_THRESHOLD       280     //判断电机转完一圈后溢出的阈值

/* 电机定位设置 */
#define GPS_MAX_VOL     5000   //定位电压，max=32767。注：定位的电压不要太大，否则驱动芯片容易损坏
#define GPS_VOL_RAMP    5       //电压增加速率，单位：1uq
#define GPS_TIME        3000U    //定位时间，单位：1ms

/* 电流采样 */
#define VOL_GAIN    50.0f       //电压增益
#define SAMP_R  0.005f          //采样电阻值，单位：欧姆
#define ADC_RATIO 0.000805f     //ADC分辨率，= (3.3f / 4096)
#define MID_VOL 1.65f           //AD采样芯片 中点电压

/* 电机控制对象结构体 */                
typedef struct{
	/* 电机状态变量 */
	uint8_t sta;        //电机状态，1开启，0关闭
	uint8_t resetAngle_flag;	//零位校准完成标志
	uint8_t offset_current_flag;	//静态电流计算完成标志
	uint8_t initial_flag;	//初始化完成标志，1完成
	Motor_mode mode;       //电机控制模式
	
	/* 用户调用变量 */
    int32_t target_speed;  	//目标速	度，单位：RPM
	int32_t target_angle;		//目标角度，单位：度
	int32_t mea_angle;			//当前角度，单位：度
    int32_t mea_speed;      //当前转速，单位：RPM	
	uint16_t physic_angle;        //输出轴角度，0~360度
	
	/* 私有变量 */
	int32_t open_vq;	//开环/角度闭环电压vq
	int16_t target_iq;	//目标iq，单位：mA，范围：-16384~16384 
	int16_t target_id;	//目标id，单位：mA，范围：-16384~16384 
	int32_t speed_limit;	//速度限制，单位：RPM
	int32_t target_angle_pre; /* 目标角度旧值，用于判断有没有新的目标指令 */
	int32_t current_limit;	//电流限制，单位：mA 
    uint8_t pole;       //电机极对数
	uint16_t control_pre;	//电机控制频率计数
    int8_t dir;         //电机方向，1顺时针，-1逆时针
    int8_t dir_pre;     //电机之前的方向，用于判断电机方向有没有发生改变
	uint32_t ele_angle;		//实际电角度，范围：编码器最小值 x 极对数 ~ 编码器最大值 x 极对数
    int16_t motor_axle_angle;         //电机轴位置，Q15格式，范围：-32768~32767 
    int16_t motor_axle_angle_pre;   //电机轴之前的位置，Q15格式，用于计算电机旋转的圈数
    int32_t cir;        //电机旋转的总圈数，区分方向
    int16_t out_vq;         //电机q轴电压，Q15格式，范围：-32768~32767
    int16_t out_vd;         //电机d轴电压，Q15格式，范围：-32768~32767
    int16_t mea_iq;         //电机q轴电流，单位：mA
    int16_t mea_id;         //电机d轴电流，单位：mA
    int16_t mea_ia;         //电机A相电流，单位：mA
    int16_t mea_ib;         //电机B相电流，单位：mA
    int16_t mea_ic;         //电机C相电流，单位：mA
    int16_t offset_ia;  //A相静态电流，储存电机停止时对应通道的ADC原始值
    int16_t offset_ib;  //B相静态电流，储存电机停止时对应通道的ADC原始值
    int16_t offset_ic;  //C相静态电流，储存电机停止时对应通道的ADC原始值
    int16_t compensate_ia; //A相静态补偿电流，补偿硬件上的误差
    int16_t compensate_ib; //B相静态补偿电流，补偿硬件上的误差
    int16_t compensate_ic; //C相静态补偿电流，补偿硬件上的误差
    int16_t offset_angle;   //偏置角度，Q15格式，范围：-32768~32767
    uint32_t stall_time; //堵转计时
    uint32_t static_time; //电机静止计时，判断电机速度是不是为0
    uint16_t openLoop_fre; //开环下角度自增频率
    dev_encoder *_encoder;  //指向电机使用的编码器结构体变量
    TIM_HandleTypeDef *_timer; //指向电机使用的定时器结构体变量
    Lpf15 *_speed_lpf;          //指向电机使用的速度滤波器器结构体变量
    Lpf15 *_iq_lpf;             //指向电机使用的Q轴电流滤波器器结构体变量
    Lpf15 *_id_lpf;             //指向电机使用的D轴电流滤波器器结构体变量
    PID_Q15_TypeDef *_speed_pid;                    // 速度PID句柄
	PID_Q15_TypeDef *_iq_pid;                    //电流PID句柄 
	PID_Q15_TypeDef *_id_pid;					//电流PID句柄 
	PID_Q15_TypeDef *_pos_pid;                    // 位置PID句柄
	TrapPlanQ15 *_trap_planner;           // 梯形加速器句柄
	Lpf *_speed_ramp_lpff;		//速度模式下用来限制目标速度变化速率，防止Uq突变太快 
}dev_motor;

/****** 电机1 ******/
extern dev_motor motor1; //电机1结构体变量

/* 用户调用函数 */
void BldcInit(dev_motor *pmotor); //电机初始化
void MotorHandler(dev_motor *pmotor); //电机高频控制入口（中断调用）
void MotorEnable(dev_motor *pmotor, uint8_t en); //电机使能/失能
void MotorSetMode(dev_motor *pmotor, Motor_mode mode); //设置控制模式
void MotorSetSpeed(dev_motor *pmotor, int32_t target_speed); //设置目标速度
void MotorSetAngle(dev_motor *pmotor, int32_t target_angle); //设置目标角度

/* 私有函数 */
void SetPhaseVol(dev_motor *pmotor); //计算并输出三相电压
int8_t ResetAngle(dev_motor *pmotor, uint32_t interval_ms); //电角度零位校准
int32_t GetMotorCir(dev_motor *pmotor); //获取旋转圈数
int32_t GetMotorPos(dev_motor *pmotor); //获取电角度位置
int32_t GetMotorSpeed(dev_motor *pmotor, uint32_t interval_us); //获取电机转速
void UpdateMotorSta(dev_motor *pmotor, uint32_t interval_us); //更新电机状态
int8_t GetOffsetCurrent(dev_motor *pmotor); //静态电流零偏校准
void GetPhaseCurrent(dev_motor *pmotor); //计算三相电流
void GetIqId(dev_motor *pmotor); //计算 q/d 轴电流
int8_t GetMotorDir(dev_motor *pmotor); //获取转动方向
void ClearEncoder(dev_motor *pmotor); //清零编码器
int8_t StallCheck(dev_motor *pmotor); //堵转检测
int32_t NormalizeAngle(int32_t angle); //角度归一化
int32_t GetElectricity(int32_t angle, uint8_t pole); //计算电角度

#endif


