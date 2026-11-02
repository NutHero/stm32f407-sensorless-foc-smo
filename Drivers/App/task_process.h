#ifndef __TASK_PROCESS
#define __TASK_PROCESS

#include "./SYSTEM/sys/sys.h"
#include "filter.h"
#include "bldc_control.h"
#include "sincos16.h"
#include "time.h"

#define BUS_CURRENT_LIMIT   2400    //母线电流限流，单位：mA

/* 进程状态枚举 */
typedef enum{
    PROC_GET_OFFSET = 0,       //计算电机静态电流
    PROC_RESET_ANGLE = 1,             //电机角度校准
    PROC_OPEN_MOTOR = 2,              //开启电机
    PROC_CLOSE_MOTOR = 3,             //关闭电机
    PROC_WAIT_CMD = 4,                //等待指令
    PROC_GET_ANGLE = 5,               //得到电机轴位置
    PROC_STORE_ANGLE = 6,             //储存电机轴位置
    PROC_GET_BUS_CUR = 7,          //得到母线电流
	PROC_GET_BUS_VOL = 9,			//得到母线电压
	PROC_GET_TEMP = 10,     //得到功率板温度
    PROC_ERROR = 8,                   //错误
}SysProcess;

/* 系统状态结构体 */
typedef struct
{
	SysProcess current_process; //当前进程
	uint32_t tick;	//程序计时
}sys_sta_;

extern sys_sta_ sys_sta;

void sys_process_run(dev_motor * pmotor); //运行任务管理
void task_scheduler_start(void); //开启系统任务调度
void task_scheduler_stop(void); //关闭系统任务调度 

#endif
