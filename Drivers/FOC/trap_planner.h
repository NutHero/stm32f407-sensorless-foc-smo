#ifndef __TRAP_PLANNER_H
#define __TRAP_PLANNER_H


#include "./SYSTEM/sys/sys.h"
#include <math.h>

//// 梯形加减速规划器
//typedef struct {
//    // 参数设置
//    float max_vel;      // 最大速度
//    float max_acc;      // 最大加速度
//    float now_max_vel;	// 当前最大速度
//	
//    // 状态变量
//    float start_pos;    // 起始位置
//    float target_pos;   // 目标位置
//    float current_pos;  // 当前位置
//    float current_vel;  // 当前速度
//    
//    // 时间计算
//    float acc_time;     // 加速段时间
//    float const_time;   // 匀速段时间
//    float dec_time;     // 减速段时间
//    
//    // 运行状态
//    float elapsed_time; // 已运行时间
//    uint8_t is_finished; // 完成标志
//} TrapPlan;
//extern TrapPlan planner1; 

//// 函数声明
//void trap_init(TrapPlan *plan, float max_vel, float max_acc);
//void trap_set_target(TrapPlan *plan, float target);
//void trap_update(TrapPlan *plan, float dt);
//float trap_get_pos(TrapPlan *plan);
//float trap_get_vel(TrapPlan *plan);
//uint8_t trap_is_done(TrapPlan *plan);

/******************************************** 定点数实现 ************************************************/
// 梯形加减速规划器（完全Q15格式）
typedef struct {
    // 参数设置
    int32_t max_vel_q;      // 最大速度 (Q15)
    int32_t max_acc_q;      // 最大加速度 (Q15)
    int32_t now_max_vel_q;  // 当前最大速度 (Q15)
    
    // 状态变量
    int32_t start_pos_q;    // 起始位置 (Q15)
    int32_t target_pos_q;   // 目标位置 (Q15)
    int32_t current_pos_q;  // 当前位置 (Q15)
    int32_t current_vel_q;  // 当前速度 (Q15)
    
    // 时间计算
    int32_t acc_time_q;     // 加速段时间 (Q15)
    int32_t const_time_q;   // 匀速段时间 (Q15)
    int32_t dec_time_q;     // 减速段时间 (Q15)
    
    // 运行状态
    int32_t elapsed_time_q; // 已运行时间 (Q15)
    uint8_t is_finished;    // 完成标志
} TrapPlanQ15;
extern TrapPlanQ15 m1_planner; 

// 函数声明
void trap_init_q15(TrapPlanQ15 *plan, int32_t max_vel, int32_t max_acc);
void trap_set_target_q15(TrapPlanQ15 *plan, int32_t target);
void trap_update_q15(TrapPlanQ15 *plan, int32_t dt);
int32_t trap_get_pos_q15(TrapPlanQ15 *plan);
int32_t trap_get_vel_q15(TrapPlanQ15 *plan);
uint8_t trap_is_done_q15(TrapPlanQ15 *plan);

#endif
