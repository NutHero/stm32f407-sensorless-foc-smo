#include "trap_planner.h"

//TrapPlan planner1; 

///**
// * @brief 初始化梯形规划器
// * @plan 指向梯形规划器结构体变量
// * @maxvel 最大速度
// * @maxacc 最大加速度
// */
//void trap_init(TrapPlan *plan, float max_vel, float max_acc) 
//{
//    plan->max_vel = max_vel;
//    plan->max_acc = max_acc;
//    plan->is_finished = 1;
//}

///**
// * @brief 梯形加速器规划电机位移曲线
// * @plan 指向梯形规划器结构体变量
// * @target 目标位置
// */
//void trap_set_target(TrapPlan *plan, float target)
//{
//    float distance,vel_reach_time,acc_distance,total_acc_distance,max_reachable_vel;
//    int8_t dir;
//    // 记录起始位置
//    plan->start_pos = plan->current_pos;
//    plan->target_pos = target;
//    plan->elapsed_time = 0;
//    plan->is_finished = 0;
//    
//    // 计算需要移动的距离
//    distance = fabsf(target - plan->start_pos);
//    dir = (target >= plan->start_pos) ? 1 : -1;
//    
//    // 计算加速到最大速度需要的距离
//    vel_reach_time = plan->max_vel / plan->max_acc;//v = a * t -> t = v / a
//    acc_distance = 0.5f * plan->max_acc * vel_reach_time * vel_reach_time;//s = 1/2 * a * t^2
//    total_acc_distance = 2 * acc_distance; // 加速+减速总距离
//    
//    if (distance >= total_acc_distance) 
//    {
//        // 距离足够，能加速到最大速度（梯形）
//        plan->acc_time = vel_reach_time;
//        plan->dec_time = vel_reach_time;
//				plan->now_max_vel = plan->max_vel;
//        plan->const_time = (distance - total_acc_distance) / plan->max_vel;
//    }   
//    else 
//    {
//        // 距离不够，达不到最大速度（三角形）
//        max_reachable_vel = sqrtf(distance * plan->max_acc); //Vmax = √(d * amax)
//        plan->now_max_vel = max_reachable_vel; //更新最大速度
//        plan->acc_time = max_reachable_vel / plan->max_acc;
//        plan->dec_time = plan->acc_time;
//        plan->const_time = 0;
//    }
//}

///**
// * @brief 更新规划的期望位置和期望速度，周期调用
// * @plan 指向梯形规划器结构体变量
// * @dt 调用间隔
// */
//void trap_update(TrapPlan *plan, float dt) 
//{
//    float vel = 0,pos = 0, t,t_const,t_dec;
//    int8_t dir;
//    if (plan->is_finished) return;
//    
//    // 累计运行时间
//    plan->elapsed_time += dt;
//    t = plan->elapsed_time;
//    
//    // 计算方向
//    dir = (plan->target_pos >= plan->start_pos) ? 1 : -1;
//    
//    if (t < plan->acc_time) 
//    {
//        // 加速阶段
//        vel = plan->max_acc * t;
//        pos = 0.5f * plan->max_acc * t * t; // s = 1/2 * a * t^2
//    }
//    else if (t < plan->acc_time + plan->const_time) 
//    {
//        // 匀速阶段
//        t_const = t - plan->acc_time; 
//        vel = plan->now_max_vel;
//        pos = 0.5f * plan->max_acc * plan->acc_time * plan->acc_time
//              + plan->now_max_vel * t_const;// s = 1/2 * a *ta^2 + vmax * tc;
//    }
//    else if (t < plan->acc_time + plan->const_time + plan->dec_time) 
//    {
//        // 减速阶段
//        t_dec = t - plan->acc_time - plan->const_time;
//        vel = plan->now_max_vel - plan->max_acc * t_dec;
//        pos = 0.5f * plan->max_acc * plan->acc_time * plan->acc_time
//              + plan->now_max_vel * plan->const_time
//              + plan->now_max_vel * t_dec - 0.5f * plan->max_acc * t_dec * t_dec;//s = 1/2 * vmax * ta^2 + vmax * tc + vmax * td - 1/2 * vmax * t^2;
//    }
//    else 
//    {
//        // 运动完成
//        vel = 0;
//        pos = 0.5f * plan->max_acc * plan->acc_time * plan->acc_time
//              + plan->now_max_vel * plan->const_time
//              + plan->now_max_vel * plan->dec_time - 0.5f * plan->max_acc * plan->dec_time * plan->dec_time;
//        plan->is_finished = 1;
//    }
//    
//    // 考虑方向
//    plan->current_vel = vel * dir;
//    plan->current_pos = plan->start_pos + pos * dir;
//}

//// 获取当前位置指令
//float trap_get_pos(TrapPlan *plan) 
//{
//    return plan->current_pos;
//}

//// 获取当前速度指令
//float trap_get_vel(TrapPlan *plan) 
//{
//    return plan->current_vel;
//}

//// 检查是否完成
//uint8_t trap_is_done(TrapPlan *plan) 
//{
//    return plan->is_finished;
//}

/****************************************** 定点数实现 **************************************************/
TrapPlanQ15 m1_planner; 

#define Q_MUL(a, b) ((int32_t)(((int64_t)(a) * (b)) >> 15)) //Q格式数相乘
#define Q_DIV(a, b) ((int32_t)(((int64_t)(a) << 15) / (b))) //Q格式数相除

/**
 * @brief 定点开方函数
 * @param x_q 输入
 * @return 结果
 */
int32_t Q_sqrt(int32_t x_q)
{
    if (x_q <= 0) return 0;
    
    int32_t x = x_q;
    int32_t y = 0;
    int32_t m = 0x40000000;
    
    while (m != 0)
    {
        int32_t b = y | m;
        y >>= 1;
        
        if (x >= b)
        {
            x -= b;
            y |= m;
        }
        m >>= 2;
    }
    return y;
}

/**
 * @brief 初始化梯形规划器
 * @param plan 梯形规划器结构体句斌指针
 * @param max_vel_q 最大速度设置
 * @param max_acc_q 最大加速度设置
 */
void trap_init_q15(TrapPlanQ15 *plan, int32_t max_vel_q, int32_t max_acc_q) 
{
    plan->max_vel_q = max_vel_q;
    plan->max_acc_q = max_acc_q;
    plan->is_finished = 1;
}

/**
 * @brief 梯形加速器规划电机位移曲线
 * @param plan 梯形规划器结构体句斌指针
 * @param target_q 目标位置
 */
void trap_set_target_q15(TrapPlanQ15 *plan, int32_t target_q)
{
    int32_t distance_q, vel_reach_time_q, acc_distance_q;
    int32_t total_acc_distance_q, max_reachable_vel_q;
    int8_t dir;
    
    // 记录起始位置
    plan->start_pos_q = plan->current_pos_q;
    plan->target_pos_q = target_q;
    plan->elapsed_time_q = 0;
    plan->is_finished = 0;
    
    // 计算需要移动的距离
    distance_q = target_q - plan->start_pos_q;
    if (distance_q < 0)
    {
        dir = -1;
        distance_q = -distance_q;
    }
    else
    {
        dir = 1;
    }
    
    // 计算加速到最大速度需要的距离和时间
    // t = v / a
    vel_reach_time_q = Q_DIV(plan->max_vel_q, plan->max_acc_q);
    
    // s = 0.5 * a * t^2
    acc_distance_q = Q_MUL(plan->max_acc_q, Q_MUL(vel_reach_time_q, vel_reach_time_q)) >> 1;  
    
    // 加速+减速总距离
    total_acc_distance_q = acc_distance_q << 1;
    
    if (distance_q >= total_acc_distance_q)
    {
        // 距离足够，能加速到最大速度（梯形）
        plan->acc_time_q = vel_reach_time_q;
        plan->dec_time_q = vel_reach_time_q;
        plan->now_max_vel_q = plan->max_vel_q;
        // 匀速段时间 = (总距离 - 加减速距离) / 最大速度
        plan->const_time_q = Q_DIV((distance_q - total_acc_distance_q), plan->max_vel_q);
    }
    else
    {
        // 距离不够，达不到最大速度（三角形）
        // Vmax = √(d * a)
        max_reachable_vel_q = Q_sqrt(distance_q * plan->max_acc_q);
        plan->now_max_vel_q = max_reachable_vel_q;
        plan->acc_time_q = Q_DIV(max_reachable_vel_q, plan->max_acc_q);
        plan->dec_time_q = plan->acc_time_q;
        plan->const_time_q = 0;
    }
}

/**
 * @brief 更新梯形规划器
 * @note 周期调用
 * @param plan 梯形规划器结构体句斌指针
 * @param dt_q 调用间隔
 */
void trap_update_q15(TrapPlanQ15 *plan, int32_t dt_q)
{
    int32_t v, p, t, td;
    int8_t dir;
    
    if (plan->is_finished) return;
    
    // 累计时间
    plan->elapsed_time_q += dt_q;
    t = plan->elapsed_time_q;
    dir = (plan->target_pos_q >= plan->start_pos_q) ? 1 : -1;
    
    if (t < plan->acc_time_q)
    {
        // 加速阶段: v = a*t, p = 0.5*a*t2
        v = Q_MUL(plan->max_acc_q, t);
        p = Q_MUL(plan->max_acc_q, Q_MUL(t, t)) >> 1;
    }
    else if (t < plan->acc_time_q + plan->const_time_q)
    {
        // 匀速阶段
        v = plan->now_max_vel_q;
        p = (Q_MUL(plan->max_acc_q, Q_MUL(plan->acc_time_q, plan->acc_time_q)) >> 1) +
             Q_MUL(plan->now_max_vel_q, t - plan->acc_time_q);
    }
    else if (t < plan->acc_time_q + plan->const_time_q + plan->dec_time_q)
    {
        // 减速阶段
        td = t - plan->acc_time_q - plan->const_time_q;  // 减速段时间
        
        // 速度: v = vmax - a*td
        v = plan->now_max_vel_q - Q_MUL(plan->max_acc_q, td);
        
        // 位置: 加速段 + 匀速段 + 减速段
        p = (Q_MUL(plan->max_acc_q, Q_MUL(plan->acc_time_q, plan->acc_time_q)) >> 1) +  // 加速段
             Q_MUL(plan->now_max_vel_q, plan->const_time_q) +                          // 匀速段
             Q_MUL(plan->now_max_vel_q, td) -                                          // 减速段速度部分
            (Q_MUL(plan->max_acc_q, Q_MUL(td, td)) >> 1);                             // 减速段加速度部分
    }
    else
    {
        // 运动完成
        v = 0;
        p = (Q_MUL(plan->max_acc_q, Q_MUL(plan->acc_time_q, plan->acc_time_q)) >> 1) +  // 加速段
             Q_MUL(plan->now_max_vel_q, plan->const_time_q) +                          // 匀速段
             Q_MUL(plan->now_max_vel_q, plan->dec_time_q) -                           // 减速段速度部分
            (Q_MUL(plan->max_acc_q, Q_MUL(plan->dec_time_q, plan->dec_time_q)) >> 1); // 减速段加速度部分
        plan->is_finished = 1;
    }
    
    // 应用方向
    plan->current_vel_q = (dir > 0) ? v : -v;
    plan->current_pos_q = plan->start_pos_q + ((dir > 0) ? p : -p);
}

// 获取当前位置指令
int32_t trap_get_pos_q15(TrapPlanQ15 *plan)
{
    return plan->current_pos_q;
}

// 获取当前速度指令
int32_t trap_get_vel_q15(TrapPlanQ15 *plan)
{
    return plan->current_vel_q;
}

// 检查是否完成
uint8_t trap_is_done_q15(TrapPlanQ15 *plan)
{
    return plan->is_finished;
}
