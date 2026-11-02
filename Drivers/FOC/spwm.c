#include "spwm.h"
#include "clarke_park.h"

/**
 * @brief SPWM算法
 * @param uq q轴电压
 * @param ud d轴电压
 * @param angle 电角度，弧度值(0~2π)
 * @param pwm_period PWM定时器重装载值
 * @param ccrs 输出数组，存放三个通道的CCR值
 */
void set_spwm(float uq, float ud, float angle, uint16_t pwm_period, uint16_t ccrs[3])
{
    RevParkf rpark;
    RevClarkef rclark; 
    rpark = RevPark_Transform(uq, ud, angle); //帕克逆变换
    rclark = RevClarke_Transform(rpark.ualpha, rpark.ubeta); //克拉克逆变换
    //printf("%f,%f,%f\r\n", rclark.ua, rclark.ub, rclark.uc);
    
    // 将范围从[-1,1]映射到[0,1]
    // 映射公式：y = (x + 1) / 2
    ccrs[0] = (uint16_t)(pwm_period * (rclark.ua + 1.0f) * 0.5f);
    ccrs[1] = (uint16_t)(pwm_period * (rclark.ub + 1.0f) * 0.5f);
    ccrs[2] = (uint16_t)(pwm_period * (rclark.uc + 1.0f) * 0.5f);
}

/**
 * @brief SPWM算法，Q15格式实现
 * @param uq q轴电压，Q15格式(-32768~32767)
 * @param ud d轴电压，Q15格式(-32768~32767)
 * @param angle 电角度，Q15格式(-32768~32767)
 * @param pwm_period PWM定时器重装载值
 * @param ccrs 输出数组，存放三个通道的CCR值
 */
void set_spwm_q15(int16_t uq, int16_t ud, int16_t angle, uint16_t pwm_period, uint16_t ccrs[3])
{
    RevPark rpark;
    RevClarke rclark; 
    rpark = RevPark_Transform15(uq, ud, angle); //帕克逆变换
    rclark = RevClarke_Transform15(rpark.ualpha, rpark.ubeta); //克拉克逆变换
    
    // 将范围从[-32768,32767]映射到[0,32767]
    // 映射公式：y = (x + 32768) >> 1
    ccrs[0] = ((int32_t)pwm_period * (rclark.ua + 32768)) >> 16;
    ccrs[1] = ((int32_t)pwm_period * (rclark.ub + 32768)) >> 16;
    ccrs[2] = ((int32_t)pwm_period * (rclark.uc + 32768)) >> 16;
    //printf("%d, %d, %d\r\n", ccrs[0], ccrs[1], ccrs[2]);
}

//    float ele = 1;
//    float uq = 0.15;
//float ele1 = 0;
//float offset_ele = 0.1;
//float offset_ang;
//float k = -100;
//    float ia, ib, ic;
//    float phase_shift = 2.0f * 3.1415926535f / 3.0f;  // 120度相位差
//    uint16_t ccrs[3];
//    while(1)
//    {
//        if(ele > 6.28f)ele -= 6.28;
//        ele += 0.1;
////        set_spwm(uq, 0, ele, 2000, ccrs);
////        //printf("%d,%d,%d\r\n", ccrs[0], ccrs[1], ccrs[2]);
////        ia = ccrs[0] / 10.0f - 100;
////        ib = ccrs[1] / 10.0f - 100;
////        ic = ccrs[2] / 10.0f - 100;
//        ia =  k* sinf( ele + offset_ang);
//        ib = k* sinf( ele - phase_shift + offset_ang);
//        ic =  k* sinf(ele + phase_shift + offset_ang);
//        ele1 = ele + offset_ele;
//        if(ele1 > 6.28f)ele1 -= 6.28;
//        clarkef = Clarke_Transform(ic, ib);
//        parkf = Park_Transform(clarkef.ialpha, clarkef.ibeta, ele1);
//        printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", uq, ia, ib, ic, clarkef.ialpha, clarkef.ibeta, parkf.iq, ele,ele1);
//        HAL_Delay(1);
//    }
