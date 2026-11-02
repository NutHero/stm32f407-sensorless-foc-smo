#include "clarke_park.h"

#define SQRT3_DIV3  18919 //√3/3，q15格式，0.577 x 32768 = 18919
#define SQRT3_DIV2  28378 //√3/2，q15格式，0.867 x 32768 = 28378

/** 
 * @brief 帕克逆变换，两相直流转两相交流，q15格式定点化实现
 * @param uq q轴电压 ，Q15格式（-32768~32767 对应 -1.0~0.99997）
 * @param ud d轴电压，Q15格式（-32768~32767 对应 -1.0~0.99997）
 * @param angle 转子电角度，Q15格式（-32768~32767 对应 -180°~180°）
 * @retval RevPark 结构体，包含变换后的α轴和β轴电压分量，Q15格式
 */
RevPark RevPark_Transform15(int16_t uq, int16_t ud, int16_t angle)
{
    RevPark temp;
    Trig_Components trig;
    trig = MCM_Trig_Functions(angle); //查表获取sin,cos值
    //printf("%d\n", angle);
    //printf("%d,%d\n", trig.hSin, trig.hCos);
    if(ud == 0)
    {
        temp.ualpha = -((int32_t)uq * trig.hSin) >> 15;
        temp.ubeta = ((int32_t)uq * trig.hCos) >> 15;    
    }
    else
    {
        temp.ualpha = (-((int32_t)uq * trig.hSin) >> 15) + (((int32_t)ud * trig.hCos) >> 15);
        temp.ubeta = (((int32_t)uq * trig.hCos) >> 15) + (((int32_t)ud * trig.hSin) >> 15); 
    }        
    return temp;
}

/**
 * @brief 帕克变换，两相交流转两相直流，q15格式定点化实现
 * @param ialpha α轴电流分量，Q15格式（-32768~32767 对应 -1.0~0.99997）
 * @param ibeta β轴电流分量，Q15格式（-32768~32767 对应 -1.0~0.99997）
 * @param angle 转子电角度，Q15格式（0~32767 对应 0°~180°，-32768~-1 对应 -180°~0°）
 * @return Park 结构体，包含变换后的d轴和q轴电流分量，Q15格式
 */
Park Park_Transform15(int16_t ialpha, int16_t ibeta, int16_t angle)
{
    Park temp;
    Trig_Components trig;
    trig = MCM_Trig_Functions(angle); //查表获取sin,cos值
    temp.iq = -(((int32_t)ialpha * trig.hSin) >> 15) + (((int32_t)ibeta * trig.hCos) >> 15);
    temp.id = (((int32_t)ialpha * trig.hCos) >> 15) + ((ibeta * trig.hSin) >> 15);
    return temp;
}
/**
 * @brief 克拉克变换，三相交流转两相交流，q15格式定点化实现
 * @param ia A相电流分量，Q15格式（-32768~32767 对应 -1.0~0.99997）
 * @param ib B相电流分量，Q15格式（-32768~32767 对应 -1.0~0.99997）
 *          注意：C相电流通过 ic = -ia - ib 约束得出，无需显式输入
 * @return Clarke 结构体，包含变换后的α轴和β轴电流分量，Q15格式
 */
Clarke Clarke_Transform15(int16_t ia, int16_t ib)
{
    Clarke temp;
    temp.ialpha = ia;
    temp.ibeta = (((int32_t)SQRT3_DIV3 * ia) >> 15) + (((int32_t)SQRT3_DIV3 * 2 * ib) >> 15); 
    //SQRT3_DIV3为14位，乘以2为15位，最终结果还是在31位以内，不需要强制转为int64类型
    return temp;
}
/**
 * @brief 克拉克逆变换，两相交流转三相交流，q15格式定点化实现
 * @param ualpha α轴电压分量，Q15格式（-32768~32767 对应 -1.0~0.99997）
 * @param ubeta β轴电压分量，Q15格式（-32768~32767 对应 -1.0~0.99997）
 * @return RevClarke 结构体，包含逆变换后的三相电压分量，Q15格式
 */
RevClarke RevClarke_Transform15(int16_t ualpha, int16_t ubeta)
{
    RevClarke temp;
    temp.ua = ualpha;
    temp.ub = (((int32_t)SQRT3_DIV2 * ubeta) >> 15) - (ualpha >> 1);
    temp.uc = -(ualpha >> 1) - (((int32_t)SQRT3_DIV2 * ubeta) >> 15);
    return temp;
}

/**********************************************浮点数实现*************************************************/
#define SQRT3  1.732f //√3
/** 
 * @brief 帕克逆变换，两相直流转两相交流
 * @param uq q轴电压
 * @param ud d轴电压
 * @param angle 转子电角度，弧度值
 * @retval RevPark 结构体，包含变换后的α轴和β轴电压分量
 */
RevParkf RevPark_Transform(float uq, float ud, float angle)
{
    RevParkf temp;
    if(ud == 0)
    {        
        temp.ualpha = -uq * sin(angle);
        temp.ubeta = uq * cos(angle);
    }
    else
    {
        temp.ualpha = ud * cos(angle) - uq * sin(angle);
        temp.ubeta = uq * cos(angle) + ud * sin(angle);
    }
    return temp;
}

/**
 * @brief 帕克变换，两相交流转两相直流
 * @param ialpha α轴电流分量
 * @param ibeta β轴电流分量
 * @param angle 转子电角度，弧度值
 * @return Park 结构体，包含变换后的d轴和q轴电流分量
 */
Parkf Park_Transform(float ialpha, float ibeta, float angle)
{
    Parkf temp;
    temp.iq = -ialpha * sin(angle) + ibeta * cos(angle);
    temp.id = ialpha * cos(angle) + ibeta * sin(angle);
    return temp;
}
/**
 * @brief 克拉克变换，三相交流转两相交流
 * @param ia A相电流分量
 * @param ib B相电流分量
 *          注意：C相电流通过 ic = -ia - ib 约束得出，无需显式输入
 * @return Clarke 结构体，包含变换后的α轴和β轴电流分量
 */
Clarkef Clarke_Transform(float ia, float ib)
{
    Clarkef temp;
    temp.ialpha = ia;
    temp.ibeta = (ia + 2 * ib) / SQRT3;
    return temp;
}
/**
 * @brief 克拉克逆变换，两相交流转三相交流
 * @param ualpha α轴电压分量
 * @param ubeta β轴电压分量
 * @return RevClarke 结构体，包含逆变换后的三相电压分量
 */
RevClarkef RevClarke_Transform(float ualpha, float ubeta)
{
    RevClarkef temp;
    temp.ua = ualpha;
    temp.ub = (SQRT3 * ubeta - ualpha) / 2;
    temp.uc = (-ualpha - SQRT3 * ubeta) / 2;
    return temp;
}
