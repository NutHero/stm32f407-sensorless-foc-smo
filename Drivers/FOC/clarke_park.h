#ifndef __CLARK_PARK_H
#define __CLARK_PARK_H

#include "./SYSTEM/sys/sys.h"
#include "math.h"
#include "sincos16.h"

/*α-β坐标系电压分量（逆帕克变换输出） */
typedef struct{
    int16_t ualpha; /* α轴电压，Q15格式 */
    int16_t ubeta;  /* β轴电压，Q15格式 */
}RevPark;

/* d-q坐标系电流分量（帕克变换输出） */
typedef struct{
    int16_t iq; /* q轴电流（转矩分量），Q15格式 */
    int16_t id; /* d轴电流（励磁分量），Q15格式 */
}Park;

/* α-β坐标系电流分量（克拉克变换输出） */
typedef struct{
    int16_t ialpha; /* α轴电流，Q15格式 */
    int16_t ibeta; /* β轴电流，Q15格式 */
}Clarke;

/* 三相坐标系电压分量（逆克拉克变换输出） */
typedef struct{
    int16_t ua; /* A相电压，Q15格式 */
    int16_t ub; /* B相电压，Q15格式 */
    int16_t uc; /* C相电压，Q15格式 */
}RevClarke;

RevPark RevPark_Transform15(int16_t uq, int16_t ud, int16_t angle); //帕克变换
Park Park_Transform15(int16_t ialpha, int16_t ibeta, int16_t angle); //帕克逆变换
Clarke Clarke_Transform15(int16_t ia, int16_t ib); //克拉克变换
RevClarke RevClarke_Transform15(int16_t ualpha, int16_t ubeta); //克拉克逆变换

/**********************************************浮点数实现*************************************************/
/*α-β坐标系电压分量（逆帕克变换输出） */
typedef struct{
    float ualpha; /* α轴电压，Q15格式 */
    float ubeta;  /* β轴电压，Q15格式 */
}RevParkf;

/* d-q坐标系电流分量（帕克变换输出） */
typedef struct{
    float iq; /* q轴电流（转矩分量） */
    float id; /* d轴电流（励磁分量） */
}Parkf;

/* α-β坐标系电流分量（克拉克变换输出） */
typedef struct{
    float ialpha; /* α轴电流 */
    float ibeta; /* β轴电流 */
}Clarkef;

/* 三相坐标系电压分量（逆克拉克变换输出） */
typedef struct{
    float ua; /* A相电压 */
    float ub; /* B相电压 */
    float uc; /* C相电压 */
}RevClarkef;

RevParkf RevPark_Transform(float uq, float ud, float angle); //帕克变换
Parkf Park_Transform(float ialpha, float ibeta, float angle); //帕克逆变换
Clarkef Clarke_Transform(float ia, float ib); //克拉克变换
RevClarkef RevClarke_Transform(float ualpha, float ubeta); //克拉克逆变换

#endif
