#include "quen.h"

RingQuen ringquen1,ringquen2,ringquen3; //队列

/**
 * @brief 环形队列滑动滤波器
 * @param data 新采集的数据
 * @param pquen 指向队列结构体变量
 * @return 滤波后的平均值
 */
uint16_t ring_quen_filter(uint16_t data, RingQuen* pquen)
{
    if(pquen->count < QUEN_SIZE)  // 如果是前QUEN_SIZE个数据，队列未满
    {    
        pquen->buffer[pquen->count] = data; // 直接添加到队列
        pquen->sum += data; // 累加和   
        pquen->count++; // 更新计数
    } 
    else  // 队列已满，进行环形替换
    {
        pquen->sum -= pquen->buffer[pquen->idx];  // 从累加和中减去要替换的旧值
        pquen->buffer[pquen->idx] = data; // 替换为新值
        pquen->sum += data; // 累加新值
        pquen->idx = (pquen->idx + 1) % QUEN_SIZE; // 更新环形索引
    }
    return (pquen->count > 0) ? (pquen->sum / pquen->count) : 0; // 计算平均值
}
