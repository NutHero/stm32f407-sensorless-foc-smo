#ifndef __QUEN_H
#define __QUEN_H

#include "./SYSTEM/sys/sys.h"

#define QUEN_SIZE   40  // 队列长度

/* 环形队列结构体定义 */
typedef struct{
    uint16_t buffer[QUEN_SIZE]; // 储存的数据
    uint8_t idx; // 当前索引
    uint32_t sum; // 累加和
    uint8_t count; // 有效数据计数
}RingQuen;
extern RingQuen ringquen1,ringquen2,ringquen3; //队列

uint16_t ring_quen_filter(uint16_t data, RingQuen* pquen); //环形队列滑动滤波器

#endif
