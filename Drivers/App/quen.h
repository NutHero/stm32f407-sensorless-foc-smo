#ifndef __QUEN_H
#define __QUEN_H

#include "./SYSTEM/sys/sys.h"

/* 队列长度 = 电流反馈滑动平均的点数。
   注意：ADC 注入组 20kHz 触发，队列长度 N 对应 (N-1)/2 * 50us 的群延时。
   原来 N=40 就是 2ms 窗口、975us 群延时，转速一高相位滞后就爆炸：
       1500rpm(175Hz) 滞后 61 度，2000rpm(233Hz) 滞后 82 度，
   反电动势观测器看到的是近 1ms 之前电流对应的电压，角度必然跑偏，
   表现为"1500rpm 以上电流变大、2000rpm 堵转"。
   ADC 采样点固定在 PWM 谷点(低侧全开，见 time.h 的 ADC_SAMPLE_POINT)，
   开关噪声本来就小，取 4 点(200us 窗口、75us 延时)足够，剩下的高频噪声由
   dq 解调低通和速度反馈滤波处理。 */
#define QUEN_SIZE   4  // 队列长度

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
