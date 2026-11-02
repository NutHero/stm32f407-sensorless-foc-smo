#ifndef __SENSOR_H
#define __SENSOR_H

#include "./SYSTEM/sys/sys.h"
#include "myiic.h"
#include "spi.h"

/* MT6701片选引脚定义 */
#define MT6701_GPIO_PORT     GPIOC
#define MT6701_GPIO_PIN      GPIO_PIN_9
#define MT6701_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PC口时钟使能 */
/* MT6701 CS操作 */
#define MT6701_CS(x)        do{ x ? \
                              HAL_GPIO_WritePin(MT6701_GPIO_PORT, MT6701_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(MT6701_GPIO_PORT, MT6701_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)  

///* 编码器参数(增量式) */
//#define ENCODER_LINE_NUM  1024.0f   //编码器线数
//#define REDU_RATIO  21.0f           //电机减速比
//#define QUADRA_FACTOR   4.0f        //定时器采样倍频
     
/* 编码器参数(绝对式) */                          
#define ENCODER_BIT_NUM     14      //编码器位数
#define ENCODER_FACTOR  ((uint32_t)2 << ((uint32_t)ENCODER_BIT_NUM - 1)) //编码器单圈计数值
#define ENCODER_OVER_THRESHOLD  ((uint32_t)(ENCODER_FACTOR * 0.8f))    //编码器溢出判定阈值，最大值的80%
#define ENCODER_RATIO   ((float)360 / ENCODER_FACTOR)     //编码器分辨率
    
/* 编码器状态管理结构体 */
typedef struct{
    int8_t dir;                 //编码器计数方向，1顺时针，-1逆时针，0静止
    int32_t over_cnt;           //编码器溢出次数：区分正负
    int32_t over_pre_cnt;       //编码器之前溢出的次数，用于计算编码器一定时间输出的脉冲数
    int32_t value;              //编码器当前计数值
    int32_t pre_value;          //编码器计数旧值，用于计算编码器一定时间输出的脉冲数
    int32_t pulse_diff;         //编码器一定时间输出的计数值，区分方向
    int32_t total_pulse;        //编码器总共输出的计数值，区分方向
    uint16_t offset_value;       //编码器零位偏差，设置为0时电机自动校准，否则跳过校准步骤
    uint8_t change_flag;        //标记编码器计数值是否发生改变，1发生改变；0没有
}dev_encoder;
extern dev_encoder m1_encoder;

void encode_initial(void);
uint8_t as5600_read_byte(uint8_t reg); //从AS5600读一个字节
uint16_t as5600_read_2byte(uint8_t h_add, uint8_t l_add); //从AS5600读取两个字节
uint16_t as5600_read_angle(void); //从AS5600读取角度数据  
void updat_encoder_sta(dev_encoder *pencoder); //更新编码器状态
uint16_t MT6701_init(void); //MT6701初始化
uint16_t MT6701_GetRawData(void); //从MT6701读取数据
float angle_to_radian_(float angle); // 角度值转换为弧度值
float radian_to_angle(float radian); //弧度值转换为角度值
 
#endif
