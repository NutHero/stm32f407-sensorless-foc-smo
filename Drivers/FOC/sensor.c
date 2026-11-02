#include "sensor.h"

#define PI 3.1415926 //圆周率π
#define PI_2 6.28318530f   //两倍圆周率 

#define AS5600_ADD          0x36  //AS5600地址，最后一位是读写位，读(1), 写(0) 
#define AS5600_DATA_HADD    0x0C  //数据寄存器高4位地址
#define AS5600_DATA_LADD    0x0D  //数据寄存器低8位地址

#define MT6701_ADD          0x0C  //MT6701地址,原地址0x06
#define MT6701_DATA_HADD    0x03  //高位数据寄存器
#define MT6701_DATA_LADD    0x04  //低位数据寄存器

dev_encoder m1_encoder; //编码器1结构体变量

//编码器初始化
void encode_initial(void)
{
	m1_encoder.offset_value = 0; //13191
}

/**
 * @brief 从AS5600读一个字节
 * @param reg 要读取的寄存器地址
 * @retval 读取的字节
 */
uint8_t as5600_read_byte(uint8_t reg)
{
    uint8_t temp;
    __disable_irq();
    iic_start(); /* 第一个开始信号为写数据 */
    iic_send_byte((AS5600_ADD << 1) | 0x00); /* 发送写命令 */
    iic_wait_ack();
    iic_send_byte(reg); /* 选择数据寄存器地址 */
    iic_wait_ack();
    iic_start(); /* 第二个开始信号为读数据, 注：IIC协议中，写数据和读数据需要单独的开始信号 */
    iic_send_byte((AS5600_ADD << 1) | 0x01); /* 发送读命令 */
    iic_wait_ack();
    temp = iic_read_byte(0); /* 读数据寄存器值 */
    iic_stop();
    __enable_irq();
    return temp;
}

/**
 * @brief 从AS5600读取两个字节
 * @param h_add 高位寄存器地址
 * @param l_add 低位寄存器地址
 * @retval 读取的两个字节
 */
uint16_t as5600_read_2byte(uint8_t h_add, uint8_t l_add)
{
    uint16_t temp1;
    uint8_t temp2;
    temp1 = as5600_read_byte(h_add) & 0x000f;
    temp2 = as5600_read_byte(l_add);
    return (temp1 << 8) | temp2;
}

/**
 * @brief 从AS5600读取角度数据
 * @retval 读取的数据
 */
uint16_t as5600_read_angle(void)
{
    uint16_t angle;
    angle = as5600_read_2byte(AS5600_DATA_HADD, AS5600_DATA_LADD);
    return angle;
}

/**
 * @brief MT6701初始化
 * @retval 
 */
uint16_t MT6701_init(void)
{
//    GPIO_InitTypeDef gpio_init_struct; 
//    MT6701_GPIO_CLK_ENABLE(); //时钟使能
//    gpio_init_struct.Pin = MT6701_GPIO_PIN; //引脚
//    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP; //推挽输出
//    gpio_init_struct.Pull = GPIO_PULLUP;  //上拉
//    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH; //高速
//    HAL_GPIO_Init(MT6701_GPIO_PORT, &gpio_init_struct); //初始化引脚
}    

/**
 * @brief 从MT6701读取数据
 * @retval 读取的数据
 */
uint16_t MT6701_GetRawData(void)
{
	uint16_t txData = 0xFFFF;
	uint16_t timeout = 200;
    uint16_t dat;
	while(HAL_SPI_GetState(&g_spi3_handler) != HAL_SPI_STATE_READY)
    {
		if(timeout-- == 0)return 1;//超时直接返回，避免执行后面代码
        __NOP();__NOP();__NOP();__NOP();
    }
	MT6701_CS(0);	
    SPI3->DR = txData;
    timeout = 200;
    while((SPI3->SR & SPI_SR_TXE) == 0)
    {
        if(timeout-- == 0)return 1;
        __NOP();__NOP();__NOP();__NOP();
    };	
    timeout = 200;
    while((SPI3->SR & SPI_SR_RXNE) == 0) //等待接收缓冲区为空
    {
        if(timeout-- == 0)return 1;
        __NOP();__NOP();__NOP();__NOP();
    };
    dat = SPI3->DR;
	MT6701_CS(1);
	return 16384 - (dat >> 2); //右移动两位，取出高位置14位角度原始数据
}

/**
 * @brief 角度值转换为弧度值
 * @note 弧度值 = 角度 * PI / 180
         角度 = 弧度 * 180 / PI
 * @param angle 输入角度
 * @retval 弧度值
 */
float angle_to_radian_(float angle)
{
    return (angle * PI) / 180;
}

/**
 * @brief 弧度值转换为角度值
 * @note 弧度值 = 角度 * PI / 180
         角度 = 弧度 * 180 / PI
 * @param angle 输入弧度
 * @retval 角度值
 */
float radian_to_angle(float radian)
{
    return (radian * 180) / PI;
}

/**
 * @brief 更新编码器状态
 * @note 更新编码器计数值，圈数，方向，总计数值
 * @param pencoder 指向编码器结构体变量
 */
void updat_encoder_sta(dev_encoder *pencoder)
{
    int32_t raw_diff,ang;
    const int32_t factor1 = ENCODER_OVER_THRESHOLD;
    ang = (int32_t)MT6701_GetRawData();
    pencoder->value = ang - pencoder->offset_value;
    if(pencoder->value < 0)
    {
        pencoder->value = pencoder->value + ENCODER_FACTOR;
    }
    else if(pencoder->value > ENCODER_FACTOR)
    {
        pencoder->value = pencoder->value - ENCODER_FACTOR;
    }
    raw_diff = pencoder->value - pencoder->pre_value;
//    printf("%d,%d\n", raw_diff, factor1);
    if(raw_diff < -factor1)
    {
        pencoder->pulse_diff = ENCODER_FACTOR + raw_diff;
        pencoder->over_pre_cnt = pencoder->over_cnt;
        pencoder->over_cnt += 1;
    }
    else if(raw_diff > factor1)
    {
        pencoder->pulse_diff = raw_diff - ENCODER_FACTOR;
        pencoder->over_pre_cnt = pencoder->over_cnt;
        pencoder->over_cnt -= 1;
    }
    else
    {    
        pencoder->pulse_diff = raw_diff;
    }
    if(pencoder->pulse_diff > 0)
    {
        pencoder->dir = 1;
    }
    else if(pencoder->pulse_diff < 0)
    {
        pencoder->dir = -1;
    }
    else
    {
        pencoder->dir = 0;
    }
    if(pencoder->pre_value == pencoder->value)
    {
        pencoder->change_flag = 0;
    }
    else
    {
        pencoder->change_flag = 1;
    }
    pencoder->total_pulse += pencoder->pulse_diff;
    pencoder->pre_value = pencoder->value;
}
