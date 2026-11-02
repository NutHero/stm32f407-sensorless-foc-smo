#include "./BSP/IIC/iic.h"
#include "./SYSTEM/delay/delay.h"

void VL_SDA_IN_(void) 	
	{  
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
} 
void VL_SDA_OUT_(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}

/* 软件模拟I2C */
void iic_start(void)//发送开始信号
{	
	SDA(1);
	SCL(1);
    delay_us(10);
	SDA(0); 
	SCL(0);
}

void iic_stop(void)//发送停止信号
{
	SCL(1);
	SDA(0);
	delay_us(10);
	SDA(1);	
}

void iic_ack(void)//发送应答信号
{
	SCL(0);
	SDA(0);
	delay_us(10);
	SCL(1);
	delay_us(10);
	SCL(0);
}

void iic_nack(void)//发送非应答信号
{
	SCL(0);
	SDA(1);
	delay_us(10);
	SCL(1);
	delay_us(10);
	SCL(0);
}

uint8_t iic_wait_ack(void)//等待应答
{
	SCL(1);//SCL为高电平时主机才可以从从机读数据
	delay_us(10);
	if(RSDA)
	{
		return 0;//返回0代表无效应答	
	}
	SCL(0);
	return 1;//返回1代表有效应答	
}

void iic_write_byte(uint8_t dat)//写字节
{
	uint8_t temp = dat;
	for(uint8_t i = 0; i < 8; i++)
	{
		temp = dat & 0x80;//获取高位
		SCL(0);
		delay_us(10);
		if(temp > 0)
		{
			SDA(1);
		}
		else
        {
            SDA(0);
        }
		SCL(1);//拉高SCL保持数据稳定
		delay_us(10);
		SCL(0);//拉低SCL以改变SDA
		dat <<= 1;
	}
    SDA(1);
}

uint8_t iic_read_byte(void)//读字节
{
	uint8_t dat = 0;
    VL_SDA_IN_();
	for(uint8_t i = 0; i < 8; i++)
	{
		SCL(0);
		delay_us(10);
		SCL(1);
		dat <<= 1;
		if(RSDA) dat +=1;
		delay_us(10);
	}
    VL_SDA_OUT_();
	return dat;//返回读到的数据
}

void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    gpio_init_struct.Pin = GPIO_PIN_12;                   /* SCL引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);       /* 初始化SCL引脚 */

    gpio_init_struct.Pin = GPIO_PIN_13;                   /* SDA引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;            /* 开漏输出 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);       /* 初始化SDA引脚 */    
}