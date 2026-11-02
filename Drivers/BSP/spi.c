#include "spi.h"

SPI_HandleTypeDef g_spi3_handler; /* SPI3句柄 */

/**
 * @brief       SPI初始化代码
 * @note        主机模式, 8位数据, 禁止硬件片选
 * @param       无
 * @retval      无
 */
void spi3_init(void)
{
    __HAL_RCC_SPI3_CLK_ENABLE(); /* SPI3时钟使能 */

    g_spi3_handler.Instance = SPI3;                                /* SPI3 */
    g_spi3_handler.Init.Mode = SPI_MODE_MASTER;                    /* 设置SPI工作模式，设置为主模式 */
    g_spi3_handler.Init.Direction = SPI_DIRECTION_2LINES;          /* 设置SPI单向或者双向的数据模式:SPI设置为双线模式 */
    g_spi3_handler.Init.DataSize = SPI_DATASIZE_16BIT;             /* 设置SPI的数据大小:SPI发送接收16位帧结构 */
    g_spi3_handler.Init.CLKPolarity = SPI_POLARITY_LOW;            /* 串行同步时钟的空闲状态为低电平 */
    g_spi3_handler.Init.CLKPhase = SPI_PHASE_2EDGE;                /* 串行同步时钟的第二个跳变沿（上升或下降）数据被采样 */
    g_spi3_handler.Init.NSS = SPI_NSS_SOFT;                        /* NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制 */
    g_spi3_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* 定义波特率预分频的值:波特率预分频值为256 */
    g_spi3_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;               /* 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始 */
    g_spi3_handler.Init.TIMode = SPI_TIMODE_DISABLE;               /* 关闭TI模式 */
    g_spi3_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; /* 关闭硬件CRC校验 */
    g_spi3_handler.Init.CRCPolynomial = 7;                         /* CRC值计算的多项式 */
    HAL_SPI_Init(&g_spi3_handler);                                 /* 初始化 */
    
    spi3_set_speed(SPI_SPEED_4); /* SPI3 速度设置Mhz */
    __HAL_SPI_ENABLE(&g_spi3_handler); /* 使能SPI3 */
    //spi1_read_write_byte(0Xff); /* 启动传输, 实际上就是产生8个时钟脉冲, 达到清空DR的作用, 非必需 */
}

/**
 * @brief       SPI底层驱动，时钟使能，引脚配置
 *   @note      此函数会被HAL_SPI_Init()调用
 * @param       hspi:SPI句柄
 * @retval      无
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    if (hspi->Instance == SPI3)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE(); /* GPIOC时钟使能 */
        
        //CS片选引脚PC9(推挽输出)
        gpio_init_struct.Pin = GPIO_PIN_9;
        gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &gpio_init_struct);
        
        /* SCK - PC10 */
        gpio_init_struct.Pin = GPIO_PIN_10;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF6_SPI3; //引脚复用
        HAL_GPIO_Init(GPIOC, &gpio_init_struct);
        
        /* MISO - PC11 */
        gpio_init_struct.Pin = GPIO_PIN_11;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF6_SPI3; //引脚复用
        HAL_GPIO_Init(GPIOC, &gpio_init_struct);
        
        /* MOSI - PC12 */
        gpio_init_struct.Pin = GPIO_PIN_12;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF6_SPI3; //引脚复用
        HAL_GPIO_Init(GPIOC, &gpio_init_struct);
    }
}

/**
 * @brief       SPI1速度设置函数
 *   @note      SPI1时钟选择来自APB2, 即PCLK2, 为84Mhz
 *              SPI速度 = PCLK2 / 2^(speed + 1)
 * @param       speed   : SPI1时钟分频系数
 *                        取值为SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_256
 * @retval      无
 */
void spi3_set_speed(uint8_t speed)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed)); /* 判断有效性 */
    __HAL_SPI_DISABLE(&g_spi3_handler);             /* 关闭SPI */
    g_spi3_handler.Instance->CR1 &= 0XFFC7;         /* 位3-5清零，用来设置波特率 */
    g_spi3_handler.Instance->CR1 |= speed << 3;     /* 设置SPI速度 */
    __HAL_SPI_ENABLE(&g_spi3_handler);              /* 使能SPI */
}

//函数功能：读写数据16位
//txdata：要发送的数据
//返回值：读到的数据
uint16_t spi3_read_write_data16(uint16_t txdata)
{
    uint16_t rxdata = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //CS拉低
    HAL_SPI_TransmitReceive(&g_spi3_handler, (uint8_t*)&txdata, (uint8_t*)&rxdata, 1, 1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); //CS拉高    
    return rxdata;
}

//函数功能：写数据16位
//txdata：要发送的数据
void spi3_write_data16(uint16_t txdata)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //CS拉低
    HAL_SPI_Transmit(&g_spi3_handler, (uint8_t*)&txdata, 2, 1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); //CS拉高
}
