#include "gpio.h"

/**
 * @brief 初始化部分GPIO
 * @return 无
 */
void gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    DEBUG_CLOCK_ENABLE();                                 /* 时钟使能 */

    gpio_init_struct.Pin = DEBUG_PIN;                   /* 引脚选择 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(DEBUG_PORT, &gpio_init_struct);       /* 初始化引脚 */
}
