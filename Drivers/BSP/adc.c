#include "adc.h"

ADC_HandleTypeDef g_adc_injected_handle;  // ADC注入组句柄
/* 存储ADC采样原始值，[0]电机1A相电流，[1]电机1B相电流，[2]电机1C相电流 */
uint16_t adc_value[3]; 


/**
 * @brief       ADC注入组初始化
 * @note        使用TIM1通道4触发注入组采样
 */
void adc_injected_init(void)
{
    __HAL_RCC_ADC3_CLK_ENABLE();  // 使能ADC3时钟
    __HAL_RCC_GPIOF_CLK_ENABLE(); //使能GPIOF时钟
    
    // 引脚配置 
    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);
    
    // ADC基本配置
    g_adc_injected_handle.Instance = ADC3;
    g_adc_injected_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2; //时钟：42Mhz
    g_adc_injected_handle.Init.Resolution = ADC_RESOLUTION_12B;
    g_adc_injected_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_adc_injected_handle.Init.ScanConvMode = ENABLE;       // 启用扫描模式
    g_adc_injected_handle.Init.ContinuousConvMode = DISABLE; // 禁用连续转换
    g_adc_injected_handle.Init.DiscontinuousConvMode = DISABLE;
    g_adc_injected_handle.Init.NbrOfDiscConversion = 0;
    g_adc_injected_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START; // 软件触发（实际由注入组触发）
    g_adc_injected_handle.Init.DMAContinuousRequests = DISABLE;       // 禁用DMA
    g_adc_injected_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&g_adc_injected_handle);

    // 注入组配置
    ADC_InjectionConfTypeDef sConfigInjected = {0};
    sConfigInjected.InjectedNbrOfConversion = 3;  // 3个注入通道
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES; //配置采样时间：3个时钟周期
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4; // TIM1通道4触发
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING; //上升沿触发
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;
    
    // 配置注入通道
    sConfigInjected.InjectedChannel = ADC_CHANNEL_4; // PF6对应ADC3通道4
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    HAL_ADCEx_InjectedConfigChannel(&g_adc_injected_handle, &sConfigInjected);
    
    sConfigInjected.InjectedChannel = ADC_CHANNEL_5; // PF7对应ADC3通道5
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    HAL_ADCEx_InjectedConfigChannel(&g_adc_injected_handle, &sConfigInjected);

    sConfigInjected.InjectedChannel = ADC_CHANNEL_6; // PF8对应ADC3通道6
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    HAL_ADCEx_InjectedConfigChannel(&g_adc_injected_handle, &sConfigInjected);
    
    // 启用注入组中断
    HAL_ADCEx_InjectedStart_IT(&g_adc_injected_handle);
    
    // 配置ADC中断
    HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/**
 * @brief       ADC全局中断服务函数
 */
void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&g_adc_injected_handle);
}

/**
 * @brief 注入组转换完成回调函数
 * @note ADC采样，电机控制，Ts=50us
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC3) 
    {     
		/* 获取原始ADC值 */
        adc_value[0] = ring_quen_filter(HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1), &ringquen1);
        adc_value[1] = ring_quen_filter(HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2), &ringquen2);
        adc_value[2] = ring_quen_filter(HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3), &ringquen3);
		MotorHandler(&motor1);
	}
}

