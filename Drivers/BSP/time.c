#include "time.h"

TIM_HandleTypeDef g_tim6_handle; //定时器6句柄
TIM_HandleTypeDef g_tim1_handle; //定时器1句柄
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0}; /* 刹车和死区时间配置 */

/**
 * @brief 定时器1初始化
 * @param pre 分频值
 * @param pod 重装载值
 */
void time1_init(uint32_t pre, uint32_t pod)
{  
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef timx_oc_npwm_chy={0};            /* 定时器输出 ,用于配置定时器通道的模式 */
    
    g_tim1_handle.Instance = TIM1;                            /* 定时器选择*/
    g_tim1_handle.Init.Prescaler = pre;                       /* 定时器分频 */
    g_tim1_handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1; /* 中心对齐模式 */
    g_tim1_handle.Init.Period = pod;                          /* 自动重装载值 */
    g_tim1_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* 时钟分频 */
    g_tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /* 使能TIM1_ARR进行缓冲 */
    g_tim1_handle.Init.RepetitionCounter = 0;                  /* 重复计数器初始值 */
    
    /* 初始化基本定时器参数 */
    HAL_TIM_PWM_Init(&g_tim1_handle);
    
    /* 刹车和死区配置 */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
    sBreakDeadTimeConfig.DeadTime = TIM1_DIE_ARE;	//设置死区
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;  
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim1_handle, &sBreakDeadTimeConfig);
    
    /* 配置PWM输出通道参数 */
    timx_oc_npwm_chy.OCMode = TIM_OCMODE_PWM1;                 /* 模式选择 PWM模式1 */
    timx_oc_npwm_chy.Pulse = 0;                                /* 初始比较值，占空比0% */
    timx_oc_npwm_chy.OCPolarity = TIM_OCPOLARITY_HIGH;         /* 输出极性：高电平有效 */
    timx_oc_npwm_chy.OCNPolarity = TIM_OCNPOLARITY_HIGH;       /* 互补输出极性：高电平有效 */
    timx_oc_npwm_chy.OCFastMode = TIM_OCFAST_DISABLE;          /* 快速模式禁用 */
    timx_oc_npwm_chy.OCIdleState = TIM_OCIDLESTATE_RESET;      /* 空闲状态：低电平 */
    timx_oc_npwm_chy.OCNIdleState = TIM_OCNIDLESTATE_RESET;    /* 互补空闲状态：低电平 */
    
    /* 配置通道1（主输出和互补输出） */
    HAL_TIM_PWM_ConfigChannel(&g_tim1_handle, &timx_oc_npwm_chy, TIM_CHANNEL_1);
    
    /* 配置通道2（主输出和互补输出） */
    HAL_TIM_PWM_ConfigChannel(&g_tim1_handle, &timx_oc_npwm_chy, TIM_CHANNEL_2);
    
    /* 配置通道3（主输出和互补输出） */
    HAL_TIM_PWM_ConfigChannel(&g_tim1_handle, &timx_oc_npwm_chy, TIM_CHANNEL_3);
    
    /* 配置通道4（仅主输出，用于触发ADC） */
    timx_oc_npwm_chy.OCNIdleState = TIM_OCNIDLESTATE_RESET;    /* 互补输出禁用 */
    HAL_TIM_PWM_ConfigChannel(&g_tim1_handle, &timx_oc_npwm_chy, TIM_CHANNEL_4);
    
    /* 主模式触发源配置：通道4比较事件触发TRGO */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF; 
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&g_tim1_handle, &sMasterConfig);
    
    /* 关闭PWM输出（主输出和互补输出） */
    HAL_TIM_PWM_Stop(&g_tim1_handle, TIM_CHANNEL_1);          /* 关闭通道1主输出 */
    HAL_TIMEx_PWMN_Stop(&g_tim1_handle, TIM_CHANNEL_1);       /* 关闭通道1互补输出 */
    
    HAL_TIM_PWM_Stop(&g_tim1_handle, TIM_CHANNEL_2);          /* 关闭通道2主输出 */
    HAL_TIMEx_PWMN_Stop(&g_tim1_handle, TIM_CHANNEL_2);       /* 关闭通道2互补输出 */
    
    HAL_TIM_PWM_Stop(&g_tim1_handle, TIM_CHANNEL_3);          /* 关闭通道3主输出 */
    HAL_TIMEx_PWMN_Stop(&g_tim1_handle, TIM_CHANNEL_3);       /* 关闭通道3互补输出 */
   
    HAL_TIM_PWM_Start(&g_tim1_handle, TIM_CHANNEL_4);          /* 开启通道4主输出 */
    
    TIM1->CCR4 = ADC_SAMPLE_POINT; //ADC采样点设置
}


/**
 * @brief 定时器PWM驱动初始化
 * @retval 无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_init_struct; /* GPIO结构体句柄 */
    
    if(htim->Instance == TIM1)
    {
        /* 使能时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();  /* GPIOA时钟使能 */
        __HAL_RCC_GPIOB_CLK_ENABLE();  /* GPIOB时钟使能（互补输出引脚） */
        __HAL_RCC_TIM1_CLK_ENABLE();   /* 定时器1时钟使能 */
        
        /* 配置主输出引脚 PA8,PA9,PA10  */
        gpio_init_struct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10; 
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;       /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;           /* 无上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH; /* 高速 */
        gpio_init_struct.Alternate = GPIO_AF1_TIM1;    /* 复用为TIM1 */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
        
        /* 配置互补输出引脚 PB13，PB0, PB1 */
        gpio_init_struct.Pin = GPIO_PIN_13 | GPIO_PIN_0 | GPIO_PIN_1; 
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;       /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;           /* 无上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH; /* 高速 */
        gpio_init_struct.Alternate = GPIO_AF1_TIM1;    /* 复用为TIM1 */
        HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    }
}

/**
 * @brief 定时器6初始化
 * @param pre 分频值
 * @param pod 重装载值
 */
void time6_init(uint32_t pre,uint32_t pod)
{
    g_tim6_handle.Instance = TIM6;//定时器6地址
    g_tim6_handle.Init.Prescaler = pre;//分频值
    g_tim6_handle.Init.Period = pod;//重新装载值
    HAL_TIM_Base_Init(&g_tim6_handle);//定时器初始化
    HAL_TIM_Base_Stop_IT(&g_tim6_handle); //关闭定时器中断
}

/**
 * @brief 定时器底层驱动初始化
 * @retval 无
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();//时钟使能
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);//优先级设置
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);//总中断使能
    }
}

/**
 * @brief 定时器6中断服务函数
 * @retval 无
 */
void TIM6_DAC_IRQHandler()
{
    HAL_TIM_IRQHandler(&g_tim6_handle);//定时器中断处理函数，该函数会调用回调函数
}

