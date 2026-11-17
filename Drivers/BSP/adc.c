#include "adc.h"
#include <math.h>

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
    g_adc_injected_handle.Init.ContinuousConvMode = DISABLE;        /* 普通组单次软件触发, 不做连续转换(原因见 adc_regular_update) */
    g_adc_injected_handle.Init.DiscontinuousConvMode = DISABLE;
    g_adc_injected_handle.Init.NbrOfDiscConversion = 0;
    g_adc_injected_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START; // 软件触发（实际由注入组触发）
    g_adc_injected_handle.Init.DMAContinuousRequests = DISABLE;      /* 普通组不用 DMA */
    g_adc_injected_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    g_adc_injected_handle.Init.NbrOfConversion = 1;   /* 普通组只用 rank1, 靠改 SQR3 换通道 */
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

/* ================= 普通组(规则组): 温度 / 母线电流 / 母线电压 ================= */

uint16_t adc_regular_value[3];          /* [0]NTC  [1]母线电流(平均)  [2]母线电压 */
float    adc_ibus_raw_avg;              /* 母线电流本轮平均原始值(调试用) */
uint16_t adc_ibus_raw_min = 0xFFFF;     /* 本轮最小原始值 */
uint16_t adc_ibus_raw_max;              /* 本轮最大原始值 */

/**
 * @brief       普通组单通道转换(软件触发 + 轮询 EOC)
 * @param       ch       ADC 通道号
 * @param       timeout  轮询次数上限
 * @retval      >= 0 转换结果; -1 = 超时(被注入组打断), 本次采样作废
 * @note        普通组只用 rank1, 每次改 SQR3 换通道 -> 一次转换出一个结果。
 */
static int32_t adc_regular_convert(uint32_t ch, uint32_t timeout)
{
    uint32_t t;

    (void)ADC3->DR;                     /* 丢掉上一轮可能残留的数据并清 EOC */
    ADC3->SQR3 = ch;                    /* SQ1 = 通道号(序列长度 1) */
    ADC3->CR2 |= ADC_CR2_SWSTART;

    for(t = 0; t < timeout; t++)
    {
        if(ADC3->SR & ADC_SR_EOC)
        {
            return (int32_t)ADC3->DR;   /* 读 DR 同时清 EOC */
        }
    }
    return -1;
}

/**
 * @brief       母线电流采样: 在跨 1 个以上 PWM 周期的窗口内多点平均
 * @retval      平滑后的原始值(0~4095)
 * @note        母线电流被 PWM 斩波, 单点采样值取决于采样瞬间的 PWM 相位。
 *              10ms 状态机由 TIM6 触发, 而 TIM6 中断会被 20kHz 电流环
 *              (ADC 中断, 优先级 0) 随机推迟 0~30us, 于是同一个 10ms 采样点
 *              每次落在斩波波形的不同位置 -> 读数大幅跳动。
 *              这里连续转换 ADC_IBUS_OVERSAMPLE 次(约 110us = 2.2 个 PWM 周期),
 *              取平均 -> 与起始时刻无关, 得到真正的直流平均电流。
 *              单次转换之间不做等待, 中间可能被注入组(电流环)插入, 那几次
 *              转换只是被推迟, 完成后 EOC 照样会置位, 不会丢数据。
 */
static uint16_t adc_ibus_sample(void)
{
    static float ibus_f = -1.0f;        /* 平滑后的原始值, <0 表示还没采过 */
    uint32_t i, n = 0, sum = 0;
    uint16_t mn = 0xFFFF, mx = 0;
    float raw;
    int32_t v;

    for(i = 0; i < ADC_IBUS_OVERSAMPLE; i++)
    {
        v = adc_regular_convert(ADC_CHANNEL_8, ADC_IBUS_TIMEOUT);
        if(v < 0)
        {
            continue;                   /* 超时的那次丢掉, 不影响平均 */
        }
        sum += (uint32_t)v;
        n++;
        if((uint16_t)v < mn) mn = (uint16_t)v;
        if((uint16_t)v > mx) mx = (uint16_t)v;
    }

    if(n == 0)                          /* 全部超时: 保留上一次的有效值 */
    {
        return adc_regular_value[ADC_REG_IBUS_IDX];
    }

    raw = (float)sum / (float)n;
    adc_ibus_raw_avg = raw;
    adc_ibus_raw_min = mn;
    adc_ibus_raw_max = mx;

    if(ibus_f < 0.0f)
    {
        ibus_f = raw;                   /* 第一次直接赋值, 不做平滑 */
    }
    else
    {
        ibus_f += ADC_IBUS_IIR * (raw - ibus_f);
    }

    return (uint16_t)(ibus_f + 0.5f);
}

/**
 * @brief       采集一轮: 温度 / 母线电流 / 母线电压
 * @note        在 10ms 状态机里调用。温度、电压各一次单转换(约 2.3us),
 *              母线电流做 48 次多点平均(约 110us)。合计约 115us, 占 10ms 的 1.2%。
 *              某一路被电流环注入组打断时保留上一次的有效值。
 *              不用 "3 rank + 连续转换 + DMA" 的原因: 注入触发会打断普通组序列
 *              并从 rank1 重新开始, 而 DMA 指针按转换次数推进, 一被打断三个通道
 *              的数据就会串位; 单次转换没有这个问题。
 * @retval      无
 */
void adc_regular_update(void)
{
    int32_t v;

    v = adc_regular_convert(ADC_CHANNEL_7, ADC_CONV_TIMEOUT);       /* PF9  NTC 温度 */
    if(v >= 0) adc_regular_value[ADC_REG_NTC_IDX] = (uint16_t)v;

    adc_regular_value[ADC_REG_IBUS_IDX] = adc_ibus_sample();        /* PF10 母线电流(平均) */

    v = adc_regular_convert(ADC_CHANNEL_10, ADC_CONV_TIMEOUT);      /* PC0  母线电压 */
    if(v >= 0) adc_regular_value[ADC_REG_VBUS_IDX] = (uint16_t)v;
}

/**
 * @brief       普通组(规则组)初始化
 * @note        PF9=ADC3_IN7(NTC) / PF10=ADC3_IN8(母线电流) / PC0=ADC3_IN10(母线电压)
 *              ADC3 的位配置(NbrOfConversion=1 / 不做连续转换)在 adc_injected_init
 *              里完成, 这里只开时钟、把三个脚设成模拟输入, 然后先采一轮。
 * @retval      无
 */
void adc_regular_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* PF9 / PF10 / PC0 全部配成模拟输入 */
    GPIO_InitStruct.Pin  = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 采样时间必须显式设长: NTC 分压的源阻抗约 5k, 复位默认 3 个周期(71ns)采不准。
       这里只借用它把 SMPR1/SMPR2 里对应通道的采样时间写好,
       序列长度是 1, 每次转换时会重新写 SQR3 指定通道。 */
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset       = 0;

    sConfig.Channel = ADC_CHANNEL_7;                /* PF9  NTC */
    HAL_ADC_ConfigChannel(&g_adc_injected_handle, &sConfig);
    sConfig.Channel = ADC_CHANNEL_8;                /* PF10 母线电流 */
    HAL_ADC_ConfigChannel(&g_adc_injected_handle, &sConfig);
    sConfig.Channel = ADC_CHANNEL_10;               /* PC0  母线电压 */
    HAL_ADC_ConfigChannel(&g_adc_injected_handle, &sConfig);

    adc_regular_update();               /* 先采一轮, 状态机第一次读到的就是有效值 */
}

/**
 * @brief       原始值 -> 引脚电压 [V]
 */
static float adc_raw_to_volt(uint16_t raw)
{
    return (float)raw * (ADC_VREF / ADC_FULL_SCALE);
}

/**
 * @brief       读取母线电压 [V]
 * @note        Vbus = Vadc * 8, 母线 24V 时 ADC 约 3.0V(满量程 26.4V)
 */
float ADC_GetBusVoltage_V(void)
{
    return adc_raw_to_volt(adc_regular_value[ADC_REG_VBUS_IDX]) * BUS_VOL_RATIO;
}

/**
 * @brief       读取母线电流 [mA]
 * @note        I = (Vadc - Voffset) / (50 * 0.005) = (Vadc - Voffset) * 4A/V
 *              3.3V 满量程对应 13.2A; 小于零点死区的值当 0 处理。
 */
float ADC_GetBusCurrent_mA(void)
{
    float v, i;

    v = adc_raw_to_volt(adc_regular_value[ADC_REG_IBUS_IDX]);
    i = (v - BUS_CUR_VOFFSET) / (BUS_CUR_GAIN * BUS_CUR_RSHUNT) * 1000.0f;

    if(i > -BUS_CUR_ZERO_MA && i < BUS_CUR_ZERO_MA)
    {
        i = 0.0f;
    }
    return i;
}

/**
 * @brief       读取驱动板温度 [度]
 * @note        NTC 分压 -> 阻值 -> B 常数公式: 1/T = 1/T25 + ln(R/R25)/B
 *              25度时 ADC 约 1.65V(raw 2048), 100度时约 0.29V(raw 365)。
 */
float ADC_GetTemperature_C(void)
{
    float v, r, t;

    v = adc_raw_to_volt(adc_regular_value[ADC_REG_NTC_IDX]);

    /* 采样电压贴到电源轨 -> 认为 NTC 开路或短路, 返回故障值, 避免 log 溢出 */
    if(v <= 0.002f || v >= (ADC_VREF - 0.002f))
    {
        return ADC_NTC_FAULT_C;
    }

#if ADC_NTC_PULLUP_TO_VCC
    r = ADC_NTC_RPULL * v / (ADC_VREF - v);         /* NTC 下拉到 GND */
#else
    r = ADC_NTC_RPULL * (ADC_VREF - v) / v;         /* NTC 接到 VCC */
#endif

    t = 1.0f / (1.0f / 298.15f + logf(r / ADC_NTC_R25) / ADC_NTC_B) - 273.15f;
    return t;
}
