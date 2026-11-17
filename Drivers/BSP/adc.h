#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "bldc_control.h"
#include "quen.h"

extern ADC_HandleTypeDef g_adc_injected_handle;
extern uint16_t adc_value[3]; 

void adc_injected_init(void);
/* ================= 普通组(规则组): 温度 / 母线电流 / 母线电压 ================= */
/* 通道分配(与 adc_regular_value[] 下标一一对应):
     [0] PF9  ADC3_IN7   驱动板 NTC      HNTC0603-103F3450FA: R25=10k, B(25/50)=3450
     [1] PF10 ADC3_IN8   母线电流        5mohm 采样电阻 + INA204A2(增益 50V/V), 多点平均
     [2] PC0  ADC3_IN10  母线电压        分压系数 K = 1/8 (母线 24V -> 3.0V)
   说明: 与电流环共用一个 ADC3。电流环走注入组(20kHz 硬件触发, 优先级高),
         这三路走普通组, 10ms 状态机里软件触发后轮询读取(母线电流一次采 48 点)。 */
#define ADC_REG_NTC_IDX       0
#define ADC_REG_IBUS_IDX      1
#define ADC_REG_VBUS_IDX      2

/* --- ADC 基准 --- */
#define ADC_VREF              3.3f          /* 参考电压 [V] */
#define ADC_FULL_SCALE        4095.0f       /* 12bit 满量程 */

/* --- NTC 分压 ---
   已确认: 板上是 "NTC 下拉到 GND + 10k 上拉到 3.3V" -> 温度升高、阻值减小、电压升高。
   若以后换成 "NTC 接 3.3V + 10k 下拉到 GND"(电压随温度降低), 把下面改成 0。 */
#define ADC_NTC_PULLUP_TO_VCC 1
#define ADC_NTC_R25           10000.0f      /* 25度标称阻值 [ohm] */
#define ADC_NTC_B             3450.0f       /* B 常数(25/50度) */
#define ADC_NTC_RPULL         10000.0f      /* 分压对侧电阻 [ohm] */
#define ADC_NTC_FAULT_C       (-40.0f)      /* 采样开路/短路时返回的故障值 [度] */

/* --- 母线电流: I = (Vadc - Voffset) / (GAIN * RSHUNT) ---
   INA204A2 增益 50, 5mohm -> 0.25V/A, 3.3V 满量程约 13.2A。
   已确认: 单向采样(REF 接地, 0A 输出 0V) -> Voffset = 0。
   若以后改成双向采样(REF 偏置到 1.65V, 0A 输出 1.65V), 把 Voffset 改成 1.65。 */
#define BUS_CUR_GAIN          50.0f         /* 运放增益 [V/V] */
#define BUS_CUR_RSHUNT        0.005f        /* 采样电阻 [ohm] */
#define BUS_CUR_VOFFSET       0.0f          /* 零电流对应的输出电压 [V] */
#define BUS_CUR_ZERO_MA       30.0f         /* 零点死区 [mA], 抑制零漂 */

/* --- 母线电压: Vbus = Vadc * RATIO --- */
#define BUS_VOL_RATIO         8.0f          /* 分压系数的倒数, K = 1/8 */

/* --- 普通组转换超时: 单次转换 96 个 ADC 周期约 2.3us(@42MHz),
       这里给约 10us 的轮询余量, 超时说明被注入组打断, 本次采样作废 --- */
#define ADC_CONV_TIMEOUT      200

/* --- 母线电流多点平均 ---------------------------------------------------------
   母线电流是 20kHz PWM 的斩波波形, 瞬时值在"导通=相电流/关断=0"之间跳。
   单点采样落在哪个相位, 完全取决于 10ms 定时器中断被电流环(优先级 0)
   推迟了多少微秒 —— 推迟量每次都不同, 所以读出来就是大幅随机跳动。
   解决办法: 在一次采样窗口里连续转换多次(总时长覆盖 1 个以上 PWM 周期 50us),
   再取平均, 这样结果与采样起始时刻无关, 才是真正的直流平均值。
   单次转换 96 个 ADC 时钟 = 2.29us, 48 次 ≈ 110us ≈ 2.2 个 PWM 周期。 */
#define ADC_IBUS_OVERSAMPLE   48
#define ADC_IBUS_TIMEOUT      200           /* 单次转换轮询上限(约 10us) */
#define ADC_IBUS_IIR          0.5f          /* 跨周期平滑系数: 0.5 约 20ms 跟随, 1=不平滑 */

extern uint16_t adc_regular_value[3];
extern float    adc_ibus_raw_avg;           /* 本轮平均原始值(调试用) */
extern uint16_t adc_ibus_raw_min;           /* 本轮最小原始值(看纹波幅度) */
extern uint16_t adc_ibus_raw_max;           /* 本轮最大原始值 */

void  adc_regular_init(void);
void  adc_regular_update(void);
float ADC_GetTemperature_C(void);
float ADC_GetBusCurrent_mA(void);
float ADC_GetBusVoltage_V(void);

#endif 
