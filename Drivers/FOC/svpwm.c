#include "svpwm.h"

// 定点数定义：Q15格式（1位符号位，15位小数位）

// 原式 = 根号3 x Ts / 母线电压；Ts = 1时-> 原式 = 根号3 / 母线电压 = 根号3 x (1 / 母线电压)
#define VOL_BUS  12 //母线电压，单位：V
#define SQRT_3_Q15 56756  //√3，虽然这里的√3超过了q15格式范围，但在SVPWM计算中√3都会被/2，所以最终结果还是不会溢出
#define VOL_BUS_INV_Q15 (int16_t)(1.0f / VOL_BUS * 32767) 
//母线电压(倒数表示)； 1/12.0 * 32768 ≈ 0.08333 * 32768
//#define FACTOR_Q15 ((SQRT_3_Q15 * VOL_BUS_INV_Q15) >> 15)  //SVPWM计算因子

//SVPWM缩放系数：等于32768时，占空比最大可以达到50%，波形为完整的马鞍波
// 
//
#define FACTOR_Q15 32767 
#define TS_Q15  FACTOR_Q15    //定时器周期

///**
// * @brief SVPWM算法，Q15定点化实现
// * @param ualpha α轴电压，Q15格式(-32768~32767)
// * @param ubeta β轴电压，Q15格式(-32768~32767)  
// * @param pwm_period PWM定时器重装载值
// * @param ccrs 输出数组，存放三个通道的CCR值
// */
//void set_svpwm_q15(int16_t ualpha, int16_t ubeta, uint16_t pwm_period, uint16_t ccrs[3])
//{
//    int32_t U1, U2, U3;
//    int32_t Tfirst, Tsecond; 
//    uint8_t sect = 0; //扇区
//    int32_t X,Y,Z;
//    int32_t T0, T7; 
//    int32_t taon, tbon, tcon;
//    int32_t ta, tb, tc;
//    int32_t sum;
//    /* 判断扇区 */
//    U1 = ubeta;
//    U2 = ((((int32_t)SQRT_3_Q15 * ualpha) >> 15) - ubeta) >> 1;
//    U3 = (((-(int32_t)SQRT_3_Q15 * ualpha) >> 15) - ubeta) >> 1;
//    if(U1 > 0)sect = 1;   
//    if(U2 > 0)sect |= 2;    
//    if(U3 > 0)sect |= 4;    
//    //printf("%d\n",sect);    
//    /* 计算Tfirst,Tsecond */
//    //用于计算扇区内第一个和第二个非零电压矢量
////    X = ((int32_t)FACTOR_Q15 * ubeta) >> 15;
////    /* 注：((SQRT_3_Q15 * ualpha) >> 16) 原式等于：(SQRT_3_Q15 × ualpha × 0.5)
////     * = (SQRT_3_Q15 × ualpha × 0.5) / 32768 
////     * = (SQRT_3_Q15 × ualpha) / (32768 × 2)
////     * = (SQRT_3_Q15 × ualpha) / 65536
////     * = (SQRT_3_Q15 × ualpha) >> 16  // 因为65536 = 2^16
////     **/
////    Y = ((int32_t)FACTOR_Q15 * ((((int32_t)SQRT_3_Q15 * ualpha) >> 16) + (ubeta >> 1))) >> 15; 
////    Z = ((int32_t)FACTOR_Q15 * (((-(int32_t)SQRT_3_Q15 * ualpha) >> 16) + (ubeta >> 1))) >> 15;

//    /* 当FACTOR_Q15等于32767，即1时，以上3个公式化简为 */
//    X = ubeta;
//    Y = (((int32_t)SQRT_3_Q15 * ualpha) >> 16) + (ubeta >> 1); 
//    Z = ((-(int32_t)SQRT_3_Q15 * ualpha) >> 16) + (ubeta >> 1);
//    switch(sect)
//    {
//        case 0: Tfirst = 0; Tsecond = 0; break;
//        case 1: Tfirst = Z; Tsecond = Y; break;
//        case 2: Tfirst = Y; Tsecond = -X; break;
//        case 3: Tfirst = -Z; Tsecond = X; break;
//        case 4: Tfirst = -X; Tsecond = Z; break;
//        case 5: Tfirst = X; Tsecond = -Y; break;
//        case 6: Tfirst = -Y; Tsecond = -Z; break;
//        default: break;
//    }
//    //printf("%f,%f\n",Tfirst, Tsecond);
//    T0 = T7 = (TS_Q15 - Tfirst - Tsecond) >> 1;
//    /* 过调制处理 */
//    if((Tfirst + Tsecond) > TS_Q15)
//    {
//        sum = Tfirst + Tsecond;
//        Tfirst = ((int64_t)Tfirst * TS_Q15) / sum;
//        Tsecond = ((int64_t)Tsecond * TS_Q15) / sum;
//        T0 = T7 = 0;
//    }
//    /* 换相时间确定 */
//    taon = T0 >> 1; //T0 / 2;
//    tbon = taon + (Tfirst >> 1); //T0 / 2 + Tfirst / 2;
//    tcon = tbon + (Tsecond >> 1); //T0 / 2 + Tfirst / 2 + Tsecond / 2;
//    /* 限制时间范围 [0, 1] */
//    taon = (taon < 0) ? 0 : (taon > 32767 ? 32767 : taon);
//    tbon = (tbon < 0) ? 0 : (tbon > 32767 ? 32767 : tbon);
//    tcon = (tcon < 0) ? 0 : (tcon > 32767 ? 32767 : tcon);    
//    switch(sect)
//    {
//        case 0: ta = 0; tb = 0; tc = 0; break;
//        case 1: ta = tbon; tb = taon; tc = tcon; break;
//        case 2: ta = taon; tb = tcon; tc = tbon; break;
//        case 3: ta = taon; tb = tbon; tc = tcon; break;
//        case 4: ta = tcon; tb = tbon; tc = taon; break;
//        case 5: ta = tcon; tb = taon; tc = tbon; break;
//        case 6: ta = tbon; tb = tcon; tc = taon; break;
//        default: break;
//    }    
//    //printf("%d,%d,%d\n",ta,tb,tc); //打印占空比用于调试
//    /* 转换为PWM占空比：Q15 → 实际计数值 */
//    ccrs[0] = ((int64_t)ta * pwm_period) >> 15;
//    ccrs[1] = ((int64_t)tb * pwm_period) >> 15; 
//    ccrs[2] = ((int64_t)tc * pwm_period) >> 15;
//    //printf("%d,%d,%d\n",ccrs[0],ccrs[1],ccrs[2]);
//}

/********************************************浮点化实现***************************************************/
//#define TS  1    //定时器周期，这里直接使用1ms便于计算
//#define SQRT_3 1.732f   //√3
//#define VOL_BUSf 12 //母线电压，单位：V
//#define FACTOR 1  //SVPWM缩放系数，等于1时，占空比最大可以达到50%，波形完整；如果超过1，容易过调制导致波形失真

///**
// * @brief SVPWM算法
// * 
// * @param ualpha α轴电压
// * @param ubeta β轴电压
// * @param pwm_period PWM定时器重装载值
// * @param ccrs 输出数组，存放三个通道的CCR值
// */
//void set_svpwm(float ualpha, float ubeta, uint16_t pwm_period, uint16_t ccrs[3])
//{
//    float U1, U2, U3;
//    float Tfirst, Tsecond; 
//    uint8_t sect = 0; //扇区
//    float X,Y,Z;
//    float T0, T7; 
//    float taon, tbon, tcon;
//    float ta, tb, tc;
//    float sum;
//    /* 判断扇区 */
//    U1 = ubeta;
//    U2 = (SQRT_3 * ualpha - ubeta) / 2;
//    U3 = (-SQRT_3 * ualpha - ubeta) / 2;
//    if(U1 > 0)sect = 1;   
//    if(U2 > 0)sect |= 2;    
//    if(U3 > 0)sect |= 4;    
//    //printf("%d\n",sect);
//    //计算Tfirst,Tsecond
//    //用于计算扇区内第一个和第二个非零电压矢量
//    X = FACTOR * ubeta;
//    Y = FACTOR * (SQRT_3 * 0.5f * ualpha + 0.5f * ubeta);
//    Z = FACTOR * (-SQRT_3 * 0.5f * ualpha + 0.5f * ubeta);
//    switch(sect)
//    {
//        case 1: Tfirst = Z; Tsecond = Y; break;
//        case 2: Tfirst = Y; Tsecond = -X; break;
//        case 3: Tfirst = -Z; Tsecond = X; break;
//        case 4: Tfirst = -X; Tsecond = Z; break;
//        case 5: Tfirst = X; Tsecond = -Y; break;
//        case 6: Tfirst = -Y; Tsecond = -Z; break;
//        default: break;
//    }
//    //printf("%f,%f\n",Tfirst, Tsecond);
//    T0 = (TS - Tfirst - Tsecond) / 2;
//    T7 = T0;
//    /* 过调制处理 */
//    if((Tfirst + Tsecond) > TS)
//    {
//        sum = Tfirst + Tsecond;
//        Tfirst = Tfirst * TS / sum;
//        Tsecond = Tsecond * TS / sum;
//        T0 = T7 = 0;
//    }
//    /* 换相时间确定 */
//    taon = T0 / 2;
//    tbon = taon + Tfirst / 2;
//    tcon = tbon + Tsecond / 2;
//    /* 时间范围限制：[0，1] */
//    if(taon > 1)taon = 1;
//    else if(taon < 0)taon = 0;
//    if(tbon > 1)tbon = 1;
//    else if(tbon < 0)tbon = 0;
//    if(tcon > 1)tcon = 1;
//    else if(tcon < 0)tcon = 0;
//    switch(sect)
//    {
//        case 1: ta = tbon; tb = taon; tc = tcon; break;
//        case 2: ta = taon; tb = tcon; tc = tbon; break;
//        case 3: ta = taon; tb = tbon; tc = tcon; break;
//        case 4: ta = tcon; tb = tbon; tc = taon; break;
//        case 5: ta = tcon; tb = taon; tc = tbon; break;
//        case 6: ta = tbon; tb = tcon; tc = taon; break;
//        default: break;
//    }
//    /* 将切换时间设置为对应的比较值 */
//    //printf("%f,%f,%f\r\n",ta,tb,tc); //打印占空比用于调试
//    ccrs[0] = pwm_period * ta;
//    ccrs[1] = pwm_period * tb;
//    ccrs[2] = pwm_period * tc;
//    //printf("%d,%d,%d\n",ccrs[0],ccrs[1],ccrs[2]);
//}


void set_svpwm_q15(int16_t ualpha, int16_t ubeta, uint16_t pwm_period, uint16_t ccrs[3])
{
	int16_t t1sat,t2sat;
	int16_t t1,t2;
	int16_t taon,tbon,tcon;
	int16_t Sector;
	int16_t X,Y,Z;
	int16_t A,B,C;
	int16_t Vref1,Vref2,Vref3;
	int16_t PDC_U,PDC_V,PDC_W;
		
	/* ualpha和ubeta输入范围:-16384~+16383 */
	ualpha = ualpha >> 1;
	ubeta = ubeta >> 1;	
	Vref1 = ubeta;
	Vref2 = (short)(((long)ualpha * (long)28377) >> 15) -(short)(((long)ubeta * (long)16384) >> 15);
	Vref3 = 0-(short)(((long)ualpha * (long)28377) >> 15) -(short)(((long)ubeta * (long)16384) >> 15);
	X =	(short)(((long)Vref1 * (long)pwm_period) >> 14);
	Y =	0-(short)(((long)Vref3 * (long)pwm_period) >> 14);
	Z =	0-(short)(((long)Vref2 * (long)pwm_period) >> 14);				//16383max
	/* 计算扇区 */
	if(Vref1 > 0) A = 1;
	else A = 0;
	if(Vref2 > 0) B = 2;
	else B = 0;
	if(Vref3 > 0) C = 4;
	else C = 0;	
	Sector = A + B + C;
	switch(Sector)
	{
		case 1: t1 = Z; t2 = Y; break;			
		case 2: t1 = Y; t2 = 0-X; break;
		case 3: t1 = 0-Z; t2 = X; break;
		case 4: t1 = 0-X; t2 = Z; break;
		case 5: t1 = X; t2 = 0-Y; break;
		case 6: t1 = 0-Y; t2 = 0-Z; break;
		default: t1 = 0; t2 = 0; break;
	}
	if((t1 + t2) > pwm_period) //过调制处理
	{
		t1sat = (short)(((long)pwm_period * (long)t1)/(t1 + t2));
		t2sat = pwm_period - t1sat;
		t1 = t1sat;
		t2 = t2sat;
	}
	taon = (pwm_period - t1-t2) >> 1;
	tbon = taon + t1;
	tcon = tbon + t2;							//tcon>tbon>taon
	switch(Sector)
	{
		case 1: PDC_U = tbon; PDC_V = taon; PDC_W = tcon; break;
		case 2: PDC_U = taon; PDC_V = tcon; PDC_W = tbon; break;
		case 3: PDC_U = taon; PDC_V = tbon; PDC_W = tcon; break;
		case 4: PDC_U = tcon; PDC_V = tbon; PDC_W = taon; break;
		case 5: PDC_U = tcon; PDC_V = taon; PDC_W = tbon; break;
		case 6: PDC_U = tbon; PDC_V = tcon; PDC_W = taon; break;
		default: PDC_U = pwm_period >> 1; PDC_V = pwm_period >> 1; PDC_W = pwm_period >> 1; break;
	}
	ccrs[0] = PDC_U;
	ccrs[1] = PDC_V;
	ccrs[2] = PDC_W;
	//printf("%d,%d,%d\n", PDC_U, PDC_V, PDC_W);
}

