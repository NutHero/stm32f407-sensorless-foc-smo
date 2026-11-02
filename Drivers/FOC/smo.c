#include "smo.h"

#define TWO_PI	6.283185f	//2π
#define PI	3.1415926f	//圆周率

#define MA_TO_A        (1 / 1000.0f)   // 电流单位mA转A
#define Q15_TO_V        (M1_POWER_VOL / (1.7320508f * 32768.0f))   // ≈0.000732

#define M1_PHASE_R		0.4f	//相电阻，单位：Ω
#define M1_PHASE_L		 0.000156f	//相电感，单位：H。表贴式BLDC，Ld约等于Lq

// |E| = sqrt(Ealpha^2 + Ebeta^2) = 磁链 x 电角速度
//磁链用于验证估计的Ealpha，Ebeta是否准确
#define LINKAGE		0.0021f		//磁链，单位：Wb
#define TS	0.0001	//周期，单位：秒
#define WE_LIMIT 7000.0f   // PLL 积分抗饱和(电角速度 rad/s)
Lpf smo_lpf_valpha = {0, 0.1};
Lpf smo_lpf_vbeta = {0, 0.1};
/* ===== 无感启动参数 ===== */
#define SMO_OL_ANGLE_STEP   0.019f   /* 开环强拖: 每次(100us)推进的电角度, 约 190rad/s 电频率 */
#define SMO_BEMF_THRESHOLD  0.15f    /* 反电动势幅值超过该值(V)才允许切换，
计算过程：先打印强拖稳定后的 Eamp = sqrt(Ealpha_esc2 + Ebeta_esc2)，把阈值设成它的 40%~60%；转速阈值设成实际切换转速的 30%~50%。 
*/
#define SMO_SWITCH_SPEED    60.0f    /* 且估计电角速度超过该值(rad/s)才允许切换 */
Smo smo;

//滑膜观测器初始化
void BLDC_Smo_Init(void)
{
	smo.kp_pll = 20;
	smo.ki_pll = 10000;
	smo.theta_err_limit = 1.047197; //误差范围：60度
	smo.h = 2; //给定滑膜增益
}

//滑膜控制器：速度,角度估计
void Smo_Control(uint32_t interver_us)
{
	float ts = 0.0;	//计算周期，单位：秒
	float ia_f = 0, ib_f = 0;
	smo.we_mea = motor1.mea_speed * (PI / 30) * M1_POLE_PAIRS; //转速RPM 转为 电角速度
	smo.theta_mea = (motor1.ele_angle % ENCODER_FACTOR) * (TWO_PI / ENCODER_FACTOR); //实际电角度，0~2π
	ts = interver_us / 1000000.0f; //将时间单位转为秒
	
	//定点数转浮点
	RevPark rp_smo = RevPark_Transform15(motor1.out_vq, motor1.out_vd, motor1.motor_axle_angle);
	ia_f = motor1.mea_ia * MA_TO_A;	//将三相电流单位转为A
	ib_f = motor1.mea_ib * MA_TO_A; //将三相电流单位转为A
	Clarkef cl_smo = Clarke_Transform(ia_f, ib_f);
	smo.Ualpha = rp_smo.ualpha * Q15_TO_V;          // 实际施加的 alpha 轴电压(V)
	smo.Ubeta = rp_smo.ubeta  * Q15_TO_V;          // 实际施加的 beta  轴电压(V)
	smo.Ialpha_mea = cl_smo.ialpha;  
	smo.Ibeta_mea  = cl_smo.ibeta;  
	
	//计算估计电流
	smo.Ialpha_esc = smo.Ialpha_esc_pre + (ts / M1_PHASE_L) * (\
					-M1_PHASE_R * smo.Ialpha_esc_pre \
					+smo.Ualpha - smo.Valpha_esc);
	smo.Ibeta_esc = smo.Ibeta_esc_pre + (ts / M1_PHASE_L) * (\
					-M1_PHASE_R * smo.Ibeta_esc_pre \
					+smo.Ubeta - smo.Vbeta_esc);
	smo.Ialpha_esc_pre = smo.Ialpha_esc;
	smo.Ibeta_esc_pre = smo.Ibeta_esc;	
	//计算电流误差
	smo.Ialpha_err = smo.Ialpha_esc - smo.Ialpha_mea;
	smo.Ibeta_err = smo.Ibeta_esc - smo.Ibeta_mea;
	
	//滑膜切换项
	float Ialpha_err_limit = 0, Ibeta_err_limit = 0;
	Ialpha_err_limit = smo.Ialpha_err;
	_constrain(Ialpha_err_limit, -1, 1);
	smo.Valpha_esc = smo.h * Ialpha_err_limit;
	
	Ibeta_err_limit = smo.Ibeta_err;
	_constrain(Ibeta_err_limit, -1, 1);
	smo.Vbeta_esc = smo.h * Ibeta_err_limit;
	
	//对Q,D轴电压滤波得到反电动势Eq,Ed
	smo.Ealpha_esc = LowPass_Filter(smo.Valpha_esc, &smo_lpf_valpha);
	smo.Ebeta_esc = LowPass_Filter(smo.Vbeta_esc, &smo_lpf_vbeta);
	
	//计算滑膜增益
	float Ialpha_err_abs = 0, Ibeta_err_abs = 0;
	Ialpha_err_abs = smo.Ialpha_err;
	Ibeta_err_abs = smo.Ibeta_err;
	if(Ialpha_err_abs < 0)Ialpha_err_abs = -Ialpha_err_abs;
	if(Ibeta_err_abs < 0)Ibeta_err_abs = -Ibeta_err_abs;
	smo.h1 = -M1_PHASE_R * Ialpha_err_abs + smo.Ealpha_esc * Ialpha_err_limit;
	smo.h2 = -M1_PHASE_R * Ibeta_err_abs + smo.Ebeta_esc * Ibeta_err_limit;
	
	//PLL计算
	/* 反电动势矢量的手性随转向改变: 反转时 E 整体反号,
	   鉴相需同步取反, 否则 PLL 变正反馈、反转无法锁定(表现为堵转) */
	float pll_dir = (motor1.out_vq >= 0) ? 1.0f : -1.0f;
	smo.theta_err = pll_dir * (-smo.Ealpha_esc * cosf(smo.theta_est) - smo.Ebeta_esc * sinf(smo.theta_est));
	_constrain(smo.theta_err, -smo.theta_err_limit, smo.theta_err_limit);

	smo.sum_pll += smo.ki_pll * smo.theta_err * ts;
	_constrain(smo.sum_pll, -WE_LIMIT, WE_LIMIT);
	smo.we_est = smo.sum_pll + smo.kp_pll * smo.theta_err;

	smo.theta_est += smo.we_est * ts;
	while(smo.theta_est >= TWO_PI) smo.theta_est -= TWO_PI;
	while(smo.theta_est < 0.0f)    smo.theta_est += TWO_PI;
}


/* 将电角度[rad]转换为换相用的 Q15 角度(motor_axle_angle)。
 * Q15 一圈 = 65536 (0~32767 对应 0~pi, -32768~-1 对应 pi~2pi)。
 * 注意: 这是电角度; 电机轴机械角 = theta_est / M1_POLE_PAIRS。 */
int16_t Smo_ElecToQ15(float theta_rad)
{
	int32_t a = (int32_t)(theta_rad * (65536.0f / TWO_PI)) & 0xFFFF;
	if(a >= 32768) a -= 65536;
	return (int16_t)a;
}

/* 无感启动: 推进开环强拖电角度, 并检测反电动势。
 * 返回 1 = 已检测到反电动势, 可切无感(用 smo.theta_est 换相);
 *      0 = 仍在开环强拖(用 smo.ol_theta 换相)。 */
uint8_t Smo_Startup(void)
{
	float Eamp;

	/* 1) 开环强拖: 按固定步长推进电角度 */
	if(motor1.open_vq >= 0) smo.ol_theta += SMO_OL_ANGLE_STEP;
	else                    smo.ol_theta -= SMO_OL_ANGLE_STEP;
	if(smo.ol_theta >= TWO_PI) smo.ol_theta -= TWO_PI;
	if(smo.ol_theta < 0.0f)    smo.ol_theta += TWO_PI;

	/* 2) 反电动势检测 -> 切换无感 */
	if(smo.start_state == 0)
	{
		Eamp = sqrtf(smo.Ealpha_esc * smo.Ealpha_esc + smo.Ebeta_esc * smo.Ebeta_esc);
		if(Eamp > SMO_BEMF_THRESHOLD && fabsf(smo.we_est) > SMO_SWITCH_SPEED)
		{
			smo.start_state = 1;
		}
	}
	return smo.start_state;
}

/* 复位启动状态(重新启动前调用)。 */
void Smo_Startup_Reset(void)
{
	smo.start_state = 0;
	smo.ol_theta = 0.0f;
	smo.sum_pll = 0.0f;
	smo.theta_est = 0.0f;
	smo.Ialpha_esc_pre = 0.0f;
	smo.Ibeta_esc_pre = 0.0f;
}
