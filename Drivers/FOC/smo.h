#ifndef __SMO_H
#define __SMO_H

#include "./SYSTEM/sys/sys.h"
#include "bldc_control.h"

/* ==================== 低速 I/f 运行段参数(见 smo.c 顶部说明) ====================
 * 反电动势量与电压误差预算同量级时, 纯反电动势观测器给不出可用换相角。
 * SMO_IF_RPM_MAX 以下固定用"电流闭环 I/f":
 *   换相角 = 给定频率积分(同步拖动, 转速严格等于给定, 与反电动势幅值无关),
 *   电压   = iq 电流闭环(转矩恒定, 不受母线/温度/磁链误差影响),
 *   观测器 只继续跟踪, 为上面的无感段准备切换。
 * I/f 段可以一直跑到 0rpm, 也能平滑过零换向, 不会再出现
 * "观测器失锁 -> 退回强拖 -> 强拖斜坡冲到 409rpm -> 再切换 -> 再失锁" 的往复
 * (这就是原来 100rpm 以下"转一下停一下"的直接原因)。
 * 注: 修好相电阻辨识(smo.R_est)后, 无感切换点已从 164rpm 下移到约 116rpm,
 *     这里的回切点必须比切换点更低, 否则刚切过去就被判"低速"再退回来。 */
#define SMO_IF_RPM_MAX		95.0f	/* 无感段给定降到该转速以下 -> 交回 I/f 段(滞环) */
#define SMO_IF_WE_STOP		2.0f	/* 停机判定: 频率低于该值 [rad/s] */
#define SMO_IF_STOP_RPM		5		/* 停机判定: 转速低于该值 [RPM] */
/* 换向过零窗口 [rad/s]: 频率降到这个值以内才允许翻转 smo.dir。
 * 换向不是"把 dir 取反"——dir 一翻，磁场角速度就从 +w 阶跃成 -w，转差 2w，
 * 那是异步机的反接制动(plugging)，转矩方向随转子位置乱跳，电流环必然饱和，
 * 现象就是"抖一会、有时能换过去有时换不过去"。正确做法是让带符号的频率
 * 沿斜坡减到 0、过零之后再反向加速，转子同步减速->静止->反向，全程没有转差阶跃。*/
#define SMO_REV_WE_ZERO		3.0f	/* 视为"磁场已停住"的角速度门限 [rad/s], 约 4rpm */

typedef struct
{
	/* ---- alpha-beta 坐标系 ---- */
	float Ealpha_esc,Ebeta_esc; //alpha,beta轴反电动势估计，单位：V（由dq反变换得到，仅用于调试观察）
	float Ialpha_esc, Ibeta_esc;	//alpha,beta轴估计电流，单位：A
	float Ialpha_esc_pre, Ibeta_esc_pre;	//alpha,beta轴估计电流旧值，单位：A
	float Ialpha_err, Ibeta_err;	//alpha,beta轴电流误差，单位：A
	float ia,ib,ic;	//A,B,C三相实际电流，单位：A
	float Ualpha,Ubeta;	//上一拍实际施加的alpha,beta轴电压，单位：V
	float Ialpha_mea, Ibeta_mea;	//alpha,beta轴实际电流，单位：A
	float Valpha_esc,Vbeta_esc; //alpha,beta轴观测器校正量，稳态等于反电动势，单位：V

	/* ---- 旋转坐标系(dq，用 theta_est 定向) ---- */
	float Ed_f,Eq_f;	//滤波后的d,q轴反电动势，单位：V（dq下是直流量，滤波不产生相位滞后）
	float Eamp;			//反电动势幅值 |E|，单位：V（已按观测器传递函数还原）

	/* ---- PLL ---- */
	float we_est; //电角速度估计值，单位：rad/s
	float we_mea;	//实际电角速度，单位：rad/s
	float theta_err;	//角度误差值，单位：rad
	float theta_err_limit;	//限制角度误差范围
	float sum_pll;	//锁相环积分项
	float ki_pll;	//锁相环积分系数（已归一化，与|E|无关）
	float kp_pll;	//锁相环比例系数（已归一化，与|E|无关）
	float theta_mea;	//电角度测量值(编码器真值)，单位：rad。范围：0~2π
	float theta_est;	//电角度估计值，单位：rad。范围：0~2π

	/* ---- 换相角 ---- */
	int16_t theta_app_q15;	//上一拍合成三相电压时真正用过的电角度(Q15)，观测器必须用这个角做逆变换
	float lag_comp;		//换相角超前补偿量，单位：rad
	float angle_err_deg;	//换相角 - 编码器真值，单位：度（仅调试，正常应基本不随转速变化）

	float h,h1, h2;	//观测器校正量幅值（调试用）

	/* ---- 无感启动 ---- */
	uint8_t start_state;	/* 0 = 开环强拖, 1 = 已切入无感 */
	uint16_t sw_cnt;		/* 切换条件连续满足计数, 单位: 控制周期(100us) */
	uint16_t sw_win;		/* 切换判决窗口计时, 单位: 控制周期(100us) */
	uint16_t acq_div;		/* 快速定相分频计数 */
	float   dth_deg;		/* 估计角 - 强拖角, 单位: 度(切换判据的关键量, 串口监视用) */
	uint16_t ph_bad_cnt;	/* 估计角反相(相差>120度)持续计数, 用于重新捕获 */
	float   ol_theta;		/* 开环强拖电角度 [rad] */
	float   ol_we;			/* 当前强拖角速度 [rad/s]，【带符号】: 正=正转, 负=反转。
							   旧版这里是幅值, 方向另外由 dir 决定 —— 那样换向只能靠翻 dir,
							   频率必然阶跃反号; 现在频率本身带符号, 斜坡可以连续过零。*/
	int8_t  dir;			/* 锁定转向: 1正转, -1反转。只在 I/f 频率过零时翻转(见 SMO_REV_WE_ZERO),
							   保证任何时刻 sign(ol_we) == dir */
	uint16_t align_cnt;		/* 换向/重启后的定角计时, 单位: 控制周期(100us), 为0时才开始推进强拖角 */
	uint32_t start_tick;		/* 启动已耗时, 单位: 控制周期(0.1ms), 诊断+强拖电流升级用 */

	/* ---- 在线参数修正 ---- */
	float psi_k;		/* 磁链前馈修正系数: |E|/(LINKAGE*|we|)，1.0表示 LINKAGE 准确 */
	float theta_trim;	/* 换相角自校准修正量 [rad](d轴电流归零)，见 smo.c 的 SMO_TRIM */
	float R_est;		/* 相电阻在线辨识 [欧]。定角时转子不转(E=0), R=V/I 可直接测出来,
						   测到的值还包含 MOSFET 压降/死区的等效电阻 —— 正是观测器模型需要的那个 R。
						   原来写死 0.4 欧, 实际约 0.225 欧: 2A 时凭空多出 0.45V 假反电动势,
						   50rpm 的真实反电动势只有 0.077V, 直接被埋掉 —— 这是低速切换下限的根本原因 */
	uint8_t R_valid;	/* 1 = R_est 已由本次定角辨识得到, 0 = 仍是初值 */

	/* ---- 低速 I/f 运行段 ---- */
	float   if_ref_we;	/* I/f 频率给定 [rad/s]，带符号，由目标转速换算 */
	float   if_iref;	/* I/f 转矩电流给定 [A]，上电/切换时从 0 斜坡建立 */
	float   if_i;		/* I/f 电流环积分项 [V] */
	float   if_vq;		/* I/f 电流环输出电压 [V] */
	float   if_iq_f;	/* 滤波后的转矩电流 [A](已按转向折算) */
	uint8_t if_init;	/* 1 = 下一拍电流环按"上一拍电压"无扰预置积分 */
}Smo;
extern Smo smo;

void BLDC_Smo_Init(void);
void Smo_Control(uint32_t interver_us);
uint8_t Smo_Startup(void);			/* 返回1表示检测到反电动势, 可切无感 */
int16_t Smo_ElecToQ15(float theta_rad);	/* 电角度[rad] -> Q15换相角(motor_axle_angle) */
float Smo_CommTheta(void);		/* 换相角: 估计角 + 观测器滞后/延时补偿 */
int32_t Smo_GetSpeedRpm(void);		/* 观测器估计转速(RPM)，无感速度环反馈 */
int32_t Smo_BemfFfQ15(int32_t ref_rpm);	/* 反电动势电压前馈(Q15)，按给定转速计算 */
int32_t Smo_VdDecoupleQ15(int32_t iq_ma);	/* d轴解耦电压前馈(Q15)，-we*L*Iq */
int32_t Smo_StartupVqQ15(void);		/* 启动阶段(定角/强拖)施加的电压(Q15) */
uint16_t Smo_GetStartTimeMs(void);	/* 本次启动耗时(ms), 切换成功后保留最终值 */
void Smo_Startup_Reset(void);		/* 复位启动状态 */
void Smo_Startup_Restart(void);		/* 失锁后重启: 强拖角从当前估计角接续 */

/* ---- 低速 I/f 运行段 ---- */
void  Smo_Startup_SetRef(int32_t target_rpm);	/* 设置 I/f 频率给定(=目标转速) */
void  Smo_IfEnter(void);						/* 进入 I/f 段: 相位/频率/电压全部无扰接续 */
int32_t Smo_IfRun(uint32_t interver_us, int32_t iq_ma);	/* I/f 电流闭环, 返回 Vq(Q15) */
float Smo_IfTheta(void);						/* I/f 段实际换相角 [rad] */
int32_t Smo_IfSpeedRpm(void);					/* I/f 段磁场转速 [RPM]，带符号(负=反转) */
int32_t Smo_SwitchVqQ15(int32_t ref_rpm);		/* 切入无感瞬间接续的 Vq(Q15) */

#endif
