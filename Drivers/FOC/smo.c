#include "smo.h"

/* ============================================================================
 * 无感 FOC 反电动势观测器 + PLL
 *
 * 本版针对"1500rpm 以上电流变大、2000rpm 以上堵转"做了 5 处结构性修改：
 *
 * 1) 校正量由 bang-bang(h*sat) 改为比例型 V = K*e
 *    旧版 h = 1.6|E|+0.5，8000rpm 时 h≈20V，一个 100us 周期就把估计电流拉动
 *    h*ts/L = 20V*100us/156uH = 12.8A，抖振幅度远大于反电动势本身，反电动势
 *    估计被抖振淹没，高速必然失锁。比例型稳态为 V = E*K/(R+K+jwL)，没有抖振，
 *    幅值/相位都可以解析还原（见下面的 smo_obs_comp 与 Smo_CommTheta）。
 *
 * 2) 反电动势提取改为"旋转坐标系解调"
 *    用估计角把 V 投影到 dq，dq 下反电动势是直流量，低通滤波不产生相位滞后。
 *    旧版在 alpha-beta 下用 q=0.2 的低通(tau≈400us)，1500rpm(175Hz) 滞后 30 度、
 *    8000rpm(933Hz) 滞后 67 度，且滞后量随转速变化，没法用固定补偿抵消。
 *
 * 3) 换相角随转速自动补偿：atan(wL/(R+K)) + w*Td
 *    前者是观测器自身滞后，后者是电流采样/PWM更新的等效延时。
 *
 * 4) PLL 用 |E| 归一化后, 带宽再按转速调度 (wn = 0.22|we|, 60~700rad/s)。
 *    固定高带宽在低速会围着转子来回摆(实测 350rpm 估计转速摆 ±20%),
 *    切换判据永远凑不满, 一直切不进无感。
 *
 * 5) 观测器统一使用"上一拍合成电压时真正用过的角度" smo.theta_app_q15。
 *    旧版用的是本拍刚更新的换相角，两者相差 w*Ts（2000rpm 8.4度，8000rpm 33度），
 *    相当于给电压矢量附加了一个旋转误差，高速时反电动势估计直接跑偏。
 *
 * 6) 低速(140rpm 以下)不用观测器换相, 改成"电流闭环 I/f"段。
 *    反电动势 100rpm 只有 0.15V, 与电流采样误差同量级, 纯反电动势观测器在这个区间
 *    给不出可用角度。I/f 是同步拖动: 转速严格等于给定频率, 与反电动势无关, 所以低速
 *    能稳稳跑到 0rpm; 观测器只继续跟踪, 为 164rpm 以上的无感段准备切换。详见下面
 *    "启动 / 低速 I/f 段"的参数说明。
 * ========================================================================== */

#define TWO_PI			6.283185f
#define PI				3.1415926f

#define MA_TO_A			(1.0f / 1000.0f)							/* 电流 mA -> A */
#define Q15_TO_V		(M1_POWER_VOL / (1.7320508f * 32768.0f))	/* Q15电压 -> V（SVPWM: 32767 对应 Vdc/√3）*/

#define M1_PHASE_R		0.225f		/* 相电阻【初值】[欧]。运行时用 smo.R_est(定角在线辨识)覆盖,
									   只有当本次启动没走到定角(带速重启)时才用这个初值 */
#define M1_PHASE_L		0.000156f	/* 相电感 [H]，表贴式电机 Ld≈Lq */
#define LINKAGE			0.0021f		/* 永磁体磁链 [Wb] */

/* ---------- 观测器参数 ---------- */
#define SMO_K_OBS		1.0f		/* 比例增益 [V/A]。误差极点 (R+K)/L = 9.0k rad/s，
									   100us 离散极点 1-ts*(R+K)/L = +0.10，无振铃 */
/* 幅值还原与相位补偿由 smo_obs_comp() 按精确传递函数计算，不在这里给常数 */
#define SMO_V_LIMIT		16.0f		/* 校正量限幅 [V]，8000rpm 反电动势约 12.3V */

/* ---------- PLL 参数（已归一化：单位 1/s 与 1/s^2，与 |E| 无关）---------- */
#define SMO_PLL_KP		160.0f		/* 仅初值, 运行时按转速重算 */
#define SMO_PLL_KI		9000.0f		/* 仅初值, 运行时按转速重算 */
/* 环路带宽按电角速度调度: wn = SMO_PLL_KWE*|we|, 限幅[MIN,MAX]; ki=wn^2, kp=2*zeta*wn
   低速必须慢: 350rpm 时电频率只有 43Hz, wn 若高于它, 估计值会跟着电流纹波来回摆,
   切换判据 |theta_est - ol_theta| < 90 度凑不满 -> 永远切不进无感。
   高速要快: 加速时 PLL 的稳态相位滞后 = a/wn^2, wn 太小会把换相角拖晚。*/
#define SMO_PLL_KWE		0.22f
#define SMO_PLL_WN_MIN	60.0f		/* 约 9.5Hz, 400rpm 以下一直用它 */
#define SMO_PLL_WN_MAX	700.0f		/* 约 111Hz, 8000rpm(5866rad/s) 时生效 */
#define SMO_PLL_ZETA	1.15f
#define SMO_WE_LIMIT	7000.0f		/* 积分限幅 [rad/s]，约 9500rpm */
#define SMO_EAMP_FLOOR	0.10f		/* 归一化下限 [V]，防止低速除零放大噪声 */

/* ---------- 反电动势解调低通（dq 下是直流量，可以取得重一些）---------- */
/* 解调低通系数按转速调度: q = clamp(SMO_DEMOD_KWE*|we|*ts, Q_MIN, Q_MAX)
   dq 下反电动势是直流量, 滤波不产生相位滞后; 但低速时系数取大了, 电流纹波和 6 次
   谐波会直接灌进 PLL。取 q 正比于电周期, 使滤波器的相对衰减与转速无关。*/
#define SMO_DEMOD_KWE	0.35f
#define SMO_DEMOD_Q_MIN	0.015f		/* 下限: tau 约 6.7ms, fc 约 24Hz */
#define SMO_DEMOD_Q_MAX	0.20f

/* ---------- 换相角补偿 ---------- */
#define SMO_COMP_EN		1			/* 0 = 关闭补偿，用于对比测试 */
/* 等效延时 [s]。原来是 150us(按采样+滤波+PWM更新估的), 实测偏大:
   把自校准 theta_trim 在各转速收敛后的总补偿角(模型补偿+trim)反算回来,
   1000~6000rpm 依次是 4.3/6.6/8.9/11.3/13.6/18.3/22.8/27.4 度,
   与 Td=15us 的模型(4.9/7.3/9.7/12.0/14.4/19.0/23.4/27.7)只差 0.3~0.8 度,
   而 Td=150us 时偏大 2.2~13.9 度 —— 也就是说这个"延时"实际只有十几微秒:
   观测器用的是上一拍真正施加的电压角, 而电流是本拍采样, 两者部分抵消掉了滤波延时。
   Td 取大了会过补偿(换相角超前), 高速段就变成 d 轴电流。*/
#define SMO_COMP_TDLY	0.000015f	/* 等效延时 [s]，由各转速实测自校准量反算得到 */
#define SMO_COMP_GAIN	1.0f		/* 补偿增益，残余角度误差靠微调这一个系数校正 */

/* ---------- 换相角自校准（d 轴电流归零）---------- */
/* 上面的传递函数补偿只能做到"大致对": Tdly、L、R 都是估出来的，加上 100us 一拍
   的离散化误差，残余角度误差会随转速线性增长。而残余角度误差在高速会直接变成
   d 轴电流: 稳态 id = -(Vq/R)*d_theta（Vq 是施加的 q 轴电压）。R=0.4 欧、8000rpm 时
   Vq 约 12.3V，偏 1 度就是 0.5A、偏 3 度就是 1.5A 的纯 d 轴电流 —— 不产生转矩, 只吃
   铜损和电压裕量(Vq 还要多出 we*L*id)，
   这正是"1500 转以上电流变大、2000 转直接堵转"的机理。
   这里直接拿 id 做慢速积分，把换相角修正到 id≈0，等于现场自动标定换相延时，
   不依赖编码器、不依赖模型参数准不准。
   符号: 电压超前(d_theta>0) -> id<0，所以修正量变化率取 +Ki*id。*/
#define SMO_TRIM_EN		1			/* 0 = 关闭自校准，只用模型补偿 */
#define SMO_TRIM_KI		4.0f		/* 积分增益 [1/s]，闭环时间常数 0.25s */
#define SMO_TRIM_MAX	0.35f		/* 修正量限幅 [rad]，约 20 度（安全兜底；实测收敛值 <2 度）*/
#define SMO_TRIM_WMIN	300.0f		/* 估计电角速度超过该值才自校准 [rad/s]，约 410rpm */
#define SMO_TRIM_VFLOOR	1.0f		/* 电压归一化下限 [V]，防止低速把噪声放大 */
#define SMO_TRIM_DB		0.025f		/* 死区 [A]：电流采样残余零漂不应该被积分成角度 */

/* ---------- 启动 / 低速 I/f 段 ----------
   原来是"定角 -> 开环电压强拖(斜坡冲到 300rad/s=409rpm) -> 切换"。
   两个结构性问题在低速段会致命:
     1) 强拖斜坡的目标固定是 409rpm, 与给定转速无关。给定 50rpm 时它先把电机
        拖到 409rpm, 切进无感后速度环再往下降; 降到 100rpm 以下观测器锁不住,
        200ms 后判失锁 -> Smo_Startup_Restart -> 又从低速开始爬 -> 再拖到 409rpm。
        这就是"低于 100rpm 转一下停一下"的往复过程。
     2) 强拖施加 Vq 的角度比 FOC 角超前了 90 度(定角时转子 d 轴就停在强拖角+90),
        而观测器估计的是转子 d 轴。切换瞬间换相角要跳 90 度 -> 转矩瞬间反向,
        每切一次抖一下。
   现在改成:
     - "定角 + 频率斜坡"同时就是低速运行段: 频率朝【目标转速】靠拢, 不再固定冲到 409rpm;
     - 斜坡段的电压由 iq 电流闭环给(等效恒转矩拖动), 转矩不随母线/温度/磁链漂移,
       转子带载也不容易失步;
     - 换相角统一按 FOC 定义: 施加 Vq 的角度 = 强拖角 - dir*90, 使"强拖角"就是
       转子 d 轴角, 和无感段的 theta_est 同一定义, 切换时只差一个负载角(十几度);
     - 失锁/换向时不再从头重启, 而是 Smo_IfEnter(): 相位、频率、电压三者全部接续;
     - 【换向】频率 ol_we 带符号, 换向 = 让频率沿斜坡减到 0、过零后再反向加速,
       转向(dir)只在过零点翻。旧版 ol_we 是幅值、换向靠翻 dir, 等于把磁场速度
       从 +w 阶跃成 -w(反接制动, 转差 2w), 转子被冲击转矩反复打 -> "换向抖一会,
       有时才换过去"。 */
#define SMO_OL_WE_MAX		300.0f	/* I/f 频率上限 [rad/s], 约 410rpm(观测器一直不锁定时也不冲高速) */
#define SMO_OL_ACC			900.0f	/* I/f 频率斜坡角加速度 [rad/s^2], 0->300rad/s 约 0.33s */
/* 停机判定 SMO_IF_WE_STOP / SMO_IF_STOP_RPM 在 smo.h(速度环也要用) */

/* I/f 电流环. 被控对象 Vq->iq 近似一阶: 直流增益 1/R=2.5A/V, 极点 R/L=2.5krad/s。
   穿越频率约 400rad/s(64Hz): 反馈是 200us 更新一次的原始 iq 再叠 480Hz 一阶滤波,
   延时约 0.3ms, 穿越处相位裕度 > 50 度。太小会跟不上负载, 太大会被 PWM 纹波带跑。 */
/* 注意单位: I/f 电流环内部全部用【安培】。原来这里写成 2000.0f(mA), 而限幅用的是
   current_limit*0.001=5.0(A), "2000 > 5" 直接把电流给定顶成 5A —— 这一条就是实测
   电流 5.8A、电机发热、转矩冲击的来源。改成 2.0A。 */
#define SMO_IF_I_REF		2.0f	/* I/f 转矩电流给定 [A]。低速带载带不动就加大这个(上限见下面的限幅) */
#define SMO_IF_I_MAX_FRAC	0.50f	/* 电流给定不超过 current_limit 的这个比例(留余量给调节) */
#define SMO_IF_I_RAMP		40.0f	/* 电流给定斜率 [A/s], 0->2A 约 50ms(软起动, 避免电流冲击) */
#define SMO_IF_KP			0.40f	/* 电流环比例 [V/A] */
#define SMO_IF_KI			45.0f	/* 电流环积分 [V/(A*s)] */
#define SMO_IF_I_NEG		0.60f	/* 积分项允许的负向范围 [V]。
									   原来下限是 0, 前馈一旦偏大(以前 R 写死 0.4 欧, 实际 0.225),
									   积分就永远修不回来, 实测给定 2A 却稳定跑 2.6A —— 多出来的 30%
									   电流全变成铜损和噪声。现在允许积分往负走, 参数残差能被吃掉 */
#define SMO_IF_VMAX			5.0f	/* I/f 电压上限 [V], 约 12A 的堵转能力, 也留足调速余量 */
#define SMO_IF_VQ_MIN_FRAC	0.60f	/* 电压下限 = 该比例 * 反电动势: I/f 段 Vq 不能真降到 0,
									   0V 等于把磁场撤掉(转子立刻失去同步、电流环彻底失控) */
#define SMO_IF_IQ_LPF		0.30f	/* iq 反馈滤波系数(100us 一次, 约 480Hz) */

/* ---------- 定角 ---------- */
#define SMO_ALIGN_V			0.45f	/* 定角电压 [V], 约 1.1A(按实测 0.225 欧算是 2A) */
#define SMO_ALIGN_CNT		800		/* 启动/重启后定角 800 拍(80ms) */
#define SMO_ALIGN_RPM		60.0f	/* 只有 |实际转速| 低于该值才做定角 */

/* ---------- 相电阻在线辨识（借定角这一段做）----------
   定角时转子被钉住、E=0，绕组上只剩 R*I：施加已知电压、读回电流就能直接算出 R。
   测出来的值包含 MOSFET 导通压降/死区的等效电阻，正好是 dq 模型里该用的"有效电阻"，
   比 datasheet 的相电阻更适合喂给观测器和电压前馈。温度每升 100 度 R 涨 40%，
   每次启动重新测一次，自动跟随温度与母线变化。
   实测这台电机：定角 0.45V / 2.0A -> 0.225 欧；而代码原来写死 0.4 欧，
   2.6A 时就凭空多出 0.45V 假反电动势（50rpm 真实反电动势才 0.077V），
   观测器输出被模型误差彻底淹没 —— 这就是"最低只能 165rpm"的真正原因。 */
#define SMO_RID_FROM		400		/* 定角剩 400 拍(40ms)之后才开始采: 避开电流建立与转子摆动 */
#define SMO_RID_MIN_I		0.30f	/* 电流小于该值不辨识 [A]，避免除小数放大零漂 */
#define SMO_RID_LPF			0.02f	/* 辨识值滤波系数(100us 一拍, 时间常数约 5ms) */
#define SMO_RID_MIN			0.05f	/* 合法范围 [欧] */
#define SMO_RID_MAX			1.00f

/* ---------- I/f -> 无感 切换判据 ---------- */
/* 三个条件都要满足: 反电动势够大(信号可信) + 电角速度够高(PLL 能跟上) + 相位基本对齐。
   这两个门槛原来定在 164rpm, 是被【电阻参数错误】逼出来的: 代码里 R=0.4 欧, 实际约
   0.225 欧, 2A 电流就凭空多出 0.45V 假反电动势, 而 100rpm 的真实反电动势只有 0.154V
   —— 观测器输出被模型误差完全淹没, 只能靠"把转速提上去让 E 追上误差"来切换。
   现在 R 由定角在线辨识(smo.R_est), 120rpm 时 E=0.185V 已经明显大于残余误差
   (电流零漂/电阻温度漂移约 30~50mV), 所以门槛可以下移到 ~116rpm。
   再往下(80rpm 以下)就是真实物理极限了: E<0.12V, 与死区压降/采样零漂同量级。 */
#define SMO_BEMF_THRESHOLD	0.12f	/* 反电动势幅值阈值 [V]，切换无感用(约 78rpm) */
#define SMO_SWITCH_SPEED	85.0f	/* 估计电角速度阈值 [rad/s], 约 116rpm */
#define SMO_SWITCH_PHASE	1.222f	/* 强拖角与估计角相差 < 70 度才允许切换(相位不能差太多,
									   离失步点 90 度仍有余量; 原来 60 度太紧, 实测长期卡在 60~65 度) */
#define SMO_SWITCH_CONFIRM	80		/* 判决窗口内至少 80 拍(8ms)满足才切换 */
#define SMO_SWITCH_WINDOW		100		/* 判决窗口长度 100 拍 = 10ms(原 20ms) */
#define SMO_SWITCH_I_KEEP	0.6f	/* 切入无感瞬间保留的转矩电流 [A], 见 Smo_SwitchVqQ15 */
#define SMO_PHASE_BAD		2.094f	/* 相差 > 120 度判定 PLL 收敛到反相点 */
#define SMO_PHASE_BAD_CNT	1000	/* 反相持续 1000 拍(100ms) 后重新捕获 */
#define SMO_ALIGN_LAG		0.35f	/* 重新捕获时估计角相对强拖角回退约 20 度 */
/* 强拖阶段的快速定相: 反电动势矢量角 = 转子角 + dir*90 度, 所以
   转子角 = atan2(Ebeta, Ealpha) - dir*90 度, 直接算出来比等 PLL 收敛快得多。*/
#define SMO_ACQ_EAMP	0.20f		/* |V| 超过该值才定相 [V]，对应 |E| 约 0.28V */
#define SMO_ACQ_DIFF	1.047f		/* 偏差超过 60 度就直接拉过去 */
#define SMO_ACQ_PULL	0.05f		/* 偏差不大时轻微牵引(缰绳作用), 主要靠 PLL */
#define SMO_ACQ_DIV		4			/* 每 4 拍(400us)定相一次 */
#define SMO_TS_PERIOD		0.0001f	/* 强拖角推进周期 [s]，与 MotorHandler 控制周期一致 */

Smo smo;

/**
 * @brief  电角度[rad] -> 查表用的 Q15 角度
 */
static int16_t smo_rad_to_q15(float th)
{
	int32_t a = (int32_t)(th * (65536.0f / TWO_PI)) & 0xFFFF;
	if(a >= 32768) a -= 65536;
	return (int16_t)a;
}

/**
 * @brief  查表求 sin/cos（比 sinf/cosf 快很多，中断里每拍都要用）
 */
static void smo_sincos(float th, float *s, float *c)
{
	Trig_Components t = MCM_Trig_Functions(smo_rad_to_q15(th));
	*s = (float)t.hSin * (1.0f / 32768.0f);
	*c = (float)t.hCos * (1.0f / 32768.0f);
}

/**
 * @brief  观测器(+电流反馈延时)的传递函数补偿量
 * @param  we    估计电角速度 [rad/s]
 * @param  lag   输出：反电动势估计相对真实反电动势的相位滞后 [rad]（带符号）
 * @param  scale 输出：幅值还原系数，|E| = |V| * scale
 * @note   电流反馈有 Td 的延时(采样+滑动平均+PWM更新)，因此
 *             V/E = K*e^(-jwTd) / (R + K*e^(-jwTd) + jwL)
 *         展开后：相位滞后 = w*Td + atan2(wL - K*sin(w*Td), R + K*cos(w*Td))
 *                 幅值比   = K / sqrt((R+K*cos)^2 + (wL - K*sin)^2)
 *         低速时退化成 atan(wL/(R+K)) + w*Td；高速时两者差别很大
 *         (8000rpm 精确式 36 度，而按 atan(wL/R) 近似算会到 66 度)，
 *         所以直接按精确式算，避免过补偿。
 *         延时项与电感项在 wL ≈ K*wTd 附近会互相抵消，这也是为什么
 *         只补 atan(wL/(R+K)) 是不够的。
 */
static void smo_obs_comp(float we, float *lag, float *scale)
{
	float wt = fabsf(we);
	float a  = wt * SMO_COMP_TDLY;		/* 延时对应的电角度 [rad] */
	float sn, cs, re, im, den;

	smo_sincos(a, &sn, &cs);
	re = smo.R_est + SMO_K_OBS * cs;
	im = wt * M1_PHASE_L - SMO_K_OBS * sn;
	den = sqrtf(re * re + im * im);

	*scale = den / SMO_K_OBS;
	*lag = a + atan2f(im, re);
	if(we < 0.0f) *lag = -*lag;
}

/**
 * @brief  观测器初始化
 */
void BLDC_Smo_Init(void)
{
	smo.kp_pll = SMO_PLL_KP;
	smo.ki_pll = SMO_PLL_KI;
	smo.theta_err_limit = 1.047197f;	/* 角度误差限幅 60 度 */
	smo.h = 0.0f;
	smo.dir = (OPEN_LOOP_VOL >= 0) ? 1 : -1;	/* 锁定转向，由强拖电压符号决定 */
	smo.align_cnt = SMO_ALIGN_CNT;	/* 上电后第一次启动也先定角 */
	smo.start_tick = 0;
	smo.ol_we = 0.0f;				/* I/f 频率从 0 起步, 由斜坡升到给定 */
	smo.psi_k = 1.0f;
	smo.theta_trim = 0.0f;
	smo.R_est = M1_PHASE_R;		/* 还没辨识, 先用初值; 第一次定角后会覆盖 */
	smo.R_valid = 0;
	smo.theta_app_q15 = 0;
	smo.lag_comp = 0.0f;
	smo.Eamp = 0.0f;
	smo.Ed_f = 0.0f;
	smo.Eq_f = 0.0f;
	smo.if_ref_we = 0.0f;
	smo.if_iref = 0.0f;
	smo.if_i = 0.0f;
	smo.if_vq = 0.0f;
	smo.if_iq_f = 0.0f;
	smo.if_init = 1;
}

/**
 * @brief  反电动势观测器主函数，每 100us 调用一次
 * @param  interver_us 调用周期，单位：us
 * @note   输入：上一拍施加的电压(out_vq/out_vd 与 theta_app_q15)、本拍采样电流
 *         输出：smo.theta_est(估计电角度)、smo.we_est(估计电角速度)、smo.Eamp(反电动势幅值)
 */
void Smo_Control(uint32_t interver_us)
{
	float ts, s, c, ed, eq, emag, scale, we_abs;
	RevPark rp;
	Clarkef cl;

	if(interver_us == 0) interver_us = 100;
	ts = (float)interver_us * 1e-6f;

	/* 调试用真值（磁编码器）：只做比对，不参与控制 */
	smo.we_mea = (float)motor1.mea_speed * (PI / 30.0f) * (float)M1_POLE_PAIRS;
	smo.theta_mea = (float)(motor1.ele_angle % (int32_t)ENCODER_FACTOR) * (TWO_PI / (float)ENCODER_FACTOR);

	/* ---------- 1. 输入：上一拍施加的电压 + 本拍采样电流 ---------- */
	/* 注意：必须用上一拍合成电压时的角度，不能用本拍刚更新的换相角 */
	rp = RevPark_Transform15(motor1.out_vq, motor1.out_vd, smo.theta_app_q15);
	smo.Ualpha = (float)rp.ualpha * Q15_TO_V;
	smo.Ubeta  = (float)rp.ubeta  * Q15_TO_V;

	cl = Clarke_Transform(motor1.mea_ia * MA_TO_A, motor1.mea_ib * MA_TO_A);
	smo.Ialpha_mea = cl.ialpha;
	smo.Ibeta_mea  = cl.ibeta;

	/* ---------- 2. 电流观测器（一阶线性）----------
	   L dIe/dt = U - R*Ie - V ，V = K*(Ie - I)
	   误差动态 L de/dt = -(R+K)e + E，稳态 V = E*K/(R+K+jwL) */
	/* 模型里的 R 必须用辨识值: 用错 1.8 倍, 2.6A 时就是 0.45V 的假反电动势,
	   低速下它比真实反电动势大 6 倍, 观测器根本锁不住 */
	smo.Ialpha_esc = smo.Ialpha_esc_pre + (ts / M1_PHASE_L) * \
					(-smo.R_est * smo.Ialpha_esc_pre + smo.Ualpha - smo.Valpha_esc);
	smo.Ibeta_esc = smo.Ibeta_esc_pre + (ts / M1_PHASE_L) * \
					(-smo.R_est * smo.Ibeta_esc_pre + smo.Ubeta - smo.Vbeta_esc);
	_constrain(smo.Ialpha_esc, -60.0f, 60.0f);
	_constrain(smo.Ibeta_esc, -60.0f, 60.0f);
	smo.Ialpha_esc_pre = smo.Ialpha_esc;
	smo.Ibeta_esc_pre  = smo.Ibeta_esc;

	smo.Ialpha_err = smo.Ialpha_esc - smo.Ialpha_mea;
	smo.Ibeta_err  = smo.Ibeta_esc  - smo.Ibeta_mea;

	smo.Valpha_esc = SMO_K_OBS * smo.Ialpha_err;
	smo.Vbeta_esc  = SMO_K_OBS * smo.Ibeta_err;
	_constrain(smo.Valpha_esc, -SMO_V_LIMIT, SMO_V_LIMIT);
	_constrain(smo.Vbeta_esc,  -SMO_V_LIMIT, SMO_V_LIMIT);

	/* ---------- 3. 旋转坐标系解调 ----------
	   ed/eq 在 dq 下是直流量，低通滤波不会引入随转速变化的相位滞后 */
	smo_sincos(smo.theta_est, &s, &c);
	ed =  smo.Valpha_esc * c + smo.Vbeta_esc * s;
	eq = -smo.Valpha_esc * s + smo.Vbeta_esc * c;
	{
		/* 系数随转速调度, 说明见 SMO_DEMOD_KWE */
		float qd = SMO_DEMOD_KWE * fabsf(smo.we_est) * ts;
		if(qd < SMO_DEMOD_Q_MIN) qd = SMO_DEMOD_Q_MIN;
		if(qd > SMO_DEMOD_Q_MAX) qd = SMO_DEMOD_Q_MAX;
		smo.Ed_f += qd * (ed - smo.Ed_f);
		smo.Eq_f += qd * (eq - smo.Eq_f);
	}

	smo.Ealpha_esc = smo.Ed_f * c - smo.Eq_f * s;	/* 反变换回 alpha-beta，调试观察用 */
	smo.Ebeta_esc  = smo.Ed_f * s + smo.Eq_f * c;

	emag = sqrtf(smo.Ed_f * smo.Ed_f + smo.Eq_f * smo.Eq_f);	/* 校正量幅值 |V| */
	smo.h = emag;

	/* 幅值还原 + 换相角补偿量（观测器与电流反馈延时的精确补偿，见 smo_obs_comp） */
	we_abs = fabsf(smo.we_est);
	smo_obs_comp(smo.we_est, &smo.lag_comp, &scale);
	smo.Eamp = emag * scale;
	smo.lag_comp = (SMO_COMP_EN) ? (smo.lag_comp * SMO_COMP_GAIN) : 0.0f;

	/* ---------- 3b. 换相角自校准：把 d 轴电流积到 0 ----------
	   id 在估计坐标系下测量，稳态 id = -(Vq/R)*d_theta，所以 id*R/|Vq| 就是负的
	   角度误差(标幺值)。归一化后环路增益与转速、电压无关，时间常数恒为 1/KI。
	   只在已切入无感、正转、且反电动势够大(信号可信)时才积分。 */
#if SMO_TRIM_EN
	if(smo.start_state == 1 && smo.dir > 0 && smo.we_est > SMO_TRIM_WMIN)
	{
		float id_a = (float)motor1.mea_id * MA_TO_A;
		float vabs = fabsf((float)motor1.out_vq) * Q15_TO_V;
		if(vabs < SMO_TRIM_VFLOOR) vabs = SMO_TRIM_VFLOOR;
		/* 死区: 电流零点残余误差(实测十几毫安)如果被积分, 会以每秒一度多的速度
		   把换相角慢慢带偏, 所以先扣掉死区再积分 */
		if(id_a >  SMO_TRIM_DB)      id_a -= SMO_TRIM_DB;
		else if(id_a < -SMO_TRIM_DB) id_a += SMO_TRIM_DB;
		else                         id_a  = 0.0f;
		smo.theta_trim += SMO_TRIM_KI * (id_a * smo.R_est / vabs) * ts;
		_constrain(smo.theta_trim, -SMO_TRIM_MAX, SMO_TRIM_MAX);
	}
#endif

	/* ---------- 4. PLL ----------
	   ed 归一化后就是角度误差的正弦值，环路增益与 |E| 无关。
	   转向由 smo.dir 决定：正转时 E 矢量超前转子 90 度，反转时滞后 90 度，
	   两种情况的误差极性相反，所以要乘 dir。 */
	smo.theta_err = -(smo.Ed_f) / (emag + SMO_EAMP_FLOOR) * (float)smo.dir;
	_constrain(smo.theta_err, -smo.theta_err_limit, smo.theta_err_limit);
	{
		/* |E| 归一化: 环路增益与幅值无关; 带宽再按转速调度(见 SMO_PLL_KWE) */
		float g  = emag / (emag + SMO_EAMP_FLOOR);
		float wn = SMO_PLL_KWE * we_abs;
		if(wn < SMO_PLL_WN_MIN) wn = SMO_PLL_WN_MIN;
		if(wn > SMO_PLL_WN_MAX) wn = SMO_PLL_WN_MAX;
		smo.ki_pll = wn * wn;
		smo.kp_pll = 2.0f * SMO_PLL_ZETA * wn;
		smo.sum_pll += g * smo.ki_pll * smo.theta_err * ts;
		_constrain(smo.sum_pll, -SMO_WE_LIMIT, SMO_WE_LIMIT);
		smo.we_est = smo.sum_pll + g * smo.kp_pll * smo.theta_err;
	}

	smo.theta_est += smo.we_est * ts;
	while(smo.theta_est >= TWO_PI) smo.theta_est -= TWO_PI;
	while(smo.theta_est < 0.0f)    smo.theta_est += TWO_PI;

	/* ---------- 4b. 强拖阶段快速定相 ----------
	   反电动势矢量角 = 转子角 + dir*90 度 => 转子角 = atan2(Eb,Ea) - dir*90 度。
	   强拖时用这个角把 theta_est 拉正, 比等 PLL 从任意相位收敛快得多;
	   也避免 PLL 低速摆动把切换判据一直卡住。切进无感后不再干预。 */
	if(smo.start_state == 0 && emag > SMO_ACQ_EAMP)
	{
		if(++smo.acq_div >= SMO_ACQ_DIV)
		{
			float thE, d;
			smo.acq_div = 0;
			thE = atan2f(smo.Ebeta_esc, smo.Ealpha_esc) - (float)smo.dir * (PI * 0.5f);
			while(thE < 0.0f)    thE += TWO_PI;
			while(thE >= TWO_PI) thE -= TWO_PI;
			d = thE - smo.theta_est;
			while(d > PI)  d -= TWO_PI;
			while(d < -PI) d += TWO_PI;
			if(fabsf(d) > SMO_ACQ_DIFF)
			{
				smo.theta_est = thE;        	/* 差太多: 直接拉过去 */
				smo.sum_pll   = smo.we_est;  	/* 清掉旧的误差积分, 别继续冲 */
			}
			else
			{
				smo.theta_est += SMO_ACQ_PULL * d;
			}
			while(smo.theta_est < 0.0f)    smo.theta_est += TWO_PI;
			while(smo.theta_est >= TWO_PI) smo.theta_est -= TWO_PI;
		}
	}
	else
	{
		smo.acq_div = 0;
	}

	/* ---------- 5. 前馈磁链在线修正 ----------
	   psi_k = |E|/(LINKAGE*|we|)：用实测反电动势校正 LINKAGE 偏差，
	   避免磁链参数不准把电压前馈顶到限幅（高速时前馈几乎占满整个母线电压） */
	if(smo.start_state == 1 && we_abs > 300.0f)
	{
		float k = smo.Eamp / (LINKAGE * we_abs);
		_constrain(k, 0.7f, 1.4f);
		smo.psi_k += 0.0002f * (k - smo.psi_k);
	}

	/* ---------- 6. 记录本拍的真实电压角，下一拍观测器要用 ---------- */
	smo.theta_app_q15 = motor1.motor_axle_angle;

	/* ---------- 7. 调试：换相角与编码器真值之差 ---------- */
	{
		float d = smo.theta_est + smo.lag_comp - smo.theta_mea;
		while(d > PI)  d -= TWO_PI;
		while(d < -PI) d += TWO_PI;
		smo.angle_err_deg = d * (180.0f / PI);
	}
}

/**
 * @brief  换相角：估计角 + 观测器滞后/延时补偿
 * @note   观测器自身：V/E = K/(R+K+jwL)，滞后 atan(wL/(R+K))
 *         采样延时：电流在 PWM 谷点采样、下一拍才更新占空比，加上电流滑动平均，
 *         电压真正作用到绕组的时刻比估计角滞后约 Td（见 SMO_COMP_TDLY）
 *         两项都随转速增长：2000rpm 约 22 度，8000rpm 约 84 度（w*Td 占大头）。
 *         不补偿的后果就是高速换相角偏晚、转矩角接近 90 度、电流暴涨直到堵转。
 *         补偿量是否合适可以直接看串口6 打印的第 9 列（换相角-编码器真值）：
 *         它应该基本不随转速变化；若随转速单调漂移，微调 SMO_COMP_GAIN 即可。
 * @return 换相用电角度 [rad]，范围 0~2π
 */
float Smo_CommTheta(void)
{
	/* smo.lag_comp 已在 Smo_Control 里按当前转速算好（含 SMO_COMP_EN / SMO_COMP_GAIN） */
	float th = smo.theta_est + smo.lag_comp + (float)smo.dir * smo.theta_trim;

	while(th >= TWO_PI) th -= TWO_PI;
	while(th < 0.0f)    th += TWO_PI;
	return th;
}

/**
 * @brief  电角度[rad] 转 Q15 角度（motor_axle_angle）
 * @note   Q15 一圈 = 65536（0~32767 对应 0~pi，-32768~-1 对应 pi~2pi）
 */
int16_t Smo_ElecToQ15(float theta_rad)
{
	int32_t a = (int32_t)(theta_rad * (65536.0f / TWO_PI)) & 0xFFFF;
	if(a >= 32768) a -= 65536;
	return (int16_t)a;
}

/**
 * @brief  观测器估计的机械转速
 * @return 转速 [RPM]
 */
int32_t Smo_GetSpeedRpm(void)
{
	return (int32_t)(smo.we_est * (60.0f / (TWO_PI * (float)M1_POLE_PAIRS)));
}

/**
 * @brief  反电动势电压前馈（Q15），按给定转速计算
 * @param  ref_rpm 给定机械转速 [RPM]
 * @return q 轴电压前馈，Q15
 * @note   用给定转速而不是实测转速：给定为 0 时前馈也归零，停机时电压才能退下去；
 *         加速时前馈先抬起来，PI 只补电阻压降/负载，不会掉速。
 *         psi_k 是磁链在线修正系数，用于消除 LINKAGE 参数偏差。
 */
int32_t Smo_BemfFfQ15(int32_t ref_rpm)
{
	float we_ref = (float)ref_rpm * (TWO_PI * (float)M1_POLE_PAIRS / 60.0f);
	return (int32_t)(LINKAGE * smo.psi_k * we_ref / Q15_TO_V);
}

/**
 * @brief  d 轴解耦电压前馈 Vd = -we*L*Iq（Q15）
 * @param  iq_ma q 轴电流，单位：mA（带符号）
 * @return d 轴电压前馈，Q15
 * @note   电机 dq 模型: Vd = R*Id + L*dId/dt - we*L*Iq，Id 控制在 0 时 Vd_ff = -we*L*Iq。
 *         2000rpm/5A 时约 1.1V，8000rpm/3A 时约 2.7V，不补会让电流矢量偏到 d 轴。
 */
int32_t Smo_VdDecoupleQ15(int32_t iq_ma)
{
	float v = -(smo.we_est * M1_PHASE_L) * ((float)iq_ma * MA_TO_A) / Q15_TO_V;
	_constrain(v, -8000.0f, 8000.0f);
	return (int32_t)v;
}

/**
 * @brief  定角阶段应该施加的电压 [Q15]
 * @note   只有定角(align)阶段用固定电压: 磁场不转, 电流只受相电阻限制, 小电压即可。
 *         定角结束后进入 I/f 频率斜坡, 电压改由 Smo_IfRun() 的电流闭环给 ——
 *         恒压拖动在低速不可靠(R 随温度变、母线会掉), 恒流拖动转矩才是恒定的。
 */
int32_t Smo_StartupVqQ15(void)
{
	return (int32_t)(SMO_ALIGN_V / Q15_TO_V);
}

/**
 * @brief  切入无感瞬间要接续的 q 轴电压 [Q15]
 * @param  ref_rpm 切换瞬间的速度给定(斜坡后的估计转速) [RPM]
 * @return q 轴电压 [Q15]
 * @note   = 反电动势前馈 + 一点点转矩电流(R*SMO_SWITCH_I_KEEP, 按转向取符号)。
 *         不能直接接续 I/f 段的电压: I/f 段那 2A 电流全在 d 轴上(几乎不产生转矩),
 *         切到无感后电流矢量立刻转到 q 轴, 同样的电压就变成满载转矩 -> 电机"冲一下"。
 *         也不能让电流塌到 0: 那样切换后转矩为 0, 会被负载拖慢, 转速掉下去又判失锁。
 *         所以留 0.6A: 转矩只有满载的 30%, 冲击小, 又足以维持转速。
 */
int32_t Smo_SwitchVqQ15(int32_t ref_rpm)
{
	float v = (float)Smo_BemfFfQ15(ref_rpm);

	v += (float)smo.dir * SMO_SWITCH_I_KEEP * smo.R_est;
	return (int32_t)(v / Q15_TO_V);
}

/**
 * @brief  设置 I/f 频率给定(=目标转速)
 * @param  target_rpm 目标机械转速 [RPM]，带符号
 * @note   频率斜坡朝这个值靠拢，ol_we 也带符号。换向时不需要(也不允许)翻 smo.dir:
 *         给定符号一变，斜坡先把频率减到 0、再反向加速；Smo_Startup 在过零点
 *         翻转向并补偿 180 度相位，磁场速度与换相角全程连续。
 *         如果直接把 dir 取反(旧做法)，磁场速度会阶跃 2w，就是反接制动。
 */
void Smo_Startup_SetRef(int32_t target_rpm)
{
	smo.if_ref_we = (float)target_rpm * (TWO_PI * (float)M1_POLE_PAIRS / 60.0f);
}

/**
 * @brief  I/f 段实际换相角(施加 Vq 的角度)
 * @return 电角度 [rad]，0~2π
 * @note   为什么是 -dir*90: RevPark 把 uq 放在 "θ_app+90"(uq>0) 或 "θ_app-90"(uq<0),
 *         而 uq = dir*vq 随转向变号, 两者代进去后【实际施加的电流矢量角恒等于
 *         ol_theta, 与转向无关】:
 *             dir=+1: θ_app = ol_theta-90, uq=+v -> 矢量角 = θ_app+90 = ol_theta
 *             dir=-1: θ_app = ol_theta+90, uq=-v -> 矢量角 = θ_app-90 = ol_theta
 *         转子在 I/f 同步拖动时贴着电流矢量(只差一个负载角), 所以 ol_theta 就是
 *         转子角, 与无感段的 theta_est 同一定义, 切换时只差一个负载角。
 *         旧版直接拿 ol_theta 当换相角, 矢量落在 ol_theta+90 -> 切无感时换相角跳
 *         90 度, 转矩瞬间反向, 每切换一次抖一下。
 */
float Smo_IfTheta(void)
{
	float th = smo.ol_theta - (float)smo.dir * (PI * 0.5f);

	while(th >= TWO_PI) th -= TWO_PI;
	while(th < 0.0f)    th += TWO_PI;
	return th;
}

/**
 * @brief  I/f 段磁场转速 [RPM]
 * @note   同步拖动时转子转速就等于磁场频率，所以这个值既是"给定"也是"实际转速"，
 *         堵转检测/停机判定都用它(低速段观测器和编码器都不可靠)。
 *         【带符号】: 负值表示反转。旧版返回幅值, 反转时 est_speed 恒为正,
 *         所有按符号判方向的地方(失锁判据/调试列/速度环反馈)都会判错。
 */
int32_t Smo_IfSpeedRpm(void)
{
	return (int32_t)(smo.ol_we * (60.0f / (TWO_PI * (float)M1_POLE_PAIRS)));
}

/**
 * @brief  进入低速 I/f 段：相位/频率/电压三者无扰接续
 * @note   失锁、换向、给定降到低速时都走这里，取代原来"从头重启强拖"的做法：
 *         1) 相位: 换相角从当前正在用的角度接续(+90 度换算成强拖角)，换相不跳变；
 *         2) 频率: 观测器可信时接到转子当前转速(带符号)，否则保留原值；一律不取
 *            给定值 —— 换向要的是"先减速到 0 再反向"，直接取给定就是磁场阶跃反号;
 *         3) 电压: 电流环积分按上一拍电压预置，Vq 不跳变；电流给定与反馈不清零
 *            (转子还在转, 清掉就没有制动转矩了)。
 *         观测器状态(sum_pll/theta_est)不清零，随时可以重新接管。
 */
void Smo_IfEnter(void)
{
	uint8_t trusted;

	/* 交接前就判断观测器这一拍是否可信: 刚从无感段过来(start_state==1)时
	   we_est 一定有效; 否则要反电动势足够大才算数 */
	trusted = (smo.start_state == 1) || (smo.Eamp > SMO_BEMF_THRESHOLD);

	smo.start_state = 0;
	smo.sw_cnt = 0;
	smo.sw_win = 0;
	smo.acq_div = 0;
	smo.ph_bad_cnt = 0;
	smo.start_tick = 0;
	/* 定角: 带速交回 I/f 段时绝不定角(定角等于反向制动), 所以原来是无条件清零。
	   但上电第一拍(field 还没转过、转子静止)正相反 —— 那时必须定角, 否则磁场
	   一上来就从随机位置猛拖, 同时相电阻也辨识不了(R_est 一直是初值)。
	   判据: 频率≈0 且 转子≈停 且 观测器速度≈0 -> 保留 reset 时给的 align_cnt。 */
	if(!(fabsf(smo.ol_we) < 1.0f &&
	     fabsf((float)motor1.mea_speed) < SMO_ALIGN_RPM &&
	     fabsf(smo.we_est) < 5.0f))
	{
		smo.align_cnt = 0;
	}

	smo.ol_theta = Smo_CommTheta() + (float)smo.dir * (PI * 0.5f);
	while(smo.ol_theta >= TWO_PI) smo.ol_theta -= TWO_PI;
	while(smo.ol_theta < 0.0f)    smo.ol_theta += TWO_PI;

	/* 频率接续规则(ol_we 带符号, 负号 = 反转):
	     1) 换向请求(给定与当前转向相反): 绝不在这里把频率设成"新给定的幅值"。
	        旧版就是那么干的, 换向瞬间磁场速度从 +w 阶跃成 -w, 阶跃 2w, 只能靠
	        冲击转矩去反拖转子 —— 这就是"抖一会、有时才换得过去"的主因。
	        现在只把频率接到"转子当前转速"(同向), 再交给斜坡把它减到 0、过零、反向。
	     2) 同向交接(无感->I/f, 或失锁恢复): 从转子当前转速接续, 否则磁场比转子
	        慢一大截, 交接瞬间就是一次反拖。
	     两种情况的共同约束: 只采用与当前转向同号的 we_est, 保证 sign(ol_we)==dir
	     这个不变量(换相角的 dir*90 偏置依赖它), 且不大于 3 倍频率上限(防 PLL 野值)。 */
	if(trusted)
	{
		float wm = smo.we_est;

		if((((float)smo.dir * wm) > 0.0f) &&
		   (fabsf(wm) > fabsf(smo.ol_we)) &&
		   (fabsf(wm) < (3.0f * SMO_OL_WE_MAX)))
		{
			smo.ol_we = wm;
		}
	}
	if(smo.ol_we >  SMO_OL_WE_MAX) smo.ol_we =  SMO_OL_WE_MAX;
	if(smo.ol_we < -SMO_OL_WE_MAX) smo.ol_we = -SMO_OL_WE_MAX;

	/* 电流给定与电流反馈滤波器都不清零: 换向/失锁时转子还带着转速, 电流一塌
	   就没有制动转矩, 转子自由滑行, 磁场却在别处转 —— 能否重新同步全靠运气,
	   这就是"概率能换过去"的另一半原因。只有电压积分需要按上一拍电压重预置。 */
	smo.if_init = 1;
}

/**
 * @brief  低速 I/f 电流闭环：把转矩电流稳到给定值，返回 q 轴电压
 * @param  interver_us 调用周期 [us]
 * @param  iq_ma 本拍 q 轴电流(未滤波)，单位：mA
 * @return q 轴电压 [Q15]，已按转向取符号
 * @note   I/f 段磁场按给定频率旋转，转矩 = Kt*|I|*sin(负载角)，所以只要把电流稳住，
 *         转矩就是恒定的，转子不会因为齿槽/负载突变而失步。电压前馈按
 *         "相电阻压降 + 反电动势"给，PI 只补模型误差，低速时输出基本贴着反电动势走。
 */
int32_t Smo_IfRun(uint32_t interver_us, int32_t iq_ma)
{
	float ts, iref, iq, err, ff, vq, vmin;

	if(interver_us == 0) interver_us = 100;
	ts = (float)interver_us * 1e-6f;

	/* 1) 电流给定: 从 0 按 SMO_IF_I_RAMP 爬升(软起动), 上限受电机电流限制。
	   单位统一成【A】: SMO_IF_I_REF=2.0A, current_limit 是 mA 要 *0.001 换过来,
	   两边都换成 A 之后再比较, 否则 2000 和 5 直接比会把给定顶到 5A。 */
	iref = SMO_IF_I_REF;
	{
		float lim = (float)motor1.current_limit * 0.001f * SMO_IF_I_MAX_FRAC;
		if(lim < 0.2f) lim = 0.2f;			/* current_limit 没初始化时给个保底 */
		if(iref > lim) iref = lim;
		if(iref < 0.0f) iref = 0.0f;
	}
	if(smo.if_iref < iref)
	{
		smo.if_iref += SMO_IF_I_RAMP * ts;
		if(smo.if_iref > iref) smo.if_iref = iref;
	}
	else if(smo.if_iref > iref)
	{
		smo.if_iref -= SMO_IF_I_RAMP * ts;
		if(smo.if_iref < iref) smo.if_iref = iref;
	}
	iref = smo.if_iref;

	/* 2) 反馈: 按转向折算的转矩电流(反转时为负), 一阶滤波抑制 PWM 纹波 */
	iq = (float)iq_ma * (1.0f / 1000.0f) * (float)smo.dir;
	smo.if_iq_f += SMO_IF_IQ_LPF * (iq - smo.if_iq_f);
	iq = smo.if_iq_f;

	/* 3) 前馈 + PI */
	/* psi_k 是观测器在线辨识出来的, 极端情况下(除零)可能是 NaN/inf。它是 Vq 前馈的
	   乘数, 一旦坏了, vq 全是 NaN, 而 (int32_t)NaN = 0 —— 现象就是 out_vq 恒 0、
	   电机完全不出力、电流环积分也一起变 NaN。所以先做一次自愈。 */
	if(!(smo.psi_k == smo.psi_k) || fabsf(smo.psi_k) > 10.0f)
	{
		smo.psi_k = 1.0f;
	}
	ff  = smo.R_est * iref + LINKAGE * smo.psi_k * fabsf(smo.ol_we);
	/* 电压下限: I/f 段的 Vq 不只是"转矩电压", 它同时是维持磁场的电压。Vq 掉到反电动势
	   以下, 电流就反向、磁场被撤掉, 转子立刻失步, 电流环再也调不回来。所以给一个
	   "至少撑住 SMO_IF_VQ_MIN_FRAC 倍反电动势"的下限。 */
	vmin = SMO_IF_VQ_MIN_FRAC * LINKAGE * smo.psi_k * fabsf(smo.ol_we);
	if(vmin > SMO_IF_VMAX) vmin = SMO_IF_VMAX;

	err = iref - iq;

	if(smo.if_init)					/* 无扰接管: 积分预置成"上一拍真正加过的电压" */
	{
		smo.if_i = fabsf((float)motor1.out_vq) * Q15_TO_V - ff - SMO_IF_KP * err;
		_constrain(smo.if_i, -SMO_IF_I_NEG, SMO_IF_VMAX);
		smo.if_init = 0;
	}

	vq = ff + SMO_IF_KP * err + smo.if_i;
	if(!(vq == vq))					/* NaN 自愈: 清积分重新起 */
	{
		smo.if_i = 0.0f;
		smo.if_iq_f = 0.0f;
		vq = ff;
	}
	if(vq > SMO_IF_VMAX) vq = SMO_IF_VMAX;
	if(vq < vmin)        vq = vmin;		/* 下限不是 0: 详见上面 vmin 的说明 */
	/* 输出饱和时停止积分(上限饱和 或 下限饱和) */
	if(!((vq >= SMO_IF_VMAX && err > 0.0f) || (vq <= vmin && err < 0.0f)))
	{
		smo.if_i += SMO_IF_KI * err * ts;
	}
	_constrain(smo.if_i, -SMO_IF_I_NEG, SMO_IF_VMAX);

	smo.if_vq = vq;
	return (int32_t)((float)smo.dir * vq / Q15_TO_V);
}

/**
 * @brief  开环强拖 + 切换判据
 * @return 1 = 已切入无感，0 = 仍在强拖
 */
uint8_t Smo_Startup(void)
{
	float dth, th_if;

	/* 启动耗时统计(每拍 0.1ms): 诊断用, 同时作为强拖电流升级的判据。
	   切换成功后不再累加, 保留最终值, 由 Smo_GetStartTimeMs() 读出 */
	if(smo.start_state == 0)
	{
		smo.start_tick++;
	}

	/* 0) 定角：只有转子基本停住时才做
	   把电角度钉住、施加 d 轴电压，转子被拉到定位点后自然停住；转子还在转的时候
	   这等于给了一个反向制动，十几安培的电流可能把驱动打坏。 */
	if(smo.align_cnt > 0)
	{
		if(fabsf((float)motor1.mea_speed) < SMO_ALIGN_RPM)
		{
			/* 顺带辨识相电阻: 转子被钉住 -> E=0 -> 绕组上只剩 R*I。
			   取 dq 电流的模值, 与定角角/编码器角度是否对齐无关(旋转不改变矢量长度)。
			   只采定角最后 40ms, 避开电流建立与转子吸附过程中的反电动势。 */
			if(smo.align_cnt < SMO_RID_FROM)
			{
				float id_a = (float)motor1.mea_id * 0.001f;
				float iq_a = (float)motor1.mea_iq * 0.001f;
				float im   = sqrtf(id_a * id_a + iq_a * iq_a);

				if(im > SMO_RID_MIN_I)
				{
					float r = SMO_ALIGN_V / im;
					if(r > SMO_RID_MIN && r < SMO_RID_MAX)
					{
						if(smo.R_valid == 0)
						{
							smo.R_est   = r;	/* 第一次直接取值, 后面再滤波 */
							smo.R_valid = 1;
						}
						else
						{
							smo.R_est += SMO_RID_LPF * (r - smo.R_est);
						}
					}
				}
			}

			if(--smo.align_cnt == 0u)
			{
				/* 定角结束 -> I/f 频率斜坡从 0 起步。
				   电流给定直接接到"定角电流"(= 定角电压/相电阻), 电流环积分再按定角电压
				   预置, 这样从"定角"过渡到"电流闭环"时电压和电流都不跳变。 */
				smo.ol_we = 0.0f;
				smo.if_iref = SMO_ALIGN_V / smo.R_est;
				smo.if_init = 1;
			}
			return 0;
		}
		/* 转子还在转: 放弃定角, 直接从当前频率接续 */
		smo.align_cnt = 0;
		smo.ol_we = 0.0f;
		smo.if_iref = 0.0f;
		smo.if_init = 1;
	}

	/* 1) 频率斜坡(I/f): 角速度朝给定频率 if_ref_we 平滑靠拢
	   与旧版的关键区别: 斜坡目标是【给定转速】, 不再是固定 300rad/s。
	   给定 50rpm 时磁场就只转到 50rpm, 不会先把电机拖到 409rpm 再往下降
	   (那个"来回冲"正是低速转一下停一下的根因)。
	   给定为 0 时频率一路降到 0: 磁场停住、转子跟着停住, 停机不再"抖一下"。 */
	{
		float ref = smo.if_ref_we;			/* 【带符号】给定频率: 负 = 反转 */
		float acc = SMO_OL_ACC * SMO_TS_PERIOD;

		if(ref >  SMO_OL_WE_MAX) ref =  SMO_OL_WE_MAX;
		if(ref < -SMO_OL_WE_MAX) ref = -SMO_OL_WE_MAX;

		/* 斜坡目标是带符号的给定频率, 所以换向时 ol_we 从 +w 一路减到 0、再继续
		   变负 —— 磁场速度连续过零, 转子同步减速 -> 静止 -> 反向加速, 全程没有
		   转差阶跃。旧版 ol_we 是幅值、方向另由 dir 决定, 换向只能靠翻 dir,
		   频率必然从 +w 阶跃到 -w, 那就是反接制动: 转差 2w, 电流环饱和, 抖。
		   这里绝对不能再把 ref 改成 |we_est|(原来那句"带速重启"的写法):
		   磁场频率是控制器自己给的, 观测器量到的就是它, 两边互相跟随就是正反馈,
		   we_est 抖高一点 -> ref 抬高 -> 磁场加速 -> we_est 更高 ... 实测设定
		   50rpm 被棘轮抬到 ~150rpm 后既回不到给定也切不进无感。带速接续已经由
		   Smo_IfEnter() 一次性做完。 */
		if(smo.ol_we < ref)
		{
			smo.ol_we += acc;
			if(smo.ol_we > ref) smo.ol_we = ref;
		}
		else if(smo.ol_we > ref)
		{
			smo.ol_we -= acc;
			if(smo.ol_we < ref) smo.ol_we = ref;
		}
	}

	/* 过零换向: 频率落到 0 附近、且给定方向与当前转向相反, 才翻转向。
	   【这里绝对不能动 ol_theta】。物理矢量角 = θ_app ± 90(uq 的符号决定), 而
	   θ_app = ol_theta - dir*90、uq 也随 dir 变号, 两者同时翻转, 互相抵消:
	     dir=+1: θ_app=ol_theta-90, uq=+v -> 矢量角 = θ_app+90 = ol_theta
	     dir=-1: θ_app=ol_theta+90, uq=-v -> 矢量角 = θ_app-90 = ol_theta
	   也就是"实际施加的电流矢量角恒等于 ol_theta", 与转向无关。翻 dir 时物理矢量
	   本来就是连续的; 旧版在这里补 dir*180, 等于把电流矢量瞬间反了 180 度 ——
	   转子正好落在 180 度的【不稳定平衡点】上, 之后磁场倒着转、转矩却仍指向前方,
	   转子被推着继续正转, 永远换不过去(表现为"抖"和"反转进不了闭环")。
	   同时清掉 PLL 的积分和速度估计: 它们还带着原方向的速度, 不清的话 theta_est
	   会被拉着继续往原方向转, 切换判据 dth 要很久才对得上。 */
	if(((smo.if_ref_we > 0.0f) && (smo.dir < 0) && (smo.ol_we >= -SMO_REV_WE_ZERO)) ||
	   ((smo.if_ref_we < 0.0f) && (smo.dir > 0) && (smo.ol_we <=  SMO_REV_WE_ZERO)))
	{
		int8_t dn = (smo.if_ref_we > 0.0f) ? 1 : -1;

		smo.dir        = dn;
		smo.ol_we      = 0.0f;			/* 从零重新起拖, 方向对了再加速 */
		smo.sum_pll    = 0.0f;
		smo.we_est     = 0.0f;
		smo.sw_cnt     = 0;
		smo.sw_win     = 0;
		smo.ph_bad_cnt = 0;
	}

	smo.ol_theta += smo.ol_we * SMO_TS_PERIOD;
	if(smo.ol_theta >= TWO_PI) smo.ol_theta -= TWO_PI;
	if(smo.ol_theta < 0.0f)    smo.ol_theta += TWO_PI;

	/* I/f 段的"转子角"参考(与无感段 theta_est 同一个量, 切换判据就比它俩):
	   前面已经推导过, 实际施加的电流矢量角恒等于 ol_theta(与 dir 无关), 转子空载时
	   贴在电流矢量上, 所以转子角 = ol_theta = Smo_IfTheta() + dir*90。
	   旧写法固定 +90: 正转恰好对, 反转就差了 180 度 -> dth 恒为 ±180,
	   切换判据 |dth|<70 永远不满足, 而且每 100ms 被判一次"相位反相(>120度)"把
	   theta_est 硬拽到反相点 -> 现象就是反转一直抖、永远切不进无感。 */
	th_if = Smo_IfTheta() + (float)smo.dir * (PI * 0.5f);
	while(th_if >= TWO_PI) th_if -= TWO_PI;
	while(th_if < 0.0f)    th_if += TWO_PI;

	/* 2) 切换判据：反电动势够大 + 转速够高 + 强拖角与估计角基本同步 + 连续确认
	   相位判据很关键：PLL 在低速可能收敛到相差 180 度的平衡点，一旦接管就是反相换相，
	   直接反转/失锁。强拖时转子角比施加电压角滞后一个负载角(通常 10~40 度)，
	   所以判据不能取太紧。 */
	if(smo.start_state == 0)
	{
		/* 换向过程中(转向还没转到给定方向)一律不判切换: 此刻反电动势还是原方向
		   的, 一旦接管, 换相角与磁场差 180 度, 一接管就反转/抖。等 dir 翻过来、
		   频率从 0 重新建起来之后再按正常判据切。 */
		if(smo.if_ref_we * (float)smo.dir < 0.0f)
		{
			smo.sw_cnt     = 0;
			smo.sw_win     = 0;
			smo.ph_bad_cnt = 0;
			return 0;
		}

		dth = smo.theta_est - th_if;
		while(dth > PI)  dth -= TWO_PI;
		while(dth < -PI) dth += TWO_PI;

		if(fabsf(dth) > SMO_PHASE_BAD)
		{
			if(++smo.ph_bad_cnt > SMO_PHASE_BAD_CNT)
			{
				smo.ph_bad_cnt = 0;
				/* 转子角滞后于施加电压角：反转方向回退一个负载角，重新捕获 */
				smo.theta_est = th_if - (float)smo.dir * SMO_ALIGN_LAG;
				if(smo.theta_est < 0.0f)     smo.theta_est += TWO_PI;
				if(smo.theta_est >= TWO_PI)  smo.theta_est -= TWO_PI;
				dth = smo.theta_est - th_if;
				while(dth > PI)  dth -= TWO_PI;
				while(dth < -PI) dth += TWO_PI;
			}
		}
		else
		{
			smo.ph_bad_cnt = 0;
		}

		smo.dth_deg = dth * (180.0f / PI);

		/* 判决窗口: 20ms 内满足条件的拍数够多就切换。
		   原来是"连续 100 拍", 估计角只要抖一下就清零重来, 低速时几乎不可能
		   连续满足 -> 一直切不进无感(实测 sensorless_flag 永远是 0)。 */
		if(smo.Eamp > SMO_BEMF_THRESHOLD && fabsf(smo.we_est) > SMO_SWITCH_SPEED &&
		   fabsf(dth) < SMO_SWITCH_PHASE)
		{
			if(smo.sw_cnt < 0xFFFF) smo.sw_cnt++;
		}
		if(++smo.sw_win >= SMO_SWITCH_WINDOW)
		{
			if(smo.sw_cnt >= SMO_SWITCH_CONFIRM) smo.start_state = 1;
			smo.sw_win = 0;
			smo.sw_cnt = 0;
		}
	}
	return smo.start_state;
}

/**
 * @brief  启动已耗时 [ms]，0 表示已经切入无感（评估启动快慢用）
 */
uint16_t Smo_GetStartTimeMs(void)
{
	return (uint16_t)(smo.start_tick / 10u);
}

/**
 * @brief  复位启动状态（停机时调用）
 */
void Smo_Startup_Reset(void)
{
	smo.start_state = 0;
	smo.sw_cnt = 0;
	smo.sw_win = 0;
	smo.acq_div = 0;
	smo.ph_bad_cnt = 0;
	smo.ol_theta = 0.0f;
	smo.align_cnt = SMO_ALIGN_CNT;	/* 停机后再启动也要先定角 */
	smo.start_tick = 0;
	smo.ol_we = 0.0f;				/* I/f 频率从 0 起步, 由斜坡升到给定 */
	smo.if_ref_we = 0.0f;
	smo.if_iref = 0.0f;
	smo.if_i = 0.0f;
	smo.if_vq = 0.0f;
	smo.if_iq_f = 0.0f;
	smo.if_init = 1;
	smo.sum_pll = 0.0f;
	smo.theta_est = 0.0f;
	smo.we_est = 0.0f;
	smo.theta_err = 0.0f;
	smo.Ialpha_esc_pre = 0.0f;
	smo.Ibeta_esc_pre = 0.0f;
	smo.Valpha_esc = 0.0f;
	smo.Vbeta_esc = 0.0f;
	smo.Ed_f = 0.0f;
	smo.Eq_f = 0.0f;
	smo.Eamp = 0.0f;
	smo.lag_comp = 0.0f;
	smo.angle_err_deg = 0.0f;
	/* psi_k 是辨识出来的物理参数，停机不清零 */
}

/**
 * @brief  失锁/换向后重启：交回低速 I/f 段，相位/频率/电压全部接续
 * @note   保留这个函数只是为了兼容旧调用点，实现完全等于 Smo_IfEnter()。
 *         原来的实现会 align_cnt = SMO_ALIGN_CNT，即带速重新定角 —— 那等于给正在
 *         转的转子一个反向制动；并且 ol_we 会被夹到 30rad/s 以上，和当前转速对不上。
 *         这两点都会让重启瞬间"抖一下"，是低速往复的另一半原因。
 */
void Smo_Startup_Restart(void)
{
	Smo_IfEnter();
}
