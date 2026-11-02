# 无感 FOC 电机控制器（STM32F407 + 滑模观测器）

> 一套跑在 STM32F407 上的无位置传感器（Sensorless）磁场定向控制固件。不需要电机尾部的位置传感器，靠滑模观测器从相电流中反推转子角度来驱动三相无刷电机。

[![MCU](https://img.shields.io/badge/MCU-STM32F407ZETx-03234B)](https://www.st.com/)
[![IDE](https://img.shields.io/badge/IDE-Keil%20MDK%205-199A37)]()
[![Compiler](https://img.shields.io/badge/ARMCC-5.06-blue)]()
[![License](https://img.shields.io/badge/license-MIT-lightgrey)]()

---

## 这是什么

一套完整的无刷电机控制固件，包含：

- **电流环 / 速度环 / 位置环** 三环串级控制，全部 Q15 定点实现（Cortex-M4F 上无浮点依赖的实时性保障）
- **滑模观测器（SMO）+ PLL 锁相环** 估算转子电角度与转速
- **无感启动**：开环强拖 → 反电动势幅值/转速双判据 → 平滑切换到观测器闭环
- **七段式 SVPWM**，TIM1 中心对齐 10 kHz，双沿触发 ADC 实现 20 kHz 三相同步采样
- **上电自校准**：电流零漂偏置在线校准 + d 轴注入对齐
- **VOFA+ 上位机通道**：实时波形上传 + 在线修改 PID 参数，无需重新烧录

自研代码约 3800 行（不含 ST HAL 与 CMSIS 驱动）。

---

## 关键参数

| 项目 | 指标 |
|---|---|
| 主控 | STM32F407ZETx，Cortex-M4F @168 MHz |
| PWM | TIM1 中心对齐，10 kHz（28 MHz 计数，ARR=1399） |
| 电流采样 | ADC3 注入组，TIM1_CH4 双沿触发，**20 kHz**（Ts = 50 µs） |
| 控制周期 | 100 µs（每 2 次采样执行一次控制） |
| 位置反馈 | MT6701 磁编码器（14 bit，SPI3）——**仅用于调试期真值比对** |
| 观测器 | 滑模观测器 + PLL，估计电角度与转速 |
| 电机参数 | 7 极对，额定 24 V，相电阻 0.4 Ω，相电感 156 µH |
| 通信 | USART6 @1 Mbps（调试/指令）、USART3 @1 Mbps（VOFA+ 波形） |
| 开发环境 | Keil MDK 5，ARMCC 5（-O1） |

---

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│  应用层  Drivers/App                                     │
│  10 ms 进程状态机 · VOFA+ 指令解析 · ADC 环形队列滤波      │
├─────────────────────────────────────────────────────────┤
│  算法层  Drivers/FOC                                     │
│  SMO 观测器 · Q15 PID · SVPWM · Clarke/Park · S 曲线规划  │
├─────────────────────────────────────────────────────────┤
│  BSP 层  Drivers/BSP                                     │
│  adc / time / spi / sram / gpio / led（引脚与时钟收敛于此）│
├─────────────────────────────────────────────────────────┤
│  ST HAL + CMSIS                                          │
└─────────────────────────────────────────────────────────┘
```

**控制链路**

```
TIM1 10kHz ──CH4触发──> ADC3 注入采样 ──50us──> MotorHandler
                                                      │
                                              每2次执行一次 (100us)
                                                      ↓
                                        ┌─────────────┴─────────────┐
                                        │  SMO 估角 → PID → SVPWM   │
                                        └─────────────┬─────────────┘
                                                      ↓
                                         三相全桥 ──> 电机 ──> 电流反馈
```

---

## 核心实现

### 1. 电流采样与自校准

TIM1 中心对齐模式下 CH4 比较事件在每个 PWM 周期触发两次，把 ADC 触发点放在计数器低区（`ADC_SAMPLE_POINT = 50`），既避开了开关噪声，又把有效采样率提到 20 kHz。

上电后前 1024 次采样做累加平均得到三相零漂偏置，运行中逐拍减去；再做一次 d 轴注入把转子"拉"到已知角度，记录编码器偏置完成电气角对齐。

### 2. 滑模观测器（SMO）

- 以 αβ 轴电流误差构造滑模面，符号函数 + 边界层抑制抖振（`h = 2`）
- 观测出的等效控制量经一阶低通滤波得到反电动势 `Eα / Eβ`
- PLL 从反电动势中提取转子角度：`dθ = −Eα·cos θ − Eβ·sin θ`，PI 调节后积分得到角度
- 转向前馈：反电动势矢量随旋转方向翻转 180°，用 `out_vq` 的符号做方向补偿，解决反向旋转时 PLL 收敛到反向平衡点的问题

### 3. 无感启动与切换

```
上电校准 → 开环强拖 → 反电动势判据 → SMO 闭环
                        |E| > 0.15 V
                        |we_est| > 60 rad/s
```

强拖阶段以固定步长（0.019 rad / 100 µs）推进电角度、固定电压幅值拖动转子；当反电动势幅值与估计转速同时越过阈值，把换相角从 `ol_theta` 切换为 `theta_est`，环路无扰切换。

### 4. 定点 PID

Q10 系数、Q15 数据的位置式 PID，积分项用 64 位累加防止溢出，输出饱和时回退积分累加值做抗饱和。速度环 / 电流环共用一套实现，仅参数不同。

### 5. 上位机联调

VOFA+ JustFloat 协议上传 11 路浮点观测量（估计角 vs 实测角、αβ 反电动势、PLL 输出等），同时解析 `S M:1-SPEED_P:120 E` 格式的指令在线改写 PID 参数，省去反复烧录的时间。

---

## 目录结构

```
.
├── User/                 main.c / 中断向量 / HAL 配置
├── Drivers/
│   ├── FOC/              控制算法（核心，自研）
│   │   ├── bldc_control.c  电机状态机 + 各环路
│   │   ├── smo.c           滑模观测器 + PLL
│   │   ├── pid.c           Q15 位置式 PID
│   │   ├── svpwm.c         七段式 SVPWM
│   │   ├── clarke_park.c   Clarke / Park 变换
│   │   ├── trap_planner.c  S 曲线位置规划
│   │   ├── sensor.c        编码器读取与测速
│   │   └── filter.c        一阶 IIR 滤波
│   ├── BSP/              板级外设：adc / time / spi / sram / gpio / led
│   ├── App/              10 ms 状态机 / VOFA+ 协议 / 环形队列
│   ├── SYSTEM/           时钟、延时、printf 重定向
│   ├── CMSIS/            ARM CMSIS
│   └── STM32F4xx_HAL_Driver/
├── Projects/MDK-ARM/     Keil 工程（BLDC_FOC_SMO.uvprojx）
└── Output/               编译输出（不入库）
```

---

## 编译与运行

1. 用 Keil uVision 5 打开 `Projects/MDK-ARM/BLDC_FOC_SMO.uvprojx`
2. 编译器选择 **ARM Compiler 5**（工程已固定 `uAC6=0`）
3. Build（F7），输出到 `Output/bldc_foc_smo.hex`
4. 烧录后串口 6 / 串口 3 均为 **1 Mbps**，`printf` 输出观测数据

> 源码中的注释为 **GB2312 编码**，在 Keil 中请将 Edit → Configuration → Encoding 设为 `Chinese GB2312`，否则中文注释显示为乱码。

---

## 关键代码索引

| 内容 | 位置 |
|---|---|
| 电流环中断入口 | `Drivers/BSP/adc.c` → `HAL_ADCEx_InjectedConvCpltCallback()` |
| 电机主状态机 | `Drivers/FOC/bldc_control.c` → `MotorHandler()` |
| 滑模观测器 | `Drivers/FOC/smo.c` → `Smo_Control()` |
| 无感启动切换 | `Drivers/FOC/smo.c` → `Smo_Startup()` |
| 相电压输出 | `Drivers/FOC/bldc_control.c` → `SetPhaseVol()` |
| SVPWM 扇区法 | `Drivers/FOC/svpwm.c` → `set_svpwm_q15()` |
| 10 ms 进程调度 | `Drivers/App/task_process.c` → `sys_process_run()` |
| 上位机指令解析 | `Drivers/App/decodeVofa+.c` → `parse_vofa_command()` |

---

## 后续计划

- [ ] 把速度环 / 位置环也切到观测器角度，实现全模式无感
- [ ] 观测器加入转速自适应滑模增益，拓宽调速范围
- [ ] 收敛 BSP 边界：把 MT6701 驱动从算法层移到 `Drivers/BSP`
- [ ] 电流环中断优先级提到最高（当前 1 ms 定时器会抢占）

---

## 说明

- ST HAL 驱动与 CMSIS 遵循各自原始许可（BSD-3-Clause / Apache-2.0）。
- `Drivers/FOC`、`Drivers/BSP`、`Drivers/App` 下代码为作者原创。
