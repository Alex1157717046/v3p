# v3p-lab —— DengFOC V3P 实操课程源码

配套文档：`12-实操实验方案.md`（40 个实验）。
本工程把它变成**可编译、可烧录、可逐步验证**的代码。

## 0. 先看结论：原方案有 5 处必须修正

| # | 原方案说法 | 实际情况（依据原理图核对） | 影响 |
|---|---|---|---|
| 1 | 功率级是「分立 NMOS + **EG2133**」 | 是 **FD6287T**（另一路 FD6287）三相栅极驱动 | 器件参数要改；实验结论方向不变 |
| 2 | ★★★「V3P 死区由 MCU 决定，死区扫描实验可做」 | ❌ **错的**。FD6287T 的 `HIN1~3` 与 `LIN1#~3#` 在板上被接成**同一组网络**，每相只有 1 根 MCU PWM 线 | **E1.6 死区扫描做不了**（改为"死区测量 + 死区补偿"） |
| 3 | ESP32 用 MCPWM 的死区设置 | 官方 DengFOC 用 **LEDC**（`ledcSetup(ch,30000,8)`），LEDC 根本没有死区功能 | 同上 |
| 4 | STM32 的 TIM1 互补输出 + BDTR 死区「与这块板严丝合缝」 | ❌ TIM1 引脚在本设计里给了 MOTOR_EN/TXD/RXD/空网络，**完全没用上 TIM1**；本板也不需要互补 | 附录 G 的这条优势不成立 |
| 5 | （无人提到）STM32 转接板的两个硬伤 | ① **M1 三路 PWM 跨 TIM2/TIM3**；② **`MOTOR_EN` 网名写成了 `A8`，与 `PA8` 没连上** | 见 `docs/STM32转接板引脚映射对照表.md` §8.3、§11.1 |

详细逐条核对见 `docs/01-硬件核实报告.md`。

## 1. 环境

```bash
# 首次
python3 -m pip install -r tools/requirements.txt      # 上位机
~/.platformio/penv/bin/pio run -e exp00_selftest      # 编译（会自动下载 ESP32 平台）
```

**macOS 上不需要 mingw64、不需要设任何环境变量**（详见仓库根的 `macOS运行指南.md`）。

## 2. 烧录与查看

```bash
cd v3p-lab
~/.platformio/penv/bin/pio run -e exp03_svpwm7 -t upload     # 编译并烧录
python3 tools/host_scope.py -p /dev/cu.usbserial-XXXX        # 打开波形上位机
```

> 端口名用 `ls /dev/cu.*` 查；选 `cu.*` 不要选 `tty.*`。
> 上位机和一个文本串口监视器**不能同时开**（同一个口）。

## 3. 目录

```
v3p-lab/
├── platformio.ini            每个实验一个 env
├── lib/v3plab/               公共库（从零实现，不依赖 SimpleFOC）
│   ├── v3p_board.h           ★ 板级引脚/参数（已按原理图核对）
│   ├── v3p_math.h            Clarke/Park/SPWM/SVPWM/六步/死区补偿
│   ├── v3p_modulate.h        调制方式统一入口
│   ├── v3p_pwm.h/.cpp        三相 PWM（LEDC，频率可动态改）
│   ├── v3p_current.h/.cpp    INA240 + 分流 电流采样
│   ├── v3p_encoder.h/.cpp    AS5600（缓存式读取，避开 I2C 拖慢控制环）
│   ├── v3p_pid.h             PI + 三种抗饱和 + 低通
│   ├── v3p_telemetry.h/.cpp  二进制波形帧
│   └── v3p_console.h/.cpp    串口命令行
├── src/expXX_*.cpp           12 个实验（阶段 0~2）
├── tools/host_scope.py       上位机（pyqtgraph 实时波形 + 命令行）
└── docs/                     硬件核实报告、实验对照表
```

## 4. 已完成的实验：**阶段 0 ~ 7 全部 40 个实验**

| 阶段 | env | 个数 | 状态 |
|---|---|---|---|
| 0 基线与测量 | `exp00`~`exp11`（部分） | 4 | ✅ |
| 1 功率级与调制 | `exp01`~`exp07` | 7 | ✅ |
| 2 采样与坐标变换 | `exp08`~`exp11` | 6 | ✅ |
| 3 三环闭环 | `exp12`~`exp17` | 6 | ✅ |
| 4 观测器与无感 | `exp20`~`exp26` | 7 | ✅ |
| 5 辨识/故障/效率 | `exp30`~`exp34` | 6 | ✅ |
| 6 弱磁与对标 | `exp40`、`exp41` | 3（E6.3 为文档） | ✅ |
| 7 Q15 定点化 | `stm32f103-foc/`（独立工程） | 1（大实验） | ✅ |

**验证状态：**
- ESP32 侧 **32 个 env，`pio run` 一次全部编译通过**（`default_envs` 已设为全部）
- STM32 侧 **1 个 env 编译通过**（Flash 81%、RAM 8.8%）
- Q15 定点库在 PC 上做了**逐点数值对比**：sin 0.014%、Clarke/Park 0.021%、
  SVPWM 线电压 0.009%、PI 0.008%（详见 `docs/02-实验清单与源码对照.md`）

逐实验的对照表与每个实验的串口命令见 **`docs/02-实验清单与源码对照.md`**。

### 目录补充

```
v3p-lab/
├── stm32f103-foc/            阶段 7：STM32 Q15 定点化（独立 PlatformIO 工程）
│   ├── lib/focq15/foc_q15.h  Q15 FOC 数学库（可在 PC 上单独编译验证）
│   ├── src/main.cpp          浮点 vs 定点对比固件（DWT 测周期）
│   └── tools/q15_selftest.cpp 量化验证程序
```

## 5. 安全须知（每次上电前读一遍）

1. **驱动板默认失能**：所有实验上电时 `MOTOR_EN` 都是低电平，要显式 `en 1` / `start` 才有输出。
2. **第一次上电用限流电源**：12 V / 0.3 A 起步。
3. **三相占空比 0.5/0.5/0.5 = 零电压**，这是上电与停止时的安全状态。
4. ⚠ `exp07_gatedrive` 的 **mode 2（100% 占空比）会往电机灌直流**，必须在限流下做。
5. **示波器地线接相线 = 短路桥臂**。测相电压/VB-VS 必须差分探头。
6. 转接板天线丝印方向必须与 V3P 板一致，否则烧板。
