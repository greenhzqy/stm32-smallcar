# CAR — STM32F103 四路红外循迹小车（附电赛 H 题设计文档）

> 基于 STM32F103C8T6 + TB6612 的差速循迹小车：4 路 TCRT5000 红外循迹、位置式 PID 纠偏、渐进式丢线搜索、弯道自适应降速，OLED 实时显示偏差与速度。仓库同时收录电赛 H 题「车载平衡滚球运动控制系统」的设计文档与实现计划（**尚未编码实现**）。

## 项目简介

这是一个**个人学习 / 电赛备赛项目**（提交历史 2026-08-06，共 7 次提交，单人维护）。它把「让一辆两轮差速小车沿黑线稳定跑完一圈」这件事做完整：传感器去抖、偏差建模、PID 整定、丢线自救、弯道降速、片上调试显示，全部在裸机主循环里用纯 C 实现，无 RTOS。

适合两类读者：一是想跟着一个**能实际跑起来的最小循迹系统**学习 PID 与电机控制的初学者；二是需要参考「STM32F103 + TB6612 + 红外循迹」硬件连线与代码组织方式的同学。

仓库里的 `docs/superpowers/` 是下一阶段目标：在现有小车基础上加装平衡滚球装置（舵机摆杆 + VL53L0X 测距 + MPU6050 姿态 + 串级 PID）去完成电赛 H 题。**这部分目前只有设计与计划文档，代码尚未落地**，请勿按已实现功能理解。

## 功能特性

- **4 路红外循迹**：L1 / L2 / R2 / R1 一字排开，读取结果为加权偏差（L1 = -3、L2 = -1、R2 = +1、R1 = +3），越靠外侧的传感器权重越大，符合「偏得越多、纠正越猛」的直觉。
- **软件去抖**：每个传感器连读 3 次做多数表决（3 次中至少 2 次为 1 才判为黑线），抑制红外模块在黑白边界的抖动，避免电机来回摇摆；单次读取约 3 μs，占用 5 ms 控制周期不到 0.2%。
- **位置式 PID 纠偏**：`Kp = 10.0`、`Ki = 0.15`、`Kd = 8.0`，积分项限幅 ±3、总输出限幅 ±400，抑制积分饱和与转向过冲。
- **渐进式丢线搜索**：四路全白（丢线）时冻结积分，按上次偏差方向搜索，搜索力度从 3 起步、每 5 ms 增加 0.5、上限 8——丢线越久转得越猛，找回线后自动复位计数。
- **弯道自适应降速**：按偏差绝对值分三档基础速度（>3 → 250，>1.5 → 350，否则 450），急弯与丢线搜索时降速通过，直线段全速。
- **差速转向 + 硬限幅**：左右轮速 = 基础速度 ∓ PID 输出，并钳位到 [0, 999]，防止速度溢出与反向翻转。
- **TB6612 双路电机驱动**：每路 2 根方向 GPIO + 1 路 PWM，`motor_setspeed()` 统一处理正/反转与绝对值占空比。
- **0.96 寸 OLED 片上调试**：软件 I2C 驱动 SSD1306，每 500 ms 刷新偏差（×10）与当前基础速度，无需串口即可现场观察整定效果。
- **CubeMX 可复现配置**：`CAR.ioc` 完整保存引脚、时钟树与外设参数，改外设不必手写初始化。

## 硬件与引脚分配

以下引脚定义取自 `CAR.ioc` 与 `Core/Inc/main.h`，并经 `Core/Src/gpio.c`、`Core/Src/tim.c`、`Core/Src/track.c` 交叉核对。

| 功能 | 引脚 | 配置 | CubeMX 标签 |
| --- | --- | --- | --- |
| 循迹传感器（代码中的 R2，次右） | PA0 | GPIO 输入，无上下拉 | `track1` |
| 循迹传感器（代码中的 L2，次左） | PA1 | GPIO 输入，无上下拉 | `track2` |
| 循迹传感器（代码中的 L1，最左） | PA2 | GPIO 输入，无上下拉 | `track3` |
| 循迹传感器（代码中的 R1，最右） | PA3 | GPIO 输入，无上下拉 | `rack4` |
| 左电机 PWM | PA6 | TIM3_CH1，复用推挽，高速 | — |
| 右电机 PWM | PA7 | TIM3_CH2，复用推挽，高速 | — |
| 左电机方向 AIN1 | PB0 | 推挽输出，高速 | `ain1` |
| 左电机方向 AIN2 | PB1 | 推挽输出，高速 | `ain2` |
| 右电机方向 BIN1 | PB10 | 推挽输出，高速 | `bin3` |
| 右电机方向 BIN2 | PB11 | 推挽输出，高速 | `bin4` |
| OLED SCL（软件 I2C） | PB8 | 推挽输出，上拉 | — |
| OLED SDA（软件 I2C） | PB9 | 推挽输出，上拉 | — |
| SWD 调试 | PA13 / PA14 | Serial Wire | — |
| 外部晶振 | PD0 / PD1 | HSE 8 MHz | — |

- **主控**：STM32F103C8T6（LQFP48，Cortex-M3，IROM 64 KB / IRAM 20 KB）。
- **时钟**：HSE 8 MHz 经 PLL ×9 → SYSCLK 72 MHz，APB1 = 36 MHz，APB2 = 72 MHz。
- **PWM 参数**：TIM3，`Prescaler = 3`、`Period = 999`，即 72 MHz / 4 / 1000 = **18 kHz** PWM，占空比分辨率 0…999（与代码中的速度限幅一致）。
- **电机方向逻辑**：左轮正转 PB0 = 1 / PB1 = 0，右轮正转 PB10 = 0 / PB11 = 1（左右电机镜像安装）。
- **传感器约定**：代码中 `BLACK = 1`、`WHITE = 0`，红外模块需先调好比较电位器使「黑线输出 1、白底输出 0」。

## 目录结构

```text
CAR/
├── CAR.ioc                     # STM32CubeMX 工程配置（STM32F103C8T6 / TIM3 PWM / 引脚标签）
├── .mxproject                  # CubeMX 工程元数据（生成文件与头文件路径记录）
├── .gitignore                  # 忽略 Keil 编译产物、调试器本地配置等
├── Core/
│   ├── Inc/
│   │   ├── main.h              # 引脚标签宏定义 + Error_Handler 声明
│   │   ├── gpio.h / tim.h      # 外设初始化接口
│   │   ├── stm32f1xx_hal_conf.h
│   │   ├── stm32f1xx_it.h
│   │   ├── motor.h             # 电机驱动接口
│   │   ├── track.h             # 循迹传感器接口
│   │   ├── OLED.h              # OLED 接口
│   │   ├── OLED_Font.h         # 8×16 ASCII 字库
│   │   └── struct_typedef.h    # 早期自定义整型别名（当前无文件引用）
│   └── Src/
│       ├── main.c              # 主循环：读传感器 → 算偏差 → PID → 速度决策 → OLED
│       ├── motor.c             # TB6612 方向控制 + TIM3 占空比输出
│       ├── track.c             # 4 路红外读取 + 三取二去抖 + 权重定义注释
│       ├── OLED.c              # 软件 I2C 时序 + SSD1306 初始化/清屏/字符/数字显示
│       ├── gpio.c              # MX_GPIO_Init（传感器输入、方向输出、I2C 引脚）
│       ├── tim.c               # MX_TIM3_Init（PWM 通道 + GPIO 复用配置）
│       ├── stm32f1xx_it.c / stm32f1xx_hal_msp.c
│       └── system_stm32f1xx.c
├── Drivers/
│   ├── CMSIS/                  # ARM CMSIS 内核头文件 + STM32F1 设备头文件（各含原厂 LICENSE.txt）
│   └── STM32F1xx_HAL_Driver/   # ST HAL 驱动（Inc、Inc/Legacy、Src）
├── MDK-ARM/
│   ├── CAR.uvprojx             # Keil MDK 工程（目标 CAR，输出 CAR.hex）
│   ├── CAR.uvoptx              # 调试/下载/窗口配置
│   ├── startup_stm32f103xb.s   # 启动文件
│   ├── RTE/_CAR/RTE_Components.h
│   ├── DebugConfig/            # 调试器配置
│   └── CAR/                    # 编译产物（.axf/.hex/.map/.o 等，已被 .gitignore 忽略）
├── docs/superpowers/
│   ├── specs/2026-08-06-电赛H题-车载平衡滚球控制系统-设计.md
│   └── plans/2026-08-06-H题实现计划.md
├── 赛题/
│   └── H题_车载平衡滚球运动控制系统.pdf
└── 寻迹小车学习/
    └── .obsidian-bak/          # 仅剩 Obsidian 配置备份，笔记正文已移出本仓库
```

## 快速开始

### 1. 环境准备

| 项目 | 版本 / 说明 |
| --- | --- |
| IDE | Keil MDK-ARM V5（本机 `CAR.build_log.htm` 记录为 µVision V5.24.2.0） |
| 编译器 | ARMCC V5.06 update 5（工程 `uAC6 = 0`，使用 AC5 而非 AC6） |
| 器件支持包 | `Keil.STM32F1xx_DFP.2.2.0`、`ARM.CMSIS.5.0.1` |
| 下载器 | ST-Link（工程调试器为 ST-Link，接 PA13 / PA14 / GND / 3V3） |
| 代码生成（可选） | STM32CubeMX 6.11.1 + STM32Cube FW_F1 V1.8.7 |
| 硬件 | STM32F103C8T6 最小系统板、TB6612 双路驱动、4× TCRT5000、0.96 寸 I2C OLED、2 个直流减速电机 + 车轮、电池组 |

### 2. 编译

1. 用 Keil µVision 打开 `MDK-ARM/CAR.uvprojx`。
2. 目标选择 `CAR`，直接 Build（F7）；工程已勾选生成 HEX（`CreateHexFile = 1`），编译成功后产物在 `MDK-ARM/CAR/CAR.hex`。
3. 最近一次构建记录（`MDK-ARM/CAR/CAR.build_log.htm`）：**0 Error(s), 3 Warning(s)**，警告均为 `motor.h` / `OLED.h` / `track.h` 末尾缺少换行；占用 `Code = 7326`、`RO-data = 1822`、`RW-data = 24`、`ZI-data = 1704` 字节。

> 编译产物（`.o` / `.axf` / `.hex` / `.map` 等）已被 `.gitignore` 排除，克隆仓库后需要自行编译一次。

### 3. 烧录

1. ST-Link 连接目标板 SWD（PA13 = SWDIO、PA14 = SWCLK），供电共地。
2. 在 Keil 中直接 Download（F8）下载到 Flash；也可用 STM32CubeProgrammer 等工具烧写 `MDK-ARM/CAR/CAR.hex`。

### 4. 运行与调试

1. 上电即自动进入循迹循环，无按键、无串口交互。
2. 把车放到赛道上（传感器正对黑线），先确认红外模块输出与代码约定一致：**黑线为 1、白底为 0**；不一致需调模块上的比较电位器。
3. 观察 OLED：第 2 行显示偏差 ×10（负数表示车偏左），第 4 行显示当前基础速度（250 / 350 / 450）。
4. 调参只需改 `Core/Src/main.c` 中 `while (1)` 前的 4 个变量：`Kp` / `Ki` / `Kd` / 以及搜索与限幅阈值，重新编译下载即可。

### 5. 重新生成外设代码（可选）

用 STM32CubeMX 打开 `CAR.ioc`（固件包选择 FW_F1 V1.8.7）。工程已开启 `KeepUserCode`，`USER CODE BEGIN/END` 之间的手写代码会被保留；但 `motor.c` / `track.c` / `OLED.c` 是自行添加的文件，不在 CubeMX 管理范围内，重新生成后需确认 Keil 工程仍包含它们。

## 控制算法说明

单次 5 ms 控制周期内的处理顺序（对应 `Core/Src/main.c` 主循环）：

1. **采样**：`Track_L1/L2/R2/R1()` 各连读 3 次取多数，输出 0 / 1。
2. **偏差计算**：`error = -3·L1 - 1·L2 + 1·R2 + 3·R1`，即传感器的加权和；全 0 表示丢线。
3. **丢线处理**：冻结积分项，按 `last_error` 符号决定搜索方向，`search_power = min(3 + 0.5 × loss_count, 8)`，`error = 方向 × 力度`。
4. **PID**：`output = Kp·error + Ki·Σerror + Kd·(error − last_error)`；`Σerror` 限幅 ±3（仅在未丢线时累加），`output` 限幅 ±400。
5. **速度决策**：`|error|` 分档决定 `base_speed`（250 / 350 / 450）。
6. **输出**：`left = base_speed − output`、`right = base_speed + output`，各自钳位到 [0, 999] 后写入 TIM3 比较寄存器。
7. **显示**：每 500 ms 刷新 OLED，随后 `HAL_Delay(5)` 结束本轮。

当前参数（硬编码于 `main.c`）：

| 参数 | 值 | 代码注释中的作用 |
| --- | --- | --- |
| `Kp` | 10.0 | 比例系数，纠偏主力 |
| `Ki` | 0.15 | 积分系数，修正物理不对称 |
| `Kd` | 8.0 | 微分系数，抑制震荡 |
| 积分限幅 | ±3 | 抑制积分饱和 |
| 输出限幅 | ±400 | 限制转向量 |
| 基础速度分档 | 250 / 350 / 450 | 急弯 / 缓弯 / 直线 |
| 轮速钳位 | 0 … 999 | 对齐 PWM 占空比分辨率 |

## 技术栈

- **MCU**：STM32F103C8T6（Cortex-M3 @ 72 MHz，64 KB Flash / 20 KB RAM）
- **固件库**：STM32Cube HAL（FW_F1 V1.8.7）+ CMSIS（ArmCMSIS 5.0.1 / DFP 2.2.0）
- **配置工具**：STM32CubeMX 6.11.1（`CAR.ioc`）
- **工具链**：Keil MDK-ARM V5 + ARMCC V5.06（AC5），启动文件 `startup_stm32f103xb.s`
- **语言与架构**：纯 C，裸机轮询式主循环（无 RTOS、无中断驱动控制）
- **外设使用**：TIM3 双通道 PWM、GPIO 输入（传感器）/ 输出（电机方向、软件 I2C）
- **执行器 / 传感器**：TB6612 双路电机驱动、4× TCRT5000 红外反射传感器、SSD1306 0.96 寸 OLED（GPIO 模拟 I2C，器件地址 0x78）

## 文档索引

| 文档 | 位置 | 内容 |
| --- | --- | --- |
| 电赛 H 题设计文档 | `docs/superpowers/specs/2026-08-06-电赛H题-车载平衡滚球控制系统-设计.md` | 赛题要求与分值拆解、硬件选型与引脚规划、供电方案、机械结构、模块分层、串级 PID 与多频率任务调度设计、开发阶段规划、风险表 |
| 电赛 H 题实现计划 | `docs/superpowers/plans/2026-08-06-H题实现计划.md` | 拆成 10 个可执行任务（MPU6050 / VL53L0X / 舵机 TIM1 / 按键 / chassis / pendulum 串级 PID / task_manager / main 整合 / 联调 / ST 官方 API 升级），每步含代码与验证方式 |
| 赛题原件 | `赛题/H题_车载平衡滚球运动控制系统.pdf` | 竞赛题目 PDF（约 483 KB） |

三份材料都围绕「**下一步要做什么**」，与当前可运行的循迹代码相互独立。

## 备注与已知问题

1. **H 题功能尚未实现**：仓库中不存在 `imu.c` / `ball.c` / `pendulum.c` / `chassis.c` / `task_manager.c`，相关要求仅停留在设计文档与实现计划中；`main.c` 里也没有对应的占位代码。
2. **`Core/Src/main.c` 不是纯 UTF-8 文件**：少数中文注释中存在无法按 UTF-8 解码的字节（显示为乱码替换符），建议统一另存为 UTF-8 修复。
3. **注释与代码不一致**：`main.c` 中 OLED 刷新处的注释写「每 200ms 刷新一次」，实际判断条件是 `HAL_GetTick() - oled_timer > 500`，即 500 ms。
4. **传感器标签与代码映射顺序不一致**：CubeMX 标签按 PA0→PA3 为 `track1` / `track2` / `track3` / `rack4`，而 `track.c` 的读取顺序是 L1 = PA2、L2 = PA1、R2 = PA0、R1 = PA3；其中 `rack4` 应为 `track4` 的拼写遗留。改动接线时务必以代码为准。
5. **控制周期并非严格 5 ms**：周期由 `HAL_Delay(5)` 与 OLED 阻塞式软件 I2C 刷新共同决定，OLED 刷新那一轮会明显变长；代码注释所称「保证 PID 积分/微分按时间计算」只在无刷新的周期内近似成立，PID 未按实际 `dt` 归一。
6. **`Core/Inc/struct_typedef.h` 已无引用**：它仅被列入 Keil 工程，当前没有任何 `.c` 文件包含它；其中 `int8_t` / `uint32_t` 等 `typedef` 与标准 `<stdint.h>` 重复，若日后启用存在类型冲突风险。
7. **显示尺寸不满足 H 题要求**：现行 0.96 寸 OLED 达不到赛题「显示装置不小于 2 英寸」的硬性要求，设计文档已把它列为**确定性风险**，需更换为 2.42 寸 OLED 或 LCD。
8. **PID 与速度阈值为经验值**：仓库内没有整定记录或实测数据（如跑圈时间、稳态偏差），参数仅在代码注释中说明用途，换车体或换赛道需重新整定。
9. **`寻迹小车学习/` 目录已空**：笔记正文已移出本仓库，工作区仅剩 `.obsidian-bak/`（Obsidian 的配置备份，非学习内容）。如需查阅原笔记，可从 git 历史中检出（HEAD 中共有 22 个 `learning-notes/` 与 `DEBUG_LOG.md` 文件）。
10. **仓库根目录没有 LICENSE 文件**：`Drivers/CMSIS` 与 `Drivers/STM32F1xx_HAL_Driver` 各自附带原厂 `LICENSE.txt`（ARM / ST），用户代码部分未声明许可协议。
