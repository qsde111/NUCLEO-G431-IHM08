# Host Command RX (LPUART1 + DMA Circular) 任务记录

日期：2026-02-19

## 本次实现了什么

目标：给 FOC 学习项目先搭一个“上位机指令 -> MCU -> 后续电机控制”的接收接口，电机功能先不实现。

已完成：

- LPUART1 **RX DMA circular** 环形缓冲接收（轮询 NDTR 取新字节，不依赖中断回调）。
- ASCII 指令解析（解耦分层）：
  - 支持 `P100` / `V20` / `C1` 这类“1 个字母 + 可选数字参数”的命令。
  - 以 `\\r` / `\\n` / `;` 作为命令结束符。
  - 额外支持“无结束符”的情况：如果收到一串字节后 **超过 10ms 没有新字节**，会自动 flush 当作一条命令（可通过宏调整）。
- 在 `MotorApp` 中接入解析结果，先做 **占位接口**（仅保存 setpoint/标志位，不做电机动作）。

## 分层与文件

- `BSP/bsp_uart_rx_dma.[ch]`
  - 只负责：启动 UART DMA 接收、从 circular DMA buffer 中“取增量字节片段”。
  - API：`BspUartRxDma_Start()` + `BspUartRxDma_Poll(..., on_bytes_cb)`

- `Components/host_cmd_parser.[ch]`
  - 只负责：把字节流解析为命令 `HostCmd { op, has_value, value }`，并放入一个小队列（默认 8 深度）。
  - 解析规则：首字符为命令字母（自动转大写）；后面若是数字（支持 `-12`/`3.14`），则记为 `value`。

- `App/host_cmd_app.[ch]`
  - 负责把 BSP 的 “DMA 新数据” 喂给 Components parser，并做一个“10ms idle flush”的策略。
  - 宏：
    - `HOST_CMD_APP_RX_DMA_BUF_LEN` 默认 256
    - `HOST_CMD_APP_IDLE_FLUSH_MS` 默认 10

- `App/motor_app.[ch]`
  - `MotorApp_Init()` 里启动 RX DMA。
  - `MotorApp_Loop()` 每次都会处理新命令；而原来的 1kHz encoder stream 仍保持不变。
  - 目前占位字段（后续你可以直接接控制逻辑）：
    - `target_pos_deg`：收到 `Pxxx` 更新
    - `target_vel_rad_s`：收到 `Vxxx` 更新
    - `calib_request` / `calib_request_pending`：收到 `C` 更新
    - `last_host_cmd` / `last_host_cmd_tick_ms`：调试用

## 当前行为说明（方便你调试）

- 上位机发送建议带结束符：`P100\\r\\n`、`V20\\n`、`C1;`
- 如果你发送 `P100` 不带结束符：停止发送后等待约 `HOST_CMD_APP_IDLE_FLUSH_MS`（默认 10ms），也会被当成一条命令解析。
- 解析到命令后 **不会回发 ACK**（避免和当前 `JustFloat_Pack4` 的二进制流混在一起导致上位机解析混乱）。

## 已知限制 / 后续可改进点

- DMA 环形缓冲如果长期不 poll，可能丢字节（目前主循环会频繁跑，短命令问题不大）。
- 目前命令只支持单参数；以后要扩展多参数（例如 `C1,xxx`）可以在 `Components/host_cmd_parser.c` 里扩展语法。
- 若你希望“上位机可确认命令是否收到”，建议另开一种纯 ASCII 模式/或另开一条串口/或在 JustFloat 流里加一个状态字段（不建议把 ASCII ACK 直接混入二进制 stream）。

---

## 2026-02-19：`C1` dir/zero_offset 校准（开环 SVPWM 脚手架）

- 触发：上位机发送 `C1` 开始；`C0` 中止。
- Tick：控制循环跑在 ADC Injected ISR（20kHz）；PC8(`S_Pin`) 输出脉冲用于测量 ISR 占用。
- PWM/ADC：TIM1 CH4(NoOutput) 产生 TRGO2 触发 ADC1+ADC2 Injected 同步采样；仅在校准期间使能 TIM1 CH1-3 + CH1N-3N 输出。
- 流程：ALIGN(0.5s, `Ud=0.08`) -> SPIN(0.3s, `Uq=0.06`, 5Hz 电角) -> 计算 `elec_dir` 与 `elec_zero_offset_rad`。
- 结果：`MotorApp.elec_dir`、`MotorApp.elec_zero_offset_rad`，并置位 `calib_done`（失败置位 `calib_fail`）。
- 数据流：`JustFloat_Pack4` 的第 3/4 个 float 改为输出 `elec_zero_offset_rad` 与 `elec_dir`（方便上位机观测校准结果）。
- 代码：`Components/motor_calib.[ch]`、`Components/svpwm.[ch]`、`BSP/bsp_tim1_pwm.[ch]`、`BSP/bsp_adc_inj_pair.[ch]`、`App/motor_app.[ch]`、`Core/Src/adc.c`（修正 ADC2 也用 TRGO2 外部触发）。


## 2026-02-20：实验结果

首先发现有一个改动是在void MX_ADC2_Init(void)中增加了代码
```c
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
```
类似ADC1，这里其实不需要增加，配置了 multimode.Mode = ADC_DUALMODE_REGSIMULT;这个东西之后ADC1 和ADC2会同步触发的，按照我看了我上一个项目的配置，这里不主动修改成TRGO2触发，ADC2也会和ADC1一起触发的，而且CubeMX中设置了ADC2 是Slave之后也不能配置这个了，所以我从Cube MX中再次Generate code之后，这里的代码也会自动消失

驱动器芯片下桥臂是低电平有效，但是上桥臂是高电平有效啊，我上电之后逻辑分析仪发现三路上桥臂默认是高电平状态，下桥臂也是高电平状态，这里应该不是被BKIN禁止了，因为我给指令C1会6路信号出现PWM控制。这里应该是上电初始化，以及停止控制的时候没有做，上电后，应该是CH1/2/3都是低电平，而CHN1/2/3都是高电平，停止控制，IDLE状态什么的同理。

发现给定指令C1后，即使六路都有占空比，但是这个占空比从开始，一直都是不变化的，其中上桥臂的占空比为46%，其他路同理，没有变化。将PC8加入逻辑分析仪，发现没有信号。后我在`stm32g4xx_it.c`的`void ADC1_2_IRQHandler(void)`函数中，加入了HAL库的翻转电平函数，此后能在逻辑分析仪中观察到每个PWM低电平信号的中心处左右，PC8会翻转一轮电平

### 2026-02-20：修复 & Debug 手段

- 根因：`HAL_ADCEx_InjectedConvCpltCallback()` 里之前等 `ADC1` + `ADC2` 都回调到齐才调用控制逻辑；但在你这种 **ADC1 主 / ADC2 从** 的用法里，常见做法是“只在 ADC1 回调里做控制”（ADC2 不一定会单独触发回调），导致控制逻辑没跑 -> PWM 占空比一直停在初始值，PC8 也没脉冲。
- 修复：改成 **只在 ADC1 回调触发**，并在 ADC1 回调里直接读取 ADC2 的 Injected 值；同时 ADC2 只 `HAL_ADCEx_InjectedStart()`（不启用 IT）。
- 观测：
  - PC8(`S_Pin`) 现在由 `MotorApp_OnAdcPair()` 用 BSRR 拉高/拉低输出脉冲（不用在 `ADC1_2_IRQHandler` 里 Toggle 了）。
  - `JustFloat_Pack4` 在校准运行时每 1ms **交替发送**两种帧：普通帧(encoder/offset/dir) 与 Debug 帧(dutyA/dutyB/dutyC/state)。Debug 帧的第 4 个 float 为 `state`：1=ALIGN，2=SPIN，3=DONE，4=FAIL。

### 2026-02-20：TIM1 6PWM 上电/停止 Idle 状态修复

- 现象：上电空闲时，TIM1 CH1/2/3（上桥臂）在逻辑分析仪上默认为高电平；校准结束 `DisableOutputs()` 后也保持高电平。
- 根因：之前 `DisableOutputs()` 用 `HAL_TIM_PWM_Stop/HAL_TIMEx_PWMN_Stop` 会把 CCxE/CCxNE 清掉，PWM 引脚变成 Hi-Z；而 IHM08M1 输入侧有上拉/默认态，导致你看到“空闲=高电平”。另外只启动 CH4(NoOutput) 做 TRGO2 时，HAL 会把 MOE 打开，但 CH1-3/CHxN 没有使能也会导致引脚不受控。
- 修复策略：**保持 CCxE/CCxNE 使能，让引脚始终被 TIM1 驱动**；启停只通过 BDTR.MOE 门控，并利用 CubeMX 已配置的 OSSI/OSSR + IdleState（CHx idle=RESET，CHxN idle=SET）实现“空闲=上桥臂低/下桥臂高”。
- 代码：新增 `BspTim1Pwm_ArmIdleOutputs()`；并在 `BspTim1Pwm_StartTrigger()` 里自动执行一次（上电启动 TRGO2 后立刻把 6PWM 引脚拉到 Idle）；`BspTim1Pwm_DisableOutputs()` 改为 `__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY()`（不再 Stop 通道）。
- 预期观测：上电空闲与校准结束后，CH1/2/3=低电平，CH1N/2N/3N=高电平；下发 `C1` 才出现 PWM。

### 2026-02-20：电流采样（Ia/Ib）上电 offset + 单位换算框架

- 目标：先把 **ADC 原始值 -> 去偏置 -> 物理电流(A)** 这条链路跑通，后面再接入电流环（PI + SVPWM）。
- offset：上电后在 **PWM 输出未使能**（MOE=0，空闲态）期间累计 1000 次注入采样（20kHz -> 约 50ms）求平均，作为 `offset_raw`。
- 换算：`I[A] = (raw - offset_raw) * (Vref/ADCmax) / (Rshunt * Gain)`；当前常量：`Rshunt=0.01Ω`，`Gain=5.18`，`Vref=3.3V`，`ADCmax=4095`。
- 分层：新增纯组件 `Components/current_sense.h`（无 HAL 依赖）实现 offset 累计与换算；`App/motor_app.c` 在 ADC ISR 中推样本并更新 `ia_a/ib_a`。
- 打印：新增 HostCmd `D1` 切到“电流页”JustFloat 输出：offset 未就绪时发 `rawA/rawB/avgA/avgB`；就绪后发 `Ia/Ib/offsetA/offsetB`（`D0` 回到原来的 encoder/校准页）。

## 2026-02-20：ADC采样实验结果

学会了D1页和D0页的使用，首先发现D1页中，通道2无值，但是通道4有值(V相电流OFFSET校准成功)
在`adc.c`的void MX_ADC2_Init(void)中，参考ADC1加入
```c
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
```
上电后采样正常
发现在CubeMX中错误的将ADC1 2配置成`Dual regular simultaneous mode only`,修改成`Dual injected simultaneous mode only`，正确的同步注入触发模式，再次`Generate Code`，生成代码后系统会自动删除上述
```c
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
```
部分，再次上电，D1页通道2，4都没有值，采样和V相电流校准都失败，加入上述代码同样失败。
参考了之前项目中ADC初始化的程序，似乎**双重主从模式下，建议先启动从机(ADC2)，再启动主机(ADC1)**，找到bsp_adc_inj_pair.c中的初始化`HAL_ADCEx_InjectedStart_IT(ctx->hadc1)`部分，将adc2的初始化放在adc1前边，再次烧录上电，D1页的通道2、4有值，采样成功
此外在D1页中，下达指令C1，校准过程中通道1、2(U V相电流)形状类似马鞍波，猜测已经做了SVPWM的零序电压注入？同时两相相差90deg波形正确。