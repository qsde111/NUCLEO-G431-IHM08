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
