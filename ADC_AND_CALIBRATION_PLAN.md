# ADC读取 + 电机校准（zero_offset / 相序-方向）方案草案

日期：2026-02-19

> 这是下一阶段的设计文档，用于后续实现“换电机/随便换相序 -> 跑一次校准 -> 系统照样能FOC”的能力。

---

## 1. 目标拆解

你描述的校准，核心要解决两件事（假设已知极对数 `pole_pairs`）：

1) **编码器机械角 -> 电角** 的映射：

- `theta_e = dir * pole_pairs * theta_mech + zero_offset`（再做 0~2π wrap）
- 其中：
  - `dir ∈ {+1, -1}`：编码器增大时，对应电角是“正转”还是“反转”
  - `zero_offset`：编码器零位与电角零位的偏移

2) **电流采样**：为电流环/保护/更可靠的校准提供实时相电流（以及母线电压等）。

---

## 2. 分层架构建议（解耦）

### BSP（硬件胶水）

- `BspPwmTim1`：PWM/死区/使能脚（后续可加刹车、故障输入）。
- `BspAdcDual`：ADC1/ADC2 配置与触发（Injected + Regular）。
- `BspFlashKv`（可选）：把校准参数存进 Flash（或先放 RAM，后面再做持久化）。

BSP 层只做“把硬件能力抽出来”，不做FOC数学、不做校准逻辑。

### Components（算法/协议）

- `svpwm.[ch]`：输入 `Uα/Uβ` 或 `Ud/Uq + theta_e`，输出三相占空比（0~1）。
- `current_recon.[ch]`：
  - 输入：两路采样电流 + sector/采样相信息
  - 输出：`ia/ib/ic`（第三相用 `-(ia+ib)` 重建）
- `motor_calib.[ch]`：
  - 状态机：对齐/慢速开环旋转/计算 `dir` 与 `zero_offset`
  - 输入：编码器角（rad）、（可选）电流反馈/母线电压
  - 输出：校准结果结构体 `{dir, zero_offset, ...}`

Components 层不直接调用 HAL；输入输出都用纯数据结构，方便单元测试/复用。

### App（编排/调度）

- `MotorApp` 里组合：
  - HostCmd（你这次已有）
  - Encoder 读取
  - PWM 更新
  - ADC 数据获取
  - Calib 状态机推进

App 负责把“命令触发校准”“校准期间禁止闭环”等策略串起来。

---

## 3. 校准：开环电压 vs 电流环（怎么选）

结论建议：**先做 SVPWM + 开环电压校准**（能跑起来、依赖少），等 ADC/电流采样稳定后，再把校准升级成“电流环限流版”。

### A) 开环电压法（推荐先做）

优点：
- 不依赖 ADC，先把“SVPWM + 角度链路 + 编码器”跑通。
- 适合做 “dir 判断” 和 “zero_offset 粗校准”。

风险/注意：
- 不能直接控电流，可能在堵转/负载大时电流偏大；需要把 `Ud_align` / `Uq_spin` 设小，并加超时、温升/电流保护（后续由 ADC 完成）。

典型流程（状态机）：

1) `ALIGN`：施加 `Ud = Ud_align, Uq = 0`，`theta_e_cmd = 0` 固定，持续 `t_align`（例如 300~800ms）
2) 记录对齐后的编码器机械角 `theta_mech_align`
3) `SPIN`：让 `theta_e_cmd += omega_e * dt`（很慢，比如 5~20Hz 电角频率），持续一小段时间（例如 200~500ms）
4) 记录旋转期间编码器角变化 `d_theta_mech`
5) 判断方向：
   - 若 `d_theta_mech` 与 `omega_e` 同号：`dir = +1`
   - 否则 `dir = -1`
6) 计算 `zero_offset`：
   - 目标是在 ALIGN 时：`theta_e_meas = 0`
   - 所以 `zero_offset = wrap_2pi(0 - dir * pole_pairs * theta_mech_align)`

这样即使你换相序/换电机，最终都会吸收到 `dir` 与 `zero_offset` 中。

### B) 电流环校准（后续增强）

等 ADC 能稳定采样两相电流后，把 ALIGN 改成“控 Id”的方式：
- `Id_target > 0`（对齐用）
- `Iq_target = 0`

优点：
- 电流可控，校准更安全、更一致。
- 后续你做闭环FOC，也必须要电流环。

---

## 4. ADC读取方案（G431 两个ADC + 每周期采两路）

### 4.1 你真正需要的信号（建议优先级）

FOC 最小集：
- `ia`、`ib`（第三相 `ic = -(ia+ib)`）
- `vbus`（用于 SVPWM 电压归一化、欠压保护）

可选：
- 温度、母线电流、反电势观测等

### 4.2 触发与模式建议

- 相电流：**Injected conversion**（采样时刻严格、抖动小）
  - 触发源：TIM1 的某个比较事件（通常选在 PWM 中心附近的采样点）
  - ADC1/ADC2 同时采样两路（dual injected 同步）
- 低速量（vbus/温度等）：**Regular conversion + DMA**（例如 1kHz 或更低）

### 4.3 “每周期采哪两路”的算法（3-shunt + 2 ADC 常用做法）

假设是 **低边三电阻采样**（很多电机板都是这种），则在某些扇区/占空比组合下，并不是三相都能在同一时刻可靠采到。

常用策略：每个 PWM 周期只采“两相”，并按 SVM 扇区选择：

- Sector 1：采 `Ia, Ib`
- Sector 2：采 `Ib, Ic`
- Sector 3：采 `Ia, Ic`
- Sector 4：采 `Ia, Ib`
- Sector 5：采 `Ib, Ic`
- Sector 6：采 `Ia, Ic`

第三相用 `-(ia+ib)` 重建（注意要在同一采样时刻对应的相）。

实现上可以把“扇区 -> ADC通道对”的查表放 Components（纯逻辑），BSP 只负责把“选好的通道”写入 ADC 注入序列寄存器。

> 注：具体的可测性跟 PWM 插入的采样点、死区、以及你的电流采样硬件位置有关；上面的表是常见模板，最终要结合 IHM08M1 的电流采样电路确认。

### 4.4 Offset 校准（电流零漂）

建议在上电后做一次：
- 关 PWM（或三相 50% 且不导通，取决于驱动架构）
- 采样 N 次 `Ia/Ib/...`，求平均作为 offset
- 后续实时电流 = ADC_raw - offset

这一步建议作为 `current_sense` 组件的一部分（可复用/可单测）。

---

## 5. 建议你按这个顺序推进（可落地）

1) 做 `svpwm`（哪怕先不闭环）
2) 做 `C1` 校准状态机（先开环电压版）：
   - 能算出 `dir`、`zero_offset` 并打印/保存
3) 开 ADC：
   - 先把 `vbus` 用 regular+DMA 跑通
   - 再上 injected 双 ADC 的两相电流采样
4) 加电流 offset 校准 + 过流保护
5) 上电流环（Id/Iq PI）
6) 把校准升级成“电流环对齐 + 慢速开环旋转确认方向”

---

## 6. 我需要你确认/补充的信息（避免走错路）

1) IHM08M1 上你用的电流采样是：**三电阻低边**？还是两电阻？还是单电阻？
2) 你打算的 PWM 频率/控制频率是多少（例如 20kHz）？
3) 你在 CubeMX 里目前 TIM1 的 PWM 模式是边沿对齐还是中心对齐？
4) ADC 引脚映射：哪几个通道分别是 `Ia/Ib/Ic/Vbus`（如果还没配，给我你想用的引脚/原理图信息也行）
5) 2204 电机的极对数 `pole_pairs` 你是否确定（常见 7，但要以你的电机为准）

### 6.1 已确认（来自你 2026-02-19 的回复）

- 电流采样：IHM08M1 **三电阻低边采样**，采样电阻 `0.01Ω`，电流放大增益约 `5.18`。
- PWM/控制频率：PWM `20kHz`，电流环也计划 `20kHz`（ADC Injected 中断驱动）。
- TIM1：中心对齐 `CenterAligned1`，CH1-CH3 `PWM Mode1`；新增 CH4 `NoOutput` 作为 TRGO2 触发源（OC4 rising，PWM Mode2，pulse=3820）。
- ADC 引脚（相电流）：
  - `Ia`：`PA0` = `ADC1_IN1 / ADC2_IN1`
  - `Ib`：`PC1` = `ADC1_IN7 / ADC2_IN7`
  - `Ic`：`PC0` = `ADC1_IN6 / ADC2_IN6`
- 极对数：`pole_pairs = 7`

### 6.2 “每周期采哪两路”的实现提示

你现在 ADC1/ADC2 都只配了 1 个 Injected rank（每个触发只采 1 路），所以每个 PWM 周期能拿到两路同步电流样本。

- “能否软件切换通道”：可以。思路是在**每次 Injected 转换完成后**（ISR 里）把下一周期要采的通道写进 ADC 的 Injected 序列（JSQR 的 rank1 通道）。
- “用什么算法”：先从 **SVM 扇区 -> (两相采样对)** 的查表开始（最常见：`(A,B)` / `(B,C)` / `(A,C)` 三种组合轮换），再结合你的采样点与最小导通时间做更严谨的可测性判断。

