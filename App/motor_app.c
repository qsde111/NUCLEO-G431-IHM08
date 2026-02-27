#include "motor_app.h"

#include "main.h"

#include <string.h>

/* Control tick is driven by ADC injected interrupt (TIM1 TRGO2) */
#ifndef MOTORAPP_CTRL_HZ
#define MOTORAPP_CTRL_HZ (20000.0f)
#endif

#ifndef MOTORAPP_ADC_MAX_COUNTS
#define MOTORAPP_ADC_MAX_COUNTS (4095.0f)
#endif

#ifndef MOTORAPP_ADC_VREF_V
#define MOTORAPP_ADC_VREF_V (3.3f)
#endif

#ifndef MOTORAPP_CURRENT_SHUNT_OHM
#define MOTORAPP_CURRENT_SHUNT_OHM (0.01f)
#endif

#ifndef MOTORAPP_CURRENT_AMP_GAIN
#define MOTORAPP_CURRENT_AMP_GAIN (5.18f)
#endif

#ifndef MOTORAPP_CURRENT_OFFSET_SAMPLES
#define MOTORAPP_CURRENT_OFFSET_SAMPLES (1000U)
#endif

#ifndef MOTORAPP_VBUS_V
#define MOTORAPP_VBUS_V (12.0f)
#endif

#ifndef MOTORAPP_I_LIMIT_A
#define MOTORAPP_I_LIMIT_A (3.0f)
#endif

#ifndef MOTORAPP_ICTRL_KP
#define MOTORAPP_ICTRL_KP (0.5655f)
#endif

#ifndef MOTORAPP_ICTRL_KI
#define MOTORAPP_ICTRL_KI (703.7f)
#endif

#ifndef MOTORAPP_V_LIMIT_PU
#define MOTORAPP_V_LIMIT_PU (0.57735026919f) /* 1/sqrt(3) */
#endif

#ifndef MOTORAPP_ENCODER_READ_DIV
/* Encoder read rate inside ADC ISR: enc_hz = CTRL_HZ / DIV. DIV=1 -> 20kHz. */
#define MOTORAPP_ENCODER_READ_DIV (1U) /* ADC 中断内的编码器读取频率 */
#endif

// static void MotorApp_OnAdcPair(void *user, uint16_t adc1, uint16_t adc2);

static float MotorApp_Wrap2Pi(float x)
{
    const float two_pi = 6.28318530718f;
    while (x >= two_pi)
    {
        x -= two_pi;
    }
    while (x < 0.0f)
    {
        x += two_pi;
    }
    return x;
}

/* 当前读取的机械角度转换成当前转子的真实电角度*/
static float MotorApp_ElecAngleRad(const MotorApp *ctx)
{
    if (ctx == 0)
    {
        return 0.0f;
    }

    const float theta_e = ((float)ctx->elec_dir * ctx->calib.p.pole_pairs * ctx->pos_mech_rad) + ctx->elec_zero_offset_rad;
    return MotorApp_Wrap2Pi(theta_e);
}

static void MotorApp_CalibStart(MotorApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->calib_done = 0U;
    ctx->calib_fail = 0U;
    ctx->vtest_active = 0U;
    ctx->i_loop_enabled = 0U;
    ctx->i_loop_enable_pending = 0U;
    ctx->iq_ref_a = 0.0f;
    FocCurrentCtrl_Reset(&ctx->i_ctrl);
    (void)BspTim1Pwm_EnableOutputs(&ctx->pwm);
    MotorCalib_Start(&ctx->calib, ctx->pos_mech_rad);
}

static void MotorApp_CalibAbort(MotorApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->vtest_active = 0U;
    ctx->i_loop_enabled = 0U;
    ctx->i_loop_enable_pending = 0U;
    ctx->iq_ref_a = 0.0f;
    FocCurrentCtrl_Reset(&ctx->i_ctrl);
    MotorCalib_Abort(&ctx->calib);
    ctx->calib_done = 0U;
    ctx->calib_fail = 0U;
    (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
}

/* 通过给定ud uq计算三路pwm占空比并改变对应CCR值 */
static void MotorApp_OutputVdqSc(MotorApp *ctx, float ud, float uq, float sin_theta_e, float cos_theta_e)
{
    if (ctx == 0)
    {
        return;
    }

    /* 反Park变换 */
    const float u_alpha = (ud * cos_theta_e) - (uq * sin_theta_e);
    const float u_beta = (ud * sin_theta_e) + (uq * cos_theta_e);

    SvpwmOut out = {0};
    Svpwm_Calc(u_alpha, u_beta, &out);
    ctx->dbg_duty_a = out.duty_a;
    ctx->dbg_duty_b = out.duty_b;
    ctx->dbg_duty_c = out.duty_c;
    BspTim1Pwm_SetDuty(&ctx->pwm, out.duty_a, out.duty_b, out.duty_c);
}

static void MotorApp_HandleHostCmd(MotorApp *ctx, const HostCmd *cmd)
{
    if ((ctx == 0) || (cmd == 0))
    {
        return;
    }

    ctx->last_host_cmd = *cmd;
    ctx->last_host_cmd_tick_ms = HAL_GetTick();

    switch (cmd->op)
    {
    case 'P':
        if (cmd->has_value != 0U)
        {
            ctx->target_pos_deg = cmd->value;
        }
        break;
    case 'V':
        if (cmd->has_value != 0U)
        {
            ctx->target_vel_rad_s = cmd->value;
        }
        break;
    case 'C':
        ctx->calib_request = (cmd->has_value != 0U) ? (uint8_t)cmd->value : 1U;
        ctx->calib_request_pending = 1U;
        break;
    case 'D':
        if (cmd->has_value != 0U)
        {
            ctx->stream_page = (uint8_t)cmd->value;
        }
        else
        {
            ctx->stream_page = (uint8_t)(ctx->stream_page + 1U);
        }
        break;
    case 'I':
        if (cmd->has_value != 0U)
        {
            float iq = cmd->value;
            if (iq > ctx->i_limit_a)
            {
                iq = ctx->i_limit_a;
            }
            if (iq < -ctx->i_limit_a)
            {
                iq = -ctx->i_limit_a;
            }

            ctx->id_ref_a = 0.0f;
            ctx->iq_ref_a = iq;

            ctx->vtest_active = 0U;
            MotorCalib_Abort(&ctx->calib);

            if (ctx->i_loop_enabled == 0U)
            {
                ctx->i_loop_enable_pending = 1U;
                (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
            }
        }
        else
        {
            ctx->i_loop_enable_pending = 0U;
            ctx->i_loop_enabled = 0U;
            ctx->iq_ref_a = 0.0f;
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
        }
        break;

        /* 将转子强拖到U相 */
    case 'T':
        if ((cmd->has_value != 0U) && (cmd->value == 0.0f))
        {
            ctx->vtest_active = 0U;
            ctx->i_loop_enable_pending = 0U;
            ctx->i_loop_enabled = 0U;
            ctx->iq_ref_a = 0.0f;
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
        }
        else
        {
            float ud = (cmd->has_value != 0U) ? cmd->value : 0.05f;
            if (ud > 0.2f)
            {
                ud = 0.2f;
            }
            if (ud < -0.2f)
            {
                ud = -0.2f;
            }

            ctx->vtest_ud = ud;
            ctx->vtest_uq = 0.0f;
            ctx->vtest_active = 1U;
            ctx->i_loop_enable_pending = 0U;
            ctx->i_loop_enabled = 0U;
            ctx->iq_ref_a = 0.0f;
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            (void)BspTim1Pwm_EnableOutputs(&ctx->pwm);
        }
        break;
    default:
        break;
    }
}

/* 控制 tick（约 20kHz）入口：由 ADC injected 转换完成回调触发。
 * - 采样电流（带 U/V/W offset 两阶段校准）
 * - 推进校准状态机（C1）/ 电流环（I）/ 电压测试（T）
 * - 计算并输出 SVPWM 占空比 */
static void MotorApp_OnAdcPair(void *user, uint16_t adc1, uint16_t adc2)
{
    MotorApp *ctx = (MotorApp *)user;
    if (ctx == 0)
    {
        return;
    }

    /* ISR profiling pulse on PC8 (S_Pin): high at entry, low at exit */
    S_GPIO_Port->BSRR = (uint32_t)S_Pin;

    ctx->adc1_raw = adc1;
    ctx->adc2_raw = adc2;
    ctx->adc_isr_count++;

/* ISR中断内编码器读取分频 */
#if (MOTORAPP_ENCODER_READ_DIV > 0U)
    if (ctx->enc_div_countdown == 0U)
    {
        ctx->raw21 = Mt6835_ReadRaw21(&ctx->encoder);
        ctx->pos_mech_rad = Mt6835_Raw21ToRad(ctx->raw21);
        ctx->enc_div_countdown = (uint16_t)(MOTORAPP_ENCODER_READ_DIV - 1U);
    }
    else
    {
        ctx->enc_div_countdown--;
    }
#endif

    /* 电流零偏校准状态机，先校准U V相电流采样的零偏，后切换状态校准W相 */
    if ((ctx->i_offset_ready == 0U) && (ctx->pwm.outputs_enabled == 0U))
    {
        CurrentSenseOffset2_Push(&ctx->i_ab_offset, adc1, adc2);

        if (CurrentSenseOffset2_Ready(&ctx->i_ab_offset) != 0U)
        {
            if (ctx->i_offset_stage == 0U)
            {
                ctx->i_u_offset_raw = CurrentSenseOffset2_OffsetA(&ctx->i_ab_offset);
                ctx->i_v_offset_raw = CurrentSenseOffset2_OffsetB(&ctx->i_ab_offset);

                ctx->i_offset_stage = 1U;
                CurrentSenseOffset2_Init(&ctx->i_ab_offset, MOTORAPP_CURRENT_OFFSET_SAMPLES);
                /* Sample U+W to calibrate W offset too (future dynamic 2-shunt sampling support) */
                BspAdcInjPair_SetRank1Channels(&ctx->adc_inj, LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_6);
            }
            else if (ctx->i_offset_stage == 1U)
            {
                ctx->i_w_offset_raw = CurrentSenseOffset2_OffsetB(&ctx->i_ab_offset);

                ctx->i_offset_stage = 2U;
                ctx->i_offset_ready = 1U;
                /* Back to default U+V sampling */
                BspAdcInjPair_SetRank1Channels(&ctx->adc_inj, LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_7);
            }
        }
    }

    /* 原始数据转实际物理电流（安培） */
    if (ctx->i_offset_ready != 0U)
    {
        ctx->ia_a = -CurrentSense_RawToCurrentA(adc1, ctx->i_u_offset_raw, MOTORAPP_ADC_MAX_COUNTS, MOTORAPP_ADC_VREF_V,
                                                MOTORAPP_CURRENT_SHUNT_OHM, MOTORAPP_CURRENT_AMP_GAIN);
        ctx->ib_a = -CurrentSense_RawToCurrentA(adc2, ctx->i_v_offset_raw, MOTORAPP_ADC_MAX_COUNTS, MOTORAPP_ADC_VREF_V,
                                                MOTORAPP_CURRENT_SHUNT_OHM, MOTORAPP_CURRENT_AMP_GAIN);
        ctx->ic_a = -(ctx->ia_a + ctx->ib_a);
    }
    else
    {
        ctx->ia_a = 0.0f;
        ctx->ib_a = 0.0f;
        ctx->ic_a = 0.0f;
    }

    /* 磁极校准时的状态机流转和校准参数计算 */
    MotorCalib_Tick(&ctx->calib, 1.0f / MOTORAPP_CTRL_HZ, ctx->pos_mech_rad);

    MotorCalibCmd cmd = {0}; // 磁极校准阶段给定Uq Ud 电角度数值缓冲区
    const uint8_t have_cmd = MotorCalib_GetCmd(&ctx->calib, &cmd);
    ctx->dbg_ud = cmd.ud;
    ctx->dbg_uq = cmd.uq;
    ctx->dbg_theta_e = cmd.theta_e;
    ctx->dbg_calib_state = (uint8_t)MotorCalib_State(&ctx->calib);

    /* 处于校准状态 */
    if (have_cmd != 0U)
    {
        float s = 0.0f;
        float c = 1.0f;
        /* 处于校准状态，计算当前设定角度的 sin 和 cos */
        BspTrig_SinCos(cmd.theta_e, &s, &c);

        MotorApp_OutputVdqSc(ctx, cmd.ud, cmd.uq, s, c);
    }
    else if ((ctx->i_loop_enabled != 0U) && (ctx->i_offset_ready != 0U))
    {
        const float theta_e = MotorApp_ElecAngleRad(ctx);
        float s = 0.0f;
        float c = 1.0f;
        BspTrig_SinCos(theta_e, &s, &c);

        FocCurrentCtrlOut iout = {0};
        FocCurrentCtrl_StepSc(&ctx->i_ctrl, ctx->ia_a, ctx->ib_a, ctx->ic_a, s, c, ctx->id_ref_a, ctx->iq_ref_a, &iout);

        ctx->dbg_theta_e = theta_e;
        ctx->dbg_ud = iout.ud_pu;
        ctx->dbg_uq = iout.uq_pu;
        ctx->dbg_id_a = iout.id_a;
        ctx->dbg_iq_a = iout.iq_a;
        ctx->dbg_ud_pu = iout.ud_pu;
        ctx->dbg_uq_pu = iout.uq_pu;

        MotorApp_OutputVdqSc(ctx, iout.ud_pu, iout.uq_pu, s, c);
    }
    else if (ctx->vtest_active != 0U)
    {
        MotorApp_OutputVdqSc(ctx, ctx->vtest_ud, ctx->vtest_uq, 0.0f, 1.0f);
    }
    else if (ctx->pwm.outputs_enabled != 0U)
    {
        BspTim1Pwm_SetNeutral(&ctx->pwm);
    }

    const MotorCalibState st = MotorCalib_State(&ctx->calib);
    if ((st == MOTOR_CALIB_DONE) && (ctx->calib_done == 0U))
    {
        ctx->elec_dir = MotorCalib_Dir(&ctx->calib);
        ctx->elec_zero_offset_rad = MotorCalib_ZeroOffsetRad(&ctx->calib);
        ctx->calib_done = 1U;
        (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
    }
    else if ((st == MOTOR_CALIB_FAIL) && (ctx->calib_fail == 0U))
    {
        ctx->calib_fail = 1U;
        (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
    }

    S_GPIO_Port->BSRR = (uint32_t)S_Pin << 16U;
}

void MotorApp_Init(MotorApp *ctx, UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   TIM_HandleTypeDef *htim_pwm, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{
    if (ctx == 0)
    {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));

    BspTrig_Init();

    CurrentSenseOffset2_Init(&ctx->i_ab_offset, MOTORAPP_CURRENT_OFFSET_SAMPLES);
    ctx->i_offset_stage = 0U;
    ctx->i_offset_ready = 0U;
    ctx->i_u_offset_raw = 0U;
    ctx->i_v_offset_raw = 0U;
    ctx->i_w_offset_raw = 0U;

    ctx->enc_div_countdown = 0U;

    ctx->i_limit_a = MOTORAPP_I_LIMIT_A;
    ctx->id_ref_a = 0.0f;
    ctx->iq_ref_a = 0.0f;
    ctx->i_loop_enabled = 0U;
    ctx->i_loop_enable_pending = 0U;
    FocCurrentCtrl_Init(&ctx->i_ctrl, MOTORAPP_ICTRL_KP, MOTORAPP_ICTRL_KI, 1.0f / MOTORAPP_CTRL_HZ, MOTORAPP_VBUS_V,
                        MOTORAPP_V_LIMIT_PU);

    BspUartDma_Init(&ctx->uart, huart);
    HostCmdApp_Init(&ctx->host_cmd, huart);
    (void)HostCmdApp_Start(&ctx->host_cmd);

    BspTim1Pwm_Init(&ctx->pwm, htim_pwm);
    (void)BspTim1Pwm_StartTrigger(&ctx->pwm);

    BspAdcInjPair_Init(&ctx->adc_inj, hadc1, hadc2);
    BspAdcInjPair_RegisterCallback(&ctx->adc_inj, ctx, MotorApp_OnAdcPair);
    BspAdcInjPair_SetRank1Channels(&ctx->adc_inj, LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_7);
    (void)BspAdcInjPair_Start(&ctx->adc_inj);

    BspSpi3Fast_Init(&ctx->spi, hspi, cs_port, cs_pin);

    Mt6835BusOps bus = {
        .user = &ctx->spi,
        .transfer8 = BspSpi3Fast_Transfer8,
        .cs_low = BspSpi3Fast_CsLow,
        .cs_high = BspSpi3Fast_CsHigh,
    };
    Mt6835_Init(&ctx->encoder, &bus);

    /* 磁极校准参数配置台 */
    MotorCalibParams calib = {
        .pole_pairs = 7.0f,                                 // 极对数
        .ud_align = 0.1f,                                   // 对齐阶段d轴电压
        .uq_spin = 0.07f,                                   // 开环拖动阶段q轴电压
        .omega_e_rad_s = 31.4159265f,                       /* 开环拖动时的电角速度 2*pi*5Hz */
        .align_ticks = (uint32_t)(MOTORAPP_CTRL_HZ * 0.5f), // 对齐阶段保持时间
        .spin_ticks = (uint32_t)(MOTORAPP_CTRL_HZ * 0.3f),  // 拖动阶段持续时间
        .min_move_rad = 0.2f,                               // 拖动阶段最小机械移动角度
    };
    MotorCalib_Init(&ctx->calib, &calib);
    ctx->elec_dir = 1;
    ctx->elec_zero_offset_rad = 0.0f;

    ctx->last_stream_tick_ms = HAL_GetTick();
    ctx->stream_page = 0U;
    ctx->vtest_active = 0U;
    ctx->vtest_ud = 0.0f;
    ctx->vtest_uq = 0.0f;
}

void MotorApp_Loop(MotorApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    HostCmdApp_Loop(&ctx->host_cmd);
    HostCmd cmd = {0};
    while (HostCmdApp_Pop(&ctx->host_cmd, &cmd) != 0U)
    {
        MotorApp_HandleHostCmd(ctx, &cmd);
    }

    if (ctx->calib_request_pending != 0U)
    {
        ctx->calib_request_pending = 0U;
        if (ctx->calib_request == 0U)
        {
            MotorApp_CalibAbort(ctx);
        }
        else
        {
            MotorApp_CalibStart(ctx);
        }
    }

    if (ctx->i_loop_enable_pending != 0U)
    {
        if (ctx->i_offset_ready != 0U)
        {
            ctx->i_loop_enable_pending = 0U;
            ctx->i_loop_enabled = 1U;
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            (void)BspTim1Pwm_EnableOutputs(&ctx->pwm);
        }
    }

    const uint32_t now = HAL_GetTick();
    if (now == ctx->last_stream_tick_ms)
    {
        return;
    }
    ctx->last_stream_tick_ms = now;

    if (ctx->stream_page == 1U)
    {
        if (ctx->i_offset_ready == 0U)
        {
            const float n = (ctx->i_ab_offset.sample_count != 0U) ? (float)ctx->i_ab_offset.sample_count : 1.0f;
            const float avg_a = (float)ctx->i_ab_offset.sum_a / n;
            const float avg_b = (float)ctx->i_ab_offset.sum_b / n;
            JustFloat_Pack4((float)ctx->adc1_raw, (float)ctx->adc2_raw, avg_a, avg_b, ctx->tx_frame);
        }
        else
        {
            /* a、b路采样电流，和a、b路采样电流的零偏值raw */
            JustFloat_Pack4(ctx->ia_a, ctx->ib_a, (float)ctx->i_u_offset_raw, (float)ctx->i_v_offset_raw, ctx->tx_frame);
        }
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 2U)
    {
        if (ctx->i_offset_ready == 0U)
        {
            const float n = (ctx->i_ab_offset.sample_count != 0U) ? (float)ctx->i_ab_offset.sample_count : 1.0f;
            const float avg_a = (float)ctx->i_ab_offset.sum_a / n;
            const float avg_b = (float)ctx->i_ab_offset.sum_b / n;
            JustFloat_Pack4((float)ctx->i_offset_stage, (float)ctx->i_ab_offset.sample_count, avg_a, avg_b, ctx->tx_frame);
        }
        else
        {
            /* U V W相电流采样零偏值 */
            JustFloat_Pack4((float)ctx->i_u_offset_raw, (float)ctx->i_v_offset_raw, (float)ctx->i_w_offset_raw, 1.0f,
                            ctx->tx_frame);
        }
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 3U)
    {
        /* FOC 电流环闭环监控：D轴实际电流，Q轴实际电流，D轴输出电压，Q轴输出电压 */
        JustFloat_Pack4(ctx->dbg_id_a, ctx->dbg_iq_a, ctx->dbg_ud_pu, ctx->dbg_uq_pu, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    const uint8_t calib_running =
        (ctx->dbg_calib_state == (uint8_t)MOTOR_CALIB_ALIGN) || (ctx->dbg_calib_state == (uint8_t)MOTOR_CALIB_SPIN);
    if (calib_running != 0U)
    {
        ctx->tx_debug_toggle ^= 1U;
        if (ctx->tx_debug_toggle != 0U)
        {
            JustFloat_Pack4(ctx->dbg_duty_a, ctx->dbg_duty_b, ctx->dbg_duty_c, (float)ctx->dbg_calib_state, ctx->tx_frame);
        }
        else
        {
            JustFloat_Pack4((float)ctx->raw21, ctx->pos_mech_rad, ctx->elec_zero_offset_rad, (float)ctx->elec_dir,
                            ctx->tx_frame);
        }
    }
    else
    {
        /* 非磁极校准状态：编码器原始值，转子机械角度，电角度零点，编码器方向 */
        JustFloat_Pack4((float)ctx->raw21, ctx->pos_mech_rad, ctx->elec_zero_offset_rad, (float)ctx->elec_dir, ctx->tx_frame);
    }
    (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
}
