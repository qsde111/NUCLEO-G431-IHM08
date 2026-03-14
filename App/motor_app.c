#include "motor_app.h"

#include "main.h"

#include "iq_lut_comp.h"
#include "signal_config.h"

#include <math.h>
#include <string.h>

/* Control tick is driven by ADC injected interrupt (TIM1 TRGO2) */
#ifndef MOTORAPP_CTRL_HZ
#define MOTORAPP_CTRL_HZ (20000.0f)
#endif

#ifndef MOTORAPP_STREAM_USE_ISR_DIV
/* 0: use HAL_GetTick() (1kHz max). 1: use ADC ISR divider (supports 2k/5kHz, etc). */
#define MOTORAPP_STREAM_USE_ISR_DIV (1U)
#endif

#ifndef MOTORAPP_STREAM_DIV
/* Stream rate when MOTORAPP_STREAM_USE_ISR_DIV=1: stream_hz = CTRL_HZ / DIV. DIV=10->2kHz, DIV=4->5kHz. */
#define MOTORAPP_STREAM_DIV (10U)
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

#ifndef MOTORAPP_CURRENT_MIN_WINDOW_TICKS
#define MOTORAPP_CURRENT_MIN_WINDOW_TICKS (420U)
#endif

#ifndef MOTORAPP_CURRENT_OFFSET_SAMPLES
#define MOTORAPP_CURRENT_OFFSET_SAMPLES (1000U)
#endif

#ifndef MOTORAPP_VBUS_V
#define MOTORAPP_VBUS_V (12.0f)
#endif

#ifndef MOTORAPP_VBUS_DIV
/* Vbus = Vadc * MOTORAPP_VBUS_DIV (e.g. 12V -> ~0.626V with divider ~19.15) */
#define MOTORAPP_VBUS_DIV (19.15f)
#endif

#ifndef MOTORAPP_VBUS_SAMPLE_MS
#define MOTORAPP_VBUS_SAMPLE_MS (10U)
#endif

#ifndef MOTORAPP_VBUS_FILTER_ALPHA
#define MOTORAPP_VBUS_FILTER_ALPHA (0.1f)
#endif

#ifndef MOTORAPP_I_LIMIT_A
#define MOTORAPP_I_LIMIT_A (3.0f)
#endif

#ifndef MOTORAPP_I_TRIP_A
#define MOTORAPP_I_TRIP_A (6.0f)
#endif

#ifndef MOTORAPP_ICTRL_KP
#define MOTORAPP_ICTRL_KP (0.1131f)
#endif

#ifndef MOTORAPP_ICTRL_KI
#define MOTORAPP_ICTRL_KI (1407.4f)
#endif

/* 反电动势前馈使能开关 */
#ifndef MOTORAPP_BEMF_FF_ENABLE
#define MOTORAPP_BEMF_FF_ENABLE (1U)
#endif

#ifndef MOTORAPP_BEMF_KE_V_PER_RAD_S
/* Ke from Kv: Ke = 60 / (2*pi*Kv_rpm_per_V)  (units: V / (rad/s)) */
#define MOTORAPP_BEMF_KE_V_PER_RAD_S (0.00415f)
#endif

#ifndef MOTORAPP_SPEED_LOOP_DIV
/* Speed loop update rate inside ADC ISR: spd_hz = CTRL_HZ / DIV. DIV=20 -> 1kHz. */
#define MOTORAPP_SPEED_LOOP_DIV (20U)
#endif

#ifndef MOTORAPP_SPDCTRL_KP
#define MOTORAPP_SPDCTRL_KP (0.105376f)
#endif

#ifndef MOTORAPP_SPDCTRL_KI
#define MOTORAPP_SPDCTRL_KI (6.620959f)
#endif

// #ifndef MOTORAPP_SPDCTRL_KP
// #define MOTORAPP_SPDCTRL_KP (0.105376f)
// #endif

// #ifndef MOTORAPP_SPDCTRL_KI
// #define MOTORAPP_SPDCTRL_KI (5.296767f)
// #endif

#ifndef MOTORAPP_SCTRL_IQ_LIMIT_A
#define MOTORAPP_SCTRL_IQ_LIMIT_A (2.0f)
#endif

/* 速度指令给定最小阈值，防止模拟信号波动启动速度环 */
#ifndef MOTORAPP_SCTRL_STOP_EPS
#define MOTORAPP_SCTRL_STOP_EPS (0.01f)
#endif

/* 速度指令限制 */
#ifndef MOTORAPP_SCTRL_OMEGA_LIMIT_RAD_S
#define MOTORAPP_SCTRL_OMEGA_LIMIT_RAD_S (1600.0f)
#endif

/* Iq LUT 补偿使能开关（前馈）：低速实验用（查表 + 线性插值） */
#ifndef MOTORAPP_IQ_LUT_COMP_ENABLE
#define MOTORAPP_IQ_LUT_COMP_ENABLE (0U)
#endif

#ifndef MOTORAPP_IQ_LUT_COMP_GAIN
#define MOTORAPP_IQ_LUT_COMP_GAIN (1.0f)
#endif

#ifndef MOTORAPP_IQ_LUT_COMP_MIN_OMEGA_RAD_S
#define MOTORAPP_IQ_LUT_COMP_MIN_OMEGA_RAD_S (5.0f)
#endif

#ifndef MOTORAPP_IQ_LUT_COMP_MAX_OMEGA_RAD_S
#define MOTORAPP_IQ_LUT_COMP_MAX_OMEGA_RAD_S (30.0f)
#endif

#ifndef MOTORAPP_IQ_LUT_COMP_LIMIT_A
#define MOTORAPP_IQ_LUT_COMP_LIMIT_A (1.0f)
#endif

/* S-Curve轨迹规划宏开关 */
#ifndef MOTORAPP_SPD_REF_S_CURVE_ENABLE
#define MOTORAPP_SPD_REF_S_CURVE_ENABLE (1U)
#endif

#ifndef MOTORAPP_SPD_REF_A_MAX_RAD_S2
#define MOTORAPP_SPD_REF_A_MAX_RAD_S2 (500.0f)
#endif

#ifndef MOTORAPP_SPD_REF_J_MAX_RAD_S3
#define MOTORAPP_SPD_REF_J_MAX_RAD_S3 (5000.0f)
#endif

/* 轨迹发生器作为P控制器，速度差->期望加速度 比例系数 */
#ifndef MOTORAPP_SPD_REF_K_A
#define MOTORAPP_SPD_REF_K_A (2.0f)
#endif

/* 电流环输出，电压矢量限幅(母线电压标幺值) */
#ifndef MOTORAPP_V_LIMIT_PU
#define MOTORAPP_V_LIMIT_PU (0.57735026919f) /* 1/sqrt(3) */
#endif

#ifndef MOTORAPP_SPD_PLL_KP
#define MOTORAPP_SPD_PLL_KP (1005.0f)
#endif

#ifndef MOTORAPP_SPD_PLL_KI
#define MOTORAPP_SPD_PLL_KI (252662.0f)
#endif

/* MT835系统带宽寄存器地址 */
#ifndef MOTORAPP_MT6835_REG_BW_ADDR
#define MOTORAPP_MT6835_REG_BW_ADDR (0x011U)
#endif

#ifndef MOTORAPP_MT6835_REG_BW_BITS
#define MOTORAPP_MT6835_REG_BW_BITS (7U)
#endif

#ifndef MOTORAPP_MT6835_EEPROM_QUIET_S
#define MOTORAPP_MT6835_EEPROM_QUIET_S (8.0f)
#endif

#define MOTORAPP_MT6835_EEPROM_QUIET_TICKS ((uint32_t)(MOTORAPP_CTRL_HZ * MOTORAPP_MT6835_EEPROM_QUIET_S))

#ifndef MOTORAPP_ENCODER_READ_DIV
/* Encoder read rate inside ADC ISR: enc_hz = CTRL_HZ / DIV. DIV=1 -> 20kHz. */
#define MOTORAPP_ENCODER_READ_DIV (1U) /* ADC 中断内的编码器读取频率 */
#endif

#ifndef MOTORAPP_THETA_CTRL_PREDICT_ENABLE
/* 1: theta_ctrl = theta_meas + omega_e * Tcomp, only for current loop control frame. */
#define MOTORAPP_THETA_CTRL_PREDICT_ENABLE (1U)
#endif

#ifndef MOTORAPP_THETA_CTRL_TCOMP_SCALE
/* Fine-tune around one encoder pipeline period; 1.0f means full encoder period compensation. */
#define MOTORAPP_THETA_CTRL_TCOMP_SCALE (0.9f)
#endif

#ifndef MOTORAPP_THETA_CTRL_TCOMP_S
#define MOTORAPP_THETA_CTRL_TCOMP_S                                                                                            \
    (MOTORAPP_THETA_CTRL_TCOMP_SCALE * (((float)MOTORAPP_ENCODER_READ_DIV) * (1.0f / MOTORAPP_CTRL_HZ)))
#endif

// static void MotorApp_OnAdcPair(void *user, uint16_t adc1, uint16_t adc2);

static volatile uint32_t g_mt6835_quiet_ticks = 0U;

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

static float MotorApp_WrapPi(float x)
{
    const float pi = 3.14159265359f;
    const float two_pi = 6.28318530718f;
    while (x >= pi)
    {
        x -= two_pi;
    }
    while (x < -pi)
    {
        x += two_pi;
    }
    return x;
}

/* 机械角度转换为双变换坐标系下的电角度 */
static float MotorApp_ElecAngleFromMechRad(const MotorApp *ctx, float mech_rad)
{
    if (ctx == 0)
    {
        return 0.0f;
    }

    const float theta_e = ((float)ctx->elec_dir * ctx->calib.p.pole_pairs * mech_rad) + ctx->elec_zero_offset_rad;
    return MotorApp_Wrap2Pi(theta_e);
}

/* 电角度前馈补偿值 */
static float MotorApp_ThetaCtrlDeltaRad(const MotorApp *ctx)
{
    if (ctx == 0)
    {
        return 0.0f;
    }

#if (MOTORAPP_THETA_CTRL_PREDICT_ENABLE != 0U)
    const float omega_e_rad_s = ctx->calib.p.pole_pairs * ctx->dbg_omega_pll_rad_s;
    return omega_e_rad_s * MOTORAPP_THETA_CTRL_TCOMP_S;
#else
    return 0.0f;
#endif
}

/* 计算经过前馈预测补偿后的控制用电角度 */
static float MotorApp_ElecAngleCtrlRad(const MotorApp *ctx)
{
    if (ctx == 0)
    {
        return 0.0f;
    }

    /* 计算当前的实测电角度（包含机械偏置和极对数转换）*/
    const float theta_e_meas = MotorApp_ElecAngleFromMechRad(ctx, ctx->pos_mech_rad);

    /* 在实测值基础上累加前馈补偿量，并进行 2Pi 归一化 */
    return MotorApp_Wrap2Pi(theta_e_meas + MotorApp_ThetaCtrlDeltaRad(ctx));
}

static uint8_t MotorApp_Mt6835QuietActive(void)
{
    return (g_mt6835_quiet_ticks != 0U) ? 1U : 0U;
}

static void MotorApp_CalibStart(MotorApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->spd_loop_enabled = 0U;
    ctx->target_vel_rad_s = 0.0f;
    FocSpeedCtrl_Reset(&ctx->spd_ctrl);
    ctx->spd_loop_div_countdown = 0U;
    SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
    SignalLogSweep_Reset(&ctx->iq_sweep);
    ctx->iq_sweep_request_pending = 0U;
    ctx->iq_sweep_div_countdown = 0U;
    ctx->iq_sweep_a = 0.0f;

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

    ctx->spd_loop_enabled = 0U;
    ctx->target_vel_rad_s = 0.0f;
    FocSpeedCtrl_Reset(&ctx->spd_ctrl);
    ctx->spd_loop_div_countdown = 0U;
    SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
    SignalLogSweep_Reset(&ctx->iq_sweep);
    ctx->iq_sweep_request_pending = 0U;
    ctx->iq_sweep_div_countdown = 0U;
    ctx->iq_sweep_a = 0.0f;

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

/* 将逻辑枚举(AB/AC)翻译成真实的 ADC Channel 宏 */
static void MotorApp_MapCurrentPairChannels(CurrentSensePair pair, uint32_t *adc1_ch, uint32_t *adc2_ch)
{
    if ((adc1_ch == 0) || (adc2_ch == 0))
    {
        return;
    }

    switch (pair)
    {
    case CURRENT_SENSE_PAIR_AB:
        *adc1_ch = LL_ADC_CHANNEL_1;
        *adc2_ch = LL_ADC_CHANNEL_7;
        break;
    case CURRENT_SENSE_PAIR_AC:
        *adc1_ch = LL_ADC_CHANNEL_1;
        *adc2_ch = LL_ADC_CHANNEL_6;
        break;
    case CURRENT_SENSE_PAIR_BC:
        *adc1_ch = LL_ADC_CHANNEL_7;
        *adc2_ch = LL_ADC_CHANNEL_6;
        break;
    default:
        *adc1_ch = LL_ADC_CHANNEL_1;
        *adc2_ch = LL_ADC_CHANNEL_7;
        break;
    }
}

/* 将新选出的 pair 更新到 ADC 硬件通道映射，并覆盖 ctx->i_pair_active */
static void MotorApp_ProgramCurrentPair(MotorApp *ctx, CurrentSensePair pair)
{
    uint32_t adc1_ch = 0U;
    uint32_t adc2_ch = 0U;

    if (ctx == 0)
    {
        return;
    }

    MotorApp_MapCurrentPairChannels(pair, &adc1_ch, &adc2_ch);
    BspAdcInjPair_SetRank1Channels(&ctx->adc_inj, adc1_ch, adc2_ch);

    /* 把配好的 pair 存进全局变量 */
    ctx->i_pair_active = pair;
}

/* 通过给定ud uq计算三路pwm占空比并改变对应CCR值 */
static void MotorApp_OutputVdqSc(MotorApp *ctx, float ud, float uq, float sin_theta_e, float cos_theta_e)
{
    if (ctx == 0)
    {
        return;
    }

    /* Inverse Park Transform */
    const float u_alpha = (ud * cos_theta_e) - (uq * sin_theta_e);
    const float u_beta = (ud * sin_theta_e) + (uq * cos_theta_e);

    SvpwmOut out = {0};
    CurrentSense3ShuntDecision current_decision = {0};
    Svpwm_Calc(u_alpha, u_beta, &out);
    ctx->dbg_duty_a = out.duty_a;
    ctx->dbg_duty_b = out.duty_b;
    ctx->dbg_duty_c = out.duty_c;
    CurrentSense3Shunt_SelectPair(out.duty_a, out.duty_b, out.duty_c, ctx->pwm.period, MOTORAPP_CURRENT_MIN_WINDOW_TICKS,
                                  ctx->i_pair_active, &current_decision);
    ctx->dbg_svm_sector = out.sector;
    ctx->dbg_i_pair_next = (uint8_t)current_decision.pair;
    ctx->dbg_i_pair_valid = current_decision.pair_valid;
    ctx->dbg_i_valid_mask = current_decision.valid_mask;
    ctx->dbg_i_low_window_a_ticks = current_decision.low_window_a_ticks;
    ctx->dbg_i_low_window_b_ticks = current_decision.low_window_b_ticks;
    ctx->dbg_i_low_window_c_ticks = current_decision.low_window_c_ticks;
    BspTim1Pwm_SetDuty(&ctx->pwm, out.duty_a, out.duty_b, out.duty_c);
    MotorApp_ProgramCurrentPair(ctx, current_decision.pair);
}

static void MotorApp_HandleHostCmd(MotorApp *ctx, const HostCmd *cmd)
{
    if ((ctx == 0) || (cmd == 0))
    {
        return;
    }

    ctx->last_host_cmd = *cmd;
    ctx->last_host_cmd_tick_ms = HAL_GetTick();

    const uint8_t mt6835_quiet = MotorApp_Mt6835QuietActive();

    switch (cmd->op)
    {
    case 'P':
        if (cmd->has_value != 0U)
        {
            ctx->target_pos_deg = cmd->value;
        }
        break;
    case 'V':
        if (mt6835_quiet != 0U)
        {
            break;
        }
        if ((cmd->has_value != 0U) && (fabsf(cmd->value) > MOTORAPP_SCTRL_STOP_EPS))
        {
            if (ctx->fault_overcurrent != 0U)
            {
                /* Overcurrent fault latched: ignore enable requests until user stops/clears. */
                ctx->i_loop_enable_pending = 0U;
                ctx->i_loop_enabled = 0U;
                ctx->spd_loop_enabled = 0U;
                ctx->target_vel_rad_s = 0.0f;
                ctx->iq_ref_a = 0.0f;
                FocSpeedCtrl_Reset(&ctx->spd_ctrl);
                SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
                (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
                break;
            }

            float omega = cmd->value;
            if (omega > MOTORAPP_SCTRL_OMEGA_LIMIT_RAD_S)
            {
                omega = MOTORAPP_SCTRL_OMEGA_LIMIT_RAD_S;
            }
            if (omega < -MOTORAPP_SCTRL_OMEGA_LIMIT_RAD_S)
            {
                omega = -MOTORAPP_SCTRL_OMEGA_LIMIT_RAD_S;
            }

            const uint8_t restart_speed_path = ((ctx->spd_loop_enabled == 0U) || (ctx->i_loop_enabled == 0U)) ? 1U : 0U;

            ctx->target_vel_rad_s = omega;
            ctx->spd_loop_div_countdown = 0U;
            SignalLogSweep_Reset(&ctx->iq_sweep);

            if (restart_speed_path != 0U)
            {
                /* Keep the planner/PI continuous across target changes.
                 * Re-seed only when speed mode is entered from a stopped state. */
                FocSpeedCtrl_Reset(&ctx->spd_ctrl);
                SCurveVel_Reset(&ctx->spd_ref_plan, ctx->dbg_omega_pll_rad_s);
                ctx->id_ref_a = 0.0f;
                ctx->iq_ref_a = 0.0f;
            }

            /* 所有状态重置完毕后，最后再放开中断权限 */
            ctx->spd_loop_enabled = 1U;

            ctx->iq_sweep_request_pending = 0U;
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;

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
            /* Stop speed loop and disable outputs. */
            ctx->i_loop_enable_pending = 0U;
            ctx->i_loop_enabled = 0U;
            ctx->spd_loop_enabled = 0U;
            ctx->target_vel_rad_s = 0.0f;
            ctx->iq_ref_a = 0.0f;
            ctx->spd_loop_div_countdown = 0U;
            FocSpeedCtrl_Reset(&ctx->spd_ctrl);
            SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
            SignalLogSweep_Reset(&ctx->iq_sweep);
            ctx->iq_sweep_request_pending = 0U;
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
        }
        break;
    case 'C':
        if (mt6835_quiet != 0U)
        {
            break;
        }
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
        if (mt6835_quiet != 0U)
        {
            break;
        }
        if (cmd->has_value != 0U)
        {
            if (ctx->fault_overcurrent != 0U)
            {
                /* Overcurrent fault latched: ignore enable requests until user stops/clears. */
                ctx->i_loop_enable_pending = 0U;
                ctx->i_loop_enabled = 0U;
                ctx->spd_loop_enabled = 0U;
                ctx->target_vel_rad_s = 0.0f;
                ctx->iq_ref_a = 0.0f;
                ctx->spd_loop_div_countdown = 0U;
                FocSpeedCtrl_Reset(&ctx->spd_ctrl);
                SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
                (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
                break;
            }

            /* Direct current command: leave speed loop mode. */
            ctx->spd_loop_enabled = 0U;
            ctx->target_vel_rad_s = 0.0f;
            ctx->spd_loop_div_countdown = 0U;
            FocSpeedCtrl_Reset(&ctx->spd_ctrl);
            SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
            SignalLogSweep_Reset(&ctx->iq_sweep);
            ctx->iq_sweep_request_pending = 0U;
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;

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
            ctx->spd_loop_enabled = 0U;
            ctx->target_vel_rad_s = 0.0f;
            ctx->iq_ref_a = 0.0f;
            ctx->spd_loop_div_countdown = 0U;
            FocSpeedCtrl_Reset(&ctx->spd_ctrl);
            SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
            SignalLogSweep_Reset(&ctx->iq_sweep);
            ctx->iq_sweep_request_pending = 0U;
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            ctx->fault_overcurrent = 0U;
            (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
        }
        break;

    case 'M':
    {
        /* M0: read reg 0x011, M1: write BW[2:0], M2: EEPROM burn */
        uint8_t subcmd = 0U;
        if (cmd->has_value != 0U)
        {
            if (cmd->value >= 1.5f)
            {
                subcmd = 2U;
            }
            else if (cmd->value >= 0.5f)
            {
                subcmd = 1U;
            }
        }

        if (mt6835_quiet != 0U)
        {
            ctx->mt6835_reg_op = 6U;
            ctx->stream_page = 4U;
            break;
        }

        /* Pause encoder DMA pipeline to avoid SPI3 bus conflict with blocking register access. */
        ctx->enc_dma_enable = 0U;
        const uint32_t t0 = HAL_GetTick();

        /* 5ms 等待总线空闲，超时标记数据无效，打印错误码 */
        while ((ctx->enc_dma.busy != 0U) && ((HAL_GetTick() - t0) < 5U))
        {
        }
        if (ctx->enc_dma.busy != 0U)
        {
            ctx->mt6835_reg011_valid = 0U; // 标记数据无效
            ctx->mt6835_reg_op = 3U;
            ctx->enc_dma_enable = 1U;
            ctx->stream_page = 4U;
            break;
        }

        if (subcmd == 2U)
        {
            uint8_t ack = 0U;
            const uint8_t ok = Mt6835_BurnEeprom(&ctx->encoder, &ack);

            ctx->i_loop_enable_pending = 0U;
            ctx->i_loop_enabled = 0U;
            ctx->spd_loop_enabled = 0U;
            ctx->target_vel_rad_s = 0.0f;
            ctx->id_ref_a = 0.0f;
            ctx->iq_ref_a = 0.0f;
            ctx->spd_loop_div_countdown = 0U;
            ctx->calib_request = 0U;
            ctx->calib_request_pending = 0U;
            FocSpeedCtrl_Reset(&ctx->spd_ctrl);
            SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
            SignalLogSweep_Reset(&ctx->iq_sweep);
            ctx->iq_sweep_request_pending = 0U;
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;
            ctx->vtest_active = 0U;
            MotorCalib_Abort(&ctx->calib);
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);

            ctx->mt6835_reg011 = ack;
            ctx->mt6835_reg011_valid = ok;
            ctx->mt6835_reg_op = (ok != 0U) ? 4U : 5U;
            if (ok != 0U)
            {
                g_mt6835_quiet_ticks = MOTORAPP_MT6835_EEPROM_QUIET_TICKS;
            }
        }
        else
        {
            uint8_t v = 0U;
            if (Mt6835_ReadReg8(&ctx->encoder, (uint16_t)MOTORAPP_MT6835_REG_BW_ADDR, &v) == 0U)
            {
                ctx->mt6835_reg011_valid = 0U;
                ctx->mt6835_reg_op = 3U;
                ctx->enc_dma_enable = 1U;
                ctx->stream_page = 4U;
                break;
            }

            ctx->mt6835_reg011 = v;
            ctx->mt6835_reg011_valid = 1U;
            ctx->mt6835_reg_op = 1U;

            if (subcmd == 1U)
            {
                const uint8_t new_v = (uint8_t)((v & 0xF8U) | ((uint8_t)MOTORAPP_MT6835_REG_BW_BITS & 0x07U));
                uint8_t ok = Mt6835_WriteReg8(&ctx->encoder, (uint16_t)MOTORAPP_MT6835_REG_BW_ADDR, new_v);
                uint8_t rd = 0U;
                ok = ((ok != 0U) && (Mt6835_ReadReg8(&ctx->encoder, (uint16_t)MOTORAPP_MT6835_REG_BW_ADDR, &rd) != 0U)) ? 1U
                                                                                                                        : 0U;
                ctx->mt6835_reg011 = rd;
                ctx->mt6835_reg011_valid = ok;
                ctx->mt6835_reg_op = (ok != 0U) ? 2U : 3U;
            }
        }

        if (g_mt6835_quiet_ticks == 0U)
        {
            ctx->enc_dma_enable = 1U;
        }
        ctx->stream_page = 4U;
    }
    break;

    case 'F':
        if (mt6835_quiet != 0U)
        {
            break;
        }
        /* F1: start log-sweep injection on Iq_cmd, F0: stop */
        ctx->iq_sweep_request = ((cmd->has_value == 0U) || (cmd->value != 0.0f)) ? 1U : 0U;
        ctx->iq_sweep_request_pending = 1U;
        ctx->stream_page = 6U;
        break;

        /* 将转子强拖到U相 */
    case 'T':
        if (mt6835_quiet != 0U)
        {
            break;
        }
        if ((cmd->has_value != 0U) && (cmd->value == 0.0f))
        {
            ctx->spd_loop_enabled = 0U;
            ctx->target_vel_rad_s = 0.0f;
            ctx->spd_loop_div_countdown = 0U;
            FocSpeedCtrl_Reset(&ctx->spd_ctrl);
            SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
            SignalLogSweep_Reset(&ctx->iq_sweep);
            ctx->iq_sweep_request_pending = 0U;
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;
            ctx->vtest_active = 0U;
            ctx->i_loop_enable_pending = 0U;
            ctx->i_loop_enabled = 0U;
            ctx->iq_ref_a = 0.0f;
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
        }
        else
        {
            ctx->spd_loop_enabled = 0U;
            ctx->target_vel_rad_s = 0.0f;
            ctx->spd_loop_div_countdown = 0U;
            FocSpeedCtrl_Reset(&ctx->spd_ctrl);
            SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
            SignalLogSweep_Reset(&ctx->iq_sweep);
            ctx->iq_sweep_request_pending = 0U;
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;

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

    /* 锁存本轮ADC采样通道组合 */
    const CurrentSensePair sampled_pair = (ctx != 0) ? ctx->i_pair_active : CURRENT_SENSE_PAIR_AB;
    if (ctx == 0)
    {
        return;
    }

    /* ISR profiling pulse on PC8 (S_Pin): high at entry, low at exit */
    S_GPIO_Port->BSRR = (uint32_t)S_Pin;

    ctx->adc1_raw = adc1;
    ctx->adc2_raw = adc2;

    /* 心跳监控（Telemetry / Profiling）变量 */
    ctx->adc_isr_count++;

    /* 数据流打印分频计数器 */
#if (MOTORAPP_STREAM_USE_ISR_DIV != 0U)
#if (MOTORAPP_STREAM_DIV > 0U)
    if (ctx->stream_div_countdown == 0U)
    {
        ctx->stream_div_countdown = (uint16_t)(MOTORAPP_STREAM_DIV - 1U);
        if (ctx->stream_pending != 0xFFFFU)
        {
            ctx->stream_pending++;
        }
    }
    else
    {
        ctx->stream_div_countdown--;
    }
#endif
#endif

    /* Pop encoder sample completed by SPI DMA ISR (pipeline: 1 tick latency) */
    uint32_t raw21 = 0U;
    if (BspMt6835Dma_PopRaw21(&ctx->enc_dma, &raw21) != 0U)
    {
        ctx->raw21 = raw21;
        ctx->raw21_corr = Mt6835AngleCorr_ApplyRaw21(raw21);
        ctx->pos_mech_rad = Mt6835_Raw21ToRad(ctx->raw21_corr);

        const float theta = ctx->pos_mech_rad;
        const float dt = ((float)MOTORAPP_ENCODER_READ_DIV) * (1.0f / MOTORAPP_CTRL_HZ);
        /* 第一次启动，速度计算初始化 */
        if (ctx->spd_valid == 0U)
        {
            ctx->spd_valid = 1U;
            ctx->spd_theta_prev_rad = theta;
            ctx->spd_omega_diff_rad_s = 0.0f;
            ctx->spd_pll_theta_hat_rad = theta;
            ctx->spd_pll_omega_int_rad_s = 0.0f;
            ctx->spd_omega_pll_rad_s = 0.0f;
        }
        else
        {
            const float dtheta = MotorApp_WrapPi(theta - ctx->spd_theta_prev_rad);
            ctx->spd_theta_prev_rad = theta;
            ctx->spd_omega_diff_rad_s = dtheta / dt;

            const float e = MotorApp_WrapPi(theta - ctx->spd_pll_theta_hat_rad);
            ctx->spd_pll_omega_int_rad_s += (MOTORAPP_SPD_PLL_KI * e * dt);
            ctx->spd_omega_pll_rad_s = ctx->spd_pll_omega_int_rad_s + (MOTORAPP_SPD_PLL_KP * e);
            ctx->spd_pll_theta_hat_rad = MotorApp_Wrap2Pi(ctx->spd_pll_theta_hat_rad + (ctx->spd_omega_pll_rad_s * dt));
        }

        /* 统一符号：让 Iq>0 时 speed 为正（即使编码器角度递减，elec_dir=-1） */
        ctx->dbg_omega_diff_rad_s = (float)ctx->elec_dir * ctx->spd_omega_diff_rad_s;
        ctx->dbg_omega_pll_rad_s = (float)ctx->elec_dir * ctx->spd_omega_pll_rad_s;
    }

    /* Start next encoder DMA transaction (frequency divider) */
#if (MOTORAPP_ENCODER_READ_DIV > 0U)
    /* 主循环是否允许读编码器(写编码器寄存器时冻结) */
    /* 编码器 EEPROM 烧录空闲倒计时 */
    if (g_mt6835_quiet_ticks != 0U)
    {
        g_mt6835_quiet_ticks--;
        ctx->enc_div_countdown = 0U;
        if (g_mt6835_quiet_ticks == 0U)
        {
            ctx->enc_dma_enable = 1U;
        }
    }
    /* 写 MT6835 寄存器时冻结 */
    else if (ctx->enc_dma_enable == 0U)
    {
        ctx->enc_div_countdown = 0U;
    }
    else if (ctx->enc_div_countdown == 0U)
    {
        (void)BspMt6835Dma_TryStart(&ctx->enc_dma);
        ctx->enc_div_countdown = (uint16_t)(MOTORAPP_ENCODER_READ_DIV - 1U);
    }
    else
    {
        ctx->enc_div_countdown--;
    }
#endif

    ctx->theta_e_meas_rad = MotorApp_ElecAngleFromMechRad(ctx, ctx->pos_mech_rad);
    ctx->theta_e_ctrl_rad = MotorApp_ElecAngleCtrlRad(ctx);

    /* 原始电角度 */
    ctx->dbg_theta_e_meas = ctx->theta_e_meas_rad;

    /* 经过前馈预测补偿后的控制用电角度 */
    ctx->dbg_theta_e_ctrl = ctx->theta_e_ctrl_rad;

    /* 电角度前馈补偿值_Deg*/
    ctx->dbg_theta_e_delta_deg = MotorApp_ThetaCtrlDeltaRad(ctx) * (180.0f / 3.14159265359f);

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
                /* Sample A+C to calibrate C offset. */
                MotorApp_ProgramCurrentPair(ctx, CURRENT_SENSE_PAIR_AC);
            }
            else if (ctx->i_offset_stage == 1U)
            {
                ctx->i_w_offset_raw = CurrentSenseOffset2_OffsetB(&ctx->i_ab_offset);

                ctx->i_offset_stage = 2U;
                ctx->i_offset_ready = 1U;
                /* Back to default A+B sampling before dynamic selection takes over. */
                MotorApp_ProgramCurrentPair(ctx, CURRENT_SENSE_PAIR_AB);
            }
        }
    }

    /* 原始数据转实际物理电流（安培） */
    if (ctx->i_offset_ready != 0U)
    {
        /* offset_raw[3] = {... } */
        const uint16_t offset_raw[CURRENT_SENSE_PHASE_COUNT] = {
            ctx->i_u_offset_raw,
            ctx->i_v_offset_raw,
            ctx->i_w_offset_raw,
        };
        ctx->dbg_i_pair_active = (uint8_t)sampled_pair;
        CurrentSense3Shunt_Reconstruct(sampled_pair, adc1, adc2, offset_raw, MOTORAPP_ADC_MAX_COUNTS, MOTORAPP_ADC_VREF_V,
                                       MOTORAPP_CURRENT_SHUNT_OHM, MOTORAPP_CURRENT_AMP_GAIN, -1.0f, &ctx->ia_a, &ctx->ib_a,
                                       &ctx->ic_a);
    }
    else
    {
        ctx->dbg_i_pair_active = (uint8_t)sampled_pair;
        ctx->ia_a = 0.0f;
        ctx->ib_a = 0.0f;
        ctx->ic_a = 0.0f;
    }

    /* Minimal software overcurrent trip (latched) */
    if ((ctx->fault_overcurrent == 0U) && (ctx->i_offset_ready != 0U) && (ctx->pwm.outputs_enabled != 0U))
    {
        const float ia = fabsf(ctx->ia_a);
        const float ib = fabsf(ctx->ib_a);
        const float ic = fabsf(ctx->ic_a);
        if ((ia > ctx->i_trip_a) || (ib > ctx->i_trip_a) || (ic > ctx->i_trip_a))
        {
            ctx->fault_overcurrent = 1U;
            ctx->i_loop_enable_pending = 0U;
            ctx->i_loop_enabled = 0U;
            ctx->spd_loop_enabled = 0U;
            ctx->target_vel_rad_s = 0.0f;
            ctx->iq_ref_a = 0.0f;
            ctx->spd_loop_div_countdown = 0U;
            FocSpeedCtrl_Reset(&ctx->spd_ctrl);
            SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);
            SignalLogSweep_Reset(&ctx->iq_sweep);
            ctx->iq_sweep_request_pending = 0U;
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;
            ctx->vtest_active = 0U;
            FocCurrentCtrl_Reset(&ctx->i_ctrl);
            MotorCalib_Abort(&ctx->calib);
            (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
        }
    }

    /* 锁存错误标志，直接跳到函数末尾，退出本次中断*/
    if (ctx->fault_overcurrent != 0U)
    {
        goto isr_exit;
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
        ctx->dbg_ud_pu = cmd.ud;
        ctx->dbg_uq_pu = cmd.uq;
        ctx->dbg_u_mag_pu = sqrtf((cmd.ud * cmd.ud) + (cmd.uq * cmd.uq));

        MotorApp_OutputVdqSc(ctx, cmd.ud, cmd.uq, s, c);
    }
    else if ((ctx->i_loop_enabled != 0U) && (ctx->i_offset_ready != 0U))
    {
        /* Handle sweep start/stop request in ISR to keep dt consistent */
        if (ctx->iq_sweep_request_pending != 0U)
        {
            ctx->iq_sweep_request_pending = 0U;
            if (ctx->iq_sweep_request != 0U)
            {
/* 扫频信号分频 */
#if (MOTORAPP_LOG_SWEEP_DIV > 0U)
                const float dt_sweep = ((float)MOTORAPP_LOG_SWEEP_DIV) * (1.0f / MOTORAPP_CTRL_HZ);
#else
                const float dt_sweep = 1.0f / MOTORAPP_CTRL_HZ;
#endif

                SignalLogSweep_Start(&ctx->iq_sweep, MOTORAPP_LOG_SWEEP_AMP_A, MOTORAPP_LOG_SWEEP_F_START_HZ,
                                     MOTORAPP_LOG_SWEEP_F_END_HZ, MOTORAPP_LOG_SWEEP_DURATION_S, dt_sweep);
                ctx->iq_sweep_div_countdown = 0U;
                ctx->iq_sweep_a = 0.0f;
            }
            else
            {
                SignalLogSweep_Stop(&ctx->iq_sweep);
                ctx->iq_sweep_a = 0.0f;
                ctx->iq_sweep_div_countdown = 0U;
            }
        }

        /* Speed loop (outer): update Iq_ref at lower rate, current loop still runs at 20kHz */
        if (ctx->spd_loop_enabled != 0U)
        {
#if (MOTORAPP_SPEED_LOOP_DIV > 0U)
            if (ctx->spd_loop_div_countdown == 0U)
            {
                float omega_ref = ctx->target_vel_rad_s;
#if (MOTORAPP_SPD_REF_S_CURVE_ENABLE != 0U)
                omega_ref = SCurveVel_Step(&ctx->spd_ref_plan, omega_ref);
#else
                /* 同步实际速度至S-Curve规划器内部状态，确保闭环瞬间速度平滑衔接 */
                SCurveVel_Reset(&ctx->spd_ref_plan, omega_ref);
#endif
                const float omega_meas = ctx->dbg_omega_pll_rad_s;
                ctx->id_ref_a = 0.0f;
                ctx->iq_ref_a = FocSpeedCtrl_Step(&ctx->spd_ctrl, omega_ref, omega_meas);
                ctx->spd_loop_div_countdown = (uint16_t)(MOTORAPP_SPEED_LOOP_DIV - 1U);
            }
            else
            {
                ctx->spd_loop_div_countdown--;
            }
#else
            float omega_ref = ctx->target_vel_rad_s;
#if (MOTORAPP_SPD_REF_S_CURVE_ENABLE != 0U)
            omega_ref = SCurveVel_Step(&ctx->spd_ref_plan, omega_ref);
#else
            SCurveVel_Reset(&ctx->spd_ref_plan, omega_ref);
#endif
            const float omega_meas = ctx->dbg_omega_pll_rad_s;
            ctx->id_ref_a = 0.0f;
            ctx->iq_ref_a = FocSpeedCtrl_Step(&ctx->spd_ctrl, omega_ref, omega_meas);
#endif
        }
        else
        {
            ctx->spd_loop_div_countdown = 0U;
            SCurveVel_Reset(&ctx->spd_ref_plan, ctx->target_vel_rad_s);
        }

        /* Iq log-sweep injection (for system identification): Iq_cmd = Iq_base + sweep */
        if (ctx->iq_sweep.active != 0U)
        {
#if (MOTORAPP_LOG_SWEEP_DIV > 0U)
            if (ctx->iq_sweep_div_countdown == 0U)
            {
                ctx->iq_sweep_a = SignalLogSweep_Step(&ctx->iq_sweep);
                ctx->iq_sweep_div_countdown = (uint16_t)(MOTORAPP_LOG_SWEEP_DIV - 1U);
            }
            else
            {
                ctx->iq_sweep_div_countdown--;
            }
#else
            ctx->iq_sweep_a = SignalLogSweep_Step(&ctx->iq_sweep);
#endif
        }
        else
        {
            ctx->iq_sweep_div_countdown = 0U;
            ctx->iq_sweep_a = 0.0f;
        }

        float iq_cmd_a = ctx->iq_ref_a + ctx->iq_sweep_a;

        float iq_comp_a = 0.0f;
#if (MOTORAPP_IQ_LUT_COMP_ENABLE != 0U)
        if ((ctx->spd_loop_enabled != 0U) && (ctx->calib_done != 0U))
        {
            const float omega_abs = fabsf(ctx->spd_ref_plan.v);
            if ((omega_abs >= MOTORAPP_IQ_LUT_COMP_MIN_OMEGA_RAD_S) && (omega_abs <= MOTORAPP_IQ_LUT_COMP_MAX_OMEGA_RAD_S))
            {
                iq_comp_a = MOTORAPP_IQ_LUT_COMP_GAIN * IqLutComp_SampleRaw21(ctx->raw21_corr);
                if (iq_comp_a > MOTORAPP_IQ_LUT_COMP_LIMIT_A)
                {
                    iq_comp_a = MOTORAPP_IQ_LUT_COMP_LIMIT_A;
                }
                else if (iq_comp_a < -MOTORAPP_IQ_LUT_COMP_LIMIT_A)
                {
                    iq_comp_a = -MOTORAPP_IQ_LUT_COMP_LIMIT_A;
                }
            }
        }
#endif
        ctx->dbg_iq_comp_a = iq_comp_a;
        iq_cmd_a += iq_comp_a;
        if (iq_cmd_a > ctx->i_limit_a)
        {
            iq_cmd_a = ctx->i_limit_a;
        }
        if (iq_cmd_a < -ctx->i_limit_a)
        {
            iq_cmd_a = -ctx->i_limit_a;
        }
        ctx->dbg_iq_cmd_a = iq_cmd_a;

        const float theta_e = ctx->theta_e_ctrl_rad;
        float s = 0.0f;
        float c = 1.0f;
        BspTrig_SinCos(theta_e, &s, &c);

        FocCurrentCtrlOut iout = {0};
        float uq_ff_v = 0.0f;
#if (MOTORAPP_BEMF_FF_ENABLE != 0U)
        uq_ff_v = MOTORAPP_BEMF_KE_V_PER_RAD_S * ctx->dbg_omega_pll_rad_s; // 反电动势前馈
#endif
        FocCurrentCtrl_StepScFf(&ctx->i_ctrl, ctx->ia_a, ctx->ib_a, ctx->ic_a, s, c, ctx->id_ref_a, iq_cmd_a, 0.0f, uq_ff_v,
                                &iout);

        ctx->dbg_theta_e = theta_e;
        ctx->dbg_ud = iout.ud_pu;
        ctx->dbg_uq = iout.uq_pu;
        ctx->dbg_id_a = iout.id_a;
        ctx->dbg_iq_a = iout.iq_a;

        /* 占空比生成的归一化 */
        /* pu-Per Unit 标幺值 */
        ctx->dbg_ud_pu = iout.ud_pu;
        ctx->dbg_uq_pu = iout.uq_pu;

        /* 交、直轴电压矢量和 */
        ctx->dbg_u_mag_pu = sqrtf((iout.ud_pu * iout.ud_pu) + (iout.uq_pu * iout.uq_pu));

        MotorApp_OutputVdqSc(ctx, iout.ud_pu, iout.uq_pu, s, c);
    }
    /* d轴开环强拖 */
    else if (ctx->vtest_active != 0U)
    {
        ctx->dbg_theta_e = 0.0f;
        ctx->dbg_ud = ctx->vtest_ud;
        ctx->dbg_uq = ctx->vtest_uq;
        ctx->dbg_ud_pu = ctx->vtest_ud;
        ctx->dbg_uq_pu = ctx->vtest_uq;
        ctx->dbg_u_mag_pu = sqrtf((ctx->vtest_ud * ctx->vtest_ud) + (ctx->vtest_uq * ctx->vtest_uq));
        MotorApp_OutputVdqSc(ctx, ctx->vtest_ud, ctx->vtest_uq, 0.0f, 1.0f);
    }
    else if (ctx->pwm.outputs_enabled != 0U)
    {
        ctx->dbg_ud = 0.0f;
        ctx->dbg_uq = 0.0f;
        ctx->dbg_ud_pu = 0.0f;
        ctx->dbg_uq_pu = 0.0f;
        ctx->dbg_u_mag_pu = 0.0f;
        BspTim1Pwm_SetNeutral(&ctx->pwm);
    }
    else
    {
        ctx->dbg_ud = 0.0f;
        ctx->dbg_uq = 0.0f;
        ctx->dbg_ud_pu = 0.0f;
        ctx->dbg_uq_pu = 0.0f;
        ctx->dbg_u_mag_pu = 0.0f;
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

isr_exit:
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
    ctx->i_pair_active = CURRENT_SENSE_PAIR_AB;

    ctx->vbus_raw = 0U;
    ctx->vbus_v = MOTORAPP_VBUS_V;
    ctx->last_vbus_tick_ms = HAL_GetTick();

    ctx->i_trip_a = MOTORAPP_I_TRIP_A;
    ctx->fault_overcurrent = 0U;

    ctx->enc_div_countdown = 0U;
    ctx->enc_dma_enable = 1U;

    ctx->i_limit_a = MOTORAPP_I_LIMIT_A;
    ctx->id_ref_a = 0.0f;
    ctx->iq_ref_a = 0.0f;
    ctx->i_loop_enabled = 0U;
    ctx->i_loop_enable_pending = 0U;
    FocCurrentCtrl_Init(&ctx->i_ctrl, MOTORAPP_ICTRL_KP, MOTORAPP_ICTRL_KI, 1.0f / MOTORAPP_CTRL_HZ, ctx->vbus_v,
                        MOTORAPP_V_LIMIT_PU);

    ctx->target_vel_rad_s = 0.0f;
    ctx->spd_loop_enabled = 0U;
    ctx->spd_loop_div_countdown = 0U;
    FocSpeedCtrl_Init(&ctx->spd_ctrl, MOTORAPP_SPDCTRL_KP, MOTORAPP_SPDCTRL_KI,
                      ((float)MOTORAPP_SPEED_LOOP_DIV) * (1.0f / MOTORAPP_CTRL_HZ), MOTORAPP_SCTRL_IQ_LIMIT_A);
    SCurveVel_Init(&ctx->spd_ref_plan, ((float)MOTORAPP_SPEED_LOOP_DIV) * (1.0f / MOTORAPP_CTRL_HZ),
                   MOTORAPP_SPD_REF_A_MAX_RAD_S2, MOTORAPP_SPD_REF_J_MAX_RAD_S3, MOTORAPP_SPD_REF_K_A);
    SCurveVel_Reset(&ctx->spd_ref_plan, 0.0f);

    SignalLogSweep_Reset(&ctx->iq_sweep);
    ctx->iq_sweep_request = 0U;
    ctx->iq_sweep_request_pending = 0U;
    ctx->iq_sweep_div_countdown = 0U;
    ctx->iq_sweep_a = 0.0f;

    BspUartDma_Init(&ctx->uart, huart);
    HostCmdApp_Init(&ctx->host_cmd, huart);
    (void)HostCmdApp_Start(&ctx->host_cmd);

    BspTim1Pwm_Init(&ctx->pwm, htim_pwm);
    (void)BspTim1Pwm_StartTrigger(&ctx->pwm);

    BspAdcInjPair_Init(&ctx->adc_inj, hadc1, hadc2);
    BspAdcInjPair_RegisterCallback(&ctx->adc_inj, ctx, MotorApp_OnAdcPair);
    MotorApp_ProgramCurrentPair(ctx, CURRENT_SENSE_PAIR_AB);
    (void)BspAdcInjPair_Start(&ctx->adc_inj);

    BspSpi3Fast_Init(&ctx->spi, hspi, cs_port, cs_pin);

    BspMt6835Dma_Init(&ctx->enc_dma, hspi, cs_port, cs_pin);

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
        .ud_align = 0.09f,                                  // 对齐阶段d轴电压
        .uq_spin = 0.07f,                                   // 开环拖动阶段q轴电压
        .omega_e_rad_s = 31.4159265f,                       /* 开环拖动时的电角速度 2*pi*5Hz */
        .align_ticks = (uint32_t)(MOTORAPP_CTRL_HZ * 0.5f), // 对齐阶段保持时间
        .spin_ticks = (uint32_t)(MOTORAPP_CTRL_HZ * 0.3f),  // 拖动阶段持续时间
        .min_move_rad = 0.2f,                               // 拖动阶段最小机械移动角度
    };
    MotorCalib_Init(&ctx->calib, &calib);
    ctx->elec_dir = -1;
    ctx->elec_zero_offset_rad = 0.620399f;

    ctx->last_stream_tick_ms = HAL_GetTick();
    ctx->stream_pending = 0U;
    ctx->stream_div_countdown = 0U;
    ctx->stream_page = 0U;
    ctx->vtest_active = 0U;
    ctx->vtest_ud = 0.0f;
    ctx->vtest_uq = 0.0f;
    g_mt6835_quiet_ticks = 0U;
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

    /* Vbus slow sampling (ADC1 regular) */
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - ctx->last_vbus_tick_ms) >= MOTORAPP_VBUS_SAMPLE_MS)
    {
        ctx->last_vbus_tick_ms = now_ms;

        /* 启动ADC规则组转换通道 */
        if (HAL_ADC_Start(ctx->adc_inj.hadc1) == HAL_OK)
        {
            /* 检测ADC转换完成标志位EOC，最多1ms(1U) */
            if (HAL_ADC_PollForConversion(ctx->adc_inj.hadc1, 1U) == HAL_OK)
            {
                ctx->vbus_raw = (uint16_t)HAL_ADC_GetValue(ctx->adc_inj.hadc1);
                const float vadc = (float)ctx->vbus_raw * (MOTORAPP_ADC_VREF_V / MOTORAPP_ADC_MAX_COUNTS);
                const float vbus_new = vadc * MOTORAPP_VBUS_DIV;
                ctx->vbus_v = ctx->vbus_v + (MOTORAPP_VBUS_FILTER_ALPHA * (vbus_new - ctx->vbus_v));
                ctx->i_ctrl.vbus_v = ctx->vbus_v;
            }
            /* 注意：不要调用 HAL_ADC_Stop()，它会关闭 ADC 并强行停止 injected 转换，导致 20kHz 控制中断消失 */
        }
    }

#if (MOTORAPP_STREAM_USE_ISR_DIV != 0U)
    if ((ctx->stream_pending == 0U) || (BspUartDma_TxReady(&ctx->uart) == 0U))
    {
        return;
    }
    ctx->stream_pending--;
#else
    if (now_ms == ctx->last_stream_tick_ms)
    {
        return;
    }
    ctx->last_stream_tick_ms = now_ms;
    if (BspUartDma_TxReady(&ctx->uart) == 0U)
    {
        return;
    }
#endif

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
            /* U V W相电流采样零偏值、V_bus监测 */
            JustFloat_Pack4((float)ctx->i_u_offset_raw, (float)ctx->i_v_offset_raw, (float)ctx->i_w_offset_raw, ctx->vbus_v,
                            ctx->tx_frame);
        }
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 3U)
    {
        /* D3：Id/Iq + 速度估计对比（直接差分 vs PLL） */
        JustFloat_Pack4(ctx->dbg_id_a, ctx->dbg_iq_a, ctx->dbg_omega_diff_rad_s, ctx->dbg_omega_pll_rad_s, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 4U)
    {
        /* D4：MT6835 reg0x011 读写调试：reg(byte), high5, BW[2:0], op(1=read,2=write+verify,3=fail) */
        const uint8_t reg = ctx->mt6835_reg011;
        const uint8_t high5 = (uint8_t)((reg >> 3) & 0x1FU);
        const uint8_t bw3 = (uint8_t)(reg & 0x07U);
        JustFloat_Pack4((float)reg, (float)high5, (float)bw3, (float)ctx->mt6835_reg_op, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 5U)
    {
        /* D5：速度环监控：omega_ref / omega_pll / Iq_ref / Iq_meas */
        JustFloat_Pack4(ctx->spd_ref_plan.v, ctx->dbg_omega_pll_rad_s, ctx->iq_ref_a, ctx->dbg_iq_a, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 6U)
    {
        /* D6：系统辨识采集：反馈速度 / 交轴反馈电流 / 交轴总给定电流 / 扫频信号标志位 */
        JustFloat_Pack4(ctx->dbg_omega_pll_rad_s, ctx->dbg_iq_a, ctx->dbg_iq_cmd_a, (float)ctx->iq_sweep.active, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 7U)
    {
        /* D7：全周期 LUT 采集：raw21 / omega_pll / Iq_ref / Iq_meas */
        JustFloat_Pack4((float)ctx->raw21, ctx->dbg_omega_pll_rad_s, ctx->iq_ref_a, ctx->dbg_iq_a, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 8U)
    {
        /* D8：补偿观测：iq_a / Iq_ref / iq_sweep_a / Iq_cmd */
        JustFloat_Pack4(ctx->dbg_iq_a, ctx->iq_ref_a, ctx->iq_sweep_a, ctx->dbg_iq_cmd_a, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 9U)
    {
        JustFloat_Pack4((float)ctx->dbg_i_low_window_a_ticks, (float)ctx->dbg_i_low_window_b_ticks,
                        (float)ctx->dbg_i_low_window_c_ticks, (float)ctx->dbg_i_valid_mask, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 10U)
    {
        /* 本周期ADC采样通道组、下周期ADC采样通道组、采样通道有效性、本周期扇区 */
        JustFloat_Pack4((float)ctx->dbg_i_pair_active, (float)ctx->dbg_i_pair_next, (float)ctx->dbg_i_pair_valid,
                        (float)ctx->dbg_svm_sector, ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 11U)
    {
        /* spi通讯补偿前电角度、spi通讯补偿后电角度、补偿电角度增量、电压矢量和 */
        JustFloat_Pack4(ctx->dbg_theta_e_meas, ctx->dbg_theta_e_ctrl, ctx->dbg_theta_e_delta_deg, ctx->dbg_u_mag_pu,
                        ctx->tx_frame);
        (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
        return;
    }

    if (ctx->stream_page == 12U)
    {
        JustFloat_Pack4(ctx->dbg_ud_pu, ctx->dbg_uq_pu, ctx->dbg_u_mag_pu, ctx->dbg_theta_e_delta_deg, ctx->tx_frame);
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
