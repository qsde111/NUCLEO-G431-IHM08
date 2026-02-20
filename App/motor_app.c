#include "motor_app.h"

#include "main.h"

#include <math.h>
#include <string.h>

/* Control tick is driven by ADC injected interrupt (TIM1 TRGO2) */
#ifndef MOTORAPP_CTRL_HZ
#define MOTORAPP_CTRL_HZ (20000.0f)
#endif

static void MotorApp_OnAdcPair(void *user, uint16_t adc1, uint16_t adc2);

static void MotorApp_CalibStart(MotorApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->calib_done = 0U;
    ctx->calib_fail = 0U;
    (void)BspTim1Pwm_EnableOutputs(&ctx->pwm);
    MotorCalib_Start(&ctx->calib, ctx->pos_mech_rad);
}

static void MotorApp_CalibAbort(MotorApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    MotorCalib_Abort(&ctx->calib);
    ctx->calib_done = 0U;
    ctx->calib_fail = 0U;
    (void)BspTim1Pwm_DisableOutputs(&ctx->pwm);
}

static void MotorApp_OutputVdq(MotorApp *ctx, float ud, float uq, float theta_e)
{
    if (ctx == 0)
    {
        return;
    }

    const float s = sinf(theta_e);
    const float c = cosf(theta_e);

    const float u_alpha = (ud * c) - (uq * s);
    const float u_beta = (ud * s) + (uq * c);

    SvpwmOut out = {0};
    Svpwm_Calc(u_alpha, u_beta, &out);
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
    default:
        break;
    }
}

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

    MotorCalib_Tick(&ctx->calib, 1.0f / MOTORAPP_CTRL_HZ, ctx->pos_mech_rad);

    MotorCalibCmd cmd = {0};
    if (MotorCalib_GetCmd(&ctx->calib, &cmd) != 0U)
    {
        MotorApp_OutputVdq(ctx, cmd.ud, cmd.uq, cmd.theta_e);
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

    MotorCalibParams calib = {
        .pole_pairs = 7.0f,
        .ud_align = 0.08f,
        .uq_spin = 0.06f,
        .omega_e_rad_s = 31.4159265f, /* 2*pi*5Hz */
        .align_ticks = (uint32_t)(MOTORAPP_CTRL_HZ * 0.5f),
        .spin_ticks = (uint32_t)(MOTORAPP_CTRL_HZ * 0.3f),
        .min_move_rad = 0.2f,
    };
    MotorCalib_Init(&ctx->calib, &calib);
    ctx->elec_dir = 1;
    ctx->elec_zero_offset_rad = 0.0f;

    ctx->last_stream_tick_ms = HAL_GetTick();
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

    const uint32_t now = HAL_GetTick();
    if (now == ctx->last_stream_tick_ms)
    {
        return;
    }
    ctx->last_stream_tick_ms = now;

    ctx->raw21 = Mt6835_ReadRaw21(&ctx->encoder);
    ctx->pos_mech_rad = Mt6835_Raw21ToRad(ctx->raw21);

    JustFloat_Pack4((float)ctx->raw21, ctx->pos_mech_rad, ctx->elec_zero_offset_rad, (float)ctx->elec_dir, ctx->tx_frame);
    (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
}
