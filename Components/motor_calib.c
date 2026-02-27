#include "motor_calib.h"

#include <string.h>

static float MotorCalib_Wrap2Pi(float x)
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

static float MotorCalib_WrapPi(float x)
{
    const float pi = 3.14159265359f;
    const float two_pi = 6.28318530718f;
    while (x > pi)
    {
        x -= two_pi;
    }
    while (x < -pi)
    {
        x += two_pi;
    }
    return x;
}

void MotorCalib_Init(MotorCalib *ctx, const MotorCalibParams *params)
{
    if (ctx == 0)
    {
        return;
    }

    (void)memset(ctx, 0, sizeof(*ctx));

    if (params != 0)
    {
        ctx->p = *params;
    }

    ctx->state = MOTOR_CALIB_IDLE;
    ctx->dir = 1;
    ctx->zero_offset_rad = 0.0f;
}

void MotorCalib_Start(MotorCalib *ctx, float theta_mech_rad)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->state = MOTOR_CALIB_ALIGN; // 将状态机切换为对齐模式（给 D 轴注入恒定电压，迫使电机锁定在电角度 0 度）
    ctx->tick = 0U;                 // 计时器清零，用于控制对齐模式和后续旋转模式的持续时间
    ctx->theta_e_cmd = 0.0f;
    ctx->theta_mech_align = theta_mech_rad; // 对齐时-机械角度
    ctx->theta_mech_start = theta_mech_rad; // 旋转开始时-机械角度
    ctx->theta_mech_end = theta_mech_rad;   // 旋转结束时-机械角度
    ctx->dir = 1;
    ctx->zero_offset_rad = 0.0f;
}

void MotorCalib_Abort(MotorCalib *ctx)
{
    if (ctx == 0)
    {
        return;
    }
    ctx->state = MOTOR_CALIB_IDLE;
    ctx->tick = 0U;
    ctx->theta_e_cmd = 0.0f;
}

/* 磁极校准时的状态机流转和给定Uq Ud 电角度计算 */
void MotorCalib_Tick(MotorCalib *ctx, float dt_s, float theta_mech_rad)
{
    if (ctx == 0)
    {
        return;
    }

    switch (ctx->state)
    {
    case MOTOR_CALIB_IDLE:
    case MOTOR_CALIB_DONE:
    case MOTOR_CALIB_FAIL:
        return;

    case MOTOR_CALIB_ALIGN:
        ctx->tick++;
        if (ctx->tick >= ctx->p.align_ticks)
        {
            ctx->theta_mech_align = theta_mech_rad;
            ctx->theta_mech_start = theta_mech_rad;
            ctx->tick = 0U;
            ctx->state = MOTOR_CALIB_SPIN;
        }
        return;

    case MOTOR_CALIB_SPIN:
        ctx->theta_e_cmd = MotorCalib_Wrap2Pi(ctx->theta_e_cmd + (ctx->p.omega_e_rad_s * dt_s));
        ctx->tick++;
        if (ctx->tick >= ctx->p.spin_ticks)
        {
            ctx->theta_mech_end = theta_mech_rad;

            /* 转子实际走过的距离，考虑了0到360度过零点的环绕处理 */
            const float d_mech = MotorCalib_WrapPi(ctx->theta_mech_end - ctx->theta_mech_start);

            if ((d_mech > ctx->p.min_move_rad) || (d_mech < -ctx->p.min_move_rad))
            {
                ctx->dir = (d_mech >= 0.0f) ? 1 : -1;
                ctx->zero_offset_rad = MotorCalib_Wrap2Pi(-(float)ctx->dir * ctx->p.pole_pairs * ctx->theta_mech_align);
                ctx->state = MOTOR_CALIB_DONE;
            }
            else
            {
                ctx->state = MOTOR_CALIB_FAIL;
            }
        }
        return;
    }
}

/* 将磁极校准阶段的Uq Ud 电角度给定值从MotorCalib *ctx搬运到外部缓冲区MotorCalibCmd *out */
uint8_t MotorCalib_GetCmd(const MotorCalib *ctx, MotorCalibCmd *out)
{
    if ((ctx == 0) || (out == 0))
    {
        return 0U;
    }

    if (ctx->state == MOTOR_CALIB_ALIGN)
    {
        out->ud = ctx->p.ud_align;
        out->uq = 0.0f;
        out->theta_e = 0.0f;
        return 1U;
    }
    if (ctx->state == MOTOR_CALIB_SPIN)
    {
        out->ud = 0.0f;
        out->uq = ctx->p.uq_spin;
        out->theta_e = ctx->theta_e_cmd;
        return 1U;
    }

    out->ud = 0.0f;
    out->uq = 0.0f;
    out->theta_e = 0.0f;
    return 0U;
}

MotorCalibState MotorCalib_State(const MotorCalib *ctx)
{
    if (ctx == 0)
    {
        return MOTOR_CALIB_IDLE;
    }
    return ctx->state;
}

int8_t MotorCalib_Dir(const MotorCalib *ctx)
{
    if (ctx == 0)
    {
        return 1;
    }
    return ctx->dir;
}

float MotorCalib_ZeroOffsetRad(const MotorCalib *ctx)
{
    if (ctx == 0)
    {
        return 0.0f;
    }
    return ctx->zero_offset_rad;
}
