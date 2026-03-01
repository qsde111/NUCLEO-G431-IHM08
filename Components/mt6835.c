#include "mt6835.h"

#define MT6835_COUNTS_PER_REV (2097152.0f)
#define MT6835_TWO_PI (6.28318530718f)

void Mt6835_Init(Mt6835 *ctx, const Mt6835BusOps *bus)
{
    if ((ctx == 0) || (bus == 0))
    {
        return;
    }

    ctx->bus = *bus;
    ctx->stream_started = 0U;
}

/* 读取MT6835 编码器里的 21位 绝对位置原始数据并返回 */
uint32_t Mt6835_ReadRaw21(Mt6835 *ctx)
{
    if ((ctx == 0) || (ctx->bus.transfer8 == 0) || (ctx->bus.cs_low == 0) || (ctx->bus.cs_high == 0))
    {
        return 0U;
    }

    uint8_t b20_13 = 0U;
    uint8_t b12_5 = 0U;
    uint8_t b4_0_status = 0U;

    ctx->bus.cs_low(ctx->bus.user);

    if (ctx->stream_started == 0U)
    {
        (void)ctx->bus.transfer8(ctx->bus.user, MT6835_STREAM_CMD_H);
        (void)ctx->bus.transfer8(ctx->bus.user, MT6835_STREAM_CMD_L);
        ctx->stream_started = 0U;
    }

    b20_13 = ctx->bus.transfer8(ctx->bus.user, 0x00U);
    b12_5 = ctx->bus.transfer8(ctx->bus.user, 0x00U);
    b4_0_status = ctx->bus.transfer8(ctx->bus.user, 0x00U);
    // (void)ctx->bus.transfer8(ctx->bus.user, 0x00U);

    ctx->bus.cs_high(ctx->bus.user);

    return (((uint32_t)b20_13 << 13) | ((uint32_t)b12_5 << 5) | ((uint32_t)b4_0_status >> 3));
}

/* 读取指定寄存器(8bit)，按手册：发送 0x3xxx(读指令+地址)，再发 1 字节 dummy 挤出寄存器值（总 24 clocks） */
uint8_t Mt6835_ReadReg8(Mt6835 *ctx, uint16_t reg, uint8_t *val_out)
{
    if ((ctx == 0) || (val_out == 0) || (ctx->bus.transfer8 == 0) || (ctx->bus.cs_low == 0) || (ctx->bus.cs_high == 0))
    {
        return 0U;
    }

    const uint16_t cmd = (uint16_t)(0x3000U | (reg & 0x0FFFU));

    ctx->bus.cs_low(ctx->bus.user);
    (void)ctx->bus.transfer8(ctx->bus.user, (uint8_t)(cmd >> 8));
    (void)ctx->bus.transfer8(ctx->bus.user, (uint8_t)(cmd & 0xFFU));
    const uint8_t rx = ctx->bus.transfer8(ctx->bus.user, 0x00U);
    ctx->bus.cs_high(ctx->bus.user);

    *val_out = rx;
    return 1U;
}

/* 写指定寄存器(8bit)，发送 0x6xxx(写指令+地址)，再发 1 字节数据（总 24 clocks） */
uint8_t Mt6835_WriteReg8(Mt6835 *ctx, uint16_t reg, uint8_t val)
{
    if ((ctx == 0) || (ctx->bus.transfer8 == 0) || (ctx->bus.cs_low == 0) || (ctx->bus.cs_high == 0))
    {
        return 0U;
    }

    const uint16_t cmd = (uint16_t)(0x6000U | (reg & 0x0FFFU));

    ctx->bus.cs_low(ctx->bus.user);
    (void)ctx->bus.transfer8(ctx->bus.user, (uint8_t)(cmd >> 8));
    (void)ctx->bus.transfer8(ctx->bus.user, (uint8_t)(cmd & 0xFFU));
    (void)ctx->bus.transfer8(ctx->bus.user, val);
    ctx->bus.cs_high(ctx->bus.user);

    return 1U;
}

/* 编码器读取数值转换为机械角度 */
float Mt6835_Raw21ToRad(uint32_t raw21)
{
    return (float)(raw21 & 0x1FFFFFU) * (MT6835_TWO_PI / MT6835_COUNTS_PER_REV);
}

float Mt6835_Raw21ToDeg(uint32_t raw21)
{
    return (float)(raw21 & 0x1FFFFFU) * (360.0f / MT6835_COUNTS_PER_REV);
}
