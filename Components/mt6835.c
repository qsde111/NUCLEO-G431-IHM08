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
    (void)ctx->bus.transfer8(ctx->bus.user, 0x00U);

    ctx->bus.cs_high(ctx->bus.user);

    return (((uint32_t)b20_13 << 13) | ((uint32_t)b12_5 << 5) | ((uint32_t)b4_0_status >> 3));
}

float Mt6835_Raw21ToRad(uint32_t raw21)
{
    return (float)(raw21 & 0x1FFFFFU) * (MT6835_TWO_PI / MT6835_COUNTS_PER_REV);
}

float Mt6835_Raw21ToDeg(uint32_t raw21)
{
    return (float)(raw21 & 0x1FFFFFU) * (360.0f / MT6835_COUNTS_PER_REV);
}
