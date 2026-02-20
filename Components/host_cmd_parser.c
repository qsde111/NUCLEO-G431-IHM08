#include "host_cmd_parser.h"

#include <string.h>

static uint8_t HostCmdParser_IsWs(char c)
{
    return ((c == ' ') || (c == '\t')) ? 1U : 0U;
}

static char HostCmdParser_ToUpper(char c)
{
    if ((c >= 'a') && (c <= 'z'))
    {
        return (char)(c - ('a' - 'A'));
    }
    return c;
}

static uint8_t HostCmdParser_ParseFloat(const char *s, float *out, const char **endp)
{
    if (s == 0)
    {
        return 0U;
    }

    const char *p = s;
    while (HostCmdParser_IsWs(*p))
    {
        p++;
    }

    float sign = 1.0f;
    if (*p == '+')
    {
        p++;
    }
    else if (*p == '-')
    {
        sign = -1.0f;
        p++;
    }

    uint32_t int_part = 0U;
    uint32_t frac_part = 0U;
    uint32_t frac_div = 1U;
    uint8_t have_digit = 0U;

    while ((*p >= '0') && (*p <= '9'))
    {
        have_digit = 1U;
        int_part = (uint32_t)(int_part * 10U + (uint32_t)(*p - '0'));
        p++;
    }

    if (*p == '.')
    {
        p++;
        while ((*p >= '0') && (*p <= '9'))
        {
            have_digit = 1U;
            frac_part = (uint32_t)(frac_part * 10U + (uint32_t)(*p - '0'));
            frac_div = (uint32_t)(frac_div * 10U);
            p++;
        }
    }

    if (have_digit == 0U)
    {
        if (endp != 0)
        {
            *endp = s;
        }
        return 0U;
    }

    float v = (float)int_part;
    if (frac_div > 1U)
    {
        v += (float)frac_part / (float)frac_div;
    }
    v *= sign;

    if (out != 0)
    {
        *out = v;
    }
    if (endp != 0)
    {
        *endp = p;
    }
    return 1U;
}

static void HostCmdParser_QueuePush(HostCmdParser *ctx, const HostCmd *cmd)
{
    if ((ctx == 0) || (cmd == 0))
    {
        return;
    }
    if (ctx->q_count >= (uint8_t)HOST_CMD_QUEUE_LEN)
    {
        return;
    }

    ctx->queue[ctx->q_wr] = *cmd;
    ctx->q_wr = (uint8_t)((ctx->q_wr + 1U) % (uint8_t)HOST_CMD_QUEUE_LEN);
    ctx->q_count++;
}

static void HostCmdParser_ParseAndEnqueue(HostCmdParser *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->line[ctx->line_len] = '\0';

    const char *p = ctx->line;
    while (HostCmdParser_IsWs(*p))
    {
        p++;
    }

    if (*p == '\0')
    {
        ctx->line_len = 0U;
        ctx->line_overflow = 0U;
        return;
    }

    HostCmd cmd = {0};
    cmd.op = HostCmdParser_ToUpper(*p);
    p++;

    while (HostCmdParser_IsWs(*p))
    {
        p++;
    }

    if (*p != '\0')
    {
        float v = 0.0f;
        const char *endp = 0;
        if (HostCmdParser_ParseFloat(p, &v, &endp) != 0U)
        {
            while (HostCmdParser_IsWs(*endp))
            {
                endp++;
            }
            if (*endp == '\0')
            {
                cmd.has_value = 1U;
                cmd.value = v;
            }
        }
    }

    HostCmdParser_QueuePush(ctx, &cmd);
    ctx->line_len = 0U;
    ctx->line_overflow = 0U;
}

void HostCmdParser_Init(HostCmdParser *ctx)
{
    if (ctx == 0)
    {
        return;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
}

void HostCmdParser_Feed(HostCmdParser *ctx, const uint8_t *data, uint16_t len)
{
    if ((ctx == 0) || (data == 0) || (len == 0U))
    {
        return;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        const char c = (char)data[i];
        const uint8_t is_delim = ((c == '\n') || (c == '\r') || (c == ';')) ? 1U : 0U;
        if (is_delim != 0U)
        {
            if (ctx->line_overflow == 0U)
            {
                HostCmdParser_ParseAndEnqueue(ctx);
            }
            else
            {
                ctx->line_len = 0U;
                ctx->line_overflow = 0U;
            }
            continue;
        }

        if (ctx->line_overflow != 0U)
        {
            continue;
        }

        if ((c == '\b') || (c == 0x7FU))
        {
            if (ctx->line_len != 0U)
            {
                ctx->line_len--;
            }
            continue;
        }

        if ((c < 0x20) || (c > 0x7EU))
        {
            continue;
        }

        if (ctx->line_len >= (uint16_t)HOST_CMD_MAX_LINE)
        {
            ctx->line_len = 0U;
            ctx->line_overflow = 1U;
            continue;
        }

        ctx->line[ctx->line_len] = c;
        ctx->line_len++;
    }
}

void HostCmdParser_FlushLine(HostCmdParser *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    if (ctx->line_overflow != 0U)
    {
        ctx->line_len = 0U;
        ctx->line_overflow = 0U;
        return;
    }

    if (ctx->line_len == 0U)
    {
        return;
    }

    HostCmdParser_ParseAndEnqueue(ctx);
}

uint8_t HostCmdParser_Pop(HostCmdParser *ctx, HostCmd *out)
{
    if ((ctx == 0) || (out == 0))
    {
        return 0U;
    }
    if (ctx->q_count == 0U)
    {
        return 0U;
    }

    *out = ctx->queue[ctx->q_rd];
    ctx->q_rd = (uint8_t)((ctx->q_rd + 1U) % (uint8_t)HOST_CMD_QUEUE_LEN);
    ctx->q_count--;
    return 1U;
}

uint8_t HostCmdParser_HasPartialLine(const HostCmdParser *ctx)
{
    if (ctx == 0)
    {
        return 0U;
    }
    return (ctx->line_len != 0U) ? 1U : 0U;
}

