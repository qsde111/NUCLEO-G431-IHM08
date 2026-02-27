#include "host_cmd_parser.h"

#include <string.h>

/* Host 命令解析器：
 * - Feed() 把字节流拼成一行；遇到 `\r`/`\n`/`;` 时解析并入队
 * - 支持退格、忽略不可打印字符、溢出保护
 * - 若上位机不发送结束符，可由上层根据超时调用 FlushLine() */

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
/**
 * @brief
 *
 * @param s 字符串line中剩余数字字符首位地址的指针
 * @param out  外部用来接收数值的临时缓冲区的指针
 * @param endp
 * 外部用来记录解析到哪里的字符指针的指针，是一个二级指针，由于要改变该指针的值(指向的地址)，所以要传入指针的地址-二级指针
 * @return uint8_t
 */
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

    uint32_t int_part = 0U;  // Integer Part （整数部分）
    uint32_t frac_part = 0U; // Fraction Part （小数部分）
    uint32_t frac_div = 1U;  // Fraction Divider （小数的除数）
    uint8_t have_digit = 0U; // Have Digit （是否包含数字）

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

/**
 * @brief   处理 HostCmdParser *ctx->line中字符串的首字母指令和后续数值
 * 将指令存放到HostCmdParser *ctx->HostCmd queue[HOST_CMD_QUEUE_LEN];中
 * @param ctx
 */
static void HostCmdParser_ParseAndEnqueue(HostCmdParser *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->line[ctx->line_len] = '\0'; /* 手动补 '\0' 方便按 C 字符串解析 */

    const char *p = ctx->line;     // line数组名在表达式中自动退化为指向其首元素的指针，指针p指向了line首元素的地址
    while (HostCmdParser_IsWs(*p)) /* 跳过前导空格 */
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
    cmd.op = HostCmdParser_ToUpper(*p); /* 小写指令字母转换为大写,放入HostCmd结构体下的op中 */
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

/**
 * @brief 处理新增字节流并提取有效指令。
 * * 详细说明：
 * 从 data 指向的地址读取 len 字节，过滤退格符及非打印字符。
 * 将清洗后的数据组装至HostCmdParser *ctx中的字符串line 缓冲区。
 * 一旦识别到结束符，立即调用 HostCmdParser_ParseAndEnqueue 进行解析。
 * * @param ctx  HostCmdParser 结构体指针。
 * @param data DMA 接收缓冲区的新数据起始地址。
 * @param len  本次处理的字节长度。
 */
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
            /* 遇到结束符，缓冲区内已构成完整指令，触发解析并入队 */
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

        if ((c == '\b') || (c == 0x7FU)) /* 处理退格符 */
        {
            if (ctx->line_len != 0U)
            {
                ctx->line_len--;
            }
            continue;
        }

        if ((c < 0x20) || (c > 0x7EU)) /* 忽略非打印字符 */
        {
            continue;
        }

        if (ctx->line_len >= (uint16_t)HOST_CMD_MAX_LINE) /* 溢出判断 */
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
/**
 * @brief 将ctx->queue中存解析好的指令搬运到外部传入的HostCmd *out中
 */
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
