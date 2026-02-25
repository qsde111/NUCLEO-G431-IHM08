#include "host_cmd_parser.h"

#include <stdlib.h>

/* 严格分隔符模式：仅解析以 `\r` / `\n` / `;` 结束的命令。
 * 命令格式：`<OP><value>`，例如 `V10` / `C1` / `D3` / `I-0.5`。
 * 约束：本解析器不处理空格、不自动大小写转换。 */

static void HostCmdParser_QueuePush(HostCmdParser *ctx, const HostCmd *cmd)
{
    ctx->queue[ctx->q_wr] = *cmd;
    ctx->q_wr = (uint8_t)((ctx->q_wr + 1U) % (uint8_t)HOST_CMD_QUEUE_LEN);

    if (ctx->q_count >= (uint8_t)HOST_CMD_QUEUE_LEN)
    {
        /* 队列满：覆盖最旧的一条（丢弃 q_rd 所指向的元素） */
        ctx->q_rd = ctx->q_wr;
    }
    else
    {
        ctx->q_count++;
    }
}

static void HostCmdParser_ParseAndEnqueue(HostCmdParser *ctx)
{
    /* Feed() 只会保证 line_len <= HOST_CMD_MAX_LINE */
    ctx->line[ctx->line_len] = '\0';

    HostCmd cmd = {0};
    cmd.op = ctx->line[0];

    if (ctx->line_len > 1U)
    {
        const char *value_str = &ctx->line[1];
        char *endp = 0;
        const float v = strtof(value_str, &endp);
        if ((endp != 0) && (endp != value_str))
        {
            if (*endp == '\0')
            {
                cmd.has_value = 1U;
                cmd.value = v;
            }
        }
    }

    HostCmdParser_QueuePush(ctx, &cmd);
    ctx->line_len = 0U;
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
            /* 严格分隔符模式：仅在分隔符到来时解析并入队 */
            if (ctx->line_len != 0U)
            {
                HostCmdParser_ParseAndEnqueue(ctx);
            }
            continue;
        }

        /* 最小边界保护：只截断，不做溢出状态机/回退/过滤 */
        if (ctx->line_len < (uint16_t)HOST_CMD_MAX_LINE)
        {
            ctx->line[ctx->line_len] = c;
            ctx->line_len++;
        }
    }
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
