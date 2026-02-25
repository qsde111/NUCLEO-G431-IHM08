#ifndef COMPONENTS_HOST_CMD_PARSER_H
#define COMPONENTS_HOST_CMD_PARSER_H

/**
 * @file host_cmd_parser.h
 * @brief Host 命令解析器（严格分隔符模式）。
 *
 * 约束（本项目实验用）：
 * - 上位机命令必须以 `\r` / `\n` / `;` 结束，否则不会被解析入队。
 * - 命令格式：`<op><value>`，例如 `V10`、`I-0.5`、`C1`、`D3`；`value` 可省略（如 `D`）。
 * - 为了减少阅读负担，不再支持“无结束符 idle flush / 退格 / 非打印字符过滤”等终端友好逻辑。
 * - 本解析器不会自动转换大小写、不会跳过空格；上位机需发送大写且无空格的命令。
 */

#include <stdint.h>

#ifndef HOST_CMD_MAX_LINE
#define HOST_CMD_MAX_LINE (32U)
#endif

#ifndef HOST_CMD_QUEUE_LEN
#define HOST_CMD_QUEUE_LEN (8U)
#endif

typedef struct
{
    char op;
    float value;
    uint8_t has_value;
} HostCmd;

typedef struct
{
    char line[HOST_CMD_MAX_LINE + 1U];
    uint16_t line_len;

    HostCmd queue[HOST_CMD_QUEUE_LEN];
    uint8_t q_wr;
    uint8_t q_rd;
    uint8_t q_count;
} HostCmdParser;

/**
 * @brief 喂入新增字节流；当遇到分隔符（`\r`/`\n`/`;`）时解析并入队。
 * @param ctx 解析器上下文。
 * @param data 字节数据。
 * @param len 字节数。
 */
void HostCmdParser_Feed(HostCmdParser *ctx, const uint8_t *data, uint16_t len);

/**
 * @brief 弹出一条已解析的命令。
 * @param ctx 解析器上下文。
 * @param out 输出命令。
 * @return 1=成功；0=队列为空或参数无效。
 */
uint8_t HostCmdParser_Pop(HostCmdParser *ctx, HostCmd *out);

#endif /* COMPONENTS_HOST_CMD_PARSER_H */
