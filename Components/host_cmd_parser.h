#ifndef COMPONENTS_HOST_CMD_PARSER_H
#define COMPONENTS_HOST_CMD_PARSER_H

/**
 * @file host_cmd_parser.h
 * @brief Host 命令解析器（字节流 -> HostCmd 队列）。
 *
 * 解析规则：
 * - 命令以 `\r` / `\n` / `;` 结束。
 * - 格式：`<op><value>`，例如 `V10`、`I-0.5`、`C1`、`D3`；`value` 可省略（如 `D`）。
 * - 自动跳过前导空格，并将 op 转换为大写；数值支持 `-12` / `3.14`。
 *
 * 说明：
 * - 为了适配串口终端/人手输入，Feed() 会处理退格、忽略不可打印字符，并提供溢出保护。
 * - 若上位机可能不发送结束符，可在上层根据超时调用 HostCmdParser_FlushLine()。
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
    char op;           // Operation Code，操作码，存放指令的字母部分
    float value;       // 存放指令的数值部分
    uint8_t has_value; // 是否有指令
} HostCmd;

typedef struct
{
    char line[HOST_CMD_MAX_LINE + 1U];
    uint16_t line_len;
    uint8_t line_overflow;

    HostCmd queue[HOST_CMD_QUEUE_LEN]; // Ring Buffer环形队列，解决指令接收太快，处理太慢的问题
    uint8_t q_wr;                      // Write 写索引，代表当前结构体数哪个位置为空，可以写入指令
    uint8_t q_rd;                      // 读索引
    uint8_t q_count;                   // 队列里存了多少个还没有被处理的指令
} HostCmdParser;

/**
 * @brief 初始化解析器。
 * @param ctx 解析器上下文。
 */
void HostCmdParser_Init(HostCmdParser *ctx);

void HostCmdParser_Feed(HostCmdParser *ctx, const uint8_t *data, uint16_t len);

/**
 * @brief 立即把当前缓冲区当作一条命令进行解析入队（用于“无结束符 + 超时 flush”策略）。
 * @param ctx 解析器上下文。
 */
void HostCmdParser_FlushLine(HostCmdParser *ctx);

uint8_t HostCmdParser_Pop(HostCmdParser *ctx, HostCmd *out);

/**
 * @brief 查询是否存在“未结束的一行”（用于上层超时 flush）。
 * @param ctx 解析器上下文。
 * @return 1=有；0=无或参数无效。
 */
uint8_t HostCmdParser_HasPartialLine(const HostCmdParser *ctx);

#endif /* COMPONENTS_HOST_CMD_PARSER_H */
