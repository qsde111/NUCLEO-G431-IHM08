#ifndef COMPONENTS_HOST_CMD_PARSER_H
#define COMPONENTS_HOST_CMD_PARSER_H

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
    uint8_t line_overflow;

    HostCmd queue[HOST_CMD_QUEUE_LEN];
    uint8_t q_wr;
    uint8_t q_rd;
    uint8_t q_count;
} HostCmdParser;

void HostCmdParser_Init(HostCmdParser *ctx);
void HostCmdParser_Feed(HostCmdParser *ctx, const uint8_t *data, uint16_t len);
void HostCmdParser_FlushLine(HostCmdParser *ctx);
uint8_t HostCmdParser_Pop(HostCmdParser *ctx, HostCmd *out);
uint8_t HostCmdParser_HasPartialLine(const HostCmdParser *ctx);

#endif /* COMPONENTS_HOST_CMD_PARSER_H */

