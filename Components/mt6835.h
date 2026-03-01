#ifndef COMPONENTS_MT6835_H
#define COMPONENTS_MT6835_H

#include <stdint.h>

#define MT6835_STREAM_CMD_H (0xA0U)
#define MT6835_STREAM_CMD_L (0x03U)

typedef struct
{
    void *user;
    uint8_t (*transfer8)(void *user, uint8_t tx_byte);
    void (*cs_low)(void *user);
    void (*cs_high)(void *user);
} Mt6835BusOps;

typedef struct
{
    Mt6835BusOps bus;
    uint8_t stream_started;
} Mt6835;

void Mt6835_Init(Mt6835 *ctx, const Mt6835BusOps *bus);
uint32_t Mt6835_ReadRaw21(Mt6835 *ctx);
uint8_t Mt6835_ReadReg8(Mt6835 *ctx, uint16_t reg, uint8_t *val_out);
uint8_t Mt6835_WriteReg8(Mt6835 *ctx, uint16_t reg, uint8_t val);
float Mt6835_Raw21ToRad(uint32_t raw21);
float Mt6835_Raw21ToDeg(uint32_t raw21);

#endif /* COMPONENTS_MT6835_H */
