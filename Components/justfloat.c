#include "justfloat.h"

#include <string.h>

void JustFloat_Pack4(float f1, float f2, float f3, float f4, uint8_t frame_out[20])
{
    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7fU};

    if (frame_out == 0)
    {
        return;
    }

    memcpy(&frame_out[0], &f1, sizeof(float));
    memcpy(&frame_out[4], &f2, sizeof(float));
    memcpy(&frame_out[8], &f3, sizeof(float));
    memcpy(&frame_out[12], &f4, sizeof(float));
    memcpy(&frame_out[16], tail, sizeof(tail));
}
