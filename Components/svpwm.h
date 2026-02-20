#ifndef COMPONENTS_SVPWM_H
#define COMPONENTS_SVPWM_H

#include <stdint.h>

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector; /* 1..6, 0 = invalid */
} SvpwmOut;

void Svpwm_Calc(float u_alpha, float u_beta, SvpwmOut *out);

#endif /* COMPONENTS_SVPWM_H */

