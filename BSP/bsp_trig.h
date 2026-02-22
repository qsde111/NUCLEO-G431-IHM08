#ifndef BSP_TRIG_H
#define BSP_TRIG_H

#ifdef __cplusplus
extern "C" {
#endif

void BspTrig_Init(void);
void BspTrig_SinCos(float theta_rad, float *sin_out, float *cos_out);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TRIG_H */

