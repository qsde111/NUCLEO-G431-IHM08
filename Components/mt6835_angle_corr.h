#ifndef COMPONENTS_MT6835_ANGLE_CORR_H
#define COMPONENTS_MT6835_ANGLE_CORR_H

#include <stdint.h>

/**
 * @brief MT6835 mechanical-angle correction LUT.
 *
 * The LUT maps raw 21-bit MT6835 counts to corrected 21-bit counts before
 * speed estimation and electrical-angle generation.
 */
uint32_t Mt6835AngleCorr_ApplyRaw21(uint32_t raw21);

#endif /* COMPONENTS_MT6835_ANGLE_CORR_H */
