#ifndef COMPONENTS_MOTOR_CALIB_H
#define COMPONENTS_MOTOR_CALIB_H

#include <stdint.h>

typedef enum
{
    MOTOR_CALIB_IDLE = 0,
    MOTOR_CALIB_ALIGN,
    MOTOR_CALIB_SPIN,
    MOTOR_CALIB_DONE,
    MOTOR_CALIB_FAIL,
} MotorCalibState;

typedef struct
{
    float pole_pairs;

    /* Per-unit voltage commands (normalized to Vbus) */
    float ud_align;
    float uq_spin;

    /* Open-loop electrical speed during SPIN */
    float omega_e_rad_s;

    /* Timing in control ticks (e.g. 20kHz tick) */
    uint32_t align_ticks;
    uint32_t spin_ticks;

    /* Minimum mechanical movement required to determine direction */
    float min_move_rad;
} MotorCalibParams;

typedef struct
{
    float ud;
    float uq;
    float theta_e;
} MotorCalibCmd;

typedef struct
{
    MotorCalibState state;
    MotorCalibParams p;

    uint32_t tick;
    float theta_e_cmd;

    float theta_mech_align;
    float theta_mech_start;
    float theta_mech_end;

    int8_t dir;            /* +1 or -1 */
    float zero_offset_rad; /* 0..2pi */
} MotorCalib;

void MotorCalib_Init(MotorCalib *ctx, const MotorCalibParams *params);
void MotorCalib_Start(MotorCalib *ctx, float theta_mech_rad);
void MotorCalib_Abort(MotorCalib *ctx);

void MotorCalib_Tick(MotorCalib *ctx, float dt_s, float theta_mech_rad);
uint8_t MotorCalib_GetCmd(const MotorCalib *ctx, MotorCalibCmd *out);

MotorCalibState MotorCalib_State(const MotorCalib *ctx);
int8_t MotorCalib_Dir(const MotorCalib *ctx);
float MotorCalib_ZeroOffsetRad(const MotorCalib *ctx);

#endif /* COMPONENTS_MOTOR_CALIB_H */

