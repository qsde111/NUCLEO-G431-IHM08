#ifndef COMPONENTS_MOTOR_CALIB_H
#define COMPONENTS_MOTOR_CALIB_H

#include <stdint.h>

/* 在C语言底层，枚举（enum）其实就是整型数字（int）。
IDLE 默认是 0，ALIGN 是 1，SPIN 是 2，DONE 是 3，FAIL 是 4。 */
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
    uint32_t align_ticks; // 对齐阶段保持时间
    uint32_t spin_ticks;  // 拖动阶段持续时间

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

    uint32_t tick; // 高精度软件定时器，每次自增代表时间过去了一次ADC中断周期
    float theta_e_cmd;

    float theta_mech_align; // 对齐时-机械角度
    float theta_mech_start; // 旋转开始时-机械角度
    float theta_mech_end;   // 旋转结束时-机械角度

    int8_t dir;            /* +1 or -1 */
    float zero_offset_rad; /* 0..2pi 电角度零点偏移量 */
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
