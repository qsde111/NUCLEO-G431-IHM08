#ifndef APP_MOTOR_APP_H
#define APP_MOTOR_APP_H

/**
 * @file motor_app.h
 * @brief 电机实验应用（FOC 框架 + HostCmd + PWM/ADC/Encoder 组合）。
 *
 * HostCmd 命令表（建议以 `\r` / `\n` / `;` 结束；也支持 idle flush）：
 *
 * - `P<deg>`：占位（保存目标位置，当前不参与控制）。例如 `P90`。
 * - `V<rad_s>`：占位（保存目标速度，当前不参与控制）。例如 `V10`。
 * - `C1` / `C`：启动一次校准（ALIGN -> SPIN -> DONE/FAIL），校准完成会自动关闭 PWM 输出。
 * - `C0`：中止校准并关闭 PWM 输出（软件停机手段；硬件急停更可靠）。
 * - `I<iq_A>`：设置电流环 `Iq_ref`（单位 A，带限幅），并在 offset 就绪后自动使能输出。
 * - `I`：关闭电流环并关闭 PWM 输出。
 * - `T<ud_pu>`：开环电压测试（Ud，单位为 per-unit，默认 0.05，带限幅）。
 * - `T0`：停止电压测试并关闭 PWM 输出。
 * - `M0`：读取 MT6835 寄存器 `0x011`（内部滤波带宽 BW[2:0]）并切到 D4 页打印。
 * - `M1`：写 MT6835 寄存器 `0x011` 的 BW[2:0]（高 5 位保持不变，BW 默认写 7），再读回校验并切到 D4 页打印。
 * - `D` / `D<n>`：切换/设置数据流页面（JustFloat_Pack4）。
 */

#include "bsp_adc_inj_pair.h"
#include "bsp_mt6835_dma.h"
#include "bsp_spi3_fast.h"
#include "bsp_tim1_pwm.h"
#include "bsp_trig.h"
#include "bsp_uart_dma.h"
#include "current_sense.h"
#include "foc_current_ctrl.h"
#include "host_cmd_app.h"
#include "justfloat.h"
#include "motor_calib.h"
#include "mt6835.h"
#include "svpwm.h"

typedef struct
{
    BspUartDma uart;
    HostCmdApp host_cmd;
    BspTim1Pwm pwm;
    BspAdcInjPair adc_inj;
    BspMt6835Dma enc_dma;
    BspSpi3Fast spi;
    Mt6835 encoder;

    uint8_t tx_frame[20];
    uint32_t last_stream_tick_ms;
    uint32_t raw21;
    float pos_mech_rad;
    uint16_t enc_div_countdown;
    uint8_t enc_dma_enable;

    uint16_t vbus_raw;
    float vbus_v;
    uint32_t last_vbus_tick_ms;

    uint16_t adc1_raw;
    uint16_t adc2_raw;
    CurrentSenseOffset2 i_ab_offset;
    uint16_t i_u_offset_raw;
    uint16_t i_v_offset_raw;
    uint16_t i_w_offset_raw;
    uint8_t i_offset_stage;
    uint8_t i_offset_ready;
    float ia_a;
    float ib_a;
    float ic_a;

    uint8_t spd_valid;
    float spd_theta_prev_rad;
    float spd_omega_diff_rad_s;
    float spd_pll_theta_hat_rad;
    float spd_pll_omega_int_rad_s;
    float spd_omega_pll_rad_s;

    float i_trip_a;
    uint8_t fault_overcurrent;

    uint32_t adc_isr_count;
    uint8_t dbg_calib_state;
    float dbg_theta_e;
    float dbg_ud;
    float dbg_uq;
    float dbg_duty_a;
    float dbg_duty_b;
    float dbg_duty_c;
    float dbg_id_a;
    float dbg_iq_a;
    float dbg_ud_pu;
    float dbg_uq_pu;
    float dbg_omega_diff_rad_s;
    float dbg_omega_pll_rad_s;

    HostCmd last_host_cmd;
    uint32_t last_host_cmd_tick_ms;

    float target_pos_deg;
    float target_vel_rad_s;
    uint8_t calib_request;
    uint8_t calib_request_pending;

    MotorCalib calib;
    int8_t elec_dir;
    float elec_zero_offset_rad;
    uint8_t calib_done;
    uint8_t calib_fail;

    FocCurrentCtrl i_ctrl;
    float id_ref_a; // 直轴给定电流
    float iq_ref_a; // 交轴给定电流
    float i_limit_a;
    uint8_t i_loop_enabled;
    uint8_t i_loop_enable_pending;

    uint8_t tx_debug_toggle;
    uint8_t stream_page;

    uint8_t vtest_active;
    float vtest_ud;
    float vtest_uq;

    uint8_t mt6835_reg011;
    uint8_t mt6835_reg011_valid;
    uint8_t mt6835_reg_op;
} MotorApp;

/**
 * @brief 初始化 MotorApp（启动 ADC Injected + PWM trigger + UART RX DMA 等）。
 * @param ctx 应用上下文。
 * @param huart Host 串口。
 * @param hspi 编码器 SPI。
 * @param cs_port 编码器 CS 端口。
 * @param cs_pin 编码器 CS 引脚。
 * @param htim_pwm TIM1（PWM + TRGO2 触发 ADC injected）。
 * @param hadc1 ADC1。
 * @param hadc2 ADC2。
 */
void MotorApp_Init(MotorApp *ctx, UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   TIM_HandleTypeDef *htim_pwm, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);

/**
 * @brief 主循环任务：处理 HostCmd + 数据流输出等（控制 tick 由 ADC injected ISR 驱动）。
 * @param ctx 应用上下文。
 */
void MotorApp_Loop(MotorApp *ctx);

#endif /* APP_MOTOR_APP_H */
