#ifndef COMPONENTS_SIGNAL_CONFIG_H
#define COMPONENTS_SIGNAL_CONFIG_H

/* ========= MotorApp: log sweep (chirp) injection defaults =========
 *
 * Used by HostCmd `F1/F0` in MotorApp.
 * You can reuse `SignalLogSweep` elsewhere with different parameters.
 */

#ifndef MOTORAPP_LOG_SWEEP_DIV
/* sweep_hz = CTRL_HZ / DIV */
#define MOTORAPP_LOG_SWEEP_DIV (10U)
#endif

#ifndef MOTORAPP_LOG_SWEEP_AMP_A
#define MOTORAPP_LOG_SWEEP_AMP_A (0.1f)
#endif

#ifndef MOTORAPP_LOG_SWEEP_F_START_HZ
#define MOTORAPP_LOG_SWEEP_F_START_HZ (1.0f)
#endif

#ifndef MOTORAPP_LOG_SWEEP_F_END_HZ
#define MOTORAPP_LOG_SWEEP_F_END_HZ (50.0f)
#endif

#ifndef MOTORAPP_LOG_SWEEP_DURATION_S
#define MOTORAPP_LOG_SWEEP_DURATION_S (20.0f)
#endif

#endif /* COMPONENTS_SIGNAL_CONFIG_H */
