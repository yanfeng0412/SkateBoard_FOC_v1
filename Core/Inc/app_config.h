#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ================================================================
 * app_config.h  -- project-wide hardware & feature configuration
 *
 * Edit ONLY this file to adapt the firmware to different hardware.
 * All motor parameters, PID tuning constants, and build switches
 * are centralised here so they can be found and changed in one place.
 * ================================================================ */

/* -----------------------------------------------------------------------
 * Motor hardware
 * ----------------------------------------------------------------------- */

/* =======================================================================
 * User Defined Limits
 * ======================================================================= */
/* Target mechanical speed limit for the motor. 
 * Other limits (EMI bounds, rate limiters) scale automatically from this. */
#define USER_MAX_SPEED_KMH          25.0f


/* =======================================================================
 * Hardware Parameters 
 * ======================================================================= */
/* Number of electrical pole pairs.
 * 14-pole motor = 7 pole pairs.
 * Affects FOC electrical-angle calculation AND Hall speed measurement. */
#define MOTOR_POLE_PAIRS            7U

/* Motor winding direction polarity: +1 or -1.
 * Flip the sign if the wheel spins backwards under positive throttle. */
#define MOTOR_DIRECTION             (-1)

/* Physical wheel outer diameter in millimetres. */
#define MOTOR_WHEEL_DIAMETER_MM     50.0f

/* Motor KV rating (RPM / V, no-load, unloaded).\n * Used for back-EMF estimate and FOC_MAX_PLAUSIBLE_KMH auto-calculation. */
#define MOTOR_KV                    270.0f

/* Phase resistance (Ohm, line-to-neutral, measured or datasheet).
 * Used as reference for VQ_CAL current estimate only �?voltage-mode FOC
 * does not require this for control, but it helps choose safe VQ_CAL.
 * At VQ_CAL × Vbus / MOTOR_PHASE_R <= I_max.  E.g. 0.10 × 24 / 0.1 = 24 A. */
#define MOTOR_PHASE_R               0.10f

/* Phase inductance (Henry, line-to-neutral).
 * Placeholder �?reserved for future current-loop implementation.
 * Typical hub motor: 50-200 µH. */
#define MOTOR_PHASE_L               100e-6f

/* Nominal and maximum bus voltage (V).
 * Nominal: used for power calculations.
 * Max: fault threshold �?above this value disable drive output. */
#define MOTOR_BUS_VOLTAGE_NOM       24.0f
#define MOTOR_BUS_VOLTAGE_MAX       29.4f   /* 7S Li-ion fully charged */
#define MOTOR_BUS_VOLTAGE_MIN       18.0f   /* 7S Li-ion cutoff */

/* -----------------------------------------------------------------------
 * FOC Hall auto-mapping calibration parameters
 * ----------------------------------------------------------------------- */

/* Open-loop drive voltage during calibration (normalised 0..1).
 *
 * DC bus current estimate (stall, 3-phase sinusoidal SPWM):
 *   I_dc = 3 × (VQ × Vbus / 2 / R_phase)² × R_phase / Vbus
 *   Example R_phase=0.05 Ω (typical hub motor), VQ=0.15, Vbus=24 V:
 *   I_dc �?4 A  �? set bench power supply limit to �?10 A.
 *
 * If power supply is limited to 4 A: use VQ=0.10 (I_dc �?1.8 A).
 * If power supply is limited to 8 A: use VQ=0.15 (I_dc �?4 A).
 * If motor still stalls after supply limit raised: increase to 0.20. */
#define FOC_CAL_VQ                  0.06f   /* Q1: Dropped calibration voltage for gentler current */

/* Sweep rate: milliseconds per electrical degree.
 * 3 ms/deg = ~8 RPM mech (7pp) -- faster sequence. */
#define FOC_CAL_STEP_MS             5U      /* Q4: Accelerated open-loop Hall check sweep */

/* Number of full MECHANICAL revolutions to sweep during calibration.
 * Total time = POLE_PAIRS × MECH_REVS × 360 × STEP_MS
 *            = 7 × 2 × 360 × 3 ms = 15.1 s                          */
#define FOC_CAL_MECH_REVS           2

/* Minimum samples required per Hall position to count as valid.
 * With stall-rejection (only count when stall_counter < 30), each
 * crossing of a sector gives at most 30 samples.  With 2 mech revs
 * = 14 electrical revs = ~14 crossings per sector, expected ~14*10
 * samples per sector (10 out of 30 typically accumulate cleanly).
 * Set threshold to just 10 to accept positions seen even once cleanly.  */
#define FOC_CAL_MIN_SAMPLES         10

/* -----------------------------------------------------------------------
 * Hall speed measurement
 * ----------------------------------------------------------------------- */

/* Speed sample window (seconds).  HallSpeed_Timer() is called every
 * HALL_SPEED_PERIOD seconds from app_timer; count resets each call. */
#define HALL_SPEED_PERIOD           0.1f

/* IIR low-pass filter coefficient applied to the speed output.
 * Range (0, 1].  Higher alpha -> heavier smoothing, more lag. */
#define RPM_FILTER_ALPHA            0.2f

/* =======================================================================
 * FOC PID parameters -- tune on hardware; start with small Ki
 *
 * NOTE: The PID loop runs very fast (~毫秒�?, but Hall speed only updates
 * every 100ms. Therefore, D-term MUST be 0, and P/I gains must be VERY low
 * to prevent the PID from overreacting between Hall updates.
 * ======================================================================= */
#define FOC_SPEED_PID_KP            0.06f
#define FOC_SPEED_PID_KI            0.05f
#define FOC_SPEED_PID_KD            0.000f
#define FOC_SPEED_PID_ILIM          40.0f   /* 恢复：开放积分上限，允许爬升以克服高速反电动势 */
#define FOC_SPEED_PID_OLIM          0.80f   /* 恢复：放开占空比，允许输出高达 80% 的母线电压 */

/* Position loop : output is speed setpoint (km/h) */
#define FOC_POS_PID_KP              0.05f
#define FOC_POS_PID_KI              0.001f
#define FOC_POS_PID_KD              0.002f
#define FOC_POS_PID_ILIM            20.0f   /* km/h integral limit        */
#define FOC_POS_PID_OLIM            20.0f   /* km/h max speed command     */

/* -----------------------------------------------------------------------
 * Temperature ADC thresholds
 * (raw NTC ADC counts; lower counts = hotter on a pull-up divider)
 * ----------------------------------------------------------------------- */
#define TEMPERATURE_HIGH            30
#define TEMPERATURE_WARNING         50
#define TEMPERATURE_ERROR           500

/* -----------------------------------------------------------------------
 * Debug / test-mode switches  (0 = off, 1 = on)
 * ----------------------------------------------------------------------- */

/* UART3_LOCAL_TEST_MODE
 *   1 = bypass NRF watchdog; accept all commands via UART3 directly.
 *   0 = production mode -- NRF link must be alive or watchdog fires. */
#ifndef UART3_LOCAL_TEST_MODE
#define UART3_LOCAL_TEST_MODE       1
#endif

/* DISABLE_TEMPERATURE_SENSOR
 *   1 = NTC not populated; sensor reads are skipped / faked.
 *   0 = NTC thermistor installed and checked at boot. */
#ifndef DISABLE_TEMPERATURE_SENSOR
#define DISABLE_TEMPERATURE_SENSOR  1
#endif

/* -----------------------------------------------------------------------
 * Power-on self-check items  (0 = skip, 1 = must pass before running)
 * ----------------------------------------------------------------------- */
#define SELFCHECK_BATTERY           1
#define SELFCHECK_HALL              1
#define SELFCHECK_CURRENT           1

/* SELFCHECK_TEMPERATURE is derived from DISABLE_TEMPERATURE_SENSOR. */
#if DISABLE_TEMPERATURE_SENSOR
#  define SELFCHECK_TEMPERATURE     0
#else
#  define SELFCHECK_TEMPERATURE     1
#endif

/* -----------------------------------------------------------------------
 * Open-loop startup parameters
 *
 * When |normalised speed| < FOC_OL_EXIT_SPEED_KMH the drive angle is
 * swept open-loop (no Hall feedback) to overcome static friction and
 * get the rotor moving.  Above the threshold the angle switches to
 * Hall-based closed-loop estimation.
 *
 * FOC_OL_EXIT_SPEED_KMH: hand-off threshold in km/h.
 * FOC_OL_DEG_PER_MS    : open-loop sweep rate scaling.
 *   effective_rate = startup_vq_limit * FOC_OL_DEG_PER_MS  (el-deg/ms)
 *   at max vq=0.8, value=2.0 -> 1600 el-deg/s ~= 0.65 km/h (> threshold)
 * ----------------------------------------------------------------------- */
/* FOC_OL_KMH_TO_VQ: converts target speed (km/h) to the OL clamp voltage (Vq normalised).
 * Derived from back-EMF: Vq = speed_m_s / (KV_rad_s_per_V * r_m)
 *   = speed_kmh / 3.6 / (KV_rpm_V * 2*pi/60 * wheel_r_m)
 *   = speed_kmh * 60 / (3.6 * KV * Vbus * pi * wheel_d_m)
 *
 * FOC_OL_TORQUE_MARGIN: multiplier on top of the pure BEMF estimate to provide
 * torque headroom for friction (bearing + hub motor drag).  Pure BEMF gives
 * exactly zero torque margin at the target speed; the motor freewheels rather
 * than accelerating.  1.6 means +60% headroom above BEMF — at 4 km/h this
 * raises OL Vq from 0.073 to 0.117, enough to overcome static friction.
 * Examples with margin (KV=270, Vbus=24, D=50mm):
 *   2 km/h  → 0.053 Vq
 *   4 km/h  → 0.117 Vq  (was 0.073, too low to overcome friction)
 *   7 km/h  → 0.204 Vq
 *   10 km/h → 0.262 Vq                                                      */
#define FOC_OL_TORQUE_MARGIN       1.6f
#define FOC_OL_KMH_TO_VQ  (FOC_OL_TORQUE_MARGIN * 60.0f / (3.6f * MOTOR_KV * MOTOR_BUS_VOLTAGE_NOM \
                           * 3.14159f * (MOTOR_WHEEL_DIAMETER_MM * 0.001f)))

/* Minimum OL voltage regardless of setpoint, so the motor always at least
 * reaches FOC_OL_EXIT_SPEED_KMH (1.5 km/h) even at very low speed commands.
 * 0.04 → free-run ≈ 2.4 km/h > 1.5 km/h exit threshold.                   */
#define FOC_OL_MIN_VQ              0.06f
#define FOC_OL_EXIT_SPEED_KMH       1.5f

/* FOC_USE_OPEN_LOOP
 *   1 = OL sweep on startup, hand off to CL when speed confirmed (Hall diag / cogging workaround).
 *   0 = always closed-loop from the first tick; PID drives angle directly from Hall.
 *       Use this once Hall is calibrated and motor starts reliably. */
#define FOC_USE_OPEN_LOOP           0
#define FOC_OL_EXIT_MIN_HALL_COUNT  4U    /* restore stricter entry to avoid EMI/noise early handoff */
#define FOC_OL_EXIT_HOLD_MS         120U  /* restore hold time: require stable movement before CL */
#define FOC_OL_LOCK_MAX_ERR_DEG     45.0f
#define FOC_OL_ABORT_SPEED_KMH      10.0f
#define FOC_OL_DEG_PER_MS           2.0f  /* el-deg per ms per unit of drive_vq (slow sweep; motor synchronises, then free-runs) */

/* FOC_MAX_PLAUSIBLE_KMH
 * Hall EMI plausibility clamp.  Any speed reading above this value is
 * considered a false trigger from inrush-current noise and is clamped.
 * Scaled automatically to 1.5x the user's max speed limit.
 */
#define FOC_MAX_PLAUSIBLE_KMH       (USER_MAX_SPEED_KMH * 1.5f)

/* FOC_SPEED_RATE_LIMIT_KMHS
 * Maximum rate of change of the PID speed feedback (km/h per second).
 * Dynamically limits acceleration to reach user max speed in 1 second.
 */
#define FOC_SPEED_RATE_LIMIT_KMHS   (USER_MAX_SPEED_KMH * 1.0f)

/* FOC_CALIBRATE_ON_BOOT
 *   1 = run FOC_CalibrateHallOffset() automatically at power-on.
 *       Wheel MUST be free to rotate during boot (~4 s).
 *       Result is printed on UART1 and stored in foc_hall_offset_deg.
 *   0 = skip auto-calibration (use foc_hall_offset_deg default = 0).
 *       Trigger manually via UART1 command 'c' during debugging.
 *
 * NOTE: calibration result is NOT saved to flash yet - it resets on
 * every reboot unless you add Store / flash persistence.
 */
#ifndef FOC_CALIBRATE_ON_BOOT
#define FOC_CALIBRATE_ON_BOOT      1 // ENABLED: Must run to learn new Hall offset   /* Hall LUT calibrated; skip sweep at every boot */
#endif

/* FOC_HALL_DIAG_ON_BOOT
 *   1 = run FOC_HallDiagnostic() for 15 s at boot BEFORE calibration.
 *       Rotate the wheel by hand during this window.
 *       All 6 Hall positions must appear �?if not, fix wiring first.
 *   0 = skip (normal operation).
 */
#ifndef FOC_HALL_DIAG_ON_BOOT
#define FOC_HALL_DIAG_ON_BOOT      0
#endif

#endif /* __APP_CONFIG_H */
