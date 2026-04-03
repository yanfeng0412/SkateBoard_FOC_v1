#ifndef __FOC_H
#define __FOC_H

#include "stm32f1xx_hal.h"
#include "app_config.h"
#include <stdint.h>

/* ================================================================
 * FOC — sinusoidal voltage-mode vector control
 *
 * Hardware assumptions:
 *   High-side PWM : TIM3 CH2(PA7=UH)  CH3(PB0=VH)  CH4(PB1=WH)
 *   Low-side GPIO : PA6(UL)  PB7(VL)  PB8(WL)  — all SET in FOC mode
 *
 * In FOC mode all three low-side enables are driven HIGH and the
 * three high-side channels carry sinusoidal duty cycles.
 * The gate driver must provide its own shoot-through deadtime
 * (IR2104 / DRV8xxx style).
 *
 * Current sensing: only a single DC-bus shunt (adc_values[0]/PA1).
 * Individual phase currents are unavailable, so this is voltage-mode
 * control. "Torque" is approximated by controlling Vq (no current loop).
 *
 * Angle: Hall sensors give 60° resolution; DWT-based linear interpolation
 * within each sector improves effective resolution at speed.
 * ================================================================ */

/* --------------------------------------------------------------- *
 * Control modes
 * ------------------------------------------------------------- */
typedef enum {
    CTRL_MODE_BLDC     = 0,   /* Original 6-step via motor.c (default)  */
    CTRL_MODE_TORQUE   = 1,   /* Direct Vq — open-loop torque proxy      */
    CTRL_MODE_SPEED    = 2,   /* Speed PID → Vq, Hall speed feedback     */
    CTRL_MODE_POSITION = 3,   /* Position PID → speed PID → Vq           */
} FOC_ControlMode_t;

/* --------------------------------------------------------------- *
 * PID controller
 * ------------------------------------------------------------- */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;   /* anti-windup clamp */
    float output_limit;     /* symmetric output clamp ±value */
} PID_t;

/* --------------------------------------------------------------- *
 * FOC state (singleton, exposed for debug / telemetry)
 * ------------------------------------------------------------- */
typedef struct {
    FOC_ControlMode_t mode;

    /* Angle estimation */
    float  angle_deg;           /* estimated electrical angle 0..360  */
    float  angle_rad;           /* same in radians 0..2π              */
    float  position_accum;      /* accumulated mechanical degrees since boot */

    /* Setpoints */
    float  speed_ref;           /* km/h   (SPEED / POSITION modes)    */
    float  torque_ref;          /* 0..1   (TORQUE mode)                */
    float  position_ref;        /* mechanical degrees (POSITION mode)  */

    /* Output */
    float  vq;                  /* current normalised Vq 0..1         */
    float  startup_angle_err_deg; /* Hall angle minus OL angle, wrapped */
    uint8_t ol_active;          /* 1 while startup is open-loop       */

    /* Filtered speed feedback exposed for diagnostics                */
    float  spd_fb_filt;         /* rate-limited spd fed to PID (km/h) */

    /* Protection */
    float  current_limit;       /* max |vq|, default 0.8              */
    float  startup_vq_limit;    /* soft-start ramp: 0 → current_limit  */

    /* Controllers */
    PID_t  pid_speed;
    PID_t  pid_position;
} FOC_State_t;

/* --------------------------------------------------------------- *
 * Motor parameters — configured in app_config.h
 * ------------------------------------------------------------- */
/* Pole pairs alias (source of truth: MOTOR_POLE_PAIRS in app_config.h) */
#define FOC_POLE_PAIRS          MOTOR_POLE_PAIRS

/* Hall electrical offset (degrees). Set by FOC_CalibrateHallOffset().
 * Can also be forced manually: foc_hall_offset_deg = -120.0f; etc.       */
extern float foc_hall_offset_deg;

/* PID gains are defined in app_config.h as FOC_SPEED_PID_KP/_KI/... etc. */

/* --------------------------------------------------------------- *
 * Global FOC state — include foc.h and use 'foc.xxx' for diagnostics
 * ------------------------------------------------------------- */
extern FOC_State_t foc;

/* --------------------------------------------------------------- *
 * Public API
 * ------------------------------------------------------------- */
void  FOC_Init(void);
void  FOC_SetMode(FOC_ControlMode_t mode);
void  FOC_SetSpeedRef(float speed_kmh);
void  FOC_SetTorqueRef(float torque_norm);      /* 0..1           */
void  FOC_SetPositionRef(float position_deg);   /* mechanical deg */
void  FOC_Run(void);    /* call each main-loop iteration (~1 ms)   */
float FOC_CalibrateHallOffset(void); /* open-loop Hall offset auto-detect    */
void  FOC_HallDiagnostic(uint32_t duration_ms); /* print Hall A/B/C on manual rotation */

/* Hall transition callbacks — called from EXTI handler in hall.c    */
int   FOC_NotifyHallTransition(void);   /* returns 1=accepted, 0=rejected   */
void  FOC_HallPositionAccumulate(void); /* call after dir update when accepted */

/* PID helpers (exposed for external calibration / telemetry) */
void  PID_Init(PID_t *pid, float kp, float ki, float kd,
               float ilim, float olim);
void  PID_Reset(PID_t *pid);
float PID_Compute(PID_t *pid, float setpoint, float feedback, float dt);

#endif /* __FOC_H */
