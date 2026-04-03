#include "foc.h"
#include "hall.h"
#include "motor.h"
#include "app_timer.h"
#include "serial_dbg.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

/* Forward declarations of static helpers defined later in this file */
static void FOC_Stop(void);
#if 0
  static void FOC_ForceAngle(float angle_deg, float vq_norm, uint32_t hold_ms); 
  static void FOC_SweepToAngle(float from_deg, float to_deg, float vq_norm, uint32_t step_ms);
#endif
static float FOC_WrapDiffDeg(float lhs_deg, float rhs_deg);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Hall alignment offset in electrical degrees.
 * 0.0 = ideal (LUT matches physical winding).
 * Written by FOC_CalibrateHallOffset(); can also be set manually.
 * Valid values: multiples of 60 ( -300, -240, -180, -120, -60, 0, 60 ... ) */
float foc_hall_offset_deg = 0.0f;

static float FOC_WrapDiffDeg(float lhs_deg, float rhs_deg)
{
    float diff = lhs_deg - rhs_deg;
    while (diff >  180.0f) { diff -= 360.0f; }
    while (diff < -180.0f) { diff += 360.0f; }
    return diff;
}

/* ================================================================
 * DWT cycle-counter microsecond timer
 * Works on Cortex-M3.  64 MHz �?64 counts per µs.
 * DWT is initialised once in FOC_Init().
 * ================================================================ */
static void DWT_Enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Returns timestamp in microseconds (wraps after ~67 s at 64 MHz) */
static uint32_t DWT_Micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* ================================================================
 * Hall angle estimation with DWT interpolation
 *
 * Hall position �?electrical angle sector base (forward direction):
 *   pos 1 (A=0,B=1,C=0) �?0°   pos 2 (A=0,B=1,C=1) �?60°
 *   pos 3 (A=0,B=0,C=1) �?120° pos 4 (A=1,B=0,C=1) �?180°
 *   pos 5 (A=1,B=0,C=0) �?240° pos 6 (A=1,B=1,C=0) �?300°
 *
 * Between transitions we interpolate linearly using the duration
 * of the previously completed sector.  The fraction is clamped to
 * [0,1] so we never overshoot into the next sector.
 * ================================================================ */
/* Hall position �?electrical angle sector base.
 *
 * Physical FORWARD rotation: pos DECREASING
 *   5 �?4 �?3 �?2 �?1 �?6 �?5 �?...
 * (Confirmed by HALL_DIAG: hand CW rotation gives this sequence.)
 *
 * Values derived directly from CAL measured drive-angles (what angle
 * the open-loop sweep was at when the motor passed through each sector).
 * foc_hall_offset_deg is added on top at runtime (from CAL).
 *
 * CAL measured (latest run):
 *   pos=1 �?157°   pos=2 �?113°   pos=3 �?(not seen, interpolated ~53°�?0°)
 *   pos=4 �?339°   pos=5 �?297°   pos=6 �?270°
 *
 * Rounded to nearest 30° for clean sector centres:
 *   pos=1�?50°, pos=2�?20°, pos=3�?0°, pos=4�?30°, pos=5�?00°, pos=6�?70°
 * ================================================================ */
static const float k_sector_base[7] = {
    0.0f,    /* [0] unused  */
    60.0f,   /* pos=1 (measured ~60) */
    0.0f,    /* pos=2 (measured ~360) */
    300.0f,  /* pos=3 (measured ~300) */
    240.0f,  /* pos=4 (measured ~240) */
    180.0f,  /* pos=5 (measured ~180) */
    120.0f,  /* pos=6 (measured ~120) */
};

static volatile uint32_t s_trans_us        = 0U;     /* DWT time of last transition    */
static volatile uint32_t s_sector_dur_us   = 1000000U; /* last sector duration (~standstill init) */
static volatile float    s_pos_accum_deg   = 0.0f;   /* accumulated mechanical degrees */

/* Rate-limited speed feedback: written by FOC_Run, reset by FOC_SetMode.
 * Prevents a single EMI-induced Hall burst from instantly saturating the
 * PID.  Real motor acceleration is << FOC_SPEED_RATE_LIMIT_KMHS.          */
static float s_spd_fb_filt = 0.0f;

/* Called from HAL_GPIO_EXTI_Callback in hall.c on every Hall edge.
 *
 * Returns 1 (accepted) / 0 (rejected).
 *
 * Debounce raised from 50 us to 500 us: empirically the strongest
 * EMI burst at startup (the one that gave spd_raw=-53.9 km/h) had an
 * apparent inter-transition interval of ~449 us.  500 us rejects it
 * while still allowing real transitions up to ~80 km/h wheel speed.
 *
 * If the transition is rejected the caller MUST NOT update Hall
 * position, count, or direction �?that would corrupt the estimates
 * with noise.  Caller should call FOC_HallPositionAccumulate() after
 * updating direction when this function returns 1.                  */
int FOC_NotifyHallTransition(void)
{
    uint32_t now = DWT_Micros();
    uint32_t dur = now - s_trans_us;

    if (dur < 500U) { return 0; }       /* debounce: skip glitches < 500 us */

    s_sector_dur_us = dur;
    s_trans_us      = now;
    return 1;
}

/* Accumulate mechanical position for the transition that was just accepted.
 * Must be called AFTER Hall_directionLoopCheck() so the freshly-set
 * hall_direction is used (avoids a one-transition sign delay).            */
void FOC_HallPositionAccumulate(void)
{
    float step = 60.0f / (float)FOC_POLE_PAIRS;
    if      (hall_direction > 0) { s_pos_accum_deg += step; }
    else if (hall_direction < 0) { s_pos_accum_deg -= step; }
}

/* Returns interpolated electrical angle in degrees [0, 360) */
static float Hall_ComputeAngleDeg(void)
{
    int pos = hall_sensor.position;
    if (pos < 1 || pos > 6) { return 0.0f; }

    float base = k_sector_base[pos] + foc_hall_offset_deg;

    /* DWT interpolation within the current sector.
     *
     * Without interpolation the drive angle is frozen at the sector centre
     * for the entire sector duration.  The 6 sectors are very uneven on this
     * motor (30°/90°/60°/30°/120°/30°), so the 120° sector alone produces
     * an average angle error of 60°, which limits torque to cos(60°)=50%
     * and creates large reactive currents (measured: ~5 A free-spin at 10 km/h).
     *
     * Fix: interpolate from the current sector base toward the NEXT sector
     * base, scaled by (elapsed_us / last_sector_dur_us).  The span matches
     * the physical sector width, so the drive angle reaches the next sector
     * base exactly at the Hall transition �?seamless handoff, zero reset bump.
     * Average angle error drops from ~30°+ to near 0°.
     *
     * Forward direction (hall_direction > 0) : pos DECREASING 5�?�?�?�?�?�?
     * next_pos when forward: if pos==1 then 6, else pos-1
     * next_pos when reverse: if pos==6 then 1, else pos+1
     */
    if (s_sector_dur_us > 0U && hall_direction != 0) {
        int next_pos;
        /* hall_direction already incorporates MOTOR_DIRECTION (= s_pending * DIRECTION).
         * Forward (hall_direction > 0): physical pos DECREASING 5->4->3->2->1->6->5
         * Reverse (hall_direction < 0): pos INCREASING 5->6->1->2->3->4->5        */
        if (hall_direction > 0) {
            next_pos = (pos == 1) ? 6 : (pos - 1);
        } else {
            next_pos = (pos == 6) ? 1 : (pos + 1);
        }

          float next_base = k_sector_base[next_pos] + foc_hall_offset_deg;

          /* Shortest arc to the next base. FOC_WrapDiffDeg handles the sign naturally! */
          float span = FOC_WrapDiffDeg(next_base, base);
          uint32_t elapsed_us = DWT_Micros() - s_trans_us;

          /* If no Hall edge in 500 ms the motor is stopped.
           * Do NOT extrapolate to the next-sector boundary (frac=1) -- that
           * puts the Q-axis 60°+90°=150° ahead of the actual rotor, which
           * produces cos(150°)=-0.87 braking torque and causes cold-start stall.
           * Use frac=0 (sector base) instead: worst-case angle error is 30° at
           * the sector midpoint, giving cos(30°)=87% torque efficiency.        */
          float frac;
          if (elapsed_us > 500000U) {
              frac = 0.0f;   /* motor stopped: stay at sector base */
          } else {
              frac = (float)elapsed_us / (float)s_sector_dur_us;
              if (frac > 1.0f) { frac = 1.0f; }
          }

        base += span * frac;
    }

    while (base >= 360.0f) { base -= 360.0f; }
    while (base <    0.0f) { base += 360.0f; }
    return base;
}

/* ================================================================
 * PID controller
 * ================================================================ */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float ilim, float olim)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = ilim;
    pid->output_limit   = olim;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Compute(PID_t *pid, float setpoint, float feedback, float dt)
{
    float error = setpoint - feedback;

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) { pid->integral =  pid->integral_limit; }
    if (pid->integral < -pid->integral_limit) { pid->integral = -pid->integral_limit; }

    float derivative = (dt > 1e-6f) ? ((error - pid->prev_error) / dt) : 0.0f;
    pid->prev_error = error;

    float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    /* Anti-windup: if the output is already saturated AND the error is still
     * pushing it deeper into saturation (same sign), undo this step's
     * integral contribution.  The integral is only allowed to accumulate
     * when it helps REDUCE saturation (i.e. when it is opposing the error).
     * This prevents the classic wind-up scenario where the integrator charges
     * to its limit during an overshoot and then takes many seconds to drain. */
    if ((out > pid->output_limit  && error > 0.0f) ||
        (out < -pid->output_limit && error < 0.0f)) {
        pid->integral -= error * dt;   /* back out the step just added */
    }

    if (out >  pid->output_limit) { out =  pid->output_limit; }
    if (out < -pid->output_limit) { out = -pid->output_limit; }
    return out;
}

/* ================================================================
 * FOC state
 * ================================================================ */
FOC_State_t foc = {CTRL_MODE_BLDC};

void FOC_Init(void)
{
    DWT_Enable();
    s_trans_us      = DWT_Micros();
    s_sector_dur_us = 1000000U;
    s_pos_accum_deg = 0.0f;

    memset(&foc, 0, sizeof(foc));
    foc.mode             = CTRL_MODE_BLDC;
    foc.current_limit    = 0.8f;
    foc.startup_vq_limit = 0.0f;

    PID_Init(&foc.pid_speed,
             FOC_SPEED_PID_KP, FOC_SPEED_PID_KI, FOC_SPEED_PID_KD,
             FOC_SPEED_PID_ILIM, FOC_SPEED_PID_OLIM);
    PID_Init(&foc.pid_position,
             FOC_POS_PID_KP, FOC_POS_PID_KI, FOC_POS_PID_KD,
             FOC_POS_PID_ILIM, FOC_POS_PID_OLIM);
}

void FOC_SetMode(FOC_ControlMode_t mode)
{
    if (mode != foc.mode) {
        /* Safe transition: kill all gate outputs first, then wait 1 ms
         * (= 30 PWM cycles @ 30 kHz) to ensure all gate charge drains
         * before changing which bridges are enabled.                    */
        FOC_Stop();
        HAL_Delay(1U);
        PID_Reset(&foc.pid_speed);
        PID_Reset(&foc.pid_position);
        foc.vq              = 0.0f;
        foc.startup_vq_limit = 0.0f;  /* re-ramp from zero on every mode change */
        s_spd_fb_filt        = 0.0f;  /* reset EMI rate-limiter                 */
    }
    foc.mode = mode;
}

void FOC_SetSpeedRef(float speed_kmh)    { foc.speed_ref    = speed_kmh; }
void FOC_SetTorqueRef(float torque_norm) { foc.torque_ref   = torque_norm; }
void FOC_SetPositionRef(float pos_deg)   { foc.position_ref = pos_deg; }

/* ================================================================
 * Sinusoidal three-phase output
 *
 * Maps [-1, 1] normalised voltage -> TIM3 CCR in [0, MOTOR_ARR].
 * Phase channels: CCR2 = U(PA7), CCR3 = V(PB0), CCR4 = W(PB1)
 *
 * Drive angle convention (caller side): angle_rad increases as the
 * motor rotates forward (Hall new-decode: pos 1�?�?...6 = forward).
 * Internally eff_angle = 2π �?angle_rad so the UVW sinusoidal field
 * sweeps in the direction that creates FORWARD torque (this motor's
 * winding sequence produces a reverse-rotating field for positive
 * eff_angle �?see inner comment for empirical evidence).
 *
 * All three low-side SD pins are driven HIGH so every half-bridge
 * is active. The gate driver's built-in deadtime prevents cross-
 * conduction (verified requirement: DRV8302 / IR2104 / equivalent).
 * ================================================================ */
static void FOC_ApplySinusoidal(float vq_norm, float angle_rad)
{
    /* Clamp */
    if (vq_norm >  1.0f) { vq_norm =  1.0f; }
    if (vq_norm < -1.0f) { vq_norm = -1.0f; }

    float eff_angle = angle_rad;  /* angle offset handled by caller (FOC_Run adds +pi/2 for Q-axis) */
    /* U=CH2(PA7) V=CH3(PB0) W=CH4(PB1) */
    float va = sinf(eff_angle)                               * vq_norm;
    float vb = sinf(eff_angle - (float)(2.0 * M_PI / 3.0))  * vq_norm;
    float vc = sinf(eff_angle + (float)(2.0 * M_PI / 3.0))  * vq_norm;

    /* Bias to unsigned range then scale to timer ARR */
    uint16_t ccr_a = (uint16_t)((0.5f + 0.5f * va) * (float)MOTOR_ARR);
    uint16_t ccr_b = (uint16_t)((0.5f + 0.5f * vb) * (float)MOTOR_ARR);
    uint16_t ccr_c = (uint16_t)((0.5f + 0.5f * vc) * (float)MOTOR_ARR);

    if (ccr_a > (uint16_t)MOTOR_ARR) { ccr_a = (uint16_t)MOTOR_ARR; }
    if (ccr_b > (uint16_t)MOTOR_ARR) { ccr_b = (uint16_t)MOTOR_ARR; }
    if (ccr_c > (uint16_t)MOTOR_ARR) { ccr_c = (uint16_t)MOTOR_ARR; }

    /* For BLDC_Motor original mapping:
     * UH = TIM3_CH2
     * VH = TIM3_CH3
     * WH = TIM3_CH4
     * UL = PA6
     * VL = PB7
     * WL = PB8
     */
    TIM3->CCR2 = ccr_a;
    TIM3->CCR3 = ccr_b;
    TIM3->CCR4 = ccr_c;

    /* Low-side active HIGH for enabling SD. We enable all 3 for 3-phase SPWM */
    HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);
}

static void FOC_Stop(void)
{
    TIM3->CCR2 = 0U;
    TIM3->CCR3 = 0U;
    TIM3->CCR4 = 0U;
    HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_RESET);
}

/* ================================================================
 * FOC main run �?call once per main-loop iteration (�? ms)
 * CTRL_MODE_BLDC is handled by Motor_Control() in app_task; skip here.
 * ================================================================ */
void FOC_Run(void)
{
    if (foc.mode == CTRL_MODE_BLDC) { return; }

    /* Update angle and position */
    foc.angle_deg      = Hall_ComputeAngleDeg();
    foc.position_accum = s_pos_accum_deg;

    /* NOTE: Phase advance compensation intentionally removed.
     *
     * REASON 1 (loop_lag): The previous constant (0.001 s = 1 ms) assumed a
     * 1 ms control-loop delay, but Hall_ComputeAngleDeg() uses DWT for
     * microsecond-accurate real-time interpolation.  The actual PWM latency
     * is one PWM cycle (~33 µs).  At 7 km/h that gave 31° of spurious advance,
     * causing the effective current vector to overshoot the Q-axis by ~30°,
     * producing ~2 A of reactive D-axis current at no load.
     *
     * REASON 2 (ind_lag): MOTOR_PHASE_R and MOTOR_PHASE_L in app_config.h are
     * explicitly marked as PLACEHOLDER values.  Feeding unverified numbers into
     * arctan(wL/R) produced another ~28° of incorrect advance at 7 km/h.
     *
     * DWT interpolation + the +π/2 Q-axis offset in FOC_Run output is
     * sufficient for clean operation up to the target speed range.
     * Re-add ind_lag ONLY after measuring R and L on this specific motor.    */
    while (foc.angle_deg >= 360.0f) { foc.angle_deg -= 360.0f; }
    while (foc.angle_deg <  0.0f)   { foc.angle_deg += 360.0f; }
    foc.angle_rad = foc.angle_deg * ((float)M_PI / 180.0f);

    /* Compute dt in seconds */
    static uint32_t last_ms = 0U;
    uint32_t now_ms = APP_time;
    float dt = (float)(now_ms - last_ms) * 0.001f;
    if (dt < 0.0001f || dt > 0.5f) { dt = 0.001f; }
    last_ms = now_ms;

      /* Soft-start ramp: startup_vq_limit climbs from 0 to current_limit at
       * 0.002 per ms call �?reaches 0.8 in ~400 ms, preventing cold-start
       * current spike when the rotor is stationary and angle is uncertain. */
      foc.startup_vq_limit += 0.002f;
    if (foc.startup_vq_limit > foc.current_limit) {
        foc.startup_vq_limit = foc.current_limit;
    }

    float vq = 0.0f;

    /* hall_speed sign convention: positive = forward.
     * hall_direction (hall.c) = s_pending × DIRECTION, so DIRECTION is
     * already baked into hall_speed.  Do NOT multiply by DIRECTION again.
     *
     * Three-stage EMI filter:
     *   1. Plausibility clamp: any reading above FOC_MAX_PLAUSIBLE_KMH
     *      (physically impossible for a skateboard) is hard-limited.
     *   2. Rate limiter: even a clamped 40 km/h artefact cannot change the
     *      PID feedback faster than FOC_SPEED_RATE_LIMIT_KMHS (25 km/h/s).
     *      A real motor cannot change speed faster than this; the limiter
     *      therefore passes genuine dynamics but attenuates EMI spikes.
     *   3. The 500 us Hall debounce (FOC_NotifyHallTransition) ensures that
     *      false Hall edges from inrush current do not inflate hall_speed in
     *      the first place.
     *
     * Without the rate limiter the bus voltage dropped to 12 V because:
     *   EMI inflated hall_speed to 8-54 km/h  (real speed ~0.5 km/h)
     *   -> PID applied vq = -0.800 (max braking)
     *   -> braking current induced more EMI
     *   -> positive feedback loop collapsed the supply.                     */
    const float spd_raw_fb  = hall_speed;   /* DIRECTION already in hall_direction */
    const float spd_clamped = (fabsf(spd_raw_fb) > FOC_MAX_PLAUSIBLE_KMH)
                              ? (spd_raw_fb > 0.0f ?  FOC_MAX_PLAUSIBLE_KMH
                                                    : -FOC_MAX_PLAUSIBLE_KMH)
                              : spd_raw_fb;
    {
        const float max_delta = FOC_SPEED_RATE_LIMIT_KMHS * dt;
        if      ((spd_clamped - s_spd_fb_filt) >  max_delta) { s_spd_fb_filt += max_delta; }
        else if ((spd_clamped - s_spd_fb_filt) < -max_delta) { s_spd_fb_filt -= max_delta; }
        else                                                   { s_spd_fb_filt  = spd_clamped; }
    }
    const float spd_fb = s_spd_fb_filt;
    foc.spd_fb_filt    = spd_fb;        /* expose for diagnostics */
    const float pos_fb  = foc.position_accum;  /* DIRECTION already in hall_direction */

    switch (foc.mode) {

    case CTRL_MODE_TORQUE:
        vq = foc.torque_ref;
        break;

    case CTRL_MODE_SPEED:
        vq = PID_Compute(&foc.pid_speed, foc.speed_ref, spd_fb, dt);
        break;

    case CTRL_MODE_POSITION: {
        float spd_cmd = PID_Compute(&foc.pid_position,
                                    foc.position_ref,
                                    pos_fb, dt);
        vq = PID_Compute(&foc.pid_speed, spd_cmd, spd_fb, dt);
        break;
    }

    default:
        break;
    }

    /* Apply soft-start current limit (ramps up from 0 on mode change) */
    if (vq >  foc.startup_vq_limit) { vq =  foc.startup_vq_limit; }
    if (vq < -foc.startup_vq_limit) { vq = -foc.startup_vq_limit; }

    if (vq == 0.0f) {
        foc.vq = 0.0f;
        foc.ol_active = 0U;
        foc.startup_angle_err_deg = 0.0f;
        FOC_Stop();
        return;
    }

    float drive_vq = vq;
    float drive_angle_rad;

#if FOC_USE_OPEN_LOOP
    /* ---- Open-loop sweep (Hall diagnostic / cogging workaround) ----
     * Sweeps the angle at a rate proportional to drive_vq.
     * Set FOC_USE_OPEN_LOOP 0 in app_config.h for normal closed-loop operation. */
    {
        static float ol_angle_deg = 0.0f;
        float step = fabsf(drive_vq) * FOC_OL_DEG_PER_MS * (dt * 1000.0f);
        if (drive_vq < 0.0f) { step = -step; }
        ol_angle_deg += step;
        if (ol_angle_deg >= 360.0f) { ol_angle_deg -= 360.0f; }
        if (ol_angle_deg <    0.0f) { ol_angle_deg += 360.0f; }
        drive_angle_rad = ol_angle_deg * ((float)M_PI / 180.0f);
        foc.ol_active = 1U;
    }
#else
    /* ---- Closed-loop: Hall angle drives everything, PID owns vq ---- */
    drive_angle_rad = foc.angle_rad;
    foc.ol_active   = 0U;
#endif

    foc.vq = drive_vq;
    float q_axis_angle = drive_angle_rad + (float)(M_PI / 2.0f);
    FOC_ApplySinusoidal(drive_vq, q_axis_angle);
}
#if 0/* ================================================================
 * FOC_ForceAngle  (internal helper for calibration)
 *
 * Applies a static magnetic field at angle_deg for hold_ms milliseconds.
 * angle_deg is in "drive angle" space �?the same convention used by
 * FOC_Run (i.e. directly from the Hall LUT, passed unchanged to
 * FOC_ApplySinusoidal which handles the DIRECTION flip internally).      */
static void FOC_ForceAngle(float angle_deg, float vq_norm, uint32_t hold_ms)
{
    if (vq_norm >  0.80f) { vq_norm =  0.80f; }
    if (vq_norm < -0.80f) { vq_norm = -0.80f; }

    HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);

    /* Pass angle directly �?identical to how FOC_Run feeds foc.angle_deg.
     * FOC_ApplySinusoidal takes care of the DIRECTION flip.               */
    float drive_rad = angle_deg * (float)(M_PI / 180.0);

    for (uint32_t t = 0; t < hold_ms; t++) {
        FOC_ApplySinusoidal(vq_norm, drive_rad);
        HAL_Delay(1U);
    }
}

/* ================================================================
 * FOC_SweepToAngle  (internal helper for calibration)
 *
 * Slowly advances the drive angle from from_deg toward to_deg in 1-degree
 * steps, dragging the rotor along synchronously in the FORWARD direction.
 *
 * "Drive angle" is the same convention as FOC_Run: the angle from the Hall
 * LUT, passed directly to FOC_ApplySinusoidal which handles the DIRECTION
 * flip internally.
 *
 * DIRECTION=-1: FOC_ApplySinusoidal maps drive_angle �?eff_angle = 2π-drive.
 *   Increasing drive_angle �?decreasing eff_angle �?forward motion. �?
 * DIRECTION=+1: eff_angle = drive_angle �?increasing drive �?forward. �?
 *
 * WHY NO PRE-INVERSION:
 *   The previous version pre-inverted for DIRECTION=-1, which double-inverted
 *   with FOC_ApplySinusoidal's own flip, netting eff_angle = drive_angle
 *   (increasing) = REVERSE for DIRECTION=-1.  Evidence: pre-align moved
 *   Hall pos 3�? (decreasing = reverse), confirming the field was going
 *   backward and pushing the rotor into cogging notches instead of through.
 *
 * step_ms  : milliseconds per electrical degree.  20 ms/deg = 50 el-deg/s.
 *   For a 7 pole-pair motor: 50/7/360 �?1.2 RPM mech (very slow).        */
static void FOC_SweepToAngle(float from_deg, float to_deg,
                              float vq_norm, uint32_t step_ms)
{
    if (vq_norm >  0.80f) { vq_norm =  0.80f; }
    if (vq_norm < -0.80f) { vq_norm = -0.80f; }

    /* Angular distance in drive-angle space (forward = increasing, wrap 360) */
    float range = to_deg - from_deg;
    while (range <   0.0f) { range += 360.0f; }
    if    (range > 360.0f) { range  = 360.0f; }
    int steps = (int)range;
    if (steps == 0) { return; }

    HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);

    float angle = from_deg;
    for (int s = 0; s < steps; s++) {
        /* Pass directly �?FOC_ApplySinusoidal handles DIRECTION flip */
        FOC_ApplySinusoidal(vq_norm, angle * (float)(M_PI / 180.0));
        HAL_Delay(step_ms);
        angle += 1.0f;
        if (angle >= 360.0f) { angle -= 360.0f; }
    }
}
#endif

/* ================================================================
 * FOC_CalibrateHallOffset  �? Hall auto-mapping
 *
 * 开环扫3圈电气角，连续采样每个Hall pos对应的drive_angle�?
 * 用圆形均值求每个pos的中心drive_angle，计算与名义LUT的偏差，
 * �?个pos的圆形均值得到全局offset�?
 *
 * 不需要转�?跟随"——只需要电机能转动（VQ够），数据自然积累�?
 * 总时�? 3×360步�?0ms = 21.6秒�?
 * ================================================================ */
float FOC_CalibrateHallOffset(void)
{
    const float    VQ_CAL  = FOC_CAL_VQ;
    const uint32_t STEP_MS = FOC_CAL_STEP_MS;
    const int      N_STEPS = (int)FOC_POLE_PAIRS * FOC_CAL_MECH_REVS * 360;

    float sin_sum[7] = {0.0f};
    float cos_sum[7] = {0.0f};
    int   cnt[7]     = {0};

    Serial_Printf("[CAL] Auto-map: open-loop sweep x%d mech revs (~%lu s, wheel must be free)\r\n",
                  FOC_CAL_MECH_REVS,
                  (uint32_t)((uint32_t)FOC_POLE_PAIRS * FOC_CAL_MECH_REVS * 360u * FOC_CAL_STEP_MS / 1000u));

    HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);

    const int STALL_STEPS = 200;
    int stall_counter = 0;
    int prev_stall    = 0;
    int last_hall_pos = -1;
    int last_print_pos = -1;

    float angle = 0.0f;
    for (int s = 0; s < N_STEPS; s++) {
        // NOTE: The hardware is physically reversed if MOTOR_DIRECTION is -1.
        // During calibration we MUST sweep in the direction that produces physical positive Hall movement.
        // For MOTOR_DIRECTION == -1, a negative drive angle sweeps forward.
        float drive_angle = (MOTOR_DIRECTION == 1) ? angle : (360.0f - angle);
        
        FOC_ApplySinusoidal(VQ_CAL, drive_angle * ((float)M_PI / 180.0f));
        HAL_Delay(STEP_MS);

        Hall_Position();
        int pos = hall_sensor.position;

        if (pos != last_print_pos && pos >= 1 && pos <= 6) {
            Serial_Printf("[CAL_RT] step=%4d drive=%6.1f pos=%d lut=%.0f stall=%d prev=%d %s\r\n",
                          s, drive_angle, pos, k_sector_base[pos], stall_counter, prev_stall, 
                          (prev_stall >= 40) ? "SKIP" : "OK");
            last_print_pos = pos;
        }

        if (pos == last_hall_pos) {
            stall_counter++;
            if (stall_counter >= STALL_STEPS) {
                FOC_Stop();
                Serial_Printf("[CAL] ABORTED: motor stalled at pos=%d drive=%.1f\r\n", pos, drive_angle);
                return 0.0f;
            }
        } else {
            prev_stall    = stall_counter;
            stall_counter = 0;
            last_hall_pos = pos;
        }

        if (pos >= 1 && pos <= 6) {
            if (prev_stall < 40) {
                float a_rad  = drive_angle * ((float)M_PI / 180.0f);
                sin_sum[pos] += sinf(a_rad);
                cos_sum[pos] += cosf(a_rad);
                cnt[pos]++;
            }
        }

        angle += 1.0f;
        if (angle >= 360.0f) { angle -= 360.0f; }
    }

    FOC_Stop();

    float off_sin = 0.0f, off_cos = 0.0f;
    int   valid   = 0;

    Serial_Printf("[CAL] pos  nominal  measured   diff   cnt\r\n");
    for (int i = 1; i <= 6; i++) {
        if (cnt[i] < FOC_CAL_MIN_SAMPLES) {
            Serial_Printf("[CAL]  %d    %.0f      (skipped, cnt=%d < min=%d)\r\n",
                          i, k_sector_base[i], cnt[i], FOC_CAL_MIN_SAMPLES);
            continue;
        }
        float mean_deg = atan2f(sin_sum[i] / (float)cnt[i],
                                cos_sum[i] / (float)cnt[i])
                         * (180.0f / (float)M_PI);
        if (mean_deg < 0.0f) { mean_deg += 360.0f; }

        float diff = mean_deg - k_sector_base[i];
        while (diff >  180.0f) { diff -= 360.0f; }
        while (diff < -180.0f) { diff += 360.0f; }

        Serial_Printf("[CAL]  %d    %.0f      %.1f      %+.1f   %d\r\n",
                      i, k_sector_base[i], mean_deg, diff, cnt[i]);

        float diff_rad = diff * ((float)M_PI / 180.0f);
        off_sin += sinf(diff_rad);
        off_cos += cosf(diff_rad);
        valid++;
    }

    if (valid < 2) {
        Serial_Printf("[CAL] FAILED: only %d/6 positions seen\r\n", valid);
        return 0.0f;
    }

    float offset_raw = atan2f(off_sin, off_cos) * (180.0f / (float)M_PI);
    float offset     = offset_raw;
    foc_hall_offset_deg = offset;

    if (valid < 6) {
        Serial_Printf("[CAL] WARNING: only %d/6 positions mapped (motor cogging).\r\n"
                      "[CAL]   Offset still usable if symmetrical.\r\n", valid);
    }
    Serial_Printf("[CAL] PASS: Hall offset=%.1f deg (raw=%.1f, %d/6 pos)\r\n",
                  offset, offset_raw, valid);

    return offset;
}

/* ================================================================
 * FOC_HallDiagnostic
 *
 * Diagnostic tool: polls Hall A/B/C every 10 ms for duration_ms.
 * Prints the raw bit pattern and decoded position whenever any bit
 * changes.  At the end prints a histogram of how many times each of
 * the 6 valid positions was seen.
 *
 * Use this to verify Hall wiring BEFORE running calibration:
 *   1. Call FOC_HallDiagnostic(10000) at boot (10 s window).
 *   2. Slowly rotate the wheel by hand one full mechanical turn.
 *   3. All 6 positions (1-6) should appear in the histogram.
 *      If fewer than 6 appear �?Hall wiring / sensor fault.
 *      If INVALID states appear �?Hall sensor stuck or open-circuit.
 * ================================================================ */
void FOC_HallDiagnostic(uint32_t duration_ms)
{
    /* Position histogram: index 0 = invalid, 1-6 = valid */
    uint32_t cnt[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
    int prev_a = -1, prev_b = -1, prev_c = -1;

    Serial_Printf("[HALL_DIAG] Rotate wheel slowly for %lu ms...\r\n"
                  "[HALL_DIAG] Expected: all 6 positions appear.\r\n",
                  duration_ms);

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < duration_ms)
    {
        Hall_Position();   /* reads GPIO and decodes */
        int a   = hall_sensor.HallA;
        int b   = hall_sensor.HallB;
        int c   = hall_sensor.HallC;
        int pos = hall_sensor.position;

        /* Accumulate histogram every sample */
        if (pos >= 1 && pos <= 6) { cnt[pos]++; } else { cnt[0]++; }

        /* Print only on pin change */
        if (a != prev_a || b != prev_b || c != prev_c)
        {
            if (pos >= 1 && pos <= 6) {
                Serial_Printf("[HALL] A=%d B=%d C=%d  pos=%d  lut=%.0f deg\r\n",
                              a, b, c, pos, k_sector_base[pos]);
            } else {
                Serial_Printf("[HALL] A=%d B=%d C=%d  pos=INVALID (stuck/open?)\r\n",
                              a, b, c);
            }
            prev_a = a; prev_b = b; prev_c = c;
        }
        HAL_Delay(10U);
    }

    /* Summary histogram */
    Serial_Printf("[HALL_DIAG] --- Histogram (count per position) ---\r\n");
    for (int i = 1; i <= 6; i++) {
        Serial_Printf("  pos=%d  lut=%.0f deg  : %lu samples  %s\r\n",
                      i, k_sector_base[i], cnt[i],
                      (cnt[i] > 0U) ? "OK" : "NEVER SEEN!");
    }
    if (cnt[0] > 0U) {
        Serial_Printf("  INVALID state         : %lu samples  <-- wiring fault!\r\n",
                      cnt[0]);
    }
    int seen = 0;
    for (int i = 1; i <= 6; i++) { if (cnt[i] > 0U) seen++; }
    Serial_Printf("[HALL_DIAG] %d/6 positions seen.  %s\r\n",
                  seen,
                  (seen == 6) ? "Hall OK -- proceed to calibration."
                              : "FAULT -- check Hall wiring before calibration!");
}









