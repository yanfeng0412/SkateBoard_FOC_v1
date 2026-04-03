#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * TIM3 PWM channels for high-side gate drivers
 *   CH2 = PA7 (UH)   CH3 = PB0 (VH)   CH4 = PB1 (WH)
 * ----------------------------------------------------------------------- */
extern TIM_HandleTypeDef htim3;

#define MOTOR_ARR   299U   /* TIM3 ARR: 72 MHz / 8 / 300 = 30 kHz */

/* Low-side SD enable GPIO (active per-step commutation) */
#define MOTOR_UL_PIN   GPIO_PIN_6      /* PA6 */
#define MOTOR_UL_PORT  GPIOA
#define MOTOR_VL_PIN   GPIO_PIN_7      /* PB7 */
#define MOTOR_VL_PORT  GPIOB
#define MOTOR_WL_PIN   GPIO_PIN_8      /* PB8 */
#define MOTOR_WL_PORT  GPIOB

/* -----------------------------------------------------------------------
 * Motor parameters  — adjust to your motor
 * ----------------------------------------------------------------------- */
/* Direction polarity: +1 or -1
 * If the motor rotates backwards, change to +1 in motor.h */
#define MOTOR_DIRECTION     (-1)

/* Back-EMF constant (V·s/rad).  Verify with: Motor_calculate_Ke() */
#define MOTOR_KE            0.130f
#define MOTOR_DELTA_V       0.1f    /* head-room above BEMF (V) */

/* Speed IIR filter.  α=0.005 → reaches Speed=1 in ~6ms from standstill
 * (was 0.0001 which needed 690ms just to produce any drive output). */
#define MOTOR_FILTER_ALPHA  0.005f

/* Braking direction debounce samples */
#define MOTOR_BRAKE_DEBOUNCE 5

/* -----------------------------------------------------------------------
 * Extern state (writable for fault recovery)
 * ----------------------------------------------------------------------- */
extern float    Motor_filter_prev;   /* IIR state — reset to 0 on fault */
extern uint16_t Motor_Speed;         /* current CCR value */
extern uint8_t  isCoasting;          /* 1 while wheel spins above min duty */

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */
void    Motor_Init(void);
void    Motor_Set(uint8_t position);
void    Motor_Control_Init(void);
void    Motor_Direction(int direction);
void    Motor_Control(int inputSpeed);
uint16_t calculate_min_slip_duty(void);

#endif /* __MOTOR_H */
