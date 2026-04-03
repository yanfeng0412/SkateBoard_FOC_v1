#ifndef __HALL_H
#define __HALL_H

#include "stm32f1xx_hal.h"
#include "app_config.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Hall sensor GPIO  (PB12 = A  PB13 = B  PB14 = C, pull-up, EXTI both-edge)
 * ----------------------------------------------------------------------- */
#define HALL_A_PIN  GPIO_PIN_12
#define HALL_B_PIN  GPIO_PIN_13
#define HALL_C_PIN  GPIO_PIN_14
#define HALL_PORT   GPIOB

/* Motor direction, pole pairs, wheel diameter, speed filter and sample
 * period are all configured in app_config.h.  Aliases are kept here so
 * the rest of the hall driver code does not need to change. */
#define DIRECTION               MOTOR_DIRECTION
#define HALL_POLE_PAIRS         MOTOR_POLE_PAIRS
#define HALL_TRANSITIONS_PER_REV  (6U * HALL_POLE_PAIRS)   /* = 42 for 7pp */
#define WHEEL_DIAMETER_MM       MOTOR_WHEEL_DIAMETER_MM

/* -----------------------------------------------------------------------
 * Hall sensor decoded state
 * ----------------------------------------------------------------------- */
typedef struct {
    int HallA;
    int HallB;
    int HallC;
    int position;   /* decoded 1–6 */
} HallSensor_t;

extern HallSensor_t   hall_sensor;
extern volatile uint16_t hall_count;     /* pulses in current sample window */
extern float          hall_speed;        /* km/h */
extern float          hall_roll_speed;   /* rev/s */
extern int            hall_direction;    /* +1 forward / -1 reverse / 0 unknown */

void  Hall_Init(void);
void  Hall_GetPosition(void);
void  Hall_Position(void);
float Hall_GetSpeed(void);
void  HallSpeed_Timer(void);  /* call every 100 ms from app_timer */

#endif /* __HALL_H */
