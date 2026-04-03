#ifndef __LED_H
#define __LED_H

#include "stm32f1xx_hal.h"

/* -----------------------------------------------------------------------
 * LED GPIO (active-low: GPIO_PIN_SET = off, GPIO_PIN_RESET = on)
 * PA11 = GREEN  PB2 = RED  PC13 = ERROR (on-board)
 * ----------------------------------------------------------------------- */
#define LED_GREEN_PIN   GPIO_PIN_11
#define LED_GREEN_PORT  GPIOA
#define LED_RED_PIN     GPIO_PIN_2
#define LED_RED_PORT    GPIOB
#define LED_ERROR_PIN   GPIO_PIN_13
#define LED_ERROR_PORT  GPIOC

typedef enum { LED_STATE_OFF = 0, LED_STATE_ON = 1 } LED_State_t;

typedef enum {
    LED_ERROR_IDX = 0,
    LED_GREEN_IDX = 1,
    LED_BLUE_IDX  = 2,   /* reserved */
    LED_RED_IDX   = 3
} LED_Choose_t;

typedef enum {
    LED_STATUS_OFF     = 0,
    LED_STATUS_ON      = 1,
    LED_STATUS_FLICKER = 2
} LED_Status_t;

extern LED_Status_t LED_control[4];

void LED_Init(void);
void LED_Set(int led, LED_State_t state);
void LED_Turn(int led);
void LED_Control(LED_Choose_t led, LED_Status_t status);
void LED_Timer(void);  /* call every 200 ms from app_timer */

#endif /* __LED_H */
