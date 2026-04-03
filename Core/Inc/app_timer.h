#ifndef __APP_TIMER_H
#define __APP_TIMER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * TIM4-based 1 kHz system tick  (PSC=7199, ARR=9 at 72 MHz → 1000 Hz)
 * ----------------------------------------------------------------------- */

/* Periodic task intervals (ms) */
#define HEARTBEAT_COUNT  150U
#define HALLSPEED_COUNT  100U
#define LED_COUNT        200U
#define NTP_COUNT         50U
#define WATCHDOG_COUNT   700U
#define RUNTIME_COUNT    (60U * 1000U)   /* 1 min */

/* -----------------------------------------------------------------------
 * NTP 4-timestamp struct (matches remote controller protocol)
 * ----------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint32_t t1;   /* request sent */
    uint32_t t2;   /* request received */
    uint32_t t3;   /* response sent */
    uint32_t t4;   /* response received */
} NTPtime_t;

extern volatile uint32_t APP_time;         /* system uptime, ms */
extern volatile uint32_t NTP_time;         /* network-sync time, ms */
extern uint8_t           heartbeat_send_flag;  /* set every 150 ms */

void AppTimer_Init(void);

#endif /* __APP_TIMER_H */
