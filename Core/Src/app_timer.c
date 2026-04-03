#include "app_timer.h"
#include "hall.h"
#include "led.h"
#include "rc_cmd.h"
#include "storage.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

volatile uint32_t APP_time         = 0;
volatile uint32_t NTP_time         = 0;
uint8_t           heartbeat_send_flag = 0;

/* Per-task last-fire timestamps */
static uint32_t heartbeat_time = 0;
static uint32_t hall_speed_time = 0;
static uint32_t led_time        = 0;
static uint32_t ntp_tick_time   = 0;
static uint32_t watchdog_time   = 0;
static uint32_t runtime_time    = 0;

/* Shared state is now owned by app_task.c */
#include "app_task.h"

/* ---------------------------------------------------------------------- */
void AppTimer_Init(void)
{
    /* TIM4 already configured by MX_TIM4_Init().
     * Enable NVIC and start with update interrupt. */
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_TIM_Base_Start_IT(&htim4);
}

/* ---------------------------------------------------------------------- */
/* Override the HAL weak callback — fires every 1 ms from TIM4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) { return; }

    APP_time++;
    NTP_time++;
    if (APP_time > 0xFFFFFFFEUL) { APP_time = 0; }

    /* Heartbeat TX flag (150 ms) */
    if (APP_time - heartbeat_time >= HEARTBEAT_COUNT) {
        heartbeat_time = APP_time;
        heartbeat_send_flag = 1;
    }

    /* Hall speed recalculation (100 ms) */
    if (APP_time - hall_speed_time >= HALLSPEED_COUNT) {
        hall_speed_time = APP_time;
        HallSpeed_Timer();
    }

    /* LED flicker (200 ms) */
    if (APP_time - led_time >= LED_COUNT) {
        led_time = APP_time;
        LED_Timer();
    }

    /* NTP rate limiter */
    if (NTP_req == 1U) {
        if (APP_time - ntp_tick_time >= NTP_COUNT) {
            NTP_req = 0U;
            ntp_tick_time = APP_time;
        }
    }

    /* Software watchdog escalation (700 ms) */
    if (APP_time - watchdog_time >= WATCHDOG_COUNT) {
        watchdog_time = APP_time;
        if (watchdog_status == CMD_STATUS_WARNING) {
            watchdog_status = CMD_STATUS_ERROR;
        } else if (watchdog_status == CMD_STATUS_NORMAL) {
            watchdog_status = CMD_STATUS_WARNING;
        }
        /* CMD_STATUS_ERROR stays error */
    }

    /* Runtime accumulation (1 min) */
    if (APP_time - runtime_time >= RUNTIME_COUNT) {
        runtime_time = APP_time;
        Store_runTime(1);
    }
}
