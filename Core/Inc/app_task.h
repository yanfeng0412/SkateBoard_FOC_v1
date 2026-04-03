#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "stm32f1xx_hal.h"
#include "app_config.h"
#include "rc_cmd.h"
#include "foc.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Shared application state
 * Previously in main.c USER CODE PV — now owned by app_task.c.
 * Other modules that need these values should include app_task.h.
 * ----------------------------------------------------------------------- */
extern int16_t      SetSpeedRC;       /* speed command from RC (-50..50) */
extern uint16_t     LED_debug;        /* TIM2 CH1 debug-LED PWM duty 0..99 */
extern uint32_t     time_controllor;  /* last received controller timestamp */
extern cmd_status_t NTP_status;
extern cmd_status_t watchdog_status;
extern cmd_status_t sys_status;
extern uint8_t      NTP_req;
extern uint8_t      log_write_flag;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/* Call once from main() USER CODE 2, after all MX_xxx_Init() calls */
void App_Task_Init(void);

/* Call once per while(1) iteration from main() USER CODE 3 */
void App_Task_Tick(void);

#endif /* __APP_TASK_H */
