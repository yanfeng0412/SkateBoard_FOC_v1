/* app_task.c
 * Application-level logic — moved from main.c USER CODE sections.
 * main.c now only keeps HAL/peripheral init and calls App_Task_Init()
 * + App_Task_Tick().
 */
#include "app_task.h"
#include "motor.h"
#include "hall.h"
#include "led.h"
#include "adc_sensor.h"
#include "serial_dbg.h"
#include "serial_rf.h"
#include "rc_cmd.h"
#include "app_timer.h"
#include "storage.h"
#include "foc.h"
#include <string.h>
#include <stdint.h>

/* HAL handles defined in main.c — needed for TIM2 LED PWM */
extern TIM_HandleTypeDef htim2;

/* -----------------------------------------------------------------------
 * Shared application state (definitions — declared extern in app_task.h)
 * ----------------------------------------------------------------------- */
int16_t      SetSpeedRC      = 0;
uint16_t     LED_debug       = 95;
uint32_t     time_controllor = 0xFFFFFFFFUL;
cmd_status_t NTP_status      = CMD_STATUS_ERROR;
cmd_status_t watchdog_status = CMD_STATUS_ERROR;
cmd_status_t sys_status      = CMD_STATUS_ERROR;
uint8_t      NTP_req         = 0;
uint8_t      log_write_flag  = 0;

/* Temperature ADC thresholds are defined in app_config.h. */

/* ======================================================================
 * Software watchdog
 * ==================================================================== */
static void softWatchDog_timeout(void)
{
#if UART3_LOCAL_TEST_MODE
    watchdog_status = CMD_STATUS_NORMAL;
    return;
#endif
    SetSpeedRC        = 0;
    Motor_filter_prev = 0.0f;
    Motor_Control(0);
    if (foc.mode != CTRL_MODE_BLDC) {
        FOC_SetTorqueRef(0.0f);
        FOC_SetSpeedRef(0.0f);
    }
    NTP_status = CMD_STATUS_ERROR;
}

static void softWatchDog_feed(void)
{
    watchdog_status = CMD_STATUS_NORMAL;
}

static void softWatchDog_loopCheck(void)
{
#if UART3_LOCAL_TEST_MODE
    watchdog_status = CMD_STATUS_NORMAL;
    return;
#endif
    if (watchdog_status == CMD_STATUS_ERROR) {
        softWatchDog_timeout();
        LED_Control(LED_ERROR_IDX, LED_STATUS_ON);
        if (log_write_flag == 0) {
            log_write_flag = 1;
            Log_add(LOG_HEARTBEAT, "Heartbeat loss.");
        }
    } else {
        if (NTP_status != CMD_STATUS_ERROR) {
            LED_Control(LED_ERROR_IDX, LED_STATUS_OFF);
        }
        log_write_flag = 0;
    }
}

/* ======================================================================
 * RC command handlers (previously static in main.c)
 * ==================================================================== */
static void CMD_speed_control(uint8_t *pack)
{
    RC_SpeedControl_t *sp = (RC_SpeedControl_t *)pack;
    SetSpeedRC = (int16_t)((100 - sp->speed_target) / 2);
    /* When a speed command arrives, switch back to BLDC mode if needed */
    if (foc.mode != CTRL_MODE_BLDC) {
        FOC_SetMode(CTRL_MODE_BLDC);
    }
#if UART3_LOCAL_TEST_MODE
    watchdog_status = CMD_STATUS_NORMAL;
#endif
}

static void CMD_speed_report(void)
{
    RC_SpeedControl_t sp = {0};
    sp.speed_current = (int16_t)hall_speed;
    RC_send((uint8_t *)&sp, CMD_SPEED_REPORT, sizeof(RC_SpeedControl_t));
}

static void CMD_time_controllor_report(uint8_t *pack)
{
    uint32_t t = *((uint32_t *)pack);
    time_controllor = t;
    if (((int64_t)NTP_time - (int64_t)t) <  100 &&
        ((int64_t)NTP_time - (int64_t)t) > -50) {
        NTP_status = CMD_STATUS_NORMAL;
    } else {
        NTP_status = CMD_STATUS_ERROR;
    }
    uint8_t ts[4];
    *((uint32_t *)ts) = NTP_time;
    RC_send(ts, CMD_TIME_MOTOR_RPO, sizeof(uint32_t));
}

static void CMD_time_motor_report(uint8_t *pack)
{
    uint16_t t = *((uint16_t *)pack);
    Serial_Printf("CMD_TIME_MOTOR_RPO:%d\r\n", t);
}

static void CMD_temperature_report(void)
{
#if DISABLE_TEMPERATURE_SENSOR
    uint16_t temperature = 250U;
#else
    uint16_t temperature = adc_values[3];
#endif
    uint8_t arr[2];
    *((uint16_t *)arr) = temperature;
    RC_send(arr, CMD_TEMPERATURE_RPO, sizeof(uint16_t));
}

static void CMD_batteryVoltage_report(void)
{
    uint16_t v = adc_values[1];
    uint8_t arr[2];
    *((uint16_t *)arr) = v;
    RC_send(arr, CMD_TEMPERATURE_RPO, sizeof(uint16_t));
}

static void CMD_timeSynchronous(uint8_t *pack)
{
    if (NTP_status != CMD_STATUS_NORMAL) {
        NTPtime_t *np = (NTPtime_t *)pack;
        const int64_t t1 = (int64_t)np->t1;
        const int64_t t2 = (int64_t)np->t2;
        const int64_t t3 = (int64_t)np->t3;
        const int64_t t4 = (int64_t)NTP_time;
        const int32_t delay  = (int32_t)((t4 - t1) - (t3 - t2));
        const int32_t offset = (int32_t)(((t2 - t1) + (t3 - t4)) / 2);
        np->t1 = NTP_time;
        RC_send((uint8_t *)np, CMD_TIME_SYNCHRONOUS, sizeof(NTPtime_t));
        if (delay < 150 && delay > 0) {
            static int64_t tempoffset[3];
            static uint8_t tempcount = 0;
            if ((NTP_time + (uint32_t)offset) > 0) {
                if (tempcount == 0 || offset != tempoffset[tempcount - 1]) {
                    tempoffset[tempcount++] = offset;
                }
            }
            if (tempcount >= 3) {
                int64_t max_val = tempoffset[0], min_val = tempoffset[0];
                for (int i = 1; i < 3; ++i) {
                    if (tempoffset[i] > max_val) { max_val = tempoffset[i]; }
                    if (tempoffset[i] < min_val) { min_val = tempoffset[i]; }
                }
                if ((max_val - min_val) < 50) {
                    if (((tempoffset[2] + NTP_time) >= 0 ||
                         (uint32_t)(-tempoffset[2]) <= NTP_time) &&
                        NTP_status == CMD_STATUS_ERROR) {
                        int64_t adj = (int64_t)NTP_time + tempoffset[2];
                        if ((adj - (int64_t)np->t3) < 100 &&
                            (adj - (int64_t)np->t3) > -50) {
                            __disable_irq();
                            NTP_time = (uint32_t)adj;
                            __enable_irq();
                            NTP_status = CMD_STATUS_NORMAL;
                        }
                    }
                }
                tempcount = 0;
                memset(tempoffset, 0, sizeof(tempoffset));
            }
        }
    }
    softWatchDog_feed();
}

static void CMD_heartbeat(uint8_t *pack)
{
    RC_heartbeat_t *p = (RC_heartbeat_t *)pack;
    SetSpeedRC = -((int16_t)p->speed - 100);
    softWatchDog_feed();
}

/* -----------------------------------------------------------------------
 * FOC control command (new — CMD_FOC_CONTROL 0x40)
 * Packet payload: RC_FocControl_t
 * ----------------------------------------------------------------------- */
static void CMD_foc_control(uint8_t *pack)
{
    RC_FocControl_t *p = (RC_FocControl_t *)pack;
    FOC_ControlMode_t mode = (FOC_ControlMode_t)(p->mode);

    if (p->current_limit > 0.0f && p->current_limit <= 1.0f) {
        foc.current_limit = p->current_limit;
    }

    /* Only reset startup / OL when the control MODE actually changes.
     * Sending a new speed ref in the same mode must NOT restart OL. */
    if (foc.mode != mode) {
        FOC_SetMode(mode);
    }

    switch (mode) {
    case CTRL_MODE_BLDC:     SetSpeedRC = (int)p->ref;   break;
    case CTRL_MODE_TORQUE:   FOC_SetTorqueRef(p->ref);   break;
    case CTRL_MODE_SPEED:    FOC_SetSpeedRef(p->ref);    break;
    case CTRL_MODE_POSITION: FOC_SetPositionRef(p->ref); break;
    default: break;
    }
    softWatchDog_feed();
}

/* -----------------------------------------------------------------------
 * FOC status report (new — CMD_FOC_STATUS 0x41)
 * ----------------------------------------------------------------------- */
static void CMD_foc_status_report(void)
{
    RC_FocStatus_t st;
    st.mode           = (uint8_t)foc.mode;
    st.angle_deg      = foc.angle_deg;
    st.speed_kmh      = hall_speed;
    st.vq             = foc.vq;
    st.position_accum = foc.position_accum;
    RC_send((uint8_t *)&st, CMD_FOC_STATUS, sizeof(RC_FocStatus_t));
}

/* ======================================================================
 * RC packet parser (moved from main.c)
 * ==================================================================== */
static int8_t RC_unpack(const uint8_t *byteArray)
{
    if (byteArray[0] != 0x5BU) { return -1; }
    size_t  packet_size;
    uint8_t end_flag = 0;
    for (packet_size = 1U; packet_size < SERIAL_RF_RX_BUF_SIZE - 2U; ++packet_size) {
        if (byteArray[packet_size] == 0x5DU &&
            byteArray[packet_size + 2U] == 0x00U) {
            packet_size += 2U;
            end_flag = 1;
            break;
        }
    }
    if (!end_flag) { return -1; }

    uint8_t pack[SERIAL_RF_RX_BUF_SIZE];
    memcpy(pack, byteArray, packet_size);
    uint8_t crc8 = pack[packet_size - 1U];
    if (crc8 != Calculate_CRC8(pack, (uint16_t)(packet_size - 1U))) { return -1; }

    typedef union {
        uint8_t   raw[SERIAL_RF_RX_BUF_SIZE];
        RC_pack_t pack;
    } PackConverter;
    PackConverter cv;
    memcpy(cv.raw, pack, packet_size);

#if !UART3_LOCAL_TEST_MODE
    {
        uint32_t t = cv.pack.timestamp;
        /* Accept timestamp=0 as a legacy bench-test packet.
         * Do not force NTP failure on such frames. */
        if (t != 0U) {
            if (((int64_t)NTP_time - (int64_t)t) <  100 &&
                ((int64_t)NTP_time - (int64_t)t) > -100) {
                NTP_status = CMD_STATUS_NORMAL;
            } else {
                NTP_status = CMD_STATUS_ERROR;
            }
        }
    }
#endif

    switch (cv.pack.cmd_id) {
    case CMD_SPEED_CONTROL:        CMD_speed_control(cv.pack.data);           return 0;
    case CMD_SPEED_REPORT:         CMD_speed_report();                        return 0;
    case CMD_SPEED_REPORT_REQ:                                                return 0;
    case CMD_TIME_CONTROLLOR_RPO:  CMD_time_controllor_report(cv.pack.data); return 0;
    case CMD_TIME_MOTOR_RPO:       CMD_time_motor_report(cv.pack.data);      return 0;
    case CMD_TEMPERATURE_RPO:      CMD_temperature_report();                  return 0;
    case CMD_TIME_SYNCHRONOUS:     CMD_timeSynchronous(cv.pack.data);         return 0;
    case CMD_HEARTBEAT:            CMD_heartbeat(cv.pack.data);               return 0;
    case CMD_FOC_CONTROL:          CMD_foc_control(cv.pack.data);             return 0;
    case CMD_FOC_STATUS:           CMD_foc_status_report();                   return 0;
    default:                                                                   return -2;
    }
}

static void Serial_3_Rx_loopCheck(void)
{
    if (serial_rf_rx_flag == 1U) {
        serial_rf_rx_flag = 0U;
        RC_unpack((const uint8_t *)serial_rf_rx_packet);
    }
}

static void Serial_Rx_loopCheck(void)
{
    if (serial_dbg_rx_flag == 1U) {
        serial_dbg_rx_flag = 0U;
        switch (serial_dbg_rx_packet[0]) {
        case '0':
            Serial_Printf("\r\n-- ADC --\r\n");
            Serial_Printf("ADC_curr:%d\r\n",  adc_values[0]);
            Serial_Printf("ADC_bat:%d\r\n",   adc_values[1]);
            Serial_Printf("ADC_NTC:%d\r\n",   adc_values[3]);
            Serial_Printf("-- FOC --\r\n");
            Serial_Printf("mode:%d vq:%.3f angle:%.1f pos:%.1f\r\n",
                          (int)foc.mode, foc.vq, foc.angle_deg, foc.position_accum);
            break;
        case '1':
            Log_print();
            Log_add(LOG_NORMAL, "serial print.");
            break;
        case '2':
            Log_reset();
            Log_add(LOG_WARNING, "Log Reset.");
            break;
        case '3':
            Serial_Printf("Speed:%.2f Dir:%d\r\n", hall_speed, hall_direction);
            break;
        case '4':
            for (uint16_t i = 1U; i < USE_AREA; i++) {
                uint16_t d = Store_Data[i];
                if (i == 5 || i == 7 || i == 9 || i == 16 || i == 17 || i == 19) {
                    Serial_Printf("%d", d);
                } else {
                    Serial_Printf("%c%c", (char)(d >> 8), (char)(d & 0xFFU));
                }
                HAL_Delay(1);
            }
            break;
        case '5':
            Store_Save();
            Serial_Printf("Save OK");
            break;
        case '6':
            Flash_ErasePage(FLASH_STORE_ADDR);
            Serial_Printf("Erase OK");
            break;
        case 'c':
        case 'C':
            /* Manual FOC Hall-offset calibration — wheel must be free. */
            Serial_Printf("[CAL] Starting Hall offset calibration (~4s)...\r\n");
            {
                float off = FOC_CalibrateHallOffset();
                Serial_Printf("[CAL] Done: offset=%.1f deg\r\n", off);
            }
            break;
        case 't':
        case 'T':
            /* Torque direction test: drive at Vq=0.20 for 3 s, then stop.
             * Watch which way the wheel spins.
             * If backward: flip MOTOR_DIRECTION in app_config.h.           */
            Serial_Printf("[TEST] Torque test: Vq=+0.20 for 3 s — watch direction!\r\n");
            FOC_SetMode(CTRL_MODE_TORQUE);
            FOC_SetTorqueRef(0.20f);
            HAL_Delay(3000);
            FOC_SetTorqueRef(0.0f);
            FOC_SetMode(CTRL_MODE_BLDC);   /* cuts gate outputs */
            Serial_Printf("[TEST] Done. spd=%.2f dir=%+d\r\n", hall_speed, hall_direction);
            break;
        default:
            break;
        }
    }
}

static void writeFlash_loopCheck(void)
{
    static uint32_t lastWrite_time   = 0U;
    static uint32_t lastNoSpeed_time = 0U;
    if (APP_time - lastWrite_time > (5UL * 60UL * 1000UL)) {
        lastWrite_time = APP_time;
        if (hall_speed == 0.0f && SetSpeedRC == 0) {
            if (APP_time - lastNoSpeed_time > (20UL * 1000UL)) {
                Store_Save();
            }
        } else {
            lastNoSpeed_time = APP_time;
        }
    }
}

/* ======================================================================
 * Power-on self-check (skipped in UART3_LOCAL_TEST_MODE)
 * SELFCHECK_* and TEMPERATURE_* constants are defined in app_config.h.
 * ==================================================================== */
#if !UART3_LOCAL_TEST_MODE
static uint8_t selfCheck(void)
{
    uint8_t check_count = 0;
    if (SELFCHECK_TEMPERATURE) { check_count++; }
    if (SELFCHECK_BATTERY)     { check_count++; }
    if (SELFCHECK_HALL)        { check_count++; }
    if (SELFCHECK_CURRENT)     { check_count++; }

    for (;;) {
        LED_Control(LED_ERROR_IDX, LED_STATUS_ON);
        LED_Control(LED_RED_IDX,   LED_STATUS_ON);
        Serial_Rx_loopCheck();

        uint8_t success_count = 0;

        if (SELFCHECK_TEMPERATURE) {
            uint8_t ok = 0;
            for (int8_t i = 10; i > 0; i--) {
                uint16_t t = adc_values[3];
                if (t > TEMPERATURE_HIGH && t < TEMPERATURE_ERROR) { ok++; }
                else { ok = 0; }
                if (ok >= 5) { success_count++; break; }
            }
        }

        if (SELFCHECK_BATTERY) {
            uint8_t ok = 0;
            for (int8_t i = 10; i > 0; i--) {
                float bv = ADC_GetBatteryVoltage();
                if (bv > 30.0f && bv < 45.0f) { ok++; }
                else { ok = 0; }
                if (ok >= 5) { success_count++; break; }
            }
        }

        if (SELFCHECK_HALL) {
            int pos = hall_sensor.position;
            if (pos >= 1 && pos <= 6) { success_count++; }
        }

        if (SELFCHECK_CURRENT) {
            uint16_t cur = adc_values[0];
            if (cur != 0U && cur < 5U) { success_count++; }
        }

        if (success_count == check_count) {
            LED_Control(LED_RED_IDX, LED_STATUS_OFF);
            Log_add(LOG_NORMAL, "selfCheck Success.");
            return 0;
        }

        static uint8_t sc_log_flag = 0;
        if (sc_log_flag == 0) {
            sc_log_flag = 1;
            Log_add(LOG_NORMAL, "selfCheck Error.");
        }
    }
}
#endif /* !UART3_LOCAL_TEST_MODE */

/* ======================================================================
 * App_Task_Init
 * ==================================================================== */
void App_Task_Init(void)
{
    AppTimer_Init();
    LED_Init();
    Serial_Init();
    Serial_RF_Init();
    Motor_Control_Init();
    Store_Init();
    ADC_Sensor_Init();
    RC_setSysInit();
    FOC_Init();

    watchdog_status = CMD_STATUS_NORMAL;
    sys_status      = CMD_STATUS_NORMAL;

#if UART3_LOCAL_TEST_MODE
    NTP_status      = CMD_STATUS_NORMAL;
    watchdog_status = CMD_STATUS_NORMAL;
#endif

    /* TIM2 CH1 debug PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_debug);

    Log_add(LOG_NORMAL, "System start.");

#if FOC_HALL_DIAG_ON_BOOT
    Serial_Printf("[BOOT] Hall diagnostic starting (15 s -- rotate wheel slowly)...\r\n");
    FOC_HallDiagnostic(15000U);
#endif

#if FOC_CALIBRATE_ON_BOOT
    Serial_Printf("[BOOT] FOC calibration starting (wheel must be free)...\r\n");
    float calib_offset = FOC_CalibrateHallOffset();
    Serial_Printf("[BOOT] FOC calib done: offset=%.1f deg  (foc_hall_offset_deg=%.1f)\r\n",
                  calib_offset, foc_hall_offset_deg);
    Log_add(LOG_NORMAL, "FOC calib done.");
    /* Wait for motor coasting to stop after CAL open-loop sweep.
     * hall_speed is a 100ms sample — wait until it reads near zero. */
    Serial_Printf("[BOOT] Waiting for motor to coast down...\r\n");
    {
        uint32_t t0 = APP_time;
        uint8_t  stopped_cnt = 0U;
        while (APP_time - t0 < 10000U) {  /* max 10 s */
            HAL_Delay(100U);
            /* Actively flush speed counter — do not rely on TIM4 alignment */
            HallSpeed_Timer();
            if (fabsf(hall_speed) < 0.5f) {
                stopped_cnt++;
                Serial_Printf("[BOOT] coast spd=%.2f (%u/3)\r\n", hall_speed, stopped_cnt);
                if (stopped_cnt >= 3U) { break; }  /* 3 consecutive windows < 0.5 km/h */
            } else {
                stopped_cnt = 0U;
                Serial_Printf("[BOOT] coast spd=%.2f\r\n", hall_speed);
            }
        }
    }
    Serial_Printf("[BOOT] Motor stopped (spd=%.2f). Starting test.\r\n", hall_speed);
#endif

    /* Boot in BLDC mode — send CMD_FOC_CONTROL (0x40) to activate FOC modes */
    Serial_Printf("[BOOT] Ready. BLDC mode.\r\n");

#if 0  /* MOS 静态占空比测试 — 硬件验证用，正常不启用 */
    Serial_Printf("\r\n[TEST] === 静态占空比验尸 ===\r\n");
    Serial_Printf("[TEST] 请将万用表打到【直流电压档 (DC V)】\r\n");
    Serial_Printf("[TEST] 黑表笔接 GND，红表笔依次测驱动板 U、V、W 接口\r\n");
    FOC_SetMode(CTRL_MODE_BLDC);
    /* U=50%  V=60%  W=70% */
    TIM3->CCR2 = (uint16_t)(MOTOR_ARR * 0.50f);
    TIM3->CCR3 = (uint16_t)(MOTOR_ARR * 0.60f);
    TIM3->CCR4 = (uint16_t)(MOTOR_ARR * 0.70f);
    HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);
    Serial_Printf("[TEST] 已强制定死占空比，你有 15 秒钟的时间测量...\r\n");
    HAL_Delay(15000U);
    FOC_SetMode(CTRL_MODE_BLDC);
    Serial_Printf("[TEST] 测试结束，输出已关闭。\r\n");
#endif  /* MOS 静态占空比测试 */

#if !UART3_LOCAL_TEST_MODE
    selfCheck();
#endif
}

/* ======================================================================
 * App_Task_Tick  — one pass of the main control loop (~1 ms period)
 * ==================================================================== */
void App_Task_Tick(void)
{
    /* ---------------------------------------------------------------- */

    /* Debug LED brightness */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_debug);

    softWatchDog_loopCheck();

    /* NTP sync recovery (non-local-test mode only) */
#if !UART3_LOCAL_TEST_MODE
    if (NTP_status == CMD_STATUS_ERROR) {
        Motor_Control(0);
        while (NTP_status == CMD_STATUS_ERROR) {
            softWatchDog_loopCheck();
            LED_Control(LED_ERROR_IDX, LED_STATUS_FLICKER);
            NTP_req = 1U;
            if (NTP_req != 1U) {
                NTPtime_t sp = {0};
                sp.t1 = NTP_time;
                RC_send((uint8_t *)&sp, CMD_TIME_SYNCHRONOUS, sizeof(NTPtime_t));
                NTP_req = 1U;
            }
            writeFlash_loopCheck();
            Serial_Rx_loopCheck();
            if (serial_rf_rx_flag == 1U) {
                serial_rf_rx_flag = 0U;
                RC_unpack((const uint8_t *)serial_rf_rx_packet);
            }
        }
        LED_Control(LED_ERROR_IDX, LED_STATUS_OFF);
        uint8_t ts[4];
        *((uint32_t *)ts) = NTP_time;
        RC_send(ts, CMD_TIME_MOTOR_RPO, sizeof(uint32_t));
        Log_add(LOG_NTP_SUCCESS, "NTP_status success");
    }
#endif

    /* Temperature monitoring */
#if DISABLE_TEMPERATURE_SENSOR
    cmd_status_t sys_status_temperature = CMD_STATUS_NORMAL;
    float        temperature            = 25.0f;
#else
    cmd_status_t sys_status_temperature = CMD_STATUS_ERROR;
    float        temperature            = ADC_GetTemperature();
    if      (temperature <= 80.0f)                        { sys_status_temperature = CMD_STATUS_NORMAL;  }
    else if (temperature > 80.0f && temperature < 90.0f)  { sys_status_temperature = CMD_STATUS_WARNING; }
    else if (temperature >= 90.0f)                        { sys_status_temperature = CMD_STATUS_ERROR;   }
#endif

    /* Battery monitoring */
    cmd_status_t sys_status_battery = CMD_STATUS_ERROR;
    float battery = ADC_GetBatteryVoltage();
    if      (battery < 31.0f && battery > 30.0f)  { sys_status_battery = CMD_STATUS_WARNING; }
    else if (battery <= 30.0f || battery > 45.0f) { sys_status_battery = CMD_STATUS_ERROR;   }
    else                                           { sys_status_battery = CMD_STATUS_NORMAL;  }

    /* Aggregate system status */
    sys_status = CMD_STATUS_NORMAL;
    if      (sys_status_battery     == CMD_STATUS_WARNING) { sys_status = CMD_STATUS_WARNING; }
    else if (sys_status_temperature == CMD_STATUS_WARNING) { sys_status = CMD_STATUS_WARNING; }
    if      (sys_status_temperature == CMD_STATUS_ERROR)   { sys_status = CMD_STATUS_ERROR; }
#if !UART3_LOCAL_TEST_MODE
    else if (watchdog_status        == CMD_STATUS_ERROR)   { sys_status = CMD_STATUS_ERROR; }
#endif
    else if (sys_status_battery     == CMD_STATUS_ERROR)   { sys_status = CMD_STATUS_WARNING; }

    /* Motor drive — BLDC or FOC depending on active mode */
    if (sys_status != CMD_STATUS_ERROR) {
        if (foc.mode == CTRL_MODE_BLDC) {
            Motor_Control(SetSpeedRC);
        } else {
            FOC_Run();
        }
    } else {
        Motor_Control(0);
        if (foc.mode != CTRL_MODE_BLDC) {
            FOC_SetTorqueRef(0.0f);
            FOC_SetSpeedRef(0.0f);
        }
        LED_debug = 100U;
    }

    /* Key input PA12 (expansion placeholder) */
    (void)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);

    /* Heartbeat TX + UART1 FOC diagnostics (every 100 ms) */
    if (heartbeat_send_flag == 1U) {
        heartbeat_send_flag = 0U;
        RC_heartbeat_t hb;
        hb.speed       = hall_speed;
        hb.battery     = battery;
        hb.temperature = temperature;
        uint8_t ib_bat = (sys_status_battery     != CMD_STATUS_NORMAL)  ? 1U : 0U;
        uint8_t ib_tw  = (sys_status_temperature == CMD_STATUS_WARNING) ? 1U : 0U;
        uint8_t ib_te  = (sys_status_temperature == CMD_STATUS_ERROR)   ? 1U : 0U;
        uint8_t ib_wd  = (watchdog_status        == CMD_STATUS_ERROR)   ? 1U : 0U;
        hb.status = RC_status_pack(ib_bat, ib_tw, ib_te, ib_wd);
        RC_send((const uint8_t *)&hb, CMD_HEARTBEAT, sizeof(RC_heartbeat_t));

        /* ---- UART1 FOC diagnostics (printed every 100 ms) ----
         * md  : control mode (0=BLDC 1=TRQ 2=SPD 3=POS)
         * ang : estimated electrical angle (deg)
         * vq  : actual applied normalised output voltage [-1,+1]
         * lim : soft-start ramp limit (0→0.8 in ~400 ms)
         * ref : speed / torque / position setpoint
         * spd : hall_speed (km/h)
         * dir : hall_direction (+1 fwd / -1 rev / 0 unknown)
         * hcnt: Hall pulse count in this 100 ms window
         * isp : speed PID integral (windup check)
         * ol  : 1=open-loop startup, 0=Hall closed-loop
         * aer : Hall angle - OL angle (deg, wrapped)
         */
        if (foc.mode != CTRL_MODE_BLDC) {
            /* spd_raw: hall_speed (km/h), positive = forward
             *          DIRECTION already applied via hall_direction in hall.c
             * spd_nm : same as spd_raw (shown for legacy format compat)
             * spd_fb : rate-limited value actually fed to the PID
             *          (EMI spikes > 25 km/h/s are attenuated here)   */
            const float _snm = hall_speed;
            Serial_Printf(
                "[FOC] md:%d ang:%6.1f vq:%+.3f lim:%.3f ref:%+.2f"
                " spd_raw:%+.2f spd_nm:%+.2f spd_fb:%+.2f"
                " dir:%+d hcnt:%3u isp:%+.4f ol:%d aer:%+5.1f\r\n",
                (int)foc.mode,
                foc.angle_deg,
                foc.vq,
                foc.startup_vq_limit,
                (foc.mode == CTRL_MODE_TORQUE)   ? foc.torque_ref   :
                (foc.mode == CTRL_MODE_POSITION) ? foc.position_ref : foc.speed_ref,
                hall_speed,
                _snm,
                foc.spd_fb_filt,
                hall_direction,
                (unsigned)hall_count,
                foc.pid_speed.integral,
                (int)foc.ol_active,
                foc.startup_angle_err_deg);
        }
    }

    /* UART receive */
    Serial_3_Rx_loopCheck();
    Serial_Rx_loopCheck();

    /* Periodic flash write */
    writeFlash_loopCheck();

    /* Idle telemetry (mileage + runtime every 5 s when stopped) */
    {
        static uint32_t lastNoSpeed_time = 0U;
        static uint32_t lastSend_time    = 0U;
        static uint8_t  sendContent_flag = 0U;
        if (hall_speed == 0.0f && SetSpeedRC == 0) {
            if (APP_time - lastNoSpeed_time > (10UL * 1000UL)) {
                if (APP_time - lastSend_time > (5UL * 1000UL)) {
                    lastSend_time = APP_time;
                    uint8_t sp[6];
                    if (sendContent_flag == 0U) {
                        sendContent_flag = 1U;
                        sp[0] = (uint8_t)(Store_Data[9]  >> 8); sp[1] = (uint8_t)(Store_Data[9]  & 0xFFU);
                        sp[2] = (uint8_t)(Store_Data[7]  >> 8); sp[3] = (uint8_t)(Store_Data[7]  & 0xFFU);
                        sp[4] = (uint8_t)(Store_Data[5]  >> 8); sp[5] = (uint8_t)(Store_Data[5]  & 0xFFU);
                        RC_send(sp, CMD_MILEAGE_RPO, sizeof(sp));
                    } else {
                        sendContent_flag = 0U;
                        sp[0] = (uint8_t)(Store_Data[16] >> 8); sp[1] = (uint8_t)(Store_Data[16] & 0xFFU);
                        sp[2] = (uint8_t)(Store_Data[17] >> 8); sp[3] = (uint8_t)(Store_Data[17] & 0xFFU);
                        sp[4] = (uint8_t)(Store_Data[19] >> 8); sp[5] = (uint8_t)(Store_Data[19] & 0xFFU);
                        RC_send(sp, CMD_RUNTIME_RPO, sizeof(sp));
                    }
                }
            }
        } else {
            lastNoSpeed_time = APP_time;
        }
    }
}
