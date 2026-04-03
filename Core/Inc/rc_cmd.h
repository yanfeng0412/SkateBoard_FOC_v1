#ifndef __RC_CMD_H
#define __RC_CMD_H

#include "stm32f1xx_hal.h"
#include "serial_rf.h"
#include "serial_dbg.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

/* -----------------------------------------------------------------------
 * RC wireless protocol command IDs
 * ----------------------------------------------------------------------- */
#define CMD_SPEED_CONTROL       0x01U
#define CMD_SPEED_REPORT        0x02U
#define CMD_SPEED_REPORT_REQ    0x03U
#define CMD_TIME_CONTROLLOR_RPO 0x11U
#define CMD_TIME_MOTOR_RPO      0x12U
#define CMD_TIME_SYNCHRONOUS    0x13U
#define CMD_TEMPERATURE_RPO     0x20U
#define CMD_MILEAGE_RPO         0x21U
#define CMD_RUNTIME_RPO         0x22U
#define CMD_HEARTBEAT           0x30U

/* FOC control (new) */
#define CMD_FOC_CONTROL         0x40U   /* remote → motor: set mode + reference */
#define CMD_FOC_STATUS          0x41U   /* motor → remote: FOC state report     */

/* -----------------------------------------------------------------------
 * Log error codes
 * ----------------------------------------------------------------------- */
#define LOG_NORMAL       0x00U
#define LOG_WARNING      0x01U
#define LOG_ERROR        0x02U
#define LOG_NTP_ERROR    0x11U
#define LOG_NTP_SUCCESS  0x12U
#define LOG_HEARTBEAT    0x13U
#define MAX_LOG_MESSAGE  100U
#define MAX_LOG_COUNT    100U

/* -----------------------------------------------------------------------
 * System status enum (shared across modules)
 * ----------------------------------------------------------------------- */
typedef enum { CMD_STATUS_NORMAL = 0, CMD_STATUS_ERROR, CMD_STATUS_WARNING } cmd_status_t;

/* -----------------------------------------------------------------------
 * Packet structures
 * ----------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t  start_flag;    /* 0x5B */
    uint32_t pack_number;
    uint32_t timestamp;     /* NTP_time at send */
    uint8_t  cmd_id;
    uint8_t  data[];        /* variable-length payload */
    /* followed by: uint8_t end_flag (0x5D), uint8_t crc8 */
} RC_pack_t;

typedef struct {
    int16_t  speed_target;
    int16_t  speed_current;
    uint32_t milometer;
    uint8_t  milometer_decimal;
    uint32_t time_send;
    uint32_t time_receive;
} RC_SpeedControl_t;

/* FOC command packet payload (cmd_id = CMD_FOC_CONTROL) */
typedef struct __attribute__((packed)) {
    uint8_t mode;           /* FOC_ControlMode_t: 0=BLDC 1=TORQUE 2=SPEED 3=POSITION */
    float   ref;            /* speed(km/h) | torque(0-1) | position(deg)              */
    float   current_limit;  /* max |Vq| normalised 0-1, 0 = keep previous             */
} RC_FocControl_t;

/* FOC status report payload (cmd_id = CMD_FOC_STATUS) */
typedef struct __attribute__((packed)) {
    uint8_t mode;
    float   angle_deg;
    float   speed_kmh;
    float   vq;
    float   position_accum;
} RC_FocStatus_t;

typedef struct {
    float   speed;        /* km/h */
    uint8_t status;       /* packed flags: see RC_status_pack */
    float   battery;      /* V */
    float   temperature;  /* °C */
} RC_heartbeat_t;

typedef struct {
    uint32_t app_time;
    uint8_t  num;
    char     message[MAX_LOG_MESSAGE];
} log_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */
void    RC_Pin_Init(void);
void    RC_setSysInit(void);

uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length);
void    RC_send(const uint8_t *data, uint8_t cmd_id, size_t data_size);
uint8_t RC_status_pack(uint8_t bat_warn, uint8_t temp_warn,
                       uint8_t temp_err, uint8_t wdog_err);

void    RC_cmd(uint8_t choice);   /* RF module configuration commands */

void    hexStringToByteArray(const char *hex, uint8_t *out, size_t out_size);
void    byteArrayToHexString(const uint8_t *in, size_t size, char *out);

void    Log_add(uint8_t code, const char *message);
void    Log_print(void);
void    Log_reset(void);

#endif /* __RC_CMD_H */
