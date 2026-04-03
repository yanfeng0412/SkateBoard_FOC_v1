#include "rc_cmd.h"
#include "app_timer.h"
#include "serial_dbg.h"
#include "serial_rf.h"
#include <string.h>
#include <stdio.h>

/* RF module command buffer */
static uint8_t Cmd_Arr[18];
static char    Cmd_String[40];

/* ---------------------------------------------------------------------- */
/* RF module control GPIO  (JTAG disabled in HAL_MspInit, freeing PB3/4)   *
 *   PB4 = PD (output, active-low power-down)                              *
 *   PB5 = SET (output, low = config mode, high = communication mode)      *
 *   PB3 = AUX (input pull-up)                                             */
void RC_Pin_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pull  = GPIO_NOPULL;
    g.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &g);

    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin  = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  /* PD low = active */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  /* SET low = config mode */
}

/* ---------------------------------------------------------------------- */
void hexStringToByteArray(const char *hex, uint8_t *out, size_t out_size)
{
    size_t len = strlen(hex);
    if ((len / 2) > out_size) return;
    for (size_t i = 0; i < len; i += 2) {
        sscanf(hex + i, "%2hhX", &out[i / 2]);
    }
}

void byteArrayToHexString(const uint8_t *in, size_t size, char *out)
{
    if (out == NULL) return;
    for (size_t i = 0; i < size; i++) {
        sprintf(out + i * 2, "%02X", in[i]);
    }
    out[size * 2] = '\0';
}

/* ---------------------------------------------------------------------- */
/* RF module configuration commands
 *   case 1: default init (AA 5A...)
 *   case 2: set channel 2 at baud transition to 9600 then back to 19200
 *   case 3: set channel 1 at baud transition                              */
void RC_cmd(uint8_t choice)
{
    memset(Cmd_Arr, 0, sizeof(Cmd_Arr));
    memset(Cmd_String, '\0', sizeof(Cmd_String));

    switch (choice) {
    case 1:
        hexStringToByteArray("AA5A22331122000000040064000000120006", Cmd_Arr, sizeof(Cmd_Arr));
        Serial_RF_SendArray(Cmd_Arr, 18);
        break;
    case 2:
        Serial_RF_SetBaudRate(9600);
        HAL_Delay(20);
        hexStringToByteArray("AA5A060814570000000500630002001200F9", Cmd_Arr, sizeof(Cmd_Arr));
        Serial_RF_SendArray(Cmd_Arr, 18);
        HAL_Delay(20);
        Serial_RF_SetBaudRate(19200);
        break;
    case 3:
        Serial_RF_SetBaudRate(9600);
        HAL_Delay(20);
        hexStringToByteArray("AA5A060814570000000500630001001200F8", Cmd_Arr, sizeof(Cmd_Arr));
        Serial_RF_SendArray(Cmd_Arr, 18);
        HAL_Delay(20);
        Serial_RF_SetBaudRate(19200);
        break;
    default:
        break;
    }
}

void RC_setSysInit(void)
{
    RC_Pin_Init();
    /* Switch module to communication mode (SET high) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

/* ---------------------------------------------------------------------- */
uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length)
{
    uint8_t crc  = 0x00U;
    uint8_t poly = 0x31U;  /* CRC-8/MAXIM */
    while (length--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8U; i++) {
            crc = (crc & 0x80U) ? ((crc << 1) ^ poly) : (crc << 1);
        }
    }
    return crc;
}

/* ---------------------------------------------------------------------- */
void RC_send(const uint8_t *data, uint8_t cmd_id, size_t data_size)
{
    const size_t header_size =
        sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint8_t);
    const size_t total_size = header_size + data_size + 2U;  /* +end_flag +crc */

    if (total_size > SERIAL_RF_TX_BUF_SIZE) return;

    uint8_t buf[SERIAL_RF_TX_BUF_SIZE];
    RC_pack_t *pack = (RC_pack_t *)buf;
    pack->start_flag  = 0x5BU;
    pack->pack_number = 0U;
    pack->timestamp   = NTP_time;
    pack->cmd_id      = cmd_id;

    memcpy(buf + header_size, data, data_size);

    size_t end_pos      = header_size + data_size;
    buf[end_pos]        = 0x5DU;
    buf[end_pos + 1U]   = Calculate_CRC8(buf, (uint16_t)(end_pos + 1U));

    Serial_RF_SendArray(buf, (uint16_t)total_size);
}

/* ---------------------------------------------------------------------- */
uint8_t RC_status_pack(uint8_t bat_warn, uint8_t temp_warn,
                       uint8_t temp_err,  uint8_t wdog_err)
{
    if (bat_warn > 1U || temp_warn > 1U || temp_err > 1U || wdog_err > 1U) {
        return 0xFFU;
    }
    return (uint8_t)(bat_warn  << 0) |
           (uint8_t)(temp_warn << 1) |
           (uint8_t)(temp_err  << 2) |
           (uint8_t)(wdog_err  << 3);
}

/* ---------------------------------------------------------------------- */
/* Circular in-RAM event log */
static log_t   Log[MAX_LOG_COUNT];
static uint16_t log_index = 0;
static uint16_t log_count = 0;

void Log_add(uint8_t code, const char *message)
{
    if (message == NULL) { message = ""; }
    Log[log_index].app_time = APP_time;
    Log[log_index].num      = code;
    strncpy(Log[log_index].message, message, (size_t)(MAX_LOG_MESSAGE - 1));
    Log[log_index].message[MAX_LOG_MESSAGE - 1] = '\0';
    log_index = (uint16_t)((log_index + 1U) % MAX_LOG_COUNT);
    if (log_count < MAX_LOG_COUNT) { log_count++; }
}

void Log_print(void)
{
    if (log_count == 0U) {
        Serial_Printf("No logs.\r\n");
        return;
    }
    uint16_t start = (log_count < MAX_LOG_COUNT) ? 0U : log_index;
    for (uint16_t i = 0; i < log_count; i++) {
        uint16_t idx = (uint16_t)((start + i) % MAX_LOG_COUNT);
        Serial_Printf("[%u] T=%u code=0x%02X %s\r\n",
                      i + 1U, Log[idx].app_time, Log[idx].num, Log[idx].message);
        HAL_Delay(10);
    }
}

void Log_reset(void)
{
    Serial_Printf("Log reset. Current log:\r\n");
    Log_print();
    memset(Log, 0, sizeof(Log));
    log_index = 0U;
    log_count = 0U;
}
