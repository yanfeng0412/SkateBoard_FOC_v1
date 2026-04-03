#ifndef __SERIAL_DBG_H
#define __SERIAL_DBG_H

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * USART1 debug serial  (PA9=TX  PA10=RX, 115200 8N1)
 * DMA circular RX with IDLE-line interrupt; DMA normal TX with queue
 * Frame format: @<payload>\r\n
 * ----------------------------------------------------------------------- */
#define SERIAL_DBG_TX_BUF_SIZE   200
#define SERIAL_DBG_RX_BUF_SIZE   200
#define SERIAL_DBG_TX_QUEUE_SIZE  10

extern char    serial_dbg_rx_packet[SERIAL_DBG_RX_BUF_SIZE];
extern uint8_t serial_dbg_rx_flag;          /* set by IDLE ISR; clear after use */

void Serial_Init(void);
void Serial_IDLE_Handler(void);             /* called from USART1_IRQHandler */
void Serial_DBG_TxCplt_Handler(void);       /* called from HAL_UART_TxCpltCallback */

void Serial_SendByte(uint8_t b);
void Serial_SendArray(const uint8_t *arr, uint16_t len);
void Serial_SendString(const char *str);
void Serial_Printf(const char *fmt, ...);

/* Queue helpers (used inside serial_dbg.c and the HAL TX callback) */
uint8_t Serial_TxQueue_IsEmpty(void);
uint8_t Serial_TxQueue_IsFull(void);

/* Redirect printf → USART1 */
int fputc(int ch, FILE *f);

#endif /* __SERIAL_DBG_H */
