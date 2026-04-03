#ifndef __SERIAL_RF_H
#define __SERIAL_RF_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * USART3 RF module serial  (PB10=TX  PB11=RX, 19200 8N1)
 * DMA circular RX with IDLE-line interrupt; DMA normal TX with binary queue
 * Raw binary packets — no text framing
 * ----------------------------------------------------------------------- */
#define SERIAL_RF_TX_BUF_SIZE   200
#define SERIAL_RF_RX_BUF_SIZE   200
#define SERIAL_RF_TX_QUEUE_SIZE  10

extern char     serial_rf_rx_packet[SERIAL_RF_RX_BUF_SIZE];
extern uint8_t  serial_rf_rx_flag;      /* set by IDLE ISR; clear after use */
extern uint16_t serial_rf_rx_length;    /* byte count of last received packet */

void Serial_RF_Init(void);
void Serial_RF_IDLE_Handler(void);              /* called from USART3_IRQHandler */
void Serial_RF_TxCplt_Handler(void);            /* called from HAL_UART_TxCpltCallback */
void Serial_RF_SetBaudRate(uint32_t baudrate);  /* for RF module config mode */

void Serial_RF_SendByte(uint8_t b);
void Serial_RF_SendArray(const uint8_t *arr, uint16_t len);

#endif /* __SERIAL_RF_H */
