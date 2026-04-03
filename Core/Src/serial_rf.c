#include "serial_rf.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

/* ---------------------------------------------------------------------- */
char     serial_rf_rx_packet[SERIAL_RF_RX_BUF_SIZE];
uint8_t  serial_rf_rx_flag   = 0;
uint16_t serial_rf_rx_length = 0;

static uint8_t  serial_rf_rx_buf[SERIAL_RF_RX_BUF_SIZE];
static uint8_t  serial_rf_tx_buf[SERIAL_RF_TX_BUF_SIZE];

/* TX binary queue (stores raw bytes + length, not strings) */
typedef struct {
    uint8_t  buf[SERIAL_RF_TX_BUF_SIZE];
    uint16_t len;
} RFTxItem_t;

static RFTxItem_t serial_rf_txq[SERIAL_RF_TX_QUEUE_SIZE];
static uint8_t    serial_rf_txq_head = 0;
static uint8_t    serial_rf_txq_tail = 0;

static uint8_t RF_TxQ_IsEmpty(void) { return serial_rf_txq_head == serial_rf_txq_tail; }
static uint8_t RF_TxQ_IsFull(void)
{
    return ((serial_rf_txq_tail + 1) % SERIAL_RF_TX_QUEUE_SIZE) == serial_rf_txq_head;
}

/* ---------------------------------------------------------------------- */
static void Serial_RF_DMA_RX_Start(void)
{
    __HAL_DMA_DISABLE(huart3.hdmarx);

    huart3.hdmarx->Instance->CPAR  = (uint32_t)&huart3.Instance->DR;
    huart3.hdmarx->Instance->CMAR  = (uint32_t)serial_rf_rx_buf;
    huart3.hdmarx->Instance->CNDTR = (uint32_t)SERIAL_RF_RX_BUF_SIZE;

    /* Clear stale flags for DMA1 channel 3 */
    DMA1->IFCR = (DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3 | DMA_IFCR_CGIF3);

    __HAL_DMA_ENABLE(huart3.hdmarx);
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

void Serial_RF_Init(void)
{
    Serial_RF_DMA_RX_Start();
}

/* ---------------------------------------------------------------------- */
void Serial_RF_IDLE_Handler(void)
{
    __HAL_DMA_DISABLE(huart3.hdmarx);

    uint16_t received =
        (uint16_t)SERIAL_RF_RX_BUF_SIZE - (uint16_t)huart3.hdmarx->Instance->CNDTR;

    if (received > 0) {
        memset(serial_rf_rx_packet, 0, SERIAL_RF_RX_BUF_SIZE);
        memcpy(serial_rf_rx_packet, serial_rf_rx_buf, received);
        serial_rf_rx_length = received;
        serial_rf_rx_flag   = 1;
    }

    huart3.hdmarx->Instance->CNDTR = (uint32_t)SERIAL_RF_RX_BUF_SIZE;
    DMA1->IFCR = (DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3 | DMA_IFCR_CGIF3);
    __HAL_DMA_ENABLE(huart3.hdmarx);
}

/* ---------------------------------------------------------------------- */
void Serial_RF_TxCplt_Handler(void)
{
    if (!RF_TxQ_IsEmpty()) {
        RFTxItem_t *item = &serial_rf_txq[serial_rf_txq_head];
        uint16_t    len  = item->len;
        if (len > 0 && len <= SERIAL_RF_TX_BUF_SIZE) {
            memcpy(serial_rf_tx_buf, item->buf, len);
            serial_rf_txq_head =
                (uint8_t)((serial_rf_txq_head + 1) % SERIAL_RF_TX_QUEUE_SIZE);
            HAL_UART_Transmit_DMA(&huart3, serial_rf_tx_buf, len);
        } else {
            serial_rf_txq_head =
                (uint8_t)((serial_rf_txq_head + 1) % SERIAL_RF_TX_QUEUE_SIZE);
        }
    }
}

/* ---------------------------------------------------------------------- */
void Serial_RF_SendByte(uint8_t b)
{
    if (huart3.gState != HAL_UART_STATE_READY) return;
    serial_rf_tx_buf[0] = b;
    HAL_UART_Transmit_DMA(&huart3, serial_rf_tx_buf, 1);
}

void Serial_RF_SendArray(const uint8_t *arr, uint16_t len)
{
    if (len == 0 || len > SERIAL_RF_TX_BUF_SIZE) return;

    if (huart3.gState != HAL_UART_STATE_READY) {
        /* Overflow: drop oldest slot if queue full, then enqueue */
        if (RF_TxQ_IsFull()) {
            serial_rf_txq_head =
                (uint8_t)((serial_rf_txq_head + 1) % SERIAL_RF_TX_QUEUE_SIZE);
        }
        memcpy(serial_rf_txq[serial_rf_txq_tail].buf, arr, len);
        serial_rf_txq[serial_rf_txq_tail].len = len;
        serial_rf_txq_tail =
            (uint8_t)((serial_rf_txq_tail + 1) % SERIAL_RF_TX_QUEUE_SIZE);
        return;
    }

    memcpy(serial_rf_tx_buf, arr, len);
    HAL_UART_Transmit_DMA(&huart3, serial_rf_tx_buf, len);
}

/* ---------------------------------------------------------------------- */
/* Re-initialise USART3 at a different baud rate (used for RF module config) */
void Serial_RF_SetBaudRate(uint32_t baudrate)
{
    /* Stop interrupts and DMA */
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_IDLE);
    __HAL_DMA_DISABLE(huart3.hdmarx);
    CLEAR_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    /* Re-init UART at new baud */
    huart3.Init.BaudRate = baudrate;
    HAL_UART_DeInit(&huart3);
    HAL_UART_Init(&huart3);

    /* Restart circular DMA RX */
    Serial_RF_DMA_RX_Start();
}
