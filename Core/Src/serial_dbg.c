#include "serial_dbg.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

/* ---------------------------------------------------------------------- */
char    serial_dbg_rx_packet[SERIAL_DBG_RX_BUF_SIZE];
uint8_t serial_dbg_rx_flag  = 0;

static uint8_t serial_dbg_rx_buf[SERIAL_DBG_RX_BUF_SIZE];
static uint8_t serial_dbg_tx_buf[SERIAL_DBG_TX_BUF_SIZE];

/* TX queue: text strings */
static char    serial_dbg_txq[SERIAL_DBG_TX_QUEUE_SIZE][SERIAL_DBG_TX_BUF_SIZE];
static uint8_t serial_dbg_txq_head = 0;
static uint8_t serial_dbg_txq_tail = 0;

uint8_t Serial_TxQueue_IsEmpty(void) { return serial_dbg_txq_head == serial_dbg_txq_tail; }
uint8_t Serial_TxQueue_IsFull(void)
{
    return ((serial_dbg_txq_tail + 1) % SERIAL_DBG_TX_QUEUE_SIZE) == serial_dbg_txq_head;
}

/* ---------------------------------------------------------------------- */
/* Start circular DMA RX + IDLE interrupt.
 * Called ONCE after MX_USART1_UART_Init() and MX_DMA_Init() have run. */
static void Serial_DBG_DMA_RX_Start(void)
{
    /* Disable channel before reconfiguring */
    __HAL_DMA_DISABLE(huart1.hdmarx);

    /* Point to our buffer */
    huart1.hdmarx->Instance->CPAR  = (uint32_t)&huart1.Instance->DR;
    huart1.hdmarx->Instance->CMAR  = (uint32_t)serial_dbg_rx_buf;
    huart1.hdmarx->Instance->CNDTR = (uint32_t)SERIAL_DBG_RX_BUF_SIZE;

    /* Clear any stale flags on DMA1 channel 5 */
    DMA1->IFCR = (DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5 | DMA_IFCR_CGIF5);

    /* Enable circular DMA and tell the UART to use it for RX */
    __HAL_DMA_ENABLE(huart1.hdmarx);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    /* Enable IDLE line interrupt (fires when RX bus goes idle) */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void Serial_Init(void)
{
    /* USART1 + DMA already configured by CubeMX MX_USART1_UART_Init() */
    Serial_DBG_DMA_RX_Start();
}

/* ---------------------------------------------------------------------- */
/* IDLE interrupt handler — called from USART1_IRQHandler USER CODE */
void Serial_IDLE_Handler(void)
{
    /* Disable DMA to read a stable CNDTR */
    __HAL_DMA_DISABLE(huart1.hdmarx);

    uint16_t received =
        (uint16_t)SERIAL_DBG_RX_BUF_SIZE - (uint16_t)huart1.hdmarx->Instance->CNDTR;

    /* Parse @payload\r\n frame */
    static uint8_t rx_state  = 0;
    static uint8_t pkt_index = 0;
    for (uint16_t i = 0; i < received; i++) {
        uint8_t byte = serial_dbg_rx_buf[i];
        if (rx_state == 0) {
            if (byte == '@' && serial_dbg_rx_flag == 0) {
                rx_state  = 1;
                pkt_index = 0;
            }
        } else if (rx_state == 1) {
            if (byte == '\r') {
                rx_state = 2;
            } else if (pkt_index < (uint8_t)(SERIAL_DBG_RX_BUF_SIZE - 1)) {
                serial_dbg_rx_packet[pkt_index++] = (char)byte;
            }
        } else if (rx_state == 2) {
            if (byte == '\n') {
                rx_state = 0;
                serial_dbg_rx_packet[pkt_index] = '\0';
                serial_dbg_rx_flag = 1;
            }
        }
    }

    /* Reset DMA and restart circular reception */
    huart1.hdmarx->Instance->CNDTR = (uint32_t)SERIAL_DBG_RX_BUF_SIZE;
    DMA1->IFCR = (DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5 | DMA_IFCR_CGIF5);
    __HAL_DMA_ENABLE(huart1.hdmarx);
}

/* ---------------------------------------------------------------------- */
/* TX complete callback — drains the string queue */
void Serial_DBG_TxCplt_Handler(void)
{
    if (!Serial_TxQueue_IsEmpty()) {
        const char *next = serial_dbg_txq[serial_dbg_txq_head];
        uint16_t len = (uint16_t)strlen(next);
        if (len > 0 && len <= SERIAL_DBG_TX_BUF_SIZE) {
            memcpy(serial_dbg_tx_buf, next, len);
            serial_dbg_txq_head =
                (uint8_t)((serial_dbg_txq_head + 1) % SERIAL_DBG_TX_QUEUE_SIZE);
            HAL_UART_Transmit_DMA(&huart1, serial_dbg_tx_buf, len);
        } else {
            serial_dbg_txq_head =
                (uint8_t)((serial_dbg_txq_head + 1) % SERIAL_DBG_TX_QUEUE_SIZE);
        }
    }
}

/* ---------------------------------------------------------------------- */
void Serial_SendByte(uint8_t b)
{
    if (huart1.gState != HAL_UART_STATE_READY) return;
    serial_dbg_tx_buf[0] = b;
    HAL_UART_Transmit_DMA(&huart1, serial_dbg_tx_buf, 1);
}

void Serial_SendArray(const uint8_t *arr, uint16_t len)
{
    if (len == 0 || len > SERIAL_DBG_TX_BUF_SIZE) return;
    if (huart1.gState != HAL_UART_STATE_READY) return;
    memcpy(serial_dbg_tx_buf, arr, len);
    HAL_UART_Transmit_DMA(&huart1, serial_dbg_tx_buf, len);
}

void Serial_SendString(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0 || len > SERIAL_DBG_TX_BUF_SIZE) return;

    if (huart1.gState != HAL_UART_STATE_READY) {
        if (!Serial_TxQueue_IsFull()) {
            strncpy(serial_dbg_txq[serial_dbg_txq_tail], str,
                    SERIAL_DBG_TX_BUF_SIZE - 1);
            serial_dbg_txq[serial_dbg_txq_tail][SERIAL_DBG_TX_BUF_SIZE - 1] = '\0';
            serial_dbg_txq_tail =
                (uint8_t)((serial_dbg_txq_tail + 1) % SERIAL_DBG_TX_QUEUE_SIZE);
        }
        return;
    }
    memcpy(serial_dbg_tx_buf, str, len);
    HAL_UART_Transmit_DMA(&huart1, serial_dbg_tx_buf, len);
}

void Serial_Printf(const char *fmt, ...)
{
    char buf[100];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial_SendString(buf);
}

int fputc(int ch, FILE *f)
{
    (void)f;
    Serial_SendByte((uint8_t)ch);
    return ch;
}

/* ---------------------------------------------------------------------- */
/* HAL UART TX complete callbacks for both USART1 and USART3 */
extern void Serial_RF_TxCplt_Handler(void);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Serial_DBG_TxCplt_Handler();
    } else if (huart->Instance == USART3) {
        Serial_RF_TxCplt_Handler();
    }
}
