#include "led.h"

LED_Status_t LED_control[4] = { LED_STATUS_OFF };

void LED_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pull  = GPIO_NOPULL;

    g.Pin = LED_GREEN_PIN;  HAL_GPIO_Init(LED_GREEN_PORT, &g);
    g.Pin = LED_RED_PIN;    HAL_GPIO_Init(LED_RED_PORT, &g);
    g.Pin = LED_ERROR_PIN;  HAL_GPIO_Init(LED_ERROR_PORT, &g);

    /* All off (active-low: high = off) */
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_PORT,   LED_RED_PIN,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_SET);
}

void LED_Set(int led, LED_State_t state)
{
    GPIO_PinState pin = (state == LED_STATE_ON) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    if (led == 1) { HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, pin); }
    else if (led == 2) { HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, pin); }
}

void LED_Turn(int led)
{
    if (led == 1) { HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN); }
    else if (led == 2) { HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN); }
}

void LED_Control(LED_Choose_t led, LED_Status_t status)
{
    LED_control[led] = status;
}

/* Called every 200 ms from app_timer (TIM4 ISR callback) */
void LED_Timer(void)
{
    for (uint8_t i = 0; i <= 3; i++) {
        switch (LED_control[i]) {
        case LED_STATUS_OFF:
            switch (i) {
            case LED_GREEN_IDX: HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);   break;
            case LED_RED_IDX:   HAL_GPIO_WritePin(LED_RED_PORT,   LED_RED_PIN,   GPIO_PIN_SET);   break;
            case LED_ERROR_IDX: HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_SET);   break;
            default: break;
            }
            break;
        case LED_STATUS_ON:
            switch (i) {
            case LED_GREEN_IDX: HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET); break;
            case LED_RED_IDX:   HAL_GPIO_WritePin(LED_RED_PORT,   LED_RED_PIN,   GPIO_PIN_RESET); break;
            case LED_ERROR_IDX: HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_RESET); break;
            default: break;
            }
            break;
        case LED_STATUS_FLICKER:
            switch (i) {
            case LED_GREEN_IDX: HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN); break;
            case LED_RED_IDX:   HAL_GPIO_TogglePin(LED_RED_PORT,   LED_RED_PIN);   break;
            case LED_ERROR_IDX: HAL_GPIO_TogglePin(LED_ERROR_PORT, LED_ERROR_PIN); break;
            default: break;
            }
            break;
        }
    }
}
