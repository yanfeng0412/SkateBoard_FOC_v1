#include "motor.h"
#include "hall.h"
#include "adc_sensor.h"

float    Motor_filter_prev = 0.0f;
uint16_t Motor_Speed       = 0U;
uint8_t  isCoasting        = 0U;

/* ---------------------------------------------------------------------- */
/* @brief: Compute minimum PWM duty (CCR) to keep voltage above back-EMF.
 *         Uses KE × ω + safety margin, normalised to MOTOR_ARR range.    */
uint16_t calculate_min_slip_duty(void)
{
    float rpm   = hall_roll_speed * 60.0f;
    float vbus  = ADC_GetBatteryVoltage();
    float omega = rpm * (2.0f * 3.14159265f / 60.0f);
    float bemf  = MOTOR_KE * omega;
    float duty  = (bemf + MOTOR_DELTA_V) / vbus;

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    uint16_t ccr = (uint16_t)(duty * 100.0f + 0.5f);   /* 0..100 */
    if (ccr > 99U) { ccr = 99U; }
    ccr = ccr * ((MOTOR_ARR + 1U) / 100U);              /* scale to ARR units (*3) */
    if (ccr > (uint16_t)MOTOR_ARR) { ccr = (uint16_t)MOTOR_ARR; }
    return ccr;
}

/* ---------------------------------------------------------------------- */
/* @brief: Initialise low-side SD GPIO and start three TIM3 PWM channels.
 *         TIM3 itself is already configured by MX_TIM3_Init().            */
void Motor_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pull  = GPIO_NOPULL;

    g.Pin = MOTOR_UL_PIN;  HAL_GPIO_Init(MOTOR_UL_PORT, &g);  /* PA6 */
    g.Pin = MOTOR_VL_PIN;  HAL_GPIO_Init(MOTOR_VL_PORT, &g);  /* PB7 */
    g.Pin = MOTOR_WL_PIN;  HAL_GPIO_Init(MOTOR_WL_PORT, &g);  /* PB8 */

    /* All SD pins low (disabled) */
    HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_RESET);

    /* Start PWM channels at duty = 0 */
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void Motor_Control_Init(void)
{
    Motor_Init();
    Hall_Init();
}

/* ---------------------------------------------------------------------- */
/* @brief: Drive the 6-step commutation outputs.
 *         Exactly matches the original GPIO states from BLDC_Motor.c.     *
 *   case 1  UH-WL:  TIM3_CH2 (PA7) PWM, UL(PA6)=SET,  VL(PB7)=RESET, WL(PB8)=SET  *
 *   case 2  VH-WL:  TIM3_CH3 (PB0) PWM, UL=RESET, VL=SET,  WL=SET   *
 *   case 3  VH-UL:  TIM3_CH3 (PB0) PWM, UL=SET,  VL=SET,  WL=RESET   *
 *   case 4  WH-UL:  TIM3_CH4 (PB1) PWM, UL=SET,  VL=RESET, WL=SET   *
 *   case 5  WH-VL:  TIM3_CH4 (PB1) PWM, UL=RESET, VL=SET,  WL=SET   *
 *   case 6  UH-VL:  TIM3_CH2 (PA7) PWM, UL=SET,  VL=SET,  WL=RESET   */
void Motor_Set(uint8_t position)
{
    switch (position) {
    case 1:
        TIM3->CCR2 = Motor_Speed; TIM3->CCR3 = 0;            TIM3->CCR4 = 0;
        HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);
        break;
    case 2:
        TIM3->CCR2 = 0;           TIM3->CCR3 = Motor_Speed;  TIM3->CCR4 = 0;
        HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);
        break;
    case 3:
        TIM3->CCR2 = 0;           TIM3->CCR3 = Motor_Speed;  TIM3->CCR4 = 0;
        HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_RESET);
        break;
    case 4:
        TIM3->CCR2 = 0;           TIM3->CCR3 = 0;            TIM3->CCR4 = Motor_Speed;
        HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);
        break;
    case 5:
        TIM3->CCR2 = 0;           TIM3->CCR3 = 0;            TIM3->CCR4 = Motor_Speed;
        HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_SET);
        break;
    case 6:
        TIM3->CCR2 = Motor_Speed; TIM3->CCR3 = 0;            TIM3->CCR4 = 0;
        HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_RESET);
        break;
    default:
        TIM3->CCR2 = 0; TIM3->CCR3 = 0; TIM3->CCR4 = 0;
        HAL_GPIO_WritePin(MOTOR_UL_PORT, MOTOR_UL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_VL_PORT, MOTOR_VL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_WL_PORT, MOTOR_WL_PIN, GPIO_PIN_RESET);
        break;
    }
}

/* ---------------------------------------------------------------------- */
/* @brief: Anti-reversal braking with debounce.
 *         Returns the braking CCR value once direction is confirmed.      */
static int16_t BrakeControl(int16_t set_speed)
{
    static int8_t counter = 0;
    uint8_t power_on = 0;
    set_speed = set_speed / 2;  /* braking force coefficient */

    if (hall_direction == -1) {
        counter++;
        if (counter >= MOTOR_BRAKE_DEBOUNCE) { counter = MOTOR_BRAKE_DEBOUNCE; power_on = 0; }
        else                                 { power_on = 1; }
    } else {
        counter--;
        if (counter <= 0) { counter = 0; power_on = 1; }
        else              { power_on = 0; }
    }

    int16_t out = (int16_t)calculate_min_slip_duty() - (-set_speed);
    if (out < 1)                  { out = 1; }
    if (out > (int16_t)MOTOR_ARR) { out = (int16_t)MOTOR_ARR; }

    return power_on ? out : 0;
}

/* ---------------------------------------------------------------------- */
void Motor_Direction(int direction)
{
    int pos = hall_sensor.position;
    int mp = 0;

    switch (direction) {
    case -1: { // Forward
        // derived perfectly from FOC calibration phase mapping
        mp = 3 - pos;
        while (mp < 1) mp += 6;
        while (mp > 6) mp -= 6;
        Motor_Set((uint8_t)mp);
        break;
    }
    case 1: { // Reverse
        // derived perfectly from FOC calibration phase mapping
        mp = 6 - pos;
        while (mp < 1) mp += 6;
        while (mp > 6) mp -= 6;
        Motor_Set((uint8_t)mp);
        break;
    }
    default:
        Motor_Set(0);
        break;
    }
}

/* ---------------------------------------------------------------------- */
/* First-order IIR speed filter �?matches original setSpeed_filter() */
static float setSpeed_filter(float input)
{
    /* Scale input: (ARR+1)/100 = 300/100 = 3 */
    input = input * ((float)(MOTOR_ARR + 1U) / 100.0f);

    /* Reset on sign reversal */
    static float last_input = 0.0f;
    int sign_change = ((last_input >= 0.0f && input < 0.0f) ||
                       (last_input <= 0.0f && input > 0.0f)) ? 1 : 0;
    last_input = input;
    if (sign_change) { Motor_filter_prev = 0.0f; }

    float output = 0.0f;
    if (!sign_change && input < Motor_filter_prev && output > input) {
        output = (MOTOR_FILTER_ALPHA * 100.0f) * input +
                 (1.0f - MOTOR_FILTER_ALPHA * 100.0f) * Motor_filter_prev;
    } else {
        output = MOTOR_FILTER_ALPHA * input +
                 (1.0f - MOTOR_FILTER_ALPHA) * Motor_filter_prev;
    }

    if (isCoasting) {
        output = (MOTOR_FILTER_ALPHA * 100.0f) * input +
                 (1.0f - MOTOR_FILTER_ALPHA * 100.0f) * Motor_filter_prev;
    }
    if (input == 0.0f) {
        output = (MOTOR_FILTER_ALPHA * 10.0f) * input +
                 (1.0f - MOTOR_FILTER_ALPHA * 10.0f) * Motor_filter_prev;
    }

    Motor_filter_prev = output;
    return output;
}

/* ---------------------------------------------------------------------- */
void Motor_Control(int inputSpeed)
{
    int Speed = (int)setSpeed_filter((float)inputSpeed);

    if (Speed > 0) {
        Motor_Direction(1 * MOTOR_DIRECTION);
        uint16_t min_duty = calculate_min_slip_duty();
        if ((uint16_t)Speed >= min_duty) {
            Motor_Speed = (uint16_t)Speed;
            isCoasting  = 0;
        } else {
            Motor_Speed = min_duty;
            isCoasting  = 1;
        }
    } else if (Speed < 0) {
        Motor_Direction(1 * MOTOR_DIRECTION);
        Motor_Speed = (uint16_t)BrakeControl((int16_t)Speed);
        isCoasting  = 0;
    } else {
        Motor_Direction(0);
        Motor_Speed = 0;
        isCoasting  = 0;
    }
}


