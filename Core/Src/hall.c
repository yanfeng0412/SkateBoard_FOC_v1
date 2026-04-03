#include "hall.h"
#include "foc.h"
#include "storage.h"

HallSensor_t   hall_sensor    = {0};
int            Hall_Movement_Log = 0;
volatile uint16_t hall_count  = 0;
static int     hall_count_temp = 0;
float          hall_speed     = 0.0f;
float          hall_roll_speed = 0.0f;
int            hall_direction = 0;

/* ---------------------------------------------------------------------- */
void Hall_GetPosition(void)
{
    hall_sensor.HallA = (int)HAL_GPIO_ReadPin(HALL_PORT, HALL_A_PIN);
    hall_sensor.HallB = (int)HAL_GPIO_ReadPin(HALL_PORT, HALL_B_PIN);
    hall_sensor.HallC = (int)HAL_GPIO_ReadPin(HALL_PORT, HALL_C_PIN);
}

void Hall_Position(void)
{
    Hall_GetPosition();
    
    int A = hall_sensor.HallA;  
    int B = hall_sensor.HallB;  
    int C = hall_sensor.HallC;  

    if (A==0 && B==1 && C==0) { hall_sensor.position = 1; }
    if (A==0 && B==1 && C==1) { hall_sensor.position = 2; }
    if (A==0 && B==0 && C==1) { hall_sensor.position = 3; }
    if (A==1 && B==0 && C==1) { hall_sensor.position = 4; }
    if (A==1 && B==0 && C==0) { hall_sensor.position = 5; }
    if (A==1 && B==1 && C==0) { hall_sensor.position = 6; }
}

void Hall_directionLoopCheck(void)
{
    int movement = Hall_Movement_Log - hall_sensor.position;
    Hall_Movement_Log = hall_sensor.position;
    if (movement ==  5) { movement = -1; }
    if (movement == -5) { movement =  1; }
    movement = -movement;

    if (movement != 1 && movement != -1) { return; }

    /* Hysteresis: require 3 consecutive steps in the same direction
     * before updating hall_direction.  This suppresses single-edge
     * bounces from EMI that previously flipped direction every ~150ms
     * (visible as dir:+1 / dir:-1 alternation in the FOC log).
     * At the normal forward speed all consecutive steps are -1 (for
     * DIRECTION=-1 motors) so the 3-step filter adds at most ~3 Hall
     * transitions of latency, which is negligible.                    */
    static int  s_pending     = 0;
    static int  s_pending_cnt = 0;

    if (movement == s_pending) {
        s_pending_cnt++;
    } else {
        s_pending     = movement;
        s_pending_cnt = 1;
    }

    if (s_pending_cnt >= 3) {
        hall_direction = s_pending * DIRECTION;
        /* Do not reset counter — keeps accepting the same direction
         * without re-qualifying after every third step.              */
    }
}

void Hall_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin  = HALL_A_PIN | HALL_B_PIN | HALL_C_PIN;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HALL_PORT, &g);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    Hall_Position();
}

/* EXTI callback — called from HAL_GPIO_EXTI_IRQHandler in stm32f1xx_it.c */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == HALL_A_PIN || GPIO_Pin == HALL_B_PIN || GPIO_Pin == HALL_C_PIN) {
        /* FOC_NotifyHallTransition() applies a 500 us hardware debounce using
         * DWT timing (same infrastructure as the angle estimator).  False
         * edges induced by motor inrush current are typically 100-450 us
         * apart; they are REJECTED here, preventing hall_count inflation that
         * previously made hall_speed read 6-54 km/h at near-zero real speed
         * and caused the PID to apply max-braking current (bus voltage drop).
         *
         * Only update position, count and direction for accepted edges.
         * FOC_HallPositionAccumulate() uses the freshly-updated direction.  */
        if (FOC_NotifyHallTransition()) {
            Hall_Position();
            hall_count_temp++;
            Hall_directionLoopCheck();
            FOC_HallPositionAccumulate();
        }
    }
}

/* ---------------------------------------------------------------------- */
/* RPM low-pass filter */
static float RPM_filter_prev = 0.0f;
static float RPM_filter(float input)
{
    float output = RPM_FILTER_ALPHA * input + (1.0f - RPM_FILTER_ALPHA) * RPM_filter_prev;
    RPM_filter_prev = output;
    return output;
}

static void Get_rollSpeed(void)
{
    float count = (float)hall_count;
    /* HALL_TRANSITIONS_PER_REV = 6 sectors × HALL_POLE_PAIRS
     * For 14-pole (7 pp) motor: 6 × 7 = 42 transitions / mech. rev     */
    float speed = (count / (float)HALL_TRANSITIONS_PER_REV) / HALL_SPEED_PERIOD;  /* rev/s */
    if (hall_direction != 0) { speed *= (float)hall_direction; }
    hall_roll_speed = RPM_filter(speed);

    /* Mileage accumulation: wheel circumference = π × 90 mm ≈ 0.2827 m
     * 7 pole pairs: 42 transitions/rev → 42 / 0.2827 ≈ 149 transitions/m
     * 1 transition ≈ 0.006731 m                                          */
    static uint16_t mileage_count = 0;
    static float    decimal       = 0.0f;
    mileage_count += (uint16_t)count;
    if (mileage_count >= 149) {
        float meter = 0.006731f * (float)mileage_count + decimal;
        uint8_t add_m = (uint8_t)meter;
        decimal = meter - (float)add_m;
        Store_Mileage(add_m);
        mileage_count = 0;
    }
}

float Hall_GetSpeed(void)
{
    Get_rollSpeed();
    hall_speed = hall_roll_speed * (3.141592f * WHEEL_DIAMETER_MM) * 3.6f / 1000.0f;
    return hall_speed;
}

/* Called every 100 ms from TIM4 callback */
void HallSpeed_Timer(void)
{
    hall_count = (uint16_t)hall_count_temp;
    hall_count_temp = 0;
    Hall_GetSpeed();
    /* Require 3 consecutive windows with count<=1 before zeroing speed.
     * One isolated noise pulse (count=1) does not falsely report motion. */
    static uint8_t s_stop_cnt = 0U;
    if (hall_count <= 1U) {
        if (s_stop_cnt < 3U) { s_stop_cnt++; }
        if (s_stop_cnt >= 3U) { hall_speed = 0.0f; }
    } else {
        s_stop_cnt = 0U;
    }
}
