#ifndef __ADC_SENSOR_H
#define __ADC_SENSOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * ADC channel mapping (4-channel circular DMA, scan mode)
 *   [0] PA1/CH1 = motor current / AGND reference
 *   [1] PA2/CH2 = battery voltage divider
 *   [2] PA4/CH4 = spare
 *   [3] PA5/CH5 = NTC thermistor
 * ----------------------------------------------------------------------- */
extern volatile uint16_t adc_values[4];

void  ADC_Sensor_Init(void);
float ADC_GetBatteryVoltage(void);
float ADC_GetTemperature(void);      /* °C from NTC + 220 kΩ series */

#endif /* __ADC_SENSOR_H */
