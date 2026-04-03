#ifndef __STORAGE_H
#define __STORAGE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Flash non-volatile storage
 * Uses the last 1 KB page of STM32F103C8T6 (64 KB flash).
 *
 * *** If you use F103CBT6 (128 KB), change FLASH_STORE_ADDR to 0x0801FC00 ***
 * ----------------------------------------------------------------------- */
#define FLASH_STORE_ADDR   0x0800FC00UL   /* last page of 64 KB device */
#define USE_AREA           25             /* number of uint16_t slots used */

/* RAM mirror of flash data (indexed 0..USE_AREA-1)
 * Layout:
 *   [0]      magic 0x1218
 *   [1..4]   "Mile" label chars
 *   [5]      meters (0–999)
 *   [6]      'm\0'
 *   [7]      kilometers (0–999)
 *   [8]      'km'
 *   [9]      kilo-kilometers
 *   [10..11] 'kkm\0'
 *   [12..15] "RunTime\0" label
 *   [16]     thousand-hours
 *   [17]     hours
 *   [18]     'h\0'
 *   [19]     minutes
 *   [20..21] 'min\0'
 *   [22..24] reserved
 */
extern uint16_t Store_Data[USE_AREA];

uint32_t Flash_ReadWord(uint32_t addr);
uint16_t Flash_ReadHalfWord(uint32_t addr);
uint8_t  Flash_ReadByte(uint32_t addr);

void Flash_ErasePage(uint32_t page_addr);
void Flash_EraseAllPages(void);   /* ⚠ erases entire flash including program! */

void Flash_WriteWord(uint32_t addr, uint32_t data);
void Flash_WriteHalfWord(uint32_t addr, uint32_t data);

void Store_Init(void);
void Store_Save(void);

void Store_Mileage(uint16_t plus_meters);
void Store_Mileage_0(void);

void Store_runTime(uint16_t plus_minutes);
void Store_runTime_0(void);

#endif /* __STORAGE_H */
