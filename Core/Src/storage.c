#include "storage.h"
#include "app_timer.h"
#include "main.h"

uint16_t Store_Data[USE_AREA];

/* ---------------------------------------------------------------------- */
/* Raw flash read helpers */
uint32_t Flash_ReadWord(uint32_t addr)     { return *((__IO uint32_t *)addr); }
uint16_t Flash_ReadHalfWord(uint32_t addr) { return *((__IO uint16_t *)addr); }
uint8_t  Flash_ReadByte(uint32_t addr)     { return *((__IO uint8_t  *)addr); }

/* ---------------------------------------------------------------------- */
void Flash_ErasePage(uint32_t page_addr)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Snapshot TIM4 counter to compensate the ~20 ms erase time */
    uint32_t cnt_before = TIM4->CNT;

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    erase.TypeErase  = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = page_addr;
    erase.NbPages    = 1;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    uint32_t cnt_after = TIM4->CNT;
    uint32_t elapsed = (cnt_after >= cnt_before)
                       ? (cnt_after - cnt_before)
                       : ((TIM4->ARR + 1) - cnt_before + cnt_after);
    APP_time += elapsed;

    __set_PRIMASK(primask);
}

/* ⚠ Erases ALL flash pages including program flash — debug/factory use only */
void Flash_EraseAllPages(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = 0x08000000UL;
    erase.NbPages     = 64;   /* STM32F103C8T6: 64 pages × 1 KB */
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();
    __set_PRIMASK(primask);
}

/* ---------------------------------------------------------------------- */
void Flash_WriteHalfWord(uint32_t addr, uint32_t data)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, (uint64_t)data);
    HAL_FLASH_Lock();
    __set_PRIMASK(primask);
}

void Flash_WriteWord(uint32_t addr, uint32_t data)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t)data);
    HAL_FLASH_Lock();
    __set_PRIMASK(primask);
}

/* ---------------------------------------------------------------------- */
void Store_Init(void)
{
    if (Flash_ReadHalfWord(FLASH_STORE_ADDR) != 0x1218U) {
        Flash_ErasePage(FLASH_STORE_ADDR);
        Flash_WriteHalfWord(FLASH_STORE_ADDR, 0x1218U);
        for (uint16_t i = 1; i < USE_AREA; i++) {
            Flash_WriteHalfWord(FLASH_STORE_ADDR + i * 2U, 0x0000U);
        }
    }
    for (uint16_t i = 0; i < USE_AREA; i++) {
        Store_Data[i] = Flash_ReadHalfWord(FLASH_STORE_ADDR + i * 2U);
    }
}

void Store_Save(void)
{
    Store_Mileage(0);
    Store_runTime(0);
    Flash_ErasePage(FLASH_STORE_ADDR);
    Store_Data[0] = 0x1218U;
    for (uint16_t i = 0; i < USE_AREA; i++) {
        Flash_WriteHalfWord(FLASH_STORE_ADDR + i * 2U, Store_Data[i]);
    }
}

/* ---------------------------------------------------------------------- */
void Store_Mileage(uint16_t plus_meters)
{
    Store_Data[1] = ((uint16_t)'M' << 8) | 'i';
    Store_Data[2] = ((uint16_t)'l' << 8) | 'e';
    Store_Data[3] = ((uint16_t)'a' << 8) | 'g';
    Store_Data[4] = ((uint16_t)'e' << 8) | ':';

    uint16_t m  = Store_Data[5];
    Store_Data[6] = ((uint16_t)'m' << 8) | '\0';
    uint16_t km = Store_Data[7];
    Store_Data[8] = ((uint16_t)'k' << 8) | 'm';
    uint16_t kkm = Store_Data[9];
    Store_Data[10] = ((uint16_t)'k' << 8) | 'k';
    Store_Data[11] = ((uint16_t)'m' << 8) | '\0';

    uint32_t total = (uint32_t)kkm * 1000000UL + (uint32_t)km * 1000UL + m;
    total += plus_meters;
    Store_Data[9] = (uint16_t)(total / 1000000UL);
    uint32_t rem  = total % 1000000UL;
    Store_Data[7] = (uint16_t)(rem / 1000UL);
    Store_Data[5] = (uint16_t)(rem % 1000UL);
}

void Store_Mileage_0(void)
{
    Store_Data[5] = 0;
    Store_Data[7] = 0;
    Store_Data[9] = 0;
}

/* ---------------------------------------------------------------------- */
void Store_runTime(uint16_t plus_minutes)
{
    Store_Data[12] = ((uint16_t)'R' << 8) | 'u';
    Store_Data[13] = ((uint16_t)'n' << 8) | 'T';
    Store_Data[14] = ((uint16_t)'i' << 8) | 'm';
    Store_Data[15] = ((uint16_t)'e' << 8) | '\0';

    uint16_t th  = Store_Data[16];
    uint16_t h   = Store_Data[17];
    Store_Data[18] = ((uint16_t)'h' << 8) | '\0';
    uint16_t min = Store_Data[19];
    Store_Data[20] = ((uint16_t)'m' << 8) | 'i';
    Store_Data[21] = ((uint16_t)'n' << 8) | '\0';

    if (plus_minutes != 0U) {
        uint32_t total_min = ((uint32_t)th * 1000UL + h) * 60UL + min;
        total_min += plus_minutes;
        Store_Data[16] = (uint16_t)(total_min / (1000UL * 60UL));
        uint32_t rem   = total_min % (1000UL * 60UL);
        Store_Data[17] = (uint16_t)(rem / 60UL);
        Store_Data[19] = (uint16_t)(rem % 60UL);
    }
}

void Store_runTime_0(void)
{
    Store_Data[16] = 0;
    Store_Data[17] = 0;
    Store_Data[19] = 0;
}
