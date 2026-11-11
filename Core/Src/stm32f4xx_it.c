#include "Bsp.h"

/* Necessario ao HAL_GetTick/HAL_Delay; startup fornece os demais handlers. */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
