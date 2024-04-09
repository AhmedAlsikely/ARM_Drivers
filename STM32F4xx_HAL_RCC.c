#include "STM32F4xx_HAL_RCC.h"

Status_t RCC_OscConfig(RCC_Cfg *RCC_InitStruct){
    Status_t RetStatus = R_OK;
    if(NULL== RCC_InitStruct)
    {
        RetStatus = NULL_POINTER;
    }
    else
    {
        /*------------------------------- HSE Configuration -------------------- */
        /*============================== HSE External Source =================== */
        if(RCC_OSCILLATORTYPE_HSE_EC == RCC_InitStruct->SystemClockSourse)
        {
            SET_BIT(RCC->CR, 16);
            SET_BIT(RCC->CR, 18); /* Enable ByPass*/
        }
        // else if()
        // {

        // }
        // if()
        // {

        // }
        // else if()
        // {

        // }
        // if()
        // {

        // }
        // else if()
        // {

        // }
        // if()
        // {

        // }
        // else if()
        // {

        // }
    }
}