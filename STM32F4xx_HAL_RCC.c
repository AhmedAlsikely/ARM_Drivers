#include "STM32F4xx_HAL_RCC.h"

static uint32_t RCC_CalculateSysClkFreq(uint32_t pllSource, uint32_t pllM, uint32_t pllN, uint32_t pllP);

Status_t RCC_Init(RCC_Cfg *Rcc_val){
    Status_t RetStatus = R_OK;
    if(NULL == Rcc_val)
    {
        RetStatus = NULL_POINTER;
    }
    else
    {
        if(RCC_OSCILLATORTYPE_PLL == Rcc_val->SystemClockSourse)
        {
            RetStatus = RCC_ConfigurePLL(&Rcc_val->PLL_Param);
            if(R_OK == RetStatus)
            {
                RetStatus = RCC_ConfigurePrescalers(&Rcc_val->BusesPrescaler);
                if(R_OK != RetStatus)
                {
                    RetStatus = R_NOK;  
                }
            }
            else
            {
                RetStatus = R_NOK; 
            }
        }
        else
        {
            RetStatus = RCC_EnableClockSource(Rcc_val->SystemClockSourse);
            if(R_OK == RetStatus)
            {
                RetStatus = RCC_ConfigurePrescalers(&Rcc_val->BusesPrescaler);
                if(R_OK != RetStatus)
                {
                    RetStatus = R_NOK;  
                }
            }
            else
            {
                RetStatus = R_NOK;   
            }
        } 
    }
    return RetStatus;
}




Status_t RCC_ConfigurePLL(PLLCFG_t *pllParam){
    Status_t RetStatus = R_OK;
    if(NULL== pllParam)
    {
        RetStatus = NULL_POINTER;
    }
    else
    {
        // Disable PLL
        CLEAR_BIT(RCC->CR, RCC_CR_HSION_Pos);

        // Wait until PLL is disabled
        while(READ_BIT(READ_REG(RCC->CR), RCC_CR_PLLRDY_Pos , 1));

        // Configure PLL source
        MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_Msk , pllParam->PLLSRC);
        // Configure PLL  multiplication factors, and division factors
        MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk , (pllParam->PLLP << RCC_PLLCFGR_PLLP_Pos));
        MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk , (pllParam->PLLN << RCC_PLLCFGR_PLLN_Pos));
        MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk , (pllParam->PLLM << RCC_PLLCFGR_PLLM_Pos));
        MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_Msk , (pllParam->PLLQ << RCC_PLLCFGR_PLLQ_Pos));

        // Enable PLL
        SET_BIT(RCC->CR , RCC_CR_PLLON_Pos);

        // Wait until PLL is ready
        while(!(READ_BIT(READ_REG(RCC->CR), RCC_CR_PLLRDY_Pos , RCC_CR_PLLRDY_Msk)));
        RetStatus = R_OK;
    }
    return RetStatus;
}


Status_t RCC_EnablePeripheralClock(uint32_t peripheral){
    Status_t RetStatus = R_OK;
    if(NULL == peripheral)
    {
        RetStatus = WRONG_PARAMETER;
    }
    else
    {
        if (peripheral < 32) {
            SET_BIT(RCC->AHB1ENR, peripheral);
        } else if (peripheral < 64) {
            SET_BIT(RCC->AHB2ENR, (peripheral - 32));
        } else if (peripheral < 128) {
            SET_BIT(RCC->APB1ENR, (peripheral - 64));
        } else if (peripheral < 160) {
            SET_BIT(RCC->APB2ENR, (peripheral - 128));
        } else{
            RetStatus = OUT_OF_RANGE;
        }
    }
    return RetStatus;
}



Status_t RCC_DisablePeripheralClock(uint32_t peripheral){
    Status_t RetStatus = R_OK;
    if(NULL == peripheral)
    {
        RetStatus = WRONG_PARAMETER;
    }
    else
    {
        if (peripheral < 32) {
            CLEAR_BIT(RCC->AHB1ENR, peripheral);
        } else if (peripheral < 64) {
            CLEAR_BIT(RCC->AHB2ENR, (peripheral - 32));
        } else if (peripheral < 128) {
            CLEAR_BIT(RCC->APB1ENR, (peripheral - 64));
        } else if (peripheral < 160) {
            CLEAR_BIT(RCC->APB2ENR, (peripheral - 128));
        } else{
            RetStatus = OUT_OF_RANGE;
        }
    }
    return RetStatus;
}


uint32_t RCC_GetSystemClockFreq(void){
    uint32_t sysclk = 0;

    // Get the system clock source
    uint32_t sysclkSrc = READ_BIT(RCC->CFGR , RCC_CFGR_SWS_Pos, RCC_CFGR_SWS_Msk) ;

    switch (sysclkSrc) {
        case RCC_CFGR_SWS_HSI: // HSI used as system clock
            sysclk = HSI_VALUE;
            break;
        case RCC_CFGR_SWS_HSE: // HSE used as system clock
            sysclk = HSE_VALUE;
            break;
        case RCC_CFGR_SWS_PLL: // PLL used as system clock

            sysclk = RCC_CalculateSysClkFreq(
                READ_BIT(RCC->PLLCFGR , RCC_PLLCFGR_PLLSRC_Pos, RCC_PLLCFGR_PLLSRC_Msk),
                READ_BIT(RCC->PLLCFGR , RCC_PLLCFGR_PLLM_Pos, RCC_PLLCFGR_PLLM_Msk),
                READ_BIT(RCC->PLLCFGR , RCC_PLLCFGR_PLLN_Pos, RCC_PLLCFGR_PLLN_Msk),
                READ_BIT(RCC->PLLCFGR , RCC_PLLCFGR_PLLP_Pos, RCC_PLLCFGR_PLLP_Msk)
            );
            break;
        // Add cases for other system clock sources if needed
    }

    return sysclk;
}



Status_t RCC_SelectSystemClockSource(RCC_ClockSource source){
    Status_t RetStatus = R_OK;
    if((source > 3) || (NULL == source))
    {
        RetStatus = WRONG_PARAMETER;
    }
    else
    {
        switch (source)
        {
        case RCC_OSCILLATORTYPE_HSI:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk , RCC_CFGR_SW_HSI);
            break;
        case RCC_OSCILLATORTYPE_HSE_EC:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk , RCC_CFGR_SW_HSE);
            SET_BIT(RCC->CR, RCC_CR_HSEBYP_Pos);
            break;
        case RCC_OSCILLATORTYPE_HSE_CRYST:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk , RCC_CFGR_SW_HSE);
            CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP_Pos);
            break;
        case RCC_OSCILLATORTYPE_PLL:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk , RCC_CFGR_SW_PLL);
            break;
        }
        RetStatus = R_OK;
    }
    return RetStatus;
}


Status_t RCC_EnableClockSource(RCC_ClockSource source){
    Status_t RetStatus = R_OK;
    if((source > 3) || (NULL == source))
    {
        RetStatus = WRONG_PARAMETER;
    }
    else
    {
        switch (source)
        {
        case RCC_OSCILLATORTYPE_HSI:
            SET_BIT(RCC->CR, RCC_CR_HSION_Pos);
            while(READ_BIT(RCC->CR, RCC_CR_HSIRDY_Pos, 1));
            break;
        case RCC_OSCILLATORTYPE_HSE_EC:
            SET_BIT(RCC->CR, RCC_CR_HSEON_Pos);
            while(READ_BIT(RCC->CR, RCC_CR_HSERDY_Pos, 1));
            break;
        case RCC_OSCILLATORTYPE_HSE_CRYST:
            SET_BIT(RCC->CR, RCC_CR_HSEON_Pos);
            while(READ_BIT(RCC->CR, RCC_CR_HSERDY_Pos, 1));
            break;
        case RCC_OSCILLATORTYPE_PLL:
            SET_BIT(RCC->CR, RCC_CR_PLLON_Pos);
            while(READ_BIT(RCC->CR, RCC_CR_PLLON_Pos, 1));
            break;
        }
        RetStatus = R_OK;
    }
    return RetStatus;
}


Status_t RCC_DisableClockSource(RCC_ClockSource source){
    Status_t RetStatus = R_OK;
    if((source > 3) || (NULL == source))
    {
        RetStatus = WRONG_PARAMETER;
    }
    else
    {
        switch (source)
        {
        case RCC_OSCILLATORTYPE_HSI:
            CLEAR_BIT(RCC->CR, RCC_CR_HSION_Pos);
            
            break;
        case RCC_OSCILLATORTYPE_HSE_EC:
            CLEAR_BIT(RCC->CR, RCC_CR_HSEON_Pos);
            break;
        case RCC_OSCILLATORTYPE_HSE_CRYST:
            CLEAR_BIT(RCC->CR, RCC_CR_HSEON_Pos);
            break;
        case RCC_OSCILLATORTYPE_PLL:
            CLEAR_BIT(RCC->CR, RCC_CR_PLLON_Pos);
            break;
        }
        RetStatus = R_OK;
    }
    return RetStatus;
}

Status_t RCC_ConfigurePrescalers(RCC_prescalers *presc_val){
    Status_t RetStatus = R_OK;
    if(NULL == presc_val)
    {
        RetStatus = NULL_POINTER;
    }
    else
    {
        // Configure AHB prescaler
        MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk , presc_val->AHBCLKDIVIDER);
        // Configure APB1 prescaler
        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk , presc_val->APB1CLKDIVIDER);
        // Configure APB2 prescaler
        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk , presc_val->APB2CLKDIVIDER);
        // Configure RTC prescaler
        MODIFY_REG(RCC->CFGR, RCC_CFGR_RTCPRE_Msk , presc_val->RTCDIVIDER);
        Status_t RetStatus = R_OK;
    }
    return RetStatus;
}



Status_t RCC_EnablePeripheralClockINLowPowerMode(uint32_t peripheral){
    Status_t RetStatus = R_OK;
    if(NULL == peripheral)
    {
        RetStatus = WRONG_PARAMETER;
    }
    else
    {
        if (peripheral < 32) {
            SET_BIT(RCC->AHB1LPENR, peripheral);
        } else if (peripheral < 64) {
            SET_BIT(RCC->AHB2LPENR, (peripheral - 32));
        } else if (peripheral < 128) {
            SET_BIT(RCC->APB1LPENR, (peripheral - 64));
        } else if (peripheral < 160) {
            SET_BIT(RCC->APB2LPENR, (peripheral - 128));
        } else{
            RetStatus = OUT_OF_RANGE;
        }
    }
    return RetStatus;
}



Status_t RCC_DISablePeripheralClockINLowPowerMode(uint32_t peripheral){
    Status_t RetStatus = R_OK;
    if(NULL == peripheral)
    {
        RetStatus = WRONG_PARAMETER;
    }
    else
    {
        if (peripheral < 32) {
            CLEAR_BIT(RCC->AHB1LPENR, peripheral);
        } else if (peripheral < 64) {
            CLEAR_BIT(RCC->AHB2LPENR, (peripheral - 32));
        } else if (peripheral < 128) {
            CLEAR_BIT(RCC->APB1LPENR, (peripheral - 64));
        } else if (peripheral < 160) {
            CLEAR_BIT(RCC->APB2LPENR, (peripheral - 128));
        } else{
            RetStatus = OUT_OF_RANGE;
        }
    }
    return RetStatus;
}


Status_t RCC_ResetPeripheral(uint32_t peripheral){
    Status_t RetStatus = R_OK;
    if(NULL == peripheral)
    {
        RetStatus = WRONG_PARAMETER;
    }
    else
    {
        if (peripheral < 32) {
            SET_BIT(RCC->AHB1RSTR, peripheral);
            CLEAR_BIT(RCC->AHB1RSTR, peripheral);
        } else if (peripheral < 64) {
            SET_BIT(RCC->AHB2RSTR, (peripheral - 32));
            CLEAR_BIT(RCC->AHB2RSTR, (peripheral - 32));
        } else if (peripheral < 128) {
            SET_BIT(RCC->APB1RSTR, (peripheral - 64));
            CLEAR_BIT(RCC->APB1RSTR, (peripheral - 64));
        } else if (peripheral < 160) {
            SET_BIT(RCC->APB2RSTR, (peripheral - 128));
            CLEAR_BIT(RCC->APB2RSTR, (peripheral - 128));
        } else{
            RetStatus = OUT_OF_RANGE;
        }
    }
    return RetStatus;
}

void RCC_DetectClockFailure(void){
    // Check CSS status flag
    if (READ_BIT(RCC->CR, RCC_CR_CSSON_Pos, RCC_CR_CSSON_Msk)) {
        // check if a clock failure occurred
        if (READ_BIT(RCC->CIR, RCC_CIR_CSSF_Pos, 1)) {
            // Reset the system     
        }
    }
}


void RCC_EnableClockSecuritySystem(void){
    SET_BIT(RCC->CR,RCC_CR_CSSON_Pos);
}


static uint32_t RCC_CalculateSysClkFreq(uint32_t pllSource, uint32_t pllM, uint32_t pllN, uint32_t pllP) {
    // Calculate system clock frequency based on PLL configuration
    uint32_t pllvco = 0, sysclk = 0;

    if (pllSource == RCC_PLLSource_HSI) {
        pllvco = (HSI_VALUE / pllM) * pllN;
    } else if (pllSource == RCC_PLLSource_HSE) {
        pllvco = (HSE_VALUE / pllM) * pllN;
    } else {
        // Invalid PLL source
        return 0;
    }

    sysclk = pllvco / pllP;

    return sysclk;
}
