/*
 * File Name: STM32F4xx_HAL_RCC.h
 * Version  : V 1.0
 * Created	:  Apr 9, 2024
 * Author	: Ahmed Alsikely
 */
#ifndef _STM32F4xx_HAL_RCC_H_
#define _STM32F4xx_HAL_RCC_H_

/*************************************** Includes *****************************************/
#include "Std_CFG.h"
#include "STM32F4xx_Peripheral_Defs.h"
/*****************************************************************************************/
/**************************************** Macros *****************************************/
/*****************************************************************************************/
#define PERIPH_BASE 0x40000000UL
#define AHB1_PERIPH_BASE (PERIPH_BASE + 0x00020000UL)
#define RCC_PERIPH_BASE (AHB1_PERIPH_BASE + 0x00003800)
#define RCC ((RCC_TypeDef *)RCC_PERIPH_BASE)

// /* @defgroup RCC_SYSCLK_TYPE oscillator type*/
// #define RCC_OSCILLATORTYPE_HSE_EC                0x00000000U /*Select high speed external with hardware configration External clock */
// #define RCC_OSCILLATORTYPE_HSE_CRYST             0x00000001U /*Select high speed external with hardware configration Crystal/ceramic resonators */
// #define RCC_OSCILLATORTYPE_HSI                   0x00000002U /*Select high speed internal*/
// #define RCC_OSCILLATORTYPE_PLL                   0x00000003U /*Select high speed PLL*/

#define HSI_VALUE               (16000000UL)
#define HSE_VALUE               (8000000UL)

#define RCC_CFGR_SWS_HSI        (0U) /* HSI used as system clock */

#define RCC_CFGR_SWS_HSE        (1U) /* HSE used as system clock */

#define RCC_CFGR_SWS_PLL        (2U) /* PLL used as system clock */

#define RCC_CFGR_SW_HSI         (0U) /* HSI Select as system clock */

#define RCC_CFGR_SW_HSE         (1U) /* HSE Select as system clock */

#define RCC_CFGR_SW_PLL         (2U) /* PLL Select as system clock */

#define RCC_PLLSource_HSI       (0U)
#define RCC_PLLSource_HSE       (1U)
/*========================= Options for Configure PreScalar of High Speed AHB Bus=====================*/
// #define RCC_CFGR_HPRE_POS   (4U)
// #define RCC_CFGR_HPRE_MSK   (0xFUL << RCC_CFGR_HPRE_POS)

#define RCC_AHB_PRE_DIV1	    (0x00000000)	/*(0<<4) */
#define RCC_AHB_PRE_DIV2	    (0x00000080)	/*(8<<4) */
#define RCC_AHB_PRE_DIV4	    (0x00000090)	/*(9<<4) */
#define RCC_AHB_PRE_DIV8	    (0x000000A0)	/*(10<<4)*/
#define RCC_AHB_PRE_DIV16	    (0x000000B0)	/*(11<<4)*/
#define RCC_AHB_PRE_DIV64	    (0x000000C0)	/*(12<<4)*/
#define RCC_AHB_PRE_DIV128	    (0x000000D0)	/*(13<<4)*/
#define RCC_AHB_PRE_DIV256	    (0x000000E0)	/*(14<<4)*/
#define RCC_AHB_PRE_DIV512	    (0x000000F0)	/*(15<<4)*/

/*========================= Options for Configure PreScalar of Low Speed APB1 Bus=====================*/
// #define RCC_CFGR_PPRE1_POS   (10U)
// #define RCC_CFGR_PPRE1_MSK   (0x7UL << RCC_CFGR_PPRE1_POS)

#define RCC_APB1_PRE_DIV1	    (0x00000000) /*(0<<10)*/
#define RCC_APB1_PRE_DIV2	    (0x00001000) /*(4<<10)*/
#define RCC_APB1_PRE_DIV4	    (0x00001400) /*(5<<10)*/
#define RCC_APB1_PRE_DIV8	    (0x00001800) /*(6<<10)*/
#define RCC_APB1_PRE_DIV16	    (0x00001C00) /*(7<<10)*/

/*========================= Options for Configure PreScalar of High Speed APB2 Bus=====================*/
//not to exceed 84 MHz
// #define RCC_CFGR_PPRE2_POS   (13U)
// #define RCC_CFGR_PPRE1_MSK   (0x7UL << RCC_CFGR_PPRE2_POS)

#define RCC_APB2_PRE_DIV1	    (0x00000000) /*(0<<13) */
#define RCC_APB2_PRE_DIV2	    (0x00008000) /*(4<<13) */
#define RCC_APB2_PRE_DIV4	    (0x0000A000) /*(5<<13)*/
#define RCC_APB2_PRE_DIV8	    (0x0000C000) /*(6<<13)*/
#define RCC_APB2_PRE_DIV16	    (0x0000E000) /*(7<<13)*/

/*=================================== The Peripherals on AHB1 bus Macros =====================================*/
#define RCC_GPIOA_POS           RCC_AHB_GPIOA_POS      
#define RCC_GPIOB_POS			RCC_AHB_GPIOB_POS	   
#define RCC_GPIOC_POS			RCC_AHB_GPIOC_POS	   
#define RCC_GPIOD_POS			RCC_AHB_GPIOD_POS       
#define RCC_GPIOE_POS			RCC_AHB_GPIOE_POS	   
#define RCC_GPIOH_POS			RCC_AHB_GPIOH_POS	   
#define RCC_CRC_POS			    RCC_AHB_CRC_POS   
#define RCC_DMA1_POS			RCC_AHB_DMA1_POS	   
#define RCC_DMA2_POS			RCC_AHB_DMA2_POS	   
/*=================================== The Peripherals on AHB2 bus=====================================*/
#define RCC_OTGFS_POS	        RCC_AHB_OTGFS_POS			
/*=================================== The Peripherals on APB1 bus=====================================*/
#define RCC_TIM2_POS			RCC_APB_TIM2_POS	
#define RCC_TIM3_POS			RCC_APB_TIM3_POS	
#define RCC_TIM4_POS			RCC_APB_TIM4_POS	
#define RCC_TIM5_POS			RCC_APB_TIM5_POS	
#define RCC_WWDG_POS			RCC_APB_WWDG_POS	
#define RCC_SPI2_POS			RCC_APB_SPI2_POS	
#define RCC_SPI3_POS			RCC_APB_SPI3_POS	
#define RCC_USART2_POS			RCC_APB_USART2_POS
#define RCC_I2C1_POS			RCC_APB_I2C1_POS	
#define RCC_I2C2_POS			RCC_APB_I2C2_POS	
#define RCC_I2C3_POS			RCC_APB_I2C3_POS	
#define RCC_PWR_POS				RCC_APB_PWR_POS

/*=================================== The Peripherals on APB2 bus=====================================*/
#define RCC_TIM1_POS			RCC_APB_TIM1_POS	 
#define RCC_USART1_POS			RCC_APB_USART1_POS
#define RCC_USART6_POS			RCC_APB_USART6_POS
#define RCC_ADC1_POS			RCC_APB_ADC1_POS	
#define RCC_SDIO_POS			RCC_APB_SDIO_POS	
#define RCC_SPI1_POS			RCC_APB_SPI1_POS	
#define RCC_SPI4_POS			RCC_APB_SPI4_POS	
#define RCC_SYSCFG_POS			RCC_APB_SYSCFG_POS
#define RCC_TIM9_POS			RCC_APB_TIM9_POS	
#define RCC_TIM10_POS			RCC_APB_TIM10_POS	
#define RCC_TIM11_POS			RCC_APB_TIM11_POS	

/***************************************************************************************************/
/**************************************** Macros Functions *****************************************/
/***************************************************************************************************/


/**************************************** Data types *****************************************/

typedef struct {
	 uint32_t PLLM;   /*PLLM Value must be between 2 and 63*/
	 uint32_t PLLP;   /*PLLP must be 2, 4, 6, or 8*/
	 uint32_t PLLN;   /*PLLN Value must be between 192 and 432*/
	 uint32_t PLLSRC; /*PLLSRC must be RCC_PLLSRC_HSI or RCC_PLLSRC_HSE*/
	 uint32_t PLLQ;   /*PLLQ Value must be between 2 and 15*/
}PLLCFG_t;

typedef struct {
	 uint32_t AHBCLKDIVIDER;   
	 uint32_t APB1CLKDIVIDER;   
     uint32_t APB2CLKDIVIDER;
     uint32_t RTCDIVIDER;
     
}RCC_prescalers;

typedef enum{
    RCC_OSCILLATORTYPE_HSE_EC = 0,
    RCC_OSCILLATORTYPE_HSE_CRYST,
    RCC_OSCILLATORTYPE_HSI,
    RCC_OSCILLATORTYPE_PLL
}RCC_ClockSource;

typedef struct {
    RCC_prescalers BusesPrescaler;
    PLLCFG_t PLL_Param;
    RCC_ClockSource SystemClockSourse;

}RCC_Cfg;

/**************************************** Software Interfaces Declarations *****************************************/

/**
  * @brief  Initializes the RCC (Reset and Clock Control) module with the specified configuration.
  * @param  Rcc_val: Pointer to a structure containing RCC configuration parameters.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg NULL_POINTER: The Rcc_val pointer is NULL.
  *            @arg R_NOK: Operation failed.
  */
Status_t RCC_Init(RCC_Cfg *Rcc_val);



/**
  * @brief  Configures the PLL (Phase Locked Loop) of the STM32F401 microcontroller.
  * @param  pllParam: Pointer to a structure containing PLL configuration parameters.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg NULL_POINTER: The pllParam pointer is NULL.
  */
Status_t RCC_ConfigurePLL(PLLCFG_t *pllParam);



/**
  * @brief  Enables the clock for the specified peripheral.
  * @param  peripheral: The peripheral to enable the clock for.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg WRONG_PARAMETER: Invalid parameter passed to the function.
  *            @arg OUT_OF_RANGE: Peripheral index out of range.
  */
Status_t RCC_EnablePeripheralClock(uint32_t peripheral);


/**
  * @brief  Disables the clock for the specified peripheral.
  * @param  peripheral: The peripheral to enable the clock for.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg WRONG_PARAMETER: Invalid parameter passed to the function.
  *            @arg OUT_OF_RANGE: Peripheral index out of range.
  */
Status_t RCC_DisablePeripheralClock(uint32_t peripheral);



/**
  * @brief  Gets the frequency of the system clock.
  * @retval uint32_t: Frequency of the system clock in Hertz (Hz).
  *          This function returns the frequency of the system clock source
  *          currently selected by the RCC_CFGR_SWS bits in the RCC_CFGR register.
  */
uint32_t RCC_GetSystemClockFreq(void);


/**
  * @brief  Selects the system clock source.
  * @param  source: The desired system clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_OSCILLATORTYPE_HSI: HSI oscillator selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_HSE_EC: HSE oscillator with external clock source selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_HSE_CRYST: HSE oscillator with crystal/ceramic resonator source selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_PLL: PLL selected as system clock source.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg WRONG_PARAMETER: Invalid parameter passed to the function.
  */
Status_t RCC_SelectSystemClockSource(RCC_ClockSource source);


/**
  * @brief  Enable clock source.
  * @param  source: The desired system clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_OSCILLATORTYPE_HSI: HSI oscillator selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_HSE_EC: HSE oscillator with external clock source selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_HSE_CRYST: HSE oscillator with crystal/ceramic resonator source selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_PLL: PLL selected as system clock source.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg WRONG_PARAMETER: Invalid parameter passed to the function.
  */
Status_t RCC_EnableClockSource(RCC_ClockSource source);


/**
  * @brief  Disable clock source.
  * @param  source: The desired system clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_OSCILLATORTYPE_HSI: HSI oscillator selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_HSE_EC: HSE oscillator with external clock source selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_HSE_CRYST: HSE oscillator with crystal/ceramic resonator source selected as system clock source.
  *            @arg RCC_OSCILLATORTYPE_PLL: PLL selected as system clock source.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg WRONG_PARAMETER: Invalid parameter passed to the function.
  */
Status_t RCC_DisableClockSource(RCC_ClockSource source);

/**
  * @brief  Configures the clock prescalers of the STM32F401 microcontroller.
  * @param  presc_val: Pointer to a structure containing prescaler configuration values.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg NULL_POINTER: The presc_val pointer is NULL.
  */
Status_t RCC_ConfigurePrescalers(RCC_prescalers *presc_val);



/**
  * @brief  Enables the clock for the specified peripheral in low-power mode.
  * @param  peripheral: The peripheral to enable the clock for.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg WRONG_PARAMETER: Invalid parameter passed to the function.
  *            @arg OUT_OF_RANGE: Peripheral index out of range.
  */
Status_t RCC_EnablePeripheralClockINLowPowerMode(uint32_t peripheral);



/**
  * @brief  DISables the clock for the specified peripheral in low-power mode.
  * @param  peripheral: The peripheral to enable the clock for.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg WRONG_PARAMETER: Invalid parameter passed to the function.
  *            @arg OUT_OF_RANGE: Peripheral index out of range.
  */
Status_t RCC_DISablePeripheralClockINLowPowerMode(uint32_t peripheral);


/**
  * @brief  Resets the specified peripheral.
  * @param  peripheral: The peripheral to reset.
  * @retval Status_t: Status of the operation.
  *          This function returns one of the following values:
  *            @arg R_OK: Operation successful.
  *            @arg WRONG_PARAMETER: Invalid parameter passed to the function.
  *            @arg OUT_OF_RANGE: Peripheral index out of range.
  */
Status_t RCC_ResetPeripheral(uint32_t peripheral);



/**
  * @brief  Detects a clock failure and resets the system if detected.
  * @note   This function is typically used to monitor the Clock Security System (CSS)
  *         status flag and perform a system reset if a clock failure is detected.
  *         The CSS monitors the integrity of the HSE oscillator clock.
  * @retval None
  */
void RCC_DetectClockFailure(void);


/**
  * @brief  Enables the Clock Security System (CSS).
  * @note   The Clock Security System (CSS) monitors the integrity of the HSE
  *         oscillator clock. When enabled, it triggers an interrupt or system reset
  *         if a clock failure is detected.
  * @retval None
  */
void RCC_EnableClockSecuritySystem(void);

#endif /*_STM32F4xx_HAL_RCC_H_*/