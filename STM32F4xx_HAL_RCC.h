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

/*****************************************************************************************/
/**************************************** Macros *****************************************/
/*****************************************************************************************/
#define PERIPH_BASE 0x40000000UL
#define AHB1_PERIPH_BASE (PERIPH_BASE + 0x00020000UL)
#define RCC_PERIPH_BASE (AHB1_PERIPH_BASE + 0x00003800)
#define RCC ((RCC_TypeDef *)RCC_PERIPH_BASE)

/* @defgroup RCC_SYSCLK_TYPE oscillator type*/
#define RCC_OSCILLATORTYPE_HSE_EC                0x00000000U /*Select high speed external with hardware configration External clock */
#define RCC_OSCILLATORTYPE_HSE_CRYST             0x00000001U /*Select high speed external with hardware configration Crystal/ceramic resonators */
#define RCC_OSCILLATORTYPE_HSI                   0x00000002U /*Select high speed internal*/
#define RCC_OSCILLATORTYPE_PLL                   0x00000003U /*Select high speed PLL*/

/*========================= Options for Configure PreScalar of High Speed AHB Bus=====================*/
#define RCC_AHB_PRE_DIV1	(0x00000000)	/*(0<<4) */
#define RCC_AHB_PRE_DIV2	(0x00000080)	/*(8<<4) */
#define RCC_AHB_PRE_DIV4	(0x00000090)	/*(9<<4) */
#define RCC_AHB_PRE_DIV8	(0x000000A0)	/*(10<<4)*/
#define RCC_AHB_PRE_DIV16	(0x000000B0)	/*(11<<4)*/
#define RCC_AHB_PRE_DIV64	(0x000000C0)	/*(12<<4)*/
#define RCC_AHB_PRE_DIV128	(0x000000D0)	/*(13<<4)*/
#define RCC_AHB_PRE_DIV256	(0x000000E0)	/*(14<<4)*/
#define RCC_AHB_PRE_DIV512	(0x000000F0)	/*(15<<4)*/

/*=================================== The Peripherals on AHB1 bus Macros =====================================*/
#define RCC_AHB1ENR_GPIOAEN_POS                 (0U)
#define RCC_AHB1ENR_GPIOBEN_POS				    (1U)
#define RCC_AHB1ENR_GPIOCEN_POS				    (2U)
#define RCC_AHB1ENR_GPIODEN_POS			        (3U)
#define RCC_AHB1ENR_GPIOEEN_POS				    (4U)
#define RCC_AHB1ENR_GPIOHEN_POS				    (7U)
#define RCC_AHB1ENR_CRCEN_POS			        (12U)
#define RCC_AHB1ENR_DMA1EN_POS				    (21U)
#define RCC_AHB1ENR_DMA2EN_POS				    (22U)
/*=================================== The Peripherals on AHB2 bus=====================================*/
#define RCC_AHB2ENR_OTGFSEN_POS					(7U)
/*=================================== The Peripherals on APB1 bus=====================================*/
#define RCC_APB1ENR_TIM2EN_POS					(0U)
#define RCC_APB1ENR_TIM3EN_POS					(1U)
#define RCC_APB1ENR_TIM4EN_POS					(2U)
#define RCC_APB1ENR_TIM5EN_POS					(3U)
#define RCC_APB1ENR_WWDGEN_POS					(11U)
#define RCC_APB1ENR_SPI2EN_POS					(14U)
#define RCC_APB1ENR_SPI3EN_POS					(15U)
#define RCC_APB1ENR_USART2EN_POS				(17U)
#define RCC_APB1ENR_I2C1EN_POS					(21U)
#define RCC_APB1ENR_I2C2EN_POS					(22U)
#define RCC_APB1ENR_I2C3EN_POS					(23U)
#define RCC_APB1ENR_PWREN_POS					(28U)

/*=================================== The Peripherals on APB2 bus=====================================*/
#define RCC_APB2ENR_TIM1EN_POS				 	(0U)
#define RCC_APB2ENR_USART1EN_POS				(4U)
#define RCC_APB2ENR_USART6EN_POS				(5U)
#define RCC_APB2ENR_ADC1EN_POS					(8U)
#define RCC_APB2ENR_SDIOEN_POS					(11U)
#define RCC_APB2ENR_SPI1EN_POS					(12U)
#define RCC_APB2ENR_SPI4EN_POS					(13U)
#define RCC_APB2ENR_SYSCFGEN_POS				(14U)
#define RCC_APB2ENR_TIM9EN_POS					(16U)
#define RCC_APB2ENR_TIM10EN_POS					(17U)
#define RCC_APB2ENR_TIM11EN_POS					(18U)

/***************************************************************************************************/
/**************************************** Macros Functions *****************************************/
/***************************************************************************************************/

/*--------------------------- The Peripherals on AHB1 bus----------------------*/
#define RCC_GPIOA_AHB1_CLK_ENABLE()     (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_POS))
#define RCC_GPIOB_AHB1_CLK_ENABLE()     (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_POS))
#define RCC_GPIOC_AHB1_CLK_ENABLE()     (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_POS))
#define RCC_GPIOD_AHB1_CLK_ENABLE()     (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_POS))
#define RCC_GPIOE_AHB1_CLK_ENABLE()     (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN_POS))
#define RCC_GPIOH_AHB1_CLK_ENABLE()     (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN_POS))
#define RCC_CRC_AHB1_CLK_ENABLE()       (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN_POS))
#define RCC_DMA1_AHB1_CLK_ENABLE()      (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN_POS))
#define RCC_DMA2_AHB1_CLK_ENABLE()      (SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN_POS)) 

#define RCC_GPIOA_AHB1_CLK_DISABLE()    (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_POS))
#define RCC_GPIOB_AHB1_CLK_DISABLE()    (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_POS))
#define RCC_GPIOC_AHB1_CLK_DISABLE()    (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_POS))
#define RCC_GPIOD_AHB1_CLK_DISABLE()    (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_POS))
#define RCC_GPIOE_AHB1_CLK_DISABLE()    (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN_POS))
#define RCC_GPIOH_AHB1_CLK_DISABLE()    (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN_POS))
#define RCC_CRC_AHB1_CLK_DISABLE()      (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN_POS))
#define RCC_DMA1_AHB1_CLK_DISABLE()     (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN_POS))
#define RCC_DMA2_AHB1_CLK_DISABLE()     (CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN_POS)) 

/*---------------------The Peripherals on AHB2 bus---------------------------*/
#define RCC_OTGFS_AHB2_CLK_ENABLE()     (SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN_POS)) 

#define RCC_OTGFS_AHB2_CLK_DISABLE()    (CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN_POS))

/*----------------------- The Peripherals on APB1 bus------------------------*/
#define RCC_TIM2_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN_POS))
#define RCC_TIM3_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN_POS))
#define RCC_TIM4_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN_POS))
#define RCC_TIM5_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN_POS))
#define RCC_WWDG_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN_POS))
#define RCC_SPI2_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN_POS))
#define RCC_SPI3_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN_POS))
#define RCC_USART2_APB1_CLK_ENABLE()    (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN_POS))
#define RCC_I2C1_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN_POS))
#define RCC_I2C2_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN_POS))
#define RCC_I2C3_APB1_CLK_ENABLE()      (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN_POS))
#define RCC_PWR_APB1_CLK_ENABLE()       (SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_POS))

#define RCC_TIM2_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN_POS))
#define RCC_TIM3_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN_POS))
#define RCC_TIM4_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN_POS))
#define RCC_TIM5_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN_POS))
#define RCC_WWDG_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN_POS))
#define RCC_SPI2_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN_POS))
#define RCC_SPI3_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN_POS))
#define RCC_USART2_APB1_CLK_DISABLE()   (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN_POS))
#define RCC_I2C1_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN_POS))
#define RCC_I2C2_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN_POS))
#define RCC_I2C3_APB1_CLK_DISABLE()     (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN_POS))
#define RCC_PWR_APB1_CLK_DISABLE()      (CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_POS))

/*---------------------- The Peripherals on APB2 bus-------------------------*/
#define RCC_TIM1_APB2_CLK_ENABLE()      (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN_POS))
#define RCC_USART1_APB2_CLK_ENABLE()    (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN_POS))
#define RCC_USART6_APB2_CLK_ENABLE()    (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN_POS))
#define RCC_ADC1_APB2_CLK_ENABLE()      (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN_POS))
#define RCC_SDIO_APB2_CLK_ENABLE()      (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SDIOEN_POS))
#define RCC_SPI1_APB2_CLK_ENABLE()      (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN_POS))
#define RCC_SPI4_APB2_CLK_ENABLE()      (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI4EN_POS))
#define RCC_SYSCF_APB2_CLK_ENABLE()     (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN_POS))
#define RCC_TIM9_APB2_CLK_ENABLE()      (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN_POS))
#define RCC_TIM10_APB2_CLK_ENABLE()     (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN_POS))
#define RCC_TIM11_APB2_CLK_ENABLE()     (SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN_POS))

#define RCC_TIM1_APB2_CLK_DISABLE()     (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN_POS))
#define RCC_USART1_APB2_CLK_DISABLE()   (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN_POS))
#define RCC_USART6_APB2_CLK_DISABLE()   (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN_POS))
#define RCC_ADC1_APB2_CLK_DISABLE()     (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN_POS))
#define RCC_SDIO_APB2_CLK_DISABLE()     (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SDIOEN_POS))
#define RCC_SPI1_APB2_CLK_DISABLE()     (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN_POS))
#define RCC_SPI4_APB2_CLK_DISABLE()     (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI4EN_POS))
#define RCC_SYSCF_APB2_CLK_DISABLE()    (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN_POS))
#define RCC_TIM9_APB2_CLK_DISABLE()     (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN_POS))
#define RCC_TIM10_APB2_CLK_DISABLE()    (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN_POS))
#define RCC_TIM11_APB2_CLK_DISABLE()    (CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN_POS))


/**************************************** Data types *****************************************/
typedef struct {
    volatile uint32_t CR;            // RCC clock control register
    volatile uint32_t PLLCFGR;       // RCC PLL configuration register
    volatile uint32_t CFGR;          // RCC clock configuration register
    volatile uint32_t CIR;           // RCC clock interrupt register
    volatile uint32_t AHB1RSTR;      // RCC AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR;      // RCC AHB2 peripheral reset register
    uint32_t          RESERVED0[2];  // Reserved
    volatile uint32_t APB1RSTR;      // RCC APB1 peripheral reset register
    volatile uint32_t APB2RSTR;      // RCC APB2 peripheral reset register
    uint32_t          RESERVED1[2];  // Reserved
    volatile uint32_t AHB1ENR;       // RCC AHB1 peripheral clock enable register
    volatile uint32_t AHB2ENR;       // RCC AHB2 peripheral clock enable register
    uint32_t          RESERVED2[2];  // Reserved
    volatile uint32_t APB1ENR;       // RCC APB1 peripheral clock enable register
    volatile uint32_t APB2ENR;       // RCC APB2 peripheral clock enable register
    uint32_t          RESERVED3[2];  // Reserved
    volatile uint32_t AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode register
    volatile uint32_t AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode register
    uint32_t          RESERVED4[2];  // Reserved
    volatile uint32_t APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode register
    volatile uint32_t APB2LPENR;     // RCC APB2 peripheral clock enable in low power mode register
    uint32_t          RESERVED5[2];  // Reserved
    volatile uint32_t BDCR;          // RCC Backup domain control register
    volatile uint32_t CSR;           // RCC clock control & status register
    uint32_t          RESERVED6[2];  // Reserved
    volatile uint32_t SSCGR;         // RCC spread spectrum clock generation register
    volatile uint32_t PLLI2SCFGR;    // RCC PLLI2S configuration register
    uint32_t          RESERVED7;     // Reserved
    volatile uint32_t DCKCFGR;       //RCC Dedicated Clocks Configuration Register
} RCC_TypeDef;

typedef struct {
	 uint32_t PLLM;   /*PLLM Value must be between 2 and 63*/
	 uint32_t PLLP;   /*PLLP must be RCC_PLLP_DIV2, RCC_PLLP_DIV4, RCC_PLLP_DIV6, or RCC_PLLP_DIV8*/
	 uint32_t PLLN;   /*PLLN Value must be between 192 and 432*/
	 uint32_t PLLSRC; /*PLLSRC must be RCC_PLLSRC_HSI or RCC_PLLSRC_HSE*/
	 uint32_t PLLQ;   /*PLLQ Value must be between 2 and 15*/
}PLLCFG_t;

typedef struct {
	 uint32_t AHBCLKDIVIDER;   
	 uint32_t APB1CLKDIVIDER;   
     uint32_t APB2CLKDIVIDER;
     uint32_t SYSTECCLKDIVIDER;
     
}RCC_prescalers;

typedef struct {
    uint32_t SystemClockSourse; /*!< the System Clock Sourse to be configured.
                                         this parameter can be a value of @ref RCC_SYSCLK_TYPE */
    PLLCFG_t PLL_Param;

}RCC_Cfg;

/**************************************** Software Interfaces Declarations *****************************************/
Status_t RCC_OscConfig(RCC_Cfg *RCC_InitStruct);

#endif /*_STM32F4xx_HAL_RCC_H_*/