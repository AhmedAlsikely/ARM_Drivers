#ifndef _STM32F4XX_PERIPHERAL_DEFS_H_
#define _STM32F4XX_PERIPHERAL_DEFS_H_

#include "Std_CFG.h"

// RCC_CR register bits
#define RCC_CR_HSION_Pos        0U
#define RCC_CR_HSION_Msk        (1UL << RCC_CR_HSION_Pos)
#define RCC_CR_HSIRDY_Pos       1U
#define RCC_CR_HSIRDY_Msk       (1UL << RCC_CR_HSIRDY_Pos)
#define RCC_CR_HSEON_Pos        16U
#define RCC_CR_HSEON_Msk        (1UL << RCC_CR_HSEON_Pos)
#define RCC_CR_HSERDY_Pos       17U
#define RCC_CR_HSERDY_Msk       (1UL << RCC_CR_HSEBYP_Pos)
#define RCC_CR_HSEBYP_Pos       18U
#define RCC_CR_HSEBYP_Msk       (1UL << RCC_CR_HSEBYP_Pos)
#define RCC_CR_CSSON_Pos        19U
#define RCC_CR_CSSON_Msk        (1UL << RCC_CR_CSSON_Pos)
#define RCC_CR_PLLON_Pos        24U
#define RCC_CR_PLLON_Msk        (1UL << RCC_CR_PLLON_Pos)
#define RCC_CR_PLLRDY_Pos       25U
#define RCC_CR_PLLRDY_Msk       (1UL << RCC_CR_PLLRDY_Pos)

// RCC_PLLCFGR register bits
#define RCC_PLLCFGR_PLLM_Pos        0
#define RCC_PLLCFGR_PLLM_Msk        (0xFUL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLN_Pos        6
#define RCC_PLLCFGR_PLLN_Msk        (0x1FFUL << RCC_PLLCFGR_PLLN_Pos)
#define RCC_PLLCFGR_PLLP_Pos        16
#define RCC_PLLCFGR_PLLP_Msk        (0x3UL << RCC_PLLCFGR_PLLP_Pos)
#define RCC_PLLCFGR_PLLQ_Pos        24
#define RCC_PLLCFGR_PLLQ_Msk        (0xFUL << RCC_PLLCFGR_PLLQ_Pos)
#define RCC_PLLCFGR_PLLSRC_Pos      22
#define RCC_PLLCFGR_PLLSRC_Msk      (1UL << RCC_PLLCFGR_PLLSRC_Pos)

// RCC_CFGR register bits
#define RCC_CFGR_SW_Pos             0
#define RCC_CFGR_SW_Msk             (3UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SWS_Pos            2
#define RCC_CFGR_SWS_Msk            (3UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_HPRE_Pos           4
#define RCC_CFGR_HPRE_Msk           (0xFUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_PPRE1_Pos          10
#define RCC_CFGR_PPRE1_Msk          (0x7UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE2_Pos          13
#define RCC_CFGR_PPRE2_Msk          (0x7UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_RTCPRE_Pos         16
#define RCC_CFGR_RTCPRE_Msk         (0x1FUL << RCC_CFGR_RTCPRE_Pos)
#define RCC_CFGR_MCO1_Pos           21
#define RCC_CFGR_MCO1_Msk           (0x3UL << RCC_CFGR_MCO1_Pos)
#define RCC_CFGR_I2SSRC_Pos         23
#define RCC_CFGR_I2SSRC_Msk         (0x1UL << RCC_CFGR_I2SSRC_Pos)
#define RCC_CFGR_MCO1PRE_Pos        24
#define RCC_CFGR_MCO1PRE_Msk        (0x7UL << RCC_CFGR_MCO1PRE_Pos)
#define RCC_CFGR_MCO2PRE_Pos        27
#define RCC_CFGR_MCO2PRE_Msk        (0x7UL << RCC_CFGR_MCO2PRE_Pos)
#define RCC_CFGR_MCO2_Pos           30
#define RCC_CFGR_MCO2_Msk           (0x3UL << RCC_CFGR_MCO2_Pos)

// RCC_CIR register bits
#define RCC_CIR_LSIRDYF_Pos         0
#define RCC_CIR_LSERDYF_Pos         1
#define RCC_CIR_HSIRDYF_Pos         2
#define RCC_CIR_HSERDYF_Pos         3
#define RCC_CIR_PLLRDYF_Pos         4
#define RCC_CIR_PLLI2SRDYF_Pos      5
#define RCC_CIR_CSSF_Pos            6


/*=================================== The Peripherals on AHB1 bus Macros =====================================*/
#define RCC_AHB_GPIOA_POS                 (0U)
#define RCC_AHB_GPIOB_POS				    (1U)
#define RCC_AHB_GPIOC_POS				    (2U)
#define RCC_AHB_GPIOD_POS			        (3U)
#define RCC_AHB_GPIOE_POS				    (4U)
#define RCC_AHB_GPIOH_POS				    (7U)
#define RCC_AHB_CRC_POS			        (12U)
#define RCC_AHB_DMA1_POS				    (21U)
#define RCC_AHB_DMA2_POS				    (22U)
/*=================================== The Peripherals on AHB2 bus=====================================*/
#define RCC_AHB_OTGFS_POS					(39U) /* 7 +32 */
/*=================================== The Peripherals on APB1 bus=====================================*/
#define RCC_APB_TIM2_POS					(64U) /*0 + 64*/
#define RCC_APB_TIM3_POS					(65U) /*1 + 64*/
#define RCC_APB_TIM4_POS					(66U) /*2 + 64*/
#define RCC_APB_TIM5_POS					(67U) /*3 + 64*/
#define RCC_APB_WWDG_POS					(75U) /*11 +64*/
#define RCC_APB_SPI2_POS					(78U) /*14 + 64*/
#define RCC_APB_SPI3_POS					(79U) /*15 + 64*/
#define RCC_APB_USART2_POS				(81U) /*17 + 64*/
#define RCC_APB_I2C1_POS					(85U) /*21 + 64*/
#define RCC_APB_I2C2_POS					(86U) /*22 + 64*/
#define RCC_APB_I2C3_POS					(87U) /*23 + 64*/
#define RCC_APB_PWR_POS					(92U) /*28 + 64*/

/*=================================== The Peripherals on APB2 bus=====================================*/
#define RCC_APB_TIM1_POS				 	(128U) /*0 + 128*/ 
#define RCC_APB_USART1_POS				(132U) /*4 + 128*/ 
#define RCC_APB_USART6_POS				(133U) /*5 + 128*/ 
#define RCC_APB_ADC1_POS					(136U) /*8 + 128*/
#define RCC_APB_SDIO_POS					(139U) /*11 + 128*/
#define RCC_APB_SPI1_POS					(140U) /*12 + 128*/
#define RCC_APB_SPI4_POS					(141U) /*13 + 128*/
#define RCC_APB_SYSCFG_POS				(142U) /*14 + 128*/
#define RCC_APB_TIM9_POS					(144U) /*16 + 128*/
#define RCC_APB_TIM10_POS					(145U) /*17 + 128*/
#define RCC_APB_TIM11_POS					(146U) /*18 + 128*/



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

#endif _STM32F4XX_PERIPHERAL_DEFS_H_