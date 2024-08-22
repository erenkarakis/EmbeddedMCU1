/*
 * stm32f407xx.h
 *
 *  Created on: Aug 19, 2024
 *      Author: Taha Eren Karakış
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*********************************************************** START Processor Specific Details START ***********************************************************/
/*
 * Arm Cortex Mx Processor NVIC (Nested Vectored Interrupt Controller) ISERx register addresses
 */

#define NVIC_ISER0 ((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t*) 0xE000E10C)

/*
 * Arm Cortex Mx Processor NVIC (Nested Vectored Interrupt Controller) ICERx register addresses
 */

#define NVIC_ICER0 ((__vo uint32_t*) 0XE000E180)
#define NVIC_ICER1 ((__vo uint32_t*) 0XE000E184)
#define NVIC_ICER2 ((__vo uint32_t*) 0XE000E188)
#define NVIC_ICER3 ((__vo uint32_t*) 0XE000E18C)

/*
 * Arm Cortex Mx Processor Priority Register address
 */

#define NVIC_PR_BASEADDR ((__vo uint32_t*) 0xE000E400)

/*********************************************************** END Processor Specific Details END ***********************************************************/

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U //112KB
#define SRAM2_BASEADDR 0x20001C00U
#define ROM_BASEADDR 0x1FFF0000U
#define SRAM SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE 0x40000000U
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR (AHB1PERIPH_BASE + 0x2800)

#define RCC_BASEADDR (AHB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define SPI1_BASEADDR (APB2PERIPH_BASE + 0x3000)
#define EXTI_BASEADDR (APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0x3800)

#define USART1_BASEADDR (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0x1400)


/***************************************** Peripheral Register Definition Structures *****************************************/
/*
 * Note: Registers of a peripheral are specific to MCU
 * Please check your device's "Referance Manuel"
 */

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
   __vo uint32_t MODER;
   __vo uint32_t OTYPER;
   __vo uint32_t OSPEEDR;
   __vo uint32_t PUPDR;
   __vo uint32_t IDR;
   __vo uint32_t ODR;
   __vo uint32_t BSSR;
   __vo uint32_t LCKR;
   __vo uint32_t AFR[2];

}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
    uint32_t RESERVED2;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR;
 
}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
    __vo uint32_t IMR; // Interrupt mask on line x
    __vo uint32_t EMR; // Event mask on line x
    __vo uint32_t RTSR; // Rising trigger event configuration bit of line x
    __vo uint32_t FTSR; // Falling trigger event configuration bit of line x
    __vo uint32_t SWIER; // Software Interrupt on line x
    __vo uint32_t PR; //  Pending bit

}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
    __vo uint32_t MEMRMP;
    __vo uint32_t PMC;
    __vo uint32_t EXTICR[4];
    uint32_t RESERVED[2];
    __vo uint32_t CMPCR;

}SYSCFG_RegDef_t;


/*
 * Peripheral definitions - Peripheral base addresses typecasted to xxx_RegDef_t
 */

#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_CLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN() (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_CLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DI() (RCC->AHB1ENR &= ~(1 << 8))


/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_CLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_CLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI() (RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_CLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_CLK_EN() (RCC->APB2ENR |= (1 << 13))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_CLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_CLK_DI() (RCC->APB2ENR &= ~(1 << 13))


/*
 * Clock enable macros for USARTx/UARTx peripherals
 */

#define USART1_CLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_CLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_CLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_EN() (RCC->APB2ENR |= (1 << 5))

/*
 * Clock disable macros for USARTx/UARTx peripherals
 */

#define USART1_CLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_CLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_CLK_DI() (RCC->APB1ENR &= ~(1 << 20))
#define USART6_CLK_DI() (RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_CLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_CLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/* ***********************************************************
 * @fn - GPIO_BASEADDR_TO_CODE
 * 
 * @brief - This function initializes the given GPIO pin
 * 
 * @param x GPIO port base address
 * 
 * 
 * @return - A code between 0 - 8 for a given GPIO base address
 * 
 * @note - none
 */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0: \
                                 (x == GPIOB) ? 1: \
                                 (x == GPIOC) ? 2: \
                                 (x == GPIOD) ? 3: \
                                 (x == GPIOD) ? 4: \
                                 (x == GPIOE) ? 5: \
                                 (x == GPIOF) ? 6: \
                                 (x == GPIOG) ? 7: \
                                 (x == GPIOH) ? 8: 0)


/*
 * IRQ (Intterupt Request) Number of STM32F407x MCU
 * NOTE: Update these macros with valid values according to your MCU
 */

#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

/*
 * Some generic macros
 */

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define HIGH ENABLE
#define LOW DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET


#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
