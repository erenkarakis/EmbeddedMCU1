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
 * Possible IRQ Priority numbers
 */

#define NVIC_IRQ_PRI0 0 
#define NVIC_IRQ_PRI1 1
#define NVIC_IRQ_PRI2 2
#define NVIC_IRQ_PRI3 3
#define NVIC_IRQ_PRI4 4
#define NVIC_IRQ_PRI5 5
#define NVIC_IRQ_PRI6 6
#define NVIC_IRQ_PRI7 7
#define NVIC_IRQ_PRI8 8 
#define NVIC_IRQ_PRI9 9
#define NVIC_IRQ_PRI10 10
#define NVIC_IRQ_PRI11 11 
#define NVIC_IRQ_PRI12 12
#define NVIC_IRQ_PRI13 13
#define NVIC_IRQ_PRI14 14
#define NVIC_IRQ_PRI15 15

/*
 * Arm Cortex Mx Processor Priority Register address
 */

#define NVIC_PR_BASEADDR ((__vo uint32_t*) 0xE000E400)

/*
 * Arm Cortex Mx Processor number of priority bits implemented in Priprity Register
 */
#define NO_PR_BITS_IMPLEMENTED 4

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
   __vo uint32_t MODER; // GPIO port mode register (GPIO_MODER) - These bits are written by software to configure the I/O direction mode
   __vo uint32_t OTYPER; // GPIO port output type register (GPIO_OTYPER) - These bits are written by software to configure the output type of the I/O port
   __vo uint32_t OSPEEDR; // GPIO port output speed register (GPIO_OSPEEDR) - These bits are written by software to configure the I/O output speed
   __vo uint32_t PUPDR; // GPIO port pull-up/pull-down register (GPIO_PUPDR) - These bits are written by software to configure the I/O pull-up or pull-down
   __vo uint32_t IDR; // GPIO port input data register (GPIO_IDR) - These bits are read-only and can be accessed in word mode only. They contain the input value of the corresponding I/O port
   __vo uint32_t ODR; // GPIO port output data register (GPIO_ODR) - These bits can be read and written by software
   __vo uint32_t BSSR; // GPIO port bit set/reset register (GPIO_BSSR) - Check referance manuel for details
   __vo uint32_t LCKR; // GPIO port configuration lock register (GPIO_LCKR) - Check referance manuel for details 
   __vo uint32_t AFR[2]; // GPIO alternate function low register - These bits are written by software to configure alternate function I/Os

}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
    __vo uint32_t CR; // RCC clock control register (RCC_CR)
    __vo uint32_t PLLCFGR; // RCC PLL configuration register (RCC_PLLCFGR) - This register is used to configure the PLL clock outputs according to the formulas (Check referance manuel for details)
    __vo uint32_t CFGR; // RCC clock configuration register (RCC_CFGR)
    __vo uint32_t CIR; // RCC clock interrupt register (RCC_CIR)
    __vo uint32_t AHB1RSTR; // RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
    __vo uint32_t AHB2RSTR; // RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
    __vo uint32_t AHB3RSTR; // RCC AHB3 peripheral reset register (RCC_AHB3RSTR)
    uint32_t RESERVED0;
    __vo uint32_t APB1RSTR; // RCC APB1 peripheral reset register (RCC_APB1RSTR)
    __vo uint32_t APB2RSTR; // RCC APB2 peripheral reset register (RCC_APB2RSTR)
    uint32_t RESERVED1[2];
    __vo uint32_t AHB1ENR; // RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
    __vo uint32_t AHB2ENR; // RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)
    __vo uint32_t AHB3ENR; // RCC AHB3 peripheral clock enable register (RCC_AHB3ENR)
    uint32_t RESERVED2;
    __vo uint32_t APB1ENR; // RCC APB1 peripheral clock enable register (RCC_APB1ENR)
    __vo uint32_t APB2ENR; // RCC APB2 peripheral clock enable register (RCC_APB2ENR)
    uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR; // RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR)
    __vo uint32_t AHB2LPENR; // RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR)
    __vo uint32_t AHB3LPENR; // RCC AHB3 peripheral clock enable in low power mode register (RCC_AHB3LPENR)
    uint32_t RESERVED4;
    __vo uint32_t APB1LPENR; // RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR)
    __vo uint32_t APB2LPENR; // RCC APB2 peripheral clock enable in low power mode register (RCC_APB2LPENR)
    uint32_t RESERVED5[2];
    __vo uint32_t BDCR; // RCC Backup domain control register (RCC_BDCR)
    __vo uint32_t CSR; // RCC clock control & status register (RCC_CSR)
    uint32_t RESERVED6[2];
    __vo uint32_t SSCGR; // RCC spread spectrum clock generation register (RCC_SSCGR)
    __vo uint32_t PLLI2SCFGR;  // RCC PLLI2S configuration register (RCC_PLLI2SCFGR)
    __vo uint32_t PLLSAICFGR; // RCC PLL configuration register (RCC_PLLSAICFGR)
    __vo uint32_t DCKCFGR; // RCC Dedicated Clock Configuration Register (RCC_DCKCFGR)
 
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
    __vo uint32_t PR; // Pending bit

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
