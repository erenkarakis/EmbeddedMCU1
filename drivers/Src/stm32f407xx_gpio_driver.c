/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 19, 2024
 *      Author: Taha Eren Karakış
 */

#include "stm32f407xx_gpio_driver.h"

/* ***********************************************************
 * @fn - GPIO_ClockControl
 * 
 * @brief - This function enables or disables peripheral clock for given GPIO port
 * 
 * @param pGPIOx Base address of the GPIO peripheral
 * @param EnOrDi ENABLE or DISABLE macros
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_CLK_EN();
        }else if (pGPIOx == GPIOB)
        {
            GPIOB_CLK_EN();
        }else if(pGPIOx == GPIOC)
        {
            GPIOC_CLK_EN();
        }else if(pGPIOx == GPIOD)
        {
            GPIOD_CLK_EN();
        }else if(pGPIOx == GPIOE)
        {
            GPIOE_CLK_EN();
        }else if(pGPIOx == GPIOF)
        {
            GPIOF_CLK_EN();
        }else if(pGPIOx == GPIOF)
        {
            GPIOF_CLK_EN();
        }else if(pGPIOx == GPIOG)
        {
            GPIOG_CLK_EN();
        }else if(pGPIOx == GPIOH)
        {
            GPIOH_CLK_EN();
        }
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_CLK_EN();
        }else if (pGPIOx == GPIOB)
        {
            GPIOB_CLK_DI();
        }else if(pGPIOx == GPIOC)
        {
            GPIOC_CLK_DI();
        }else if(pGPIOx == GPIOD)
        {
            GPIOD_CLK_DI();
        }else if(pGPIOx == GPIOE)
        {
            GPIOE_CLK_DI();
        }else if(pGPIOx == GPIOF)
        {
            GPIOF_CLK_DI();
        }else if(pGPIOx == GPIOF)
        {
            GPIOF_CLK_DI();
        }else if(pGPIOx == GPIOG)
        {
            GPIOG_CLK_DI();
        }else if(pGPIOx == GPIOH)
        {
            GPIOH_CLK_DI();
        }
    }

}

/* ***********************************************************
 * @fn - GPIO_Init
 * 
 * @brief - This function initializes the given GPIO pin
 * 
 * @param pGPIOHandle GPIO_Handle_t pointer type pin handler
 * 
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    // Configure the mode of GPIO pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->MODER |= temp;

    }else
    {
        // Interrupt mode
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // Configure the FTSR (Falling Trigger Set Register)
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the corresponding RTSR bit
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        }else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // Configure the RTSR (Rising Trigger Set Register)
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the corresponding RTSR bit
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        }else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // Configure the both FTSR and RTSR
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // Configure the GPIO port selection in SYSCFG_EXTICR (System Config. External Interrupt Control Reg.)
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_CLK_EN();
        SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);

        // Enable the EXTI interrupt delivery using IMR (Interrupt Mask Register) 
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    }

    temp = 0;

    // Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    // Configure the pull-up/pull-down resistor settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;

    // Configure the output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    // Configure the alternate functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

    }

}

/* ***********************************************************
 * @fn - GPIO_DeInit
 * 
 * @brief - This function de-initializes the given GPIO pin
 * 
 * @param pGPIOx Base address of the GPIO peripheral
 * 
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
        {
            GPIOA_REG_RESET();
        }else if (pGPIOx == GPIOB)
        {
            GPIOB_REG_RESET();
        }else if(pGPIOx == GPIOC)
        {
            GPIOC_REG_RESET();
        }else if(pGPIOx == GPIOD)
        {
            GPIOD_REG_RESET();
        }else if(pGPIOx == GPIOE)
        {
            GPIOE_REG_RESET();
        }else if(pGPIOx == GPIOF)
        {
            GPIOF_REG_RESET();
        }else if(pGPIOx == GPIOF)
        {
            GPIOF_REG_RESET();
        }else if(pGPIOx == GPIOG)
        {
            GPIOG_REG_RESET();
        }else if(pGPIOx == GPIOH)
        {
            GPIOH_REG_RESET();
        }

}

/* ***********************************************************
 * @fn - GPIO_ReadFromInputPin
 * 
 * @brief - This function reads the data from given GPIO pin
 * 
 * @param pGPIOx Base address of the GPIO peripheral
 * @param PinNumber The pin which is used for data reading
 * 
 * @return - 0 or 1
 * 
 * @note - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

    return value;
}

/* ***********************************************************
 * @fn - GPIO_ReadFromInputPort
 * 
 * @brief - This function reads the data from given GPIO port
 * 
 * @param pGPIOx Base address of the GPIO peripheral
 * 
 * 
 * @return - none
 * 
 * @note - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint8_t)pGPIOx->IDR;

    return value;
}

/* ***********************************************************
 * @fn - GPIO_WriteToOutputPin
 * 
 * @brief - This function writes the data to given GPIO pin
 * 
 * @param pGPIOx Base address of the GPIO peripheral
 * @param PinNumber The pin which is used for data writing
 * @param Value Value to be passed to the pin
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << PinNumber);
    }else
    {
         pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/* ***********************************************************
 * @fn - GPIO_WriteToOutputPin
 * 
 * @brief - This function writes the data to given GPIO port
 * 
 * @param pGPIOx Base address of the GPIO peripheral
 * @param Value Value to be passed to the port
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR |= Value;
}

/* ***********************************************************
 * @fn - GPIO_ToggleOutputPin
 * 
 * @brief - Toggles on or off the pin
 * 
 * @param pGPIOx Base address of the GPIO peripheral
 * @param PinNumber Pin number
 * 
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration and ISR handling
 */

/* ***********************************************************
 * @fn - GPIO_EnableIRQ
 * 
 * @brief - This function enbales the interrupt register
 * 
 * @param IRQNumber Interrupt request (IRQ) number to be enabled
 *
 * 
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_EnableIRQ(uint8_t IRQNumber)
{
    if(IRQNumber <= 31)
    {
        // Program ISER0 (Interrupt Set Enable Register)
        *NVIC_ISER0 |= (1 << IRQNumber);

    }else if(IRQNumber > 31 && IRQNumber < 64)
    {
        // Program ISER1 (Interrupt Set Enable Register)
        *NVIC_ISER1 |= (1 << IRQNumber % 32);

    }else if(IRQNumber >= 64 && IRQNumber < 96)
    {
        // Program ISER2 (Interrupt Set Enable Register)
        *NVIC_ISER2 |= (1 << IRQNumber % 64);
    }
}

/* ***********************************************************
 * @fn - GPIO_DisableIRQ
 * 
 * @brief - This function disables the interrupt register
 * 
 * @param IRQNumber Interrupt request (IRQ) number to be disabled
 * 
 * 
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_DisableIRQ(uint8_t IRQNumber)
{
    if(IRQNumber <= 31)
    {
        // Program ICER0 (Interrupt Clear Enable Register)
        *NVIC_ICER0 |= (1 << IRQNumber);

    }else if(IRQNumber > 31 && IRQNumber < 64)
    {
        // Program ICER1 (Interrupt Clear Enable Register)
        *NVIC_ICER1 |= (1 << IRQNumber % 32);

    }else if(IRQNumber >= 64 && IRQNumber < 96)
    {
        // Program ICER2 (Interrupt Clear Enable Register)
        *NVIC_ICER2 |= (1 << IRQNumber % 64);

    }
}

/* ***********************************************************
 * @fn - GPIO_IRQPriorityConfig
 * 
 * @brief - N/A
 * 
 * @param IRQNumber 
 * @param IRQPriority Interrupt request (IRQ) number to be enabled or disabled
 *
 * 
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_SetIRQPriority(uint8_t IRQNumber, uint8_t IRQPriority)
{
    // Find out the IPR (Interrupt Priority Reg.)
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);

}

/* ***********************************************************
 * @fn - GPIO_IRQHandling
 * 
 * @brief - N/A
 * 
 * @param PinNumber Interrupt request (IRQ) number to be enabled or disabled
 * 
 * 
 * 
 * @return - none
 * 
 * @note - none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    // Clear the EXTI PR (Pending Register) corresponding to the pin number
    if(EXTI->PR & (1 << PinNumber))
    {
        EXTI->PR |= (1 << PinNumber);
    }

}
