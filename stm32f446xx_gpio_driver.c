/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 22 Nov 2022
 *      Author: Anees
 */
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

/*******************************************************************************
 * @fn			-	GPIO_PeriCLKControl
 *
 * brief		-	Enables or Disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_PeriCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PR_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PR_CLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PR_CLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PR_CLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PR_CLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PR_CLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PR_CLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PR_CLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PR_CLK_DI();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_PR_CLK_DI();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_PR_CLK_DI();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_PR_CLK_DI();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_PR_CLK_DI();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_PR_CLK_DI();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_PR_CLK_DI();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_PR_CLK_DI();
			}
	}
}

/*******************************************************************************
 * @fn			-	GPIO_Init
 *
 * brief		-	Initialize pin configuration
 *
 * @param[in]		-	GPIO handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//Configure MODE
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	}else
	{
		//configure FT,RT or RFT register
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//set falling edge at corresponding pin
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear rising edge at corresponding pin
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//set rising edge at corresponding pin
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear falling edge at corresponding pin
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//set falling edge at corresponding pin
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//set rising edge at corresponding pin
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);

		// enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	//Configure Speed
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	//Configure pupd
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	//Configure OT type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	//Configure Alt Funt
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFRL[temp1] &= ~( 0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFRL[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/*******************************************************************************
 * @fn			-	GPIO_DeInit
 *
 * brief		-	De-Initialize pin configuration
 *
 * @param[in]		-	Base address of GPIO peripheral
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RST();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RST();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RST();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RST();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RST();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RST();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RST();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RST();
	}
}

/*******************************************************************************
 * @fn			-	GPIO_ReadFromIPin
 *
 * brief		-	Read data from GPIO pin
 *
 * @param[in]		-	Base address of GPIO peripheral
 * @param[in]		-	Pin Number
 *
 * @return		-	input value from GPIO pin (0 or 1)
 *
 * @note		-	none
 */
uint8_t GPIO_ReadFromIPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*******************************************************************************
 * @fn			-	GPIO_ReadFromIPort
 *
 * brief		-	Read data from GPIO port
 *
 * @param[in]		-	Base address of GPIO peripheral
 *
 * @return		-	input value from GPIO port (0 to 15)
 *
 * @note		-	none
 */
uint16_t GPIO_ReadFromIPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*******************************************************************************
 * @fn			-	GPIO_WriteToOPin
 *
 * brief		-	write data on GPIO pin
 *
 * @param[in]		-	Base address of GPIO peripheral
 * @param[in]		-	GPIO pin number
 * @param[in]		-	value to write(0 or 1)
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_WriteToOPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/*******************************************************************************
 * @fn			-	GPIO_WriteToOPort
 *
 * brief		-	write data on GPIO port
 *
 * @param[in]		-	Base address of GPIO peripheral
 * @param[in]		-	value to write(0 to 15)
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_WriteToOPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*******************************************************************************
 * @fn			-	GPIO_TogglePin
 *
 * brief		-	toggles the value on GPIO pin
 *
 * @param[in]		-	Base address of GPIO peripheral
 * @param[in]		-	GPIO pin number
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*******************************************************************************
 * @fn			-	GPIO_IRQITConfig
 *
 * brief		-	IRQ configuration for GPIO peripheral
 *
 * @param[in]		-	IRQ number
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*******************************************************************************
 * @fn			-	GPIO_IRQPriorityConfig
 *
 * brief		-	IRQ priority configuration for GPIO peripheral
 *
 * @param[in]		-	IRQ number
 * @param[in]		-	IRQ priority value
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//find our the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_sec = IRQNumber % 4;
	uint8_t shift_amount = (iprx_sec * 8) + (8 - NO_BITS_IMP);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*******************************************************************************
 * @fn			-	GPIO_IRQHandling
 *
 * brief		-	IRQ handling for GPIO peripheral
 *
 * @param[in]		-	GPIO pin number
 *
 * @return		-	none
 *
 * @note		-	none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
