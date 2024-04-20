/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Mar 14, 2024
 *      Author: ImadF
 */
#include"stm32f407xx_gpio_driver.h"

/*Peripheral Clock Setup*/

void GPIO_PeriCLockCOntrol(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}
		else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}
		else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}
		else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}
		else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}
		else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}
		else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}

		else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}
		else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_EN();
			}
	}
	else
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}
		else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}
		else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}
		else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}
		else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}
		else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}
		else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI();
			}
		else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}
		else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DI();
			}
	}


}

/*Init and Deinit*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_PeriCLockCOntrol(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;
	// Configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		// CONFIGURE SYSCFG TO RECEIVE INTERRUPPT ON Port
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();

			SYSCFG->EXTICR[temp1] = ( portcode << (temp2*4) );

	}

	// Configure the speed

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// Configure the PUPD settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//Configure the op type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;


	//Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{


		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<8)
		{
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0xF << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
			temp = 0;
		}

		else
		{
			pGPIOHandle->pGPIOx->AFR[1] &= ~(0xF << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8) );
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
			temp = 0;
		}

	}
}
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		{
			GPIOA_REGS_RESET();
		}
	else if(pGPIOx == GPIOB)
		{
		GPIOB_REGS_RESET();
				}
	else if(pGPIOx == GPIOC)
		{
		GPIOC_REGS_RESET();
		}
	else if(pGPIOx == GPIOD)
		{
		GPIOD_REGS_RESET();
		}
	else if(pGPIOx == GPIOE)
		{
		GPIOE_REGS_RESET();
		}
	else if(pGPIOx == GPIOF)
		{
		GPIOF_REGS_RESET();
		}
	else if(pGPIOx == GPIOG)
		{
		GPIOG_REGS_RESET();
		}
	else if(pGPIOx == GPIOH)
		{
		GPIOH_REGS_RESET();
		}
	else if(pGPIOx == GPIOI)
		{
		GPIOI_REGS_RESET();
		}
}

/*Data Read and Write*/

uint8_t GPIO_ReadfromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (Value<<PinNumber);

	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*IRQ COnfiguration and ISR Handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//touch ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}

		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			//touch ISER1
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}

		else if (IRQNumber >= 64 && IRQNumber <96)
		{
			//touch ISER2
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}
	}

	else
	{
		if(IRQNumber <= 31)
			{
				//touch ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
			}

		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			//touch ICER1
			*NVIC_ICER1 |= (1 << IRQNumber%32);

		}

		else if (IRQNumber >= 64 && IRQNumber <96)
		{
			//touch ICER2
			*NVIC_ICER2 |= (1 << IRQNumber%64);

		}
	}


}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t offset = IRQNumber/4;  //reg number 0-59
	uint8_t temp2 = IRQNumber % 4; // reg section

	//(*NVIC_PR_BASE_ADDR + offset*4) &= ~(0xFF << (temp2*8));
	*(NVIC_PR_BASE_ADDR + offset) |= IRQPriority << ((temp2*8)+4);

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1<<PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);

	}

}

