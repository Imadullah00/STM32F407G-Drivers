/*
 * 002ExternalButton.c
 *
 *  Created on: Mar 15, 2024
 *      Author: ImadF
 */


#include<stdint.h>
#include"stm32f407xx.h"

void delay(uint32_t count)
{
	for(uint32_t i = 0; i< count; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton_ext;

	GpioLed.pGPIOx = GPIOD;
	GpioButton_ext.pGPIOx = GPIOA;

		/*for LED*/
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		/*for button */
	GpioButton_ext.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton_ext.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton_ext.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton_ext.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioButton_ext.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriCLockCOntrol(GPIOD, ENABLE);

	GPIO_Init(&GpioButton_ext);
	GPIO_Init(&GpioLed);

	GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_RESET);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);

}

void EXTI0_IRQHandler(void)
	{
		delay(500000/2);
		GPIO_IRQHandling(GPIO_PIN_NO_0);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	}
