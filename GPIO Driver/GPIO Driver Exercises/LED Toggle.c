/*
 * 001Toggle_LED.c
 *
 *  Created on: Mar 15, 2024
 *      Author: ImadF
 */
#include <stdint.h>
#include "stm32f407xx.h"

void delay(uint32_t count)
{
	for(uint32_t i = 0; i< count; i++);
}

int main(void)
{

	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	GpioLed.pGPIOx = GPIOD;
	GpioButton.pGPIOx = GPIOA;

	/*for LED*/
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	/*for button */
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriCLockCOntrol(GPIOA, ENABLE);
	GPIO_Init(&GpioButton);

	GPIO_PeriCLockCOntrol(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
	while(1)
	{
		volatile uint8_t value = GPIO_ReadfromInputPin(GPIOA, GPIO_PIN_NO_0 );
		if(value)
		{
			delay(500000/2); // to account for button debouncing
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
		
	}



}

