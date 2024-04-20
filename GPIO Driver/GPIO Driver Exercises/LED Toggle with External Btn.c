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
	GpioButton_ext.pGPIOx = GPIOB;

		/*for LED*/
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		/*for button */
	GpioButton_ext.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;  //external button pin
	GpioButton_ext.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton_ext.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton_ext.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // using pull up resistors
	GpioButton_ext.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriCLockCOntrol(GPIOB, ENABLE);
	GPIO_Init(&GpioButton_ext);

	GPIO_PeriCLockCOntrol(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
	while(1)
	{
		volatile uint8_t value = GPIO_ReadfromInputPin(GPIOB, GPIO_PIN_NO_12 );
		if(!value)
		{
			delay(500000/2);
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}

}
