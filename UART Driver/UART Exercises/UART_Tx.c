/*
 * UART_Tx.c
 *
 *  Created on: Mar 27, 2024
 *      Author: ImadF
 */
#include<stdio.h>
#include<stdint.h>
#include<string.h>

#include "stm32f407xx.h"

GPIO_Handle_t GPIO_USART_handle;
USART_Handle_t USART_handle;
GPIO_Handle_t GPIO_Button;

char msg[1024] = "UART Tx testing...\n\r";

void GPIO_UASRT2_Init()
{

	GPIO_USART_handle.pGPIOx = GPIOA;

	GPIO_USART_handle.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	GPIO_USART_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_USART_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_USART_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_USART_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_USART_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Tx
	GPIO_Init(&GPIO_USART_handle);

	GPIO_USART_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;

	//Rx
	GPIO_Init(&GPIO_USART_handle);

}

void UASRT2_Init()
{

	USART_handle.pUSARTx = USART2;

	USART_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART_handle);

}

void GPIO_Button_Init()
{

	GPIO_Button.pGPIOx = GPIOA;

	GPIO_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&GPIO_Button);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main()
{
	GPIO_Button_Init();
	GPIO_UASRT2_Init();
	UASRT2_Init();

	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		while(! (GPIO_ReadfromInputPin(GPIOA, GPIO_PIN_NO_0)) );

		delay();

		USART_SendData(&USART_handle, (uint8_t*) msg, strlen(msg));
	}
}

