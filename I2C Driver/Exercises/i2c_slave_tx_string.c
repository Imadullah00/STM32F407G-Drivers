/*
 * i2c_master_tx_testing.c
 *
 *  Created on: Mar 21, 2024
 *      Author: ImadF
 */

//PB6---->SCL
//PB9----> SDA

#include "stm32f407xx.h"
#include<stdio.h>
#include<string.h>

#define SLAVE_ADDR		0X68
#define MY_ADDR 		SLAVE_ADDR


uint8_t Tx_buf [32] = "STM32 Slave Testing..";

void delay(uint32_t count)
{
	for(uint32_t i = 0; i< count; i++);
}

void I2C1_GPIO_Init()
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;

	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;

	GPIO_Init(&I2CPins);
}

I2C_Handle_t I2C1handle;


void I2C1_Init()
{

	I2C1handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY2;
	I2C1handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1handle.I2C_Config.I2C_SCL_Speed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1handle);
}

void Button_Init()
{
	/*for button */
	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&GpioButton);
}

int main(void)
{
	//uint8_t command_code;
	//command_code = 0x51;
	//uint8_t len;

	Button_Init();
	I2C1_GPIO_Init();
	I2C1_Init();


	//Configure Priority
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	I2C_PeripheralControl(I2C1handle.pI2Cx, ENABLE);

	I2C_ManageAcking(I2C1handle.pI2Cx, I2C_ACK_ENABLE);

	while(1);


}

void I2C1_EV_IRQHandler(void)
{
	/* I2C1 event interrupt*/
	I2C_EV_IRQHandling(&I2C1handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1handle);
}

uint8_t command_code;
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	static uint8_t command_code = 0;
	static uint8_t cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data. Slave has to send it
		if(command_code == 0x51)
		{
			///send length info
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));
		}

		else if (command_code == 0x52)
		{
			//Send rcvbuf contents
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[cnt++]);
		}
	}

	else if(AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting or the slave to be read. Slave has to read it.
		command_code = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}

	else if(AppEv == I2C_ERROR_AF)
	{
		//This happens only during Slave Transmission
		//Master has sent a NACK.So SLAVE should understand that master doesnt want any more data.
		command_code = 0xff;
		cnt = 0;
	}

	else if(AppEv == I2C_EV_STOP)
	{
		//This happens only during Slave Reception
		//Mastr has ended the I2C  commuincation with the slave
	}
}

