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


uint8_t rcv_buf [32];

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
	I2C1handle.I2C_Config.I2C_DeviceAddress = 0X61;
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
	uint8_t command_code;
	command_code = 0x51;
	uint8_t len;
	Button_Init();
	I2C1_GPIO_Init();
	I2C1_Init();
	I2C_PeripheralControl(I2C1handle.pI2Cx, ENABLE);

	I2C_ManageAcking(I2C1handle.pI2Cx, I2C_ACK_ENABLE);


	while(1)
	{
		while(!(GPIO_ReadfromInputPin(GPIOA, GPIO_PIN_NO_0)));

		delay(500000/2);

		//send 1 byte command command for length (write)
		I2C_MasterSendData(&I2C1handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR );

		//receive 1 byte of length info (read)
		I2C_MasterReceiveData(&I2C1handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		//send 1 byte command command for data (write)
		command_code = 0x52;
		I2C_MasterSendData(&I2C1handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		//receive len bytes of data (read)
		I2C_MasterReceiveData(&I2C1handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR );

	}

}
