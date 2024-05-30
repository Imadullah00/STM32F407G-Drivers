/*
 * ds1307.c
 *
 *  Created on: Mar 28, 2024
 *      Author: ImadF
 */
#include <stdint.h>
#include <string.h>
#include "ds1307.h"

static void ds1307_i2c_pin_config();
static void ds1307_i2c_config();
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t binry);
static uint8_t bcd_to_binary(uint8_t bcd);

I2C_Handle_t i2c_handle;

//returns 1: CH = 1: init failed
//returns 0: CH = 0: init successful
uint8_t DS1307_Init(void)
{
	 //1. initialize the I2C pins
	 ds1307_i2c_pin_config();

	 //2. initialize the I2C peripheral
	 ds1307_i2c_config();

	 //3. enable the i2c peripheral
	 I2C_PeripheralControl(DS1307_I2C, ENABLE);

	 //4. Make CLock Halt (CH) = 0
	 ds1307_write(0x00, DS1307_ADDR_SEC);

	 //5.read clock state
	 uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

	 return ((clock_state << 7) & 0x1);

}

void DS1307_SetCurrentTime(RTC_time_t *pRTC_time)
{

	uint8_t seconds, hrs = 0;

	//Convert seconds to BCD
	seconds = binary_to_bcd(pRTC_time->seconds);

	//Clear the last bit (CH)
	seconds &= ~(1 << 7);

	//Write SECONDS
	ds1307_write(seconds, DS1307_ADDR_SEC);

	//Write MINUTES
	ds1307_write(binary_to_bcd(pRTC_time->minutes) , DS1307_ADDR_MIN);

	hrs = pRTC_time->hours;

	if(pRTC_time->time_format == TIME_FORMAT_24HRS)
	{
		hrs &= ~(1 << 6);
	}
	else
	{
		hrs |= (1 << 6);
	    (pRTC_time->time_format == TIME_FORMAT_12HRS_AM) ? (hrs &= ~(1 << 5)) : (hrs |= (1 << 5));
	}

	//Write HOURS
	ds1307_write(hrs, DS1307_ADDR_HRS);


}
void DS1307_GetCurrentTime(RTC_time_t *pRTC_time)
{
	//SECONDS

	uint8_t hrs, seconds = 0;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);
	pRTC_time->seconds = bcd_to_binary(seconds);

	//MINUTES
	pRTC_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	//HOURS
	hrs = ds1307_read(DS1307_ADDR_HRS);

	if(!(hrs & (1 << 6)))
	{
		pRTC_time->time_format = TIME_FORMAT_24HRS;
		pRTC_time->hours = bcd_to_binary(hrs);
	}

	else
	{
		if(hrs & (1 << 5))
		{
			pRTC_time->time_format = TIME_FORMAT_12HRS_PM;
		}

		else
		{
			pRTC_time->time_format = TIME_FORMAT_12HRS_AM;
		}

		hrs &= (0x3 << 5);
		pRTC_time->hours = bcd_to_binary(hrs);
	}

}

void DS1307_GetCurrentDate(RTC_date_t *pRTC_date) //ERROR HERE
{
	//DAY
	pRTC_date->day = ds1307_read(DS1307_ADDR_DAY);

	//DATE
	pRTC_date->date = ds1307_read(DS1307_ADDR_DATE);

	//MONTH
	pRTC_date->month = ds1307_read(DS1307_ADDR_MONTH);

	//YEAR
	pRTC_date->year = ds1307_read(DS1307_ADDR_YEAR);
}


void DS1307_SetCurrentDate(RTC_date_t *pRTC_date)
{
	//DAY
	ds1307_write(binary_to_bcd(pRTC_date->day) , DS1307_ADDR_DAY);

	//DATE
	ds1307_write(binary_to_bcd(pRTC_date->date) , DS1307_ADDR_DATE);

	//MONTH
	ds1307_write(binary_to_bcd(pRTC_date->month), DS1307_ADDR_MONTH);

	//YEAR
	ds1307_write(binary_to_bcd(pRTC_date->year), DS1307_ADDR_YEAR);

}



static void ds1307_i2c_pin_config()
{
	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	//I2C SCL---->PB6
	//I2C SDA---->PB7

	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_GPIO_PIN_SDA;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;

	GPIO_Init(&i2c_sda);


	i2c_scl.pGPIOx = GPIOB;
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_GPIO_PIN_SCL;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;

	GPIO_Init(&i2c_scl);

}

static void ds1307_i2c_config()
{

	i2c_handle.pI2Cx = DS1307_I2C;
	//i2c_handle.DevAddr = DS1307_I2C_SLAVE_ADDR;
	i2c_handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	i2c_handle.I2C_Config.I2C_SCL_Speed = DS1307_I2C_SPEED;

	I2C_Init(&i2c_handle);
}

static void ds1307_write(uint8_t value, uint8_t reg_addr)
{
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;

	I2C_MasterSendData(&i2c_handle, tx, 2, DS1307_I2C_SLAVE_ADDR, 0);

}

static uint8_t ds1307_read(uint8_t reg_addr)
{
	I2C_MasterSendData(&i2c_handle, &reg_addr, 1, DS1307_I2C_SLAVE_ADDR, 0);

	uint8_t clock_val;
	I2C_MasterReceiveData(&i2c_handle, &clock_val, 1, DS1307_I2C_SLAVE_ADDR, 0);

	return clock_val;
}

static uint8_t binary_to_bcd(uint8_t binry)
{
	uint8_t bcd;

	if(binry < 10)
	{
		bcd = binry;
		return bcd;
	}

	else
	{
		uint8_t m = binry /10;
		uint8_t n = binry % 10;
		bcd = (uint8_t)((m << 4) | n) ;
		return bcd;
	}
}

static uint8_t bcd_to_binary(uint8_t bcd)
{
	uint8_t binry = 0;
	uint8_t m,n;

	m = (uint8_t) ((bcd >> 4) *10);
	n = bcd & (uint8_t) 0x0F;
	binry = m+n;
	return binry;
}


