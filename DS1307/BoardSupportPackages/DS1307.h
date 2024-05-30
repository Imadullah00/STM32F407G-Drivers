/*
 * ds1307.h
 *
 *  Created on: Mar 28, 2024
 *      Author: ImadF
 */

#ifndef DS1307_H_
#define DS1307_H_


#include <stdint.h>
#include "stm32f407xx.h"

#define DS1307_I2C					I2C1
#define DS1307_I2C_GPIO_PORT		GPIOB
#define DS1307_I2C_GPIO_PIN_SCL		GPIO_PIN_NO_6
#define DS1307_I2C_GPIO_PIN_SDA		GPIO_PIN_NO_7
#define DS1307_I2C_SPEED			I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD				GPIO_PIN_PU;



// REGISTER ADDRESSES
#define DS1307_ADDR_SEC				0x00
#define DS1307_ADDR_MIN				0x01
#define DS1307_ADDR_HRS				0x02
#define DS1307_ADDR_DAY				0x03
#define DS1307_ADDR_DATE			0x04
#define DS1307_ADDR_MONTH			0x05
#define DS1307_ADDR_YEAR			0x06

#define TIME_FORMAT_12HRS_AM		0
#define TIME_FORMAT_12HRS_PM		1
#define TIME_FORMAT_24HRS			2

#define DS1307_I2C_SLAVE_ADDR		0x68

#define SUNDAY						1
#define MONDAY						2
#define TUESDAY						3
#define WEDNESDAY					4
#define THURSDAY					5
#define FRIDAY						6
#define SATURDAY					7

typedef struct
{
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;

}RTC_date_t;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;

}RTC_time_t;


//function prototypes

uint8_t DS1307_Init(void);

void DS1307_SetCurrentTime(RTC_time_t *pRTC_time);
void DS1307_SetCurrentDate(RTC_date_t *pRTC_date);

void DS1307_GetCurrentDate(RTC_date_t *pRTC_date);
void DS1307_GetCurrentTime(RTC_time_t *pRTC_time);



#endif /* DS1307_H_ */
