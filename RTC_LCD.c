/*
 * RTC_LCD.c
 *
 *  Created on: Mar 28, 2024
 *      Author: ImadF
 */
#include <stdio.h>
#include <stdint.h>
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK 16000000UL

RTC_time_t rtc_current_time;
RTC_date_t rtc_current_date;

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

char* get_day_of_week(uint8_t i)
{
	char* days [] = {"SUNDAY", "MONDAY" , "TUESDAY", "WEDNESDAY" "THURSDAY", "FRIDAY",  "SATURDAY"};

	return days[i -1];

}

void number_to_string(uint8_t num , char* buf)
{
	if(num < 10)
	{
		buf[0] = '0';
		buf[1] = num+48;
	}

	else if (num>=10 && num < 99)
	{
		buf[0] = (num/10) +48;
		buf[1] = (num%10)+48;

	}
}

char* time_to_string(RTC_time_t* rtc_time)
{
	static char buf[9];

	buf[2] = ':';
	buf[5] = ':';

	number_to_string(rtc_time->hours, buf);
	number_to_string(rtc_time->minutes, &buf[3]);
	number_to_string(rtc_time->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;
}

char* date_to_string(RTC_date_t* rtc_date)
{
	static char buf [9];

	buf[2] = '/';
	buf[5] = '/';

	number_to_string(rtc_date->date, buf);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);

	return buf;

}
int main(void)
{
	//printf("RTC test \n");

	LCD_init();

	lcd_print_string("RTC Test....");

	milli_delay(2000);

	//DISPLAY CLEAR
	lcd_send_cmd(LCD_CMD_DIS_CLEAR);
	milli_delay(2);

	lcd_display_return_home();

	if(DS1307_Init())
	{
		printf("RTC Init not successful \n");
		while(1);
	}

	init_systick_timer(1);

	rtc_current_date.day = FRIDAY;
	rtc_current_date.date = 25;
	rtc_current_date.month = 1;
	rtc_current_date.year = 21;

	rtc_current_time.hours = 4;
	rtc_current_time.minutes = 15;
	rtc_current_time.seconds = 54;
	rtc_current_time.time_format = TIME_FORMAT_12HRS_PM;

	DS1307_SetCurrentDate(&rtc_current_date);
	DS1307_SetCurrentTime(&rtc_current_time);

	DS1307_GetCurrentDate(&rtc_current_date);
	DS1307_GetCurrentTime(&rtc_current_time);

	char* am_pm;

	if(rtc_current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (rtc_current_time.time_format) ? "AM" : "PM";
		lcd_print_string(time_to_string(&rtc_current_time));
		lcd_print_string((am_pm));

		//printf("Current time is %s %s", time_to_string(&rtc_current_time), am_pm);
	}

	else
	{
		lcd_print_string(time_to_string(&rtc_current_time));
		//printf("Current time is %s", time_to_string(&rtc_current_time));
	}

	//printf("Current date is %s %s", date_to_string(&rtc_current_date), get_day_of_week(rtc_current_date.day));
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&rtc_current_date));

	while(1);

	return 0;
}

void SysTick_Handler(void)
{
	DS1307_GetCurrentTime(&rtc_current_time);

	char* am_pm;
	lcd_set_cursor(1, 1);

	if(rtc_current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (rtc_current_time.time_format) ? "AM" : "PM";
		lcd_print_string(time_to_string(&rtc_current_time));
		lcd_print_string((am_pm));

		//printf("Current time is %s %s", time_to_string(&rtc_current_time), am_pm);
	}

	else
	{
		//printf("Current time is %s", time_to_string(&rtc_current_time));
		lcd_print_string(time_to_string(&rtc_current_time));

	}

	DS1307_GetCurrentDate(&rtc_current_date);
	lcd_set_cursor(2, 1);

	lcd_print_string(time_to_string(&rtc_current_time));
	lcd_print_char('<');
	lcd_print_string(get_day_of_week(rtc_current_date.day));
	lcd_print_char('>');

	//printf("Current date is %s %s", date_to_string(&rtc_current_date), get_day_of_week(rtc_current_date.day));

}
