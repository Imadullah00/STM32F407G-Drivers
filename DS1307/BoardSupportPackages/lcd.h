/*
 * lcd.h
 *
 *  Created on: Mar 28, 2024
 *      Author: ImadF
 */

#include "stm32f407xx.h"

#ifndef LCD_H_
#define LCD_H_

//BSP APIs
void LCD_init();

#define LCD_GPIO_PORT		GPIOD

#define LCD_GPIO_RS			GPIO_PIN_NO_0
#define LCD_GPIO_RW			GPIO_PIN_NO_1
#define LCD_GPIO_EN			GPIO_PIN_NO_2
#define LCD_GPIO_D4			GPIO_PIN_NO_3
#define LCD_GPIO_D5			GPIO_PIN_NO_4
#define LCD_GPIO_D6			GPIO_PIN_NO_5
#define LCD_GPIO_D7			GPIO_PIN_NO_6

#define LCD_CMD_4DL_2N_5X8F			 0x28
#define LCD_CMD_DON_CURON			 0x0E
#define LCD_CMD_DIS_CLEAR			 0x01
#define LCD_CMD_DIS_RETURN_HOME		 0x02
#define LCD_CMD_INC_ADD		 		 0x06


void lcd_send_cmd(uint8_t cmd);
void lcd_print_char(uint8_t data);
void lcd_print_string(char *message);
void lcd_display_return_home(void);
void lcd_set_cursor(uint8_t row, uint8_t column);
void milli_delay(uint32_t count);
 void us_delay(uint32_t count);




#endif /* LCD_H_ */
