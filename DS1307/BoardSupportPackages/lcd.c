/*
 * lcd.c
 *
 *  Created on: Mar 28, 2024
 *      Author: ImadF
 */
#include "lcd.h"

static void lcd_enable();


void lcd_send_cmd(uint8_t cmd)
{
	//make RS = 0; makr RnW = 0;
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//write the HIGHER nibble
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((cmd >> 4) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((cmd >> 5) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((cmd >> 6) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((cmd >> 7) & 0x1) );

	lcd_enable();

	//write the LOWER nibble
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((cmd >> 0) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((cmd >> 1) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((cmd >> 2) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((cmd >> 3) & 0x1) );

	lcd_enable();
}

void lcd_print_char(uint8_t data)
{
	//make RS = 1; makr RnW = 0;
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//write the HIGHER nibble
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((data >> 4) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((data >> 5) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((data >> 6) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((data >> 7) & 0x1) );

	lcd_enable();

	//write the LOWER nibble
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((data >> 0) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((data >> 1) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((data >> 2) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((data >> 3) & 0x1) );

	lcd_enable();
}

void lcd_print_string(char *message)
{

  do
  {
	  lcd_print_char((uint8_t)*message++);
  }
  while (*message != '\0');

}

static void lcd_enable()
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	us_delay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	us_delay(100); //needed in CMD transfer/  not needed in DATA transfer but anyways
}

void LCD_init()
{
	//OONFIGURE PINS FOR LCD CONNECTIONS
	GPIO_Handle_t LCD_gpio_handle;

	LCD_gpio_handle.pGPIOx = LCD_GPIO_PORT;
	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&LCD_gpio_handle);

	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&LCD_gpio_handle);

	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&LCD_gpio_handle);

	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&LCD_gpio_handle);

	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&LCD_gpio_handle);

	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&LCD_gpio_handle);

	LCD_gpio_handle.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&LCD_gpio_handle);

		//write RESET to all pins
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//DO THE INITIALIZATION OF LCD

	// 0. delay of at least 40ms
	milli_delay(40);

	// 1. Make RS, RnW = 0;
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//2. PROGRAM D7 D6 D5 D4 as  0 0 1 1
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//3. enable
	lcd_enable();

	//4 delay of 4.1 ms
	milli_delay(5);

	//5. REPEAT STEP 2
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//6. Delay of > 100 us
	us_delay(150);

	//7. REPEAT STEP 2
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//8. PROGRAM D7 D6 D5 D4 as  0 1 0 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//function set command
	lcd_send_cmd(LCD_CMD_4DL_2N_5X8F);

	//DISPLAY ON & CURSOR ON
	lcd_send_cmd(LCD_CMD_DON_CURON);

	//DISPLAY CLEAR
	lcd_send_cmd(LCD_CMD_DIS_CLEAR);
	milli_delay(2);

	//ENTRY MODE SET
	lcd_send_cmd(LCD_CMD_INC_ADD);
}

 void milli_delay(uint32_t count)
{
	for(uint32_t i = 0; i < (count*1000); i++);
}


 void us_delay(uint32_t count)
{
	for(uint32_t i = 0; i < (count); i++);
}


/*Cursor returns to home position */
void lcd_display_return_home(void)
{

	lcd_send_cmd(LCD_CMD_DIS_RETURN_HOME);
	/*
	 * check page number 24 of datasheet.
	 * return home command execution wait time is around 2ms
	 */
	milli_delay(2);
}


void lcd_set_cursor(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      /* Set cursor to 1st row address and add index*/
      lcd_send_cmd((column |= 0x80));
      break;

    case 2:
      /* Set cursor to 2nd row address and add index*/
        lcd_send_cmd((column |= 0xC0));
      break;

    default:
      break;
  }
}
