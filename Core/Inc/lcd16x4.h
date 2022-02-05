/**
  ******************************************************************************
  * @file		lcd16X4.h
  * @author	    Saloni Shah and Michelle Christian
  * @date		26 April 2021
  ******************************************************************************
  */
	
#ifndef __LCD16X4_H
#define __LCD16X4_H

/** Includes ---------------------------------------------------------------- */
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "delay.h"

/** Port and pin definition for 4-bit mode ---------------------------------- */
// LCD control lines
#define LCD16X4_RCC_GPIO_CONTROL	RCC_APB2Periph_GPIOB
#define LCD16X4_GPIO_CONTROL			GPIOB
#define LCD16X4_GPIO_RS						GPIOB
#define LCD16X4_GPIO_RW						GPIOB
#define LCD16X4_GPIO_EN						GPIOB
// LCD data lines
#define LCD16X4_RCC_GPIO_DATA			RCC_APB2Periph_GPIOA
#define LCD16X4_GPIO_DATA					GPIOA
#define LCD16X4_GPIO_D4						GPIOA
#define LCD16X4_GPIO_D5						GPIOA
#define LCD16X4_GPIO_D6						GPIOA
#define LCD16X4_GPIO_D7						GPIOA
// Pin definition
#define LCD16X4_PIN_RS	GPIO_Pin_5
#define LCD16X4_PIN_RW	GPIO_Pin_6
#define LCD16X4_PIN_EN	GPIO_Pin_7
#define LCD16X4_PIN_D4	GPIO_Pin_4		// 4-bit mode LSB
#define LCD16X4_PIN_D5	GPIO_Pin_5
#define LCD16X4_PIN_D6	GPIO_Pin_6
#define LCD16X4_PIN_D7	GPIO_Pin_7 	// 4-bit mode MSB

/** Display size ------------------------------------------------------------ */
// Visible characters per line of the display
#define LCD16X4_DISP_LENGTH		16
// DDRAM address of first char of line 1
#define LCD16X4_START_LINE_1	0x00
// DDRAM address of first char of line 2
#define LCD16X4_START_LINE_2	0x40
// DDRAM address of first char of line 3
#define LCD16X4_START_LINE_3	0x10
// DDRAM address of first char of line 4
#define LCD16X4_START_LINE_4	0x50

/** Delay value ------------------------------------------------------------- */
// Delay power on
#define LCD16X4_DELAY_POWER_ON   		16000
// Delay after initialize 8-bit
#define LCD16X4_DELAY_INIT      		5000
// Delay after initialize 8-bit repeated
#define LCD16X4_DELAY_INIT_REP    	64
// Delay after set 4-bit mode
#define LCD16X4_DELAY_INIT_4BIT   	64
// Delay until address counter updated after busy flag is cleared
#define LCD16X4_DELAY_BUSY_FLAG    	4
// Enable pulse width high level
#define LCD16X4_DELAY_ENABLE_PULSE	2

/** Instructions bit location ----------------------------------------------- */
#define LCD16X4_CLEAR_DISPLAY					0x01
#define LCD16X4_CURSOR_HOME						0x02
#define LCD16X4_CHARACTER_ENTRY_MODE	0x04
#define LCD16X4_DISPLAY_CURSOR_ON_OFF	0x08
#define LCD16X4_DISPLAY_CURSOR_SHIFT 	0x10
#define LCD16X4_FUNCTION_SET					0x20
#define LCD16X4_SET_CGRAM_ADDRESS	 		0x40
#define LCD16X4_SET_DDRAM_ADDRESS	 		0x80
/* Character entry mode instructions */
#define LCD16X4_INCREMENT							0x02	// Initialization setting
#define LCD16X4_DECREMENT							0x00
#define LCD16X4_DISPLAY_SHIFT_ON			0x01
#define LCD16X4_DISPLAY_SHIFT_OFF			0x00	// Initialization setting
/* Display cursor on off instructions */
#define LCD16X4_DISPLAY_ON	 					0x04
#define LCD16X4_DISPLAY_OFF	 					0x00	// Initialization setting
#define LCD16X4_CURSOR_UNDERLINE_ON	 	0x02
#define LCD16X4_CURSOR_UNDERLINE_OFF	0x00	// Initialization setting
#define LCD16X4_CURSOR_BLINK_ON	 			0x01
#define LCD16X4_CURSOR_BLINK_OFF	 		0x00	// Initialization setting
/* Display cursor shift instructions */
#define LCD16X4_DISPLAY_SHIFT					0x08
#define LCD16X4_CURSOR_MOVE						0x00
#define LCD16X4_RIGHT_SHIFT						0x04
#define LCD16X4_LEFT_SHIFT						0x00
/* Function set instructions */
#define LCD16X4_8BIT_INTERFACE				0x10	// Initialization setting
#define LCD16X4_4BIT_INTERFACE				0x00
#define LCD16X4_2LINE_MODE						0x08
#define LCD16X4_1LINE_MODE						0x00	// Initialization setting
#define LCD16X4_5X10DOT_FORMAT				0x04
#define LCD16X4_5X7DOT_FORMAT					0x00	// Initialization setting
/* Busy flag bit location */
#define LCD16X4_BUSY_FLAG							0x80

/** LCD display and cursor attributes --------------------------------------- */
#define LCD16X4_DISPLAY_OFF_CURSOR_OFF_BLINK_OFF	(LCD16X4_DISPLAY_OFF | \
	LCD16X4_CURSOR_UNDERLINE_OFF | LCD16X4_CURSOR_BLINK_OFF)
#define LCD16X4_DISPLAY_ON_CURSOR_OFF_BLINK_OFF		(LCD16X4_DISPLAY_ON | \
	LCD16X4_CURSOR_UNDERLINE_OFF | LCD16X4_CURSOR_BLINK_OFF)
#define LCD16X4_DISPLAY_ON_CURSOR_OFF_BLINK_ON		(LCD16X4_DISPLAY_ON | \
	LCD16X4_CURSOR_UNDERLINE_OFF | LCD16X4_CURSOR_BLINK_ON)
#define LCD16X4_DISPLAY_ON_CURSOR_ON_BLINK_OFF		(LCD16X4_DISPLAY_ON | \
	LCD16X4_CURSOR_UNDERLINE_ON | LCD16X4_CURSOR_BLINK_OFF)
#define LCD16X4_DISPLAY_ON_CURSOR_ON_BLINK_ON		(LCD16X4_DISPLAY_ON | \
	LCD16X4_CURSOR_UNDERLINE_ON | LCD16X4_CURSOR_BLINK_ON)

/** Public function prototypes ---------------------------------------------- */
void lcd16X4_init(uint8_t disp_attr);
void lcd16X4_write_command(uint8_t cmd);
void lcd16X4_write_data(uint8_t data);
void lcd16X4_clrscr(void);
void lcd16X4_home(void);
void lcd16X4_gotoxy(uint8_t x, uint8_t y);
uint8_t lcd16X4_getxy(void);
void lcd16X4_entry_inc(void);
void lcd16X4_entry_dec(void);
void lcd16X4_display_on(void);
void lcd16X4_display_off(void);
void lcd16X4_cursor_on(void);
void lcd16X4_cursor_off(void);
void lcd16X4_blink_on(void);
void lcd16X4_blink_off(void);
void lcd16X4_cursor_shift_left(void);
void lcd16X4_cursor_shift_right(void);
void lcd16X4_putc(const char c);
void lcd16X4_puts(const char* s);
void lcd16X4_create_custom_char(uint8_t location, uint8_t* data_bytes);
void lcd16X4_put_custom_char(uint8_t x, uint8_t y, uint8_t location);

#endif

/********************************* END OF FILE ********************************/
/******************************************************************************/
