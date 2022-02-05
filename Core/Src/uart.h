/**
  ******************************************************************************
  * @file		uart.h
  * @author	    Saloni Shah and Michelle Christian
  * @date		26 April 2021
  ******************************************************************************
  */

/** Includes ---------------------------------------------------------------- */
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "string.h"

/** UART PORT ---------------------------------------------------------------- */
#define _UART_PORT 2

	#define UART_PORT         USART2
	#define UART_TX_PIN       GPIO_Pin_2    // PA2 (USART2_TX)
	#define UART_RX_PIN       GPIO_Pin_3    // PA3 (USART2_RX)
	#define UART_GPIO_PORT_TX GPIOA
	#define UART_GPIO_PORT_RX UART_GPIO_PORT_TX

#define HEX_CHARS      "0123456789ABCDEF"

/** Public function prototypes ---------------------------------------------- */
void UART_Init(uint32_t baudrate);

void UART_SendChar(char ch);

void UART_SendInt(int32_t num);

void UART_SendStr(char *str);

void UART_SendBuf(char *buf, uint16_t bufsize);
void UART_SendBufHex(char *buf, uint16_t bufsize);
