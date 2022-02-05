/**
  ******************************************************************************
  * @file		uart.h
  * @author	    Saloni Shah and Michelle Christian
  * @date		26 April 2021
  ******************************************************************************
  */

/** Includes ---------------------------------------------------------------- */
#include "uart.h"

/**
  ******************************************************************************
  * @brief	initialize UART peripheral
  * @param	baudrate - required baud rate of UART
  * @retval	None
  ******************************************************************************
  */
void UART_Init(uint32_t baudrate) {
	GPIO_InitTypeDef PORT;

	// UART init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	PORT.GPIO_Pin = UART_TX_PIN;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP; // TX as AF with Push-Pull
	GPIO_Init(UART_GPIO_PORT_TX,&PORT);
	PORT.GPIO_Pin = UART_RX_PIN;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING; // RX as in without pull-up
	GPIO_Init(UART_GPIO_PORT_RX,&PORT);

	USART_InitTypeDef UART;
	UART.USART_BaudRate = baudrate;
	UART.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No flow control
	UART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // RX+TX mode
	UART.USART_WordLength = USART_WordLength_8b; // 8-bit frame
	UART.USART_Parity = USART_Parity_No; // No parity check
	UART.USART_StopBits = USART_StopBits_1; // 1 stop bit
	USART_Init(UART_PORT,&UART);
	USART_Cmd(UART_PORT,ENABLE);
}

/**
  ******************************************************************************
  * @brief	Print a character on terminal
  * @param	ch - character to be printed
  * @retval	None
  ******************************************************************************
  */
void UART_SendChar(char ch) {
	while (!USART_GetFlagStatus(UART_PORT,USART_FLAG_TC)); // wait for "Transmission Complete" flag cleared
	USART_SendData(UART_PORT,ch);
}

/**
  ******************************************************************************
  * @brief	Print a number on terminal
  * @param	num - number to be printed
  * @retval	None
  ******************************************************************************
  */
void UART_SendInt(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		UART_SendChar('-');
		num *= -1;
	}
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	for (i--; i >= 0; i--) UART_SendChar(str[i]);
}

/**
  ******************************************************************************
  * @brief	print a character string on terminal
  * @param	*str - pointer to string to be printed
  * @retval	None
  ******************************************************************************
  */
void UART_SendStr(char *str) {
	while (*str) UART_SendChar(*str++);
}

/**
  ******************************************************************************
  * @brief	print a character buffer on terminal
  * @param	*buf - pointer to buffer to be printed
  * 		bufsize - size of buffer to be printed
  * @retval	None
  ******************************************************************************
  */
void UART_SendBuf(char *buf, uint16_t bufsize) {
	uint16_t i;
	for (i = 0; i < bufsize; i++) UART_SendChar(*buf++);
}

/**
  ******************************************************************************
  * @brief	print a buffer on terminal in hexadecimal
  * @param	*buf - pointer to buffer to be printed
  * 		bufsize - size of buffer to be printed
  * @retval	None
  ******************************************************************************
  */
void UART_SendBufHex(char *buf, uint16_t bufsize) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		UART_SendChar(HEX_CHARS[(ch >> 4)   % 0x10]);
		UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
	}
}
