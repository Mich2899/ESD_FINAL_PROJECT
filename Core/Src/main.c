/*
 * **************************************************************
 * This project is intended to receive temperature data
 * from  (TMP36) fixed on a mobile bot and transmit it
 * through a wireless channel using NRF24L01+ module
 * and display the data on LCD
 *
 * Main function to execute transmission and receiving of data
 *
 * Authors: Saloni Shah and Michelle Christian
 * **************************************************************
 */

/** Includes ---------------------------------------------------------------- */
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_conf.h"
#include<stdio.h>
#include "uart.h"
#include "delay.h"
#include "lcd16x4.h"
#include "misc.h"
#include "nrf24.h"


// Define what part of code will be compiled:
//   0 : disable
//   1 : enable
#define RX_SINGLE      1 // Single address receiver
#define TX_SINGLE      0 // Single address transmitter


uint32_t i,j,k;

// Buffer to store a payload of maximum width
uint8_t nRF24_payload[32];
uint8_t temp_buf[32];

// Pipe number
nRF24_RXResult pipe;

// Length of received payload
uint8_t packet_length;

/** Function definitions for receiver ---------------------------------- */
#if (RX_SINGLE)

uint8_t tmp[2];

//define custom characters for LCD
uint8_t custom_veryless[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF };
uint8_t custom_less[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF };
uint8_t custom_average[] = { 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t custom_littlehigh[] = {0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t custom_high[] = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t custom_veryhigh[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t custom_degree[] = { 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/**
  ******************************************************************************
  * @brief  get temperature value buffer and display temperature value on LCD
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void read_temp() {

	//check for buffer
	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
		// Get a payload from the transceiver
		pipe = nRF24_ReadPayload(temp_buf, &packet_length);

		// Clear all pending IRQ flags
		nRF24_ClearIRQFlags();

		// Print a payload contents to UART
		UART_SendStr("Temperature: ");
		UART_SendInt(pipe);
		UART_SendBufHex((char *)temp_buf, packet_length);
		UART_SendStr(" ");
		UART_SendInt(*temp_buf);
		UART_SendStr("\r\n");

		tmp[0]=((*temp_buf)/10);
		tmp[1]=((*temp_buf)%10);


		//diffrent characters to indicate temperature intensity level
		if(10<=(*temp_buf) && (*temp_buf)<=13)
			lcd16X4_put_custom_char(0, 2, 0);
		else if(14<=(*temp_buf) && (*temp_buf)<=17)
			lcd16X4_put_custom_char(0, 2, 1);
		else if(18<=(*temp_buf) && (*temp_buf)<=21)
			lcd16X4_put_custom_char(0, 2, 2);
		else if(22<=(*temp_buf) && (*temp_buf)<=25)
			lcd16X4_put_custom_char(0, 2, 3);
		else if(26<=(*temp_buf) && (*temp_buf)<=29)
			lcd16X4_put_custom_char(0, 2, 4);
		else if(29<=(*temp_buf) && (*temp_buf)<=35)
			lcd16X4_put_custom_char(0, 2, 5);
		else
			lcd16X4_gotoxy(1,2);

		lcd16X4_puts("Temp: ");
		if(tmp[0] != 0)
			lcd16X4_write_data(tmp[0]+48);

		lcd16X4_write_data(tmp[1]+48);
		lcd16X4_put_custom_char(9, 2, 6);
		lcd16X4_puts("C");
	}
}
#endif

/** Function definitions for transmitter ---------------------------------- */
#if (TX_SINGLE)

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

#define LEFT_FOR GPIO_Pin_4
#define LEFT_BACK GPIO_Pin_5
#define RIGHT_FOR GPIO_Pin_6
#define RIGHT_BACK GPIO_Pin_7

int i=0;
int adc_value;
uint8_t temperature;
char buffer[50] = {'\0'};

// Result of packet transmission
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

nRF24_TXResult tx_res;

/**
  ******************************************************************************
  * @brief	Function to transmit data packet
  * @param	pBuf - pointer to the buffer with data to transmit
  *	   	    length - length of the data buffer in bytes
  * @retval	one of nRF24_TX_xx values
  ******************************************************************************
  */
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
	UART_SendStr("[");
	UART_SendHex8(status);
	UART_SendStr("] ");

	// Clear pending IRQ flags
	nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}

/**
  ******************************************************************************
  * @brief	Initialize the PWM module.
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void PWM_Init()
{
	// Initialization struct
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef PORT;

	// Initialize TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// Create 1kHz PWM
	// TIM2 is connected to APB1 bus that have default clock 72MHz
	// So, the frequency of TIM2 is 72MHz
	// We use prescaler 10 here
	// So, the frequency of TIM2 now is 72MHz
	TIM_TimeBaseInitStruct.TIM_Prescaler = 10;

	// TIM_Period determine the PWM frequency by this equation:
	// PWM_frequency = timer_clock / (TIM_Period + 1)
	// If we want 1kHz PWM we can calculate:
	// TIM_Period = (timer_clock / PWM_frequency) - 1
	// TIM_Period = (7.2MHz / 1kHz) - 1 = 7199
	TIM_TimeBaseInitStruct.TIM_Period = 7199;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	// Start TIM2
	TIM_Cmd(TIM2, ENABLE);

	// Initialize PWM
	// Common PWM settings
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;

	// Duty cycle calculation equation:
	// TIM_Pulse = (((TIM_Period + 1) * duty_cycle) / 100) - 1
	// Ex. 25% duty cycle:
	// 		TIM_Pulse = (((7199 + 1) * 25) / 100) - 1 = 1799
	//		TIM_Pulse = (((7199 + 1) * 75) / 100) - 1 = 5399
	// We initialize PWM value with duty cycle of 0%
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	//Initialize GPIOA (PA0 and PA1)

	// Initialize pins as push-pull alternate function (PWM output)
	PORT.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &PORT);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// Configure gpio pins for direction as open-drain output
	PORT.GPIO_Pin = LEFT_FOR | LEFT_BACK | RIGHT_FOR | RIGHT_BACK;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &PORT);

}

/**
  ******************************************************************************
  * @brief	initialize external interrupt for push button
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void EXTI15_10_Init()
{
	// Initialization struct
	GPIO_InitTypeDef PORT;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	// Initialize Pc13, PC14 and PC15 as input with pull-up resistor
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	PORT.GPIO_Mode = GPIO_Mode_IPU;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &PORT);

	// Initialize PC pins to EXTI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource15);

	// Initialize EXTI line 13, 14 and 15 for PC pins
	EXTI_InitStruct.EXTI_Line = EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);

	// Initialize NVIC for EXTI15_10 IRQ channel
	NVIC_InitStruct.NVIC_IRQChannel = 40;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

/**
  ******************************************************************************
  * @brief	ISR for external interrupt
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void EXTI15_10_IRQHandler(void)
{
	// Checks whether the interrupt is from EXTI13 or not
	if (EXTI_GetITStatus(EXTI_Line13))	//for backward direction
	{
		GPIO_WriteBit(GPIOB,LEFT_FOR,Bit_SET);   // Start left motors counter clock wise rotation
		GPIO_WriteBit(GPIOB,LEFT_BACK,Bit_RESET);

		GPIO_WriteBit(GPIOB,RIGHT_FOR,Bit_RESET);   //Start right motors counter clock wise rotation
		GPIO_WriteBit(GPIOB,RIGHT_BACK,Bit_SET);

		Delay_ms(5000);

		//move bot in forward direction
		GPIO_WriteBit(GPIOB,LEFT_FOR,Bit_SET);
		GPIO_WriteBit(GPIOB,LEFT_BACK,Bit_RESET);

		GPIO_WriteBit(GPIOB,RIGHT_FOR,Bit_SET);
		GPIO_WriteBit(GPIOB,RIGHT_BACK,Bit_RESET);
		// Clear pending bit
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
	else if (EXTI_GetITStatus(EXTI_Line14))		//for left turn
	{
		GPIO_WriteBit(GPIOB,LEFT_FOR,Bit_SET);   // Start left motors clock wise rotation
		GPIO_WriteBit(GPIOB,LEFT_BACK,Bit_RESET);

		GPIO_WriteBit(GPIOB,RIGHT_FOR,Bit_SET);   //Stop right motors
		GPIO_WriteBit(GPIOB,RIGHT_BACK,Bit_SET);

		Delay_ms(4000);

		//move bot in forward direction
		GPIO_WriteBit(GPIOB,LEFT_FOR,Bit_SET);
		GPIO_WriteBit(GPIOB,LEFT_BACK,Bit_RESET);

		GPIO_WriteBit(GPIOB,RIGHT_FOR,Bit_SET);
		GPIO_WriteBit(GPIOB,RIGHT_BACK,Bit_RESET);
		// Clear pending bit
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
	else if (EXTI_GetITStatus(EXTI_Line15))		//for right turn
	{
		GPIO_WriteBit(GPIOB,LEFT_FOR,Bit_SET);   // Stop left motors
		GPIO_WriteBit(GPIOB,LEFT_BACK,Bit_SET);

		GPIO_WriteBit(GPIOB,RIGHT_FOR,Bit_SET);   //Start right motors clock wise rotation
		GPIO_WriteBit(GPIOB,RIGHT_BACK,Bit_RESET);

		Delay_ms(4000);

		//move bot in forward direction
		GPIO_WriteBit(GPIOB,LEFT_FOR,Bit_SET);
		GPIO_WriteBit(GPIOB,LEFT_BACK,Bit_RESET);

		GPIO_WriteBit(GPIOB,RIGHT_FOR,Bit_SET);
		GPIO_WriteBit(GPIOB,RIGHT_BACK,Bit_RESET);
		// Clear pending bit
		EXTI_ClearITPendingBit(EXTI_Line15);
	}

}

/**
  ******************************************************************************
  * @brief	Initialize the ADC for temperature sensor.
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void ADC_init() {
	ADC_InitTypeDef ADC_InitStructure;

	// input of ADC (it doesn't seem to be needed, as default GPIO state is floating input)
	PORT.GPIO_Mode  = GPIO_Mode_AIN;
	PORT.GPIO_Pin   = GPIO_Pin_4 ;		//  (PA4 on STM32)
	GPIO_Init(GPIOA, &PORT);

	//clock for ADC (max 14MHz --> 72/6=12MHz)
	RCC_ADCCLKConfig (RCC_PCLK2_Div6);
	// enable ADC system clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// define ADC config
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	// we work in continuous sampling mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_RegularChannelConfig(ADC1,ADC_Channel_4, 1,ADC_SampleTime_28Cycles5); // define regular conversion config
	ADC_Init ( ADC1, &ADC_InitStructure);	//set config of ADC1

	// enable ADC
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1

	//	ADC calibration (optional, but recommended at power on)
	ADC_ResetCalibration(ADC1);	// Reset previous calibration
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);	// Start new calibration (ADC must be off at that time)
	while(ADC_GetCalibrationStatus(ADC1));

	// start conversion
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	// start conversion (will be endless as we are in continuous mode)

}

#endif // TX_SINGLE

/**
  ******************************************************************************
  * @brief	Initialize the SPI protocol for NRF module.
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void SPI_init() {
	GPIO_InitTypeDef PORT;
	SPI_InitTypeDef SPI;

	// Enable SPI2 peripheral
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

	// Enable SPI2 GPIO peripheral (PORTB)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	// Configure nRF24 IRQ pin
	PORT.GPIO_Mode  = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_Pin   = nRF24_IRQ_PIN;
	GPIO_Init(nRF24_IRQ_PORT, &PORT);

	// Configure SPI pins (SPI2)
	PORT.GPIO_Mode  = GPIO_Mode_AF_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &PORT);


	// Initialize SPI2
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI.SPI_CPOL = SPI_CPOL_Low;
	SPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(nRF24_SPI_PORT, &SPI);
	SPI_NSSInternalSoftwareConfig(nRF24_SPI_PORT, SPI_NSSInternalSoft_Set);
	SPI_Cmd(nRF24_SPI_PORT, ENABLE);
}

/**
  ******************************************************************************
  * @brief	main implementation.
  * @param	None
  * @retval	None
  ******************************************************************************
  */
int main(void) {

	//initialize UART
	UART_Init(115200);
	UART_SendStr("\r\nSTM32F103RET6 is online.\r\n");

	SPI_init();

	// Initialize delay
	Delay_Init();

	// Initialize the nRF24L01 GPIO pins
	nRF24_GPIO_Init();

	// RX/TX disabled
	nRF24_CE_L();

	// Configure the nRF24L01+
	UART_SendStr("nRF24L01+ check: ");
	if (!nRF24_Check()) {
		UART_SendStr("FAIL\r\n");
		while (1);
	}
	UART_SendStr("OK\r\n");

	// Initialize the nRF24L01 to its default state
	nRF24_Init();

	/***************************************************************************/

#if (DEMO_RX_SINGLE)

	//initialize LCD display
	lcd16X4_init(LCD16X4_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);

	// This is simple receiver with one RX pipe:
	//   - pipe#1 address: '0xE7 0x1C 0xE3'
	//   - payload: 5 bytes
	//   - RF channel: 115 (2515MHz)
	//   - data rate: 250kbps (minimum possible, to increase reception reliability)
	//   - CRC scheme: 2 byte

	// The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

	// Disable ShockBurst for all RX pipes
	nRF24_DisableAA(0xFF);

	// Set RF channel
	nRF24_SetRFChannel(115);

	// Set data rate
	nRF24_SetDataRate(nRF24_DR_250kbps);

	// Set CRC scheme
	nRF24_SetCRCScheme(nRF24_CRC_2byte);

	// Set address width, its common for all pipes (RX and TX)
	nRF24_SetAddrWidth(3);

	// Configure RX PIPE#1
	static const uint8_t nRF24_ADDR[] = { 0xE7, 0x1C, 0xE3 };
	nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
	nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 5); // Auto-ACK: disabled, payload length: 5 bytes

	// Set operational mode (PRX == receiver)
	nRF24_SetOperationalMode(nRF24_MODE_RX);

	// Wake the transceiver
	nRF24_SetPowerMode(nRF24_PWR_UP);

	// Put the transceiver to the RX mode
	nRF24_CE_H();

	//create custom characters for LCD
	lcd16X4_create_custom_char(0, custom_veryless);
	lcd16X4_create_custom_char(1, custom_less);
	lcd16X4_create_custom_char(2, custom_average);
	lcd16X4_create_custom_char(3, custom_littlehigh);
	lcd16X4_create_custom_char(4, custom_high);
	lcd16X4_create_custom_char(5, custom_veryhigh);
	lcd16X4_create_custom_char(6, custom_degree);

	lcd16X4_gotoxy(2,0);
	lcd16X4_puts("ESD PROJECT");


	// The main loop
	while (1) {

		read_temp();	//function to get temperature values and display on LCD
	}

#endif // RX_SINGLE

	/***************************************************************************/

#if (TX_SINGLE)

	// This is simple transmitter (to one logic address):
	//   - TX address: '0xE7 0x1C 0xE3'
	//   - payload: 5 bytes
	//   - RF channel: 115 (2515MHz)
	//   - data rate: 250kbps (minimum possible, to increase reception reliability)
	//   - CRC scheme: 2 byte

	// The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

	// Disable ShockBurst for all RX pipes
	nRF24_DisableAA(0xFF);

	// Set RF channel
	nRF24_SetRFChannel(115);

	// Set data rate
	nRF24_SetDataRate(nRF24_DR_250kbps);

	// Set CRC scheme
	nRF24_SetCRCScheme(nRF24_CRC_2byte);

	// Set address width, its common for all pipes (RX and TX)
	nRF24_SetAddrWidth(3);

	// Configure TX PIPE
	static const uint8_t nRF24_ADDR[] = { 0xE7, 0x1C, 0xE3 };
	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address

	// Set TX power (maximum)
	nRF24_SetTXPower(nRF24_TXPWR_0dBm);

	// Set operational mode (PTX == transmitter)
	nRF24_SetOperationalMode(nRF24_MODE_TX);

	// Clear any pending IRQ flags
	nRF24_ClearIRQFlags();

	// Wake the transceiver
	nRF24_SetPowerMode(nRF24_PWR_UP);

	//analog peripheral initialization for temperature sensor
	ADC_init();

	//external interrupt initialization for remote control
	EXTI15_10_Init();

	//GPIO initialization for motor direction control
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// Configure PB4, PB5, PB6, PB7 as open-drain output
	PORT.GPIO_Pin = LEFT_FOR | LEFT_BACK | RIGHT_FOR | RIGHT_BACK;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &PORT);

	//PWM initialization for motor speed control
	PWM_Init();

	packet_length = 5;

	TIM2->CCR1 = 7199;
	TIM2->CCR2 = 7199;

	GPIO_WriteBit(GPIOB,LEFT_FOR,Bit_SET);   // Start left motors clock wise rotation
	GPIO_WriteBit(GPIOB,LEFT_BACK,Bit_RESET);

	GPIO_WriteBit(GPIOB,RIGHT_FOR,Bit_SET);   //Start right motors clock wise rotation
	GPIO_WriteBit(GPIOB,RIGHT_BACK,Bit_RESET);

	while (1)
	{

		//get temperature values and store them in a buffer
		for (i = 0; i < packet_length; i++) {
			adc_value = (ADC_GetConversionValue(ADC1)>>2);
			temperature = (((adc_value)*(330))/1024);
			temperature =  (temperature) - 50;
			temp_buf[i] = temperature;
		}
		sprintf(buffer, "ADC=%d, T=%d C\r\n", adc_value, temperature);
		UART_SendBuf(buffer, sizeof(buffer));
		UART_SendStr("Transmission Status: ");
		UART_SendBufHex((char *)temp_buf, packet_length);

		// Transmit a packet
		Delay_ms(500);
		tx_res = nRF24_TransmitPacket(temp_buf, packet_length);
		switch (tx_res) {
		case nRF24_TX_SUCCESS:
			UART_SendStr("OK");
			break;
		case nRF24_TX_TIMEOUT:
			UART_SendStr("TIMEOUT");
			break;
		case nRF24_TX_MAXRT:
			UART_SendStr("MAX RETRANSMIT");
			break;
		default:
			UART_SendStr("ERROR");
			break;
		}
		UART_SendStr("\r\n");
		// Wait ~0.5s
		Delay_ms(500);
	}

#endif //TX_SINGLE
}
