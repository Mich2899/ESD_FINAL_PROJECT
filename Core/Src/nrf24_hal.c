/**
  ******************************************************************************
  * @file		nrf24_hal.c
  * @author	    Saloni Shah and Michelle christian
  * @date		26 April 2021
  ******************************************************************************
  */

/** Includes ---------------------------------------------------------------- */
#include "nrf24_hal.h"

/** Function definitions for enable and disable of NRF module--------------- */
void nRF24_CE_L() {                 (GPIO_ResetBits(nRF24_CE_PORT, nRF24_CE_PIN)); }
void nRF24_CE_H() {               (GPIO_SetBits(nRF24_CE_PORT, nRF24_CE_PIN)); }

void nRF24_CSN_L() {                GPIO_ResetBits(nRF24_CSN_PORT, nRF24_CSN_PIN); }
void nRF24_CSN_H() {                GPIO_SetBits(nRF24_CSN_PORT, nRF24_CSN_PIN); }

/**
  ******************************************************************************
  * @brief	Configure the GPIO lines of the nRF24L01 transceiver
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void nRF24_GPIO_Init(void) {
    GPIO_InitTypeDef PORT;

    // Enable the nRF24L01 GPIO peripherals
	RCC->APB2ENR |= nRF24_GPIO_PERIPHERALS;

    // Configure CSN pin
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_Pin = nRF24_CSN_PIN;
	GPIO_Init(nRF24_CSN_PORT, &PORT);
	nRF24_CSN_H();

	// Configure CE pin
	PORT.GPIO_Pin = nRF24_CE_PIN;
	GPIO_Init(nRF24_CE_PORT, &PORT);
	nRF24_CE_L();
}

/**
  ******************************************************************************
  * @brief	Low level SPI transmit/receive function
  * @param	data - value to transmit via SPI
  * @retval	value received from SPI
  ******************************************************************************
  */
uint8_t nRF24_LL_RW(uint8_t data) {
	 // Wait until TX buffer is empty
	while (SPI_I2S_GetFlagStatus(nRF24_SPI_PORT, SPI_I2S_FLAG_TXE) == RESET);
	// Send byte to SPI (TXE cleared)
	SPI_I2S_SendData(nRF24_SPI_PORT, data);
	// Wait while receive buffer is empty
	while (SPI_I2S_GetFlagStatus(nRF24_SPI_PORT, SPI_I2S_FLAG_RXNE) == RESET);

	// Return received byte
	return (uint8_t)SPI_I2S_ReceiveData(nRF24_SPI_PORT);
}
