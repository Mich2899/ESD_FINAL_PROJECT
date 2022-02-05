/**
  ******************************************************************************
  * @file		nrf24_hal.h
  * @author	    Saloni Shah and Michelle Christian
  * @date		26 April 2021
  ******************************************************************************
  */
#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H

/** Peripheral libraries----------------------------------------------------- */
#include <stm32f10x_gpio.h>
#include <stm32f10x_spi.h>

/** SPI port peripheral ---------------------------------------------------- */
#define nRF24_SPI_PORT             SPI2

/** nRF24 GPIO peripherals ---------------------------------------------------- */
#define nRF24_GPIO_PERIPHERALS     RCC_APB2ENR_IOPBEN

/** CE (chip enable) pin (PB11)--------------------------------------------- */
#define nRF24_CE_PORT              (GPIOB)
#define nRF24_CE_PIN               (GPIO_Pin_11)
void nRF24_CE_L();
void nRF24_CE_H();

/** CSN (chip select negative) pin (PB12)------------------------------------ */
#define nRF24_CSN_PORT             GPIOB
#define nRF24_CSN_PIN              GPIO_Pin_12
void nRF24_CSN_L();
void nRF24_CSN_H();

/** IRQ pin (PB10)---------------------------------------------------------- */
#define nRF24_IRQ_PORT             GPIOB
#define nRF24_IRQ_PIN              GPIO_Pin_10

/** Macros for the RX on/off------------------------------------------------ */
#define nRF24_RX_ON                nRF24_CE_H
#define nRF24_RX_OFF               nRF24_CE_L


/** Public function prototypes ---------------------------------------------- */
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
