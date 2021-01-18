/**
  ******************************************************************************
  * @file    mesn_uart.h
  * @author  BDufay
  * @version 
  * @date    2018-09
  * @brief   Uart driver for STM32L1x.
  *          This file provides firmware functions to manage the UART
  *          It is designed to handle both USART2  periph (on PA2/PA3 pins,
  *          connected to the included virtual com port of the debug prog) or
  *          USART3 (on PC10/PC11 pins, through the MeSN practicals extension
  *          board) on STM32L1 MCUs.
  *          Selecting the appropriate USART have to be done in u_uart.h
  *          header file.
  ******************************************************************************  
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MESN_UART_H
#define __MESN_UART_H

// Choose between USART2 or USART3
#define		USE_UART2
//#define		USE_UART3

/* Includes ------------------------------------------------------------------*/
#include "errorStatus.h"
#if defined(USE_UART2)			//PA2 and PA3 pins (VCP of STlink probe)
  #include "stm32l1xx_hal.h"
#elif defined(USE_UART3)		//PC10 and PC11 pins
  #include "stm32l1xx_hal.h"
#else
  #error "Please select USART instance"
#endif

/* Exported constants --------------------------------------------------------*/
#define UART_BUFFERSIZE 	256

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported globals var ------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void MESN_UART_Init();
void MESN_UART_PutString_Poll(uint8_t *stringToSend);
MeSN_StatusTypedef MESN_UART_GetString(uint8_t *rxString, uint32_t timeOut);

#endif /* __MESN_UART_H */
