/**
  ******************************************************************************
  * @file    mesn_uart.c
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

/* Includes ------------------------------------------------------------------*/
#include "mesn_uart.h"
#include "cmsis_os.h"

/* Public variables */

/* Private define ------------------------------------------------------------*/
#if defined(USE_UART2)
  #define UART_INSTANCE     		USART2
	#define UART_TX_PIN						GPIO_PIN_2
	#define UART_RX_PIN						GPIO_PIN_3
	#define GPIO_AF_UART					GPIO_AF7_USART2
	#define UART_PORT							GPIOA
	#define UART_GPIO_SPEED				GPIO_SPEED_VERY_LOW;
	#define UART_IRQn							USART2_IRQn
	#define __UART_CLK_ENABLE()		__USART2_CLK_ENABLE()
	#define __UART_CLK_DISABLE()	__USART2_CLK_DISABLE()
	#define TX_DATA_REG						DR
	#define	STATUS_REG						SR
	#define	RX_DATA_REG						DR
	#define	TX_EMPTY_BIT					USART_SR_TXE
	#define RX_NEMTPY_BIT					USART_SR_RXNE
#elif defined(USE_UART3)
  #define UART_INSTANCE     		USART3
	#define UART_TX_PIN						GPIO_PIN_10
	#define UART_RX_PIN						GPIO_PIN_11
	#define GPIO_AF_UART					GPIO_AF7_USART3
	#define UART_PORT							GPIOC
	#define UART_GPIO_SPEED				GPIO_SPEED_VERY_LOW;
	#define UART_IRQn							USART3_IRQn
	#define __UART_CLK_ENABLE()		__USART3_CLK_ENABLE()
	#define __UART_CLK_DISABLE()	__USART3_CLK_DISABLE()
	#define TX_DATA_REG						DR
	#define	STATUS_REG						SR
	#define	RX_DATA_REG						DR
	#define	TX_EMPTY_BIT					USART_SR_TXE
	#define RX_NEMTPY_BIT					USART_SR_RXNE
#else
  #error "Please select USART instance"
#endif

/* Private macro -------------------------------------------------------------*/
#define UART_DIV_SAMPLING16(_PCLK_, _BAUD_)         (((_PCLK_)*25)/(4*(_BAUD_)))
#define UART_DIVMANT_SAMPLING16(_PCLK_, _BAUD_)     (UART_DIV_SAMPLING16((_PCLK_), (_BAUD_))/100)
#define UART_DIVFRAQ_SAMPLING16(_PCLK_, _BAUD_)     (((UART_DIV_SAMPLING16((_PCLK_), (_BAUD_)) - (UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) * 100)) * 16 + 50) / 100)
/* UART BRR = mantissa + overflow + fraction
            = (UART DIVMANT << 4) + (UART DIVFRAQ & 0xF0) + (UART DIVFRAQ & 0x0F) */
#define UART_BRR_SAMPLING16(_PCLK_, _BAUD_)            (((UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) << 4) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0xF0)) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0x0F))

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static osMutexId uartTxSem; 	// Mutex ID
static osMutexDef(uartTxSem); 	// Mutex definition
static osMessageQDef(uartRxQ, UART_BUFFERSIZE, char); // Define message queue
static osMessageQId uartRxQ_ID;

/* Private function prototypes -----------------------------------------------*/
static void MESN_PRIV_UART_IRQHandler(void);
static MeSN_StatusTypedef MESN_PRIV_UART_GetChar(uint8_t* rxData, uint32_t timeOut);
static void MESN_PRIV_UART_PutChar_Poll(uint8_t dataToSend);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief USART init function
  * @param none
  * @retval None
  */
void MESN_UART_Init()
{

	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t tmpreg = 0x00;

	/* Peripheral clock enable */
	__UART_CLK_ENABLE();

	/* UART GPIO Configuration */
	GPIO_InitStruct.Pin = UART_TX_PIN | UART_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = UART_GPIO_SPEED;
	GPIO_InitStruct.Alternate = GPIO_AF_UART;
	HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);

	/* System interrupt init*/
	HAL_NVIC_SetPriority(UART_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(UART_IRQn);

	/* Disable the peripheral */
	UART_INSTANCE->CR1 &=  ~USART_CR1_UE;

	/*------- UART-associated USART registers setting : CR2 Configuration ------*/
	/* Configure the UART Stop Bits: Set STOP[13:12] bits according
	 * to huart->Init.StopBits value */
	MODIFY_REG(UART_INSTANCE->CR2, (0x3U << 12U), (0x00000000U));

	/*------- UART-associated USART registers setting : CR1 Configuration ------*/
	/* Configure the UART Word Length, Parity and mode: */
	tmpreg = (uint32_t)((0x1U << 3U) | (0x1U << 2U));
	MODIFY_REG(UART_INSTANCE->CR1,
						 (uint32_t)((0x1U << 12) | (0x1U << 10) | (0x1U << 9) | (0x1U << 3) | (0x1U << 2) | (0x1U << 15)),
						 tmpreg);

	/*------- UART-associated USART registers setting : CR3 Configuration ------*/
	/* Configure the UART HFC: Set CTSE and RTSE bits */
	MODIFY_REG(UART_INSTANCE->CR3, ((0x1U << 8) | (0x1U << 9)), 0x00000000U);

	/*---Configure Baudrate BRR register---*/
	UART_INSTANCE->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), 9600);

	/* In asynchronous mode, the following bits must be kept cleared:
	     - LINEN and CLKEN bits in the USART_CR2 register,
	     - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
	CLEAR_BIT(UART_INSTANCE->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
	CLEAR_BIT(UART_INSTANCE->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

	/* Enable the peripheral */
	UART_INSTANCE->CR1 |=  USART_CR1_UE;
  
	/* Create mutex for protecting concurrency on uart Tx */
	uartTxSem = osMutexCreate(osMutex(uartTxSem));

	/* Create RX queue between RX ISR and task */
	uartRxQ_ID = osMessageCreate(osMessageQ(uartRxQ), NULL);
		
	/* Enable the UART Data Register not empty Interrupt */
	UART_INSTANCE->CR1 |= USART_CR1_RXNEIE;
}

/**
  * @brief uart sending string by polling
  * @param *stringToSend: pointer to the string to be send.
  * @retval none
  */
void MESN_UART_PutString_Poll(uint8_t *stringToSend)
{
	int32_t i = 0;

	/* wait for uart tx to be free */
	osMutexWait(uartTxSem, osWaitForever);

  //Send data
  for (i=0; stringToSend[i] != '\0'; i++){
		MESN_PRIV_UART_PutChar_Poll(stringToSend[i] );
	}
/* free uart tx */
	osMutexRelease(uartTxSem);

}

/**
  * @brief uart receiving string by interruption and circular buffer
  * @param *rxstring : pointer to a buffer where the received data have
  * to be saved.
  */
MeSN_StatusTypedef MESN_UART_GetString(uint8_t *rxString, uint32_t timeOut){
	
	MeSN_StatusTypedef retVal = USER_OK;

	retVal = MESN_PRIV_UART_GetChar(rxString, timeOut);
	
	while ( (*rxString  != '\r') && (retVal == USER_OK) ){
		rxString++;
		retVal = MESN_PRIV_UART_GetChar(rxString, timeOut);
	}
	
	/* replace ENTER key ascii code '\r' by '\0' */
	if (*rxString  == '\r'){
		*rxString = '\0';
	}
	
	return retVal;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief ISR dedicated to manage RX IRQ
  * @param none
  */
static void MESN_PRIV_UART_IRQHandler(void)
{
	uint8_t tmp;

	/* check the source of IRQ */
	// Data received IRQ
	if((UART_INSTANCE->STATUS_REG & RX_NEMTPY_BIT) != 0)
	{ 
		//Retrieve received data
		tmp = (uint8_t) UART_INSTANCE->RX_DATA_REG;

		// save data in circular buffer if is not full
		if(osMessagePut(uartRxQ_ID, tmp, 0) != osOK ){
			// no flow control (hard/soft)
		}
	}
}

/**
	* @brief ISR function according to prototype defined in stm32lxx_startup.s .
	*/
#if defined(USE_UART3)
  void USART3_IRQHandler(void)
  {
    MESN_PRIV_UART_IRQHandler();
  }
#elif defined(USE_UART2)
  void USART2_IRQHandler(void)
  {
		MESN_PRIV_UART_IRQHandler();
  }
#endif

/**
  * @brief sends 8bits payload through UART by polling
  * @param dataTosend: byte to be send
  * @retval none
  */
static void MESN_PRIV_UART_PutChar_Poll(uint8_t dataToSend)
{
	// check if transmitter is ready to send
	while((UART_INSTANCE->STATUS_REG & TX_EMPTY_BIT) == 0);
	UART_INSTANCE->TX_DATA_REG = (uint8_t)(dataToSend & 0xFF);
}
	
/**
  * @brief check if any data has been received in the queue.
            Queue is feeded by UART RX ISR.
  * @param *rxData : pointer to a buffer which will store the received data
  * @param timeout : amount of time that the function should wait a data before returning
  * @retval received data byte
  */
static MeSN_StatusTypedef MESN_PRIV_UART_GetChar(uint8_t* rxData, uint32_t timeOut) {
	MeSN_StatusTypedef retVal = USER_OK;
	osEvent		event;

	/* wait for avalaible data */
	event = osMessageGet(uartRxQ_ID, timeOut);
	if (event.status == osEventMessage){
		*rxData = event.value.v;
		retVal = USER_OK;
	}
	else {
		*rxData = 0;
		retVal = USER_TIMEOUT;
	}
	
	return retVal;
}

/****END OF FILE****/
