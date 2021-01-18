/**
  ******************************************************************************
  * File Name          : MeSN_I2C.h
  * Date               : 08/01/2018
  * Description        : 
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

#include "errorStatus.h"

/* Constants -----------------------------------------------------------------*/
//#define		USE_I2C2
#define			USE_I2C1

#if defined(USE_I2C1)
	#define I2C_CTRL_PP												I2C1
	/*PB8     ------> I2C1_SCL
	  PB9     ------> I2C1_SDA */
	#define I2C_CTRL_PORT											GPIOB
	#define I2C_CTRL_SCL_PIN									GPIO_PIN_6
	#define I2C_CTRL_SDA_PIN									GPIO_PIN_7
	#define I2C_CTRL_CLK_ENABLE()							__I2C1_CLK_ENABLE()
	#define	I2C_CTRL_GPIO_CLK_ENABLE()				__HAL_RCC_GPIOB_CLK_ENABLE();
	#define I2C_CTRL_CLK_DISABLE()						__I2C1_CLK_DISABLE()
	#define GPIO_AF4_I2C_CTRL									GPIO_AF4_I2C1
	#define I2C_CTRL_EV_IRQn									I2C1_EV_IRQn
	#define I2C_CTRL_ER_IRQn									I2C1_ER_IRQn
#elif defined(USE_I2C2)
	#define I2C_CTRL_PP												I2C2
	/*PB10     ------> I2C1_SCL
		PB11     ------> I2C1_SDA */
	#define I2C_CTRL_PORT											GPIOB
	#define I2C_CTRL_SCL_PIN									GPIO_PIN_10
	#define I2C_CTRL_SDA_PIN									GPIO_PIN_11
	#define I2C_CTRL_CLK_ENABLE()							__I2C2_CLK_ENABLE()
	#define	I2C_CTRL_GPIO_CLK_ENABLE()				__HAL_RCC_GPIOB_CLK_ENABLE();
	#define I2C_CTRL_CLK_DISABLE()						__I2C2_CLK_DISABLE()
	#define GPIO_AF4_I2C_CTRL									GPIO_AF4_I2C2
	#define I2C_CTRL_EV_IRQn									I2C2_EV_IRQn
	#define I2C_CTRL_ER_IRQn									I2C2_ER_IRQn
#else
	#error "Please select I2C instance"
#endif

#define I2C_CTRL_CLOCK										400000	//in Hz
#define I2C_TIMEOUT												200			//in osTick (ms)

/* Public Function Prototypes ------------------------------------------------*/
MeSN_StatusTypedef MeSN_I2C_Init(void);
void MeSN_I2C_DeInit(void);
MeSN_StatusTypedef MeSN_I2C_Write(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
MeSN_StatusTypedef MeSN_Read(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
MeSN_StatusTypedef MeSN_I2C_RegisterRead(uint16_t DevAddress, uint16_t RegAddress, uint16_t RegAddSize, uint8_t *pData, uint16_t Size);

#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
