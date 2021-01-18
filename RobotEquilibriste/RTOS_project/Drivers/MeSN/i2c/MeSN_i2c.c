/**
  ******************************************************************************
  * File Name          : I2C.c
  * Date               : 17/02/2015 11:25:49
  * Description        : This file provides code for the configuration and
  *                      usage of the I2C instances with RTOS integration.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MeSN_i2c.h"

#include "cmsis_os.h"

/* Private TypeDefs ----------------------------------------------------------*/
typedef enum 
{  
  I2C_NOT_READY = 0,
  I2C_READY = 1,
  I2C_NEVER_USED = 2,
	I2C_ERROR = 3
}i2cState_TypeDef;

/* Privates Variables --------------------------------------------------------*/
static I2C_HandleTypeDef i2cCtrl_handle;

static i2cState_TypeDef i2cCtrlStatus = I2C_NEVER_USED;

static osMutexId i2cMutex_ID;		//Protect I2C from concurrency
static osSemaphoreId i2cTxCpltSem_ID;					//Sync between ISR and task for I2C TX (send data)
static osSemaphoreId i2cRxCpltSem_ID;					//Sync between ISR and task for I2C RX (receive data)
static osSemaphoreId i2cMemReadCpltSem_ID;		//Sync between ISR and task for I2C MemRead (Send query then receive response)

/* Private Function prototypes -----------------------------------------------*/
static MeSN_StatusTypedef MX_I2C_Init(void);

/* Public Function -----------------------------------------------------------*/

/**
	* @brief  Init I2C peripheral dedicated to communication with control sensors
	* @param  None
  * @retval None
  */
MeSN_StatusTypedef MeSN_I2C_Init(void){

	MeSN_StatusTypedef retVal = USER_ERROR;
	
	/* Init I2C bus if needed */
	if (i2cCtrlStatus == I2C_NEVER_USED){	/* I2C soft object must be created */
		
		/* Init OS control object */
		/* definition and creation of i2cTxSem */
		osMutexDef(i2cMutex);
		i2cMutex_ID = osMutexCreate(osMutex(i2cMutex));
		/* definition and creation of i2c Cplt Sem */
		osSemaphoreDef(i2cRxCpltSem);
		i2cRxCpltSem_ID = osSemaphoreCreate(osSemaphore(i2cRxCpltSem), 1);
		osSemaphoreDef(i2cTxCpltSem);
		i2cTxCpltSem_ID = osSemaphoreCreate(osSemaphore(i2cTxCpltSem), 1);
		osSemaphoreDef(i2cMemReadCpltSem);
		i2cMemReadCpltSem_ID = osSemaphoreCreate(osSemaphore(i2cMemReadCpltSem), 1);		
		
		/* I2C bus is free at startup */
		osMutexRelease(i2cMutex_ID);
		/* reset sync flag between task and ISR */
		osSemaphoreWait(i2cRxCpltSem_ID, 0);
		osSemaphoreWait(i2cTxCpltSem_ID, 0);
		osSemaphoreWait(i2cMemReadCpltSem_ID, 0);
		
		i2cCtrlStatus = I2C_NOT_READY;
	}
	
	if (i2cCtrlStatus == I2C_NOT_READY){	/*I2C bus must be configured*/
		/* Init I2C peripheral */
		if(MX_I2C_Init() != USER_OK){
			i2cCtrlStatus = I2C_ERROR;
			retVal = USER_ERROR;
		}
		else{
			i2cCtrlStatus = I2C_READY;
		}
	}
	
	if (i2cCtrlStatus == I2C_READY){	/*I2C bus already configured*/
		retVal = USER_OK;
	}

	return retVal;
}

/**
	* @brief  DeInit I2C peripheral dedicated to communication with control sensors
	* @param  None
  * @retval None
  */
void MeSN_I2C_DeInit(void){
	
  HAL_I2C_DeInit(&i2cCtrl_handle);
  
  i2cCtrlStatus = I2C_NOT_READY;
}

/**
  * @brief  Transmit in master mode an amount of data in no-blocking mode with Interrupt
  * @param  DevAddress: Target device address
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @retval Status
  */
MeSN_StatusTypedef MeSN_I2C_Write(uint16_t DevAddress, uint8_t *pData, uint16_t Size){
	
	MeSN_StatusTypedef status = USER_OK;
	uint8_t slaveAddr = DevAddress<<1;	//The device 7 bits address value in datasheet must be shift at right before call HAL interface
	
	/* Wait for Free I2C bus */
	if (osMutexWait(i2cMutex_ID, I2C_TIMEOUT) == osErrorOS){
		status = USER_TIMEOUT;
	}
	
	/* Clear Tx completed semaphore */
	osSemaphoreWait(i2cTxCpltSem_ID, 0);
	
	/* Transmit data */
	//PROTO: HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
	if(HAL_I2C_Master_Transmit_IT(&i2cCtrl_handle, slaveAddr, pData, Size) != HAL_OK)
	{
		/* Error Occured : check I2C_HandleTypeDef.ErrorCode to obtain details */
		/* try to reset the bus... */
		//i2cCtrl_handle.Instance->CR1 |= I2C_CR1_SWRST;		//Force reset state
		//i2cCtrl_handle.Instance->CR1 &= ~I2C_CR1_SWRST;		//Release reset state

		status = USER_ERROR;
	} else {	
		//wait for end of transfert
		if (osSemaphoreWait(i2cTxCpltSem_ID, I2C_TIMEOUT) == osErrorOS){
			status = USER_TIMEOUT;
		}
	}

	/* Free the bus to enable further communication */
	osMutexRelease(i2cMutex_ID);

	return status;
}

/**
  * @brief  Receive in master mode an amount of data in no-blocking mode with Interrupt
  * @param  DevAddress: Target device address
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @retval Status
  */
MeSN_StatusTypedef MeSN_Read(uint16_t DevAddress, uint8_t *pData, uint16_t Size){
	
	MeSN_StatusTypedef status = USER_OK;
	uint8_t slaveAddr = DevAddress<<1;	//The device 7 bits address value in datasheet must be shift at right before call HAL interface
	
	/* Wait for Free I2C bus */
	if (osMutexWait(i2cMutex_ID, I2C_TIMEOUT) == osErrorOS){
		status = USER_TIMEOUT;
	}
	
	/* Clear Rx completed semphore */
	osSemaphoreWait(i2cRxCpltSem_ID, 0);
	
	/* Transmit data */
	//PROTO: HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
	if(HAL_I2C_Master_Receive_IT(&i2cCtrl_handle, slaveAddr, pData, Size) != HAL_OK)
	{
		/* Error Occured : check I2C_HandleTypeDef.ErrorCode to obtain details */
		/* try to reset the bus... */
		//i2cCtrl_handle.Instance->CR1 |= I2C_CR1_SWRST;		//Force reset state
		//i2cCtrl_handle.Instance->CR1 &= ~I2C_CR1_SWRST;		//Release reset state

		status = USER_ERROR;
	} else {
		/* Wait for Rx completed */
		if (osSemaphoreWait(i2cRxCpltSem_ID, I2C_TIMEOUT) == osErrorOS){
			status = USER_TIMEOUT;
		}
	}
	
	/* Free the bus to enable further communication */
	osMutexRelease(i2cMutex_ID);
	
	return status;
}

/**
  * @brief  Read an amount of data in no-blocking mode with Interrupt from a specific register address
  * @param  DevAddress: Target device I2C address
  * @param  RegAddress: Internal register address
  * @param  RegAddSize: Size of internal register address
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be read
  * @retval Status
  */
MeSN_StatusTypedef MeSN_I2C_RegisterRead(uint16_t DevAddress, uint16_t RegAddress, uint16_t RegAddSize, uint8_t *pData, uint16_t Size){
	
	MeSN_StatusTypedef status = USER_OK;
	uint8_t slaveAddr = DevAddress<<1;	//The device 7 bits address value in datasheet must be shift at right before call HAL interface
	
	/* Wait for Free I2C bus */
	if (osMutexWait(i2cMutex_ID, I2C_TIMEOUT) == osErrorOS){
		status = USER_TIMEOUT;
	}
	
	/* Clear MemRead completed semaphore */
	osSemaphoreWait(i2cMemReadCpltSem_ID, 0);
	
	/* Ask for data and then read */
	//PROTO: HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
	if(HAL_I2C_Mem_Read_IT(&i2cCtrl_handle, slaveAddr, RegAddress, RegAddSize, pData, Size) != HAL_OK)
	{
		/* Error Occured : check I2C_HandleTypeDef.ErrorCode to obtain details */
		/* try to reset the bus... */
		//i2cCtrl_handle.Instance->CR1 |= I2C_CR1_SWRST;		//Force reset state
		//i2cCtrl_handle.Instance->CR1 &= ~I2C_CR1_SWRST;		//Release reset state
		status = USER_ERROR;
	}
	else{
		/* Wait for MemRead completed */
		if (osSemaphoreWait(i2cMemReadCpltSem_ID, I2C_TIMEOUT) == osErrorOS){
			status = USER_TIMEOUT;
		}
	}
	/* Free the bus to enable further communication */
	osMutexRelease(i2cMutex_ID);
	
	return status;
}

/* Private Function ----------------------------------------------------------*/

/**
	* @brief  Configure the low level I2C peripheral according to external
	*					components needs :
	*					Admissible clock = 100kHz or 400kHz
	*					Address bits = 7bits
	* @param  None
  * @retval None
  */
static MeSN_StatusTypedef MX_I2C_Init(void)
{
	MeSN_StatusTypedef retVal = USER_ERROR;

	i2cCtrl_handle.Instance = I2C_CTRL_PP;
  i2cCtrl_handle.Init.ClockSpeed = I2C_CTRL_CLOCK;
  i2cCtrl_handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
  i2cCtrl_handle.Init.OwnAddress1 = 0;
  i2cCtrl_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  i2cCtrl_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  i2cCtrl_handle.Init.OwnAddress2 = 0;
  i2cCtrl_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  i2cCtrl_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  if( HAL_I2C_Init(&i2cCtrl_handle) != HAL_OK){
  	retVal = USER_ERROR;
  }
  else{
  	retVal = USER_OK;
  }

  return retVal;
}

/**
  * @brief  Initializes the I2C MCU Specific Package.
	*					ie CLOCKS/GPIO/IRQ/DMA
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module.
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C_CTRL_PP)
  {
    /* Peripheral clock enable */
    I2C_CTRL_CLK_ENABLE();
    I2C_CTRL_GPIO_CLK_ENABLE();
  
    /* I2C GPIO Configuration */
    GPIO_InitStruct.Pin = I2C_CTRL_SCL_PIN|I2C_CTRL_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C_CTRL;
    HAL_GPIO_Init(I2C_CTRL_PORT, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(I2C_CTRL_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C_CTRL_EV_IRQn);
		HAL_NVIC_SetPriority(I2C_CTRL_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C_CTRL_ER_IRQn);
  }
}

/**
  * @brief  De-Initializes the I2C MCU Specific Package.
	*					ie CLOCKS/GPIO/IRQ/DMA
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module.
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C_CTRL_PP)
  {
    /* Peripheral clock disable */
    I2C_CTRL_CLK_DISABLE();
    /**I2C1 GPIO Configuration */
    HAL_GPIO_DeInit(I2C_CTRL_PORT, I2C_CTRL_SCL_PIN|I2C_CTRL_SDA_PIN);
    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(I2C_CTRL_EV_IRQn);
		HAL_NVIC_DisableIRQ(I2C_CTRL_ER_IRQn);
  }
} 

/* IRQ Management -------------------------------------------------------*/
#if defined(USE_I2C1)
	/**
	* @brief This function handles I2C1 event interrupt.
	*/
	void I2C1_EV_IRQHandler(void)
	{
		HAL_I2C_EV_IRQHandler(&i2cCtrl_handle);
	}

	/**
	* @brief This function handles I2C1 event interrupt.
	*/
	void I2C1_ER_IRQHandler(void)
	{
		HAL_I2C_ER_IRQHandler(&i2cCtrl_handle);
	}
#elif defined(USE_I2C2)
	/**
	* @brief This function handles I2C2 event interrupt.
	*/
	void I2C2_EV_IRQHandler(void)
	{
		HAL_I2C_EV_IRQHandler(&i2cCtrl_handle);
	}

	/**
	* @brief This function handles I2C2 event interrupt.
	*/
	void I2C2_ER_IRQHandler(void)
	{
		HAL_I2C_ER_IRQHandler(&i2cCtrl_handle);
	}
#endif
	
/**
  * @brief  Master Tx Transfer completed callbacks.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Set Tx completed flag */
	osSemaphoreRelease(i2cTxCpltSem_ID);
}

/**
  * @brief  Master Rx Transfer completed callbacks.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Set Rx completed flag */
	osSemaphoreRelease(i2cRxCpltSem_ID);
}

/**
  * @brief  Memory Rx Transfer completed callbacks.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Set MemRead completed flag */
	osSemaphoreRelease(i2cMemReadCpltSem_ID);
}

/**
  * @brief  I2C error callbacks.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /* Error Occured : Should reinit the bus ?? */
	// try to reset the bus...
	i2cCtrl_handle.Instance->CR1 |= I2C_CR1_SWRST;		//Force reset state
	i2cCtrl_handle.Instance->CR1 &= ~I2C_CR1_SWRST;		//Release reset state
	
	// release semaphores to unlock task (be carreful with erroneous data)
	osSemaphoreRelease(i2cMemReadCpltSem_ID);
	osSemaphoreRelease(i2cRxCpltSem_ID);
	osSemaphoreRelease(i2cTxCpltSem_ID);
}

/********END OF FILE****/
