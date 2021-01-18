/**
  ******************************************************************************
  * @file    MotorDriver.h
  * @author  Basile Dufay
  * @version V1.0.0
  * @date    18-Sept-2017
  * @brief   This file contains all the functions prototypes for driver layer.
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MotorDriver_H
#define __MotorDriver_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"		/* Portable definition of C integer type */

/* Exported TypeDefs -------------------------------------------------------- */

/* Exported Constants---------------------------------------------------------*/
#define MAX_SPEED 				1000

/* Exported public Fonctions--------------------------------------------------*/

/**
  * @brief  Configure le materiel bas-niveau necessaire au TB6612FNG.
  * @param  None
  * @retval None
  */
void MotorDriver_Init(void);

/**
  * @brief  Fixe la vitesse de rotation du moteur
  * @param  speed: Indique la vitesse a appliquer
	*					Ce parametre doit etre compris entre -MAX_SPEED et +MAX_SPEED
	*					dont le signe represente le sens de rotation.
  * @retval None
  */
void MotorDriver_Move(int32_t speed);

/**
  * @brief  Stoppe le moteur
  * @param  None
  * @retval None
  */
void MotorDriver_Stop(void);

#endif /*__MotorDriver_H */

/**********END OF FILE****/
