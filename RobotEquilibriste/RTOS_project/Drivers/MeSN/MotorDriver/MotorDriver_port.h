/**
  ******************************************************************************
  * @file    MotorDriver_port.h
  * @author  
  * @version V1.0.0
  * @date    
  * @brief   This file contains function prototypes and pinout assignation of
	*					 portable layer to manage low level operations of the motor bridge.
  ******************************************************************************
  */
	
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MotorDriver_Port_H
#define __MotorDriver_Port_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* External Variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim10;	//Peripheral handle for Timer (usually htimXX)

/* Exported Typedef-----------------------------------------------------------*/
typedef enum
{
  GPIO_DIRPIN_LOW = 0,
  GPIO_DIRPIN_HIGH
}GPIO_DirPinState;

/* Exported Constants---------------------------------------------------------*/
/*	Motor Pinout:
		Motor direction IN1 --> PB10 : output PP
		Motor direction IN2 --> PB11 : output PP
		MotorD PWM out --> PB12 	(Timer 10 channel 1)
*/
#define MOTOR_DIR_PORT											GPIOB
#define MOTOR_PWM_PORT											GPIOB

#define MOTOR_IN1_PIN											  GPIO_PIN_10
#define MOTOR_IN2_PIN											  GPIO_PIN_11
#define MOTOR_PWM_PIN												GPIO_PIN_12

#define MOTOR_PWM_TIM_INSTANCE							TIM10
#define MOTOR_PWM_TIM_CHANNEL								TIM_CHANNEL_1

#define PWM_FREQ						20000						//Desired PWM frequency
#define PWM_SRC_FREQ				16000000				//Timer counting frequency
#define TIMER_MAX_VAL				0xFFFF

#define TIMER_RELOAD_VAL			__HAL_TIM_GET_AUTORELOAD(&htim10)	//Number of count before overflow !! Must be <= TIMER_MAX_VAL
#define PWM_DUTYCYCLE_FULL_SCALE		(uint16_t) ( TIMER_RELOAD_VAL )		//100% duty cycle value

/* Exported Fonctions---------------------------------------------------------*/

/**
	* @brief  Configure les broches GPIO concernant le sens de rotation:
	*         2 broches sont requises, en mode output pushpull.
  * @param  None
  * @retval None
  */
void MotorDriver_Port_GPIO_Init(void);

/**
	* @brief  Configure le Timer permettant la generation de la PWM:
	*					1 broche est requise, en mode PWM.
  * @param  None
  * @retval None
  */
void MotorDriver_Port_PWM_Init(void);


/**
	* @brief  Fixe l'etat de la broche IN1
	*         en fonction de la valeur passee en parametre.
	* @param  pinState : GPIO_DIRPIN_LOW ou GPIO_DIRPIN_HIGH
  * @retval None
  */
void MotorDriver_Port_SetPin_IN1(GPIO_DirPinState pinState);

/**
	* @brief  Fixe l'etat de la broche IN2
	*         en fonction de la valeur passee en parametre.
	* @param  pinState : GPIO_DIRPIN_LOW ou GPIO_DIRPIN_HIGH
  * @retval None
  */
void MotorDriver_Port_SetPin_IN2(GPIO_DirPinState pinState);

/**
	* @brief  Regle le rapport cyclique de la PWM
	* 				en fonction de la valeur passee en parametre.
	* @param  dutyCycle : rapport cyclique souhaite (en "pour mille").
	*         Doit etre compris entre 0 et PWM_DUTYCYCLE_FULL_SCALE
  * @retval None
  */
void MotorDriver_Port_SetPWM(uint32_t dutyCycle);

#endif /*__MotorDriver_Port_H */

/**********END OF FILE****/
