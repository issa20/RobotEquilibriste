/**
  ******************************************************************************
  * @file    MotorDriver.c
  * @author  Basile Dufay
  * @version V1.0.0
  * @date    18-Sept-2017
  * @brief   This file provides a set of firmware functions to manage:
  *          + Sparkfun ROB-09457 dual Motor Bridge based on TB6612FNG 
	*					 + These functions required 2 GPIO access and 1 PWM generation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MotorDriver.h"					//check matching between prototype and implementation
#include "MotorDriver_port.h"			//access to portable layer API

/* Private Typedefs ----------------------------------------------------------*/
typedef enum 
{  
  STOP = 0,		//Stop rotation
  CW = 1,			//Clock wise rotation
	CCW = 2,		//CounterClock wise rotation
}RotDir_TypeDef;

/* Private prototypes --------------------------------------------------------*/
static inline uint32_t 					MotorDriver_Absolute(int32_t SignedNum);
static inline RotDir_TypeDef 		MotorDriver_RotDir(int32_t SignedNum);
static void 										MotorDriver_SetDir(RotDir_TypeDef rotDir);
static void											MotorDriver_SetSpeed(uint32_t rotSpeed);

/* Public functions ----------------------------------------------------------*/

/**
  * cf fichier header
  */
void MotorDriver_Init()
{
	/* Init 2 GPIO pins to manage motor direction */
	MotorDriver_Port_GPIO_Init();
	/* Init 1 PWM generation to manage motor speed */
	MotorDriver_Port_PWM_Init();
}


/**
  * cf fichier header
  */
void MotorDriver_Move(int32_t speed)
{
	RotDir_TypeDef RotDir;
	uint32_t RotSpeed;
	
	/* Extract rotation direction and speed*/
	RotSpeed = MotorDriver_Absolute(speed);
	RotDir = MotorDriver_RotDir(speed);
	
	/* Check parameters */
	if (RotSpeed > MAX_SPEED){	//Speed value exceeds limits
		RotSpeed = MAX_SPEED;
	}
	
	/* Apply motor action */
	MotorDriver_SetDir(RotDir);
	MotorDriver_SetSpeed(RotSpeed);
	
}

/**
  * cf fichier header
  */
void MotorDriver_Stop(void)
{
	/* Apply motor action */
	MotorDriver_SetDir(STOP);
	MotorDriver_SetSpeed(0);
}


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Retourne la valeur absolue d'un nombre signe.
  * @param  SignedNum: nombre signe dont il faut extraire la valeur absolue.
  * @retval nombre non-signe correspondant a la valeur absolue du param entrant.
  */
static inline uint32_t MotorDriver_Absolute(int32_t SignedNum)
{
/* compute absolute value of int32 argument */
return (unsigned) (SignedNum < 0 ? -SignedNum : SignedNum);
}

/**
  * @brief  Retourne le sens de rotation en fonction du signe de la vitesse.
  * @param  SignedNum: nombre signe representant la vitesse de rotation.
  * @retval valeur du type RotDir_TypeDef deduit du signe du param entrant.
  */
static RotDir_TypeDef MotorDriver_RotDir(int32_t Speed)
{
	RotDir_TypeDef dir = STOP;
	
	if (Speed == 0){
		dir = STOP;
	}
	else if (Speed > 0){
		dir = CW;
	}
	else if (Speed < 0) {
		dir = CCW;
	}
	
	return dir;
}

/**
  * @brief  Modifie les broches IN1 et IN2 en fonction du sens de rotation.
  * @param  rotDir: sens de rotation
	*   Ce parametre peut prendre les valeurs suivantes:
	*     @arg CW	:	clock wise (sens horaire)
	*			@arg CCW : counterColck wise (sens anti-horaire)
  * @retval none
  */
static void MotorDriver_SetDir(RotDir_TypeDef rotDir){
	/* Always set pin low first before inverting direction to ensure
	   a "brake to ground" state during transition */
	if (rotDir == CW){
		MotorDriver_Port_SetPin_IN2(GPIO_DIRPIN_LOW);
		MotorDriver_Port_SetPin_IN1(GPIO_DIRPIN_HIGH);
	}
	else if (rotDir == CCW){
		MotorDriver_Port_SetPin_IN1(GPIO_DIRPIN_LOW);
		MotorDriver_Port_SetPin_IN2(GPIO_DIRPIN_HIGH);
	}
	else if (rotDir == STOP){
		MotorDriver_Port_SetPin_IN2(GPIO_DIRPIN_LOW);
		MotorDriver_Port_SetPin_IN1(GPIO_DIRPIN_LOW);
	}
}

/**
  * @brief  Modifie le rapport cyclique de la PWM en fonction de la vitesse.
  * @param  rotSpeed: vitesse de rotation souhaitee
	*   Ce parametre doit etre une valeur positive comprise entre 0 et MAX_SPEED
  * @retval none
  */
static void MotorDriver_SetSpeed(uint32_t rotSpeed){

	MotorDriver_Port_SetPWM( (uint32_t) ((rotSpeed*PWM_DUTYCYCLE_FULL_SCALE)/MAX_SPEED));

}

/**********END OF FILE****/
