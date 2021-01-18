/**
  ******************************************************************************
  * File Name          : observateur.c
  * Date               : 17/02/2015 11:25:49
  * Description        : Header de l'observateur sur l'angle du robot
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OBSERVATEUR_H
#define __OBSERVATEUR_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

#define ARM_MATH_CM3		//Must be defined before inclusion of arm_math.h.
												//To be adjusted according to core family.

#include "arm_math.h"

/* Exported Constants---------------------------------------------------------*/
/* Exported Fonctions---------------------------------------------------------*/
/**
	* @brief Computes tilt angle of the robot according to observator algorithm.
	* @param acc_mg : 32-bit integer of the sensed acceleration value in milli-g (1 g ~ 9.81 m per square second).
	* @param rotAng_mDegSec : 32-bit integer of the sensed rotation value in milli-deg per second.
	* @return 32-bit integer value of the computed tilt angle in milli-degrees.
	*/
int32_t autoAlgo_angleObs(int32_t acc_mg, int32_t rotAng_mDegSec);

/**
	* @brief Computes command value to be applied on robot's wheel according to regulation algorithm.
	* @param tiltAngle_mDeg : 32-bit integer of the tilt angle value of the robot in milli-degrees.
	* @param rotAng_mDegSec : 32-bit integer of the sensed rotation value in milli-deg per second.
	* @return 32-bit integer value of the computed command (expressed in per thousand : in the range +/-1000).
	*/
int32_t autoAlgo_commandLaw(int32_t tiltAngle_mDeg, int32_t rotAng_mDegSec);

#endif /*__OBSERVATEUR_H */

/**********END OF FILE****/
