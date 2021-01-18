/***************************************************************************//**
	* @file errorStatus.h
	* @brief Header file of error definition for error tracking definition.
	********************************************************************************
	*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __errorStatus_H
#define __errorStatus_H

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  USER Status structures definition  
  */  
typedef enum 
{
  USER_OK       = 0x00,
  USER_ERROR    = 0x01,
  USER_TIMEOUT  = 0x02
} MeSN_StatusTypedef;

#endif /*__errorStatus_H */

/**********************END OF FILE ************/
