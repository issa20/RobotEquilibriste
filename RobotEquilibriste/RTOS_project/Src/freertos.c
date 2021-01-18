/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "MotorDriver.h"								// Pilote du moteur
#include "lsm6ds3.h"										// Pilote de la centrale inertielle
#include "mesn_uart.h"									// Pilote de la liaison UART
#include "libSBR_autom_obs-corr.h"			// Algo observateur et correcteur
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osMutexDef (Mutex);// mutex pour la protection des GPIO
osMutexId mutexGPIO ;

osMessageQDef ( queue , 2, uint32_t ); //   queue pour synchroniser entre la tache StartUART et vTaskAsservissement
osMessageQId queue ;

osMessageQDef ( queueAngle , 2, uint32_t ); // queueAngle queue pour echanger la valeur de l'angle entre la tache de vTaskAsservissement et Enregistrement
osMessageQId queueAngle ;

osMessageQDef ( queueStreamAngle , 2, uint32_t ); // queueAngle queue pour echanger la valeur de l'angle entre la tache de vTaskAsservissement et Enregistrement
osMessageQId queueStreamAngle ;

uint32_t ValAngle[100] ; // tableau de 100 dernier valeur de l'angle
uint32_t indice=0 ; //indice de tableau de valeur de angle
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartUART(void const * argument);// tache initilise les capteur et gere la commmunication UART
void vTaskAsservissement(void const * argument);//tache calcul la valeur de l'angle et la vitesse et commande le moteur
void Enregistrement(void const * argument) ;// tache fait l'enregistrement de valeur de l'angle de le tableau et fait le clignotement des led si la valeur de angle est inferieur de 25

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */


/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
 void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	MESN_UART_PutString_Poll((uint8_t*)"\r\nERROR : stack overflow from task");
	MESN_UART_PutString_Poll((uint8_t*)pcTaskName);
	while(1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
 void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	MESN_UART_PutString_Poll((uint8_t*)"\r\nERROR : Heap full!");
	while(1);
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
	queue = osMessageCreate ( osMessageQ ( queue ) , NULL );//   queue pour synchroniser entre la tache StartUART et vTaskAsservissement
	queueAngle= osMessageCreate ( osMessageQ ( queueAngle ) , NULL );// queueAngle queue pour echanger la valeur de l'angle entre la tache de vTaskAsservissement et Enregistrement
	queueStreamAngle= osMessageCreate ( osMessageQ ( queueStreamAngle ) , NULL );

	mutexGPIO =  osMutexCreate  (osMutex (Mutex));//mutex pour l'approtection des GPIO
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
	  osThreadDef(task1, vTaskAsservissement, osPriorityNormal, 0, 200);
	  defaultTaskHandle = osThreadCreate(osThread(task1), NULL);// tache initilise les capteur et gere la commmunication UART


	  osThreadDef(task2, StartUART, osPriorityNormal, 0, 200);
	  defaultTaskHandle = osThreadCreate(osThread(task2), NULL);//tache pour le calcul la valeur de l'angle et la vitesse et commande le moteur

  	  osThreadDef(task3, Enregistrement, osPriorityNormal, 0, 200);
  	  defaultTaskHandle = osThreadCreate(osThread(task3), NULL);// tache fait l'enregistrement de valeur de l'angle de le tableau et fait le clignotement des led si la valeur de angle est inferieur de 25
  //new tache

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartUART(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	uint8_t LSM6DS3_Res = 0;
	uint8_t tempString[100];
	uint8_t message[100];

	/* Init des periphs externes */
	MotorDriver_Init();
	MESN_UART_Init();
	if(LSM6DS3_begin(&LSM6DS3_Res) != IMU_SUCCESS){
		MESN_UART_PutString_Poll((uint8_t*)"\r\nIMU Error !");
		while(1);
	}

	/* Test des periphs externes */
	sprintf((char*)tempString, "\r\nInit done. LSM6DS3 reg = %02x", LSM6DS3_Res);
	MESN_UART_PutString_Poll(tempString);
	MotorDriver_Move(200);

	/* Test algo autom */
	sprintf((char*)tempString, "\r\nAngle = %ldmDeg", autoAlgo_angleObs(50,5));
	MESN_UART_PutString_Poll(tempString);
	osMessagePut ( queue , 8 , 0) ;//apres l'initialisation des capteurs on debloque la tache vTaskAsservissement pour pouvoir utiliser les capteurs
  /* Infinite loop */
  for(;;)
  {
    MESN_UART_GetString(message,osWaitForever);//lecture de commande saisie
    if(strcmp((char*)message,"read")==0){//test est ce que la commande saisie est read
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n **read**");
    	sprintf((char *)message,"\r\n %ld mdeg",ValAngle[indice]);
    	MESN_UART_PutString_Poll(message);//affichage de la dernier valeur de angle
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n ");

    }
    if(strcmp((char*)message,"dump")==0){//test est ce que la commande saisie est dump
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n **dump**");
    	int etat=1,init=indice;
        	while(etat){
        		if(++indice==100)indice=0;//tableau cyclique
        		sprintf((char *)message,"\r\n %ld mdeg",ValAngle[indice]);
        		MESN_UART_PutString_Poll(message);//affichage de 100 valeur de angles en commancant par la plus ancienne valeur qui se trouve a init + 1 jusqu'a init (tableau cyclique)
        		MESN_UART_GetString(message,0);
        		if(indice==init)etat=0;
        	}
        	MESN_UART_PutString_Poll((uint8_t*)"\r\n ");
    }
    if(strcmp((char*)message,"stream")==0){//test est ce que la commande saisie est stream
    		MESN_UART_PutString_Poll((uint8_t*)"\r\n **stream** \r\n");
    	    int etat=1;
    	    osEvent evt ;
    	    while(etat){
    	    	evt = osMessageGet ( queueAngle , 50 ); // wait for message
    	    	if ( evt.status == osEventMessage ) {
    	    			sprintf((char *)message,"\r %6ld mdeg",evt.value.v);
    	    			 MESN_UART_PutString_Poll(message);//affichage de la dernier valeur de angle jusqu'a appui sur un botton
    	    	}
    	        if(MESN_UART_GetString(message,0)==USER_OK)etat=0;//sortie en cas d'appui sur un botton
    	    }
    	    MESN_UART_PutString_Poll((uint8_t*)"\r\n ");
    }
    if(strcmp((char*)message,"help")==0){//test est ce que la commande saisie est help
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n *******************help********************");
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n read   : return last measured angle value");
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n dump   :  return last hundred angle values");
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n stream : continiously return last measured angle value and update display press ENTER to quit stream mode");
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n help   : print this menu");
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n ********************end********************");
    	MESN_UART_PutString_Poll((uint8_t*)"\r\n ");


    }

  }
  /* USER CODE END StartDefaultTask */
}
void vTaskAsservissement(void const * argument){
	int32_t GyroY,AccelX,Angle,Vitesse;

	osMessageGet ( queue , osWaitForever );
	uint32_t PrevieusTick = osKernelSysTick() ;
	while(1){
		osDelayUntil(&PrevieusTick,10); // synchronisation de 10 pour l'asservissement
		LSM6DS3_readMgAccelX( &AccelX );//recuperation de la valeur d'acceleration par rapport a x
		LSM6DS3_readMdpsGyroY(&GyroY );//recuperation de la valeur de geroscope par rapport a y
		Angle = autoAlgo_angleObs(AccelX,GyroY);//calcul de l'angle
		Vitesse=autoAlgo_commandLaw (Angle,GyroY);//calcul du commande de l'asservissement
		MotorDriver_Move(Vitesse);//commande le moteur
		osMessagePut ( queueAngle,Angle , 0 );//la tache de l'asservissemnt ne doit jamais de bloquer


	}
}
void Enregistrement(void const * argument){
	int32_t NewAngle=0;
	osEvent evt ;
	while(1){

		evt = osMessageGet ( queueAngle , osWaitForever ); // wait for message
		if ( evt.status == osEventMessage ) {
			NewAngle=evt.value.v;
			if(++indice==100)indice=0;//tableau cyclique
			ValAngle[indice]=NewAngle;//enregistrement de la nouvelle valeur de l'angle
			if(indice==100)indice=0;//tableau cyclique
		}
		osMessagePut ( queueStreamAngle,NewAngle , 0 );//envoie dans la queue pour la commande stream
		if(NewAngle<25){
			osMutexWait(mutexGPIO, osWaitForever);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);//clignotement de la led en cas angle <25
			osMutexRelease(mutexGPIO);
		}

	}
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
