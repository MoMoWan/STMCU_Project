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
#include "crc.h"
#include "gpio.h"

#include "BSP_LED.h"
#include "BSP_NRF24L01.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NRF_TX_MAXRETRY 3
#define NRF_RX_MAXRETRY 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId DefaultTaskHandle;
osThreadId NRF_RxTaskHandle;
osThreadId NRF_TxTaskHandle;
osMutexId NRF_MutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartNRF_RxTask(void const * argument);
void StartNRF_TxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of NRF_Mutex */
  osMutexDef(NRF_Mutex);
  NRF_MutexHandle = osMutexCreate(osMutex(NRF_Mutex));

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
  /* definition and creation of DefaultTask */
  osThreadDef(DefaultTask, StartDefaultTask, osPriorityNormal, 0, 64);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* definition and creation of NRF_RxTask */
  osThreadDef(NRF_RxTask, StartNRF_RxTask, osPriorityHigh, 0, 256);
  NRF_RxTaskHandle = osThreadCreate(osThread(NRF_RxTask), NULL);

  /* definition and creation of NRF_TxTask */
  osThreadDef(NRF_TxTask, StartNRF_TxTask, osPriorityHigh, 0, 256);
  NRF_TxTaskHandle = osThreadCreate(osThread(NRF_TxTask), NULL);

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
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		
		
				LED_ON;
				HAL_Delay(3);
				LED_OFF;
		
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartNRF_RxTask */
/**
* @brief Function implementing the NRF_RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNRF_RxTask */
void StartNRF_RxTask(void const * argument)
{
  /* USER CODE BEGIN StartNRF_RxTask */
	 uint8_t Rx_packet[2]={0},RetryTimes=0; 
  __IO uint8_t NRF_RxState=0;
		uint32_t CRC_Buf=0;
		uint32_t CRC_Value=0;
		

  /* Infinite loop */
  for(;;)
  {
		  osMutexWait(NRF_MutexHandle,osWaitForever);
			 NRF24L01_RX_Mode();
	   NRF_RxState=NRF24L01_RxPacket(Rx_packet);
    osMutexRelease(NRF_MutexHandle);		
		
				if( NRF_RxState != RX_OK )
				{
						for(RetryTimes=0;RetryTimes<NRF_RX_MAXRETRY;RetryTimes++)
						{
								osMutexWait(NRF_MutexHandle,osWaitForever);
								NRF_RxState=NRF24L01_RxPacket(Rx_packet);
//							 printf("Rx_packet:%d    Rx_CRC:%2x    Rx_times:%d  RxState:%x\r\n",Rx_packet[0],Rx_packet[1],RetryTimes,NRF_RxState);	
								osMutexRelease(NRF_MutexHandle);		
								if( NRF_RxState == RX_OK) break;
						}
				}
				else	
				{
						if(RetryTimes != 0)
						{
								RetryTimes=0;
						}
						CRC_Value=HAL_CRC_Calculate(&hcrc,&CRC_Buf,1);
						if( (uint8_t)CRC_Value == Rx_packet[1]  )
						{
//						  printf("Rx:%d CRC OK\r\n",Rx_packet[0]);
						}	
						else
						{
//								printf("CRC ERROR\r\n");
						}
				}
				osMutexWait(NRF_MutexHandle,osWaitForever);
				NRF24L01_TX_Mode();
				osMutexRelease(NRF_MutexHandle);

				while( osThreadResume(NRF_TxTaskHandle)  != osOK);	
				while( osThreadSuspend(NRF_RxTaskHandle) != osOK);
    osDelay(1);
  }
  /* USER CODE END StartNRF_RxTask */
}

/* USER CODE BEGIN Header_StartNRF_TxTask */
/**
* @brief Function implementing the NRF_TxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNRF_TxTask */
void StartNRF_TxTask(void const * argument)
{
  /* USER CODE BEGIN StartNRF_TxTask */
		__IO uint8_t NRF_TxState=0;
	 __IO uint8_t RetryTimes=0;
		uint8_t Tx_packet[2]={0,0};
		uint32_t CRC_Buf=0;
		uint32_t CRC_Value=0;


		while ( osThreadSuspend(NRF_TxTaskHandle) != osOK);
  /* Infinite loop */
  for(;;)
  {		

			
				Tx_packet[0]++;
				CRC_Buf=Tx_packet[0];
			
				CRC_Value=HAL_CRC_Calculate(&hcrc,&CRC_Buf,1);
				Tx_packet[1]=(uint8_t)CRC_Value;
						
				osMutexWait(NRF_MutexHandle,osWaitForever);
				NRF_TxState=NRF24L01_TxPacket(Tx_packet);
				osMutexRelease(NRF_MutexHandle);
				
				if( NRF_TxState != TX_OK && RetryTimes<NRF_TX_MAXRETRY)
				{
						for(RetryTimes=0;RetryTimes<NRF_TX_MAXRETRY;RetryTimes++)
						{
								osMutexWait(NRF_MutexHandle,osWaitForever);
								NRF_TxState=NRF24L01_TxPacket(Tx_packet); 
								printf("Tx_packet:%d   Tx_CRC:%2x    Tx_times:%d   NRF_TxState:%2x\r\n",Tx_packet[0],(uint8_t)CRC_Value,RetryTimes,NRF_TxState);
								osMutexRelease(NRF_MutexHandle);
								if( NRF_TxState == TX_OK) 	break;
						}
				}
				else
				{  
						if(RetryTimes != 0)
						{
							 printf("Tx fail\r\n \r\n");
								RetryTimes=0;
						}
						else
						{
						  printf("Tx ok\r\n \r\n");
						}
				}
				while( osThreadResume(NRF_RxTaskHandle)   != osOK);
				while( osThreadSuspend(NRF_TxTaskHandle)  != osOK);
				osDelay(1);
  }
  /* USER CODE END StartNRF_TxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
