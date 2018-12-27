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
#include "string.h"

#include "crc.h"
#include "BSP_LED.h"
#include "BSP_NRF24L01.h"
#include "BSP_OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NRF_TX_MAXRETRY 6
#define NRF_RX_MAXRETRY 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId DefaultTaskHandle;
osThreadId NRF_TxTaskHandle;
osThreadId NRF_RxTaskHandle;
osThreadId OLED_TaskHandle;
osMutexId NRFMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartNRF_TxTask(void const * argument);
void StartNRF_RxTask(void const * argument);
void StartOLED_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of NRFMutex */
  osMutexDef(NRFMutex);
  NRFMutexHandle = osMutexCreate(osMutex(NRFMutex));

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

  /* definition and creation of NRF_TxTask */
  osThreadDef(NRF_TxTask, StartNRF_TxTask, osPriorityHigh, 0, 512);
  NRF_TxTaskHandle = osThreadCreate(osThread(NRF_TxTask), NULL);

  /* definition and creation of NRF_RxTask */
  osThreadDef(NRF_RxTask, StartNRF_RxTask, osPriorityHigh, 0, 512);
  NRF_RxTaskHandle = osThreadCreate(osThread(NRF_RxTask), NULL);

  /* definition and creation of OLED_Task */
  osThreadDef(OLED_Task, StartOLED_Task, osPriorityIdle, 0, 64);
  OLED_TaskHandle = osThreadCreate(osThread(OLED_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DefaultTask thread.
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
	
		uint32_t CRC_Buf=0;
		uint32_t CRC_Value=0;
	
		uint8_t Tx_packet[32];
		char str_buf[16];
	
		memset(str_buf,0,16); 
		memset(str_buf,0,32);
	
  /* Infinite loop */
  for(;;)
  {
				Tx_packet[0]++;
				CRC_Buf=Tx_packet[0];
			 
				CRC_Value=HAL_CRC_Calculate(&hcrc,&CRC_Buf,1);
				Tx_packet[1]=(uint8_t)CRC_Value;
				
				osMutexWait(NRFMutexHandle,osWaitForever);
			 NRF24L01_TX_Mode();
				NRF24L01_TxPacket(Tx_packet);
				osMutexRelease(NRFMutexHandle);
				
				if( NRF_TxState != TX_OK && RetryTimes<NRF_TX_MAXRETRY)
				{
						for(RetryTimes=0;RetryTimes<NRF_TX_MAXRETRY;RetryTimes++)
						{
								osMutexWait(NRFMutexHandle,osWaitForever);
								NRF24L01_TxPacket(Tx_packet); 
//							 sprintf(str_buf,"%3d ox%2x %2d ox%2x",Tx_packet[0],(uint8_t)CRC_Value,RetryTimes,NRF_TxState); 
//							 OLED_ShowString(0,0,(uint8_t *)str_buf,12);
								osMutexRelease(NRFMutexHandle);
								if( NRF_TxState == TX_OK) 	break;
						}
				}
				else
				{  
						if(RetryTimes != 0)
						{
//							 OLED_ShowString(0,1,"Tx fault",12);
								RetryTimes=0;
						}
						else
						{
//							 OLED_ShowString(0,1,"        Tx ok",12);
						}
				}
				
				osMutexWait(NRFMutexHandle,osWaitForever);
				NRF24L01_RX_Mode();
				osMutexRelease(NRFMutexHandle);
				while( osThreadResume(NRF_RxTaskHandle)   != osOK);
				while( osThreadSuspend(NRF_TxTaskHandle)  != osOK);
    osDelay(1);
  }
  /* USER CODE END StartNRF_TxTask */
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
		char str_buf[16];

		memset(str_buf,0,16);		
	 while( osThreadSuspend(NRF_RxTaskHandle) != osOK);
		
  /* Infinite loop */
  for(;;)
  {

		  osMutexWait(NRFMutexHandle,osWaitForever);
	   NRF24L01_RxPacket(Rx_packet);
    osMutexRelease(NRFMutexHandle);		
		
				if( NRF_RxState != RX_OK )
				{
						for(RetryTimes=0;RetryTimes<NRF_RX_MAXRETRY;RetryTimes++)
						{
								osMutexWait(NRFMutexHandle,osWaitForever);
								NRF24L01_RxPacket(Rx_packet);
//								sprintf(str_buf,"%3d ox%2x %2d ox%2x",Rx_packet[0],Rx_packet[1],RetryTimes,NRF_RxState); 
//							 OLED_ShowString(0,2,(uint8_t *)str_buf,12);
								osMutexRelease(NRFMutexHandle);		
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
//							 sprintf(str_buf,"Rx:%d CRC OK",Rx_packet[0]); 
//							 OLED_ShowString(0,3,(uint8_t *)str_buf,12);
						}	
						else
						{
//							 OLED_ShowString(0,3,"CRC ERROR",12);
						}
				}
				sprintf(str_buf,"Rx:%d",Rx_packet[0]); 
				OLED_ShowString(0,3,(uint8_t *)str_buf,12);			 	
				while( osThreadResume(NRF_TxTaskHandle)  != osOK);	
				while( osThreadSuspend(NRF_RxTaskHandle) != osOK);
    osDelay(1);
  }
  /* USER CODE END StartNRF_RxTask */
}

/* USER CODE BEGIN Header_StartOLED_Task */
/**
* @brief Function implementing the OLED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLED_Task */
void StartOLED_Task(void const * argument)
{
  /* USER CODE BEGIN StartOLED_Task */

  /* Infinite loop */
  for(;;)
  {
			
    osDelay(10);
  }
  /* USER CODE END StartOLED_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
