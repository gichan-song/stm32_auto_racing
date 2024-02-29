/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t rx_Data[1];
extern uint8_t flag;
extern uint32_t echoTime_front, echoTime_left, echoTime_right;
extern uint32_t distance_front, distance_left, distance_right;
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
/* Definitions for changeMode */
osThreadId_t changeModeHandle;
const osThreadAttr_t changeMode_attributes = {
  .name = "changeMode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for trigUltrasonic */
osThreadId_t trigUltrasonicHandle;
const osThreadAttr_t trigUltrasonic_attributes = {
  .name = "trigUltrasonic",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void changeModefunc(void *argument);
void trigUltrasonicfunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of changeMode */
  changeModeHandle = osThreadNew(changeModefunc, NULL, &changeMode_attributes);

  /* creation of trigUltrasonic */
  trigUltrasonicHandle = osThreadNew(trigUltrasonicfunc, NULL, &trigUltrasonic_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_changeModefunc */
/**
  * @brief  Function implementing the changeMode thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_changeModefunc */
void changeModefunc(void *argument)
{
  /* USER CODE BEGIN changeModefunc */
  /* Infinite loop */
  for(;;)
  {
	  HAL_UART_Receive_IT(&huart1, rx_Data, sizeof(rx_Data));
	  	 	 	  if(rx_Data[0] == 'a') flag = !flag;
	  	 	 	  if(!flag) bt_motor();
    osDelay(1);
  }
  /* USER CODE END changeModefunc */
}

/* USER CODE BEGIN Header_trigUltrasonicfunc */
/**
* @brief Function implementing the trigUltrasonic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_trigUltrasonicfunc */
void trigUltrasonicfunc(void *argument)
{
  /* USER CODE BEGIN trigUltrasonicfunc */
  /* Infinite loop */
  for(;;)
  {
	  if(flag)
	  {
		  Trig1();
		  osDelay(2);
		  Trig3();
		  osDelay(10);
	  }
  }
  /* USER CODE END trigUltrasonicfunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

