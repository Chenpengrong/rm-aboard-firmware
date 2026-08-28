/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for Start_CAN_Task */
osThreadId_t Start_CAN_TaskHandle;
uint32_t Start_CAN_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_CAN_TaskControlBlock;
const osThreadAttr_t Start_CAN_Task_attributes = {
  .name = "Start_CAN_Task",
  .cb_mem = &Start_CAN_TaskControlBlock,
  .cb_size = sizeof(Start_CAN_TaskControlBlock),
  .stack_mem = &Start_CAN_TaskBuffer[0],
  .stack_size = sizeof(Start_CAN_TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Start_INS_Task */
osThreadId_t Start_INS_TaskHandle;
uint32_t defaultTaskBuffer[ 1024 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t Start_INS_Task_attributes = {
  .name = "Start_INS_Task",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Start_Control_T */
osThreadId_t Start_Control_THandle;
uint32_t Start_Control_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Control_TaskControlBlock;
const osThreadAttr_t Start_Control_T_attributes = {
  .name = "Start_Control_T",
  .cb_mem = &Start_Control_TaskControlBlock,
  .cb_size = sizeof(Start_Control_TaskControlBlock),
  .stack_mem = &Start_Control_TaskBuffer[0],
  .stack_size = sizeof(Start_Control_TaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Start_Detect_Ta */
osThreadId_t Start_Detect_TaHandle;
uint32_t Start_Detect_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Detect_TaskControlBlock;
const osThreadAttr_t Start_Detect_Ta_attributes = {
  .name = "Start_Detect_Ta",
  .cb_mem = &Start_Detect_TaskControlBlock,
  .cb_size = sizeof(Start_Detect_TaskControlBlock),
  .stack_mem = &Start_Detect_TaskBuffer[0],
  .stack_size = sizeof(Start_Detect_TaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CAN_Task(void *argument);
void INS_Task(void *argument);
void Control_Task(void *argument);
void Detect_Task(void *argument);

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
  /* creation of Start_CAN_Task */
  Start_CAN_TaskHandle = osThreadNew(CAN_Task, NULL, &Start_CAN_Task_attributes);

  /* creation of Start_INS_Task */
  Start_INS_TaskHandle = osThreadNew(INS_Task, NULL, &Start_INS_Task_attributes);

  /* creation of Start_Control_T */
  Start_Control_THandle = osThreadNew(Control_Task, NULL, &Start_Control_T_attributes);

  /* creation of Start_Detect_Ta */
  Start_Detect_TaHandle = osThreadNew(Detect_Task, NULL, &Start_Detect_Ta_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_CAN_Task */
/**
  * @brief  Function implementing the Start_CAN_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CAN_Task */
__weak void CAN_Task(void *argument)
{
  /* USER CODE BEGIN CAN_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the Start_INS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void *argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the Start_Control_T thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void *argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the Start_Detect_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void *argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

