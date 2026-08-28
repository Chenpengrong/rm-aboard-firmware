/**
  ******************************************************************************
  * @file           : Detect_Task.c
  * @brief          : Detect task
  * @author         : Chen
  * @date           : 2026/08/23
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  TickType_t Detect_Task_SysTick = 0;
  

 /* Infinite loop */
	for(;;)
  {
		Detect_Task_SysTick = osKernelSysTick();

		
		osDelay(1);
  }
}
  /* USER CODE END Detect_Task */