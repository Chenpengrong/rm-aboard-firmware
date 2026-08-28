/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
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


void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t Control_Task_SysTick = 0;


 /* Infinite loop */
	for(;;)
  {
		Control_Task_SysTick = osKernelSysTick();

		
		osDelay(1);
  }
}
  /* USER CODE END Control_Task */