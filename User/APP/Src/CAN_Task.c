/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
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
#include "Motor.h"

void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  TickType_t CAN_Task_SysTick = 0;

  DM_Motor_Command(&CAN2_TxFrame,&DM_Pitch_Motor,Motor_Enable);
  HAL_Delay(30);

 /* Infinite loop */
	for(;;)
  {
		CAN_Task_SysTick = osKernelSysTick();

    int16_t Current_or_Velocity[4] = {2000,2000,2000,2000};
    DJI_Motor_CAN_TxMessage(&CAN1_TxFrame,DJI_Chassis_Motor,Current_or_Velocity);
    DM_Motor_CAN_TxMessage(&CAN2_TxFrame,&DM_Pitch_Motor,0.f,0.f,2.f,0.1f,0.f);
		
		osDelay(1);
  }
}
  /* USER CODE END CAN_Task */