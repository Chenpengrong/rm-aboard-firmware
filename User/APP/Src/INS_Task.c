/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task
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
#include "Wit901c.h"
#include "Remote_Control.h"
#include "Vofa.h"

void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  TickType_t INS_Task_SysTick = 0;
  

 /* Infinite loop */
	for(;;)
  {
		INS_Task_SysTick = osKernelSysTick();

    IMU_Process();
    float acc_x = get_IMU_data_pointer()->acc[0];
    float acc_y = get_IMU_data_pointer()->acc[1];
    float acc_z = get_IMU_data_pointer()->acc[2];

    int Ch1 = get_remote_control_data_pointer()->rc.Ch1;
    int Ch2 = get_remote_control_data_pointer()->rc.Ch2;
    int Ch3 = get_remote_control_data_pointer()->rc.Ch3;
    int Ch4 = get_remote_control_data_pointer()->rc.Ch4;
		
    SetTxData(0, 1.3f);
    serial_JustFloat(&huart7, GetTxData(), tx_lenth);
    SetTxData(1, 1.3f);
    serial_JustFloat(&huart7, GetTxData(), tx_lenth);

		osDelay(1);
  }
}
  /* USER CODE END INS_Task */