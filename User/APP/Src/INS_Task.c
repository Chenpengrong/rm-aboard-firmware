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


float acc_x = 0.f;
float acc_y = 0.f;
float acc_z = 0.f;
int Ch1 = 0;
int Ch2 = 0;
int Ch3 = 0;
int Ch4 = 0;


void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  TickType_t INS_Task_SysTick = 0;
  

 /* Infinite loop */
	for(;;)
  {
		INS_Task_SysTick = osKernelSysTick();

    IMU_Process();
    acc_x = get_IMU_data_pointer()->acc[0];
    acc_y = get_IMU_data_pointer()->acc[1];
    acc_z = get_IMU_data_pointer()->acc[2];

    Ch1 = get_remote_control_data_pointer()->Ch1;
    Ch2 = get_remote_control_data_pointer()->Ch2;
    Ch3 = get_remote_control_data_pointer()->Ch3;
    Ch4 = get_remote_control_data_pointer()->Ch4;
		
    // SetTxData(0, 1.3f);
    // SetTxData(1, 2.5f);
    // serial_JustFloat(&huart7, GetTxData(), tx_lenth);

		osDelay(1);
  }
}
  /* USER CODE END INS_Task */