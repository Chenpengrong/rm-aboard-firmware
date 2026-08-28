/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mcu.h
  * @brief          : MCU peripheral initialization functions
  * @author         : Chen
  * @date           : 2026/08/23
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_MCU_H
#define BSP_MCU_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief Initializes the MCU.
  */
extern void MCU_Init(void);

#endif //BSP_MCU_H