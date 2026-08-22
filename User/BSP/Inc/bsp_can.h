/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : The header file of bsp_can.c 
  * @author         : Chen
  * @date           : 2026/08/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "can.h"

/**
 * @brief The structure that contains the Information of CAN Transmit.
 */
typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef Header;
    uint8_t             Data[8];
} CAN_TxFrame_TypeDef;

/**
 * @brief The structure that contains the Information of CAN Receive.
 */
typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef Header;
    uint8_t             Data[8];
} CAN_RxFrame_TypeDef;

/* Externs ------------------------------------------------------------------*/
extern CAN_TxFrame_TypeDef CAN1_TxFrame;
extern CAN_TxFrame_TypeDef CAN2_TxFrame;
extern void USER_CAN_AddMessageToTxMailbox(CAN_TxFrame_TypeDef *CAN_TxFrame);
extern void BSP_CAN_Init(void);

#endif