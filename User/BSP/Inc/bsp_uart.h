/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.h
  * @brief          : The header file of bsp_uart.c
  * @author         : Chen
  * @date           : 2026/08/25
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_UART_H
#define BSP_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/**
 * @brief  初始化所有 UART 外设的 DMA 多缓冲接收
 */
extern void BSP_USART_Init(void);


#endif