/**
  ******************************************************************************
  * @file           : Vofa.h
  * @brief          : The header file of VOFA+ serial debugging tool
  * @author         : Chen
  * @date           : 2026/08/28
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VOFA_H
#define VOFA_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief VOFA+ 调试通道定义
 */
#define tx_lenth   2

/**
 * @brief  设置指定通道的调试数据值
 * @param  index  通道索引 (0 ~ tx_lenth-1)
 * @param  value  数据值
 */
extern void SetTxData(uint8_t index, float value);

/**
 * @brief  获取调试数据数组指针, 用于 serial_JustFloat 发送
 * @retval 指向 tx_data 的指针
 */
extern float *GetTxData(void);

/**
 * @brief  通过串口发送格式化字符串
 * @param  huart   指向 UART 句柄的指针
 * @param  format  格式化字符串, 语法与 printf 相同
 * @param  ...     可变参数列表
 * @retval HAL_OK: 发送成功
 * @retval HAL_ERROR: 发送失败
 */
extern HAL_StatusTypeDef serial_FireWater(UART_HandleTypeDef *huart, const char *format, ...);

/**
 * @brief  通过串口以二进制形式批量发送浮点数数组
 * @param  huart  指向 UART 句柄的指针
 * @param  data   指向待发送浮点数数组的指针
 * @param  count  要发送的浮点数个数
 * @retval HAL_OK: 发送成功
 * @retval HAL_BUSY: UART 或 DMA 正忙
 * @retval HAL_ERROR: 参数无效
 */
extern HAL_StatusTypeDef serial_JustFloat(UART_HandleTypeDef *huart, float *data, uint8_t count);

#endif