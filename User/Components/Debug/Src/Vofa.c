/**
  ******************************************************************************
  * @file           : Vofa.c
  * @brief          : VOFA+ serial debugging tool functions
  * @author         : Chen
  * @date           : 2026/08/28
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Vofa.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/*** VOFA+ 调试数据存储区, 通过 SetTxData() / GetTxData() 访问 ***/
static float tx_data[tx_lenth];

//------------------------------------------------------------------------------

/**
 * @brief  设置指定通道的调试数据值
 * @param  index  通道索引 (0 ~ tx_lenth-1)
 * @param  value  数据值
 */
void SetTxData(uint8_t index, float value)
{
    if (index < tx_lenth) {
        tx_data[index] = value;
    }
}

/**
 * @brief  获取调试数据数组指针, 用于 serial_JustFloat 发送
 * @retval 指向 tx_data 的指针
 */
float *GetTxData(void)
{
    return tx_data;
}

//------------------------------------------------------------------------------

/**
 * @brief  DMA 发送缓冲区, 静态分配以保证 DMA 传输期间地址有效
 * @note   serial_FireWater 与 serial_JustFloat 共用此缓冲区, 两者互斥无需独立分配
 */
static uint8_t vofa_tx_buffer[256];

/**
 * @brief  通过 UART 发送格式化字符串, 支持类似 printf 的格式化输出
 * @param  huart   指向 UART 句柄的指针, 用于指定使用的串口
 * @param  format  格式化字符串, 语法与标准 printf 相同（如 %d、%f、%s 等）
 * @param  ...     可变参数列表, 与 format 中的格式说明符一一对应
 * @note   内部缓冲区大小为 256 字节, 若格式化后内容超过此长度将被截断
 * @note   是否支持浮点数（%f）取决于编译器的 printf 实现, 部分工具链（如 GCC）
 *         需要在链接选项中启用浮点支持, 否则 %f 可能输出为空或乱码
 * @retval HAL_OK: 发送成功
 * @retval HAL_BUSY: UART/DMA 正忙, 上一次发送未完成
 * @retval HAL_ERROR: 发送失败
 */
HAL_StatusTypeDef serial_FireWater(UART_HandleTypeDef *huart, const char *format, ...)
{
    HAL_StatusTypeDef status;
    va_list arg;
    int len;

    va_start(arg, format);
    len = vsnprintf((char *)vofa_tx_buffer, sizeof(vofa_tx_buffer), format, arg);
    va_end(arg);

    status = HAL_UART_Transmit_DMA(huart, vofa_tx_buffer, (uint16_t)len);
    return status;
}

//------------------------------------------------------------------------------

/**
 * @brief  通过 UART 以二进制形式批量发送浮点数数组, 使用 DMA 传输
 * @param  huart  指向 UART 句柄的指针, 用于指定使用的串口
 * @param  data   指向待发送浮点数数组的指针
 * @param  count  要发送的浮点数个数（必须大于 0）
 * @note   浮点数按 IEEE 754 单精度（32 位）格式发送, 采用小端字节序（LSB 在前）,
 *         与 STM32 Cortex-M 架构一致
 * @note   所有浮点数据发送完成后, 会自动附加 4 字节固定帧尾: {0x00, 0x00, 0x80, 0x7F}
 * @note   请勿在上一次 DMA 传输完成前再次调用本函数, 否则会导致数据覆盖或发送失败
 * @retval HAL_OK: 发送成功
 * @retval HAL_BUSY: UART 或 DMA 正忙
 * @retval HAL_ERROR: 参数无效
 */
HAL_StatusTypeDef serial_JustFloat(UART_HandleTypeDef *huart, float *data, uint8_t count)
{
    HAL_StatusTypeDef status;
    uint8_t total_len = 4 + 4 * count;
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f}; /* 帧尾 */

    memcpy(vofa_tx_buffer, data, 4 * count);
    memcpy(vofa_tx_buffer + count * 4, tail, 4);
    status = HAL_UART_Transmit_DMA(huart, vofa_tx_buffer, total_len);
    return status;
}