/**
  ******************************************************************************
  * @file           : Remote_Control.h
  * @brief          : The header file of remote control (SBUS protocol)
  * @author         : Chen
  * @date           : 2026/09/13
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "stdio.h"

/**
 * @brief SBUS 接收缓冲区大小
 */
#define SBUS_RX_BUF_NUM 50u

/**
 * @brief 遥控器数据帧长度（25 字节）
 */
#define RC_FRAME_LENGTH 25u

/**
 * @brief 通道值偏移量, SBUS 中值 1024 对应摇杆居中, 减去后 0 为中位
 */
#define RC_CH_VALUE_OFFSET ((int16_t)1024)

/**
 * @brief 拨杆开关状态阈值
 * @note  拨杆分上/中/下三档, 减去偏移量后对应值为 -671 / 0 / +671
 */
#define RC_SW_UP    ((int16_t)-671)
#define RC_SW_MID   ((int16_t)0)
#define RC_SW_DOWN  ((int16_t)671)



/**
 * @brief 遥控器完整数据结构, 包含摇杆通道和拨杆数据
 * @note  使用 __packed 确保与 SBUS 协议字节对齐一致
 */
typedef __packed struct
{
    int16_t Start;  /*!< 帧头标志                          */

    int16_t Ch1;    /*!< 通道 1: 右摇杆水平, 右横          */
    int16_t Ch2;    /*!< 通道 2: 左摇杆垂直, 左竖          */
    int16_t Ch3;    /*!< 通道 3: 右摇杆垂直, 右竖          */
    int16_t Ch4;    /*!< 通道 4: 左摇杆水平, 左横          */

    int16_t SA;     /*!< 拨杆 SA                           */
    int16_t SB;     /*!< 拨杆 SB                           */
    int16_t SC;     /*!< 拨杆 SC                           */
    int16_t SD;     /*!< 拨杆 SD                           */

    int16_t SE;     /*!< 拨杆 SE                           */
    int16_t SF;     /*!< 拨杆 SF                           */
    int16_t SG;     /*!< 拨杆 SG                           */
    int16_t SH;     /*!< 拨杆 SH                           */

    int16_t LD;     /*!< 拨杆 LD                           */
    int16_t RD;     /*!< 拨杆 RD                           */
    int16_t LS;     /*!< 拨杆 LS                           */
    int16_t RS;     /*!< 拨杆 RS                           */
} RC_ctrl_t;

/**
 * @brief 全局遥控器数据实例, 外部可直接读取
 */
extern RC_ctrl_t rc_ctrl;

/**
 * @brief  解析 SBUS 原始字节流为遥控器数据结构
 * @param  sbus_buf  指向 SBUS 原始数据缓冲区（25 字节）
 * @param  rc_ctrl   指向遥控器数据结构体, 用于存储解析结果
 * @note   解析后所有通道值已减去 RC_CH_VALUE_OFFSET, 中位为 0
 */
extern void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

/**
 * @brief  获取遥控器数据只读指针
 * @retval 指向 rc_ctrl 的常量指针
 */
extern const RC_ctrl_t *get_remote_control_data_pointer(void);

#endif