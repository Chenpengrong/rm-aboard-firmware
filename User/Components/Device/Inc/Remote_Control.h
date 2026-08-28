/**
  ******************************************************************************
  * @file           : Remote_Control.h
  * @brief          : The header file of remote control (SBUS protocol)
  * @author         : Chen
  * @date           : 2026/08/28
  * @version        : v1.0
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
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

/**
 * @brief 拨杆开关状态阈值
 * @note  拨杆分上/中/下三档, 减去偏移量后对应值为 -671 / 0 / +671
 */
#define RC_SW_UP    ((uint16_t)-671)
#define RC_SW_MID   ((uint16_t)0)
#define RC_SW_DOWN  ((uint16_t)671)

/**
 * @brief 键盘按键状态, 每位对应一个按键, 0 = 未按下, 1 = 按下
 */
typedef __packed struct
{
    uint16_t w : 1;     /*!< bit  0: W 键 */
    uint16_t s : 1;     /*!< bit  1: S 键 */
    uint16_t a : 1;     /*!< bit  2: A 键 */
    uint16_t d : 1;     /*!< bit  3: D 键 */
    uint16_t shift : 1; /*!< bit  4: Shift 键 */
    uint16_t ctrl : 1;  /*!< bit  5: Ctrl 键 */
    uint16_t q : 1;     /*!< bit  6: Q 键 */
    uint16_t e : 1;     /*!< bit  7: E 键 */
    uint16_t r : 1;     /*!< bit  8: R 键 */
    uint16_t f : 1;     /*!< bit  9: F 键 */
    uint16_t g : 1;     /*!< bit 10: G 键 */
    uint16_t z : 1;     /*!< bit 11: Z 键 */
    uint16_t x : 1;     /*!< bit 12: X 键 */
    uint16_t c : 1;     /*!< bit 13: C 键 */
    uint16_t v : 1;     /*!< bit 14: V 键 */
    uint16_t b : 1;     /*!< bit 15: B 键 */
} keyboard_state_t;

/**
 * @brief 遥控器完整数据结构, 包含摇杆通道和键鼠数据
 * @note  使用 __packed 确保与 SBUS 协议字节对齐一致
 */
typedef __packed struct
{
    /**
     * @brief 遥控器摇杆/拨杆通道数据, 已减去偏移量（中位为 0）
     */
    __packed struct
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
    } rc;

    /**
     * @brief 键鼠数据, 来自遥控器配套的上位机
     */
    __packed struct
    {
        int16_t mouse_x;           /*!< 鼠标 X 轴移动速度, 负值 = 向左  */
        int16_t mouse_y;           /*!< 鼠标 Y 轴移动速度, 负值 = 向下  */
        int16_t mouse_z;           /*!< 鼠标滚轮速度,     负值 = 向后  */

        int8_t left_button_down;   /*!< 鼠标左键: 0 = 未按下, 1 = 按下  */
        int8_t right_button_down;  /*!< 鼠标右键: 0 = 未按下, 1 = 按下  */

        keyboard_state_t keyboard; /*!< 键盘按键状态, 按位读取           */

        uint16_t reserved;         /*!< 保留位                          */
    } key_mouse;

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