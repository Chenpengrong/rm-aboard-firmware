/**
  ******************************************************************************
  * @file           : Remote_Control.c
  * @brief          : Remote control functions (SBUS protocol parsing)
  * @author         : Chen
  * @date           : 2026/09/13
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Remote_Control.h"

/**
 * @brief 全局遥控器数据实例
 */
RC_ctrl_t rc_ctrl;

//------------------------------------------------------------------------------

/**
 * @brief  获取遥控器数据只读指针
 * @retval 指向 rc_ctrl 的常量指针
 */
const RC_ctrl_t *get_remote_control_data_pointer(void)
{
    return &rc_ctrl;
}

//------------------------------------------------------------------------------

/**
 * @brief  解析 SBUS 原始字节流为遥控器数据结构
 * @param  sbus_buf  指向 SBUS 原始数据缓冲区（25 字节）
 * @param  rc_ctrl   指向遥控器数据结构体, 用于存储解析结果
 * @note   SBUS 协议每帧 25 字节, 包含 18 个通道（Ch1~Ch4, SA~SH, LD~RS）
 *         每个通道 11 位, 起始字节为 0x0F
 * @note   解析后所有通道值减去 RC_CH_VALUE_OFFSET(1024), 使中位归零,
 *         摇杆范围约 ±660, 拨杆位置可用 RC_SW_UP/MID/DOWN 宏判断
 * @note   该函数在 UART 接收中断或任务中调用, 参数为 NULL 时直接返回
 */
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    /* 帧头校验: SBUS 帧起始字节固定为 0x0F */
    if (sbus_buf[0] != 0x0F)
    {
        return;
    }

    /* 帧头标志 */
    rc_ctrl->Start = sbus_buf[0];

    /* 通道 1~4: 摇杆通道, 每个通道 11 位, 跨字节拼接 */
    rc_ctrl->Ch1 = (((uint16_t)sbus_buf[1]) | ((uint16_t)((sbus_buf[2]))) << 8) & 0x07FF;
    rc_ctrl->Ch2 = ((uint16_t)((sbus_buf[2] & 0xf8) >> 3)) | (((uint16_t)(sbus_buf[3] & 0x3f)) << 5);
    rc_ctrl->Ch3 = ((uint16_t)((sbus_buf[3] & 0xc0) >> 6)) | ((((uint16_t)sbus_buf[4]) << 2)) | (((uint16_t)(sbus_buf[5] & 0x01)) << 10);
    rc_ctrl->Ch4 = ((uint16_t)((sbus_buf[5] & 0xfe) >> 1)) | (((uint16_t)(sbus_buf[6] & 0x0f)) << 7);

    /* 拨杆 SA~SH: 每个 11 位 */
    rc_ctrl->SA = ((uint16_t)((sbus_buf[6] & 0xf0) >> 4)) | (((uint16_t)(sbus_buf[7] & 0x7f)) << 4);
    rc_ctrl->SB = ((uint16_t)((sbus_buf[7] & 0x80) >> 7)) | (((uint16_t)sbus_buf[8]) << 1) | (((uint16_t)(sbus_buf[9] & 0x03)) << 9);
    rc_ctrl->SC = ((uint16_t)((sbus_buf[9] & 0xfc) >> 2)) | (((uint16_t)(sbus_buf[10] & 0x1f)) << 6);
    rc_ctrl->SD = ((uint16_t)((sbus_buf[10] & 0xe0) >> 5)) | (((uint16_t)(sbus_buf[11])) << 3);
    rc_ctrl->SE = ((uint16_t)sbus_buf[12]) | (((uint16_t)(sbus_buf[13] & 0x07)) << 8);
    rc_ctrl->SF = ((uint16_t)((sbus_buf[13] & 0xf8) >> 3)) | (((uint16_t)(sbus_buf[14] & 0x3f)) << 5);
    rc_ctrl->SG = ((uint16_t)((sbus_buf[14] & 0xc0) >> 6)) | (((uint16_t)sbus_buf[15]) << 2) | (((uint16_t)(sbus_buf[16] & 0x01)) << 10);
    rc_ctrl->SH = ((uint16_t)((sbus_buf[16] & 0xfe) >> 1)) | (((uint16_t)(sbus_buf[17] & 0x0f)) << 7);

    /* 拨杆 LD~RS: 每个 11 位 */
    rc_ctrl->LD = ((uint16_t)((sbus_buf[17] & 0xf0) >> 4)) | (((uint16_t)(sbus_buf[18] & 0x7f)) << 4);
    rc_ctrl->RD = ((uint16_t)((sbus_buf[18] & 0x80) >> 7)) | (((uint16_t)sbus_buf[19]) << 1) | (((uint16_t)(sbus_buf[20] & 0x03)) << 9);
    rc_ctrl->LS = ((uint16_t)((sbus_buf[20] & 0xfc) >> 2)) | (((uint16_t)(sbus_buf[21] & 0x1f)) << 6);
    rc_ctrl->RS = ((uint16_t)((sbus_buf[21] & 0xe0) >> 5)) | (((uint16_t)sbus_buf[22]) << 3);

    /* 减去偏移量, 使中位归零, 摇杆范围约 -671 ~ +671 */
    rc_ctrl->Ch1 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch2 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch3 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch4 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SA -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SB -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SC -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SE -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SF -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SG -= RC_CH_VALUE_OFFSET;
    rc_ctrl->SH -= RC_CH_VALUE_OFFSET;
    rc_ctrl->LD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->RD -= RC_CH_VALUE_OFFSET;
    rc_ctrl->LS -= RC_CH_VALUE_OFFSET;
    rc_ctrl->RS -= RC_CH_VALUE_OFFSET;
}