/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : Chen
  * @date           : 2026/08/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the can filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "bsp_can.h"
#include "Motor.h"

/**
 * @brief The structure that contains the Information of CAN1 and CAN2 Receive.
 */
CAN_RxFrame_TypeDef CAN_RxFIFO0Frame;
CAN_RxFrame_TypeDef CAN_RxFIFO1Frame;

/**
 * @brief The structure that contains the Information of CAN1 Transmit.
 */
CAN_TxFrame_TypeDef CAN1_TxFrame = {
    .hcan = &hcan1,
    .Header = {
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 0x08,
        .StdId = 0,
        .TransmitGlobalTime = DISABLE,
    },
    .Data = {0}
};

/**
 * @brief The structure that contains the Information of CAN2 Transmit.
 */
CAN_TxFrame_TypeDef CAN2_TxFrame = {
    .hcan = &hcan2,
    .Header = {
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 0x08,
        .StdId = 0,
        .TransmitGlobalTime = DISABLE,
    },
    .Data = {0}
};

/**
  * @brief  Configures the CAN Filter.
            CAN1: FIFO0   CAN2: FIFO1
  * @param  None
  * @retval None
  */
void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef CAN_FilterConfig;

    /* CAN1 filter: accept all standard IDs, route to FIFO0 */
    CAN_FilterConfig.FilterBank = 0;
    CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfig.FilterIdHigh = 0x0000;
    CAN_FilterConfig.FilterIdLow = 0x0000;
    CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfig.FilterMaskIdLow = 0x0000;
    CAN_FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterConfig.FilterActivation = ENABLE;
    CAN_FilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan1);

    /* CAN2 filter: accept all standard IDs, route to FIFO1 */
    CAN_FilterConfig.FilterBank = 14;
    CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfig.FilterIdHigh = 0x0000;
    CAN_FilterConfig.FilterIdLow = 0x0000;
    CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfig.FilterMaskIdLow = 0x0000;
    CAN_FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    CAN_FilterConfig.FilterActivation = ENABLE;
    CAN_FilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig);

    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

    HAL_CAN_Start(&hcan2);
}

/**
  * @brief  Function to transmit the CAN message.
  * @param  *CAN_TxFrame :the structure that contains the Information of CAN
  * @retval None
  */
void USER_CAN_AddMessageToTxMailbox(CAN_TxFrame_TypeDef *CAN_TxFrame)
{
    uint32_t TxMailbox = 0;
    HAL_CAN_AddTxMessage(CAN_TxFrame->hcan, &CAN_TxFrame->Header, CAN_TxFrame->Data, &TxMailbox);
}

/**
  * @brief  Function to converting the CAN1 received message to Fifo0.
  * @param  Identifier: Received the identifier.
  * @param  Data: Array that contains the received massage.
  * @retval None
  */
static void CAN1_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
  DJI_Motor_Info_Update(Identifier,Data,&DJI_Chassis_Motor[0]);
  DJI_Motor_Info_Update(Identifier,Data,&DJI_Chassis_Motor[1]);
  DJI_Motor_Info_Update(Identifier,Data,&DJI_Chassis_Motor[2]);
  DJI_Motor_Info_Update(Identifier,Data,&DJI_Chassis_Motor[3]);
  DJI_Motor_Info_Update(Identifier,Data,&DJI_YAW_Motor);
  DJI_Motor_Info_Update(Identifier,Data,&DJI_Paddle_Wheel_Motor);
}

/**
  * @brief  Function to converting the CAN2 received message to Fifo1.
  * @param  Identifier: Received the identifier.
  * @param  Data: Array that contains the received massage.
  * @retval None
  */
static void CAN2_RxFifo1RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
  DM_Motor_Info_Update(Identifier,Data,&DM_Pitch_Motor);
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxFIFO0Frame.Header, CAN_RxFIFO0Frame.Data);

    CAN1_RxFifo0RxHandler(&CAN_RxFIFO0Frame.Header.StdId, CAN_RxFIFO0Frame.Data);
}

/**
  * @brief  Rx FIFO 1 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_RxFIFO1Frame.Header, CAN_RxFIFO1Frame.Data);

    CAN2_RxFifo1RxHandler(&CAN_RxFIFO1Frame.Header.StdId, CAN_RxFIFO1Frame.Data);
}