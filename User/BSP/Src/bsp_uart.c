/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions
  * @author         : Chen
  * @date           : 2026/08/25
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to init the BSP_USART_Init functions
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include "usart.h"
#include "Remote_Control.h"
#include "Wit901c.h"
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength);
static void USER_USART1_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);
static void USER_UART8_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);

/* Private variables ---------------------------------------------------------*/

/**
 * @brief  UART1 (USART1) 遥控器 DMA 双缓冲接收区
 */
static uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief  UART8 Wit901C IMU DMA 双缓冲接收区, 每帧 64 字节
 */
#define IMU_RX_BUF_SIZE   64u
static uint8_t IMU_MultiRx_Buf[2][IMU_RX_BUF_SIZE];

//------------------------------------------------------------------------------

/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void)
{
	/* UART1 (USART1) 初始化: 遥控器 SBUS 接收 */
	USART_RxDMA_MultiBuffer_Init(&huart1, (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1], SBUS_RX_BUF_NUM);

	/* UART8 初始化: Wit901C IMU 接收 */
	USART_RxDMA_MultiBuffer_Init(&huart8, (uint32_t *)IMU_MultiRx_Buf[0], (uint32_t *)IMU_MultiRx_Buf[1], IMU_RX_BUF_SIZE);
}

//------------------------------------------------------------------------------

/**
  * @brief  Init the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart             pointer to a UART_HandleTypeDef structure that contains
  *                           the configuration information for the specified USART Stream.
  * @param  DstAddress        pointer to The destination memory Buffer address
  * @param  SecondMemAddress  pointer to The second memory Buffer address in case of multi buffer Transfer
  * @param  DataLength        The length of data to be transferred from source to destination
  * @retval none
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

	huart->RxXferSize = DataLength * 2;

	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	do {
		__HAL_DMA_DISABLE(huart->hdmarx);
	} while (((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

	/* Configure the source peripheral address */
	((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->DR;

	/* Configure the destination memory Buffer address */
	((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

	/* Configure DMA Stream second memory address */
	((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

	/* Configure the length of data to be transferred from source to destination */
	((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR = DataLength;

	/* Enable double memory buffer */
	SET_BIT(((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

	/* Enable DMA */
	__HAL_DMA_ENABLE(huart->hdmarx);
}

//------------------------------------------------------------------------------

/**
  * @brief  USER USART1 Reception Event Callback (Remote Control).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer
  * @retval None
  */
static void USER_USART1_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
	/* Current memory buffer used is Memory 0 */
	if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
	{
		/* Disable DMA */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* Switch Memory 0 to Memory 1 */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

		/* Reset the receive count */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM * 2);

		/* Judge whether size is equal to the length of the received data */
		if (Size == RC_FRAME_LENGTH)
		{
			/* Memory 0 data update to remote_ctrl */
			sbus_to_rc(SBUS_MultiRx_Buf[0], &rc_ctrl);
		}
	}
	/* Current memory buffer used is Memory 1 */
	else
	{
		/* Disable DMA */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* Switch Memory 1 to Memory 0 */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

		/* Reset the receive count */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM * 2);

		if (Size == RC_FRAME_LENGTH)
		{
			/* Memory 1 data update to remote_ctrl */
			sbus_to_rc(SBUS_MultiRx_Buf[1], &rc_ctrl);
		}
	}
}

//------------------------------------------------------------------------------

/**
  * @brief  USER UART8 Reception Event Callback (Wit901C IMU).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer
  * @retval None
  */
static void USER_UART8_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
	/* Current memory buffer used is Memory 0 */
	if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
	{
		/* Disable DMA */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* Switch Memory 0 to Memory 1 */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

		/* Reset the receive count */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, IMU_RX_BUF_SIZE * 2);

		/* 数据拷贝到处理缓冲区, 浮点运算移到任务中 */
		memcpy(imu_proc_buf_it, IMU_MultiRx_Buf[0], IMU_RX_BUF_SIZE);
		imu_data_ready = 1;
	}
	/* Current memory buffer used is Memory 1 */
	else
	{
		/* Disable DMA */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* Switch Memory 1 to Memory 0 */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

		/* Reset the receive count */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, IMU_RX_BUF_SIZE * 2);

		/* 数据拷贝到处理缓冲区, 浮点运算移到任务中 */
		memcpy(imu_proc_buf_it, IMU_MultiRx_Buf[1], IMU_RX_BUF_SIZE);
		imu_data_ready = 1;
	}
}

//------------------------------------------------------------------------------

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)
	{
		USER_USART1_RxHandler(huart, Size);
	}

	if (huart == &huart8)
	{
		USER_UART8_RxHandler(huart, Size);
	}

	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

	/* Enable IDLE interrupt */
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	/* Enable the DMA transfer for the receiver request */
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

	/* Enable DMA */
	__HAL_DMA_ENABLE(huart->hdmarx);
}
