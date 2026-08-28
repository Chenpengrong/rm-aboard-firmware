/**
  ******************************************************************************
  * @file           : Wit901c.c
  * @brief          : Wit901C IMU sensor functions
  * @author         : Chen
  * @date           : 2026/08/25
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Wit901c.h"
#include <string.h>

/*** IMU 数据存储区, 通过 GetIMUData() 获取 ***/
static Wit901C_t top_data = {.opened = 0};
uint8_t top_buffer[64];

/**
 * @brief ISR 写入此缓冲区并设置标志, 任务中处理
 */
volatile uint8_t imu_data_ready = 0;
uint8_t imu_proc_buf_it[64];

//------------------------------------------------------------------------------

/**
 * @brief  获取 IMU 数据指针, 只读访问
 * @retval 指向 top_data 的常量指针
 */
const Wit901C_t *get_IMU_data_pointer(void)
{
    return &top_data;
}

//------------------------------------------------------------------------------

/**
 * @brief  处理 Wit901C 数据
 * @param  data          指向接收到的原始数据缓冲区
 * @param  wit901c_data  指向 Wit901C 数据结构体, 用于存储解析后的数据
 * @retval 0x00: 成功解析数据
 * @retval 0x01: HEAD 错误
 * @retval 0x02: 校验位 SUM 错误或 TYPE 位错误
 * @retval 0x03: TYPE 位异常错误
 */
uint8_t Wit901c_Data_Process(uint8_t *data, Wit901C_t *wit901c_data)
{
	uint8_t sum = 0;

	/* 数据验证 */
	if (data[0] != WIT901C_HEAD)
		return 0x01; /* HEAD 错误 */
	switch (data[1])
	{
	case ACC_HEAD:
	case ANGVEL_HEAD:
	case ANGLE_HEAD:
		for (uint8_t i = 0; i < 10; i++)
			sum += data[i]; /* 计算校验位 */
		if (sum != data[10])
			return 0x02; /* 校验位 SUM 错误 */
		break;
	default:
		return 0x02; /* TYPE 位错误 */
	}

	/* 数据格式转换 */
	switch (data[1])
	{
	case ACC_HEAD:
		for (uint8_t i = 0; i < 3; i++)
			wit901c_data->acc[i] = 16.0f * RECIPROCAL32768 * (short)((short)data[i * 2 + 3] << 8 | data[i * 2 + 2]);
		break;
	case ANGVEL_HEAD:
		for (uint8_t i = 0; i < 3; i++)
			wit901c_data->angular[i] = 2000.0f * RECIPROCAL32768 * (short)((short)data[i * 2 + 3] << 8 | data[i * 2 + 2]);
		break;
	case ANGLE_HEAD:
		for (uint8_t i = 0; i < 3; i++)
			wit901c_data->angle[i] = 180.0f * RECIPROCAL32768 * (short)((short)data[i * 2 + 3] << 8 | data[i * 2 + 2]);
		break;
	default:
		return 0x03; /* TYPE 位异常错误 */
	}
	wit901c_data->opened = 0x01;
	return 0x00; /* 成功解析数据 */
}

//------------------------------------------------------------------------------

/**
 * @brief  在 FreeRTOS 任务中调用, 处理 ISR 中缓存的 IMU 数据
 */
void IMU_Process(void)
{
	if (!imu_data_ready) return;
	imu_data_ready = 0;

	for (int i = 0; i < 64 - 33; i++)
		Wit901c_Data_Process(&imu_proc_buf_it[i], &top_data);
}