/**
  ******************************************************************************
  * @file           : Wit901c.h
  * @brief          : The header file of Wit901C IMU sensor
  * @author         : Chen
  * @date           : 2026/08/25
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef WIT901C_H
#define WIT901C_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief typedef structure that contains the data of Wit901C IMU sensor.
 */
typedef struct
{
	float acc[3];      /*!< 加速度数据, 单位: g */
	float angular[3];  /*!< 角速度数据, 单位: °/s */
	float angle[3];    /*!< 角度数据, 单位: ° */
	uint8_t opened;    /*!< 设备是否已成功打开标志 */
} Wit901C_t;

/**
 * @brief Wit901C 私有协议串口通信各功能的数据头定义
 */
#define WIT901C_HEAD      (0x55)
#define ACC_HEAD          (0x51)
#define ANGVEL_HEAD       (0x52)
#define ANGLE_HEAD        (0x53)
#define RECIPROCAL32768   (3.0517578125e-05f)

/**
 * @brief  处理 Wit901C 数据
 * @param  data          指向接收到的原始数据缓冲区
 * @param  wit901c_data  指向 Wit901C 数据结构体, 用于存储解析后的数据
 * @retval 0x00: 成功解析数据
 * @retval 0x01: HEAD 错误
 * @retval 0x02: 校验位 SUM 错误或 TYPE 位错误
 * @retval 0x03: TYPE 位异常错误
 */
extern uint8_t Wit901c_Data_Process(uint8_t *data, Wit901C_t *wit901c_data);

/**
 * @brief  在 FreeRTOS 任务中调用, 处理 ISR 中缓存的 IMU 数据
 */
extern void IMU_Process(void);

/**
 * @brief  获取 IMU 数据指针, 只读访问
 * @retval 指向 IMU 数据的常量指针
 */
extern const Wit901C_t *get_IMU_data_pointer(void);

/*** IMU 内部缓冲区, ISR 使用 ***/
extern volatile uint8_t imu_data_ready;
extern uint8_t imu_proc_buf_it[64];

#endif