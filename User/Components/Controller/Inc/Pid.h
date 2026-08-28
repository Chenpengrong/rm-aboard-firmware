/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : PID.h
  * @brief          : The header file of PID.h 
  * @author         : Chen
  * @date           : 2026/08/23
  * @version        : v1.1
  ******************************************************************************
  * @attention      : Variables of type PID_t must be declared as global variables,
  *                  since history_error and accrued_error need to persist without being reset to zero.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H
#define PID_H

/* Including ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "math.h"

/**
 * @brief The structure that contains the Information of motor Device.
 */
typedef struct
{
    float Kp, Ki, Kd;
    float Kf;            // 前馈增益（不使用设为0）
    float accrued_error; // 积分累积（初始化设为0）
    float history_error; // 上次误差（初始化设为0）
    float max_accrued;   // 积分限幅
    float max_output;    // 输出限幅
    float dead_out;      // 死区阈值
} PID_t;

/*  Externs-------------------------------------*/
extern void PID_Init(PID_t *pid);
extern void PID_SetParam(PID_t *pid, float Kp, float Ki, float Kd, float Kf,
                         float max_accrued, float max_output, float dead_out);
extern float PIDCompute(PID_t *pid, float error);
extern float PIDCompute_Tracking(PID_t *pid, float error, float target_rate);
extern float PIDCompute_Disturbance(PID_t *pid, float error, float feedforward_static);

#endif