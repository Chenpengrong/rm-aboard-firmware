/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.h
  * @brief          : The header file of Motor.h 
  * @author         : Chen
  * @date           : 2026/08/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_H 
#define MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "Pid.h"
#include <string.h>
#include "stdbool.h"

/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
typedef enum
{
  DJI_GM6020,
  DJI_M3508,
  DJI_M2006,
  DJI_MOTOR_TYPE_NUM,
} DJI_Motor_Type_e;

/**
 * @brief typedef enum that contains the type of DM Motor Device.
 */
typedef enum
{
  DM_4310,
  DM_3507,
  DM_MOTOR_TYPE_NUM,
} DM_Motor_Type_e;

/**
 * @brief  typedef enum that control mode the type of DM_Motor.
 */
typedef enum
{
  MIT,
  POSITION_VELOCITY,
  VELOCITY,
} DM_Motor_Control_Mode_Type_e;

/**
 * @brief  typedef enum that CMD of DM_Motor .
 */
typedef enum
{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;

/**
 * @brief typedef structure that contains the information for the motor CAN transmit and recieved .
 */
typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */
} CAN_Frame_Info_Typedef;

/**
 * @brief The structure that contains the Information of DJI motor Receive.
 */
typedef struct
{
  bool Initlized;   /*!< init flag */
  int16_t Encoder;      /*!< Current encoder value */
  int16_t Last_Encoder; /*!< Last encoder value */
  int16_t Velocity;    /*!< Motor rotate velocity (RPM)*/
  int16_t Current;     /*!< Motor current value */
  uint8_t Temperature; /*!< Motor temperature */
  float    Angle;   /*!< Motor angle in degree */
} DJI_Motor_Data_Typedef;

/**
 * @brief The structure that contains the Information of DM motor Receive.
 */
typedef struct
{
  bool Initlized;   /*!< init flag */
  uint8_t ID; /*!< Motor ID */
  uint8_t State; /*!< Motor state */
  uint16_t P_int; /*!< Motor Positon  uint16 */
  uint16_t V_int; /*!< Motor Velocity uint16 */
  uint16_t T_int; /*!< Motor Torque   uint16 */
	float  Position;   /*!< Motor Positon  */
  float  Velocity;   /*!< Motor Velocity */
  float  Torque;     /*!< Motor Torque   */
  float  Temperature_MOS;   /*!< Motor Temperature_MOS   */
	float  Temperature_Rotor; /*!< Motor Temperature_Rotor */
} DM_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the param range for the DM_Motor .
 */
typedef struct 
{
  float  P_MAX;
	float  V_MAX;
	float  T_MAX;
}DM_Motor_Param_Range_Typedef;

//------------------------------------------------------------------------------

/**
 * @brief typedef structure that contains the data for the DJI Motor Device.
 */
typedef struct
{
  DJI_Motor_Type_e Type;   /*!< Type of Motor */
  CAN_Frame_Info_Typedef CANFrame;    /*!< information for the CAN Transfer */
  DJI_Motor_Data_Typedef Data;   /*!< information for the Motor Device */
}DJI_Motor_Info_Typedef;

/**
 * @brief typedef structure that contains the information for the Damiao Motor Device.
 */
typedef struct
{
  DM_Motor_Type_e Type;   /*!< Type of Motor */
  DM_Motor_Control_Mode_Type_e Control_Mode;   /*!< Control mode for the DM_Motor */
  CAN_Frame_Info_Typedef CANFrame;    /*!< information for the CAN Transfer */
  DM_Motor_Data_Typedef Data;   /*!< information for the Motor Device */
  DM_Motor_Param_Range_Typedef Param_Range;   /*!< Param range for the DM_Motor */
}DM_Motor_Info_Typedef;

/* Externs ------------------------------------------------------------------*/
extern DJI_Motor_Info_Typedef DJI_YAW_Motor,DJI_Chassis_Motor[4],DJI_Paddle_Wheel_Motor;
extern DM_Motor_Info_Typedef DM_Pitch_Motor;

extern void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor);
extern void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor);
extern void DM_Motor_Command(CAN_TxFrame_TypeDef *CAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,DM_Motor_CMD_Type_e CMD);
extern void DM_Motor_CAN_TxMessage(CAN_TxFrame_TypeDef *CAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque);
extern void DJI_Motor_CAN_TxMessage(CAN_TxFrame_TypeDef *CAN_TxFrame,DJI_Motor_Info_Typedef *DJI_Motor,int16_t Current_or_Velocity[4]);

#endif //MOTOR_H