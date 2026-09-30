/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : Motor functions 
  * @author         : Chen
  * @date           : 2026/08/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Motor.h"

/* Struct define ------------------------------------------------------------------*/

/**
 * @brief The structure that contains the Information of yaw motor.Use DJI GM6020 motor.
 */
DJI_Motor_Info_t DJI_YAW_Motor =
{
    .Type = DJI_GM6020,
    .CANFrame = {
        .TxIdentifier = 0x1FF,
        .RxIdentifier = 0x205, //ID = 1 (0x204 + ID)
    }
};

/**
 * @brief The structure that contains the Information of chassis motor.Use DJI M3508 motor.
 */ 
DJI_Motor_Info_t DJI_Chassis_Motor[4] =
{
    [0] = {
        .Type = DJI_M3508,
        .CANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x201, //ID = 1
        }
    },
    [1] = {
        .Type = DJI_M3508,
        .CANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x202, //ID = 2
        }
    },
    [2] = {
        .Type = DJI_M3508,
        .CANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x203, //ID = 3
        }
    },
    [3] = {
        .Type = DJI_M3508,
        .CANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x204, //ID = 4
        }
    },
};

/**
 * @brief The structure that contains the Information of paddle wheel motor.Use DJI M2006 motor.
 */
DJI_Motor_Info_t DJI_Paddle_Wheel_Motor =
{
    .Type = DJI_M2006,
    .CANFrame = {
        .TxIdentifier = 0x1FF,
        .RxIdentifier = 0x207, //ID = 7 (0x200 + ID)
    }
};

//------------------------------------------------------------------------------

/**
 * @brief The structure that contains the Information of joint motor.Use DM 4310 motor.
 */
DM_Motor_Info_t DM_Pitch_Motor =
{
    .Type = DM_4310,
    .Control_Mode = MIT,
    .CANFrame = {
        .TxIdentifier = 0x101,
        .RxIdentifier = 0x101, //ID = 1
    }
};

/* Function Declaration ------------------------------------------------------------------*/

float F_Loop_Constrain(float Input, float Min_Value, float Max_Value);

static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_t *,float ,uint16_t );

static float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_t *,float ,uint16_t );

static float uint_to_float(int X_int, float X_min, float X_max, int Bits);

static int float_to_uint(float x, float x_min, float x_max, int bits);

/* Rx part ------------------------------------------------------------------*/

/**
  * @brief  Update the DJI motor Information
  * @param  Identifier  pointer to the specifies the standard identifier.
  * @param  Rx_Buf  pointer to the can receive data
  * @param  DJI_Motor pointer to a DJI_Motor_Info_t structure 
  *         that contains the information of DJI motor
  * @retval None
  */
void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_t *DJI_Motor)
{
	/* check the Identifier */
	if(*Identifier != DJI_Motor->CANFrame.RxIdentifier) return;
	
	/* transforms the  general motor data */
	DJI_Motor->Data.Temperature = Rx_Buf[6];
	DJI_Motor->Data.Encoder  = ((int16_t)Rx_Buf[0] << 8 | (int16_t)Rx_Buf[1]);
	DJI_Motor->Data.Velocity = ((int16_t)Rx_Buf[2] << 8 | (int16_t)Rx_Buf[3]);
	DJI_Motor->Data.Current  = ((int16_t)Rx_Buf[4] << 8 | (int16_t)Rx_Buf[5]);

	/* transform the Encoder to angle */
	switch(DJI_Motor->Type)
	{
		case DJI_GM6020:
		    DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,1.f,8192);
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,3591.f/187.f,8192);
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,36.f,8192);
		break;
		
		default:break;
	}
}

/**
  * @brief  Update the DM_Motor Information
  * @param  Identifier:  pointer to the specifies the standard identifier.
  * @param  Rx_Buf:  pointer to the can receive data
  * @param  DM_Motor: pointer to a DM_Motor_Info_Typedef structure that contains the information of DM_Motor
  * @retval None
  */
void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_t *DM_Motor)
{
	 
	if(*Identifier != DM_Motor->CANFrame.RxIdentifier) return;
		
  DM_Motor->Data.ID = Rx_Buf[0] & 0x0F;
	DM_Motor->Data.State = Rx_Buf[0]>>4;
	DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) <<8) | ((uint16_t)(Rx_Buf[2]));
	DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) <<4) | ((uint16_t)(Rx_Buf[4])>>4);
	DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4]&0xF) <<8) | ((uint16_t)(Rx_Buf[5]));
	DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
	DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16);
  DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);

  DM_Motor->Data.Temperature_MOS   = (float)(Rx_Buf[6]);
	DM_Motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);

}

/* DM tx part ------------------------------------------------------------------*/

/**
  * @brief  Transmit enable disable save zero position Command to DM motor 
  * @param  *CAN_TxFrame: pointer to the CAN_TxFrame_TypeDef.
  * @param  *DM_Motor: pointer to the DM_Motor
  * @param  CMD: Transmit Command  (DM_Motor_CMD_Type_e)
  * @retval None
  */
void DM_Motor_Command(CAN_TxFrame_t *CAN_TxFrame,DM_Motor_Info_t *DM_Motor,DM_Motor_CMD_Type_e CMD){

	CAN_TxFrame->Header.StdId = DM_Motor->CANFrame.TxIdentifier;
  	
	CAN_TxFrame->Data[0] = 0xFF;
  CAN_TxFrame->Data[1] = 0xFF;
 	CAN_TxFrame->Data[2] = 0xFF;
	CAN_TxFrame->Data[3] = 0xFF;
	CAN_TxFrame->Data[4] = 0xFF;
	CAN_TxFrame->Data[5] = 0xFF;
	CAN_TxFrame->Data[6] = 0xFF;
	
	switch(CMD){
		 
		case Motor_Enable :
	        CAN_TxFrame->Data[7] = 0xFC; 
	    break;
      
		case Motor_Disable :
	        CAN_TxFrame->Data[7] = 0xFD; 
        break;
      
		case Motor_Save_Zero_Position :
	        CAN_TxFrame->Data[7] = 0xFE; 
		break;
			
		default:break;   
	}
	
    USER_CAN_AddMessageToTxMailbox(CAN_TxFrame);
}

/**
  * @brief  CAN Transmit DM motor Information
  * @param  *CAN_TxFrame  pointer to the CAN_TxFrame_TypeDef.
  * @param  *DM_Motor  pointer to the DM_Motor
  * @param  Postion Velocity KP KD Torgue: Target
  * @retval None
  */
void DM_Motor_CAN_TxMessage(CAN_TxFrame_t *CAN_TxFrame,DM_Motor_Info_t *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque){
	
   if(DM_Motor->Control_Mode == MIT){
		 
		uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
		
		Postion_Tmp  =  float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16) ;
		Velocity_Tmp =  float_to_uint(Velocity,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);
		Torque_Tmp   =  float_to_uint(Torque,  -DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
		KP_Tmp = float_to_uint(KP,0,500,12);
		KD_Tmp = float_to_uint(KD,0,5,12);
		
		CAN_TxFrame->Header.StdId = DM_Motor->CANFrame.TxIdentifier;
		 
		CAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
		CAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
		CAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
		CAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
		CAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);
		CAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
		CAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
		CAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);

	}else if(DM_Motor->Control_Mode == POSITION_VELOCITY){
	
		uint8_t *Postion_Tmp,*Velocity_Tmp;
		
		Postion_Tmp  = (uint8_t*) & Postion;
		Velocity_Tmp = (uint8_t*) & Velocity;
		
		CAN_TxFrame->Header.StdId = DM_Motor->CANFrame.TxIdentifier + 0x100;
		
		CAN_TxFrame->Data[0] = *(Postion_Tmp);
		CAN_TxFrame->Data[1] = *(Postion_Tmp + 1);
		CAN_TxFrame->Data[2] = *(Postion_Tmp + 2);
		CAN_TxFrame->Data[3] = *(Postion_Tmp + 3);
		CAN_TxFrame->Data[4] = *(Velocity_Tmp);
		CAN_TxFrame->Data[5] = *(Velocity_Tmp + 1);
		CAN_TxFrame->Data[6] = *(Velocity_Tmp + 2);
		CAN_TxFrame->Data[7] = *(Velocity_Tmp + 3);
		
	}else if(DM_Motor->Control_Mode == VELOCITY){
	
	  uint8_t *Velocity_Tmp;
		Velocity_Tmp = (uint8_t*) & Velocity;
		
		CAN_TxFrame->Header.StdId = DM_Motor->CANFrame.TxIdentifier + 0x200;
		
		CAN_TxFrame->Data[0] = *(Velocity_Tmp);
		CAN_TxFrame->Data[1] = *(Velocity_Tmp + 1);
		CAN_TxFrame->Data[2] = *(Velocity_Tmp + 2);
		CAN_TxFrame->Data[3] = *(Velocity_Tmp + 3);
		CAN_TxFrame->Data[4] = 0;
		CAN_TxFrame->Data[5] = 0;
		CAN_TxFrame->Data[6] = 0;
		CAN_TxFrame->Data[7] = 0;

	}
	 
	USER_CAN_AddMessageToTxMailbox(CAN_TxFrame);
}

/* DJI tx part ------------------------------------------------------------------*/

/**
  * @brief  CAN Transmit DJI motor control command (current or velocity)
  * @param  *CAN_TxFrame          pointer to the CAN_TxFrame_TypeDef.
  * @param  *DJI_Motor            pointer to the DJI_Motor_Info_t structure
  * @param  Current_or_Velocity   array of 4 int16_t values, each representing
  *                               the target current or velocity for one motor
  * @retval None
  */
void DJI_Motor_CAN_TxMessage(CAN_TxFrame_t *CAN_TxFrame,DJI_Motor_Info_t *DJI_Motor,int16_t Current_or_Velocity[4]){

  CAN_TxFrame->Header.StdId = DJI_Motor->CANFrame.TxIdentifier;

  CAN_TxFrame->Data[0] = (uint8_t)(Current_or_Velocity[0] >> 8);
  CAN_TxFrame->Data[1] = (uint8_t)Current_or_Velocity[0];
  CAN_TxFrame->Data[2] = (uint8_t)(Current_or_Velocity[1] >> 8);
  CAN_TxFrame->Data[3] = (uint8_t)Current_or_Velocity[1];
  CAN_TxFrame->Data[4] = (uint8_t)(Current_or_Velocity[2] >> 8);
  CAN_TxFrame->Data[5] = (uint8_t)Current_or_Velocity[2];
  CAN_TxFrame->Data[6] = (uint8_t)(Current_or_Velocity[3] >> 8);
  CAN_TxFrame->Data[7] = (uint8_t)Current_or_Velocity[3];

  USER_CAN_AddMessageToTxMailbox(CAN_TxFrame);
}

/* Helper Function ------------------------------------------------------------------*/

/**
  * @brief  float loop constrain
  * @param  Input    the specified variables
  * @param  minValue minimum number of the specified variables
  * @param  maxValue maximum number of the specified variables
  * @retval variables
  */
float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
  if (Max_Value < Min_Value)
  {
    return Input;
  }
  
  float len = Max_Value - Min_Value;    

  if (Input > Max_Value)
  {
      do{
          Input -= len;
      }while (Input > Max_Value);
  }
  else if (Input < Min_Value)
  {
      do{
          Input += len;
      }while (Input < Min_Value);
  }
  return Input;
}

//------------------------------------------------------------------------------

/**
  * @brief  transform the Encoder(0-8192) to anglesum(3.4E38)
  * @param  *Info        pointer to a Motor_Data_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXEncoder   the specified motor max Encoder number
  * @retval anglesum
  */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_t *Data,float Torque_Ratio,uint16_t MAXEncoder)
{
  float res1 = 0,res2 =0;
  
  if(Data == NULL) return 0;
  
  /* Judge the motor Initlized */
  if(Data->Initlized != true)
  {
    /* update the last Encoder */
    Data->Last_Encoder = Data->Encoder;

    /* reset the angle */
    Data->Angle = 0;

    /* Set the init flag */
    Data->Initlized = true;
  }
  
  /* get the possiable min Encoder err */
  if(Data->Encoder < Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder;
  }
  else if(Data->Encoder > Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder;
  }
  res2 = Data->Encoder - Data->Last_Encoder;
  
  /* update the last Encoder */
  Data->Last_Encoder = Data->Encoder;
  
  /* transforms the Encoder data to tolangle */
	if(fabsf(res1) > fabsf(res2))
	{
		Data->Angle += (float)res2/(MAXEncoder*Torque_Ratio)*360.f;
	}
	else
	{
		Data->Angle += (float)res1/(MAXEncoder*Torque_Ratio)*360.f;
	}
  
  return Data->Angle;
}

//------------------------------------------------------------------------------

/**
  * @brief  transform the Encoder(0-8192) to angle(-180-180)
  * @param  *Data        pointer to a Motor_Data_Typedef structure that 
	*					             contains the Data for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXEncoder   the specified motor max Encoder number
  * @retval angle
  */
static float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_t *Data,float torque_ratio,uint16_t MAXEncoder)
{	
  float Encoder_Err = 0.f;
  
  /* check the motor init */
  if(Data->Initlized != true)
  {
    /* update the last Encoder */
    Data->Last_Encoder = Data->Encoder;

    /* reset the angle */
    Data->Angle = Data->Encoder/(MAXEncoder*torque_ratio)*360.f;

    /* config the init flag */
    Data->Initlized = true;
  }
  
  Encoder_Err = Data->Encoder - Data->Last_Encoder;
  
  /* 0 -> MAXEncoder */		
  if(Encoder_Err > MAXEncoder*0.5f)
  {
    Data->Angle += (float)(Encoder_Err - MAXEncoder)/(MAXEncoder*torque_ratio)*360.f;
  }
  /* MAXEncoder-> 0 */		
  else if(Encoder_Err < -MAXEncoder*0.5f)
  {
    Data->Angle += (float)(Encoder_Err + MAXEncoder)/(MAXEncoder*torque_ratio)*360.f;
  }
  else
  {
    Data->Angle += (float)(Encoder_Err)/(MAXEncoder*torque_ratio)*360.f;
  }
  
  /* update the last Encoder */
  Data->Last_Encoder = Data->Encoder;
  
  /* loop constrain */
  Data->Angle = F_Loop_Constrain(Data->Angle,-180.f,180.f);

  return Data->Angle;
}

//------------------------------------------------------------------------------	

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits){

	/* converts float to unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}