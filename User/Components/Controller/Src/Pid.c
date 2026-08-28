/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : PID.c
  * @brief          : PID Algorithm
  * @author         : Chen
  * @date           : 2026/08/23
  * @version        : v1.1
  ******************************************************************************
  * @attention      : Choose suitable PID according to your actual situation
  ******************************************************************************
  */
/* USER CODE END Header */

/* Including ------------------------------------------------------------------*/
#include "Pid.h"


/**
 * @brief 初始化PID控制器
 */
void PID_Init(PID_t *pid)
{
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->Kf = 0.0f;
    pid->accrued_error = 0.0f;
    pid->history_error = 0.0f;
    pid->max_accrued = 0.0f;
    pid->max_output = 0.0f;
    pid->dead_out = 0.0f;
}

/**
 * @brief  PID参数设置
 * @param  pid         PID控制器句柄
 * @param  Kp          比例增益
 * @param  Ki          积分增益
 * @param  Kd          微分增益
 * @param  Kf          前馈增益
 * @param  max_accrued 积分限幅
 * @param  max_output  输出限幅
 * @param  dead_out    死区阈值
 */
void PID_SetParam(PID_t *pid, float Kp, float Ki, float Kd, float Kf,
                  float max_accrued, float max_output, float dead_out)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;
    pid->max_accrued = max_accrued;
    pid->max_output = max_output;
    pid->dead_out = dead_out;
}

/**
 * @brief 最简单的PID控制器计算函数
 * @param pid: PID 参数结构体指针
 * @param error: 当前误差 = 目标值 - 实际值
 * @return 输出值（已限幅）
 */
float PIDCompute(PID_t *pid, float error)
{
    // 比例项：误差直接乘以 Kp
    float p_out = pid->Kp * error;

    // 积分项：累加误差，防止积分饱和
    pid->accrued_error += error;
    if (pid->accrued_error > pid->max_accrued)
        pid->accrued_error = pid->max_accrued;
    else if (pid->accrued_error < -pid->max_accrued)
        pid->accrued_error = -pid->max_accrued;
    float i_out = pid->Ki * pid->accrued_error;

    // 微分项：用当前误差与上一次误差的差值
    float d_out = pid->Kd * (error - pid->history_error);
    pid->history_error = error; // 更新历史误差

    // 总输出 = P + I + D
    float output = p_out + i_out + d_out;

    // 输出限幅：防止超出电机允许的最大控制值
    if (output > pid->max_output)
        return pid->max_output;
    if (output < -pid->max_output)
        return -pid->max_output;

    // 死区处理：如果输出太小，直接归零（防止抖动）
    if (fabs(output) < pid->dead_out)
        return 0.0f;

    return output;
}

/**
 * @brief [跟踪型] PID 计算 (带速度/变化率前馈)
 * @description 适用于需要快速跟随目标变化的场景（如速度环、轨迹跟踪）。
            前馈项基于目标的变化率 (dv/dt)，用于减少动态滞后。
 * @param pid: PID 结构体指针
 * @param error: 当前误差 (Target - Measure)
 * @param target_rate: 目标的变化率 (例如：目标速度增量/时间，或目标加速度)
 * @return 控制输出
 */
float PIDCompute_Tracking(PID_t *pid, float error, float target_rate)
{
    // 比例项
    float p_out = pid->Kp * error;

    // 积分项
    pid->accrued_error += error;
    // 积分限幅 (Anti-windup)
    if (pid->accrued_error > pid->max_accrued)
        pid->accrued_error = pid->max_accrued;
    else if (pid->accrued_error < -pid->max_accrued)
        pid->accrued_error = -pid->max_accrued;
    
    float i_out = pid->Ki * pid->accrued_error;

    // 微分项
    float d_out = pid->Kd * (error - pid->history_error);
    pid->history_error = error; 

    // [跟踪前馈] = Kf * 目标变化率
    // 作用：预判目标走势，提前输出，减小相位滞后
    float ff_out = pid->Kf * target_rate;

    // 总和
    float output = p_out + i_out + d_out + ff_out;

    // 输出限幅
    if (output > pid->max_output)
        return pid->max_output;
    if (output < -pid->max_output)
        return -pid->max_output;

    // 死区处理
    if (fabsf(output) < pid->dead_out)
        return 0.0f;

    return output;
}


/**
 * @brief [抗扰型] PID 计算 (带静态/重力前馈)
 * @description 适用于需要克服恒定干扰的场景（如机械臂重力补偿、摩擦力补偿）。
 *              前馈项基于目标本身的状态 (例如：目标角度对应的重力分量)。
 * @param pid: PID 结构体指针
 * @param error: 当前误差 (Target - Measure)
 * @param feedforward_static: 静态前馈量 (例如：sin(target_angle) * 重力系数)
 * @return 控制输出
 */
float PIDCompute_Disturbance(PID_t *pid, float error, float feedforward_static)
{
    // 比例项
    float p_out = pid->Kp * error;

    // 积分项
    pid->accrued_error += error;
    if (pid->accrued_error > pid->max_accrued)
        pid->accrued_error = pid->max_accrued;
    else if (pid->accrued_error < -pid->max_accrued)
        pid->accrued_error = -pid->max_accrued;
    
    float i_out = pid->Ki * pid->accrued_error;

    // 微分项
    float d_out = pid->Kd * (error - pid->history_error);
    pid->history_error = error; 

    // [抗扰前馈] = Kf * 静态干扰模型值
    // 作用：直接抵消已知的恒定干扰（如重力），减轻积分项负担
    float ff_out = pid->Kf * feedforward_static;

    // 总和
    float output = p_out + i_out + d_out + ff_out;

    // 输出限幅
    if (output > pid->max_output)
        return pid->max_output;
    if (output < -pid->max_output)
        return -pid->max_output;

    // 死区处理
    if (fabsf(output) < pid->dead_out)
        return 0.0f;

    return output;
}