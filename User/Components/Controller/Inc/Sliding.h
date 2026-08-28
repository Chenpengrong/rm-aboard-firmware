/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Sliding.h
  * @brief          : Sliding Mode Control (SMC) Algorithm
  * @author         : Chen
  * @date           : 2026/08/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Supports 5 SMC modes: EXPONENT, POWER, TFSMC, VELSMC, EISMC
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SLIDING_H
#define SLIDING_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "math.h"

/* Macros --------------------------------------------------------------------*/
#define SAMPLE_PERIOD             0.002f
#define V_EORROR_INTEGRAL_MAX     2000.0f
#define P_EORROR_INTEGRAL_MAX     2000.0f

/* Enums ---------------------------------------------------------------------*/

/**
 * @brief 滑模控制模式枚举
 */
typedef enum
{
    EXPONENT,   /* 指数趋近律（线性滑模面） */
    POWER,      /* 幂次趋近律（线性滑模面） */
    TFSMC,      /* 终端快速滑模（非线性滑模面） */
    VELSMC,     /* 速度滑模（含积分） */
    EISMC       /* 误差积分滑模（位置环） */
} Rmode_e;

/* Structs -------------------------------------------------------------------*/

/**
 * @brief 误差状态结构体
 */
typedef struct
{
    float tar_now;                  /* 当前目标值 */
    float tar_last;                 /* 上一次目标值 */
    float tar_differential;         /* 目标值一阶导 */
    float tar_differential_last;    /* 上一次目标值一阶导 */
    float tar_differential_second;  /* 目标值二阶导 */

    float pos_get;                  /* 当前位置 */
    float vol_get;                  /* 当前速度 */

    float p_error;                  /* 位置误差 */
    float v_error;                  /* 速度误差（位置误差的一阶导） */

    float p_error_integral;         /* 位置误差积分 */
    float v_error_integral;         /* 速度误差积分 */

    float pos_error_eps;            /* 位置误差精度（死区） */
    float vol_error_eps;            /* 速度误差精度（死区） */
    float error_last;               /* 上一次误差 */
} RError_t;

/**
 * @brief 滑模控制参数结构体
 */
typedef struct
{
    float J;        /* 转动惯量 */
    float K;        /* 趋近律增益 */
    float c;        /* 滑模面系数（EXPONENT/POWER/VELSMC 使用） */

    float c1;       /* EISMC 滑模面系数1 */
    float c2;       /* EISMC 滑模面系数2 */

    float p;        /* TFSMC 幂次参数（p > q） */
    float q;        /* TFSMC 幂次参数 */
    float beta;     /* TFSMC 滑模面系数 */
    float epsilon;  /* 边界层厚度（抗抖振） */
} SlidingParam_t;

/**
 * @brief 滑模控制器结构体
 */
typedef struct
{
    float u;                    /* 控制输出 */
    float s;                    /* 滑模面当前值 */

    SlidingParam_t param;       /* 当前控制参数 */
    SlidingParam_t param_last;  /* 上一次控制参数（用于参数连续化） */

    RError_t error;             /* 误差状态 */
    float u_max;                /* 输出限幅 */
    Rmode_e flag;               /* 控制模式 */
    float limit;                /* 饱和函数限幅 */
} Sliding_t;

/* Externs -------------------------------------------------------------------*/

extern void SMC_Init(Sliding_t *smc);

/* 参数设置：EXPONENT / POWER / VELSMC 模式 */
extern void SMC_SetParam_Standard(Sliding_t *smc, float J, float K, float c,
                                  float epsilon, float limit, float u_max,
                                  Rmode_e flag, float pos_esp);

/* 参数设置：TFSMC 模式 */
extern void SMC_SetParam_TFSMC(Sliding_t *smc, float J, float K, float p, float q,
                               float beta, float epsilon, float limit,
                               float u_max, Rmode_e flag, float pos_esp);

/* 参数设置：EISMC 模式 */
extern void SMC_SetParam_EISMC(Sliding_t *smc, float J, float K, float c1, float c2,
                               float epsilon, float limit, float u_max,
                               Rmode_e flag, float pos_esp);

/* 误差更新：位置环 */
extern void SMC_ErrorUpdate_Position(Sliding_t *smc, float target,
                                     float pos_now, float vol_now);

/* 误差更新：速度环 */
extern void SMC_ErrorUpdate_Velocity(Sliding_t *smc, float target, float vol_now);

/* 清除所有状态 */
extern void SMC_Clear(Sliding_t *smc);

/* 清除积分项 */
extern void SMC_Integral_Clear(Sliding_t *smc);

/* 滑模控制律计算 */
extern float SMC_Calculate(Sliding_t *smc);

/* 获取当前输出 */
extern float SMC_Out(const Sliding_t *smc);

/* 设置输出值 */
extern void SMC_SetOut(Sliding_t *smc, float out);

/* 获取控制器内部数据（只读） */
extern const Sliding_t *SMC_GetData(const Sliding_t *smc);

#endif /* SLIDING_H */