/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Sliding.c
  * @brief          : Sliding Mode Control (SMC) Algorithm Implementation
  * @author         : Chen
  * @date           : 2026/08/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 5 modes: EXPONENT, POWER, TFSMC, VELSMC, EISMC
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Sliding.h"

/* Private function declarations ---------------------------------------------*/

static void SMC_OutContinuation(Sliding_t *smc);
static float SMC_Signal(float s);
static float SMC_Sat(Sliding_t *smc, float s);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief 初始化滑模控制器
 */
void SMC_Init(Sliding_t *smc)
{
    smc->param.J = 0.0f;
    smc->param.K = 0.0f;
    smc->param.c = 0.0f;
    smc->param.epsilon = 0.0f;
    smc->flag = EXPONENT;
    smc->u_max = 0.0f;
    smc->limit = 0.0f;

    smc->error.tar_now = 0.0f;
    smc->error.tar_last = 0.0f;
    smc->error.tar_differential = 0.0f;

    smc->error.p_error = 0.0f;
    smc->error.v_error = 0.0f;
    smc->error.v_error_integral = 0.0f;
    smc->error.pos_error_eps = 0.0f;
    smc->error.vol_error_eps = 0.0f;
    smc->error.pos_get = 0.0f;
    smc->error.vol_get = 0.0f;
}

/**
 * @brief 参数设置：EXPONENT / POWER / VELSMC 模式
 */
void SMC_SetParam_Standard(Sliding_t *smc, float J, float K, float c,
                           float epsilon, float limit, float u_max,
                           Rmode_e flag, float pos_esp)
{
    smc->param.J = J;
    smc->param.K = K;
    smc->param.c = c;
    smc->error.pos_error_eps = pos_esp;
    smc->flag = flag;
    smc->param.epsilon = epsilon;
    smc->u_max = u_max;
    smc->limit = limit;
    SMC_OutContinuation(smc);
}

/**
 * @brief 参数设置：TFSMC 模式
 */
void SMC_SetParam_TFSMC(Sliding_t *smc, float J, float K, float p, float q,
                        float beta, float epsilon, float limit,
                        float u_max, Rmode_e flag, float pos_esp)
{
    smc->param.J = J;
    smc->param.K = K;
    smc->param.p = p;
    smc->param.q = q;
    smc->error.pos_error_eps = pos_esp;
    smc->param.beta = beta;
    smc->flag = flag;
    smc->param.epsilon = epsilon;
    smc->u_max = u_max;
    smc->limit = limit;
    SMC_OutContinuation(smc);
}

/**
 * @brief 参数设置：EISMC 模式
 */
void SMC_SetParam_EISMC(Sliding_t *smc, float J, float K, float c1, float c2,
                        float epsilon, float limit, float u_max,
                        Rmode_e flag, float pos_esp)
{
    smc->param.J = J;
    smc->param.K = K;
    smc->param.c1 = c1;
    smc->param.c2 = c2;
    smc->error.pos_error_eps = pos_esp;
    smc->flag = flag;
    smc->param.epsilon = epsilon;
    smc->u_max = u_max;
    smc->limit = limit;
    SMC_OutContinuation(smc);
}

/**
 * @brief 误差更新：位置环
 * @param target   目标值
 * @param pos_now  当前位置
 * @param vol_now  当前速度
 */
void SMC_ErrorUpdate_Position(Sliding_t *smc, float target,
                              float pos_now, float vol_now)
{
    smc->error.tar_now = target;
    smc->error.tar_differential = (smc->error.tar_now - smc->error.tar_last)
                                  / SAMPLE_PERIOD;

    smc->error.tar_differential_second = (smc->error.tar_differential
                                          - smc->error.tar_differential_last)
                                         / SAMPLE_PERIOD;

    smc->error.p_error = pos_now - target;
    smc->error.v_error = vol_now - smc->error.tar_differential;
    smc->error.tar_last = smc->error.tar_now;

    smc->error.p_error_integral += smc->error.p_error * SAMPLE_PERIOD;

    smc->error.tar_differential_last = smc->error.tar_differential;
}

/**
 * @brief 误差更新：速度环
 * @param target   目标速度
 * @param vol_now  当前速度
 */
void SMC_ErrorUpdate_Velocity(Sliding_t *smc, float target, float vol_now)
{
    smc->error.tar_now = target;
    smc->error.tar_differential = (smc->error.tar_now - smc->error.tar_last)
                                  / SAMPLE_PERIOD;

    smc->error.v_error = vol_now - smc->error.tar_now;

    smc->error.v_error_integral += smc->error.v_error * SAMPLE_PERIOD;

    smc->error.tar_last = smc->error.tar_now;
}

/**
 * @brief 清除所有误差状态
 */
void SMC_Clear(Sliding_t *smc)
{
    smc->error.tar_now = 0.0f;
    smc->error.tar_last = 0.0f;
    smc->error.tar_differential = 0.0f;

    smc->error.p_error = 0.0f;
    smc->error.v_error = 0.0f;
    smc->error.v_error_integral = 0.0f;
    smc->error.pos_error_eps = 0.0f;
    smc->error.vol_error_eps = 0.0f;
    smc->error.pos_get = 0.0f;
    smc->error.vol_get = 0.0f;

    smc->error.tar_differential_second = 0.0f;
    smc->error.tar_differential_last = 0.0f;
    smc->error.p_error_integral = 0.0f;
}

/**
 * @brief 清除积分项
 */
void SMC_Integral_Clear(Sliding_t *smc)
{
    smc->error.v_error_integral = 0.0f;
    smc->error.p_error_integral = 0.0f;
}

/**
 * @brief 滑模控制律计算（核心函数）
 * @return 控制输出 u
 */
float SMC_Calculate(Sliding_t *smc)
{
    float u;
    float fun;
    static float pos_pow; /* TFSMC 位置幂次项，static 保持跨调用 */

    switch (smc->flag)
    {
    case EXPONENT:
        /* 线性滑模面，指数趋近律 */
        if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f)
        {
            smc->error.p_error = 0.0f;
            return 0.0f;
        }

        smc->s = smc->param.c * smc->error.p_error + smc->error.v_error;
        fun = SMC_Sat(smc, smc->s);
        u = smc->param.J * (-smc->param.c * smc->error.v_error
                            - smc->param.K * smc->s
                            - smc->param.epsilon * fun);
        break;

    case POWER:
        /* 线性滑模面，幂次趋近律 */
        if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f)
        {
            smc->error.p_error = 0.0f;
            return 0.0f;
        }

        smc->s = smc->param.c * smc->error.p_error + smc->error.v_error;
        fun = SMC_Sat(smc, smc->s);
        u = smc->param.J * (-smc->param.c * smc->error.v_error
                            - smc->param.K * smc->s
                            - smc->param.K * powf(fabsf(smc->s), smc->param.epsilon) * fun);
        break;

    case TFSMC:
        /* 终端快速滑模，非线性滑模面 */
        if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f)
        {
            smc->error.p_error = 0.0f;
            return 0.0f;
        }

        pos_pow = powf(fabsf(smc->error.p_error), smc->param.q / smc->param.p);
        if (smc->error.p_error <= 0.0f)
        {
            pos_pow = -pos_pow;
        }

        smc->s = smc->param.beta * pos_pow + smc->error.v_error;
        fun = SMC_Sat(smc, smc->s);

        if (smc->error.p_error != 0.0f)
        {
            u = smc->param.J * (smc->error.tar_differential_second
                                - smc->param.K * smc->s
                                - smc->param.epsilon * fun
                                - smc->error.v_error
                                  * ((smc->param.q * smc->param.beta) * pos_pow)
                                  / (smc->param.p * smc->error.p_error));
        }
        else
        {
            u = 0.0f;
        }
        break;

    case VELSMC:
        /* 速度滑模，含速度误差积分 */
        smc->s = smc->error.v_error + smc->param.c * smc->error.v_error_integral;
        fun = SMC_Sat(smc, smc->s);
        u = smc->param.J * (smc->error.tar_differential
                            - smc->param.c * smc->error.v_error
                            - smc->param.K * smc->s
                            - smc->param.epsilon * fun);
        break;

    case EISMC:
        /* 误差积分滑模，位置环 */
        if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f)
        {
            smc->error.p_error = 0.0f;
            return 0.0f;
        }

        smc->s = smc->param.c1 * smc->error.p_error
                 + smc->error.v_error
                 + smc->param.c2 * smc->error.p_error_integral;
        fun = SMC_Sat(smc, smc->s);
        u = smc->param.J * (-smc->param.c1 * smc->error.v_error
                            - smc->param.c2 * smc->error.p_error
                            - smc->param.K * smc->s
                            - smc->param.epsilon * fun);
        break;

    default:
        u = 0.0f;
        break;
    }

    smc->error.error_last = smc->error.p_error;

    /* 输出限幅 */
    if (u > smc->u_max)
    {
        u = smc->u_max;
    }
    if (u < -smc->u_max)
    {
        u = -smc->u_max;
    }
    smc->u = u;
    return u;
}

/**
 * @brief 获取当前输出值
 */
float SMC_Out(const Sliding_t *smc)
{
    return smc->u;
}

/**
 * @brief 设置输出值
 */
void SMC_SetOut(Sliding_t *smc, float out)
{
    smc->u = out;
}

/**
 * @brief 获取控制器内部数据（只读）
 */
const Sliding_t *SMC_GetData(const Sliding_t *smc)
{
    return smc;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 参数连续化处理
 * @note  当运行时切换参数时，对积分项进行比例缩放，防止输出跳变
 */
static void SMC_OutContinuation(Sliding_t *smc)
{
    if (smc->param.K != 0.0f && smc->param.c2 != 0.0f)
    {
        smc->error.p_error_integral = (smc->param_last.K / smc->param.K)
                                      * (smc->param_last.c2 / smc->param.c2)
                                      * smc->error.p_error_integral;
        smc->error.v_error_integral = (smc->param_last.K / smc->param.K)
                                      * (smc->param_last.c / smc->param.c)
                                      * smc->error.v_error_integral;
    }
    smc->param_last = smc->param;
}

/**
 * @brief 符号函数
 */
static float SMC_Signal(float s)
{
    if (s > 0.0f)
        return 1.0f;
    else if (s == 0.0f)
        return 0.0f;
    else
        return -1.0f;
}

/**
 * @brief 饱和函数（替代符号函数，抑制抖振）
 * @note  边界层内线性输出，边界层外符号函数输出
 */
static float SMC_Sat(Sliding_t *smc, float s)
{
    float y;
    y = s / smc->param.epsilon;
    if (fabsf(y) <= smc->limit)
        return y;
    else
        return SMC_Signal(y);
}