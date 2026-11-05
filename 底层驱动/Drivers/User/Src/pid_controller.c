/***************************************************************************************************
 * @file    pid_controller.c
 * @brief   PID控制器实现
 **************************************************************************************************/

#include "pid_controller.h"

/**
 * @brief  PID控制器初始化
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float integral_max, float output_max, float output_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measured = 0.0f;
    pid->measurement_valid = 0U;
    pid->integral_separation = 0.0f;
    pid->derivative_on_measurement = 0U;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->output_min = output_min;
}

/**
 * @brief  PID计算
 * @param  setpoint: 目标值
 * @param  measured: 测量值
 * @param  dt: 时间间隔(秒)
 * @retval 控制输出
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    if (dt <= 0.0001f) dt = 0.0001f;

    /* 比例项 */
    float p_term = pid->kp * error;

    /* 积分项 (带抗饱和) */
    /* Outside the separation band, freeze the accumulator and retain its
       existing output contribution to avoid an abrupt loss of holding PWM. */
    if (pid->integral_separation <= 0.0f ||
        (error >= -pid->integral_separation && error <= pid->integral_separation))
        pid->integral += error * dt;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    /* 微分项 */
    float derivative = (error - pid->prev_error) / dt;
    if (pid->derivative_on_measurement)
        derivative = pid->measurement_valid ? -(measured - pid->prev_measured) / dt : 0.0f;
    float d_term = pid->kd * derivative;

    /* 总输出 */
    float output = p_term + i_term + d_term;

    /* 输出限幅 */
    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;

    pid->prev_error = error;
    pid->prev_measured = measured;
    pid->measurement_valid = 1U;

    return output;
}

/**
 * @brief  复位PID控制器
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measured = 0.0f;
    pid->measurement_valid = 0U;
}

void PID_SetOptions(PID_Controller_t *pid, float integral_separation,
                    uint8_t derivative_on_measurement)
{
    pid->integral_separation = integral_separation;
    pid->derivative_on_measurement = derivative_on_measurement != 0U;
    pid->measurement_valid = 0U;
}

/**
 * @brief  设置PID参数
 */
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
