/***************************************************************************************************
 * @file    pid_controller.h
 * @brief   PID控制器 - 用于速度和位置闭环控制
 **************************************************************************************************/

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include <stdint.h>

/* PID参数结构 */
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分累积
    float prev_error;   // 上次误差
    float prev_measured;
    float integral_separation; // <= 0 disables separation; same units as error.
    uint8_t derivative_on_measurement;
    uint8_t measurement_valid;
    float integral_max; // 积分限幅
    float output_max;   // 输出限幅
    float output_min;   // 输出最小值
} PID_Controller_t;

/* 函数声明 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float integral_max, float output_max, float output_min);
float PID_Compute(PID_Controller_t *pid, float setpoint, float measured, float dt);
void PID_Reset(PID_Controller_t *pid);
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd);
void PID_SetOptions(PID_Controller_t *pid, float integral_separation,
                    uint8_t derivative_on_measurement);

#endif /* __PID_CONTROLLER_H */
