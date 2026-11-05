/***************************************************************************************************
 * @file    tb6612_motor.h
 * @brief   TB6612 四路电机驱动 - 麦轮X型布局
 * @note    motorA=左前, motorB=左后, motorC=右后, motorD=右前
 **************************************************************************************************/

#ifndef __TB6612_MOTOR_H
#define __TB6612_MOTOR_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* PWM最大值 (1000 = 100%) */
#define MOTOR_PWM_MAX           1000

/* 电机编号 */
typedef enum {
    MOTOR_A = 0,    // 左前 (Front Left)
    MOTOR_B = 1,    // 左后 (Rear Left)
    MOTOR_C = 2,    // 右后 (Rear Right)
    MOTOR_D = 3,    // 右前 (Front Right)
    MOTOR_COUNT = 4
} MotorID_t;

/* 函数声明 */
void Motor_Init(void);
/* Signed PWM: positive = wheel forward, negative = reverse, zero = stop.
 * Installation polarity A/B/C/D = -1/+1/+1/-1 is applied by the driver. */
void Motor_SetSpeed(MotorID_t motor, int16_t speed);
void Motor_SetAllSpeed(int16_t speedA, int16_t speedB, int16_t speedC, int16_t speedD);
void Motor_Stop(MotorID_t motor);
void Motor_StopAll(void);
void Motor_Enable(void);
void Motor_Disable(void);

#endif /* __TB6612_MOTOR_H */


