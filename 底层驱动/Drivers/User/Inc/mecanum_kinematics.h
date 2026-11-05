/***************************************************************************************************
 * @file    mecanum_kinematics.h
 * @brief   麦轮运动学控制系统 - X型布局
 * @note    电机布局:
 *          A(左前)     C(右前)
 *               \  ↑  /
 *                \ | /
 *                 \|/
 *               机器人中心
 *                 /|\
 *                / | \
 *               /  ↓  \
 *          B(左后)     D(右后)
 **************************************************************************************************/

#ifndef __MECANUM_KINEMATICS_H
#define __MECANUM_KINEMATICS_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <math.h>

/* 机器人物理参数 */
#define ROBOT_WHEEL_BASE        0.30f   // 轮距(前后轴距) 单位:米
#define ROBOT_TRACK_WIDTH       0.35f   // 轮距(左右轴距) 单位:米
#define ROBOT_WHEEL_RADIUS      0.04f   // 轮子半径 单位:米
#define ENCODER_PPR             68000    // 编码器每转脉冲数(减速比考虑在内)

/* 速度数据结构 */
typedef struct {
    float vx;       // X方向速度 (m/s, 前进为正)
    float vy;       // Y方向速度 (m/s, 左侧为正)
    float omega;    // 角速度 (rad/s, 逆时针为正)
} VelocityCmd_t;

/* 位置数据结构 */
typedef struct {
    float x;        // X位置 (m)
    float y;        // Y位置 (m)
    float theta;    // 航向角 (rad)
} Position_t;

/* 里程计数据结构 */
typedef struct {
    Position_t position;
    VelocityCmd_t velocity;
    uint32_t timestamp_ms;
} Odometry_t;

/* 电机速度结构 (PWM值) */
typedef struct {
    int16_t motor_a;    // 左前
    int16_t motor_b;    // 左后
    int16_t motor_c;    // 右前
    int16_t motor_d;    // 右后
} MotorSpeed_t;

/* 编码器数据结构 */
typedef struct {
    int16_t encoder_1;  // 对应motor_a
    int16_t encoder_2;  // 对应motor_b
    int16_t encoder_3;  // 对应motor_c
    int16_t encoder_4;  // 对应motor_d
} EncoderDelta_t;

/* 函数声明 */

/* 运动学正解: 编码器增量 -> 速度 -> 位置更新 */
void Mecanum_ForwardKinematics(const EncoderDelta_t *encoder_delta,
                                 float dt,
                                 Odometry_t *odom);

/* 运动学逆解: 目标速度 -> 各轮速度 */
void Mecanum_InverseKinematics(const VelocityCmd_t *vel_cmd,
                                MotorSpeed_t *motor_speed);

/* 根据编码器反馈计算实际轮速 (m/s) */
void Mecanum_ComputeWheelVelocity(const EncoderDelta_t *encoder_delta,
                                   float dt,
                                   float wheel_vel[4]);

/* 里程计初始化 */
void Mecanum_OdomInit(Odometry_t *odom);

/* 里程计复位 */
void Mecanum_OdomReset(Odometry_t *odom);

/* 速度限幅 */
void Mecanum_LimitVelocity(VelocityCmd_t *vel_cmd,
                            float max_vx,
                            float max_vy,
                            float max_omega);

#endif /* __MECANUM_KINEMATICS_H */

