/***************************************************************************************************
 * @file    mecanum_kinematics.c
 * @brief   麦轮运动学控制实现
 **************************************************************************************************/

#include "mecanum_kinematics.h"

/* 角度归一化到[-π, π] */
static float normalize_angle(float angle)
{
    while (angle > 3.14159265f) angle -= 6.28318531f;
    while (angle < -3.14159265f) angle += 6.28318531f;
    return angle;
}

/**
 * @brief  里程计初始化
 */
void Mecanum_OdomInit(Odometry_t *odom)
{
    odom->position.x = 0.0f;
    odom->position.y = 0.0f;
    odom->position.theta = 0.0f;
    odom->velocity.vx = 0.0f;
    odom->velocity.vy = 0.0f;
    odom->velocity.omega = 0.0f;
    odom->timestamp_ms = HAL_GetTick();
}

/**
 * @brief  里程计复位
 */
void Mecanum_OdomReset(Odometry_t *odom)
{
    Mecanum_OdomInit(odom);
}

/**
 * @brief  根据编码器增量计算实际轮速
 * @param  encoder_delta: 编码器增量
 * @param  dt: 时间间隔(秒)
 * @param  wheel_vel: 输出4个轮子速度(m/s)
 */
void Mecanum_ComputeWheelVelocity(const EncoderDelta_t *encoder_delta,
                                   float dt,
                                   float wheel_vel[4])
{
    if (dt < 0.0001f) dt = 0.0001f;  // 防止除零

    /* 编码器脉冲 -> 轮子转数 -> 线速度 */
    wheel_vel[0] = (encoder_delta->encoder_1 / (float)ENCODER_PPR) *
                   (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS) / dt;
    wheel_vel[1] = (encoder_delta->encoder_2 / (float)ENCODER_PPR) *
                   (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS) / dt;
    wheel_vel[2] = (encoder_delta->encoder_3 / (float)ENCODER_PPR) *
                   (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS) / dt;
    wheel_vel[3] = (encoder_delta->encoder_4 / (float)ENCODER_PPR) *
                   (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS) / dt;
}

/**
 * @brief  麦轮运动学正解: 编码器数据 -> 机器人速度和位置
 * @param  encoder_delta: 编码器增量
 * @param  dt: 时间间隔(秒)
 * @param  odom: 里程计数据(输入输出)
 * @note   X型麦轮正运动学公式:
 *         vx = (v1 + v2 + v3 + v4) / 4
 *         vy = (-v1 + v2 + v3 - v4) / 4
 *         omega = (v1 + v2 - v3 - v4) / (4 * (lx + ly))，顺时针为正
 *         其中 v1,v2,v3,v4 分别对应左前、左后、右前、右后轮速
 */
void Mecanum_ForwardKinematics(const EncoderDelta_t *encoder_delta,
                                 float dt,
                                 Odometry_t *odom)
{
    float wheel_vel[4];

    /* 计算轮速 */
    Mecanum_ComputeWheelVelocity(encoder_delta, dt, wheel_vel);

    float v1 = wheel_vel[0];  // 左前
    float v2 = wheel_vel[1];  // 左后
    float v3 = wheel_vel[3];  // 右前（电机D/编码器4）
    float v4 = wheel_vel[2];  // 右后（电机C/编码器3）

    /* 计算机器人中心速度(机器人坐标系) */
    float vx_robot = (v1 + v2 + v3 + v4) / 4.0f;
    float vy_robot = (-v1 + v2 + v3 - v4) / 4.0f;
    /* Left wheels minus right wheels: A+B-C-D, clockwise positive. */
    float omega = (v1 + v2 - v3 - v4) / (4.0f * (ROBOT_WHEEL_BASE/2.0f + ROBOT_TRACK_WIDTH/2.0f));

    odom->velocity.vx = vx_robot;
    odom->velocity.vy = vy_robot;
    odom->velocity.omega = omega;

    /* 转换到全局坐标系并更新位置 */
    float cos_theta = cosf(odom->position.theta);
    /* Heading is clockwise-positive; world XY uses counterclockwise angles. */
    float sin_theta = sinf(-odom->position.theta);

    float vx_global = vx_robot * cos_theta - vy_robot * sin_theta;
    float vy_global = vx_robot * sin_theta + vy_robot * cos_theta;

    odom->position.x += vx_global * dt;
    odom->position.y += vy_global * dt;
    odom->position.theta += omega * dt;
    odom->position.theta = normalize_angle(odom->position.theta);

    odom->timestamp_ms = HAL_GetTick();
}

/**
 * @brief  麦轮运动学逆解: 目标速度 -> 各轮PWM速度
 * @param  vel_cmd: 目标速度(机器人坐标系)
 * @param  motor_speed: 输出各电机PWM值
 * @note   X型麦轮逆运动学公式:
 *         v1 = vx - vy - omega * (lx + ly)  左前
 *         v2 = vx + vy + omega * (lx + ly)  左后
 *         v3 = vx + vy - omega * (lx + ly)  右前
 *         v4 = vx - vy + omega * (lx + ly)  右后
 */
void Mecanum_InverseKinematics(const VelocityCmd_t *vel_cmd,
                                MotorSpeed_t *motor_speed)
{
    float lx_ly = ROBOT_WHEEL_BASE/2.0f + ROBOT_TRACK_WIDTH/2.0f;

    /* 计算各轮线速度(m/s) */
    float v1 = vel_cmd->vx - vel_cmd->vy - vel_cmd->omega * lx_ly;
    float v2 = vel_cmd->vx + vel_cmd->vy + vel_cmd->omega * lx_ly;
    float v3 = vel_cmd->vx + vel_cmd->vy - vel_cmd->omega * lx_ly;
    float v4 = vel_cmd->vx - vel_cmd->vy + vel_cmd->omega * lx_ly;

    /* 转换为PWM值 (线速度 -> 转速 -> PWM) */
    /* 假设最大PWM=1000对应最大轮速约0.5m/s */
    const float PWM_PER_MPS = 2000.0f;  // PWM / (m/s)

    motor_speed->motor_a = (int16_t)(v1 * PWM_PER_MPS);
    motor_speed->motor_b = (int16_t)(v2 * PWM_PER_MPS);
    motor_speed->motor_c = (int16_t)(v4 * PWM_PER_MPS); // C=右后
    motor_speed->motor_d = (int16_t)(v3 * PWM_PER_MPS); // D=右前

    /* PWM限幅 */
    if (motor_speed->motor_a > 1000) motor_speed->motor_a = 1000;
    if (motor_speed->motor_a < -1000) motor_speed->motor_a = -1000;
    if (motor_speed->motor_b > 1000) motor_speed->motor_b = 1000;
    if (motor_speed->motor_b < -1000) motor_speed->motor_b = -1000;
    if (motor_speed->motor_c > 1000) motor_speed->motor_c = 1000;
    if (motor_speed->motor_c < -1000) motor_speed->motor_c = -1000;
    if (motor_speed->motor_d > 1000) motor_speed->motor_d = 1000;
    if (motor_speed->motor_d < -1000) motor_speed->motor_d = -1000;
}

/**
 * @brief  速度限幅
 */
void Mecanum_LimitVelocity(VelocityCmd_t *vel_cmd,
                            float max_vx,
                            float max_vy,
                            float max_omega)
{
    if (vel_cmd->vx > max_vx) vel_cmd->vx = max_vx;
    if (vel_cmd->vx < -max_vx) vel_cmd->vx = -max_vx;
    if (vel_cmd->vy > max_vy) vel_cmd->vy = max_vy;
    if (vel_cmd->vy < -max_vy) vel_cmd->vy = -max_vy;
    if (vel_cmd->omega > max_omega) vel_cmd->omega = max_omega;
    if (vel_cmd->omega < -max_omega) vel_cmd->omega = -max_omega;
}
