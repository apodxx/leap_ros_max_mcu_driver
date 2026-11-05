/***************************************************************************************************
 * @file    path_tracking.c
 * @brief   路径跟踪系统实现
 **************************************************************************************************/

#include "path_tracking.h"
#include <math.h>

/* 角度归一化到[-π, π] */
static float normalize_angle(float angle)
{
    while (angle > 3.14159265f) angle -= 6.28318531f;
    while (angle < -3.14159265f) angle += 6.28318531f;
    return angle;
}

/**
 * @brief  路径跟踪器初始化
 */
void PathTracker_Init(PathTracker_t *tracker, Waypoint_t *waypoints, uint16_t count)
{
    tracker->waypoints = waypoints;
    tracker->waypoint_count = count;
    tracker->current_waypoint = 0;
    tracker->state = PATH_IDLE;

    /* 默认PID参数 (需要根据实际调试) */
    PID_Init(&tracker->pid_x, 5.0f, 3.0f, 0.0f, 10.0f, 2.0f, -2.0f);
    PID_Init(&tracker->pid_y, 5.0f, 3.0f, 0.0f, 10.0f, 2.0f, -2.0f);
    PID_Init(&tracker->pid_theta, 5.0f, 3.0f, 0.0f, 10.0f, 2.0f, -2.0f);

    /* 默认容差 */
    tracker->position_tolerance = 0.05f;    // 5cm
    tracker->angle_tolerance = 0.1f;        // ~5.7度

    tracker->output_cmd.vx = 0.0f;
    tracker->output_cmd.vy = 0.0f;
    tracker->output_cmd.omega = 0.0f;
}

/**
 * @brief  路径跟踪更新 (每个控制周期调用)
 */
void PathTracker_Update(PathTracker_t *tracker, const Odometry_t *odom, float dt)
{
    if (tracker->state != PATH_RUNNING)
        return;

    /* 检查是否完成所有路径点 */
    if (tracker->current_waypoint >= tracker->waypoint_count)
    {
        tracker->state = PATH_REACHED;
        tracker->output_cmd.vx = 0.0f;
        tracker->output_cmd.vy = 0.0f;
        tracker->output_cmd.omega = 0.0f;
        return;
    }

    /* 获取当前目标路径点 */
    Waypoint_t *target = &tracker->waypoints[tracker->current_waypoint];

    /* 计算位置误差 (全局坐标系) */
    float dx = target->x - odom->position.x;
    float dy = target->y - odom->position.y;
    float distance = sqrtf(dx * dx + dy * dy);

    /* 计算角度误差 */
    float angle_error = normalize_angle(target->theta - odom->position.theta);

    /* 检查是否到达当前路径点 */
    if (distance < tracker->position_tolerance &&
        fabsf(angle_error) < tracker->angle_tolerance)
    {
        /* 前进到下一个路径点 */
        tracker->current_waypoint++;
        PID_Reset(&tracker->pid_x);
        PID_Reset(&tracker->pid_y);
        PID_Reset(&tracker->pid_theta);
        tracker->output_cmd.vx = 0.0f;
        tracker->output_cmd.vy = 0.0f;
        tracker->output_cmd.omega = 0.0f;
        return;
    }

    /* 转换误差到机器人坐标系 */
    float cos_theta = cosf(odom->position.theta);
    /* Inverse of the clockwise-heading body-to-world transform. */
    float sin_theta = sinf(-odom->position.theta);
    float dx_robot = dx * cos_theta + dy * sin_theta;
    float dy_robot = -dx * sin_theta + dy * cos_theta;

    /* PID控制计算速度指令 */
    float vx_cmd = PID_Compute(&tracker->pid_x, 0.0f, -dx_robot, dt);
    float vy_cmd = PID_Compute(&tracker->pid_y, 0.0f, -dy_robot, dt);
    float omega_cmd = PID_Compute(&tracker->pid_theta, 0.0f, -angle_error, dt);

    /* speed 表示期望平移速度上限，而不是 PID 输出的乘数 */
    float speed_limit = fabsf(target->speed);
    if (speed_limit < 0.01f) speed_limit = 0.01f;
    if (distance < 0.2f)  // 接近目标时减速
    {
        speed_limit *= (distance / 0.2f);
    }

    {
        float cmd_norm = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
        if (cmd_norm > speed_limit && cmd_norm > 0.0001f)
        {
            float scale = speed_limit / cmd_norm;
            vx_cmd *= scale;
            vy_cmd *= scale;
        }
    }

    /* 更新输出指令 */
    tracker->output_cmd.vx = vx_cmd;
    tracker->output_cmd.vy = vy_cmd;
    tracker->output_cmd.omega = omega_cmd;

    /* 速度限幅 */
    Mecanum_LimitVelocity(&tracker->output_cmd, 0.3f, 0.3f, 1.0f);
}

/**
 * @brief  启动路径跟踪
 */
void PathTracker_Start(PathTracker_t *tracker)
{
    if (tracker->waypoint_count > 0)
    {
        tracker->state = PATH_RUNNING;
        tracker->current_waypoint = 0;
        PID_Reset(&tracker->pid_x);
        PID_Reset(&tracker->pid_y);
        PID_Reset(&tracker->pid_theta);
    }
}

/**
 * @brief  停止路径跟踪
 */
void PathTracker_Stop(PathTracker_t *tracker)
{
    tracker->state = PATH_IDLE;
    tracker->output_cmd.vx = 0.0f;
    tracker->output_cmd.vy = 0.0f;
    tracker->output_cmd.omega = 0.0f;
}

/**
 * @brief  复位路径跟踪器
 */
void PathTracker_Reset(PathTracker_t *tracker)
{
    tracker->current_waypoint = 0;
    tracker->state = PATH_IDLE;
    PID_Reset(&tracker->pid_x);
    PID_Reset(&tracker->pid_y);
    PID_Reset(&tracker->pid_theta);
    tracker->output_cmd.vx = 0.0f;
    tracker->output_cmd.vy = 0.0f;
    tracker->output_cmd.omega = 0.0f;
}

/**
 * @brief  获取当前状态
 */
PathState_t PathTracker_GetState(const PathTracker_t *tracker)
{
    return tracker->state;
}

/**
 * @brief  获取速度指令
 */
VelocityCmd_t PathTracker_GetVelocityCmd(const PathTracker_t *tracker)
{
    return tracker->output_cmd;
}

/**
 * @brief  设置PID参数
 */
void PathTracker_SetPIDParams(PathTracker_t *tracker,
                               float kp_pos, float ki_pos, float kd_pos,
                               float kp_angle, float ki_angle, float kd_angle)
{
    (void)tracker; (void)kp_pos; (void)ki_pos; (void)kd_pos;
    (void)kp_angle; (void)ki_angle; (void)kd_angle;
}
