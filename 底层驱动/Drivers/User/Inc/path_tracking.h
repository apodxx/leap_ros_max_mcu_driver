/***************************************************************************************************
 * @file    path_tracking.h
 * @brief   路径跟踪系统 - 基于里程计和IMU的精确导航
 **************************************************************************************************/

#ifndef __PATH_TRACKING_H
#define __PATH_TRACKING_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "mecanum_kinematics.h"
#include "pid_controller.h"

/* 路径点结构 */
typedef struct {
    float x;            // X坐标 (m)
    float y;            // Y坐标 (m)
    float theta;        // 目标朝向 (rad)
    float speed;        // 期望速度 (m/s)
} Waypoint_t;

/* 路径跟踪状态 */
typedef enum {
    PATH_IDLE = 0,
    PATH_RUNNING,
    PATH_REACHED,
    PATH_ERROR
} PathState_t;

/* 路径跟踪器结构 */
typedef struct {
    Waypoint_t *waypoints;      // 路径点数组
    uint16_t waypoint_count;    // 路径点总数
    uint16_t current_waypoint;  // 当前目标路径点索引
    PathState_t state;          // 跟踪状态

    PID_Controller_t pid_x;     // X方向位置PID
    PID_Controller_t pid_y;     // Y方向位置PID
    PID_Controller_t pid_theta; // 角度PID

    float position_tolerance;   // 位置到达容差 (m)
    float angle_tolerance;      // 角度到达容差 (rad)

    VelocityCmd_t output_cmd;   // 输出速度指令
} PathTracker_t;

/* 函数声明 */

/* 路径跟踪器初始化 */
void PathTracker_Init(PathTracker_t *tracker, Waypoint_t *waypoints, uint16_t count);

/* 路径跟踪更新 (每个控制周期调用) */
void PathTracker_Update(PathTracker_t *tracker, const Odometry_t *odom, float dt);

/* 启动路径跟踪 */
void PathTracker_Start(PathTracker_t *tracker);

/* 停止路径跟踪 */
void PathTracker_Stop(PathTracker_t *tracker);

/* 复位路径跟踪器 */
void PathTracker_Reset(PathTracker_t *tracker);

/* 获取当前状态 */
PathState_t PathTracker_GetState(const PathTracker_t *tracker);

/* 获取速度指令 */
VelocityCmd_t PathTracker_GetVelocityCmd(const PathTracker_t *tracker);

/* 设置PID参数 */
void PathTracker_SetPIDParams(PathTracker_t *tracker,
                               float kp_pos, float ki_pos, float kd_pos,
                               float kp_angle, float ki_angle, float kd_angle);

#endif /* __PATH_TRACKING_H */
