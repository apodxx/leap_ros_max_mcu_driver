/***************************************************************************************************
 * @file    navigation_controller.h
 * @brief   导航控制器 - 集成IMU、编码器、运动学和路径跟踪的主控制系统
 **************************************************************************************************/

#ifndef __NAVIGATION_CONTROLLER_H
#define __NAVIGATION_CONTROLLER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "tb6612_motor.h"
#include "encoder.h"
#include "imu660rc.h"
#include "mecanum_kinematics.h"
#include "path_tracking.h"
#include "pid_controller.h"
#include "wheel_bench.h"

/* 导航控制器状态 */
typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_MANUAL,       // 手动控制模式
    NAV_STATE_AUTO,         // 自动路径跟踪模式
    NAV_STATE_ERROR
} NavState_t;

typedef enum {
    MOTION_TEST_NONE = 0,
    MOTION_TEST_CCW,
    MOTION_TEST_CW,
    MOTION_TEST_FORWARD,
    MOTION_TEST_BACKWARD,
    MOTION_TEST_LEFT,
    MOTION_TEST_RIGHT
} MotionTestMode_t;

/* 导航控制器结构 */
typedef struct {
    /* 硬件接口 */
    I2C_HandleTypeDef *hi2c;

    /* 传感器数据 */
    IMU660RC_Data_t imu_data;
    IMU660RC_Attitude_t attitude;
    EncoderDelta_t encoder_delta;
    float wheel_rpm[4];     // Latest measured RPM, motor A/B/C/D; shared by PID and telemetry.
    float wheel_target_rpm[4]; // Slew-limited RPM commands
    uint8_t wheel_startup_active; // One-shot translation startup ramp only.

    /* 里程计 */
    Odometry_t odometry;

    /* 路径跟踪器 */
    PathTracker_t path_tracker;

    /* 控制参数 */
    NavState_t state;
    float control_period;   // 控制周期(秒)
    uint32_t last_update_ms;
    /* Cached diagnostics; never re-read encoder deltas for logging. */
    uint32_t diagnostic_seq, diagnostic_overruns, diagnostic_tx_errors;
    float diagnostic_sample_ms, diagnostic_max_sample_ms;
    uint32_t diagnostic_send_ms;
    WheelBenchConfig bench;
    uint32_t bench_start_ms, bench_heartbeat_ms;
    uint32_t bench_sample_ms;
    int bench_level;
    uint8_t bench_active;
    float diagnostic_requested_rpm[4];

    /* IMU融合权重 */
    float imu_fusion_weight;  // 0.0-1.0, IMU航向角融合权重
    uint8_t imu_read_errors;

    /* 四轮速度闭环测试 */
    PID_Controller_t wheel_pid[4];
    PID_Controller_t angle_pid;
    float angle_target_rad;
    float test_target_rpm;
    uint32_t test_start_ms;
    int16_t test_pwm[4];
    uint8_t test_first_sample;
    uint8_t rotation_test_active;
    float rotation_test_angle;
    float rotation_test_start_yaw;
    uint32_t rotation_test_start_ms;
    MotionTestMode_t motion_test_mode;
    uint8_t distance_test_active;
    float distance_start_x, distance_start_y;
    float distance_progress_m;
    float distance_target_m;
    uint8_t square_test_active;
    uint8_t square_test_paused;
    uint32_t square_pause_start_ms;
    uint8_t square_test_segment;
    float square_test_heading;
    uint8_t square2_active;
    uint8_t full_map_test; // Select the nine-step route in the shared sequence runner.
    uint8_t square2_step;
    float square2_turn_target;
    float square2_turn_progress;
    float square2_last_heading;
    float distance_direction_rad; // World direction captured at test start.
    MotionTestMode_t distance_test_mode; // Retained for the final status packet.
} NavController_t;

/* 函数声明 */

/* 导航控制器初始化 */
uint8_t NavController_Init(NavController_t *nav, I2C_HandleTypeDef *hi2c, float control_period);

/* 导航控制器更新 (在主循环中调用) */
void NavController_Update(NavController_t *nav);
void NavController_SendTelemetry(const NavController_t *nav);
void NavController_SendImuDebug(const NavController_t *nav);
void NavController_SendEncoderDebug(const NavController_t *nav);
void NavController_SendMotionDebug(NavController_t *nav);
void NavController_SendInitInfo(const NavController_t *nav);
void NavController_StartSpeedTest(NavController_t *nav, float target_rpm);
void NavController_StartRotationTest(NavController_t *nav);
void NavController_StartMotionTest(NavController_t *nav, MotionTestMode_t mode);
void NavController_StartForward1mTest(NavController_t *nav);
void NavController_StartDistanceTest(NavController_t *nav, MotionTestMode_t mode);
void NavController_StartSquareTest(NavController_t *nav);
void NavController_StartSquareTest2(NavController_t *nav);
void NavController_StartFullMapTest(NavController_t *nav);
void NavController_SendRotationDebug(const NavController_t *nav, float encoder_omega);

/* 设置手动控制速度 */
void NavController_SetManualVelocity(NavController_t *nav, float vx, float vy, float omega);

/* 加载路径并启动自动导航 */
void NavController_StartAutoNav(NavController_t *nav, Waypoint_t *waypoints, uint16_t count);

/* 停止导航 */
void NavController_Stop(NavController_t *nav);

/* 复位导航系统 */
void NavController_Reset(NavController_t *nav);

/* 获取当前位姿 */
Position_t NavController_GetPosition(const NavController_t *nav);

/* 获取当前状态 */
NavState_t NavController_GetState(const NavController_t *nav);

#endif /* __NAVIGATION_CONTROLLER_H */


