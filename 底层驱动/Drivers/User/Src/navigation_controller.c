/***************************************************************************************************
 * @file    navigation_controller.c
 * @brief   导航控制器实现
 **************************************************************************************************/

#include "navigation_controller.h"
#include "wifi_spi.h"
#include "pid_controller.h"
#include "pid_param_parser.h"
#include "wheel_output_control.h"
#include <math.h>
#include <string.h>
#include <stdio.h>


#define SPEED_PID_INTEGRAL_SEPARATION_RPM 25.0f
#define ANGLE_PID_INTEGRAL_SEPARATION_RAD 0.174532925f /* 10 degrees */
#define ANGLE_CORRECTION_MAX_RAD_S 0.5f
#define MOTION_TEST_SPEED_RPM 200.0f
#define ROTATION_TEST_MAX_RAD_S 1.0f
#define DISTANCE_TEST_SLOWDOWN_M 0.3f
#define ROTATION_TIMEOUT_MS 10000U
#define IMU_STARTUP_SETTLE_MS 3000U
#define IMU_STARTUP_CALIBRATION_SAMPLES 600U
#define IMU_STARTUP_SAMPLE_PERIOD_MS 5U
#define SPEED_TEST_DURATION_MS 5000U
#define DISTANCE_TEST_DISTANCE_M 1.0f
#define DISTANCE_TEST_TIMEOUT_MS 30000U
#define SQUARE_TEST_PAUSE_MS 1000U /* Pause between segments, without blocking. */
#define PARAM_FLASH_ADDR 0x081E0000UL
#define PARAM_MAGIC 0x50494431UL
typedef struct { uint32_t magic; float kp,ki,kd; uint32_t crc; } ParamStore_t;
static float param_kp=7.5f,param_ki=0.0f,param_kd=0.0f;
static float debug_angle_err=0.0f, debug_angle_omega=0.0f, debug_angle_correction=0.0f;
static uint8_t debug_test_active=0U;
static void clear_angle_debug(void){debug_angle_err=0.0f;debug_angle_omega=0.0f;debug_angle_correction=0.0f;}
static void apply_gains(NavController_t *n){ for(int i=0;i<4;i++) PID_SetGains(&n->wheel_pid[i],param_kp,param_ki,param_kd); }
static void send_param_reply(const char *r){ char m[180]; int l=snprintf(m,sizeof(m),"{\"type\":\"param_ack\",\"result\":\"%s\",\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f}\n",r,param_kp,param_ki,param_kd); if(l>0){WIFI_SPI_SendBuffer((const uint8_t*)m,l);WIFI_SPI_UdpSendNow();} }
static float normalize_angle(float angle);
static void start_square_segment(NavController_t *nav, uint8_t segment, float heading);
static void start_square2_step(NavController_t *nav, uint8_t step);
static void send_bench_status(const NavController_t *nav, const char *status)
{
    char msg[320];
    int n=snprintf(msg,sizeof(msg),
        "{\"type\":\"wheel_bench\",\"status\":\"%s\",\"kind\":%d,\"dir\":%d,\"wheel\":%d,\"value\":%d,\"step\":%d,\"hold_ms\":%d,\"duration_ms\":%d,\"timestamp_ms\":%lu}\n",
        status,nav->bench.kind,nav->bench.dir,nav->bench.wheel,nav->bench.value,
        nav->bench.step,nav->bench.hold_ms,nav->bench.duration_ms,(unsigned long)HAL_GetTick());
    if(n>0 && n<(int)sizeof(msg) && WIFI_SPI_IsReady()) {
        WIFI_SPI_SendBuffer((const uint8_t *)msg,(uint32_t)n);
        WIFI_SPI_UdpSendNow();
    }
}
static void finish_bench(NavController_t *nav, const char *reason)
{
    nav->bench_active=0U;
    NavController_Stop(nav); /* Stop the motors before any blocking status send. */
    send_bench_status(nav,reason);
}

static void send_square2_status(const NavController_t *nav, const char *status)
{
    char message[192];
    int n = snprintf(message, sizeof(message),
        "{\"type\":\"%s\",\"status\":\"%s\",\"step\":%u,\"distance_m\":%.3f,\"turn_deg\":%.2f}\n",
        nav->full_map_test ? "full_map_test" : "square_test2",
        status, (unsigned int)nav->square2_step, nav->distance_progress_m,
        nav->square2_turn_progress * 57.2957795f);
    if (n > 0 && n < (int)sizeof(message) && WIFI_SPI_IsReady()) {
        WIFI_SPI_SendBuffer((const uint8_t *)message, (uint32_t)n);
        WIFI_SPI_UdpSendNow();
    }
}

static void finish_square2_step(NavController_t *nav)
{
    const uint8_t step = nav->square2_step;
    nav->square2_active = 0U;
    nav->distance_test_active = 0U;
    NavController_Stop(nav);
    if (step < (nav->full_map_test ? 9U : 8U)) {
        nav->square2_active = 1U;
        nav->square_test_paused = 1U;
        nav->square_pause_start_ms = HAL_GetTick();
        send_square2_status(nav, "paused");
    } else {
        send_square2_status(nav, "reached");
    }
}

static void send_distance_status(const NavController_t *nav, const char *status)
{
    char message[256];
    int n = snprintf(message, sizeof(message),
        "{\"type\":\"distance_test\",\"status\":\"%s\",\"mode\":%u,\"distance_m\":%.4f,\"target_m\":%.2f,\"segment\":%u,\"elapsed_ms\":%lu}\n",
        status, (unsigned int)nav->distance_test_mode, nav->distance_progress_m,
        nav->distance_target_m, (unsigned int)nav->square_test_segment,
        (unsigned long)(HAL_GetTick() - nav->rotation_test_start_ms));
    if (n > 0 && n < (int)sizeof(message) && WIFI_SPI_IsReady()) {
        WIFI_SPI_SendBuffer((const uint8_t *)message, (uint32_t)n);
        WIFI_SPI_UdpSendNow();
    }
}

static void finish_distance_test(NavController_t *nav, const char *reason)
{
    const uint8_t was_square2 = nav->square2_active;
    if (was_square2) send_square2_status(nav, reason);
    nav->square2_active = 0U;
    nav->distance_test_active = 0U;
    nav->square_test_paused = 0U;
    NavController_Stop(nav);
    if (!was_square2) send_distance_status(nav, reason);
}

/* Convert chassis velocity to four wheel target RPMs.  The same wheel PID
   used by the speed-test path is used for all normal navigation modes. */
static void compute_wheel_target_rpm(const VelocityCmd_t *v, float rpm[4])
{
    const float arm = ROBOT_WHEEL_BASE / 2.0f + ROBOT_TRACK_WIDTH / 2.0f;
    const float k = 60.0f / (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS);
    const float w[4] = {
        v->vx - v->vy - v->omega * arm,
        v->vx + v->vy + v->omega * arm,
        v->vx + v->vy - v->omega * arm,
        v->vx - v->vy + v->omega * arm
    };
    for (int i = 0; i < 4; ++i) rpm[i] = w[i] * k;
}

static void stop_wheel_output(NavController_t *nav)
{
    memset(nav->diagnostic_requested_rpm, 0, sizeof(nav->diagnostic_requested_rpm));
    nav->wheel_startup_active = 0U;
    WheelOutput_Reset(nav->wheel_pid, nav->wheel_target_rpm, nav->test_pwm);
    Motor_StopAll();
}

static void apply_wheel_speed_loop(NavController_t *nav, const float target_rpm[4], float dt)
{
    memcpy(nav->diagnostic_requested_rpm, target_rpm, sizeof(nav->diagnostic_requested_rpm));
    WheelOutput_Update(nav->wheel_pid, target_rpm, nav->wheel_rpm,
                       nav->wheel_target_rpm, nav->test_pwm, dt, &nav->wheel_startup_active);
    Motor_SetAllSpeed(nav->test_pwm[0], nav->test_pwm[1], nav->test_pwm[2], nav->test_pwm[3]);
}
/* Motion tests are expressed as wheel RPM setpoints and use the same
   encoder feedback PID as normal navigation. */
static void apply_motion_test_rpm(NavController_t *nav, MotionTestMode_t mode, float dt)
{
    float r = MOTION_TEST_SPEED_RPM;
    if (nav->distance_test_active) {
        /* Extend braking to 30 cm for the higher cruise speed. */
        r *= (nav->distance_target_m - nav->distance_progress_m) / DISTANCE_TEST_SLOWDOWN_M;
        if (r > MOTION_TEST_SPEED_RPM) r = MOTION_TEST_SPEED_RPM;
        if (r < 30.0f) r = 30.0f;
    }
    float target[4] = {0};
    if (mode == MOTION_TEST_CW || mode == MOTION_TEST_CCW) {
        /* Position (angle) loop for rotation tests.  The existing project
           convention is CCW negative and CW positive. */
        const float yaw = nav->odometry.position.theta;
        const float err = nav->square2_active ?
            nav->square2_turn_target - nav->square2_turn_progress :
            normalize_angle(nav->angle_target_rad - yaw);
        float omega = PID_Compute(&nav->angle_pid, 0.0f, -err, dt);
        if (omega > ROTATION_TEST_MAX_RAD_S) omega = ROTATION_TEST_MAX_RAD_S;
        if (omega < -ROTATION_TEST_MAX_RAD_S) omega = -ROTATION_TEST_MAX_RAD_S;
        debug_angle_err = err;
        debug_angle_omega = omega;
        const float arm = ROBOT_WHEEL_BASE * 0.5f + ROBOT_TRACK_WIDTH * 0.5f;
        const float rpm_per_omega = arm * 60.0f /
                                    (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS);
        const float wheel_rpm = omega * rpm_per_omega;
        debug_angle_correction = wheel_rpm;
        target[0] = target[1] = wheel_rpm;
        target[2] = target[3] = -wheel_rpm;
        apply_wheel_speed_loop(nav, target, dt);
        return;
    }
    switch (mode) {
    case MOTION_TEST_FORWARD:  target[0]=target[1]=target[2]=target[3]= r; break;
    case MOTION_TEST_BACKWARD: target[0]=target[1]=target[2]=target[3]=-r; break;
    case MOTION_TEST_LEFT:     target[0]=target[2]=-r; target[1]=target[3]= r; break;
    case MOTION_TEST_RIGHT:    target[0]=target[2]= r; target[1]=target[3]=-r; break;
    default: return;
    }

    /* Translation tests lock the heading that was captured when the test
       started.  Convert the yaw PID correction into the same wheel-RPM
       units before the common wheel speed loops run.  Rotation tests keep
       their fixed wheel pattern and are stopped by the IMU angle limit. */
    if (mode == MOTION_TEST_FORWARD || mode == MOTION_TEST_BACKWARD ||
        mode == MOTION_TEST_LEFT || mode == MOTION_TEST_RIGHT) {
        const float yaw = nav->odometry.position.theta;
        const float err = normalize_angle(nav->angle_target_rad - yaw);
        float omega = PID_Compute(&nav->angle_pid, 0.0f, -err, dt);
        if (omega > ANGLE_CORRECTION_MAX_RAD_S) omega = ANGLE_CORRECTION_MAX_RAD_S;
        if (omega < -ANGLE_CORRECTION_MAX_RAD_S) omega = -ANGLE_CORRECTION_MAX_RAD_S;
        debug_angle_err = err;
        debug_angle_omega = omega;
        const float arm = ROBOT_WHEEL_BASE * 0.5f + ROBOT_TRACK_WIDTH * 0.5f;
        const float rpm_per_omega = arm * 60.0f /
                                    (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS);
        const float correction = omega * rpm_per_omega;
        debug_angle_correction = correction;
        /* A/B/C/D = LF/LR/RR/RF. Match the rotation-test wheel pattern. */
        target[0] += correction;
        target[1] += correction;
        target[2] -= correction;
        target[3] -= correction;
    }
    apply_wheel_speed_loop(nav, target, dt);
}

static VelocityCmd_t manual_vel_cmd = {0};
static float angle_error(float a){ return normalize_angle(a); }

static float normalize_angle(float angle)
{
    while (angle > 3.14159265f) angle -= 6.28318531f;
    while (angle < -3.14159265f) angle += 6.28318531f;
    return angle;
}

/**
 * @brief  导航控制器初始化
 */
uint8_t NavController_Init(NavController_t *nav, I2C_HandleTypeDef *hi2c, float control_period)
{
    nav->hi2c = hi2c;
    nav->bench_active=0U;
    memset(&nav->bench,0,sizeof(nav->bench));
    nav->bench_start_ms=nav->bench_heartbeat_ms=0U;
    nav->bench_sample_ms=0U;nav->bench_level=0;
    nav->control_period = (control_period >= 0.005f) ? control_period : 0.02f;
    nav->last_update_ms = HAL_GetTick();
    nav->state = NAV_STATE_IDLE;
    nav->imu_fusion_weight = 0.05f; // 编码器航向对陀螺仪预测的每周期校正比例
    nav->diagnostic_seq = nav->diagnostic_overruns = nav->diagnostic_tx_errors = 0U;
    nav->diagnostic_sample_ms = nav->diagnostic_max_sample_ms = 0.0f;
    nav->diagnostic_send_ms = 0U;
    memset(nav->diagnostic_requested_rpm, 0, sizeof(nav->diagnostic_requested_rpm));
    param_kp=7.5f; param_ki=0.0f; param_kd=0.0f;
    nav->imu_read_errors = 0U;
    memset(&nav->imu_data, 0, sizeof(nav->imu_data));
    memset(&nav->attitude, 0, sizeof(nav->attitude));
    memset(&nav->encoder_delta, 0, sizeof(nav->encoder_delta));
    memset(nav->wheel_rpm, 0, sizeof(nav->wheel_rpm));
    memset(nav->wheel_target_rpm, 0, sizeof(nav->wheel_target_rpm));
    nav->test_target_rpm = 0.0f;
    nav->distance_test_active = 0U;
    nav->distance_start_x = nav->distance_start_y = 0.0f;
    nav->distance_target_m = DISTANCE_TEST_DISTANCE_M;
    nav->square_test_active = 0U;
    nav->square_test_paused = 0U;
    nav->square_pause_start_ms = 0U;
    nav->square_test_segment = 0U;
    nav->square_test_heading = 0.0f;
    nav->square2_active = 0U;
    nav->square2_step = 0U;
    nav->full_map_test = 0U;
    nav->square2_turn_target = 0.0f;
    nav->square2_turn_progress = 0.0f;
    nav->square2_last_heading = 0.0f;
    nav->distance_progress_m = 0.0f;
    nav->distance_direction_rad = 0.0f;
    nav->distance_test_mode = MOTION_TEST_NONE;
    nav->rotation_test_active = 0U;
    nav->rotation_test_angle = 0.0f;
    nav->rotation_test_start_yaw = 0.0f;
    nav->motion_test_mode = MOTION_TEST_NONE;
    nav->angle_target_rad = 0.0f;
    PID_Init(&nav->angle_pid, 4.5f, 0.2f, 0.0f, 10.0f, 0.5f, -0.5f);
    /* Disable angle integral action while the heading error exceeds 10 deg. */
    PID_SetOptions(&nav->angle_pid, ANGLE_PID_INTEGRAL_SEPARATION_RAD, 0U);
    nav->test_start_ms = 0U;
    for (int i = 0; i < 4; ++i) {
        PID_Init(&nav->wheel_pid[i], 7.5f, 0.0f, 0.0f, 500.0f, 1000.0f, -1000.0f);
        PID_SetOptions(&nav->wheel_pid[i], SPEED_PID_INTEGRAL_SEPARATION_RPM, 1U);
        nav->test_pwm[i] = 0;
    }

    apply_gains(nav);
    /* 初始化硬件 */
    Motor_Init();
    stop_wheel_output(nav);
    Encoder_Init();

    /* 初始化IMU (尝试两个可能的地址) */
    if (IMU660RC_Init(hi2c, IMU660RC_ADDR_0) != 0)
    {
        if (IMU660RC_Init(hi2c, IMU660RC_ADDR_1) != 0)
        {
            if (IMU660RC_Init(hi2c, IMU660RC_ADDR_2) != 0)
            {
                return 1;  // IMU初始化失败
            }
        }
    }

    /* Keep the vehicle stationary: allow startup settling, then average
       roughly three seconds of gyro measurements before navigation starts. */
    HAL_Delay(IMU_STARTUP_SETTLE_MS);
    if (IMU660RC_CalibrateGyro(IMU_STARTUP_CALIBRATION_SAMPLES,
                              IMU_STARTUP_SAMPLE_PERIOD_MS) != 0U)
        return 1U;
    IMU660RC_ResetYaw();
    memset(&nav->attitude, 0, sizeof(nav->attitude));

    /* 初始化里程计 */
    Mecanum_OdomInit(&nav->odometry);

    /* 复位编码器 */
    Encoder_ResetAll();

    manual_vel_cmd.vx = 0.0f;
    manual_vel_cmd.vy = 0.0f;
    manual_vel_cmd.omega = 0.0f;

    /* Initialization and calibration time must not enter the first integral. */
    nav->last_update_ms = HAL_GetTick();
    nav->attitude.timestamp_ms = nav->last_update_ms;
    return 0;
}

/**
 * @brief  导航控制器更新
 */
void NavController_Update(NavController_t *nav)
{
    uint32_t now_ms = HAL_GetTick();
    float elapsed = (now_ms - nav->last_update_ms) / 1000.0f;

    /* 严格按设定周期运行，避免主循环速度影响 PID */
    if (elapsed < nav->control_period) return;
    nav->diagnostic_seq++;
    nav->diagnostic_sample_ms = elapsed * 1000.0f;
    if (nav->diagnostic_sample_ms > nav->diagnostic_max_sample_ms)
        nav->diagnostic_max_sample_ms = nav->diagnostic_sample_ms;
    if (elapsed > nav->control_period * 1.5f) nav->diagnostic_overruns++;
    float dt = elapsed;
    if (dt > 0.1f) dt = nav->control_period;

    nav->last_update_ms = now_ms;

    if(nav->bench_active) {
        /* Deadline check precedes sensor/transport work. Unsigned subtraction
         * remains valid across the HAL tick rollover. */
        const char *reason=WheelBench_StopReason(&nav->bench,now_ms,
            nav->bench_start_ms,nav->bench_heartbeat_ms,elapsed);
        if(reason) { finish_bench(nav,reason); return; }
    }

    /* Sample before blocking IMU/communication work. Use the full elapsed
       interval for RPM even when attitude/PID dt is limited after a stall. */
    nav->encoder_delta.encoder_1 = Encoder_ReadDelta(ENCODER_1);
    nav->encoder_delta.encoder_2 = Encoder_ReadDelta(ENCODER_2);
    nav->encoder_delta.encoder_3 = Encoder_ReadDelta(ENCODER_3);
    nav->encoder_delta.encoder_4 = Encoder_ReadDelta(ENCODER_4);
    const float rpm_scale = 60.0f / ((float)ENCODER_PPR * elapsed);
    nav->wheel_rpm[0] = nav->encoder_delta.encoder_1 * rpm_scale;
    nav->wheel_rpm[1] = nav->encoder_delta.encoder_2 * rpm_scale;
    nav->wheel_rpm[2] = nav->encoder_delta.encoder_3 * rpm_scale;
    nav->wheel_rpm[3] = nav->encoder_delta.encoder_4 * rpm_scale;

    /* A/B/C/D = LF/LR/RR/RF. Clockwise is positive: (A+B-C-D).
       Keep consistent with Mecanum_ForwardKinematics and rotation tests. */
    const float encoder_omega = (nav->encoder_delta.encoder_1 +
                                 nav->encoder_delta.encoder_2 -
                                 nav->encoder_delta.encoder_3 -
                                 nav->encoder_delta.encoder_4) *
                                (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS) /
                                ((float)ENCODER_PPR * elapsed * 4.0f *
                                 (ROBOT_WHEEL_BASE / 2.0f + ROBOT_TRACK_WIDTH / 2.0f));
    const float theta_before_encoder = nav->odometry.position.theta;
    const float theta_encoder = normalize_angle(theta_before_encoder + encoder_omega * elapsed);

    /* Motion-test output is handled before the normal command parser below.
       Give the emergency-stop command priority so it can always interrupt a
       running rotation or translation test. */
    if (nav->rotation_test_active || nav->square_test_paused ||
        (nav->motion_test_mode >= MOTION_TEST_FORWARD &&
         nav->motion_test_mode <= MOTION_TEST_RIGHT)) {
        uint8_t stop_rx[128];
        uint16_t stop_n = WIFI_SPI_Read(stop_rx, sizeof(stop_rx) - 1U);
        if (stop_n > 0U) {
            stop_rx[stop_n] = 0U;
            if (strstr((char *)stop_rx, "\"cmd\":\"stop\"") != NULL) {
                NavController_Stop(nav);
                return;
            }
            if (strstr((char *)stop_rx, "\"cmd\":\"reset\"") != NULL) {
                NavController_Reset(nav);
                return;
            }
        }
    }

    if (nav->rotation_test_active) {
        nav->rotation_test_angle += encoder_omega * elapsed;
        {
            /* Physical rotation pattern: left wheels same direction, right
               wheels opposite. A=LF, B=LR, C=RR, D=RF. */
            apply_motion_test_rpm(nav, nav->motion_test_mode, dt);
        }
    }

    if (nav->motion_test_mode >= MOTION_TEST_FORWARD &&
        nav->motion_test_mode <= MOTION_TEST_RIGHT) {
        if (HAL_GetTick() - nav->rotation_test_start_ms >=
            (nav->distance_test_active ? (uint32_t)(DISTANCE_TEST_TIMEOUT_MS * nav->distance_target_m) : 10000U)) {
            if (nav->distance_test_active) {
                finish_distance_test(nav, "timeout");
                return;
            }
            nav->motion_test_mode = MOTION_TEST_NONE;
            nav->state = NAV_STATE_IDLE;
            clear_angle_debug(); stop_wheel_output(nav);
            return;
        }
        apply_motion_test_rpm(nav, nav->motion_test_mode, dt);
        /* Continue sensor updates; the motion-test output is kept below. */
    }

    /* 读取IMU数据 */
    if (IMU660RC_ReadData(nav->hi2c, &nav->imu_data) == 0)
    {
        IMU660RC_UpdateAttitude(&nav->imu_data, &nav->attitude, dt);
        nav->imu_read_errors = 0U;
    }
    else if (nav->imu_read_errors < 255U)
    {
        nav->imu_read_errors++;
    }

    /* Fuse IMU gyro prediction with encoder heading before all angle checks. */
    Mecanum_ForwardKinematics(&nav->encoder_delta, dt, &nav->odometry);
    if (nav->imu_read_errors == 0U) {
        const float gyro_rate = nav->imu_data.gyro_z - IMU660RC_GetGyroZBias();
        const float theta_prediction = normalize_angle(theta_before_encoder + gyro_rate * dt);
        const float encoder_error = normalize_angle(theta_encoder - theta_prediction);
        nav->odometry.position.theta = normalize_angle(theta_prediction +
                                                       nav->imu_fusion_weight * encoder_error);
    }

    if (nav->square_test_paused) {
        /* Keep sensors and emergency commands live while all motors are off. */
        if (nav->imu_read_errors >= 25U) {
            finish_distance_test(nav, "imu_error");
            nav->state = NAV_STATE_ERROR;
            return;
        }
        if (HAL_GetTick() - nav->square_pause_start_ms >= SQUARE_TEST_PAUSE_MS) {
            nav->square_test_paused = 0U;
            if (nav->square2_active)
                start_square2_step(nav, nav->square2_step + 1U);
            else
                start_square_segment(nav, nav->square_test_segment + 1U,
                                     nav->square_test_heading);
        }
        return;
    }

    if (nav->distance_test_active) {
        const float heading = nav->distance_direction_rad;
        nav->distance_progress_m =
            (nav->odometry.position.x - nav->distance_start_x) * cosf(heading) +
            (nav->odometry.position.y - nav->distance_start_y) * sinf(heading);
        if (nav->imu_read_errors >= 25U) {
            finish_distance_test(nav, "imu_error");
            nav->state = NAV_STATE_ERROR;
            return;
        }
        if (nav->distance_progress_m >= nav->distance_target_m) {
            if (nav->square2_active) {
                finish_square2_step(nav);
                return;
            }
            if (nav->square_test_active && nav->square_test_segment < 4U) {
                nav->distance_test_active = 0U;
                NavController_Stop(nav);
                nav->square_test_active = 1U;
                nav->square_test_paused = 1U;
                nav->square_pause_start_ms = HAL_GetTick();
                send_distance_status(nav, "paused");
                return;
            }
            finish_distance_test(nav, "reached");
            return;
        }
    }

    /* Finish at 90 degrees without waiting for the rotation to settle. */
    if (nav->rotation_test_active) {
        const uint32_t check_ms = HAL_GetTick();
        if (nav->square2_active) {
            /* Unwrap successive fused samples so a clockwise 270-degree
               command cannot become a shortest-path counterclockwise turn. */
            nav->square2_turn_progress += normalize_angle(
                nav->odometry.position.theta - nav->square2_last_heading);
            nav->square2_last_heading = nav->odometry.position.theta;
            if (nav->imu_read_errors >= 25U) {
                finish_distance_test(nav, "imu_error");
                nav->state = NAV_STATE_ERROR;
            } else if (check_ms - nav->rotation_test_start_ms >=
                       (fabsf(nav->square2_turn_target) > 3.14159265f ? 30000U : ROTATION_TIMEOUT_MS)) {
                finish_distance_test(nav, "timeout");
            } else if ((nav->square2_turn_target > 0.0f &&
                        nav->square2_turn_progress >= nav->square2_turn_target) ||
                       (nav->square2_turn_target < 0.0f &&
                        nav->square2_turn_progress <= nav->square2_turn_target)) {
                finish_square2_step(nav);
            }
            return;
        }
        nav->rotation_test_angle = normalize_angle(
            nav->odometry.position.theta - nav->rotation_test_start_yaw);
        if (check_ms - nav->rotation_test_start_ms >= ROTATION_TIMEOUT_MS) {
            NavController_Stop(nav);
            return;
        }
        if (nav->imu_read_errors >= 25U) {
            NavController_Stop(nav);
            nav->state = NAV_STATE_ERROR;
            return;
        }
        if (fabsf(nav->rotation_test_angle) >= 1.5707963f) {
            NavController_Stop(nav);
            return;
        }
        return;
    }

    if (nav->motion_test_mode >= MOTION_TEST_FORWARD &&
        nav->motion_test_mode <= MOTION_TEST_RIGHT)
        return;
    if(nav->bench_active) {
        uint8_t rx[256];
        uint16_t n=WIFI_SPI_Read(rx,sizeof(rx)-1U);
        rx[n]=0;
        if(strstr((char *)rx,"\"cmd\":\"stop\"")) { NavController_Stop(nav); return; }
        if(strstr((char *)rx,"\"cmd\":\"reset\"")) { NavController_Reset(nav); return; }
        if(strstr((char *)rx,"\"type\":\"heartbeat\"")) nav->bench_heartbeat_ms=HAL_GetTick();
        else if(strstr((char *)rx,"\"cmd\"")) send_bench_status(nav,"busy");
        if(nav->imu_read_errors>=25U) { finish_bench(nav,"imu_error"); return; }
        uint32_t elapsed_ms=HAL_GetTick()-nav->bench_start_ms;
        if(elapsed_ms>=(uint32_t)nav->bench.duration_ms) { finish_bench(nav,"completed"); return; }
        int level=WheelBench_Level(&nav->bench,elapsed_ms);
        nav->bench_sample_ms=elapsed_ms;nav->bench_level=level;
        if(nav->bench.kind==1) {
            float target[4];
            for(int i=0;i<4;++i) target[i]=(float)(WheelBench_Sign(&nav->bench,i)*level);
            apply_wheel_speed_loop(nav,target,dt);
        } else {
            /* True open loop: no PID, no minimum-PWM floor, no RPM target. */
            for(int i=0;i<4;++i) nav->test_pwm[i]=(int16_t)(WheelBench_Sign(&nav->bench,i)*level);
            Motor_SetAllSpeed(nav->test_pwm[0],nav->test_pwm[1],nav->test_pwm[2],nav->test_pwm[3]);
        }
        return;
    }
    if (nav->imu_read_errors >= 25U && nav->test_target_rpm == 0.0f && nav->imu_read_errors < 255U)
    {
        nav->state = NAV_STATE_ERROR;
        clear_angle_debug(); stop_wheel_output(nav);
        return;
    }

    /* 读取上位机通过 UDP 下发的控制命令。 */
    {
        uint8_t rx[256];
        uint16_t n = WIFI_SPI_Read(rx, sizeof(rx) - 1U);
        if (n > 0U)
        {
            float vx, vy, omega;
            rx[n] = 0U;
            float kp,ki,kd;
            if (strstr((char *)rx,"\"type\":\"param_get\"") != NULL) send_param_reply("ok");
            else if (strstr((char *)rx,"\"param_set\"") != NULL) {
                const char *error = PID_ParseParams((const char *)rx, &kp, &ki, &kd);
                if (error != NULL) send_param_reply(error);
                else {
                    (void)kp; (void)ki; (void)kd;
                    send_param_reply("locked");
                }
            }
            else if (strstr((char *)rx,"\"type\":\"param_reset\"") != NULL) send_param_reply("locked");
            else if (strstr((char *)rx, "\"cmd\":\"stop\"") != NULL)
                NavController_Stop(nav);
            else if (strstr((char *)rx, "\"cmd\":\"wheel_bench\"") != NULL) {
                WheelBenchConfig c;
                if(!WheelBench_Parse((const char *)rx,&c)) { send_bench_status(nav,"invalid"); return; }
                NavController_Stop(nav);
                nav->bench=c;
                nav->bench_active=1U;
                nav->bench_start_ms=nav->bench_heartbeat_ms=HAL_GetTick();
                nav->bench_sample_ms=0U;nav->bench_level=0;
                nav->wheel_startup_active=1U;
                nav->state=NAV_STATE_MANUAL;
                send_bench_status(nav,"started");
                return;
            }
            else if (strstr((char *)rx, "\"cmd\":\"speed_test\"") != NULL)
                NavController_StartSpeedTest(nav, 100.0f);
            else if (strstr((char *)rx, "\"cmd\":\"full_map_test\"") != NULL) {
                NavController_StartFullMapTest(nav);
                return;
            }
            else if (strstr((char *)rx, "\"cmd\":\"square_test2\"") != NULL) {
                NavController_StartSquareTest2(nav);
                return;
            }
            else if (strstr((char *)rx, "\"cmd\":\"square_test\"") != NULL) {
                NavController_StartSquareTest(nav);
                return;
            }
            else if (strstr((char *)rx, "\"cmd\":\"forward_1m\"") != NULL) {
                NavController_StartForward1mTest(nav);
                return;
            }
            else if (strstr((char *)rx, "\"cmd\":\"backward_1m\"") != NULL) {
                NavController_StartDistanceTest(nav, MOTION_TEST_BACKWARD);
                return;
            }
            else if (strstr((char *)rx, "\"cmd\":\"left_1m\"") != NULL) {
                NavController_StartDistanceTest(nav, MOTION_TEST_LEFT);
                return;
            }
            else if (strstr((char *)rx, "\"cmd\":\"right_1m\"") != NULL) {
                NavController_StartDistanceTest(nav, MOTION_TEST_RIGHT);
                return;
            }
            else if (strstr((char *)rx, "\"cmd\":\"rotate_test\"") != NULL)
                NavController_StartMotionTest(nav, MOTION_TEST_CCW);
            else if (strstr((char *)rx, "\"cmd\":\"motion_test\"") != NULL) {
                int mode = 0;
                if (sscanf((char *)rx, "{\"type\":\"cmd\",\"cmd\":\"motion_test\",\"mode\":%d", &mode) == 1 &&
                    mode >= MOTION_TEST_CCW && mode <= MOTION_TEST_RIGHT)
                    NavController_StartMotionTest(nav, (MotionTestMode_t)mode);
            }
            else if (strstr((char *)rx, "\"cmd\":\"reset\"") != NULL)
                NavController_Reset(nav);
            else if (strstr((char *)rx, "\"cmd\":\"manual\"") != NULL)
            {
                vx = vy = omega = 0.0f;
                if (sscanf((char *)rx, "{\"type\":\"cmd\",\"cmd\":\"manual\",\"vx\":%f,\"vy\":%f,\"omega\":%f", &vx, &vy, &omega) == 3)
                    NavController_SetManualVelocity(nav, vx, vy, omega);
            }
        }
    }

    /* 运行时控制 */
    if (nav->test_target_rpm != 0.0f) {
        if (HAL_GetTick() - nav->test_start_ms >= 2000U) {
            NavController_Stop(nav);
        } else {
            const float r = nav->test_target_rpm;
            const float target[4] = {r, r, r, r};
            apply_wheel_speed_loop(nav, target, dt);
        }
        return;
    }
    if (nav->rotation_test_active) return;

    /* 根据状态执行控制 */
    VelocityCmd_t vel_cmd = {0};

    switch (nav->state)
    {
        case NAV_STATE_MANUAL:
            vel_cmd = manual_vel_cmd;
            break;

        case NAV_STATE_AUTO:
            PathTracker_Update(&nav->path_tracker, &nav->odometry, dt);
            vel_cmd = PathTracker_GetVelocityCmd(&nav->path_tracker);

            /* 检查是否到达终点 */
            if (PathTracker_GetState(&nav->path_tracker) == PATH_REACHED)
            {
                nav->state = NAV_STATE_IDLE;
                clear_angle_debug(); stop_wheel_output(nav);
                return;
            }
            break;

        case NAV_STATE_IDLE:
        default:
            stop_wheel_output(nav);
            return;
    }

    /* All normal modes use the same four wheel speed PID loop as the speed
       test.  The kinematic output is a wheel RPM setpoint, while encoder
       measured RPM is the feedback already updated above. */
    {
        float target_rpm[4];
        compute_wheel_target_rpm(&vel_cmd, target_rpm);
        apply_wheel_speed_loop(nav, target_rpm, dt);
    }

    /* Heading hold angle loop for translation/manual/auto commands. */
    if (!nav->rotation_test_active && (nav->state == NAV_STATE_MANUAL || nav->state == NAV_STATE_AUTO)) {
        float err = angle_error(nav->angle_target_rad - nav->odometry.position.theta);
        vel_cmd.omega += PID_Compute(&nav->angle_pid, 0.0f, -err, dt);
    }
}

void NavController_SendTelemetry(const NavController_t *nav)
{
    if (nav->square2_active) send_square2_status(nav, nav->square_test_paused ? "paused" : "running");
    else if (nav->distance_test_active) send_distance_status(nav, "running");
    else if (nav->square_test_paused) send_distance_status(nav, "paused");
    static char message[1024]; /* Main-loop only; keep large JSON off the 4 KB stack. */
    Position_t p;
    int length;
    if (!WIFI_SPI_IsReady()) return;
    if (!nav->rotation_test_active &&
        (nav->motion_test_mode < MOTION_TEST_CCW || nav->motion_test_mode > MOTION_TEST_RIGHT))
        debug_test_active = 0U;
    p = nav->odometry.position;
    length = snprintf(message, sizeof(message),
        "{\"type\":\"telemetry\",\"debug_test_active\":%u,\"timestamp_ms\":%lu,\"x_mm\":%.1f,\"y_mm\":%.1f,\"yaw_deg\":%.2f,\"yaw_target_deg\":%.2f,\"angle_err_deg\":%.2f,\"angle_omega\":%.4f,\"angle_correction\":%.2f,\"state\":\"%s\",\"fl_rpm\":%.1f,\"fr_rpm\":%.1f,\"rl_rpm\":%.1f,\"rr_rpm\":%.1f,\"fl_rpm_target\":%.1f,\"fr_rpm_target\":%.1f,\"rl_rpm_target\":%.1f,\"rr_rpm_target\":%.1f,\"fl_pwm\":%d,\"fr_pwm\":%d,\"rl_pwm\":%d,\"rr_pwm\":%d,\"e1_total\":%ld,\"e2_total\":%ld,\"e3_total\":%ld,\"e4_total\":%ld}\n",
        debug_test_active, (unsigned long)HAL_GetTick(), p.x * 1000.0f, p.y * 1000.0f,
        p.theta * 57.2957795f, nav->angle_target_rad * 57.2957795f, debug_angle_err * 57.2957795f, debug_angle_omega, debug_angle_correction,
        nav->state == NAV_STATE_AUTO ? "AUTO" : (nav->state == NAV_STATE_MANUAL ? "MANUAL" : (nav->state == NAV_STATE_ERROR ? "ERROR" : "IDLE")),
        nav->wheel_rpm[0], nav->wheel_rpm[3], nav->wheel_rpm[1], nav->wheel_rpm[2], nav->wheel_target_rpm[0], nav->wheel_target_rpm[3], nav->wheel_target_rpm[1], nav->wheel_target_rpm[2], nav->test_pwm[0], nav->test_pwm[3], nav->test_pwm[1], nav->test_pwm[2],
        (long)Encoder_ReadTotal(ENCODER_1), (long)Encoder_ReadTotal(ENCODER_2), (long)Encoder_ReadTotal(ENCODER_3), (long)Encoder_ReadTotal(ENCODER_4));
    if (length > 0 && (uint32_t)length < sizeof(message))
    {
        WIFI_SPI_SendBuffer((const uint8_t *)message, (uint32_t)length);
        WIFI_SPI_UdpSendNow();
    }
}

/* Read cached counts only: extra Encoder_ReadDelta calls would consume
   pulses needed by the normal control/odometry update. Deltas are from the
   latest control update, not the entire debug reporting interval. */
void NavController_SendEncoderDebug(const NavController_t *nav)
{
    char message[384];
    int length;
    if (!WIFI_SPI_IsReady()) return;
    length = snprintf(message, sizeof(message),
        "{\"type\":\"encoder_debug\",\"timestamp_ms\":%lu,\"e1_delta\":%d,\"e2_delta\":%d,\"e3_delta\":%d,\"e4_delta\":%d,\"e1_total\":%ld,\"e2_total\":%ld,\"e3_total\":%ld,\"e4_total\":%ld}\n",
        (unsigned long)HAL_GetTick(),
        (int)nav->encoder_delta.encoder_1, (int)nav->encoder_delta.encoder_2,
        (int)nav->encoder_delta.encoder_3, (int)nav->encoder_delta.encoder_4,
        (long)Encoder_ReadTotal(ENCODER_1), (long)Encoder_ReadTotal(ENCODER_2),
        (long)Encoder_ReadTotal(ENCODER_3), (long)Encoder_ReadTotal(ENCODER_4));
    if (length > 0 && (uint32_t)length < sizeof(message))
    {
        WIFI_SPI_SendBuffer((const uint8_t *)message, (uint32_t)length);
        WIFI_SPI_UdpSendNow();
    }
}

/* Arrays use A/B/C/D = LF/LR/RR/RF. Values share the latest control sample.
 * Body motion below is encoder-derived, not an independent slip measurement. */
void NavController_SendMotionDebug(NavController_t *nav)
{
    static char message[1536]; /* Main-loop only, not reentrant. */
    if (!WIFI_SPI_IsReady()) return;
    const float *r = nav->wheel_rpm;
    const float k = 2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS / 60.0f;
    const float arm = (ROBOT_WHEEL_BASE + ROBOT_TRACK_WIDTH) * 0.5f;
    const uint32_t start = HAL_GetTick();
    int n = snprintf(message, sizeof(message),
        "{\"type\":\"motion_debug\",\"v\":1,\"seq\":%lu,\"timestamp_ms\":%lu,"
        "\"sample_ms\":%.1f,\"max_sample_ms\":%.1f,\"overruns\":%lu,\"send_ms\":%lu,\"tx_errors\":%lu,"
        "\"bench_kind\":%d,\"bench_dir\":%d,\"bench_wheel\":%d,\"bench_ms\":%lu,\"bench_level\":%d,"
        "\"state\":%u,\"mode\":%u,\"requested_rpm\":[%.2f,%.2f,%.2f,%.2f],"
        "\"target_rpm\":[%.2f,%.2f,%.2f,%.2f],\"rpm\":[%.2f,%.2f,%.2f,%.2f],"
        "\"pwm\":[%d,%d,%d,%d],\"delta\":[%d,%d,%d,%d],\"total\":[%ld,%ld,%ld,%ld],"
        "\"enc_body\":[%.5f,%.5f,%.5f],\"gyro_rad_s\":%.5f,\"gyro_bias\":%.6f,\"imu_errors\":%u,"
        "\"yaw_deg\":%.3f,\"target_yaw_deg\":%.3f,\"angle_omega\":%.4f,"
        "\"pid\":[%.4f,%.4f,%.4f],\"ppr\":%u,\"radius_m\":%.3f,\"arm_m\":%.3f}\n",
        (unsigned long)nav->diagnostic_seq, (unsigned long)nav->last_update_ms,
        nav->diagnostic_sample_ms, nav->diagnostic_max_sample_ms,
        (unsigned long)nav->diagnostic_overruns, (unsigned long)nav->diagnostic_send_ms,
        (unsigned long)nav->diagnostic_tx_errors,
        nav->bench_active ? nav->bench.kind : 0,nav->bench.dir,nav->bench.wheel,
        (unsigned long)(nav->bench_active ? nav->bench_sample_ms : 0U),
        nav->bench_active ? nav->bench_level : 0,
        (unsigned)nav->state, (unsigned)nav->motion_test_mode,
        nav->diagnostic_requested_rpm[0],nav->diagnostic_requested_rpm[1],nav->diagnostic_requested_rpm[2],nav->diagnostic_requested_rpm[3],
        nav->wheel_target_rpm[0],nav->wheel_target_rpm[1],nav->wheel_target_rpm[2],nav->wheel_target_rpm[3],
        r[0],r[1],r[2],r[3],nav->test_pwm[0],nav->test_pwm[1],nav->test_pwm[2],nav->test_pwm[3],
        nav->encoder_delta.encoder_1,nav->encoder_delta.encoder_2,nav->encoder_delta.encoder_3,nav->encoder_delta.encoder_4,
        (long)Encoder_ReadTotal(ENCODER_1),(long)Encoder_ReadTotal(ENCODER_2),
        (long)Encoder_ReadTotal(ENCODER_3),(long)Encoder_ReadTotal(ENCODER_4),
        k*(r[0]+r[1]+r[2]+r[3])*0.25f, k*(-r[0]+r[1]-r[2]+r[3])*0.25f,
        k*(r[0]+r[1]-r[2]-r[3])/(4.0f*arm),
        nav->imu_data.gyro_z-IMU660RC_GetGyroZBias(),IMU660RC_GetGyroZBias(),(unsigned)nav->imu_read_errors,
        nav->odometry.position.theta*57.2957795f,nav->angle_target_rad*57.2957795f,debug_angle_omega,
        nav->wheel_pid[0].kp,nav->wheel_pid[0].ki,nav->wheel_pid[0].kd,
        (unsigned)ENCODER_PPR,ROBOT_WHEEL_RADIUS,arm);
    if (n <= 0 || n >= (int)sizeof(message)) nav->diagnostic_tx_errors++;
    else {
        const uint32_t unsent = WIFI_SPI_SendBuffer((const uint8_t *)message, (uint32_t)n);
        /* Flush even a partial packet so the next JSON does not concatenate. */
        const uint8_t failed = WIFI_SPI_UdpSendNow();
        if (unsent || failed) nav->diagnostic_tx_errors++;
    }
    nav->diagnostic_send_ms = HAL_GetTick() - start;
}

void NavController_StartRotationTest(NavController_t *nav)
{
    NavController_StartMotionTest(nav, MOTION_TEST_CCW);
}

void NavController_StartMotionTest(NavController_t *nav, MotionTestMode_t mode)
{
    NavController_Stop(nav);
    nav->wheel_startup_active = (mode >= MOTION_TEST_FORWARD && mode <= MOTION_TEST_RIGHT);
    Encoder_ResetAll();
    nav->motion_test_mode = mode;
    debug_test_active = (mode >= MOTION_TEST_CCW && mode <= MOTION_TEST_RIGHT) ? 1U : 0U;
    nav->rotation_test_active = (mode == MOTION_TEST_CCW || mode == MOTION_TEST_CW) ? 1U : 0U;
    nav->angle_pid.output_max = nav->rotation_test_active ?
        ROTATION_TEST_MAX_RAD_S : ANGLE_CORRECTION_MAX_RAD_S;
    nav->angle_pid.output_min = -nav->angle_pid.output_max;
    nav->rotation_test_angle = 0.0f;
    /* Use the same fused heading as angle feedback and the stop condition. */
    nav->rotation_test_start_yaw = nav->odometry.position.theta;
    nav->angle_target_rad = normalize_angle(nav->rotation_test_start_yaw +
        ((mode == MOTION_TEST_CCW) ? -1.5707963f :
         (mode == MOTION_TEST_CW) ? 1.5707963f : 0.0f));
    nav->rotation_test_start_ms = HAL_GetTick();
    nav->last_update_ms = nav->rotation_test_start_ms;
    nav->state = NAV_STATE_IDLE;
    for (int i = 0; i < 4; ++i) {
        PID_Reset(&nav->wheel_pid[i]);
        nav->test_pwm[i] = 0;
        nav->wheel_target_rpm[i] = 0.0f;
    }
    PID_Reset(&nav->angle_pid);
}

void NavController_StartForward1mTest(NavController_t *nav)
{
    NavController_StartDistanceTest(nav, MOTION_TEST_FORWARD);
}

void NavController_StartDistanceTest(NavController_t *nav, MotionTestMode_t mode)
{
    if (mode < MOTION_TEST_FORWARD || mode > MOTION_TEST_RIGHT) return;
    NavController_StartMotionTest(nav, mode);
    nav->distance_test_mode = mode;
    nav->distance_target_m = DISTANCE_TEST_DISTANCE_M;
    nav->square_test_segment = 0U;
    /* Project onto the requested travel axis, not the heading-hold axis.
       Existing odometry maps the left-test wheel pattern to positive body Y. */
    float offset = 0.0f;
    if (mode == MOTION_TEST_BACKWARD) offset = 3.14159265f;
    else if (mode == MOTION_TEST_LEFT) offset = 1.57079633f;
    else if (mode == MOTION_TEST_RIGHT) offset = -1.57079633f;
    nav->distance_direction_rad = normalize_angle(-nav->rotation_test_start_yaw + offset);
    nav->distance_start_x = nav->odometry.position.x;
    nav->distance_start_y = nav->odometry.position.y;
    nav->distance_progress_m = 0.0f;
    nav->distance_test_active = 1U;
    send_distance_status(nav, "started");
}

static void start_square_segment(NavController_t *nav, uint8_t segment, float heading)
{
    static const MotionTestMode_t modes[4] = {
        MOTION_TEST_FORWARD, MOTION_TEST_LEFT, MOTION_TEST_BACKWARD, MOTION_TEST_RIGHT
    };
    static const float distances[4] = {1.55f, 3.0f, 1.55f, 3.0f};
    static const float offsets[4] = {0.0f, 1.57079633f, 3.14159265f, -1.57079633f};
    NavController_StartMotionTest(nav, modes[segment - 1U]);
    nav->square_test_active = 1U;
    nav->square_test_segment = segment;
    nav->square_test_heading = heading;
    nav->angle_target_rad = heading;
    nav->distance_test_mode = modes[segment - 1U];
    nav->distance_target_m = distances[segment - 1U];
    nav->distance_direction_rad = normalize_angle(-heading + offsets[segment - 1U]);
    nav->distance_start_x = nav->odometry.position.x;
    nav->distance_start_y = nav->odometry.position.y;
    nav->distance_progress_m = 0.0f;
    nav->distance_test_active = 1U;
    send_distance_status(nav, "started");
}

void NavController_StartSquareTest(NavController_t *nav)
{
    start_square_segment(nav, 1U, nav->odometry.position.theta);
}

static void start_square2_step(NavController_t *nav, uint8_t step)
{
    static const MotionTestMode_t full_modes[2] = {
        MOTION_TEST_FORWARD, 
				MOTION_TEST_LEFT, 
				//MOTION_TEST_FORWARD,
        //MOTION_TEST_CW,
				//MOTION_TEST_FORWARD, 
				//MOTION_TEST_CW,
        //MOTION_TEST_FORWARD, 
				//MOTION_TEST_RIGHT, 
				//MOTION_TEST_FORWARD
    };
    static const float full_distances[9] = {0.55f, 1.25f, /*3.6f, 0.0f, 1.58f, 0.0f, 3.6f, 0.35f, 0.5f*/};
    /* The local full-map route may contain only a subset of the nine steps. */
    if (nav->full_map_test && step > sizeof(full_modes) / sizeof(full_modes[0])) {
        nav->square2_active = 0U;
        NavController_Stop(nav);
        send_square2_status(nav, "reached");
        return;
    }
    if (step == 0U || step > (nav->full_map_test ? 9U : 8U)) {
        NavController_Stop(nav);
        return;
    }
    const uint8_t is_turn = nav->full_map_test ? (step == 4U || step == 6U) : (step % 2U == 0U);
    const MotionTestMode_t mode = nav->full_map_test ? full_modes[step - 1U] : !is_turn ? MOTION_TEST_FORWARD :
        (step == 8U ? MOTION_TEST_CW : MOTION_TEST_CCW);
    nav->square2_active = 0U;
    NavController_StartMotionTest(nav, mode);
    nav->square2_active = 1U;
    nav->square2_step = step;
    nav->square_test_segment = 0U;
    nav->square2_turn_progress = 0.0f;
    nav->square2_last_heading = nav->odometry.position.theta;
    nav->distance_progress_m = 0.0f;
    if (is_turn) {
        nav->square2_turn_target = nav->full_map_test ? 1.57079633f :
            step == 8U ? 4.71238898f : -1.57079633f;
        nav->angle_target_rad = normalize_angle(nav->square2_last_heading + nav->square2_turn_target);
    } else {
        nav->square2_turn_target = 0.0f;
        const float offset = mode == MOTION_TEST_LEFT ? 1.57079633f :
                             mode == MOTION_TEST_RIGHT ? -1.57079633f : 0.0f;
        nav->distance_test_mode = mode;
        nav->distance_target_m = nav->full_map_test ? full_distances[step - 1U] :
            (step == 1U || step == 5U) ? 1.55f : 3.0f;
        nav->distance_direction_rad = normalize_angle(-nav->odometry.position.theta + offset);
        nav->distance_start_x = nav->odometry.position.x;
        nav->distance_start_y = nav->odometry.position.y;
        nav->distance_test_active = 1U;
    }
    send_square2_status(nav, "started");
}

void NavController_StartSquareTest2(NavController_t *nav)
{
    NavController_Stop(nav);
    nav->full_map_test = 0U;
    start_square2_step(nav, 1U);
}

void NavController_StartFullMapTest(NavController_t *nav)
{
    NavController_Stop(nav);
    nav->full_map_test = 1U;
    start_square2_step(nav, 1U);
}

void NavController_SendRotationDebug(const NavController_t *nav, float encoder_omega)
{
    char message[384];
    const int n = snprintf(message, sizeof(message),
        "{\"type\":\"rotation_debug\",\"encoder_omega\":%.6f,\"encoder_angle_deg\":%.3f,\"e1_total\":%ld,\"e2_total\":%ld,\"e3_total\":%ld,\"e4_total\":%ld}\n",
        encoder_omega, nav->rotation_test_angle * 57.2957795f,
        (long)Encoder_ReadTotal(ENCODER_1), (long)Encoder_ReadTotal(ENCODER_2),
        (long)Encoder_ReadTotal(ENCODER_3), (long)Encoder_ReadTotal(ENCODER_4));
    if (n > 0 && n < (int)sizeof(message) && WIFI_SPI_IsReady()) {
        WIFI_SPI_SendBuffer((const uint8_t *)message, (uint32_t)n);
        WIFI_SPI_UdpSendNow();
    }
}

void NavController_SendInitInfo(const NavController_t *nav)
{
    char message[256];
    int length = snprintf(message, sizeof(message), "{\"type\":\"init_info\",\"timestamp_ms\":%lu,\"gyro_range_dps\":2000,\"gyro_sensitivity_deg_s_lsb\":0.070000,\"encoder_ppr\":%lu,\"control_period_ms\":%.1f,\"startup_calibration\":true,\"pid_kp\":%.6f,\"pid_ki\":%.6f,\"pid_kd\":%.6f}\n", (unsigned long)HAL_GetTick(), (unsigned long)ENCODER_PPR, nav->control_period * 1000.0f, param_kp, param_ki, param_kd);
    if (length > 0 && (uint32_t)length < sizeof(message)) { WIFI_SPI_SendBuffer((const uint8_t *)message, (uint32_t)length); WIFI_SPI_UdpSendNow(); }
}

void NavController_SendImuDebug(const NavController_t *nav)
{
    char m[280]; IMU660RC_Data_t d; int n;
    if (!WIFI_SPI_IsReady() || IMU660RC_ReadData(nav->hi2c, &d) != 0U) return;
    n=snprintf(m,sizeof(m),"{\"type\":\"imu_debug\",\"timestamp_ms\":%lu,\"gyro_z_raw\":%d,\"gyro_z_rad_s\":%.7f,\"gyro_z_bias_rad_s\":%.7f,\"gyro_z_corrected_rad_s\":%.7f,\"imu_yaw_deg\":%.3f,\"temperature_c\":%.2f}\n",(unsigned long)HAL_GetTick(),d.gyro_z_raw,d.gyro_z,IMU660RC_GetGyroZBias(),d.gyro_z-IMU660RC_GetGyroZBias(),IMU660RC_GetYaw(),d.temperature);
    if(n>0 && n<(int)sizeof(m)){WIFI_SPI_SendBuffer((const uint8_t*)m,(uint32_t)n);WIFI_SPI_UdpSendNow();}
}

/**
 * @brief  设置手动控制速度
 */
void NavController_SetManualVelocity(NavController_t *nav, float vx, float vy, float omega)
{
    VelocityCmd_t requested;
    requested.vx = vx;
    requested.vy = vy;
    requested.omega = omega;
    Mecanum_LimitVelocity(&requested, 0.3f, 0.3f, 1.0f);
    if (fabsf(omega) >= 0.001f || (vx == 0.0f && vy == 0.0f)) {
        nav->wheel_startup_active = 0U;
    } else if (nav->state != NAV_STATE_MANUAL ||
               (manual_vel_cmd.vx == 0.0f && manual_vel_cmd.vy == 0.0f &&
                manual_vel_cmd.omega == 0.0f)) {
        nav->wheel_startup_active = 1U;
    }
    nav->state = NAV_STATE_MANUAL;
    if (fabsf(omega) < 0.001f) nav->angle_target_rad = nav->odometry.position.theta;
    else nav->angle_target_rad = normalize_angle(nav->odometry.position.theta + omega * 0.5f);
    PID_Reset(&nav->angle_pid);
    manual_vel_cmd = requested;
}

/**
 * @brief  启动自动导航
 */
void NavController_StartAutoNav(NavController_t *nav, Waypoint_t *waypoints, uint16_t count)
{
    nav->wheel_startup_active = 0U;
    PathTracker_Init(&nav->path_tracker, waypoints, count);
    PathTracker_Start(&nav->path_tracker);
    nav->state = NAV_STATE_AUTO;
}

/**
 * @brief  停止导航
 */
void NavController_Stop(NavController_t *nav)
{
    const uint8_t was_bench=nav->bench_active;
    nav->bench_active=0U;
    const uint8_t was_distance_test = nav->distance_test_active || nav->square_test_paused;
    const uint8_t was_square2 = nav->square2_active;
    nav->square2_active = 0U;
    nav->square_test_active = 0U;
    nav->square_test_paused = 0U;
    nav->square_pause_start_ms = 0U;
    nav->distance_test_active = 0U;
    nav->test_target_rpm = 0.0f;
    memset(nav->wheel_target_rpm, 0, sizeof(nav->wheel_target_rpm));
    memset(nav->test_pwm, 0, sizeof(nav->test_pwm));
    nav->state = NAV_STATE_IDLE;
    nav->rotation_test_active = 0U;
    nav->motion_test_mode = MOTION_TEST_NONE;
    nav->angle_pid.output_max = ANGLE_CORRECTION_MAX_RAD_S;
    nav->angle_pid.output_min = -ANGLE_CORRECTION_MAX_RAD_S;
    debug_test_active = 0U;
    stop_wheel_output(nav);
    debug_angle_err = 0.0f;
    debug_angle_omega = 0.0f;
    debug_angle_correction = 0.0f;
    PathTracker_Stop(&nav->path_tracker);
    if(was_bench) send_bench_status(nav,"stopped");
    if (was_square2) send_square2_status(nav, "stopped");
    else if (was_distance_test) send_distance_status(nav, "stopped");
}

/**
 * @brief  复位导航系统
 */
void NavController_Reset(NavController_t *nav)
{
    if (nav->distance_test_active || nav->square_test_paused || nav->square2_active) finish_distance_test(nav, "reset");
    NavController_Stop(nav);
    nav->state = NAV_STATE_IDLE;
    Mecanum_OdomReset(&nav->odometry);
    Encoder_ResetAll();
    IMU660RC_ResetYaw();
    memset(&nav->attitude, 0, sizeof(nav->attitude));
    memset(&nav->imu_data, 0, sizeof(nav->imu_data));
    memset(&nav->encoder_delta, 0, sizeof(nav->encoder_delta));
    nav->test_target_rpm = 0.0f;
    nav->rotation_test_active = 0U;
    nav->motion_test_mode = MOTION_TEST_NONE;
    debug_test_active = 0U;
    nav->test_start_ms = 0U;
    for (int i = 0; i < 4; ++i) {
        /* Reset controller state while preserving the active wheel PID gains. */
        PID_Init(&nav->wheel_pid[i], 7.5f, 0.0f, 0.0f,
                 500.0f, 1000.0f, -1000.0f);
        PID_SetOptions(&nav->wheel_pid[i], SPEED_PID_INTEGRAL_SEPARATION_RPM, 1U);
        nav->wheel_rpm[i] = 0.0f;
        nav->test_pwm[i] = 0;
    }
    nav->imu_read_errors = 0U;
    nav->last_update_ms = HAL_GetTick();
    manual_vel_cmd.vx = 0.0f;
    manual_vel_cmd.vy = 0.0f;
    manual_vel_cmd.omega = 0.0f;
    PathTracker_Reset(&nav->path_tracker);
    stop_wheel_output(nav);
    debug_angle_err = 0.0f;
    debug_angle_omega = 0.0f;
    debug_angle_correction = 0.0f;
}

/**
 * @brief  获取当前位姿
 */
Position_t NavController_GetPosition(const NavController_t *nav)
{
    return nav->odometry.position;
}

/**
 * @brief  获取当前状态
 */
NavState_t NavController_GetState(const NavController_t *nav)
{
    return nav->state;
}

void NavController_StartSpeedTest(NavController_t *nav, float target_rpm) { NavController_Stop(nav); Encoder_ResetAll(); memset(&nav->encoder_delta, 0, sizeof(nav->encoder_delta)); memset(nav->wheel_rpm, 0, sizeof(nav->wheel_rpm)); nav->test_first_sample=1U; nav->test_target_rpm=target_rpm; nav->test_start_ms=HAL_GetTick(); nav->last_update_ms=nav->test_start_ms; for(int i=0;i<4;i++){PID_Reset(&nav->wheel_pid[i]); nav->test_pwm[i]=0;} }









