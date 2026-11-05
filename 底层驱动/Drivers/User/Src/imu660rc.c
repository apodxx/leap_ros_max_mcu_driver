/***************************************************************************************************
 * @file    imu660rc.c
 * @brief   IMU660RC 六轴陀螺仪驱动实现
 **************************************************************************************************/

#include "imu660rc.h"

static I2C_HandleTypeDef *imu_hi2c = NULL;
static uint8_t imu_addr = 0;
static float yaw_angle = 0.0f;
/* Initial stationary Z-bias, in rad/s; replaced by startup calibration.
   Rescaled from 0.005587 rad/s by 0.07 / (2000 / 32768). */
static const float gyro_z_default_bias = 0.006407619f;
static float gyro_z_bias = 0.006407619f;

/* 转换系数 */
#define GYRO_SENSITIVITY    (0.07f * 3.14159265f / 180.0f)  // ±2000dps: 0.07 deg/s/LSB -> rad/s/LSB
#define ACC_SENSITIVITY     (4.0f * 9.80665f / 32768.0f)                 // ±4g -> m/s²
#define TEMP_SENSITIVITY    (1.0f / 256.0f)                              // LSB/°C
#define TEMP_OFFSET         25.0f

/**
 * @brief  读取IMU寄存器
 */
static uint8_t IMU660RC_ReadReg(uint8_t reg, uint8_t *data)
{
    if (HAL_I2C_Mem_Read(imu_hi2c, imu_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000) == HAL_OK)
        return 0;
    return 1;
}

/**
 * @brief  写入IMU寄存器
 */
static uint8_t IMU660RC_WriteReg(uint8_t reg, uint8_t data)
{
    if (HAL_I2C_Mem_Write(imu_hi2c, imu_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000) == HAL_OK)
        return 0;
    return 1;
}

/**
 * @brief  初始化IMU660RC
 * @param  hi2c: I2C句柄
 * @param  i2c_addr: I2C地址 (0x68 或 0x6A)
 * @retval 0: 成功, 1: 失败
 */
uint8_t IMU660RC_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
    uint8_t chip_id = 0;

    imu_hi2c = hi2c;
    imu_addr = i2c_addr;

    HAL_Delay(10);

    /* 读取芯片ID验证 */
    if (IMU660RC_ReadReg(IMU660RC_CHIP_ID, &chip_id) != 0 || chip_id != 0x70)
    {
        return 1;
    }

    /* 软件复位：CTRL3.C_SW_RESET（FUNC_CFG_ACCESS 不是复位寄存器） */
    if (IMU660RC_WriteReg(IMU660RC_CTRL3, 0x01) != 0)
        return 1;
    HAL_Delay(30);

    /* 配置控制寄存器3：地址自动递增 */
    if (IMU660RC_WriteReg(IMU660RC_CTRL3, 0x44) != 0)
        return 1;

    /* 配置加速度计量程：±4g */
    if (IMU660RC_WriteReg(IMU660RC_CTRL8, 0x01) != 0)
        return 1;

    /* 配置陀螺仪量程：±2000dps */
    if (IMU660RC_WriteReg(IMU660RC_CTRL6, 0x04) != 0)
        return 1;

    /* 配置加速度计：高性能模式, ODR=208Hz */
    if (IMU660RC_WriteReg(IMU660RC_CTRL1, 0x15) != 0)
        return 1;

    /* 配置陀螺仪：高性能模式, ODR=416Hz */
    if (IMU660RC_WriteReg(IMU660RC_CTRL2, 0x18) != 0)
        return 1;

    /* 配置LPF1滤波器 */
    if (IMU660RC_WriteReg(IMU660RC_CTRL7, 0x01) != 0)
        return 1;

    /* 配置LPF2滤波器 */
    if (IMU660RC_WriteReg(IMU660RC_CTRL9, 0x08) != 0)
        return 1;

    HAL_Delay(10);

    yaw_angle = 0.0f;
    gyro_z_bias = gyro_z_default_bias;

    return 0;
}

/**
 * @brief  读取IMU660RC传感器数据
 * @param  hi2c: I2C句柄
 * @param  data: 存储数据的结构体指针
 * @retval 0: 成功, 1: 失败
 */
uint8_t IMU660RC_ReadData(I2C_HandleTypeDef *hi2c, IMU660RC_Data_t *data)
{
    uint8_t buf[14];

    /* 连续读取温度+陀螺仪+加速度计数据 */
    if (HAL_I2C_Mem_Read(hi2c, imu_addr << 1, IMU660RC_OUT_TEMP_L,
                         I2C_MEMADD_SIZE_8BIT, buf, 14, 1000) != HAL_OK)
        return 1;

    /* 解析原始数据（小端格式）*/
    data->temp_raw = (int16_t)(buf[0] | (buf[1] << 8));
    data->gyro_x_raw = (int16_t)(buf[2] | (buf[3] << 8));
    data->gyro_y_raw = (int16_t)(buf[4] | (buf[5] << 8));
    data->gyro_z_raw = (int16_t)(buf[6] | (buf[7] << 8));
    data->acc_x_raw = (int16_t)(buf[8] | (buf[9] << 8));
    data->acc_y_raw = (int16_t)(buf[10] | (buf[11] << 8));
    data->acc_z_raw = (int16_t)(buf[12] | (buf[13] << 8));

    /* 转换为物理单位 */
    data->gyro_x = data->gyro_x_raw * GYRO_SENSITIVITY;
    data->gyro_y = data->gyro_y_raw * GYRO_SENSITIVITY;
    data->gyro_z = data->gyro_z_raw * GYRO_SENSITIVITY;
    data->acc_x = data->acc_x_raw * ACC_SENSITIVITY;
    data->acc_y = data->acc_y_raw * ACC_SENSITIVITY;
    data->acc_z = data->acc_z_raw * ACC_SENSITIVITY;
    data->temperature = data->temp_raw * TEMP_SENSITIVITY + TEMP_OFFSET;

    return 0;
}

/**
 * @brief  更新姿态角 (互补滤波)
 * @param  data: IMU数据
 * @param  attitude: 姿态数据
 * @param  dt: 时间间隔 (秒)
 */
void IMU660RC_UpdateAttitude(const IMU660RC_Data_t *data, IMU660RC_Attitude_t *attitude, float dt)
{
    if (dt <= 0.0001f) return;
    yaw_angle += (data->gyro_z - gyro_z_bias) * dt * 57.2957795f;

    /* 限制在-180到180度 */
    while (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    while (yaw_angle < -180.0f) yaw_angle += 360.0f;

    attitude->yaw = yaw_angle;
    attitude->pitch = atan2f(data->acc_x, sqrtf(data->acc_y * data->acc_y +
                                                  data->acc_z * data->acc_z)) * 57.2957795f;
    attitude->roll = atan2f(data->acc_y, data->acc_z) * 57.2957795f;

    attitude->timestamp_ms = HAL_GetTick();
}

/**
 * @brief 静止陀螺仪零偏校准。校准期间必须保持小车静止。
 */
uint8_t IMU660RC_CalibrateGyro(uint16_t samples, uint32_t sample_period_ms)
{
    IMU660RC_Data_t data;
    float sum = 0.0f;
    uint16_t i;

    if (samples == 0U || imu_hi2c == NULL) return 1U;
    for (i = 0U; i < samples; i++)
    {
        if (IMU660RC_ReadData(imu_hi2c, &data) != 0U) return 1U;
        sum += data.gyro_z;
        if (sample_period_ms > 0U) HAL_Delay(sample_period_ms);
    }
    /* The vehicle must remain stationary throughout this measurement. */
    gyro_z_bias = sum / (float)samples;
    yaw_angle = 0.0f;
    return 0U;
}

/**
 * @brief  获取当前航向角
 */
float IMU660RC_GetYaw(void)
{
    return yaw_angle;
}

float IMU660RC_GetGyroZBias(void) { return gyro_z_bias; }

/**
 * @brief  复位航向角
 */
void IMU660RC_ResetYaw(void)
{
    yaw_angle = 0.0f;
}
