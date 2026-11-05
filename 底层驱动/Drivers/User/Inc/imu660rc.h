/***************************************************************************************************
 * @file    imu660rc.h
 * @brief   IMU660RC 六轴陀螺仪驱动 (I2C接口)
 * @note    引脚配置: SCL=PB8, SDA=PB9 (I2C1)
 *          I2C地址: 0x68 (当SA0=低) 或 0x6A (当SA0=高)
 **************************************************************************************************/

#ifndef __IMU660RC_H
#define __IMU660RC_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <math.h>

/* IMU660RC I2C地址 */
#define IMU660RC_ADDR_0         0x68    // SA0=低
#define IMU660RC_ADDR_1         0x6A    // SA0=高
#define IMU660RC_ADDR_2         0x6B    // 部分模块/桥接板使用的兼容地址

/* 寄存器地址 */
#define IMU660RC_FUNC_CFG_ACCESS    0x01
#define IMU660RC_CHIP_ID            0x0F
#define IMU660RC_CTRL1              0x10
#define IMU660RC_CTRL2              0x11
#define IMU660RC_CTRL3              0x12
#define IMU660RC_CTRL6              0x15
#define IMU660RC_CTRL7              0x16
#define IMU660RC_CTRL8              0x17
#define IMU660RC_CTRL9              0x18
#define IMU660RC_STATUS_REG         0x1E
#define IMU660RC_OUT_TEMP_L         0x20
#define IMU660RC_OUT_TEMP_H         0x21
#define IMU660RC_OUTX_L_G           0x22
#define IMU660RC_OUTX_H_G           0x23
#define IMU660RC_OUTY_L_G           0x24
#define IMU660RC_OUTY_H_G           0x25
#define IMU660RC_OUTZ_L_G           0x26
#define IMU660RC_OUTZ_H_G           0x27
#define IMU660RC_OUTX_L_A           0x28
#define IMU660RC_OUTX_H_A           0x29
#define IMU660RC_OUTY_L_A           0x2A
#define IMU660RC_OUTY_H_A           0x2B
#define IMU660RC_OUTZ_L_A           0x2C
#define IMU660RC_OUTZ_H_A           0x2D

/* 数据结构 */
typedef struct {
    int16_t acc_x_raw;
    int16_t acc_y_raw;
    int16_t acc_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t temp_raw;

    float acc_x;        // 单位: m/s²
    float acc_y;
    float acc_z;
    float gyro_x;       // 单位: rad/s
    float gyro_y;
    float gyro_z;
    float temperature;  // 单位: °C
} IMU660RC_Data_t;

/* 姿态数据结构 */
typedef struct {
    float yaw;      // 航向角 (度)
    float pitch;    // 俯仰角 (度)
    float roll;     // 横滚角 (度)
    uint32_t timestamp_ms;
} IMU660RC_Attitude_t;

/* 函数声明 */
uint8_t IMU660RC_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
uint8_t IMU660RC_ReadData(I2C_HandleTypeDef *hi2c, IMU660RC_Data_t *data);
void IMU660RC_UpdateAttitude(const IMU660RC_Data_t *data, IMU660RC_Attitude_t *attitude, float dt);
uint8_t IMU660RC_CalibrateGyro(uint16_t samples, uint32_t sample_period_ms);
float IMU660RC_GetYaw(void);
float IMU660RC_GetGyroZBias(void);
void IMU660RC_ResetYaw(void);

#endif /* __IMU660RC_H */
