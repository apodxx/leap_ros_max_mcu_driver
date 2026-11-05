/***************************************************************************************************
 * @file    main.c
 * @brief   麦轮小车导航系统主程序
 * @version V1.0
 * @date    2026-09-10
 * @note    完整的路径跟踪导航系统，集成四路电机、编码器、IMU陀螺仪
 **************************************************************************************************/

#include "main.h"
#include "navigation_controller.h"
#include "tb6612_motor.h"
#include "led.h"
#include "wifi_spi.h"
#include "encoder.h"
#include <stdio.h>

/* I2C句柄 */
I2C_HandleTypeDef hi2c1;

/* 导航控制器 */
NavController_t nav_controller;

/* 函数声明 */
void SystemClock_Config(void);
static void MX_I2C1_Init(void);

/* 示例路径: 正方形路径 (边长1米) */
/* legacy square path removed */
/*
    {0.0f, 0.0f, 0.0f,           0.2f},  // 起点
    {1.0f, 0.0f, 0.0f,           0.2f},  // 向前1米
    {1.0f, 1.0f, 1.5707963f,     0.2f},  // 左转90度,向前1米
    {0.0f, 1.0f, 3.14159265f,    0.2f},  // 左转90度,向前1米
    {0.0f, 0.0f, -1.5707963f,    0.2f},  // 左转90度,向前1米(回到起点)
*/

/***************************************************************************************************
 * @brief  主函数
 **************************************************************************************************/
int main(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    SystemClock_Config();

    LED_Init();
    MX_I2C1_Init();
    WIFI_SPI_Init();
    /* WiFi module configuration must match the PC running the Qt UI. */
    WIFI_SPI_ModuleInit(WIFI_SPI_SSID, WIFI_SPI_PASSWORD);
    WIFI_SPI_SocketConnect("UDP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT,
                           WIFI_SPI_LOCAL_PORT);
    if (NavController_Init(&nav_controller, &hi2c1, 0.02f) != 0U)
        Error_Handler();

    /* Publish the calibrated zero reference before the first integration. */
    Motor_StopAll();
    NavController_SendTelemetry(&nav_controller);
    // NavController_SendImuDebug(&nav_controller);  // IMU text debugging paused.
    NavController_SendInitInfo(&nav_controller);
    uint32_t startup_time = HAL_GetTick();
    nav_controller.last_update_ms = startup_time;
    uint32_t last_telemetry = startup_time;
    // uint32_t last_imu_debug = startup_time;
    uint32_t last_encoder_debug = startup_time;

    while (1)
    {
        NavController_Update(&nav_controller);
        uint32_t now = HAL_GetTick();
        /* Diagnostic firmware: cap telemetry at 10 Hz to reduce blocking SPI
         * traffic. The requested control period remains 20 ms. */
        if ((now - last_telemetry) >= 100U) {
            NavController_SendTelemetry(&nav_controller);
            last_telemetry = now;
        }
        if ((now - last_encoder_debug) >= 100U) {
            NavController_SendMotionDebug(&nav_controller);
            last_encoder_debug = now;
        }
        LED1_Toggle;
        HAL_Delay(1);
    }
}

/***************************************************************************************************
 * @brief  I2C1初始化 (PB8=SCL, PB9=SDA)
 **************************************************************************************************/
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x60404E72;  // 400kHz @ 240MHz APB1
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置模拟滤波器 */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置数字滤波器 */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/***************************************************************************************************
 * @brief  系统时钟配置 - 480MHz
 **************************************************************************************************/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/***************************************************************************************************
 * @brief  错误处理函数
 **************************************************************************************************/
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* LED快速闪烁表示错误 */
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* 用户可以在这里添加自己的实现来报告文件名和行号 */
}
#endif














