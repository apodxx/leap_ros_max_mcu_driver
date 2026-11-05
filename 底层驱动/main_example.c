/***************************************************************************************************
 * @file    main_example.c
 * @brief   麦轮小车导航系统示例程序
 * @note    使用方法:
 *          1. 将此文件内容集成到你的main.c中
 *          2. 确保所有驱动文件已添加到项目中
 *          3. 配置I2C1 (PB8/PB9)
 *          4. 根据实际情况修改路径点和PID参数
 **************************************************************************************************/

#include "main.h"
#include "navigation_controller.h"
#include <stdio.h>

/* 外部变量 */
I2C_HandleTypeDef hi2c1;

/* 函数声明 */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void Error_Handler(void);

/* 全局变量 */
NavController_t nav_controller;

/* 示例路径: 正方形路径 (边长1米) */
Waypoint_t square_path[] = {
    {0.0f, 0.0f, 0.0f,           0.2f},  // 起点
    {1.0f, 0.0f, 0.0f,           0.2f},  // 向前1米
    {1.0f, 1.0f, 1.5707963f,     0.2f},  // 左转,向前1米 (90度)
    {0.0f, 1.0f, 3.14159265f,    0.2f},  // 左转,向前1米 (180度)
    {0.0f, 0.0f, -1.5707963f,    0.2f},  // 左转,向前1米 (270度)
};

/**
 * @brief  主函数
 */
int main(void)
{
    /* 系统初始化 */
    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    /* 延时等待系统稳定 */
    HAL_Delay(100);

    /* 初始化导航控制器 */
    if (NavController_Init(&nav_controller, &hi2c1, 0.02f) != 0)  // 20ms控制周期
    {
        /* IMU初始化失败,闪烁LED提示 */
        while (1)
        {
            HAL_Delay(200);
        }
    }

    /* 设置PID参数 (需要根据实际调试) */
    PathTracker_SetPIDParams(&nav_controller.path_tracker,
                              1.5f, 0.01f, 0.2f,   // 位置PID: Kp, Ki, Kd
                              2.0f, 0.05f, 0.3f);  // 角度PID: Kp, Ki, Kd

    /* 等待3秒后启动 */
    HAL_Delay(3000);

    /* 启动自动导航 */
    NavController_StartAutoNav(&nav_controller, square_path,
                                sizeof(square_path) / sizeof(Waypoint_t));

    uint32_t last_print = HAL_GetTick();

    /* 主循环 */
    while (1)
    {
        /* 更新导航控制器 (20ms周期) */
        NavController_Update(&nav_controller);

        /* 每500ms打印一次状态 */
        if (HAL_GetTick() - last_print > 500)
        {
            last_print = HAL_GetTick();

            Position_t pos = NavController_GetPosition(&nav_controller);

            /* 通过串口或USB打印位姿信息 */
            printf("Pos: X=%.3f Y=%.3f Theta=%.2f deg, State=%d\r\n",
                   pos.x, pos.y, pos.theta * 57.2957795f,
                   NavController_GetState(&nav_controller));
        }

        /* 检查是否完成 */
        if (NavController_GetState(&nav_controller) == NAV_STATE_IDLE)
        {
            /* 任务完成,停止 */
            break;
        }

        HAL_Delay(1);
    }

    /* 停止所有电机 */
    NavController_Stop(&nav_controller);

    while (1)
    {
        HAL_Delay(1000);
    }
}

/**
 * @brief  I2C1初始化 (PB8=SCL, PB9=SDA)
 */
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

/**
 * @brief  I2C MSP初始化
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        /* I2C1 GPIO配置: PB8=SCL, PB9=SDA */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
 * @brief  GPIO初始化
 */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

/**
 * @brief  系统时钟配置 (480MHz)
 */
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

/**
 * @brief  错误处理
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
