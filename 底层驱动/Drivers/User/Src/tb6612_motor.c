/***************************************************************************************************
 * @file    tb6612_motor.c
 * @brief   TB6612 四路电机驱动实现
 * @note    引脚配置:
 *          STBY=PB0 (共用使能)
 *          PWM_A=PD12 (TIM4_CH1), AIN1=PD8,  AIN2=PD9
 *          PWM_B=PD13 (TIM4_CH2), BIN1=PD10, BIN2=PD11
 *          PWM_C=PD14 (TIM4_CH3), CIN1=PC0,  CIN2=PC1
 *          PWM_D=PD15 (TIM4_CH4), DIN1=PA7,  DIN2=PB14
 **************************************************************************************************/

#include "tb6612_motor.h"

extern void Error_Handler(void);

/* PWM定时器配置 */
#define MOTOR_PWM_FREQUENCY     20000U          // 20kHz PWM频率
#define MOTOR_TIMER_CLOCK       240000000U      // TIM4时钟 240MHz
#define MOTOR_TIMER_PSC         23U             // 预分频器
#define MOTOR_TIMER_PERIOD      ((MOTOR_TIMER_CLOCK / (MOTOR_TIMER_PSC + 1U) / MOTOR_PWM_FREQUENCY) - 1U)

static TIM_HandleTypeDef htim4;

/* 电机引脚定义 */
typedef struct {
    GPIO_TypeDef *pwm_port;
    uint16_t pwm_pin;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    uint32_t tim_channel;
} MotorPin_t;

static const MotorPin_t motor_pins[MOTOR_COUNT] = {
    /* Motor A - 左前 */
    {GPIOD, GPIO_PIN_12, GPIOD, GPIO_PIN_8,  GPIOD, GPIO_PIN_9,  TIM_CHANNEL_1},
    /* Motor B - 左后 */
    {GPIOD, GPIO_PIN_13, GPIOD, GPIO_PIN_10, GPIOD, GPIO_PIN_11, TIM_CHANNEL_2},
    /* Motor C - 右后：TIM4_CH3 */
    {GPIOD, GPIO_PIN_14, GPIOC, GPIO_PIN_0,  GPIOC, GPIO_PIN_1,  TIM_CHANNEL_3},
    /* Motor D - 右前：TIM4_CH4 */
    {GPIOD, GPIO_PIN_15, GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_14, TIM_CHANNEL_4}
};


/* 机械安装方向校准：逻辑正值统一表示车辆前进 */
static const int8_t motor_direction_sign[MOTOR_COUNT] = {-1, 1, 1, -1};
/* GPIO初始化 */
static void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 使能时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* STBY引脚初始化 (PB0) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 初始化所有电机的IN1/IN2引脚 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        HAL_GPIO_WritePin(motor_pins[i].in1_port, motor_pins[i].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_pins[i].in2_port, motor_pins[i].in2_pin, GPIO_PIN_RESET);

        gpio.Pin = motor_pins[i].in1_pin;
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(motor_pins[i].in1_port, &gpio);

        gpio.Pin = motor_pins[i].in2_pin;
        HAL_GPIO_Init(motor_pins[i].in2_port, &gpio);
    }

    /* 初始化PWM引脚 (PD12/13/14/15, AF2) */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);
}

/* 电机初始化 */
void Motor_Init(void)
{
    TIM_OC_InitTypeDef output = {0};

    Motor_GPIO_Init();

    /* 配置TIM4 PWM */
    __HAL_RCC_TIM4_CLK_ENABLE();
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = MOTOR_TIMER_PSC;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = MOTOR_TIMER_PERIOD;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置4个PWM通道 */
    output.OCMode = TIM_OCMODE_PWM1;
    output.Pulse = 0;
    output.OCPolarity = TIM_OCPOLARITY_HIGH;
    output.OCFastMode = TIM_OCFAST_DISABLE;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        if (HAL_TIM_PWM_ConfigChannel(&htim4, &output, motor_pins[i].tim_channel) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_PWM_Start(&htim4, motor_pins[i].tim_channel) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* 使能驱动器 */
    Motor_Enable();
}

/* 设置单个电机速度 */
void Motor_SetSpeed(MotorID_t motor, int16_t speed)
{
    uint32_t duty;
    uint16_t compare;

    if ((unsigned int)motor >= MOTOR_COUNT) return;

    /* 限幅 */
    if (speed > MOTOR_PWM_MAX)
    {
        speed = MOTOR_PWM_MAX;
    }
    else if (speed < -MOTOR_PWM_MAX)
    {
        speed = -MOTOR_PWM_MAX;
    }

    /* Apply installation polarity after clamping to avoid int16_t overflow. */
    speed = (int16_t)(speed * motor_direction_sign[motor]);

    /* 先关闭PWM避免H桥短路 */
    __HAL_TIM_SET_COMPARE(&htim4, motor_pins[motor].tim_channel, 0U);

    /* 设置方向 */
    if (speed > 0)
    {
        HAL_GPIO_WritePin(motor_pins[motor].in1_port, motor_pins[motor].in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_pins[motor].in2_port, motor_pins[motor].in2_pin, GPIO_PIN_RESET);
        duty = (uint32_t)speed;
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(motor_pins[motor].in1_port, motor_pins[motor].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_pins[motor].in2_port, motor_pins[motor].in2_pin, GPIO_PIN_SET);
        duty = (uint32_t)(-speed);
    }
    else
    {
        HAL_GPIO_WritePin(motor_pins[motor].in1_port, motor_pins[motor].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_pins[motor].in2_port, motor_pins[motor].in2_pin, GPIO_PIN_RESET);
        duty = 0U;
    }

    /* 设置PWM占空比 */
    compare = (uint16_t)((duty * (MOTOR_TIMER_PERIOD + 1U)) / MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim4, motor_pins[motor].tim_channel, compare);
}

/* 同时设置所有电机速度 */
void Motor_SetAllSpeed(int16_t speedA, int16_t speedB, int16_t speedC, int16_t speedD)
{
    Motor_SetSpeed(MOTOR_A, speedA);
    Motor_SetSpeed(MOTOR_B, speedB);
    Motor_SetSpeed(MOTOR_C, speedC);
    Motor_SetSpeed(MOTOR_D, speedD);
}

/* 停止单个电机 */
void Motor_Stop(MotorID_t motor)
{
    Motor_SetSpeed(motor, 0);
}

/* 停止所有电机 */
void Motor_StopAll(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        Motor_Stop((MotorID_t)i);
    }
}

/* 使能电机驱动器 */
void Motor_Enable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/* 禁用电机驱动器 */
void Motor_Disable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}



