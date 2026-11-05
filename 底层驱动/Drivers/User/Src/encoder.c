/***************************************************************************************************
 * @file    encoder.c
 * @brief   四路编码器驱动实现
 **************************************************************************************************/

#include "encoder.h"

extern void Error_Handler(void);

#define ENCODER_INPUT_FILTER    10U

/* 编码器定时器句柄 */
static TIM_HandleTypeDef htim1, htim2, htim3, htim5;

/* 编码器配置结构 */
typedef struct {
    TIM_HandleTypeDef *htim;
    TIM_TypeDef *instance;
    GPIO_TypeDef *port_a;
    uint16_t pin_a;
    GPIO_TypeDef *port_b;
    uint16_t pin_b;
    uint32_t af;
    uint16_t previous_count;
    int32_t total_count;
} EncoderConfig_t;

static EncoderConfig_t encoders[ENCODER_COUNT] = {
    {&htim5, TIM5, GPIOA, GPIO_PIN_0,  GPIOA, GPIO_PIN_1,  GPIO_AF2_TIM5, 0, 0},  // Encoder 1
    {&htim3, TIM3, GPIOB, GPIO_PIN_4,  GPIOB, GPIO_PIN_5,  GPIO_AF2_TIM3, 0, 0},  // Encoder 2
    {&htim1, TIM1, GPIOA, GPIO_PIN_8,  GPIOE, GPIO_PIN_11, GPIO_AF1_TIM1, 0, 0},  // Encoder 3
    {&htim2, TIM2, GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_3,  GPIO_AF1_TIM2, 0, 0}   // Encoder 4
};

/* Encoder polarity: forward motion is positive for all wheels. */
static const int8_t encoder_direction_sign[ENCODER_COUNT] = {-1, -1, 1, 1};

/* GPIO和定时器初始化 */
static void Encoder_GPIO_Clock_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
}

/* 单个编码器初始化 */
static void Encoder_Single_Init(EncoderID_t id)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_Encoder_InitTypeDef encoder_config = {0};
    EncoderConfig_t *enc = &encoders[id];

    /* GPIO配置 */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = enc->af;

    gpio.Pin = enc->pin_a;
    HAL_GPIO_Init(enc->port_a, &gpio);

    gpio.Pin = enc->pin_b;
    gpio.Alternate = enc->af;
    HAL_GPIO_Init(enc->port_b, &gpio);

    /* 定时器编码器模式配置 */
    enc->htim->Instance = enc->instance;
    enc->htim->Init.Prescaler = 0U;
    enc->htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    enc->htim->Init.Period = 0xFFFFU;
    enc->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    enc->htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = ENCODER_INPUT_FILTER;
    encoder_config.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Filter = ENCODER_INPUT_FILTER;

    if (HAL_TIM_Encoder_Init(enc->htim, &encoder_config) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }

    /* 复位计数器 */
    __HAL_TIM_SET_COUNTER(enc->htim, 0U);
    enc->previous_count = 0U;
    enc->total_count = 0;
}

/* 编码器初始化 */
void Encoder_Init(void)
{
    Encoder_GPIO_Clock_Init();

    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
        Encoder_Single_Init((EncoderID_t)i);
    }
}

/* 读取编码器增量值 */
int16_t Encoder_ReadDelta(EncoderID_t encoder)
{
    if (encoder >= ENCODER_COUNT) return 0;

    EncoderConfig_t *enc = &encoders[encoder];
    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);

    /* 模运算保持脉冲精度 */
    int16_t delta = (int16_t)(uint16_t)(current - enc->previous_count);
    enc->previous_count = current;
    delta = (int16_t)(delta * encoder_direction_sign[encoder]);
    enc->total_count += delta;

    return delta;
}

/* 读取编码器累计值 */
int32_t Encoder_ReadTotal(EncoderID_t encoder)
{
    if (encoder >= ENCODER_COUNT) return 0;
    return encoders[encoder].total_count;
}

/* 复位单个编码器 */
void Encoder_Reset(EncoderID_t encoder)
{
    if (encoder >= ENCODER_COUNT) return;

    EncoderConfig_t *enc = &encoders[encoder];
    __HAL_TIM_SET_COUNTER(enc->htim, 0U);
    enc->previous_count = 0U;
    enc->total_count = 0;
}

/* 复位所有编码器 */
void Encoder_ResetAll(void)
{
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
        Encoder_Reset((EncoderID_t)i);
    }
}


