/***************************************************************************************************
 * @file    encoder.h
 * @brief   四路编码器驱动 - TIM1/2/3/5编码器模式
 * @note    编码器1: E1A=PA0, E1B=PA1 (TIM5_CH1/CH2)
 *          编码器2: E2A=PB4, E2B=PB5 (TIM3_CH1/CH2)
 *          编码器3: E3A=PA8, E3B=PE11 (TIM1_CH1/CH2)
 *          编码器4: E4A=PA15, E4B=PB3 (TIM2_CH1/CH2)
 **************************************************************************************************/

#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* 编码器编号 (对应电机编号) */
typedef enum {
    ENCODER_1 = 0,    // 对应 Motor A (左前)
    ENCODER_2 = 1,    // 对应 Motor B (左后)
    ENCODER_3 = 2,    // 对应 Motor C (右前)
    ENCODER_4 = 3,    // 对应 Motor D (右后)
    ENCODER_COUNT = 4
} EncoderID_t;

/* 函数声明 */
void Encoder_Init(void);
int16_t Encoder_ReadDelta(EncoderID_t encoder);
int32_t Encoder_ReadTotal(EncoderID_t encoder);
void Encoder_Reset(EncoderID_t encoder);
void Encoder_ResetAll(void);

#endif /* __ENCODER_H */
