/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Implementation of Quadrature Encoder BSP driver for STM32
  ******************************************************************************
  */

#include "encoder.h"
#include <string.h>

#define PI_CONST 3.14159265358979323846f

HAL_StatusTypeDef Encoder_Init(encoder_t *enc, TIM_HandleTypeDef *htim, float cpr_wheel, float wheel_diam_mm, bool inverted)
{
    if (enc == NULL || htim == NULL) {
        return HAL_ERROR;
    }

    memset(enc, 0, sizeof(encoder_t));

    enc->htim          = htim;
    enc->cpr_wheel     = (cpr_wheel > 0.0f) ? cpr_wheel : ENCODER_DEFAULT_CPR_WHEEL;
    enc->wheel_diam_mm = (wheel_diam_mm > 0.0f) ? wheel_diam_mm : ENCODER_DEFAULT_WHEEL_DIAM_MM;
    enc->inverted      = inverted ? -1 : 1;
    enc->is_32bit      = (IS_TIM_32B_COUNTER_INSTANCE(htim->Instance) != 0U);

    /* Reset hardware timer counter */
    __HAL_TIM_SET_COUNTER(htim, 0);
    enc->last_counter = 0;

    /* Start encoder interface on both channels (TI1 and TI2) */
    if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void Encoder_Update(encoder_t *enc, float dt_seconds)
{
    if (enc == NULL || enc->htim == NULL) {
        return;
    }

    uint32_t current_cnt = __HAL_TIM_GET_COUNTER(enc->htim);
    int32_t delta;

    if (enc->is_32bit) {
        delta = (int32_t)(current_cnt - enc->last_counter);
    } else {
        /* Handle 16-bit roll-over */
        delta = (int16_t)((uint16_t)current_cnt - (uint16_t)enc->last_counter);
    }
    enc->last_counter = current_cnt;

    /* Apply direction polarity */
    delta *= enc->inverted;

    enc->delta_ticks = delta;
    enc->total_ticks += delta;

    /* Calculate RPM and distance */
    if (dt_seconds > 0.0001f && enc->cpr_wheel > 0.0f) {
        float raw_rpm = ((float)delta / enc->cpr_wheel) * (60.0f / dt_seconds);

        /* Filter RPM */
        enc->rpm = (0.7f * enc->rpm) + (0.3f * raw_rpm);

        /* Linear speed and distance */
        float circumference_mm = PI_CONST * enc->wheel_diam_mm;
        enc->speed_mm_s = (enc->rpm / 60.0f) * circumference_mm;
        enc->distance_mm = ((float)enc->total_ticks / enc->cpr_wheel) * circumference_mm;
    }
}

void Encoder_Reset(encoder_t *enc)
{
    if (enc == NULL || enc->htim == NULL) {
        return;
    }

    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->last_counter = 0;
    enc->total_ticks  = 0;
    enc->delta_ticks  = 0;
    enc->rpm          = 0.0f;
    enc->speed_mm_s   = 0.0f;
    enc->distance_mm  = 0.0f;
}
