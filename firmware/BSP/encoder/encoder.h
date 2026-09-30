/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Quadrature encoder driver for STM32
  ******************************************************************************
  */

#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Datasheet FIT0483: 7*2*100 pulses/rev (2800 ticks/rev in 4x mode) */
#define ENCODER_DEFAULT_CPR_WHEEL       2800.0f
#define ENCODER_DEFAULT_WHEEL_DIAM_MM   43.0f    /* Wheel diameter in mm */

/**
  * @brief Encoder Instance Handle
  */
typedef struct {
    TIM_HandleTypeDef *htim;          /* Timer handle */
    uint32_t           last_counter;  /* Previous counter value */
    int64_t            total_ticks;   /* Cumulative ticks */
    int32_t            delta_ticks;   /* Ticks in last update */
    float              cpr_wheel;     /* Counts per revolution */
    float              wheel_diam_mm; /* Wheel diameter in mm */
    float              rpm;           /* Speed in RPM */
    float              speed_mm_s;    /* Speed in mm/s */
    float              distance_mm;   /* Distance in mm */
    bool               is_32bit;      /* True if 32-bit timer */
    int8_t             inverted;      /* Direction factor (+1 or -1) */
} encoder_t;

/**
  * @brief  Initialize an encoder instance and start hardware timer decoding
  * @param  enc: Pointer to encoder_t handle
  * @param  htim: Timer handle configured in Encoder Mode (e.g. &htim2, &htim4)
  * @param  cpr_wheel: Counts per revolution of wheel (e.g. 4800.0f)
  * @param  wheel_diam_mm: Wheel diameter in mm (e.g. 43.0f)
  * @param  inverted: Invert direction count if true
  * @retval HAL_OK on success, HAL_ERROR otherwise
  */
HAL_StatusTypeDef Encoder_Init(encoder_t *enc, TIM_HandleTypeDef *htim, float cpr_wheel, float wheel_diam_mm, bool inverted);

/**
  * @brief  Update encoder ticks, speed and distance (call at regular periodic intervals, e.g. 10ms..50ms)
  * @param  enc: Pointer to encoder_t handle
  * @param  dt_seconds: Time elapsed since last update in seconds (e.g. 0.020f for 20ms)
  */
void Encoder_Update(encoder_t *enc, float dt_seconds);

/**
  * @brief  Reset encoder tick count and odometry distance to zero
  * @param  enc: Pointer to encoder_t handle
  */
void Encoder_Reset(encoder_t *enc);

/**
  * @brief  Get cumulative tick count
  */
static inline int64_t Encoder_GetTicks(const encoder_t *enc)
{
    return enc ? enc->total_ticks : 0;
}

/**
  * @brief  Get ticks counted during the last sample window
  */
static inline int32_t Encoder_GetDelta(const encoder_t *enc)
{
    return enc ? enc->delta_ticks : 0;
}

/**
  * @brief  Get filtered wheel speed in RPM
  */
static inline float Encoder_GetRPM(const encoder_t *enc)
{
    return enc ? enc->rpm : 0.0f;
}

/**
  * @brief  Get linear speed in mm/s
  */
static inline float Encoder_GetSpeed(const encoder_t *enc)
{
    return enc ? enc->speed_mm_s : 0.0f;
}

/**
  * @brief  Get cumulative traveled distance in mm
  */
static inline float Encoder_GetDistance(const encoder_t *enc)
{
    return enc ? enc->distance_mm : 0.0f;
}

#endif /* BSP_ENCODER_H */
