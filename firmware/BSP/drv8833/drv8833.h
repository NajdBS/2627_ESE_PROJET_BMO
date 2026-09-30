/**
  ******************************************************************************
  * @file    drv8833.h
  * @brief   DRV8833 Dual H-Bridge motor driver for STM32
  ******************************************************************************
  */

#ifndef BSP_DRV8833_H
#define BSP_DRV8833_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
  * @brief Motor operational state
  */
typedef enum {
    DRV8833_STATE_COAST = 0, /* Low-power freewheel (IN1=0, IN2=0) */
    DRV8833_STATE_FORWARD,   /* Rotating forward (IN1=PWM, IN2=0)  */
    DRV8833_STATE_REVERSE,   /* Rotating backward (IN1=0, IN2=PWM) */
    DRV8833_STATE_BRAKE      /* Active electronic brake (IN1=1, IN2=1) */
} drv8833_state_t;

/**
  * @brief DRV8833 Motor Instance Handle
  */
typedef struct {
    TIM_HandleTypeDef *htim;          /* Timer handle */
    uint32_t           ch_in1;        /* Timer channel for IN1 */
    uint32_t           ch_in2;        /* Timer channel for IN2 */
    uint32_t           max_pwm;       /* Timer period (ARR) */
    int8_t             inverted;      /* Direction inversion factor (+1 or -1) */
    int16_t            current_speed; /* Current speed [-1000..+1000] */
    drv8833_state_t    state;         /* Motor state */
} drv8833_t;

/**
  * @brief  Initialize a motor handle and start PWM channels
  * @param  motor: Pointer to drv8833_t instance
  * @param  htim: Timer handle generating PWM
  * @param  ch_in1: Timer channel for IN1
  * @param  ch_in2: Timer channel for IN2
  * @param  inverted: true to invert direction (+1 becomes reverse, useful for mirrored wheels)
  * @retval HAL_OK on success, HAL_ERROR otherwise
  */
HAL_StatusTypeDef DRV8833_Init(drv8833_t *motor, TIM_HandleTypeDef *htim, uint32_t ch_in1, uint32_t ch_in2, bool inverted);

/**
  * @brief  Set motor speed in per-mille (-1000 to +1000, corresponding to -100.0% to +100.0%)
  * @param  motor: Pointer to drv8833_t instance
  * @param  speed_permille: Speed value clamped between -1000 and +1000
  */
void DRV8833_SetSpeed(drv8833_t *motor, int16_t speed_permille);

/**
  * @brief  Set raw PWM pulse width directly
  * @param  motor: Pointer to drv8833_t instance
  * @param  raw_pwm: Pulse width clamped between -max_pwm and +max_pwm
  */
void DRV8833_SetRawPWM(drv8833_t *motor, int16_t raw_pwm);

/**
  * @brief  Brake the motor (IN1=1, IN2=1)
  * @param  motor: Pointer to drv8833_t instance
  */
void DRV8833_Brake(drv8833_t *motor);

/**
  * @brief  Set motor in coast mode (IN1=0, IN2=0)
  * @param  motor: Pointer to drv8833_t instance
  */
void DRV8833_Coast(drv8833_t *motor);

/**
  * @brief  Stop PWM generation on timer channels
  * @param  motor: Pointer to drv8833_t instance
  */
void DRV8833_Stop(drv8833_t *motor);

#endif /* BSP_DRV8833_H */
