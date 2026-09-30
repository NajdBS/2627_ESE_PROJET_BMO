/**
  ******************************************************************************
  * @file    drv8833.c
  * @brief   Implementation of BSP driver for TI DRV8833 Dual H-Bridge on STM32
  ******************************************************************************
  */

#include "drv8833.h"

HAL_StatusTypeDef DRV8833_Init(drv8833_t *motor, TIM_HandleTypeDef *htim, uint32_t ch_in1, uint32_t ch_in2, bool inverted)
{
    if (motor == NULL || htim == NULL) {
        return HAL_ERROR;
    }

    motor->htim          = htim;
    motor->ch_in1        = ch_in1;
    motor->ch_in2        = ch_in2;
    motor->max_pwm       = __HAL_TIM_GET_AUTORELOAD(htim);
    motor->inverted      = inverted ? -1 : 1;
    motor->current_speed = 0;
    motor->state         = DRV8833_STATE_COAST;

    /* Ensure duty cycles start at 0 */
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in1, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in2, 0);

    /* Start PWM output on both channels */
    if (HAL_TIM_PWM_Start(motor->htim, motor->ch_in1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(motor->htim, motor->ch_in2) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void DRV8833_SetSpeed(drv8833_t *motor, int16_t speed_permille)
{
    if (motor == NULL || motor->htim == NULL) {
        return;
    }

    /* Clamp speed between -1000 and +1000 */
    if (speed_permille > 1000) {
        speed_permille = 1000;
    } else if (speed_permille < -1000) {
        speed_permille = -1000;
    }

    motor->current_speed = speed_permille;

    /* Apply inversion factor */
    int32_t effective_speed = (int32_t)speed_permille * motor->inverted;

    if (effective_speed > 0) {
        uint32_t pulse = ((uint32_t)effective_speed * motor->max_pwm) / 1000;
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in1, pulse);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in2, 0);
        motor->state = DRV8833_STATE_FORWARD;
    } else if (effective_speed < 0) {
        uint32_t pulse = ((uint32_t)(-effective_speed) * motor->max_pwm) / 1000;
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in1, 0);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in2, pulse);
        motor->state = DRV8833_STATE_REVERSE;
    } else {
        /* Zero speed: Coast */
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in1, 0);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in2, 0);
        motor->state = DRV8833_STATE_COAST;
    }
}

void DRV8833_SetRawPWM(drv8833_t *motor, int16_t raw_pwm)
{
    if (motor == NULL || motor->htim == NULL) {
        return;
    }

    int32_t max_val = (int32_t)motor->max_pwm;
    if (raw_pwm > max_val) {
        raw_pwm = max_val;
    } else if (raw_pwm < -max_val) {
        raw_pwm = -max_val;
    }

    /* Map to per-mille */
    int16_t permille = (int16_t)(((int32_t)raw_pwm * 1000) / max_val);
    DRV8833_SetSpeed(motor, permille);
}

void DRV8833_Brake(drv8833_t *motor)
{
    if (motor == NULL || motor->htim == NULL) {
        return;
    }

    /* Brake mode: IN1 = 1, IN2 = 1 */
    uint32_t high_duty = motor->max_pwm + 1;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in1, high_duty);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in2, high_duty);

    motor->current_speed = 0;
    motor->state = DRV8833_STATE_BRAKE;
}

void DRV8833_Coast(drv8833_t *motor)
{
    if (motor == NULL || motor->htim == NULL) {
        return;
    }

    /* Coast mode: IN1 = 0, IN2 = 0 */
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in1, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_in2, 0);

    motor->current_speed = 0;
    motor->state = DRV8833_STATE_COAST;
}

void DRV8833_Stop(drv8833_t *motor)
{
    if (motor == NULL || motor->htim == NULL) {
        return;
    }

    DRV8833_Coast(motor);
    HAL_TIM_PWM_Stop(motor->htim, motor->ch_in1);
    HAL_TIM_PWM_Stop(motor->htim, motor->ch_in2);
}
