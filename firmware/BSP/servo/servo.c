/**
  ******************************************************************************
  * @file    servo.c
  * @brief   RC Servo Motor driver implementation
  ******************************************************************************
  */

#include "servo.h"

HAL_StatusTypeDef SERVO_Init(servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (servo == NULL || htim == NULL) {
        return HAL_ERROR;
    }

    servo->htim = htim;
    servo->channel = channel;
    servo->angle = 90;

    if (HAL_TIM_PWM_Start(servo->htim, servo->channel) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Enable main output for advanced timers (TIM1 / TIM8) */
    __HAL_TIM_MOE_ENABLE(servo->htim);

    /* Start at open position */
    SERVO_Release(servo);

    return HAL_OK;
}

void SERVO_SetAngle(servo_t *servo, uint8_t angle)
{
    if (servo == NULL || servo->htim == NULL) {
        return;
    }

    if (angle > 180) {
        angle = 180;
    }

    /* Map 0..180 deg to 1000..2000 us */
    uint16_t pulse = SERVO_MIN_PULSE_US + ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;

    SERVO_SetPulse(servo, pulse);
    servo->angle = angle;
}

void SERVO_SetPulse(servo_t *servo, uint16_t pulse_us)
{
    if (servo == NULL || servo->htim == NULL) {
        return;
    }

    if (pulse_us < SERVO_MIN_PULSE_US) {
        pulse_us = SERVO_MIN_PULSE_US;
    } else if (pulse_us > SERVO_MAX_PULSE_US) {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
}

void SERVO_Grip(servo_t *servo)
{
    SERVO_SetAngle(servo, SERVO_GRIPPER_CLOSE_DEG);
}

void SERVO_Release(servo_t *servo)
{
    SERVO_SetAngle(servo, SERVO_GRIPPER_OPEN_DEG);
}

void SERVO_Stop(servo_t *servo)
{
    if (servo == NULL || servo->htim == NULL) {
        return;
    }

    HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}
