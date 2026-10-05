/**
  ******************************************************************************
  * @file    servo.h
  * @brief   RC Servo Motor driver for STM32 HAL
  ******************************************************************************
  */

#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define SERVO_MIN_PULSE_US       1000  /* Pulse width for 0 deg (in us) */
#define SERVO_MAX_PULSE_US       2000  /* Pulse width for 180 deg (in us) */

/* Gripper preset angles for 66 mm soda cans */
#define SERVO_GRIPPER_OPEN_DEG   0     /* Jaws open to receive/release can */
#define SERVO_GRIPPER_CLOSE_DEG  140   /* Jaws clamped onto can */

/**
  * @brief Servo instance handle
  */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    uint8_t            angle;
} servo_t;

/**
  * @brief  Initialize servo on timer channel and start PWM output
  * @param  servo: Pointer to servo_t handle
  * @param  htim: Timer handle (e.g. &htim8)
  * @param  channel: Timer channel (e.g. TIM_CHANNEL_1)
  * @retval HAL_OK on success, HAL_ERROR otherwise
  */
HAL_StatusTypeDef SERVO_Init(servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);

/**
  * @brief  Set servo angle in degrees (0 to 180)
  * @param  servo: Pointer to servo_t handle
  * @param  angle: Target angle in degrees (0..180)
  */
void SERVO_SetAngle(servo_t *servo, uint8_t angle);

/**
  * @brief  Set raw PWM pulse width in microseconds (1000 to 2000 us)
  * @param  servo: Pointer to servo_t handle
  * @param  pulse_us: Pulse width in us
  */
void SERVO_SetPulse(servo_t *servo, uint16_t pulse_us);

/**
  * @brief  Close gripper jaws to grasp a can
  * @param  servo: Pointer to servo_t handle
  */
void SERVO_Grip(servo_t *servo);

/**
  * @brief  Open gripper jaws to release a can
  * @param  servo: Pointer to servo_t handle
  */
void SERVO_Release(servo_t *servo);

/**
  * @brief  Stop PWM output to relax the servo motor
  * @param  servo: Pointer to servo_t handle
  */
void SERVO_Stop(servo_t *servo);

#endif /* BSP_SERVO_H */
