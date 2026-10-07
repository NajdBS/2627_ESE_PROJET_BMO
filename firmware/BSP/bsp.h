/**
  ******************************************************************************
  * @file    bsp.h
  * @brief   Board Support Package facade for BMO Robot
  ******************************************************************************
  */

#ifndef BSP_H
#define BSP_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Drivers */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "bmo_screen.h"
#include "drv8833.h"
#include "encoder.h"
#include "servo.h"
#include "neopixel.h"
#include "vl53l0x.h"
#include "apds9960.h"
#include "mpu6050.h"
#include "ydlidar_x2.h"

/**
  * @brief Global BMO Hardware Handle
  */
typedef struct {
    /* Actuators */
    drv8833_t    motor_left;
    drv8833_t    motor_right;
    servo_t      gripper;

    /* Sensors */
    encoder_t    enc_left;
    encoder_t    enc_right;
    mpu6050_t    imu;
    vl53l0x_t    tof;
    apds9960_t   color;
    ydlidar_x2_t lidar;

    /* Hardware Presence & Status Flags */
    bool oled_ok;
    bool motors_ok;
    bool encoders_ok;
    bool gripper_ok;
    bool neopixel_ok;
    bool imu_ok;
    bool tof_ok;
    bool color_ok;
    bool lidar_ok;
} bsp_t;

/* Global BSP instance */
extern bsp_t g_bsp;

/**
  * @brief  Initialize all onboard hardware peripherals, actuators, and sensors.
  *         Safe against missing or unpowered optional sensors.
  * @retval true if baseline BSP initialized successfully.
  */
bool BSP_Init(void);

/**
  * @brief  Periodic update routine for sensors requiring continuous polling/integration (e.g. IMU).
  * @param  dt_seconds: Delta time since previous update call in seconds.
  */
void BSP_Update(float dt_seconds);

#endif /* BSP_H */
