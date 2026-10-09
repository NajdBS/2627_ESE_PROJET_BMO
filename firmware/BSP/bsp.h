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
#include "buzzer.h"
#include "bq27220.h"
#include "bq25896.h"

/**
  * @brief Global BMO Hardware Handle
  */
typedef struct {
    /* Actuators & Audio */
    drv8833_t    motor_left;
    drv8833_t    motor_right;
    servo_t      gripper;
    buzzer_t     buzzer;

    /* Power Management */
    bq27220_t    fuel_gauge;
    bq25896_t    charger;

    /* Sensors */
    encoder_t    enc_left;
    encoder_t    enc_right;
    mpu6050_t    imu;
    vl53l0x_t    tof_left;
    vl53l0x_t    tof_right;
    /* vl53l0x_t    tof_center; */  /* Optional 3rd Center ToF (XSHUT on EXT_PB13 -> 0x32) */
    apds9960_t   color;
    ydlidar_x2_t lidar;

    /* Hardware Presence & Status Flags */
    bool oled_ok;
    bool motors_ok;
    bool encoders_ok;
    bool gripper_ok;
    bool neopixel_ok;
    bool imu_ok;
    bool tof_left_ok;
    bool tof_right_ok;
    /* bool tof_center_ok; */       /* Optional 3rd Center ToF status */
    bool color_ok;
    bool lidar_ok;
    bool buzzer_ok;
    bool fuel_gauge_ok;
    bool charger_ok;
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
