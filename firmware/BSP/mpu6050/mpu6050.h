/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU-6050 6-DOF IMU (Accelerometer + Gyroscope) driver for STM32 HAL
  ******************************************************************************
  */

#ifndef BSP_MPU6050_H
#define BSP_MPU6050_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C Address Definitions (8-bit for STM32 HAL) */
#define MPU6050_I2C_ADDR_DEFAULT    (0x68 << 1)  /* 0xD0 when AD0 pin is GND */
#define MPU6050_I2C_ADDR_ALT        (0x69 << 1)  /* 0xD2 when AD0 pin is VCC */

/* Device Identification */
#define MPU6050_WHO_AM_I_VAL        (0x68)

/**
  * @brief MPU-6050 Device Handle
  */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr_8bit;
    bool is_initialized;

    /* Raw 16-bit register values */
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    /* Scaled engineering units */
    float accel_x;          /* Acceleration in g (+/- 2g) */
    float accel_y;          /* Acceleration in g */
    float accel_z;          /* Acceleration in g */
    float temperature;      /* Temperature in deg C */
    float gyro_x;           /* Angular rate in deg/s (+/- 500 deg/s) */
    float gyro_y;           /* Angular rate in deg/s */
    float gyro_z;           /* Angular rate in deg/s */

    /* Calibration zero-rate offsets */
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;

    /* Integrated robot heading (Yaw) */
    float yaw;              /* Current heading in degrees (continuous) */
} mpu6050_t;

/**
  * @brief  Initialize MPU-6050 on the specified I2C bus.
  *         Configures DLPF to 42 Hz, Gyro to +/-500 deg/s, Accel to +/-2g.
  * @param  dev: Pointer to MPU-6050 device handle.
  * @param  hi2c: Pointer to STM32 HAL I2C handle (e.g. &hi2c3).
  * @retval true if device was detected and configured successfully, false otherwise.
  */
bool MPU6050_Init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c);

/**
  * @brief  Calibrate gyroscope zero-rate offset by averaging stationary samples.
  *         The robot MUST be completely stationary during this call (~1 second).
  * @param  dev: Pointer to MPU-6050 device handle.
  * @param  samples: Number of samples to average (e.g. 100 to 200).
  * @retval true if calibration succeeded, false on I2C error.
  */
bool MPU6050_Calibrate(mpu6050_t *dev, uint16_t samples);

/**
  * @brief  Read all 14 sensor registers (Accel, Temp, Gyro) in a single I2C burst.
  * @param  dev: Pointer to MPU-6050 device handle.
  * @retval true if read succeeded, false on I2C error.
  */
bool MPU6050_ReadRaw(mpu6050_t *dev);

/**
  * @brief  Read sensor data and integrate heading angle (Yaw).
  *         Applies offset correction and deadband filtering.
  * @param  dev: Pointer to MPU-6050 device handle.
  * @param  dt_seconds: Elapsed time since last update in seconds (e.g. 0.01f for 100 Hz).
  * @retval true if update succeeded, false on I2C error.
  */
bool MPU6050_Update(mpu6050_t *dev, float dt_seconds);

/**
  * @brief  Reset the integrated heading angle (Yaw) to 0.0 degrees.
  * @param  dev: Pointer to MPU-6050 device handle.
  */
void MPU6050_ResetYaw(mpu6050_t *dev);

#endif /* BSP_MPU6050_H */
