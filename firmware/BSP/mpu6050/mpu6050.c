/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU-6050 6-DOF IMU (Accelerometer + Gyroscope) driver for STM32 HAL
  ******************************************************************************
  */

#include "mpu6050.h"
#include <math.h>
#include <string.h>

/* Internal Register Map */
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_REG_TEMP_OUT_H     0x41
#define MPU6050_REG_GYRO_XOUT_H    0x43
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_PWR_MGMT_2     0x6C
#define MPU6050_REG_WHO_AM_I       0x75

/* Sensitivity scale factors */
#define MPU6050_ACCEL_SCALE_2G     16384.0f  /* LSB/g for +/- 2g range */
#define MPU6050_GYRO_SCALE_500     65.5f     /* LSB/(deg/s) for +/- 500 deg/s range */
#define MPU6050_TEMP_SCALE         340.0f    /* LSB/deg C */
#define MPU6050_TEMP_OFFSET        36.53f    /* deg C offset */

/* Yaw deadband to prevent stationary drift */
#define MPU6050_GYRO_DEADBAND_DPS  0.15f     /* Ignore noise below 0.15 deg/s */

/* Static Helper Functions */
static bool MPU6050_WriteReg(mpu6050_t *dev, uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr_8bit, reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK);
}

static bool MPU6050_ReadReg(mpu6050_t *dev, uint8_t reg, uint8_t *val)
{
    return (HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr_8bit, reg,
                             I2C_MEMADD_SIZE_8BIT, val, 1, 100) == HAL_OK);
}

bool MPU6050_Init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (!dev || !hi2c) {
        return false;
    }

    memset(dev, 0, sizeof(mpu6050_t));
    dev->hi2c = hi2c;
    dev->dev_addr_8bit = MPU6050_I2C_ADDR_DEFAULT;

    /* Detect device on default address (0x68 << 1 = 0xD0) */
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->dev_addr_8bit, 3, 50) != HAL_OK) {
        /* Try alternate address if AD0 is tied to VCC (0x69 << 1 = 0xD2) */
        dev->dev_addr_8bit = MPU6050_I2C_ADDR_ALT;
        if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->dev_addr_8bit, 3, 50) != HAL_OK) {
            return false;
        }
    }

    /* Verify WHO_AM_I register */
    uint8_t who_am_i = 0;
    if (!MPU6050_ReadReg(dev, MPU6050_REG_WHO_AM_I, &who_am_i) ||
        who_am_i != MPU6050_WHO_AM_I_VAL) {
        return false;
    }

    /* Device reset */
    if (!MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x80)) {
        return false;
    }
    HAL_Delay(100);

    /* Wake up device and select PLL with X axis gyroscope reference (0x01) */
    if (!MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x01)) {
        return false;
    }
    HAL_Delay(10);

    /* Enable all accelerometer and gyroscope axes */
    if (!MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_2, 0x00)) {
        return false;
    }

    /* Set sample rate divider: 1 kHz / (1 + 7) = 125 Hz sample rate */
    if (!MPU6050_WriteReg(dev, MPU6050_REG_SMPLRT_DIV, 0x07)) {
        return false;
    }

    /* Configure DLPF: 42 Hz bandwidth to filter out robot chassis vibrations */
    if (!MPU6050_WriteReg(dev, MPU6050_REG_CONFIG, 0x03)) {
        return false;
    }

    /* Configure Gyroscope: +/- 500 deg/s full scale (FS_SEL = 1, 0x08) */
    if (!MPU6050_WriteReg(dev, MPU6050_REG_GYRO_CONFIG, 0x08)) {
        return false;
    }

    /* Configure Accelerometer: +/- 2g full scale (AFS_SEL = 0, 0x00) */
    if (!MPU6050_WriteReg(dev, MPU6050_REG_ACCEL_CONFIG, 0x00)) {
        return false;
    }

    dev->is_initialized = true;
    return true;
}

bool MPU6050_Calibrate(mpu6050_t *dev, uint16_t samples)
{
    if (!dev || !dev->is_initialized || samples == 0) {
        return false;
    }

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;

    for (uint16_t i = 0; i < samples; i++) {
        if (!MPU6050_ReadRaw(dev)) {
            return false;
        }
        sum_gx += dev->gyro_x;
        sum_gy += dev->gyro_y;
        sum_gz += dev->gyro_z;
        HAL_Delay(5);
    }

    dev->gyro_x_offset = sum_gx / (float)samples;
    dev->gyro_y_offset = sum_gy / (float)samples;
    dev->gyro_z_offset = sum_gz / (float)samples;

    dev->yaw = 0.0f;
    return true;
}

bool MPU6050_ReadRaw(mpu6050_t *dev)
{
    if (!dev || !dev->is_initialized) {
        return false;
    }

    uint8_t buf[14];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr_8bit,
                                                MPU6050_REG_ACCEL_XOUT_H,
                                                I2C_MEMADD_SIZE_8BIT,
                                                buf, 14, 100);
    if (status != HAL_OK) {
        return false;
    }

    /* Combine high and low register bytes */
    dev->accel_x_raw = (int16_t)((buf[0] << 8) | buf[1]);
    dev->accel_y_raw = (int16_t)((buf[2] << 8) | buf[3]);
    dev->accel_z_raw = (int16_t)((buf[4] << 8) | buf[5]);
    dev->temp_raw    = (int16_t)((buf[6] << 8) | buf[7]);
    dev->gyro_x_raw  = (int16_t)((buf[8] << 8) | buf[9]);
    dev->gyro_y_raw  = (int16_t)((buf[10] << 8) | buf[11]);
    dev->gyro_z_raw  = (int16_t)((buf[12] << 8) | buf[13]);

    /* Convert to engineering units */
    dev->accel_x = (float)dev->accel_x_raw / MPU6050_ACCEL_SCALE_2G;
    dev->accel_y = (float)dev->accel_y_raw / MPU6050_ACCEL_SCALE_2G;
    dev->accel_z = (float)dev->accel_z_raw / MPU6050_ACCEL_SCALE_2G;

    dev->temperature = ((float)dev->temp_raw / MPU6050_TEMP_SCALE) + MPU6050_TEMP_OFFSET;

    dev->gyro_x = (float)dev->gyro_x_raw / MPU6050_GYRO_SCALE_500;
    dev->gyro_y = (float)dev->gyro_y_raw / MPU6050_GYRO_SCALE_500;
    dev->gyro_z = (float)dev->gyro_z_raw / MPU6050_GYRO_SCALE_500;

    return true;
}

bool MPU6050_Update(mpu6050_t *dev, float dt_seconds)
{
    if (!MPU6050_ReadRaw(dev)) {
        return false;
    }

    /* Apply zero-rate offset correction on Z axis */
    float gz_rate = dev->gyro_z - dev->gyro_z_offset;

    /* Deadband filter to prevent stationary drift */
    if (fabsf(gz_rate) < MPU6050_GYRO_DEADBAND_DPS) {
        gz_rate = 0.0f;
    }

    /* Integrate angular rate into heading */
    dev->yaw += gz_rate * dt_seconds;

    return true;
}

void MPU6050_ResetYaw(mpu6050_t *dev)
{
    if (dev) {
        dev->yaw = 0.0f;
    }
}
