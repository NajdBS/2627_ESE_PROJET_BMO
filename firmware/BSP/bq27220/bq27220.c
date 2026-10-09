/**
  ******************************************************************************
  * @file    bq27220.c
  * @brief   Implementation of BQ27220 Single-Cell Fuel Gauge Driver
  ******************************************************************************
  */

#include "bq27220.h"
#include <string.h>

#define BQ27220_I2C_TIMEOUT_MS  100

/**
  * @brief  Helper to read a 16-bit word from BQ27220 (little-endian standard).
  */
static HAL_StatusTypeDef BQ27220_ReadWord(bq27220_t *dev, uint8_t cmd, uint16_t *value)
{
    if (!dev || !dev->hi2c || !value) {
        return HAL_ERROR;
    }

    uint8_t buf[2] = {0};
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c,
                                                BQ27220_I2C_ADDR,
                                                cmd,
                                                I2C_MEMADD_SIZE_8BIT,
                                                buf,
                                                2,
                                                BQ27220_I2C_TIMEOUT_MS);

    if (status == HAL_OK) {
        *value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }

    return status;
}

HAL_StatusTypeDef BQ27220_Init(bq27220_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (!dev || !hi2c) {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(bq27220_t));
    dev->hi2c = hi2c;

    /* Verify device presence on the I2C bus */
    if (HAL_I2C_IsDeviceReady(dev->hi2c, BQ27220_I2C_ADDR, 3, BQ27220_I2C_TIMEOUT_MS) != HAL_OK) {
        dev->initialized = false;
        return HAL_ERROR;
    }

    /* Test reading voltage to ensure communication is operational */
    uint16_t test_v = 0;
    if (BQ27220_ReadVoltage(dev, &test_v) != HAL_OK) {
        dev->initialized = false;
        return HAL_ERROR;
    }

    dev->initialized = true;
    (void)BQ27220_Update(dev);

    return HAL_OK;
}

HAL_StatusTypeDef BQ27220_ReadVoltage(bq27220_t *dev, uint16_t *voltage_mv)
{
    HAL_StatusTypeDef status = BQ27220_ReadWord(dev, BQ27220_CMD_VOLTAGE, voltage_mv);
    if (status == HAL_OK && dev) {
        dev->voltage_mv = *voltage_mv;
    }
    return status;
}

HAL_StatusTypeDef BQ27220_ReadCurrent(bq27220_t *dev, int16_t *current_ma)
{
    uint16_t raw_current = 0;
    HAL_StatusTypeDef status = BQ27220_ReadWord(dev, BQ27220_CMD_CURRENT, &raw_current);
    if (status == HAL_OK) {
        *current_ma = (int16_t)raw_current;
        if (dev) {
            dev->current_ma = *current_ma;
        }
    }
    return status;
}

HAL_StatusTypeDef BQ27220_ReadSOC(bq27220_t *dev, uint16_t *soc_pct)
{
    HAL_StatusTypeDef status = BQ27220_ReadWord(dev, BQ27220_CMD_SOC, soc_pct);
    if (status == HAL_OK && dev) {
        dev->soc_percent = *soc_pct;
    }
    return status;
}

HAL_StatusTypeDef BQ27220_ReadRemainingCapacity(bq27220_t *dev, uint16_t *rem_cap_mah)
{
    HAL_StatusTypeDef status = BQ27220_ReadWord(dev, BQ27220_CMD_REMCAP, rem_cap_mah);
    if (status == HAL_OK && dev) {
        dev->remaining_capacity_mah = *rem_cap_mah;
    }
    return status;
}

HAL_StatusTypeDef BQ27220_ReadTemperature(bq27220_t *dev, float *temp_c)
{
    uint16_t raw_temp = 0;
    HAL_StatusTypeDef status = BQ27220_ReadWord(dev, BQ27220_CMD_TEMP, &raw_temp);
    if (status == HAL_OK) {
        /* Raw temperature is reported in 0.1 deg K */
        *temp_c = ((float)raw_temp / 10.0f) - 273.15f;
        if (dev) {
            dev->temperature_c = *temp_c;
        }
    }
    return status;
}

HAL_StatusTypeDef BQ27220_Update(bq27220_t *dev)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    uint16_t val16 = 0;
    int16_t  sval16 = 0;
    float    temp = 0.0f;

    BQ27220_ReadVoltage(dev, &val16);
    BQ27220_ReadCurrent(dev, &sval16);
    BQ27220_ReadSOC(dev, &val16);
    BQ27220_ReadRemainingCapacity(dev, &val16);
    BQ27220_ReadWord(dev, BQ27220_CMD_FCC, &dev->full_capacity_mah);
    BQ27220_ReadTemperature(dev, &temp);

    return HAL_OK;
}
