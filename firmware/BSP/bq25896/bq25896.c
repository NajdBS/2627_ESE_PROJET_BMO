/**
  ******************************************************************************
  * @file    bq25896.c
  * @brief   Implementation of BQ25896 / BQ25895 Fast Charger & 5V Boost Driver
  ******************************************************************************
  */

#include "bq25896.h"
#include <string.h>

#define BQ25896_I2C_TIMEOUT_MS  100

static HAL_StatusTypeDef BQ25896_ReadReg(bq25896_t *dev, uint8_t reg, uint8_t *val)
{
    if (!dev || !dev->hi2c || !val) {
        return HAL_ERROR;
    }
    return HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, val, 1, BQ25896_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef BQ25896_WriteReg(bq25896_t *dev, uint8_t reg, uint8_t val)
{
    if (!dev || !dev->hi2c) {
        return HAL_ERROR;
    }
    return HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, BQ25896_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef BQ25896_Init(bq25896_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (!dev || !hi2c) {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(bq25896_t));
    dev->hi2c = hi2c;

    /* Auto-probe address: check 0x6B (BQ25896) then 0x6A (BQ25895) */
    if (HAL_I2C_IsDeviceReady(dev->hi2c, BQ25896_I2C_ADDR, 2, BQ25896_I2C_TIMEOUT_MS) == HAL_OK) {
        dev->i2c_addr = BQ25896_I2C_ADDR;
    } else if (HAL_I2C_IsDeviceReady(dev->hi2c, BQ25895_I2C_ADDR, 2, BQ25896_I2C_TIMEOUT_MS) == HAL_OK) {
        dev->i2c_addr = BQ25895_I2C_ADDR;
    } else {
        dev->initialized = false;
        return HAL_ERROR;
    }

    /* Disable internal watchdog by default so IC registers do not reset during development */
    BQ25896_DisableWatchdog(dev);

    /* Enable continuous 1s ADC conversions (REG02: CONV_RATE = 1) */
    uint8_t reg02 = 0;
    if (BQ25896_ReadReg(dev, BQ25896_REG02, &reg02) == HAL_OK) {
        reg02 |= (1 << 6); /* CONV_RATE = 1 */
        BQ25896_WriteReg(dev, BQ25896_REG02, reg02);
    }

    dev->initialized = true;
    (void)BQ25896_Update(dev);

    return HAL_OK;
}

HAL_StatusTypeDef BQ25896_SetBoostMode(bq25896_t *dev, bool enable)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    uint8_t reg03 = 0;
    if (BQ25896_ReadReg(dev, BQ25896_REG03, &reg03) != HAL_OK) {
        return HAL_ERROR;
    }

    if (enable) {
        reg03 |= (1 << 5); /* OTG_CONFIG = 1 (Boost mode enable) */
    } else {
        reg03 &= ~(1 << 5); /* OTG_CONFIG = 0 (Boost mode disable) */
    }

    HAL_StatusTypeDef status = BQ25896_WriteReg(dev, BQ25896_REG03, reg03);
    if (status == HAL_OK) {
        dev->boost_enabled = enable;
    }
    return status;
}

HAL_StatusTypeDef BQ25896_SetCharging(bq25896_t *dev, bool enable)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    uint8_t reg03 = 0;
    if (BQ25896_ReadReg(dev, BQ25896_REG03, &reg03) != HAL_OK) {
        return HAL_ERROR;
    }

    if (enable) {
        reg03 |= (1 << 4); /* CHG_CONFIG = 1 (Charge enable) */
    } else {
        reg03 &= ~(1 << 4); /* CHG_CONFIG = 0 (Charge disable) */
    }

    return BQ25896_WriteReg(dev, BQ25896_REG03, reg03);
}

HAL_StatusTypeDef BQ25896_DisableWatchdog(bq25896_t *dev)
{
    if (!dev) {
        return HAL_ERROR;
    }

    uint8_t reg07 = 0;
    if (BQ25896_ReadReg(dev, BQ25896_REG07, &reg07) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Bits 5:4 WATCHDOG: 00 = Disable */
    reg07 &= ~((1 << 5) | (1 << 4));
    return BQ25896_WriteReg(dev, BQ25896_REG07, reg07);
}

HAL_StatusTypeDef BQ25896_ResetWatchdog(bq25896_t *dev)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    uint8_t reg03 = 0;
    if (BQ25896_ReadReg(dev, BQ25896_REG03, &reg03) != HAL_OK) {
        return HAL_ERROR;
    }

    reg03 |= (1 << 6); /* WD_RST = 1 */
    return BQ25896_WriteReg(dev, BQ25896_REG03, reg03);
}

HAL_StatusTypeDef BQ25896_SetInputCurrentLimit(bq25896_t *dev, uint16_t current_ma)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    if (current_ma < 100) current_ma = 100;
    if (current_ma > 3250) current_ma = 3250;

    uint8_t val = (uint8_t)((current_ma - 100) / 50);

    uint8_t reg00 = 0;
    if (BQ25896_ReadReg(dev, BQ25896_REG00, &reg00) != HAL_OK) {
        return HAL_ERROR;
    }

    reg00 = (reg00 & 0xC0) | (val & 0x3F);
    return BQ25896_WriteReg(dev, BQ25896_REG00, reg00);
}

HAL_StatusTypeDef BQ25896_Update(bq25896_t *dev)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    uint8_t reg_b = 0, reg_e = 0, reg_11 = 0, reg_12 = 0;

    if (BQ25896_ReadReg(dev, BQ25896_REG0B, &reg_b) == HAL_OK) {
        dev->vbus_status   = (bq_vbus_status_t)((reg_b >> 5) & 0x07);
        dev->charge_status = (bq_charge_status_t)((reg_b >> 3) & 0x03);
        dev->power_good    = ((reg_b >> 2) & 0x01) ? true : false;
    }

    /* Battery Voltage ADC (REG0E): Offset 2304 mV, Step 20 mV */
    if (BQ25896_ReadReg(dev, BQ25896_REG0E, &reg_e) == HAL_OK) {
        dev->vbat_mv = 2304 + ((reg_e & 0x7F) * 20);
    }

    /* VBUS Voltage ADC (REG11): Offset 2600 mV, Step 100 mV */
    if (BQ25896_ReadReg(dev, BQ25896_REG11, &reg_11) == HAL_OK) {
        dev->vbus_mv = 2600 + ((reg_11 & 0x7F) * 100);
    }

    /* Charge Current ADC (REG12): Offset 0 mA, Step 50 mA */
    if (BQ25896_ReadReg(dev, BQ25896_REG12, &reg_12) == HAL_OK) {
        dev->ichg_ma = (reg_12 & 0x7F) * 50;
    }

    return HAL_OK;
}
