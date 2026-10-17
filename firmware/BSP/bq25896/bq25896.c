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

/* Helper to build REG03 explicitly without unsafe read-modify-write on WD_RST */
static uint8_t BQ25896_BuildReg03(const bq25896_t *dev, bool reset_wd)
{
    uint8_t reg03 = 0x1A; /* SYS_MIN = 3.5V (101), MIN_VBAT_SEL = 0 */
    if (reset_wd) {
        reg03 |= (1 << 6); /* WD_RST = 1 (auto-clears in hardware) */
    }
    if (dev->boost_enabled) {
        reg03 |= (1 << 5); /* OTG_CONFIG = 1 */
    }
    if (dev->charging_enabled) {
        reg03 |= (1 << 4); /* CHG_CONFIG = 1 */
    }
    return reg03;
}

HAL_StatusTypeDef BQ25896_Init(bq25896_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (!dev || !hi2c) {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(bq25896_t));
    dev->hi2c = hi2c;

    /* 1. Probe address: check 0x6B (BQ25896) then 0x6A (BQ25895) */
    if (HAL_I2C_IsDeviceReady(dev->hi2c, BQ25896_I2C_ADDR, 2, BQ25896_I2C_TIMEOUT_MS) == HAL_OK) {
        dev->i2c_addr = BQ25896_I2C_ADDR;
    } else if (HAL_I2C_IsDeviceReady(dev->hi2c, BQ25895_I2C_ADDR, 2, BQ25896_I2C_TIMEOUT_MS) == HAL_OK) {
        dev->i2c_addr = BQ25895_I2C_ADDR;
    } else {
        dev->initialized = false;
        return HAL_ERROR;
    }

    /* 2. Read device identity (REG14): PN[2:0] (bits 5:3) & DEV_REV[1:0] (bits 1:0) */
    if (BQ25896_ReadReg(dev, BQ25896_REG14, &dev->dev_rev) != HAL_OK) {
        dev->initialized = false;
        return HAL_ERROR;
    }

    /* 3. Repeatable full configuration for Samsung INR18650-35E (3400 mAh) */
    dev->charging_enabled = true;
    dev->boost_enabled = false;

    /* REG02: Continuous 1s ADC conversion, BOOST_FREQ = 1.5 MHz (no RMW on CONV_START) */
    (void)BQ25896_WriteReg(dev, BQ25896_REG02, BQ25896_CFG_REG02_ADC_1HZ);

    /* REG03: CHG_CONFIG=1, OTG_CONFIG=0, SYS_MIN=3.5V, WD_RST=1 */
    (void)BQ25896_WriteReg(dev, BQ25896_REG03, BQ25896_CFG_REG03_DEFAULT);

    /* REG04: ICHG = 1664 mA (0x1A) */
    (void)BQ25896_WriteReg(dev, BQ25896_REG04, BQ25896_CFG_REG04_ICHG);

    /* REG05: IPRECHG = 384 mA, ITERM = 192 mA (0x52) */
    (void)BQ25896_WriteReg(dev, BQ25896_REG05, BQ25896_CFG_REG05_PRE_TERM);

    /* REG06: VREG = 4.192 V, BATLOWV = 3.0 V, VRECHG = 100 mV (0x5A) */
    (void)BQ25896_WriteReg(dev, BQ25896_REG06, BQ25896_CFG_REG06_VREG);

    /* REG0A: Boost preset 4.998 V, 1.2 A limit (0x72) */
    (void)BQ25896_WriteReg(dev, BQ25896_REG0A, BQ25896_CFG_REG0A_BOOST);

    dev->initialized = true;
    (void)BQ25896_Update(dev);

    return HAL_OK;
}

HAL_StatusTypeDef BQ25896_SetBoostMode(bq25896_t *dev, bool enable)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    if (enable) {
        /* Ensure Boost voltage (4.998V) and current limit (1.2A) are set before enabling OTG */
        (void)BQ25896_WriteReg(dev, BQ25896_REG0A, BQ25896_CFG_REG0A_BOOST);
    }

    dev->boost_enabled = enable;
    return BQ25896_WriteReg(dev, BQ25896_REG03, BQ25896_BuildReg03(dev, true));
}

HAL_StatusTypeDef BQ25896_SetCharging(bq25896_t *dev, bool enable)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    dev->charging_enabled = enable;
    return BQ25896_WriteReg(dev, BQ25896_REG03, BQ25896_BuildReg03(dev, false));
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

    /* Write explicit REG03 with WD_RST=1 (bit 6) */
    return BQ25896_WriteReg(dev, BQ25896_REG03, BQ25896_BuildReg03(dev, true));
}

HAL_StatusTypeDef BQ25896_SetInputCurrentLimit(bq25896_t *dev, uint16_t current_ma)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    if (current_ma < 100) current_ma = 100;
    if (current_ma > 3250) current_ma = 3250;

    uint8_t val = (uint8_t)((current_ma - 100) / 50);

    /* Explicit write to REG00: EN_HIZ=0, EN_ILIM=1 (0x40), IINLIM=val */
    uint8_t reg00 = 0x40 | (val & 0x3F);
    return BQ25896_WriteReg(dev, BQ25896_REG00, reg00);
}

HAL_StatusTypeDef BQ25896_EnterShipMode(bq25896_t *dev)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    /* Write BATFET_DLY=1 and BATFET_DIS=1 in the same transaction (0x6C) */
    return BQ25896_WriteReg(dev, BQ25896_REG09, BQ25896_CFG_REG09_SHIP_DLY);
}

HAL_StatusTypeDef BQ25896_Update(bq25896_t *dev)
{
    if (!dev || !dev->initialized) {
        return HAL_ERROR;
    }

    uint8_t reg_b = 0, reg_c1 = 0, reg_c2 = 0;
    uint8_t reg_e = 0, reg_f = 0, reg_11 = 0, reg_12 = 0;

    if (BQ25896_ReadReg(dev, BQ25896_REG0B, &reg_b) == HAL_OK) {
        dev->vbus_status   = (bq_vbus_status_t)((reg_b >> 5) & 0x07);
        dev->charge_status = (bq_charge_status_t)((reg_b >> 3) & 0x03);
        dev->power_good    = ((reg_b >> 2) & 0x01) ? true : false;
    }

    /* Double read of REG0C: 1st read returns latched faults and clears INT, 2nd read returns current state */
    if (BQ25896_ReadReg(dev, BQ25896_REG0C, &reg_c1) == HAL_OK &&
        BQ25896_ReadReg(dev, BQ25896_REG0C, &reg_c2) == HAL_OK) {
        dev->fault_latched = reg_c1;
        dev->fault_current = reg_c2;
    }

    /* Battery Voltage ADC (REG0E): Offset 2304 mV, Step 20 mV */
    if (BQ25896_ReadReg(dev, BQ25896_REG0E, &reg_e) == HAL_OK) {
        dev->vbat_mv = 2304 + ((reg_e & 0x7F) * 20);
    }

    /* System Voltage ADC (REG0F): Offset 2304 mV, Step 20 mV */
    if (BQ25896_ReadReg(dev, BQ25896_REG0F, &reg_f) == HAL_OK) {
        dev->vsys_mv = 2304 + ((reg_f & 0x7F) * 20);
    }

    /* VBUS Voltage ADC (REG11): Offset 2600 mV, Step 100 mV */
    if (BQ25896_ReadReg(dev, BQ25896_REG11, &reg_11) == HAL_OK) {
        if (reg_11 & 0x80) { /* VBUS_GD bit 7 */
            dev->vbus_mv = 2600 + ((reg_11 & 0x7F) * 100);
        } else {
            dev->vbus_mv = 0;
        }
    }

    /* Charge Current ADC (REG12): Offset 0 mA, Step 50 mA */
    if (BQ25896_ReadReg(dev, BQ25896_REG12, &reg_12) == HAL_OK) {
        dev->ichg_ma = (reg_12 & 0x7F) * 50;
    }

    return HAL_OK;
}
