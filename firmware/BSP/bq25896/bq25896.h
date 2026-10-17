/**
  ******************************************************************************
  * @file    bq25896.h
  * @brief   Driver for Texas Instruments BQ25896 / BQ25895 Fast Charger & 5V Boost
  ******************************************************************************
  */

#ifndef BQ25896_H
#define BQ25896_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>


/**
  * @brief BQ25896 and BQ25895 I2C 7-bit addresses
  */
#define BQ25896_I2C_ADDR_7BIT       0x6B
#define BQ25895_I2C_ADDR_7BIT       0x6A

#define BQ25896_I2C_ADDR            (BQ25896_I2C_ADDR_7BIT << 1) /* 0xD6 */
#define BQ25895_I2C_ADDR            (BQ25895_I2C_ADDR_7BIT << 1) /* 0xD4 */

/**
  * @brief BQ25896 Register Map
  */
#define BQ25896_REG00               0x00 /* Input Current Limit */
#define BQ25896_REG01               0x01 /* Thermal / VINDPM Offset */
#define BQ25896_REG02               0x02 /* ADC / Boost Frequency */
#define BQ25896_REG03               0x03 /* SYS Min / Charge / Boost OTG Config */
#define BQ25896_REG04               0x04 /* Fast Charge Current Limit */
#define BQ25896_REG05               0x05 /* Precharge / Termination Current */
#define BQ25896_REG06               0x06 /* Charge Voltage Limit */
#define BQ25896_REG07               0x07 /* Timer / Watchdog Setting */
#define BQ25896_REG08               0x08 /* IR Compensation / Thermal Regulation */
#define BQ25896_REG09               0x09 /* BATFET / Optimizer Configuration */
#define BQ25896_REG0A               0x0A /* Boost Voltage & Current Regulation */
#define BQ25896_REG0B               0x0B /* System Status */
#define BQ25896_REG0C               0x0C /* Fault Register */
#define BQ25896_REG0D               0x0D /* VINDPM Threshold */
#define BQ25896_REG0E               0x0E /* Battery Voltage ADC */
#define BQ25896_REG0F               0x0F /* System Voltage ADC */
#define BQ25896_REG10               0x10 /* TS ADC */
#define BQ25896_REG11               0x11 /* VBUS Voltage ADC */
#define BQ25896_REG12               0x12 /* Charge Current ADC */
#define BQ25896_REG14               0x14 /* Device Revision / Part Number */

/**
  * @brief Configuration Constants for Samsung INR18650-35E (3400 mAh)
  */
#define BQ25896_CFG_REG02_ADC_1HZ   0x5D /* CONV_RATE=1 (1s continuous ADC), BOOST_FREQ=1.5MHz */
#define BQ25896_CFG_REG03_DEFAULT   0x5A /* WD_RST=1, OTG_CONFIG=0, CHG_CONFIG=1, SYS_MIN=3.5V */
#define BQ25896_CFG_REG04_ICHG      0x1A /* ICHG = 1664 mA (26 * 64 mA, ~0.49C) */
#define BQ25896_CFG_REG05_PRE_TERM  0x52 /* IPRECHG = 384 mA, ITERM = 192 mA */
#define BQ25896_CFG_REG06_VREG      0x5A /* VREG = 4.192 V, BATLOWV = 3.0 V, VRECHG = 100 mV */
#define BQ25896_CFG_REG09_SHIP_DLY  0x6C /* TMR2X_EN=1, BATFET_DIS=1, BATFET_DLY=1 (10-15s), BATFET_RST_EN=1 */
#define BQ25896_CFG_REG0A_BOOST     0x72 /* BOOSTV = 4.998 V, BOOST_LIM = 1.2 A */

/**
  * @brief Status Bit Definitions
  */
typedef enum {
    BQ_VBUS_NONE    = 0,
    BQ_VBUS_USB_SDP = 1,
    BQ_VBUS_ADAPTER = 2,
    BQ_VBUS_OTG     = 7
} bq_vbus_status_t;

typedef enum {
    BQ_CHRG_NOT_CHARGING = 0,
    BQ_CHRG_PRECHARGE    = 1,
    BQ_CHRG_FAST_CHARGE  = 2,
    BQ_CHRG_DONE         = 3
} bq_charge_status_t;

/**
  * @brief BQ25896 Device Handle
  */
typedef struct {
    I2C_HandleTypeDef  *hi2c;
    uint8_t             i2c_addr;
    uint8_t             dev_rev;
    bq_vbus_status_t    vbus_status;
    bq_charge_status_t  charge_status;
    bool                power_good;
    uint8_t             fault_latched;
    uint8_t             fault_current;
    uint16_t            vbat_mv;
    uint16_t            vsys_mv;
    uint16_t            vbus_mv;
    uint16_t            ichg_ma;
    bool                boost_enabled;
    bool                charging_enabled;
    bool                initialized;
} bq25896_t;

HAL_StatusTypeDef BQ25896_Init(bq25896_t *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BQ25896_SetBoostMode(bq25896_t *dev, bool enable);
HAL_StatusTypeDef BQ25896_SetCharging(bq25896_t *dev, bool enable);
HAL_StatusTypeDef BQ25896_DisableWatchdog(bq25896_t *dev);
HAL_StatusTypeDef BQ25896_ResetWatchdog(bq25896_t *dev);
HAL_StatusTypeDef BQ25896_SetInputCurrentLimit(bq25896_t *dev, uint16_t current_ma);
HAL_StatusTypeDef BQ25896_EnterShipMode(bq25896_t *dev);
HAL_StatusTypeDef BQ25896_Update(bq25896_t *dev);


#endif /* BQ25896_H */
