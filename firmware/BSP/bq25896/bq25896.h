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

#ifdef __cplusplus
extern "C" {
#endif

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
    bq_vbus_status_t    vbus_status;
    bq_charge_status_t  charge_status;
    bool                power_good;
    uint16_t            vbat_mv;
    uint16_t            vbus_mv;
    uint16_t            ichg_ma;
    bool                boost_enabled;
    bool                initialized;
} bq25896_t;

/**
  * @brief  Initialize the BQ25896 / BQ25895 Charger IC.
  *         Auto-probes for address 0x6B (BQ25896) or 0x6A (BQ25895).
  * @param  dev: Pointer to device structure.
  * @param  hi2c: Pointer to STM32 I2C handle.
  * @retval HAL_OK on successful initialization.
  */
HAL_StatusTypeDef BQ25896_Init(bq25896_t *dev, I2C_HandleTypeDef *hi2c);

/**
  * @brief  Enable or disable USB OTG 5V Boost Mode (supplies 5V to LiDAR/motors).
  * @param  dev: Pointer to device structure.
  * @param  enable: true to enable 5V boost, false to disable.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ25896_SetBoostMode(bq25896_t *dev, bool enable);

/**
  * @brief  Enable or disable battery charging.
  * @param  dev: Pointer to device structure.
  * @param  enable: true to allow charging, false to inhibit.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ25896_SetCharging(bq25896_t *dev, bool enable);

/**
  * @brief  Disable internal I2C Watchdog to prevent unexpected register resets.
  * @param  dev: Pointer to device structure.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ25896_DisableWatchdog(bq25896_t *dev);

/**
  * @brief  Reset the internal I2C Watchdog timer.
  * @param  dev: Pointer to device structure.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ25896_ResetWatchdog(bq25896_t *dev);

/**
  * @brief  Set input current limit in mA (e.g. 500, 1500, 2000, 3000 mA).
  * @param  dev: Pointer to device structure.
  * @param  current_ma: Current limit in mA.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ25896_SetInputCurrentLimit(bq25896_t *dev, uint16_t current_ma);

/**
  * @brief  Poll and update telemetry status (VBUS, Charging state, ADC).
  * @param  dev: Pointer to device structure.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ25896_Update(bq25896_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BQ25896_H */
