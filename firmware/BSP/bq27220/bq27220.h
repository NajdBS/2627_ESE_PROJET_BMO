/**
  ******************************************************************************
  * @file    bq27220.h
  * @brief   Driver for Texas Instruments BQ27220 Single-Cell Fuel Gauge
  ******************************************************************************
  */

#ifndef BQ27220_H
#define BQ27220_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief BQ27220 7-bit and 8-bit I2C addresses
  */
#define BQ27220_I2C_ADDR_7BIT       0x55
#define BQ27220_I2C_ADDR            (BQ27220_I2C_ADDR_7BIT << 1) /* 0xAA */

/**
  * @brief Standard 16-bit command register addresses (LSB register index)
  */
#define BQ27220_CMD_CONTROL         0x00
#define BQ27220_CMD_AT_RATE         0x02
#define BQ27220_CMD_AR_TTE          0x04
#define BQ27220_CMD_TEMP            0x06
#define BQ27220_CMD_VOLTAGE         0x08
#define BQ27220_CMD_BAT_STATUS      0x0A
#define BQ27220_CMD_CURRENT         0x0C
#define BQ27220_CMD_REMCAP          0x10
#define BQ27220_CMD_FCC             0x12
#define BQ27220_CMD_AVG_CURRENT     0x14
#define BQ27220_CMD_TTE             0x16
#define BQ27220_CMD_TTF             0x18
#define BQ27220_CMD_CYCLE_COUNT     0x2A
#define BQ27220_CMD_SOC             0x2C
#define BQ27220_CMD_SOH             0x2E
#define BQ27220_CMD_DESIGN_CAP      0x3C

/**
  * @brief Control Subcommands
  */
#define BQ27220_SUBCMD_STATUS       0x0000
#define BQ27220_SUBCMD_DEV_NUM      0x0001
#define BQ27220_SUBCMD_FW_VER       0x0002

/**
  * @brief BQ27220 Device Handle
  */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t           voltage_mv;
    int16_t            current_ma;
    uint16_t           soc_percent;
    uint16_t           remaining_capacity_mah;
    uint16_t           full_capacity_mah;
    float              temperature_c;
    bool               initialized;
} bq27220_t;

/**
  * @brief  Initialize the BQ27220 Fuel Gauge.
  * @param  dev: Pointer to BQ27220 structure handle.
  * @param  hi2c: Pointer to STM32 I2C hardware handle.
  * @retval HAL_OK on successful communication and detection.
  */
HAL_StatusTypeDef BQ27220_Init(bq27220_t *dev, I2C_HandleTypeDef *hi2c);

/**
  * @brief  Read the cell voltage in millivolts.
  * @param  dev: Pointer to BQ27220 structure handle.
  * @param  voltage_mv: Pointer to store measured voltage (mV).
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ27220_ReadVoltage(bq27220_t *dev, uint16_t *voltage_mv);

/**
  * @brief  Read the instantaneous battery current in milliamperes.
  *         Positive indicates charging, negative indicates discharging.
  * @param  dev: Pointer to BQ27220 structure handle.
  * @param  current_ma: Pointer to store measured current (mA).
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ27220_ReadCurrent(bq27220_t *dev, int16_t *current_ma);

/**
  * @brief  Read the State-of-Charge (SOC) in percent (0 - 100%).
  * @param  dev: Pointer to BQ27220 structure handle.
  * @param  soc_pct: Pointer to store battery SOC (%).
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ27220_ReadSOC(bq27220_t *dev, uint16_t *soc_pct);

/**
  * @brief  Read the remaining battery capacity in mAh.
  * @param  dev: Pointer to BQ27220 structure handle.
  * @param  rem_cap_mah: Pointer to store remaining capacity (mAh).
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ27220_ReadRemainingCapacity(bq27220_t *dev, uint16_t *rem_cap_mah);

/**
  * @brief  Read the battery temperature in degrees Celsius.
  * @param  dev: Pointer to BQ27220 structure handle.
  * @param  temp_c: Pointer to store temperature in Celsius.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ27220_ReadTemperature(bq27220_t *dev, float *temp_c);

/**
  * @brief  Update all battery telemetry parameters at once.
  * @param  dev: Pointer to BQ27220 structure handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ27220_Update(bq27220_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BQ27220_H */
