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
#define BQ27220_CMD_OPERATION_STAT  0x3A
#define BQ27220_CMD_DESIGN_CAP      0x3C

/**
  * @brief BatteryStatus() (0x0A) Flag Bits (TRM Â§2.6 / Q5.7)
  */
#define BQ27220_STATUS_DSG          (1U << 0) /* Discharging */
#define BQ27220_STATUS_SYSDWN       (1U << 1) /* System Down threshold reached (3150 mV) */
#define BQ27220_STATUS_TDA          (1U << 2) /* Terminate Discharge Alarm (3200 mV / 6%) */
#define BQ27220_STATUS_FC           (1U << 9) /* Fully Charged */

/**
  * @brief Voltage Thresholds from Course TD2 (Q4.4)
  */
#define BQ27220_TERMINATE_DISCHARGE_MV  3200
#define BQ27220_SYSTEM_DOWN_MV          3150

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
    uint16_t           design_capacity_mah;
    uint16_t           battery_status;
    float              temperature_c;
    bool               terminate_discharge;
    bool               sys_down;
    bool               initialized;
} bq27220_t;

HAL_StatusTypeDef BQ27220_Init(bq27220_t *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BQ27220_ReadVoltage(bq27220_t *dev, uint16_t *voltage_mv);
HAL_StatusTypeDef BQ27220_ReadCurrent(bq27220_t *dev, int16_t *current_ma);
HAL_StatusTypeDef BQ27220_ReadSOC(bq27220_t *dev, uint16_t *soc_pct);
HAL_StatusTypeDef BQ27220_ReadRemainingCapacity(bq27220_t *dev, uint16_t *rem_cap_mah);
HAL_StatusTypeDef BQ27220_ReadTemperature(bq27220_t *dev, float *temp_c);
HAL_StatusTypeDef BQ27220_ReadBatteryStatus(bq27220_t *dev, uint16_t *status_flags);
HAL_StatusTypeDef BQ27220_Update(bq27220_t *dev);


#endif /* BQ27220_H */
