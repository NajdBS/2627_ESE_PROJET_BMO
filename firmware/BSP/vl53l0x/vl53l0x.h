/**
  ******************************************************************************
  * @file    vl53l0x.h
  * @brief   VL53L0X Time-of-Flight distance sensor driver for STM32 HAL
  ******************************************************************************
  */

#ifndef BSP_VL53L0X_H
#define BSP_VL53L0X_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define VL53L0X_DEFAULT_ADDRESS_8BIT  0x52
#define VL53L0X_DEFAULT_ADDRESS_7BIT  0x29
#define VL53L0X_ADDR_LEFT_7BIT        0x30
#define VL53L0X_ADDR_LEFT_8BIT        (0x30 << 1)  /* 0x60 */
#define VL53L0X_ADDR_RIGHT_7BIT       0x31
#define VL53L0X_ADDR_RIGHT_8BIT       (0x31 << 1)  /* 0x62 */
#define VL53L0X_ADDR_CENTER_7BIT      0x32
#define VL53L0X_ADDR_CENTER_8BIT      (0x32 << 1)  /* 0x64 */
#define VL53L0X_DEFAULT_TIMEOUT_MS    500

/**
  * @brief VCSEL pulse period type
  */
typedef enum {
    VL53L0X_VCSEL_PERIOD_PRE_RANGE = 0,
    VL53L0X_VCSEL_PERIOD_FINAL_RANGE
} vl53l0x_vcsel_period_type_t;

/**
  * @brief Extended measurement statistics (optional)
  */
typedef struct {
    uint16_t raw_distance_mm; /* Raw distance measured in mm */
    uint16_t signal_cnt;      /* Signal return count rate */
    uint16_t ambient_cnt;     /* Ambient count rate */
    uint16_t spad_cnt;        /* Number of enabled SPADs */
    uint8_t  range_status;    /* Range status (0 = range valid) */
} vl53l0x_stats_t;

/**
  * @brief VL53L0X Sensor Instance Handle
  */
typedef struct {
    I2C_HandleTypeDef *hi2c;                         /* I2C peripheral handle */
    uint8_t            dev_addr_8bit;                /* 8-bit I2C device address */
    uint16_t           io_timeout_ms;                /* Timeout in milliseconds */
    bool               is_timeout;                   /* True if last operation timed out */
    uint32_t           timeout_start_ms;             /* Timeout reference timestamp */
    uint8_t            stop_variable;                /* Internal calibration stop variable */
    uint32_t           measurement_timing_budget_us; /* Timing budget in microseconds */
} vl53l0x_t;

/**
  * @brief  Initialize the VL53L0X sensor and perform SPAD & reference calibrations
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  hi2c: I2C handle (e.g. &hi2c3)
  * @retval true on success, false if sensor not responding or ID invalid
  */
bool VL53L0X_Init(vl53l0x_t *dev, I2C_HandleTypeDef *hi2c);

/**
  * @brief  Reassign the sensor I2C address (useful for multi-sensor setups via XSHUT)
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  new_8bit_addr: New 8-bit I2C address (e.g. 0x54)
  */
void VL53L0X_SetAddress(vl53l0x_t *dev, uint8_t new_8bit_addr);

/**
  * @brief  Get current 8-bit I2C address
  * @param  dev: Pointer to vl53l0x_t instance
  * @retval Current 8-bit I2C address
  */
uint8_t VL53L0X_GetAddress(const vl53l0x_t *dev);

/**
  * @brief  Set return signal rate limit in MCPS (Mega Counts Per Second)
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  limit_mcps: Minimum signal rate limit (e.g. 0.25f)
  * @retval true on success, false if parameter out of range
  */
bool VL53L0X_SetSignalRateLimit(vl53l0x_t *dev, float limit_mcps);

/**
  * @brief  Get current signal rate limit
  * @param  dev: Pointer to vl53l0x_t instance
  * @retval Limit in MCPS
  */
float VL53L0X_GetSignalRateLimit(vl53l0x_t *dev);

/**
  * @brief  Set measurement timing budget in microseconds (min 20000 us = 20 ms)
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  budget_us: Timing budget in us (e.g. 33000 us)
  * @retval true on success, false if budget too short
  */
bool VL53L0X_SetMeasurementTimingBudget(vl53l0x_t *dev, uint32_t budget_us);

/**
  * @brief  Get current measurement timing budget in microseconds
  * @param  dev: Pointer to vl53l0x_t instance
  * @retval Timing budget in us
  */
uint32_t VL53L0X_GetMeasurementTimingBudget(vl53l0x_t *dev);

/**
  * @brief  Set VCSEL pulse period
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  type: Pre-range or Final-range period type
  * @param  period_pclks: Pulse period in PCLKs
  * @retval true on success, false if invalid
  */
bool VL53L0X_SetVcselPulsePeriod(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type, uint8_t period_pclks);

/**
  * @brief  Get current VCSEL pulse period
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  type: Pre-range or Final-range period type
  * @retval Pulse period in PCLKs
  */
uint8_t VL53L0X_GetVcselPulsePeriod(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type);

/**
  * @brief  Start continuous distance measurement mode
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  period_ms: Inter-measurement period in ms (0 = back-to-back continuous)
  */
void VL53L0X_StartContinuous(vl53l0x_t *dev, uint32_t period_ms);

/**
  * @brief  Stop continuous measurement mode
  * @param  dev: Pointer to vl53l0x_t instance
  */
void VL53L0X_StopContinuous(vl53l0x_t *dev);

/**
  * @brief  Read latest measured distance in continuous mode
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  extra_stats: Optional pointer to statistics struct, or NULL
  * @retval Distance in mm, or 65535 on timeout / out-of-range
  */
uint16_t VL53L0X_ReadDistanceContinuous(vl53l0x_t *dev, vl53l0x_stats_t *extra_stats);

/**
  * @brief  Perform a single-shot distance measurement
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  extra_stats: Optional pointer to statistics struct, or NULL
  * @retval Distance in mm, or 65535 on timeout / out-of-range
  */
uint16_t VL53L0X_ReadDistanceSingle(vl53l0x_t *dev, vl53l0x_stats_t *extra_stats);

/**
  * @brief  Set timeout in milliseconds for I/O operations
  * @param  dev: Pointer to vl53l0x_t instance
  * @param  timeout_ms: Timeout in ms (e.g. 500)
  */
void VL53L0X_SetTimeout(vl53l0x_t *dev, uint16_t timeout_ms);

/**
  * @brief  Get current timeout in milliseconds
  * @param  dev: Pointer to vl53l0x_t instance
  * @retval Timeout in ms
  */
uint16_t VL53L0X_GetTimeout(const vl53l0x_t *dev);

/**
  * @brief  Check and clear timeout status flag
  * @param  dev: Pointer to vl53l0x_t instance
  * @retval true if timeout occurred since last check
  */
bool VL53L0X_TimeoutOccurred(vl53l0x_t *dev);

#endif /* BSP_VL53L0X_H */
