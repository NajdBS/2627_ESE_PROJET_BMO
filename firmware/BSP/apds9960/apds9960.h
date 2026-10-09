/**
  ******************************************************************************
  * @file    apds9960.h
  * @brief   APDS-9960 RGB Color and Proximity sensor driver for STM32 HAL
  ******************************************************************************
  */

#ifndef BSP_APDS9960_H
#define BSP_APDS9960_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ==============================================================================
 * I2C Device Address & Identification
 * ============================================================================== */
#define APDS9960_I2C_ADDR_7BIT          0x39
#define APDS9960_I2C_ADDR_8BIT          (0x39 << 1)  /**< 0x72 */
#define APDS9960_CHIP_ID                0xAB        /**< Expected value in ID reg (0x92) */

/* ==============================================================================
 * Key Register Addresses (per Avago APDS-9960 Datasheet)
 * ============================================================================== */
#define APDS9960_REG_ENABLE             0x80
#define APDS9960_REG_ATIME              0x81
#define APDS9960_REG_WTIME              0x83
#define APDS9960_REG_CONFIG1            0x8D
#define APDS9960_REG_PPULSE             0x8E
#define APDS9960_REG_CONTROL            0x8F
#define APDS9960_REG_CONFIG2            0x90
#define APDS9960_REG_ID                 0x92
#define APDS9960_REG_STATUS             0x93
#define APDS9960_REG_CDATAL             0x94
#define APDS9960_REG_CDATAH             0x95
#define APDS9960_REG_RDATAL             0x96
#define APDS9960_REG_RDATAH             0x97
#define APDS9960_REG_GDATAL             0x98
#define APDS9960_REG_GDATAH             0x99
#define APDS9960_REG_BDATAL             0x9A
#define APDS9960_REG_BDATAH             0x9B
#define APDS9960_REG_PDATA              0x9C
#define APDS9960_REG_CONFIG3            0x9F

/* ==============================================================================
 * Bit Definitions
 * ============================================================================== */
/* ENABLE register (0x80) */
#define APDS9960_ENABLE_PON             (1 << 0)    /**< Power ON */
#define APDS9960_ENABLE_AEN             (1 << 1)    /**< ALS / RGBC Color Enable */
#define APDS9960_ENABLE_PEN             (1 << 2)    /**< Proximity Detect Enable */
#define APDS9960_ENABLE_WEN             (1 << 3)    /**< Wait Timer Enable */

/* STATUS register (0x93) */
#define APDS9960_STATUS_AVALID          (1 << 0)    /**< ALS / RGBC Data Valid */
#define APDS9960_STATUS_PVALID          (1 << 1)    /**< Proximity Data Valid */
#define APDS9960_STATUS_CPSAT           (1 << 7)    /**< Clear photodiode saturation */

/* ==============================================================================
 * Types & Data Structures
 * ============================================================================== */

/**
 * @brief Classified can color detected by BMO
 */
typedef enum {
    BMO_CAN_COLOR_NONE = 0,     /**< No can detected or ambient dark */
    BMO_CAN_COLOR_RED,          /**< RED can detected */
    BMO_CAN_COLOR_GREEN,        /**< GREEN can detected */
    BMO_CAN_COLOR_BLUE,         /**< BLUE can detected */
    BMO_CAN_COLOR_UNKNOWN       /**< Object detected with uncertain color */
} bmo_can_color_t;

/**
 * @brief Raw measurement data structure
 */
typedef struct {
    uint16_t red;               /**< Red channel intensity (0..65535) */
    uint16_t green;             /**< Green channel intensity (0..65535) */
    uint16_t blue;              /**< Blue channel intensity (0..65535) */
    uint16_t clear;             /**< Clear channel intensity (0..65535) */
    uint8_t  proximity;         /**< Infrared proximity (0..255) */
} apds9960_data_t;

/**
 * @brief APDS-9960 Device Handle
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr_8bit;
    bool is_initialized;
} apds9960_t;

/* ==============================================================================
 * Public Function Prototypes
 * ============================================================================== */

/**
 * @brief  Initializes the APDS-9960 sensor on the given I2C bus.
 * @param  dev: Pointer to the APDS-9960 device handle.
 * @param  hi2c: Pointer to STM32 HAL I2C handle (e.g. &hi2c3).
 * @return true if detected and successfully initialized, false otherwise.
 */
bool APDS9960_Init(apds9960_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Reads RGBC color data and proximity atomically in a single I2C burst.
 * @param  dev: Pointer to the APDS-9960 device handle.
 * @param  data: Pointer to data structure to populate.
 * @return true if read was successful and data was valid, false otherwise.
 */
bool APDS9960_ReadData(apds9960_t *dev, apds9960_data_t *data);

/**
 * @brief  Classifies the detected object into Red, Green, Blue, or None/Unknown.
 * @param  data: Pointer to current RGBC data.
 * @return Classified can color (bmo_can_color_t).
 */
bmo_can_color_t APDS9960_ClassifyColor(const apds9960_data_t *data);

/**
 * @brief  Helper to convert color enum to printable string.
 * @param  color: The color enum.
 * @return String representation ("RED", "GREEN", "BLUE", "NONE", "UNKNOWN").
 */
const char* APDS9960_ColorToString(bmo_can_color_t color);

#endif /* BSP_APDS9960_H */
