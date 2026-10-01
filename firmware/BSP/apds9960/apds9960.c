/**
  ******************************************************************************
  * @file    apds9960.c
  * @brief   Implementation of APDS-9960 RGB Color and Proximity sensor driver
  ******************************************************************************
  */

#include "apds9960.h"

#define I2C_TIMEOUT_MS  50

/* ==============================================================================
 * Private Helper Functions
 * ============================================================================== */

static inline void writeReg(apds9960_t *dev, uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
}

static inline uint8_t readReg(apds9960_t *dev, uint8_t reg)
{
    uint8_t val = 0;
    HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
    return val;
}

/* ==============================================================================
 * Public API Implementation
 * ============================================================================== */

bool APDS9960_Init(apds9960_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (dev == NULL || hi2c == NULL) {
        return false;
    }

    dev->hi2c = hi2c;
    dev->dev_addr_8bit = APDS9960_I2C_ADDR_8BIT;
    dev->is_initialized = false;

    /* 1. Verify physical presence on I2C bus */
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->dev_addr_8bit, 2, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    /* 2. Verify Silicon Chip ID (Register 0x92 must equal 0xAB) */
    uint8_t chip_id = readReg(dev, APDS9960_REG_ID);
    if (chip_id != APDS9960_CHIP_ID) {
        return false;
    }

    /* 3. Disable all engines during configuration (POR state) */
    writeReg(dev, APDS9960_REG_ENABLE, 0x00);
    HAL_Delay(10);

    /* 4. Set ADC Integration Time ATIME (0xF6 = 10 cycles = 27.8 ms, max count 10241) */
    writeReg(dev, APDS9960_REG_ATIME, 0xF6);

    /* 5. Set Wait Time WTIME (0xFF = 2.78 ms) */
    writeReg(dev, APDS9960_REG_WTIME, 0xFF);

    /* 6. Set CONFIG1 (0x40 per datasheet) */
    writeReg(dev, APDS9960_REG_CONFIG1, 0x40);

    /* 7. Configure Proximity Pulse PPULSE (8 pulses of 16 us: PPLEN=2, PPULSE=7) */
    writeReg(dev, APDS9960_REG_PPULSE, 0x87);

    /* 8. Set Gains & LED Drive (CONTROL: AGAIN=4x (0x01), PGAIN=4x (0x02 << 2), LDRIVE=100mA (0x00)) */
    writeReg(dev, APDS9960_REG_CONTROL, 0x09);

    /* 9. Set CONFIG2 (0x01 per datasheet, LED_BOOST = 100%) */
    writeReg(dev, APDS9960_REG_CONFIG2, 0x01);

    /* 10. Set CONFIG3 (0x00: all proximity photodiodes active) */
    writeReg(dev, APDS9960_REG_CONFIG3, 0x00);

    /* 11. Power ON internal oscillator (ENABLE = PON) */
    writeReg(dev, APDS9960_REG_ENABLE, APDS9960_ENABLE_PON);
    HAL_Delay(10);  /* Wait >= 7 ms exit sleep delay per datasheet */

    /* 12. Enable ALS/Color Engine (AEN) and Proximity Engine (PEN) */
    writeReg(dev, APDS9960_REG_ENABLE, APDS9960_ENABLE_PON | APDS9960_ENABLE_AEN | APDS9960_ENABLE_PEN);

    dev->is_initialized = true;
    return true;
}

bool APDS9960_ReadData(apds9960_t *dev, apds9960_data_t *data)
{
    if (dev == NULL || !dev->is_initialized || data == NULL) {
        return false;
    }

    /* Check if fresh ALS / RGBC data is available */
    uint8_t status = readReg(dev, APDS9960_REG_STATUS);
    if ((status & APDS9960_STATUS_AVALID) == 0) {
        return false;
    }

    /* Single continuous burst read of 8 bytes: CDATAL..CDATAH, RDATAL..RDATAH, GDATAL..GDATAH, BDATAL..BDATAH.
     * Reading CDATAL triggers internal 64-bit hardware latch (Avago Datasheet page 25). */
    uint8_t buf[8];
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr_8bit, APDS9960_REG_CDATAL,
                         I2C_MEMADD_SIZE_8BIT, buf, 8, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    data->clear = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    data->red   = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    data->green = (uint16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    data->blue  = (uint16_t)(buf[6] | ((uint16_t)buf[7] << 8));

    /* Read proximity distance (0..255) */
    data->proximity = readReg(dev, APDS9960_REG_PDATA);

    return true;
}

bmo_can_color_t APDS9960_ClassifyColor(const apds9960_data_t *data)
{
    if (data == NULL) {
        return BMO_CAN_COLOR_NONE;
    }

    uint32_t total = (uint32_t)data->red + (uint32_t)data->green + (uint32_t)data->blue;

    /* If ambient light / reflection is too low, no object is present in front of sensor */
    if (data->clear < 40 || total < 40) {
        return BMO_CAN_COLOR_NONE;
    }

    /* Relative chromatic ratios */
    float r_ratio = (float)data->red / (float)total;
    float g_ratio = (float)data->green / (float)total;
    float b_ratio = (float)data->blue / (float)total;

    /* RED Can Classification */
    if (r_ratio >= 0.40f &&
        data->red > (uint16_t)(data->green * 1.20f) &&
        data->red > (uint16_t)(data->blue * 1.20f)) {
        return BMO_CAN_COLOR_RED;
    }

    /* GREEN Can Classification */
    if (g_ratio >= 0.38f &&
        data->green > (uint16_t)(data->red * 1.15f) &&
        data->green > (uint16_t)(data->blue * 1.10f)) {
        return BMO_CAN_COLOR_GREEN;
    }

    /* BLUE Can Classification */
    if (b_ratio >= 0.38f &&
        data->blue > (uint16_t)(data->red * 1.20f) &&
        data->blue > (uint16_t)(data->green * 1.15f)) {
        return BMO_CAN_COLOR_BLUE;
    }

    return BMO_CAN_COLOR_UNKNOWN;
}

const char* APDS9960_ColorToString(bmo_can_color_t color)
{
    switch (color) {
        case BMO_CAN_COLOR_RED:     return "RED (ROUGE)";
        case BMO_CAN_COLOR_GREEN:   return "GREEN (VERT)";
        case BMO_CAN_COLOR_BLUE:    return "BLUE (BLEU)";
        case BMO_CAN_COLOR_UNKNOWN: return "UNKNOWN";
        case BMO_CAN_COLOR_NONE:
        default:                    return "NONE (VIDE)";
    }
}
