/**
  ******************************************************************************
  * @file    vl53l0x.c
  * @brief   Implementation of VL53L0X Time-of-Flight sensor driver for STM32 HAL
  ******************************************************************************
  */

#include "vl53l0x.h"
#include <string.h>

#define I2C_TIMEOUT_MS                      100
#define decodeVcselPeriod(reg_val)          (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks)     (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/* ==============================================================================
 * Private Register Definitions & Internal Types
 * ============================================================================== */
enum reg_addr {
    SYSRANGE_START                              = 0x00,
    SYSTEM_THRESH_HIGH                          = 0x0C,
    SYSTEM_THRESH_LOW                           = 0x0E,
    SYSTEM_SEQUENCE_CONFIG                      = 0x01,
    SYSTEM_RANGE_CONFIG                         = 0x09,
    SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,
    SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,
    GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,
    SYSTEM_INTERRUPT_CLEAR                      = 0x0B,
    RESULT_INTERRUPT_STATUS                     = 0x13,
    RESULT_RANGE_STATUS                         = 0x14,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
    RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,
    ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,
    I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,
    MSRC_CONFIG_CONTROL                         = 0x60,
    PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,
    FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
    PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,
    PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,
    SYSTEM_HISTOGRAM_BIN                        = 0x81,
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
    HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,
    MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,
    SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
    IDENTIFICATION_MODEL_ID                     = 0xC0,
    IDENTIFICATION_REVISION_ID                  = 0xC2,
    OSC_CALIBRATE_VAL                           = 0xF8,
    GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
    GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
    POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,
    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,
    ALGO_PHASECAL_LIM                           = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30
};

typedef struct {
    uint8_t tcc;
    uint8_t msrc;
    uint8_t dss;
    uint8_t pre_range;
    uint8_t final_range;
} sequence_step_enables_t;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
} sequence_step_timeouts_t;

/* ==============================================================================
 * Private Function Prototypes
 * ============================================================================== */
static void writeReg(vl53l0x_t *dev, uint8_t reg, uint8_t value);
static void writeReg16Bit(vl53l0x_t *dev, uint8_t reg, uint16_t value);
static void writeReg32Bit(vl53l0x_t *dev, uint8_t reg, uint32_t value);
static uint8_t readReg(vl53l0x_t *dev, uint8_t reg);
static uint16_t readReg16Bit(vl53l0x_t *dev, uint8_t reg);
static void writeMulti(vl53l0x_t *dev, uint8_t reg, const uint8_t *src, uint16_t count);
static void readMulti(vl53l0x_t *dev, uint8_t reg, uint8_t *dst, uint16_t count);

static bool getSpadInfo(vl53l0x_t *dev, uint8_t *count, bool *type_is_aperture);
static void getSequenceStepEnables(vl53l0x_t *dev, sequence_step_enables_t *enables);
static void getSequenceStepTimeouts(vl53l0x_t *dev, const sequence_step_enables_t *enables, sequence_step_timeouts_t *timeouts);
static bool performSingleRefCalibration(vl53l0x_t *dev, uint8_t vhv_init_byte);
static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

static inline void startTimeout(vl53l0x_t *dev) {
    dev->timeout_start_ms = HAL_GetTick();
}

static inline bool checkTimeoutExpired(const vl53l0x_t *dev) {
    return (dev->io_timeout_ms > 0 && ((uint32_t)(HAL_GetTick() - dev->timeout_start_ms)) > dev->io_timeout_ms);
}

/* ==============================================================================
 * I2C Low-Level Helpers (Re-entrant with local buffers)
 * ============================================================================== */
static void writeReg(vl53l0x_t *dev, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS);
}

static void writeReg16Bit(vl53l0x_t *dev, uint8_t reg, uint16_t value) {
    uint8_t temp[2];
    temp[0] = (uint8_t)((value >> 8) & 0xFF);
    temp[1] = (uint8_t)(value & 0xFF);
    HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, temp, 2, I2C_TIMEOUT_MS);
}

static void writeReg32Bit(vl53l0x_t *dev, uint8_t reg, uint32_t value) {
    uint8_t temp[4];
    temp[0] = (uint8_t)((value >> 24) & 0xFF);
    temp[1] = (uint8_t)((value >> 16) & 0xFF);
    temp[2] = (uint8_t)((value >> 8) & 0xFF);
    temp[3] = (uint8_t)(value & 0xFF);
    HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, temp, 4, I2C_TIMEOUT_MS);
}

static uint8_t readReg(vl53l0x_t *dev, uint8_t reg) {
    uint8_t value = 0;
    HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS);
    return value;
}

static uint16_t readReg16Bit(vl53l0x_t *dev, uint8_t reg) {
    uint8_t temp[2] = {0};
    HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, temp, 2, I2C_TIMEOUT_MS);
    return (uint16_t)(((uint16_t)temp[0] << 8) | temp[1]);
}

static void writeMulti(vl53l0x_t *dev, uint8_t reg, const uint8_t *src, uint16_t count) {
    HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)src, count, I2C_TIMEOUT_MS);
}

static void readMulti(vl53l0x_t *dev, uint8_t reg, uint8_t *dst, uint16_t count) {
    HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr_8bit, reg, I2C_MEMADD_SIZE_8BIT, dst, count, I2C_TIMEOUT_MS);
}

/* ==============================================================================
 * Public API Implementation
 * ============================================================================== */
void VL53L0X_SetAddress(vl53l0x_t *dev, uint8_t new_8bit_addr) {
    writeReg(dev, I2C_SLAVE_DEVICE_ADDRESS, (new_8bit_addr >> 1) & 0x7F);
    dev->dev_addr_8bit = new_8bit_addr;
}

uint8_t VL53L0X_GetAddress(const vl53l0x_t *dev) {
    return dev->dev_addr_8bit;
}

bool VL53L0X_Init(vl53l0x_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;
    dev->dev_addr_8bit = VL53L0X_DEFAULT_ADDRESS_8BIT;
    dev->io_timeout_ms = VL53L0X_DEFAULT_TIMEOUT_MS;
    dev->is_timeout = false;

    /* Verify Model ID (0xEE expected for VL53L0X) */
    uint8_t model_id = readReg(dev, IDENTIFICATION_MODEL_ID);
    if (model_id != 0xEE) {
        return false;
    }

    /* Set 2V8 mode (EXTSUP_HV) */
    writeReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
             readReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);

    /* Set I2C standard mode */
    writeReg(dev, 0x88, 0x00);
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    dev->stop_variable = readReg(dev, 0x91);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    /* Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks */
    writeReg(dev, MSRC_CONFIG_CONTROL, readReg(dev, MSRC_CONFIG_CONTROL) | 0x12);
    VL53L0X_SetSignalRateLimit(dev, 0.25f);
    writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    uint8_t spad_count = 0;
    bool spad_type_is_aperture = false;
    if (!getSpadInfo(dev, &spad_count, &spad_type_is_aperture)) {
        return false;
    }

    uint8_t ref_spad_map[6] = {0};
    readMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    writeReg(dev, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    writeMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* Load default tuning settings from ST API */
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x09, 0x00);
    writeReg(dev, 0x10, 0x00);
    writeReg(dev, 0x11, 0x00);
    writeReg(dev, 0x24, 0x01);
    writeReg(dev, 0x25, 0xFF);
    writeReg(dev, 0x75, 0x00);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x4E, 0x2C);
    writeReg(dev, 0x48, 0x00);
    writeReg(dev, 0x30, 0x20);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x30, 0x09);
    writeReg(dev, 0x54, 0x00);
    writeReg(dev, 0x31, 0x04);
    writeReg(dev, 0x32, 0x03);
    writeReg(dev, 0x40, 0x83);
    writeReg(dev, 0x46, 0x25);
    writeReg(dev, 0x60, 0x00);
    writeReg(dev, 0x27, 0x00);
    writeReg(dev, 0x50, 0x06);
    writeReg(dev, 0x51, 0x00);
    writeReg(dev, 0x52, 0x96);
    writeReg(dev, 0x56, 0x08);
    writeReg(dev, 0x57, 0x30);
    writeReg(dev, 0x61, 0x00);
    writeReg(dev, 0x62, 0x00);
    writeReg(dev, 0x64, 0x00);
    writeReg(dev, 0x65, 0x00);
    writeReg(dev, 0x66, 0xA0);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x22, 0x32);
    writeReg(dev, 0x47, 0x14);
    writeReg(dev, 0x49, 0xFF);
    writeReg(dev, 0x4A, 0x00);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x7A, 0x0A);
    writeReg(dev, 0x7B, 0x00);
    writeReg(dev, 0x78, 0x21);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x23, 0x34);
    writeReg(dev, 0x42, 0x00);
    writeReg(dev, 0x44, 0xFF);
    writeReg(dev, 0x45, 0x26);
    writeReg(dev, 0x46, 0x05);
    writeReg(dev, 0x40, 0x40);
    writeReg(dev, 0x0E, 0x06);
    writeReg(dev, 0x20, 0x1A);
    writeReg(dev, 0x43, 0x40);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x34, 0x03);
    writeReg(dev, 0x35, 0x44);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x31, 0x04);
    writeReg(dev, 0x4B, 0x09);
    writeReg(dev, 0x4C, 0x05);
    writeReg(dev, 0x4D, 0x04);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x44, 0x00);
    writeReg(dev, 0x45, 0x20);
    writeReg(dev, 0x47, 0x08);
    writeReg(dev, 0x48, 0x28);
    writeReg(dev, 0x67, 0x00);
    writeReg(dev, 0x70, 0x04);
    writeReg(dev, 0x71, 0x01);
    writeReg(dev, 0x72, 0xFE);
    writeReg(dev, 0x76, 0x00);
    writeReg(dev, 0x77, 0x00);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x0D, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0x01, 0xF8);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x8E, 0x01);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    /* Configure interrupt for sample ready (active low) */
    writeReg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    writeReg(dev, GPIO_HV_MUX_ACTIVE_HIGH, readReg(dev, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    dev->measurement_timing_budget_us = VL53L0X_GetMeasurementTimingBudget(dev);
    writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);
    VL53L0X_SetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    /* Single reference calibration */
    writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(dev, 0x40)) {
        return false;
    }
    writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(dev, 0x00)) {
        return false;
    }
    writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return true;
}

bool VL53L0X_SetSignalRateLimit(vl53l0x_t *dev, float limit_mcps) {
    if (limit_mcps < 0.0f || limit_mcps > 511.99f) {
        return false;
    }
    writeReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(limit_mcps * (1 << 7)));
    return true;
}

float VL53L0X_GetSignalRateLimit(vl53l0x_t *dev) {
    return (float)readReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool VL53L0X_SetMeasurementTimingBudget(vl53l0x_t *dev, uint32_t budget_us) {
    sequence_step_enables_t enables;
    sequence_step_timeouts_t timeouts;

    const uint16_t StartOverhead      = 1320;
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;
    const uint32_t MinTimingBudget    = 20000;

    if (budget_us < MinTimingBudget) {
        return false;
    }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(dev, &enables);
    getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }
    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;
        if (used_budget_us > budget_us) {
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        uint16_t final_range_timeout_mclks = (uint16_t)timeoutMicrosecondsToMclks(
            final_range_timeout_us, (uint8_t)timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
        dev->measurement_timing_budget_us = budget_us;
    }
    return true;
}

uint32_t VL53L0X_GetMeasurementTimingBudget(vl53l0x_t *dev) {
    sequence_step_enables_t enables;
    sequence_step_timeouts_t timeouts;

    const uint16_t StartOverhead      = 1910;
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(dev, &enables);
    getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }
    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    dev->measurement_timing_budget_us = budget_us;
    return budget_us;
}

bool VL53L0X_SetVcselPulsePeriod(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type, uint8_t period_pclks) {
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    sequence_step_enables_t enables;
    sequence_step_timeouts_t timeouts;

    getSequenceStepEnables(dev, &enables);
    getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE) {
        switch (period_pclks) {
            case 12: writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
            case 14: writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
            case 16: writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
            case 18: writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
            default: return false;
        }
        writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        writeReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_pre_range_timeout_mclks = (uint16_t)timeoutMicrosecondsToMclks(
            timeouts.pre_range_us, period_pclks);
        writeReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

        uint16_t new_msrc_timeout_mclks = (uint16_t)timeoutMicrosecondsToMclks(
            timeouts.msrc_dss_tcc_us, period_pclks);
        writeReg(dev, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
    } else if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE) {
        switch (period_pclks) {
            case 8:
                writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                writeReg(dev, 0xFF, 0x01);
                writeReg(dev, ALGO_PHASECAL_LIM, 0x30);
                writeReg(dev, 0xFF, 0x00);
                break;
            case 10:
                writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                writeReg(dev, 0xFF, 0x01);
                writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                writeReg(dev, 0xFF, 0x00);
                break;
            case 12:
                writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                writeReg(dev, 0xFF, 0x01);
                writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                writeReg(dev, 0xFF, 0x00);
                break;
            case 14:
                writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                writeReg(dev, 0xFF, 0x01);
                writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                writeReg(dev, 0xFF, 0x00);
                break;
            default:
                return false;
        }

        writeReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_final_range_timeout_mclks = (uint16_t)timeoutMicrosecondsToMclks(
            timeouts.final_range_us, period_pclks);

        if (enables.pre_range) {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
    } else {
        return false;
    }

    VL53L0X_SetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    uint8_t sequence_config = readReg(dev, SYSTEM_SEQUENCE_CONFIG);
    writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    performSingleRefCalibration(dev, 0x00);
    writeReg(dev, SYSTEM_SEQUENCE_CONFIG, sequence_config);

    return true;
}

uint8_t VL53L0X_GetVcselPulsePeriod(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type) {
    if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE) {
        return decodeVcselPeriod(readReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD));
    } else if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE) {
        return decodeVcselPeriod(readReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
    return 255;
}

void VL53L0X_StartContinuous(vl53l0x_t *dev, uint32_t period_ms) {
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0x91, dev->stop_variable);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    if (period_ms != 0) {
        uint16_t osc_calibrate_val = readReg16Bit(dev, OSC_CALIBRATE_VAL);
        if (osc_calibrate_val != 0) {
            period_ms *= osc_calibrate_val;
        }
        writeReg32Bit(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        writeReg(dev, SYSRANGE_START, 0x04);
    } else {
        writeReg(dev, SYSRANGE_START, 0x02);
    }
}

void VL53L0X_StopContinuous(vl53l0x_t *dev) {
    writeReg(dev, SYSRANGE_START, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0x91, 0x00);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
}

uint16_t VL53L0X_ReadDistanceContinuous(vl53l0x_t *dev, vl53l0x_stats_t *extra_stats) {
    uint8_t temp_buf[12];
    uint16_t dist = 65535;

    startTimeout(dev);
    while ((readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired(dev)) {
            dev->is_timeout = true;
            return 65535;
        }
    }

    if (extra_stats == NULL) {
        dist = readReg16Bit(dev, RESULT_RANGE_STATUS + 10);
    } else {
        readMulti(dev, 0x14, temp_buf, 12);
        extra_stats->range_status = temp_buf[0x00] >> 3;
        extra_stats->spad_cnt     = (uint16_t)((temp_buf[0x02] << 8) | temp_buf[0x03]);
        extra_stats->signal_cnt   = (uint16_t)((temp_buf[0x06] << 8) | temp_buf[0x07]);
        extra_stats->ambient_cnt  = (uint16_t)((temp_buf[0x08] << 8) | temp_buf[0x09]);
        dist                      = (uint16_t)((temp_buf[0x0A] << 8) | temp_buf[0x0B]);
        extra_stats->raw_distance_mm = dist;
    }

    writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
    return dist;
}

uint16_t VL53L0X_ReadDistanceSingle(vl53l0x_t *dev, vl53l0x_stats_t *extra_stats) {
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0x91, dev->stop_variable);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    writeReg(dev, SYSRANGE_START, 0x01);

    startTimeout(dev);
    while (readReg(dev, SYSRANGE_START) & 0x01) {
        if (checkTimeoutExpired(dev)) {
            dev->is_timeout = true;
            return 65535;
        }
    }

    return VL53L0X_ReadDistanceContinuous(dev, extra_stats);
}

bool VL53L0X_TimeoutOccurred(vl53l0x_t *dev) {
    bool tmp = dev->is_timeout;
    dev->is_timeout = false;
    return tmp;
}

void VL53L0X_SetTimeout(vl53l0x_t *dev, uint16_t timeout_ms) {
    dev->io_timeout_ms = timeout_ms;
}

uint16_t VL53L0X_GetTimeout(const vl53l0x_t *dev) {
    return dev->io_timeout_ms;
}

/* ==============================================================================
 * Private Calibration & Timing Calculations
 * ============================================================================== */
static bool getSpadInfo(vl53l0x_t *dev, uint8_t *count, bool *type_is_aperture) {
    uint8_t tmp;
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0xFF, 0x06);
    writeReg(dev, 0x83, readReg(dev, 0x83) | 0x04);
    writeReg(dev, 0xFF, 0x07);
    writeReg(dev, 0x81, 0x01);
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0x94, 0x6B);
    writeReg(dev, 0x83, 0x00);

    startTimeout(dev);
    while (readReg(dev, 0x83) == 0x00) {
        if (checkTimeoutExpired(dev)) {
            return false;
        }
    }

    writeReg(dev, 0x83, 0x01);
    tmp = readReg(dev, 0x92);
    *count = tmp & 0x7F;
    *type_is_aperture = (tmp >> 7) & 0x01;

    writeReg(dev, 0x81, 0x00);
    writeReg(dev, 0xFF, 0x06);
    writeReg(dev, 0x83, readReg(dev, 0x83) & ~0x04);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    return true;
}

static void getSequenceStepEnables(vl53l0x_t *dev, sequence_step_enables_t *enables) {
    uint8_t seq = readReg(dev, SYSTEM_SEQUENCE_CONFIG);
    enables->tcc         = (seq >> 4) & 0x1;
    enables->dss         = (seq >> 3) & 0x1;
    enables->msrc        = (seq >> 2) & 0x1;
    enables->pre_range   = (seq >> 6) & 0x1;
    enables->final_range = (seq >> 7) & 0x1;
}

static void getSequenceStepTimeouts(vl53l0x_t *dev, const sequence_step_enables_t *enables, sequence_step_timeouts_t *timeouts) {
    timeouts->pre_range_vcsel_period_pclks = VL53L0X_GetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE);

    timeouts->msrc_dss_tcc_mclks = readReg(dev, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                                          (uint8_t)timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks = decodeTimeout(readReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                                       (uint8_t)timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = VL53L0X_GetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE);

    timeouts->final_range_mclks = decodeTimeout(readReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                                         (uint8_t)timeouts->final_range_vcsel_period_pclks);
}

static uint16_t decodeTimeout(uint16_t reg_val) {
    return (uint16_t)((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encodeTimeout(uint16_t timeout_mclks) {
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static bool performSingleRefCalibration(vl53l0x_t *dev, uint8_t vhv_init_byte) {
    writeReg(dev, SYSRANGE_START, 0x01 | vhv_init_byte);
    startTimeout(dev);
    while ((readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired(dev)) {
            return false;
        }
    }
    writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg(dev, SYSRANGE_START, 0x00);
    return true;
}
