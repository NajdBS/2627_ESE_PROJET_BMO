/**
  ******************************************************************************
  * @file    bsp.c
  * @brief   Board Support Package facade implementation for BMO Robot
  ******************************************************************************
  */

#include "bsp.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* Global BSP singleton */
bsp_t g_bsp;

bool BSP_Init(void)
{
    memset(&g_bsp, 0, sizeof(bsp_t));

    printf("\r\n========================================\r\n");
    printf("   BMO SYSTEM - BOARD SUPPORT PACKAGE   \r\n");
    printf("========================================\r\n");

    /* 1. NeoPixel 12-LED Ring (PB14 on TIM15_CH1) */
    np_led_clear();
    g_bsp.neopixel_ok = true;
    printf("[BSP] NeoPixel 12-LED Ring: OK\r\n");

    /* 2. OLED Display (SSD1306 on I2C2 Fast-Mode) */
    if (HAL_I2C_IsDeviceReady(&hi2c2, SSD1306_I2C_ADDR, 3, 100) == HAL_OK) {
        ssd1306_Init();
        g_bsp.oled_ok = true;
        BMO_Screen_Init();
        BMO_Screen_SetFace(BMO_FACE_NORMAL);
        printf("[BSP] OLED Display (SSD1306 on I2C2): OK\r\n");
    } else {
        printf("[BSP] OLED Display (I2C2): Not detected\r\n");
    }

    /* 3. DRV8833 Dual DC Motor Driver (TIM3 CH1..CH4 @ 20 kHz) */
    DRV8833_Init(&g_bsp.motor_left, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, false);
    DRV8833_Init(&g_bsp.motor_right, &htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, true);
    DRV8833_Coast(&g_bsp.motor_left);
    DRV8833_Coast(&g_bsp.motor_right);
    g_bsp.motors_ok = true;
    printf("[BSP] Motors DRV8833 (TIM3 CH1..CH4): OK\r\n");

    /* 4. Quadrature Encoders (TIM2 32-bit & TIM4 16-bit) */
    Encoder_Init(&g_bsp.enc_left, &htim2, ENCODER_DEFAULT_CPR_WHEEL, ENCODER_DEFAULT_WHEEL_DIAM_MM, false);
    Encoder_Init(&g_bsp.enc_right, &htim4, ENCODER_DEFAULT_CPR_WHEEL, ENCODER_DEFAULT_WHEEL_DIAM_MM, true);
    g_bsp.encoders_ok = true;
    printf("[BSP] Quadrature Encoders (TIM2/TIM4): OK\r\n");

    /* 5. Gripper Servo (PC6 on TIM8_CH1 @ 50 Hz PWM) */
    if (SERVO_Init(&g_bsp.gripper, &htim8, TIM_CHANNEL_1) == HAL_OK) {
        g_bsp.gripper_ok = true;
        SERVO_Release(&g_bsp.gripper);
        printf("[BSP] Gripper Servo (PC6 / TIM8_CH1): OK (Released)\r\n");
    } else {
        printf("[BSP] Gripper Servo: Init error\r\n");
    }

    /* 6. MPU-6050 6-DOF IMU (I2C3 PC8/PC9) */
    if (MPU6050_Init(&g_bsp.imu, &hi2c3)) {
        g_bsp.imu_ok = true;
        printf("[BSP] MPU-6050 IMU: Detected (0x%02X). Calibrating (~1s stationary)...\r\n",
               g_bsp.imu.dev_addr_8bit >> 1);
        if (MPU6050_Calibrate(&g_bsp.imu, 150)) {
            printf("[BSP] MPU-6050 IMU: Calibrated (Gyro Z offset = %.2f deg/s)\r\n",
                   g_bsp.imu.gyro_z_offset);
        } else {
            printf("[BSP] MPU-6050 IMU: Calibration failed\r\n");
        }
    } else {
        printf("[BSP] MPU-6050 IMU (I2C3 0x68): Not detected\r\n");
    }

    /* 7. VL53L0X ToF Distance Sensors (Shutdown all sensors initially) */
    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_TOF_R_GPIO_Port, XSHUT_TOF_R_Pin, GPIO_PIN_RESET);
    /* HAL_GPIO_WritePin(EXT_PB13_GPIO_Port, EXT_PB13_Pin, GPIO_PIN_RESET); */ /* Optional 3rd Center ToF shutdown */
    HAL_Delay(10);

    /* Wake up Left ToF */
    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&g_bsp.tof_left, &hi2c3)) {
            VL53L0X_SetAddress(&g_bsp.tof_left, VL53L0X_ADDR_LEFT_8BIT);
            VL53L0X_StartContinuous(&g_bsp.tof_left, 0);
            g_bsp.tof_left_ok = true;
            printf("[BSP] VL53L0X ToF Left  (XSHUT_TOF_L -> 0x30): OK\r\n");
        }
    }
    if (!g_bsp.tof_left_ok) {
        printf("[BSP] VL53L0X ToF Left  (XSHUT_TOF_L -> 0x30): Not detected\r\n");
    }

    /* Wake up Right ToF */
    HAL_GPIO_WritePin(XSHUT_TOF_R_GPIO_Port, XSHUT_TOF_R_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&g_bsp.tof_right, &hi2c3)) {
            VL53L0X_SetAddress(&g_bsp.tof_right, VL53L0X_ADDR_RIGHT_8BIT);
            VL53L0X_StartContinuous(&g_bsp.tof_right, 0);
            g_bsp.tof_right_ok = true;
            printf("[BSP] VL53L0X ToF Right (XSHUT_TOF_R -> 0x31): OK\r\n");
        }
    }
    if (!g_bsp.tof_right_ok) {
        printf("[BSP] VL53L0X ToF Right (XSHUT_TOF_R -> 0x31): Not detected\r\n");
    }

    /* Optional 3rd Center ToF (EXT_PB13 -> 0x32) - Uncomment to activate */
    /*
    HAL_GPIO_WritePin(EXT_PB13_GPIO_Port, EXT_PB13_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&g_bsp.tof_center, &hi2c3)) {
            VL53L0X_SetAddress(&g_bsp.tof_center, VL53L0X_ADDR_CENTER_8BIT);
            VL53L0X_StartContinuous(&g_bsp.tof_center, 0);
            g_bsp.tof_center_ok = true;
            printf("[BSP] VL53L0X ToF Center (PB13 -> 0x32): OK\r\n");
        }
    }
    if (!g_bsp.tof_center_ok) {
        printf("[BSP] VL53L0X ToF Center (PB13 -> 0x32): Not detected\r\n");
    }
    */

    /* 8. APDS-9960 RGB Color & Proximity Sensor (I2C3 PC8/PC9) */
    if (HAL_I2C_IsDeviceReady(&hi2c3, APDS9960_I2C_ADDR_8BIT, 2, 50) == HAL_OK) {
        if (APDS9960_Init(&g_bsp.color, &hi2c3)) {
            g_bsp.color_ok = true;
            printf("[BSP] APDS-9960 Color/Prox (I2C3 0x39): OK\r\n");
        } else {
            printf("[BSP] APDS-9960 Color/Prox: Init failed\r\n");
        }
    } else {
        printf("[BSP] APDS-9960 Color/Prox (I2C3 0x39): Not detected\r\n");
    }

    /* 9. YDLIDAR X2 (UART1 @ 115200) */
    YDLIDAR_X2_Init(&g_bsp.lidar, &huart1);
    g_bsp.lidar_ok = true;
    printf("[BSP] YDLIDAR X2 (UART1): OK\r\n");

    /* 10. Passive Buzzer (PB5 on TIM17_CH1) */
    if (BUZZER_Init(&g_bsp.buzzer, &htim17, TIM_CHANNEL_1)) {
        g_bsp.buzzer_ok = true;
        printf("[BSP] Passive Buzzer (PB5 on TIM17_CH1): OK\r\n");
        /* Play cheerful BMO boot chime! */
        BUZZER_PlaySound(&g_bsp.buzzer, BMO_SOUND_BOOT);
    } else {
        printf("[BSP] Passive Buzzer: Init failed\r\n");
    }

    /* 11. BQ27220 Single-Cell Fuel Gauge (I2C1 PA15/PB9 @ 0x55) */
    if (BQ27220_Init(&g_bsp.fuel_gauge, &hi2c1) == HAL_OK) {
        g_bsp.fuel_gauge_ok = true;
        printf("[BSP] BQ27220 Fuel Gauge (I2C1 0x55): OK (SOC: %u%%, %u mV)\r\n",
               g_bsp.fuel_gauge.soc_percent,
               g_bsp.fuel_gauge.voltage_mv);
    } else {
        printf("[BSP] BQ27220 Fuel Gauge (I2C1 0x55): Not detected\r\n");
    }

    /* 12. BQ25896 / BQ25895 Fast Charger & 5V Boost (I2C1 PA15/PB9 @ 0x6B / 0x6A) */
    if (BQ25896_Init(&g_bsp.charger, &hi2c1) == HAL_OK) {
        g_bsp.charger_ok = true;
        printf("[BSP] BQ25896 Charger (I2C1 0x%02X): OK (VBUS: %u mV, VBAT: %u mV)\r\n",
               g_bsp.charger.i2c_addr >> 1,
               g_bsp.charger.vbus_mv,
               g_bsp.charger.vbat_mv);
    } else {
        printf("[BSP] BQ25896 Charger (I2C1): Not detected\r\n");
    }

    printf("========================================\r\n\r\n");
    return true;
}

void BSP_Update(float dt_seconds)
{
    /* High-rate IMU heading integration */
    if (g_bsp.imu_ok) {
        MPU6050_Update(&g_bsp.imu, dt_seconds);
    }

    /* Advance buzzer non-blocking sound sequencer */
    if (g_bsp.buzzer_ok) {
        BUZZER_Process(&g_bsp.buzzer, HAL_GetTick());
    }
}
