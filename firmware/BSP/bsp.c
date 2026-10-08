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

    /* 7. VL53L0X ToF Distance Sensor (I2C3 PC8/PC9) */
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&g_bsp.tof, &hi2c3)) {
            VL53L0X_StartContinuous(&g_bsp.tof, 0);
            g_bsp.tof_ok = true;
            printf("[BSP] VL53L0X ToF Distance (I2C3 0x29): OK\r\n");
        } else {
            printf("[BSP] VL53L0X ToF: Init failed\r\n");
        }
    } else {
        printf("[BSP] VL53L0X ToF (I2C3 0x29): Not detected\r\n");
    }

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
