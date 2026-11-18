/*
 * pcb_tests.h
 * Tests unitaires materiels du PCB BMO (STM32G474)
 */

#ifndef PCB_TESTS_H
#define PCB_TESTS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Scenarios de test disponibles */
#define TEST_00_I2C_SCANNER       0   /* Scan I2C1 (Power), I2C2 (OLED), I2C3 (Capteurs) */
#define TEST_01_HSI_BLINK         1   /* Horloge interne HSI (16 MHz) + LED_STATUS (PA5) */
#define TEST_02_HSE_BLINK         2   /* Quartz externe HSE + PLL (170 MHz) */
#define TEST_03_BUTTONS_LEDS      3   /* Boutons USER_BTN (PC13), PWR_BTN (PA11) & entrees */
#define TEST_04_UART_VCP_BLE      4   /* UART2 USB VCP & UART3 BLE */
#define TEST_05_OLED_DISPLAY      5   /* Ecran OLED SSD1306 (I2C2) */
#define TEST_06_BUZZER_PWM        6   /* Buzzer PWM (PB5 / TIM17) */
#define TEST_07_SERVO_PINCE       7   /* Servomoteur pince (PC6 / TIM8) */
#define TEST_08_MOTORS_DRV8833    8   /* Moteurs DC DRV8833 (TIM3 + PA4 nSLEEP) */
#define TEST_09_ENCODERS          9   /* Encodeurs roues (TIM2 PA0/PA1 & TIM4 PB6/PB7) */
#define TEST_10_NEOPIXEL_RING     10  /* Anneau NeoPixel 12 LEDs (PB14 / TIM15) */
#define TEST_11_SENSORS_RAW       11  /* Capteurs I2C3 (MPU6050, APDS9960, VL53L0X) */
#define TEST_12_POWER_BATTERY     12  /* Jauge BQ27220 & chargeur BQ25896 (I2C1) */
#define TEST_13_LIDAR_X2          13  /* LiDAR YDLIDAR X2 (USART1 DMA + PA12) */

/* Valeur par defaut si non definie dans main.c */
#ifndef PCB_ENABLE_5V_BOOST_ON_BATTERY
#define PCB_ENABLE_5V_BOOST_ON_BATTERY   1
#endif

/* Inversion logicielle du sens des moteurs DC et encodeurs (1 = inverse, 0 = normal) */
#ifndef PCB_MOTOR_LEFT_INVERTED
#define PCB_MOTOR_LEFT_INVERTED          1
#endif

#ifndef PCB_MOTOR_RIGHT_INVERTED
#define PCB_MOTOR_RIGHT_INVERTED         0
#endif

#ifndef PCB_ENC_LEFT_INVERTED
#define PCB_ENC_LEFT_INVERTED            1
#endif

#ifndef PCB_ENC_RIGHT_INVERTED
#define PCB_ENC_RIGHT_INVERTED           0
#endif

/* Flag mis a 1 par SystemClock_Config() si le quartz HSE (PF0/PF1) echoue */
extern volatile uint8_t g_hse_clock_failed;

/* Execute le test unitaire selectionne dans une boucle */
void pcb_run_test_ex(int scenario, bool enable_5v_boost);
void pcb_run_test(int scenario);

/* Fonctions de test individuelles */
void test_00_i2c_scanner(void);
void test_01_hsi_blink(void);
void test_02_hse_blink(void);
void test_03_buttons_leds(void);
void test_04_uart_vcp_ble(void);
void test_05_oled_display(void);
void test_06_buzzer_pwm(void);
void test_07_servo_pince(void);
void test_08_motors_drv8833(void);
void test_09_encoders(void);
void test_10_neopixel_ring(void);
void test_11_sensors_raw(void);
void test_12_power_battery(void);
void test_13_lidar_x2(void);

#endif /* PCB_TESTS_H */
