/*
 * pcb_tests.c
 * Tests unitaires materiels du PCB BMO (STM32G474)
 */

#include "pcb_tests.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "mpu6050.h"
#include "apds9960.h"
#include "vl53l0x.h"
#include "bq27220.h"
#include "bq25896.h"
#include "drv8833.h"
#include "servo.h"
#include "buzzer.h"
#include "neopixel.h"
#include "encoder.h"
#include "ydlidar_x2.h"

/* Etat Boost 5V configure depuis main.c */
static bool s_enable_5v_boost = (PCB_ENABLE_5V_BOOST_ON_BATTERY != 0);

/* Redirection printf vers USART2 (USB VCP @ 115200 bauds) */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}

static void ble_print(const char *str) {
    if (str != NULL) {
        size_t len = strlen(str);
        if (len > 0) {
            HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t)len, 100);
        }
    }
}

/* Configure le rail 5V via BQ25896 */
static void pcb_configure_5v_rail(bool enable_boost) {
    bq25896_t chg;
    if (BQ25896_Init(&chg, &hi2c1) == HAL_OK) {
        (void)BQ25896_DisableWatchdog(&chg);
        (void)BQ25896_SetBoostMode(&chg, enable_boost);
        printf("Boost 5V (0x%02X): %s\r\n",
               chg.i2c_addr >> 1, enable_boost ? "ON" : "OFF");
    }
}

/* Test 00 : Scanner I2C */
static const char* i2c_chip_name(I2C_HandleTypeDef *hi2c, uint8_t addr7) {
    if (hi2c == &hi2c1) {
        if (addr7 == 0x55) return "BQ27220";
        if (addr7 == 0x6B || addr7 == 0x6A) return "BQ25896";
    } else if (hi2c == &hi2c2) {
        if (addr7 == 0x3C || addr7 == 0x3D) return "OLED";
    } else if (hi2c == &hi2c3) {
        if (addr7 == 0x68 || addr7 == 0x69) return "MPU6050";
        if (addr7 == 0x39) return "APDS9960";
        if (addr7 == 0x29) return "VL53L0X";
        if (addr7 == 0x30) return "ToF_L";
        if (addr7 == 0x31) return "ToF_R";
    }
    return "?";
}

static void scan_single_i2c_bus(const char *bus_name, I2C_HandleTypeDef *hi2c) {
    printf("  %-5s: ", bus_name);
    uint8_t count = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 10) == HAL_OK) {
            count++;
            printf("0x%02X(%s) ", addr, i2c_chip_name(hi2c, addr));
        }
    }

    if (count == 0) {
        printf("aucun");
    }
    printf("\r\n");
}

void test_00_i2c_scanner(void) {
    printf("\r\n--- Test 00 : Scan I2C ---\r\n");

    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(XSHUT_TOF_R_GPIO_Port, XSHUT_TOF_R_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    uint32_t scan_num = 1;
    while (1) {
        printf("\r\nScan #%lu\r\n", scan_num++);
        scan_single_i2c_bus("I2C1", &hi2c1);
        scan_single_i2c_bus("I2C2", &hi2c2);
        scan_single_i2c_bus("I2C3", &hi2c3);

        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        HAL_Delay(3000);
    }
}

/* Test 01 : HSI Blink (16 MHz) */
void test_01_hsi_blink(void) {
    printf("\r\n--- Test 01 : HSI Blink (%lu MHz) ---\r\n", SystemCoreClock / 1000000UL);

    uint32_t count = 0;
    while (1) {
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        if ((count++ % 4) == 0) {
            printf("HSI LED OK (%lu MHz)\r\n", SystemCoreClock / 1000000UL);
        }
        HAL_Delay(500);
    }
}

/* Test 02 : HSE Blink (170 MHz) */
void test_02_hse_blink(void) {
    printf("\r\n--- Test 02 : HSE Blink (%lu MHz) ---\r\n", SystemCoreClock / 1000000UL);

    if (g_hse_clock_failed) {
        printf("ERR: HSE non detecte (PF0/PF1)\r\n");
        while (1) {
            HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
            HAL_Delay(50);
        }
    }

    printf("HSE + PLL OK (%lu MHz)\r\n", SystemCoreClock / 1000000UL);
    while (1) {
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        HAL_Delay(250);
    }
}

/* Test 03 : Boutons poussoirs et entrees GPIO */
void test_03_buttons_leds(void) {
    printf("\r\n--- Test 03 : Boutons (PC13 USER, PA11 PWR) ---\r\n");

    bool last_user = false;
    bool last_pwr = false;

    while (1) {
        bool user_pressed = (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET);
        bool pwr_pressed  = (HAL_GPIO_ReadPin(PWR_BUTTON_GPIO_Port, PWR_BUTTON_Pin) == GPIO_PIN_RESET);

        if (user_pressed != last_user || pwr_pressed != last_pwr) {
            last_user = user_pressed;
            last_pwr  = pwr_pressed;
            int nfault  = (int)HAL_GPIO_ReadPin(MOT_nFAULT_GPIO_Port, MOT_nFAULT_Pin);
            int chg_int = (int)HAL_GPIO_ReadPin(CHARGER_INT_GPIO_Port, CHARGER_INT_Pin);
            int fg_gp   = (int)HAL_GPIO_ReadPin(FG_GPOUT_GPIO_Port, FG_GPOUT_Pin);

            printf("BTN USER=%d PWR=%d | FAULT=%d CHG_INT=%d FG=%d\r\n",
                   user_pressed ? 1 : 0, pwr_pressed ? 1 : 0, nfault, chg_int, fg_gp);
        }

        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin,
                          (user_pressed || pwr_pressed) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(20);
    }
}

/* Test 04 : UART2 VCP et UART3 BLE */
void test_04_uart_vcp_ble(void) {
    HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    printf("\r\n--- Test 04 : UART2 (VCP) & UART3 (BLE) [1=ON, 0=OFF, t=Toggle] ---\r\n");
    ble_print("BMO BLE OK\r\n");

    uint8_t rx_vcp = 0;
    uint8_t rx_ble = 0;

    while (1) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
        __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);

        if (HAL_UART_Receive(&huart2, &rx_vcp, 1, 5) == HAL_OK) {
            printf("VCP: '%c'\r\n", rx_vcp);
            if (rx_vcp == '1') HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
            else if (rx_vcp == '0') HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
            else if (rx_vcp == 't') HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
            HAL_UART_Transmit(&huart3, &rx_vcp, 1, 10);
        }

        if (HAL_UART_Receive(&huart3, &rx_ble, 1, 5) == HAL_OK) {
            printf("BLE: '%c'\r\n", rx_ble);
            if (rx_ble == '1') HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
            else if (rx_ble == '0') HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
            else if (rx_ble == 't') HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
            HAL_UART_Transmit(&huart2, &rx_ble, 1, 10);
        }
    }
}

/* Test 05 : Ecran OLED SSD1306 */
void test_05_oled_display(void) {
    printf("\r\n--- Test 05 : OLED SSD1306 (I2C2 0x3C) ---\r\n");

    if (HAL_I2C_IsDeviceReady(&hi2c2, (0x3C << 1), 3, 100) != HAL_OK) {
        printf("ERR: OLED non detecte (PA8/PA9)\r\n");
        while (1) {
            HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
            HAL_Delay(100);
        }
    }

    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    printf("OLED OK\r\n");

    uint32_t counter = 0;
    char str_buf[32];

    while (1) {
        ssd1306_Fill(Black);
        ssd1306_DrawRectangle(0, 0, 127, 63, White);

        ssd1306_SetCursor(10, 4);
        ssd1306_WriteString("BMO TEST PCB", Font_6x8, White);

        ssd1306_SetCursor(10, 18);
        ssd1306_WriteString("OLED I2C2 OK", Font_7x10, White);

        snprintf(str_buf, sizeof(str_buf), "Cnt: %lu", counter++);
        ssd1306_SetCursor(10, 34);
        ssd1306_WriteString(str_buf, Font_6x8, White);

        ssd1306_FillCircle(40, 50, 3, White);
        ssd1306_FillCircle(88, 50, 3, White);
        ssd1306_Line(58, 54, 70, 54, White);

        ssd1306_UpdateScreen();
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        HAL_Delay(200);
    }
}

/* Test 06 : Buzzer PWM (PB5 / TIM17_CH1) */
static void buzzer_play_freq(uint32_t freq_hz, uint32_t duration_ms) {
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
        HAL_Delay(duration_ms);
        return;
    }

    uint32_t timer_clk = 1000000UL;
    uint32_t arr = (timer_clk / freq_hz) - 1;
    if (arr > 65535) arr = 65535;

    __HAL_TIM_SET_AUTORELOAD(&htim17, arr);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, arr / 2);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

    HAL_Delay(duration_ms);
    HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
    HAL_Delay(20);
}

void test_06_buzzer_pwm(void) {
    printf("\r\n--- Test 06 : Buzzer PWM (PB5) ---\r\n");

    while (1) {
        printf("Bip buzzer\r\n");
        buzzer_play_freq(523, 150);
        buzzer_play_freq(659, 150);
        buzzer_play_freq(784, 150);
        buzzer_play_freq(1046, 250);
        HAL_Delay(2500);
    }
}

/* Test 07 : Servomoteur pince (PC6 / TIM8_CH1) */
static void servo_set_pulse_us(uint16_t pulse_us) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse_us);
}

void test_07_servo_pince(void) {
    printf("\r\n--- Test 07 : Servo Pince (PC6) ---\r\n");
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    __HAL_TIM_MOE_ENABLE(&htim8);

    while (1) {
        printf("Servo 0 deg (1000 us)\r\n");
        servo_set_pulse_us(1000);
        HAL_Delay(1500);

        printf("Servo 90 deg (1500 us)\r\n");
        servo_set_pulse_us(1500);
        HAL_Delay(1500);

        printf("Servo 180 deg (2000 us)\r\n");
        servo_set_pulse_us(2000);
        HAL_Delay(1500);
    }
}

/* Test 08 : Moteurs DC et encodeurs */
static void mot_set_signed(int32_t left_pwm, int32_t right_pwm) {
#if PCB_MOTOR_LEFT_INVERTED
    left_pwm = -left_pwm;
#endif
#if PCB_MOTOR_RIGHT_INVERTED
    right_pwm = -right_pwm;
#endif

    if (left_pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)left_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (left_pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(-left_pwm));
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }

    if (right_pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)right_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (right_pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(-right_pwm));
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

void test_08_motors_drv8833(void) {
    printf("\r\n--- Test 08 : Moteurs & Encodeurs (invL=%d, invR=%d) ---\r\n",
           PCB_MOTOR_LEFT_INVERTED, PCB_MOTOR_RIGHT_INVERTED);
    printf("Commandes : g=gauche, d=droite, a=avant, r=arriere, s=stop, c=cycle\r\n");

    HAL_GPIO_WritePin(MOT_nSLEEP_GPIO_Port, MOT_nSLEEP_Pin, GPIO_PIN_SET);
    HAL_Delay(20);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    int32_t arr  = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim3); /* 499 */
    int32_t duty = (arr * 95) / 100;                          /* 95% PWM */

    uint8_t rx_cmd = 0;
    bool auto_cycle = true;

    while (1) {
        if (auto_cycle) {
            /* Etape 1 : Moteur gauche avant */
            uint32_t enc_l_start = __HAL_TIM_GET_COUNTER(&htim2);
            printf("\r\n[1/3] Moteur gauche avant...\r\n");
            mot_set_signed(+duty, 0);

            for (int w = 0; w < 30; w++) {
                if (HAL_UART_Receive(&huart2, &rx_cmd, 1, 50) == HAL_OK) {
                    auto_cycle = false; break;
                }
            }
            mot_set_signed(0, 0);

            int32_t delta_l = (int32_t)(__HAL_TIM_GET_COUNTER(&htim2) - enc_l_start);
#if PCB_ENC_LEFT_INVERTED
            delta_l = -delta_l;
#endif
            printf("Enc G : %ld ticks\r\n", (long)delta_l);
            HAL_Delay(1000);

            /* Etape 2 : Moteur droit avant */
            if (auto_cycle) {
                uint16_t enc_r_start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
                printf("\r\n[2/3] Moteur droit avant...\r\n");
                mot_set_signed(0, +duty);

                for (int w = 0; w < 30; w++) {
                    if (HAL_UART_Receive(&huart2, &rx_cmd, 1, 50) == HAL_OK) {
                        auto_cycle = false; break;
                    }
                }
                mot_set_signed(0, 0);

                int16_t delta_r = (int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim4) - enc_r_start);
#if PCB_ENC_RIGHT_INVERTED
                delta_r = -delta_r;
#endif
                printf("Enc D : %d ticks\r\n", (int)delta_r);
                HAL_Delay(1000);
            }

            /* Etape 3 : 2 moteurs ensemble avant */
            if (auto_cycle) {
                printf("\r\n[3/3] 2 moteurs avant...\r\n");
                mot_set_signed(+duty, +duty);

                for (int w = 0; w < 30; w++) {
                    if (HAL_UART_Receive(&huart2, &rx_cmd, 1, 50) == HAL_OK) {
                        auto_cycle = false; break;
                    }
                }
                mot_set_signed(0, 0);
                printf("Fin cycle (pause 3s)\r\n");
                HAL_Delay(3000);
            }
        }

        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
        if (HAL_UART_Receive(&huart2, &rx_cmd, 1, 100) == HAL_OK || !auto_cycle) {
            if (rx_cmd == 'g' || rx_cmd == 'G') {
                printf("Cmd: gauche avant\r\n");
                mot_set_signed(+duty, 0);
            } else if (rx_cmd == 'd' || rx_cmd == 'D') {
                printf("Cmd: droite avant\r\n");
                mot_set_signed(0, +duty);
            } else if (rx_cmd == 'a' || rx_cmd == 'A') {
                printf("Cmd: 2 roues avant\r\n");
                mot_set_signed(+duty, +duty);
            } else if (rx_cmd == 'r' || rx_cmd == 'R') {
                printf("Cmd: 2 roues arriere\r\n");
                mot_set_signed(-duty, -duty);
            } else if (rx_cmd == 's' || rx_cmd == 'S') {
                printf("Cmd: stop\r\n");
                mot_set_signed(0, 0);
            } else if (rx_cmd == 'c' || rx_cmd == 'C') {
                printf("Cmd: relance cycle\r\n");
                mot_set_signed(0, 0);
                auto_cycle = true;
            }
            rx_cmd = 0;
        }

        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    }
}

/* Test 09 : Encodeurs quadrature (TIM2 & TIM4) */
void test_09_encoders(void) {
    printf("\r\n--- Test 09 : Encodeurs (TIM2 G, TIM4 D) ---\r\n");
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    while (1) {
        int32_t cnt_l = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t cnt_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
#if PCB_ENC_LEFT_INVERTED
        cnt_l = -cnt_l;
#endif
#if PCB_ENC_RIGHT_INVERTED
        cnt_r = -cnt_r;
#endif

        printf("ENC L=%ld R=%d\r\n", (long)cnt_l, (int)cnt_r);
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        HAL_Delay(800);
    }
}

/* Test 10 : Anneau NeoPixel 12 LEDs (PB14 / TIM15_CH1 DMA) */
void test_10_neopixel_ring(void) {
    printf("\r\n--- Test 10 : NeoPixel 12 LEDs (PB14) ---\r\n");

    while (1) {
        printf("NP Rouge\r\n");
        np_led_set_all_RGB(50, 0, 0);
        np_led_render();
        HAL_Delay(1000);

        printf("NP Vert\r\n");
        np_led_set_all_RGB(0, 50, 0);
        np_led_render();
        HAL_Delay(1000);

        printf("NP Bleu\r\n");
        np_led_set_all_RGB(0, 0, 50);
        np_led_render();
        HAL_Delay(1000);

        for (int step = 0; step < 12; step++) {
            np_led_set_all_RGB(0, 0, 0);
            np_led_set_RGB(step, 0, 40, 40);
            np_led_render();
            HAL_Delay(100);
        }
    }
}

/* Test 11 : Capteurs I2C3 (MPU-6050, APDS-9960, VL53L0X) */
void test_11_sensors_raw(void) {
    printf("\r\n--- Test 11 : Capteurs I2C3 ---\r\n");

    mpu6050_t imu;
    bool imu_ok = MPU6050_Init(&imu, &hi2c3);

    apds9960_t color;
    apds9960_data_t color_data = {0};
    bool color_ok = APDS9960_Init(&color, &hi2c3);

    vl53l0x_t tof_left, tof_right;
    bool tof_l_ok = false, tof_r_ok = false;

    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_TOF_R_GPIO_Port, XSHUT_TOF_R_Pin, GPIO_PIN_RESET);
    HAL_Delay(15);

    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_SET);
    HAL_Delay(15);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK &&
        VL53L0X_Init(&tof_left, &hi2c3)) {
        VL53L0X_SetAddress(&tof_left, VL53L0X_ADDR_LEFT_8BIT);
        VL53L0X_StartContinuous(&tof_left, 0);
        tof_l_ok = true;
    }

    HAL_GPIO_WritePin(XSHUT_TOF_R_GPIO_Port, XSHUT_TOF_R_Pin, GPIO_PIN_SET);
    HAL_Delay(15);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK &&
        VL53L0X_Init(&tof_right, &hi2c3)) {
        VL53L0X_SetAddress(&tof_right, VL53L0X_ADDR_RIGHT_8BIT);
        VL53L0X_StartContinuous(&tof_right, 0);
        tof_r_ok = true;
    }

    printf("Init IMU=%d Color=%d ToF_L=%d ToF_R=%d\r\n",
           imu_ok ? 1 : 0, color_ok ? 1 : 0, tof_l_ok ? 1 : 0, tof_r_ok ? 1 : 0);

    while (1) {
        if (imu_ok) MPU6050_Update(&imu, 1.0f);
        if (color_ok) (void)APDS9960_ReadData(&color, &color_data);
        uint16_t dl = tof_l_ok ? VL53L0X_ReadDistanceContinuous(&tof_left, NULL) : 0;
        uint16_t dr = tof_r_ok ? VL53L0X_ReadDistanceContinuous(&tof_right, NULL) : 0;

        printf("GyZ:%5.1f | RGB:(%u,%u,%u) Px:%u | ToF L:%u R:%u mm\r\n", // @suppress("Float formatting support")
               imu_ok ? imu.gyro_z : 0.0f,
               color_data.red, color_data.green, color_data.blue, color_data.proximity,
               dl, dr);

        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        HAL_Delay(1000);
    }
}

/* Test 12 : Batterie I2C1 (Jauge BQ27220 & Chargeur BQ25896) */
void test_12_power_battery(void) {
    printf("\r\n--- Test 12 : Batterie & Chargeur (1=Boost ON, 0=Boost OFF) ---\r\n");

    bq27220_t gauge;
    bq25896_t charger;

    bool gauge_ok   = (BQ27220_Init(&gauge, &hi2c1) == HAL_OK);
    bool charger_ok = (BQ25896_Init(&charger, &hi2c1) == HAL_OK);

    if (charger_ok) {
        (void)BQ25896_DisableWatchdog(&charger);
        (void)BQ25896_SetBoostMode(&charger, s_enable_5v_boost);
    }

    printf("Init FG=%d Chg=%d Boost=%d\r\n",
           gauge_ok ? 1 : 0, charger_ok ? 1 : 0, s_enable_5v_boost ? 1 : 0);

    uint8_t rx_cmd = 0;

    while (1) {
        /* Attente 2s avec lecture commandes '1'/'0' */
        for (int i = 0; i < 40; i++) {
            __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
            if (HAL_UART_Receive(&huart2, &rx_cmd, 1, 50) == HAL_OK && charger_ok) {
                if (rx_cmd == '1') {
                    s_enable_5v_boost = true;
                    (void)BQ25896_SetBoostMode(&charger, true);
                    printf("Boost 5V ON\r\n");
                    break;
                } else if (rx_cmd == '0') {
                    s_enable_5v_boost = false;
                    (void)BQ25896_SetBoostMode(&charger, false);
                    printf("Boost 5V OFF\r\n");
                    break;
                }
            }
        }

        if (!gauge_ok)   gauge_ok   = (BQ27220_Init(&gauge, &hi2c1) == HAL_OK);
        if (!charger_ok) {
            charger_ok = (BQ25896_Init(&charger, &hi2c1) == HAL_OK);
            if (charger_ok) {
                (void)BQ25896_DisableWatchdog(&charger);
                (void)BQ25896_SetBoostMode(&charger, s_enable_5v_boost);
            }
        }

        if (gauge_ok)   BQ27220_Update(&gauge);
        if (charger_ok) {
            (void)BQ25896_ResetWatchdog(&charger);
            BQ25896_Update(&charger);
        }

        printf("FG: %umV %u%% %dmA | Chg: VBUS=%umV VBAT=%umV VSYS=%umV Boost=%d\r\n",
               gauge_ok ? gauge.voltage_mv : 0,
               gauge_ok ? gauge.soc_percent : 0,
               gauge_ok ? (int)gauge.current_ma : 0,
               charger_ok ? charger.vbus_mv : 0,
               charger_ok ? charger.vbat_mv : 0,
               charger_ok ? charger.vsys_mv : 0,
               (charger_ok && charger.boost_enabled) ? 1 : 0);

        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    }
}

/* Test 13 : LiDAR YDLIDAR X2 (USART1 DMA + PWM PA12) */
void test_13_lidar_x2(void) {
    printf("\r\n--- Test 13 : LiDAR X2 (USART1 PC5, PWM PA12) ---\r\n");

    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 65);

    ydlidar_x2_t lidar;
    YDLIDAR_X2_Init(&lidar, &huart1);

    uint32_t last_print = HAL_GetTick();

    while (1) {
        YDLIDAR_X2_Process(&lidar);

        if (HAL_GetTick() - last_print >= 1000) {
            last_print = HAL_GetTick();
            printf("LiDAR %.1fHz (%lu tours) | 0:%u 90:%u 180:%u 270:%u mm\r\n", // @suppress("Float formatting support")
                   lidar.scan_frequency_hz, lidar.laps_count,
                   YDLIDAR_X2_GetDistance(&lidar, 0),
                   YDLIDAR_X2_GetDistance(&lidar, 90),
                   YDLIDAR_X2_GetDistance(&lidar, 180),
                   YDLIDAR_X2_GetDistance(&lidar, 270));

            HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        }
    }
}

/* Point d'entree du selecteur de test */
void pcb_run_test_ex(int scenario, bool enable_5v_boost) {
    s_enable_5v_boost = enable_5v_boost;

    if (g_hse_clock_failed && scenario != TEST_01_HSI_BLINK) {
        printf("\r\nWARN: HSE absent -> HSI 170 MHz\r\n");
    }

    if (scenario != TEST_01_HSI_BLINK &&
        scenario != TEST_02_HSE_BLINK &&
        scenario != TEST_12_POWER_BATTERY) {
        pcb_configure_5v_rail(s_enable_5v_boost);
    }

    switch (scenario) {
        case TEST_00_I2C_SCANNER:    test_00_i2c_scanner();    break;
        case TEST_01_HSI_BLINK:      test_01_hsi_blink();      break;
        case TEST_02_HSE_BLINK:      test_02_hse_blink();      break;
        case TEST_03_BUTTONS_LEDS:   test_03_buttons_leds();   break;
        case TEST_04_UART_VCP_BLE:   test_04_uart_vcp_ble();   break;
        case TEST_05_OLED_DISPLAY:   test_05_oled_display();   break;
        case TEST_06_BUZZER_PWM:     test_06_buzzer_pwm();     break;
        case TEST_07_SERVO_PINCE:    test_07_servo_pince();    break;
        case TEST_08_MOTORS_DRV8833: test_08_motors_drv8833(); break;
        case TEST_09_ENCODERS:       test_09_encoders();       break;
        case TEST_10_NEOPIXEL_RING:  test_10_neopixel_ring();  break;
        case TEST_11_SENSORS_RAW:    test_11_sensors_raw();    break;
        case TEST_12_POWER_BATTERY:  test_12_power_battery();  break;
        case TEST_13_LIDAR_X2:       test_13_lidar_x2();       break;
        default:
            printf("ERR: Scenario %d inconnu\r\n", scenario);
            while (1) {
                HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
                HAL_Delay(100);
            }
    }
}

void pcb_run_test(int scenario) {
    pcb_run_test_ex(scenario, (PCB_ENABLE_5V_BOOST_ON_BATTERY != 0));
}
