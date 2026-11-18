# Tests Materiels PCB - Robot BMO (STM32G474)

Ce dossier regroupe les fichiers de validation unitaire du PCB (`pcb_tests.h` et `pcb_tests.c`), permettant de tester independamment chaque sous-ensemble materiel de la carte STM32G474 avant de lancer l'application FreeRTOS.

## Liste des scenarios (`TEST_00` a `TEST_13`)

| Scenario | Description | Peripheriques / Broches |
| :--- | :--- | :--- |
| `TEST_00_I2C_SCANNER` | Scan des 3 bus I2C (power, OLED, capteurs) | `I2C1` (PB8/PB9), `I2C2` (PA8/PA9), `I2C3` (PC8/PC9) |
| `TEST_01_HSI_BLINK` | Horloge interne HSI (16 MHz) et clignotement LED | `LED_STATUS` (PA5) |
| `TEST_02_HSE_BLINK` | Quartz externe HSE (PF0/PF1) + PLL 170 MHz | `HSE`, `LED_STATUS` (PA5) |
| `TEST_03_BUTTONS_LEDS` | Lecture des boutons et entrees d'etat | `USER_BTN` (PC13), `PWR_BTN` (PA11), `MOT_nFAULT`, `CHARGER_INT`, `FG_GPOUT` |
| `TEST_04_UART_VCP_BLE` | Liaison serie USB ST-Link et module BLE | `USART2` (PA2/PA3), `USART3` (PC10/PC11), `BLE_EN` (PA10) |
| `TEST_05_OLED_DISPLAY` | Affichage sur ecran OLED SSD1306 | `I2C2` (0x3C) |
| `TEST_06_BUZZER_PWM` | Generation de notes PWM sur le buzzer passif | `TIM17_CH1` (PB5) |
| `TEST_07_SERVO_PINCE` | Balayage 0 / 90 / 180 deg du servomoteur pince | `TIM8_CH1` (PC6) |
| `TEST_08_MOTORS_DRV8833` | Test moteurs DC et controle clavier + delta encodeurs | `TIM3_CH1..4` (PA6/PA7, PB0/PB1), `MOT_nSLEEP` (PA4) |
| `TEST_09_ENCODERS` | Lecture temps reel des encodeurs en quadrature | `TIM2` (PA0/PA1), `TIM4` (PB6/PB7) |
| `TEST_10_NEOPIXEL_RING` | Test de l'anneau 12 LEDs WS2812B | `TIM15_CH1` DMA (PB14) |
| `TEST_11_SENSORS_RAW` | Lecture brute IMU, couleur/proximite et ToF gauche/droit | `I2C3`: `MPU6050` (0x68), `APDS9960` (0x39), `VL53L0X` (0x30/0x31) |
| `TEST_12_POWER_BATTERY` | Lecture jauge batterie + chargeur et commande Boost 5V (`1`/`0`) | `I2C1`: `BQ27220` (0x55), `BQ25896` (0x6B) |
| `TEST_13_LIDAR_X2` | Reception trames LiDAR YDLIDAR X2 et commande moteur | `USART1` DMA (PC5), `TIM16_CH1` (PA12) |

## Options de configuration (`pcb_tests.h`)

* `PCB_ENABLE_5V_BOOST_ON_BATTERY` (`1` par defaut) : active le Boost 5V du `BQ25896` (`REG03` `OTG_CONFIG = 1`, `REG0A = 0x72`) et desactive le watchdog I2C 40 s (`REG07` bits 5:4 = `00`) pour alimenter le rail 5V sur batterie.
* `PCB_MOTOR_LEFT_INVERTED` (`1`), `PCB_MOTOR_RIGHT_INVERTED` (`0`) : correction logicielle du sens de rotation des moteurs.
* `PCB_ENC_LEFT_INVERTED` (`1`), `PCB_ENC_RIGHT_INVERTED` (`0`) : correction logicielle du sens de comptage des encodeurs.

## Utilisation dans `main.c`

Inclure `pcb_tests.h`, definir le scenario voulu et appeler `pcb_run_test(TEST_SCENARIO)` avant `vTaskStartScheduler()` (apres l'initialisation des peripheriques CubeMX) :

```c
#include "pcb_tests.h"

#define TEST_SCENARIO   TEST_08_MOTORS_DRV8833

/* Dans main(), apres MX_*_Init() : */
pcb_run_test(TEST_SCENARIO);
```

La sortie serie `printf` est redirigee vers `USART2` (ST-Link VCP, 115200 bauds).
