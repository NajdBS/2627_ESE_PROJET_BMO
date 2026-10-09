/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BTN_Pin GPIO_PIN_13
#define USER_BTN_GPIO_Port GPIOC
#define USER_BTN_EXTI_IRQn EXTI15_10_IRQn
#define INT_TOF_R_Pin GPIO_PIN_14
#define INT_TOF_R_GPIO_Port GPIOC
#define INT_TOF_R_EXTI_IRQn EXTI15_10_IRQn
#define COLOR_INT_Pin GPIO_PIN_15
#define COLOR_INT_GPIO_Port GPIOC
#define COLOR_INT_EXTI_IRQn EXTI15_10_IRQn
#define MOT_nFAULT_Pin GPIO_PIN_0
#define MOT_nFAULT_GPIO_Port GPIOC
#define MOT_nFAULT_EXTI_IRQn EXTI0_IRQn
#define XSHUT_TOF_L_Pin GPIO_PIN_2
#define XSHUT_TOF_L_GPIO_Port GPIOC
#define XSHUT_TOF_R_Pin GPIO_PIN_3
#define XSHUT_TOF_R_GPIO_Port GPIOC
#define ENC_L_A_Pin GPIO_PIN_0
#define ENC_L_A_GPIO_Port GPIOA
#define ENC_L_B_Pin GPIO_PIN_1
#define ENC_L_B_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define MOT_nSLEEP_Pin GPIO_PIN_4
#define MOT_nSLEEP_GPIO_Port GPIOA
#define LED_STATUS_Pin GPIO_PIN_5
#define LED_STATUS_GPIO_Port GPIOA
#define MOT_L_IN1_Pin GPIO_PIN_6
#define MOT_L_IN1_GPIO_Port GPIOA
#define MOT_L_IN2_Pin GPIO_PIN_7
#define MOT_L_IN2_GPIO_Port GPIOA
#define LIDAR_TX_NC_Pin GPIO_PIN_4
#define LIDAR_TX_NC_GPIO_Port GPIOC
#define LIDAR_RX_Pin GPIO_PIN_5
#define LIDAR_RX_GPIO_Port GPIOC
#define MOT_R_IN1_Pin GPIO_PIN_0
#define MOT_R_IN1_GPIO_Port GPIOB
#define MOT_R_IN2_Pin GPIO_PIN_1
#define MOT_R_IN2_GPIO_Port GPIOB
#define INT_TOF_L_Pin GPIO_PIN_10
#define INT_TOF_L_GPIO_Port GPIOB
#define INT_TOF_L_EXTI_IRQn EXTI15_10_IRQn
#define EXT_PB13_Pin GPIO_PIN_13
#define EXT_PB13_GPIO_Port GPIOB
#define NEOPIXEL_Pin GPIO_PIN_14
#define NEOPIXEL_GPIO_Port GPIOB
#define SERVO_PINCE_Pin GPIO_PIN_6
#define SERVO_PINCE_GPIO_Port GPIOC
#define SENS_SCL_Pin GPIO_PIN_8
#define SENS_SCL_GPIO_Port GPIOC
#define SENS_SDA_Pin GPIO_PIN_9
#define SENS_SDA_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_8
#define OLED_SDA_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_9
#define OLED_SCL_GPIO_Port GPIOA
#define BLE_EN_Pin GPIO_PIN_10
#define BLE_EN_GPIO_Port GPIOA
#define PWR_BUTTON_Pin GPIO_PIN_11
#define PWR_BUTTON_GPIO_Port GPIOA
#define PWR_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define LIDAR_M_CTR_Pin GPIO_PIN_12
#define LIDAR_M_CTR_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_10
#define BLE_TX_GPIO_Port GPIOC
#define BLE_RX_Pin GPIO_PIN_11
#define BLE_RX_GPIO_Port GPIOC
#define CHARGER_INT_Pin GPIO_PIN_12
#define CHARGER_INT_GPIO_Port GPIOC
#define CHARGER_INT_EXTI_IRQn EXTI15_10_IRQn
#define FG_GPOUT_Pin GPIO_PIN_2
#define FG_GPOUT_GPIO_Port GPIOD
#define MPU_INT_Pin GPIO_PIN_4
#define MPU_INT_GPIO_Port GPIOB
#define MPU_INT_EXTI_IRQn EXTI4_IRQn
#define BUZZER_PWM_Pin GPIO_PIN_5
#define BUZZER_PWM_GPIO_Port GPIOB
#define ENC_R_A_Pin GPIO_PIN_6
#define ENC_R_A_GPIO_Port GPIOB
#define ENC_R_B_Pin GPIO_PIN_7
#define ENC_R_B_GPIO_Port GPIOB
#define BAT_SCL_Pin GPIO_PIN_8
#define BAT_SCL_GPIO_Port GPIOB
#define BAT_SDA_Pin GPIO_PIN_9
#define BAT_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
