/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bmo_screen.h"
#include "ydlidar_x2.h"
#include "drv8833.h"
#include "encoder.h"
#include "vl53l0x.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t display_present = 0;
ydlidar_x2_t g_lidar;

/* DRV8833 Motors */
drv8833_t g_motor_left;
drv8833_t g_motor_right;

/* Quadrature Encoders */
encoder_t g_enc_left;
encoder_t g_enc_right;

/* VL53L0X Distance Sensor */
vl53l0x_t g_tof;
bool g_tof_present = false;

/* Gripper Servo (TIM8_CH1 on PC6) */
servo_t g_gripper_servo;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
  return len;
}

/*
static const char* Get_Sector_Name(uint16_t angle)
{
    if (angle >= 338 || angle < 23)   return "Front";
    if (angle >= 23  && angle < 68)   return "Front-Right";
    if (angle >= 68  && angle < 113)  return "Right";
    if (angle >= 113 && angle < 158)  return "Rear-Right";
    if (angle >= 158 && angle < 203)  return "Rear";
    if (angle >= 203 && angle < 248)  return "Rear-Left";
    if (angle >= 248 && angle < 293)  return "Left";
    return "Front-Left";
}

static void Print_Lidar_Telemetry(const ydlidar_x2_t *lidar)
{
    uint16_t min_dist = 0xFFFF;
    uint16_t min_angle = 0;
    for (uint16_t a = 0; a < 360; a++) {
        uint16_t d = lidar->distances[a];
        if (d >= 120 && d < min_dist) {
            min_dist = d;
            min_angle = a;
        }
    }

    uint16_t fwd = YDLIDAR_X2_GetDistance(lidar, 0);
    uint16_t rgt = YDLIDAR_X2_GetDistance(lidar, 90);
    uint16_t bck = YDLIDAR_X2_GetDistance(lidar, 180);
    uint16_t lft = YDLIDAR_X2_GetDistance(lidar, 270);

    printf("[LIDAR X2] %4.1f Hz | Laps: %5lu | Pkts: %6lu (Err: %lu)\r\n",
           lidar->scan_frequency_hz,
           lidar->laps_count,
           lidar->valid_packets_count,
           lidar->checksum_errors_count);

    if (min_dist != 0xFFFF) {
        printf("  >> Nearest: %4u mm @ %3u deg [%-11s] | Fwd: %4u mm | Rgt: %4u mm | Bck: %4u mm | Lft: %4u mm\r\n",
               min_dist, min_angle, Get_Sector_Name(min_angle), fwd, rgt, bck, lft);
    } else {
        printf("  >> Nearest: ---- mm @ --- deg [No target  ] | Fwd: %4u mm | Rgt: %4u mm | Bck: %4u mm | Lft: %4u mm\r\n",
               fwd, rgt, bck, lft);
    }
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n========================================\r\n");
  printf("   BMO SYSTEM - ENCODER TEST BENCH      \r\n");
  printf("========================================\r\n");

  /*
  YDLIDAR_X2_Init(&g_lidar, &huart1);

  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 35);
  printf("YDLIDAR X2 Motor PWM started on PB14 (TIM15_CH1) @ 10 kHz, 35%% Duty\r\n");

  if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c2, SSD1306_I2C_ADDR, 3, 1000)) {
  	display_present = 1;
  	printf("BMO OLED Display initialized on I2C2\r\n");
  	BMO_Screen_Init();
  	BMO_Screen_SetFace(BMO_FACE_LIDAR_RADAR);
  } else {
  	printf("BMO OLED Display not detected on I2C2\r\n");
  }

  DRV8833_Init(&g_motor_left, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, false);
  DRV8833_Init(&g_motor_right, &htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, true);
  DRV8833_Coast(&g_motor_left);
  DRV8833_Coast(&g_motor_right);
  printf("DRV8833 Motors initialized on TIM3 (CH1..CH4) @ 20 kHz PWM\r\n");
  */

  /* Initialize Quadrature Encoders (TIM2 32-bit & TIM4 16-bit) */
  /*
  Encoder_Init(&g_enc_left, &htim2, ENCODER_DEFAULT_CPR_WHEEL, ENCODER_DEFAULT_WHEEL_DIAM_MM, false);
  Encoder_Init(&g_enc_right, &htim4, ENCODER_DEFAULT_CPR_WHEEL, ENCODER_DEFAULT_WHEEL_DIAM_MM, true);
  printf("Quadrature Encoders initialized: Left=TIM2 (PA0/PA1), Right=TIM4 (PB6/PB7)\r\n");
  */

  /* Initialize ToF Sensor (VL53L0X on I2C3 PC8/PC9) */
  printf("Checking I2C3 for VL53L0X ToF sensor...\r\n");
  if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
      printf("[ToF] Device detected at 0x29 (0x52)!\r\n");
      if (VL53L0X_Init(&g_tof, &hi2c3)) {
          VL53L0X_StartContinuous(&g_tof, 0);
          g_tof_present = true;
          printf("[ToF] VL53L0X initialized & continuous ranging started!\r\n");
      } else {
          printf("[ToF] VL53L0X init failed!\r\n");
      }
  } else {
      printf("[ToF] No device responding on I2C3 (PC8/PC9)\r\n");
  }

  /* Initialize Gripper Servo (PC6 on TIM8_CH1 @ 50 Hz PWM) */
  printf("Initializing Gripper Servo on PC6 (TIM8_CH1)...\r\n");
  if (SERVO_Init(&g_gripper_servo, &htim8, TIM_CHANNEL_1) == HAL_OK) {
      printf("[Servo] Gripper servo initialized @ 50 Hz PWM on PC6 (Released / Open)!\r\n");
  } else {
      printf("[Servo] Failed to initialize servo!\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
  uint32_t last_anim_tick = 0;
  uint32_t last_mood_tick = 0;
  */
  uint32_t last_tof_tick = 0;
  /*
  uint32_t last_encoder_tick = 0;
  uint8_t demo_mood = (uint8_t)BMO_FACE_LIDAR_RADAR;
  */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /*
    YDLIDAR_X2_Process(&g_lidar);
    */

    /* Periodic encoder update (every 50 ms) */
    /*
    if (now - last_encoder_tick >= 50) {
        float dt = (float)(now - last_encoder_tick) / 1000.0f;
        last_encoder_tick = now;
        Encoder_Update(&g_enc_left, dt);
        Encoder_Update(&g_enc_right, dt);
    }
    */

    /* Telemetry output to VCP terminal every 100 ms */
    if (now - last_tof_tick >= 100) {
        last_tof_tick = now;

        if (g_tof_present) {
            uint16_t dist_mm = VL53L0X_ReadDistanceContinuous(&g_tof, NULL);
            if (dist_mm != 65535) {
                printf("[ToF VL53L0X] Distance: %4u mm (%4.1f cm)\r\n", dist_mm, (float)dist_mm / 10.0f);
            } else {
                printf("[ToF VL53L0X] Out of range / Timeout\r\n");
            }
        }
    }

    /* Servo Gripper Test Cycle (every 2500 ms) */
    static uint32_t last_servo_tick = 0;
    static uint8_t servo_step = 0;
    if (now - last_servo_tick >= 2500) {
        last_servo_tick = now;
        switch (servo_step) {
            case 0:
                printf("\r\n>>> [Gripper] Step 1: RELEASE Can (Open 0 deg / 1000 us) <<<\r\n");
                SERVO_Release(&g_gripper_servo);
                servo_step = 1;
                break;
            case 1:
                printf("\r\n>>> [Gripper] Step 2: GRIP Can (Clamp 140 deg / 1777 us) <<<\r\n");
                SERVO_Grip(&g_gripper_servo);
                servo_step = 2;
                break;
            case 2:
                printf("\r\n>>> [Gripper] Step 3: RELEASE Can (Open 0 deg / 1000 us) <<<\r\n");
                SERVO_Release(&g_gripper_servo);
                servo_step = 0;
                break;
        }
    }

        /*
        if (display_present) {
            uint16_t min_dist = 0xFFFF;
            uint16_t min_angle = 0;
            for (uint16_t a = 0; a < 360; a++) {
                uint16_t d = g_lidar.distances[a];
                if (d >= 120 && d < min_dist) {
                    min_dist = d;
                    min_angle = a;
                }
            }
            uint16_t fwd = YDLIDAR_X2_GetDistance(&g_lidar, 0);
            uint16_t rgt = YDLIDAR_X2_GetDistance(&g_lidar, 90);
            uint16_t bck = YDLIDAR_X2_GetDistance(&g_lidar, 180);
            uint16_t lft = YDLIDAR_X2_GetDistance(&g_lidar, 270);

            BMO_Screen_SetLidarData(g_lidar.scan_frequency_hz, fwd, rgt, bck, lft, min_dist, min_angle);
        }
        */

    /*
    if (display_present) {
    	if (now - last_anim_tick >= 40) {
    		last_anim_tick = now;
    		BMO_Screen_Update(now);
    	}

    	if (now - last_mood_tick >= 5000) {
    		last_mood_tick = now;
    		demo_mood = (demo_mood + 1) % 8;
    		BMO_Screen_SetFace((bmo_face_t)demo_mood);

    		if (demo_mood == BMO_FACE_TELEMETRY) {
    			BMO_Screen_SetTelemetry(3.92f, 0, "LIDAR 6Hz");
    		}
    	}
    }
    */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
