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

/* Helper to convert angle into intuitive 8-cardinal sector label */
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

/* Clean, aligned, professional LiDAR telemetry dashboard output */
static void Print_Lidar_Telemetry(const ydlidar_x2_t *lidar)
{
    /* Find nearest detected obstacle (excluding blind zone < 120 mm) */
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

    /* Line 1: Health & Motor spin rate */
    printf("[LIDAR X2] %4.1f Hz | Laps: %5lu | Pkts: %6lu (Err: %lu)\r\n",
           lidar->scan_frequency_hz,
           lidar->laps_count,
           lidar->valid_packets_count,
           lidar->checksum_errors_count);

    /* Line 2: Closest obstacle + 4 cardinal distances with fixed column alignment */
    if (min_dist != 0xFFFF) {
        printf("  >> Nearest: %4u mm @ %3u deg [%-11s] | Fwd: %4u mm | Rgt: %4u mm | Bck: %4u mm | Lft: %4u mm\r\n",
               min_dist, min_angle, Get_Sector_Name(min_angle), fwd, rgt, bck, lft);
    } else {
        printf("  >> Nearest: ---- mm @ --- deg [No target  ] | Fwd: %4u mm | Rgt: %4u mm | Bck: %4u mm | Lft: %4u mm\r\n",
               fwd, rgt, bck, lft);
    }
}
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
  /* USER CODE BEGIN 2 */
  printf("\r\n========================================\r\n");
  printf("   BMO SYSTEM - STM32G474 FIRMWARE     \r\n");
  printf("   OLED (I2C2) + YDLIDAR X2 (USART1 DMA)\r\n");
  printf("========================================\r\n");

  /* Initialize YDLIDAR X2 on USART1 with Circular DMA */
  YDLIDAR_X2_Init(&g_lidar, &huart1);

  /* Start LiDAR motor PWM at 10 kHz (35% duty cycle = 6 Hz nominal speed) */
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 35);
  printf("YDLIDAR X2 Motor PWM started on PB14 (TIM15_CH1) @ 10 kHz, 35%% Duty\r\n");

  // Probe I2C2 to verify OLED display presence before initialization
  if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c2, SSD1306_I2C_ADDR, 3, 1000)) {
  	display_present = 1;
  	printf("BMO OLED Display initialized on I2C2\r\n");
  	BMO_Screen_Init();
  	BMO_Screen_SetFace(BMO_FACE_LIDAR_RADAR);
  } else {
  	printf("BMO OLED Display not detected on I2C2\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_anim_tick = 0;
  uint32_t last_mood_tick = 0;
  uint32_t last_lidar_tick = 0;
  uint8_t demo_mood = (uint8_t)BMO_FACE_LIDAR_RADAR;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* Process incoming LiDAR DMA bytes continuously (non-blocking) */
    YDLIDAR_X2_Process(&g_lidar);

    /* Telemetry output to VCP terminal every 500 ms */
    if (now - last_lidar_tick >= 500) {
        last_lidar_tick = now;
        Print_Lidar_Telemetry(&g_lidar);

        if (display_present) {
            /* Find closest obstacle for OLED radar */
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
    }

    if (display_present) {
    	// Refresh facial animation at 25 FPS (every 40 ms via DMA)
    	if (now - last_anim_tick >= 40) {
    		last_anim_tick = now;
    		BMO_Screen_Update(now);
    	}

    	// Expression showcase: cycle moods every 5 seconds (8 modes, including 2D Radar)
    	if (now - last_mood_tick >= 5000) {
    		last_mood_tick = now;
    		demo_mood = (demo_mood + 1) % 8;
    		BMO_Screen_SetFace((bmo_face_t)demo_mood);

    		if (demo_mood == BMO_FACE_TELEMETRY) {
    			BMO_Screen_SetTelemetry(3.92f, 0, "LIDAR 6Hz");
    		}
    	}
    }
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
