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
#include "bsp.h"
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
/* Global hardware is managed via g_bsp (defined in bsp.h) */
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

void leds_process(void)
{
	static uint8_t state = 0;

	HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

	switch (state) {
	case 0:	/* RED */
		np_led_set_all_RGB(30, 0, 0);
		np_led_render();
		state = 1;
		break;
	case 1:	/* GREEN */
		np_led_set_all_RGB(0, 30, 0);
		np_led_render();
		state = 2;
		break;
	case 2:	/* BLUE */
		np_led_set_all_RGB(0, 0, 30);
		np_led_render();
		state = 3;
		break;
	case 3:	/* WHITE */
		np_led_set_all_RGB(25, 25, 25);
		np_led_render();
		state = 4;
		break;
	case 4:	/* YELLOW */
		np_led_set_all_RGB(30, 20, 0);
		np_led_render();
		state = 5;
		break;
	case 5:	/* PURPLE */
		np_led_set_all_RGB(30, 0, 30);
		np_led_render();
		state = 6;
		break;
	case 6:	/* CYAN */
		np_led_set_all_RGB(0, 30, 30);
		np_led_render();
		state = 7;
		break;
	case 7:	/* OFF */
		np_led_clear();
		state = 0;
		break;
	default:
		state = 0;
		break;
	}
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
  MX_I2C1_Init();
  MX_TIM17_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  BSP_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* uint32_t last_buzzer_demo_tick = 0; */
  /* uint8_t buzzer_step = 0; */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* Status LED heartbeat (500 ms) */
    static uint32_t last_led_heartbeat = 0;
    if (now - last_led_heartbeat >= 500) {
        last_led_heartbeat = now;
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    }

    /* BSP Background Tasks (IMU integration, Buzzer melody sequencer) */
    BSP_Update(0.01f);

    /* Dual VL53L0X ToF Distance Sensors loop (every 100 ms) */
    static uint32_t last_tof_tick = 0;
    if (now - last_tof_tick >= 100) {
        last_tof_tick = now;

        uint16_t dist_left = 65535;
        uint16_t dist_right = 65535;

        if (g_bsp.tof_left_ok) {
            dist_left = VL53L0X_ReadDistanceContinuous(&g_bsp.tof_left, NULL);
        }
        if (g_bsp.tof_right_ok) {
            dist_right = VL53L0X_ReadDistanceContinuous(&g_bsp.tof_right, NULL);
        }

        printf("[ToF Dual] Left (0x30): ");
        if (g_bsp.tof_left_ok && dist_left < 8190) {
            printf("%4u mm", dist_left);
        } else if (!g_bsp.tof_left_ok) {
            printf("DISCON ");
        } else {
            printf(" OUT   ");
        }

        printf("  |  Right (0x31): ");
        if (g_bsp.tof_right_ok && dist_right < 8190) {
            printf("%4u mm", dist_right);
        } else if (!g_bsp.tof_right_ok) {
            printf("DISCON ");
        } else {
            printf(" OUT   ");
        }

        // Optional 3rd Center ToF (EXT_PB13 -> 0x32) - Uncomment to test
        /*
        uint16_t dist_center = 65535;
        if (g_bsp.tof_center_ok) {
            dist_center = VL53L0X_ReadDistanceContinuous(&g_bsp.tof_center, NULL);
        }
        printf("  |  Center (0x32): ");
        if (g_bsp.tof_center_ok && dist_center < 8190) {
            printf("%4u mm", dist_center);
        } else if (!g_bsp.tof_center_ok) {
            printf("DISCON ");
        } else {
            printf(" OUT   ");
        }
        */
        printf("\r\n");
    }

    /* Buzzer Melody Demo: Cycle through BMO sound effects (commented out for ToF test) */
    /*
    if (g_bsp.buzzer_ok && (now - last_buzzer_demo_tick >= 2500)) {
        last_buzzer_demo_tick = now;

        switch (buzzer_step) {
        case 0:
            printf("[Buzzer Demo] 1. BMO_SOUND_BOOT (Power-on Arpeggio: C5-E5-G5-C6)\r\n");
            BUZZER_PlaySound(&g_bsp.buzzer, BMO_SOUND_BOOT);
            buzzer_step = 1;
            break;
        case 1:
            printf("[Buzzer Demo] 2. BMO_SOUND_CAN_FOUND (Positive Alert: G5-C6)\r\n");
            BUZZER_PlaySound(&g_bsp.buzzer, BMO_SOUND_CAN_FOUND);
            buzzer_step = 2;
            break;
        case 2:
            printf("[Buzzer Demo] 3. BMO_SOUND_GRIP_SUCCESS (Victory Fanfare: C5-G5-C6-E6)\r\n");
            BUZZER_PlaySound(&g_bsp.buzzer, BMO_SOUND_GRIP_SUCCESS);
            buzzer_step = 3;
            break;
        case 3:
            printf("[Buzzer Demo] 4. BMO_SOUND_CAN_DROPPED (Mission Complete: C6-G5)\r\n");
            BUZZER_PlaySound(&g_bsp.buzzer, BMO_SOUND_CAN_DROPPED);
            buzzer_step = 4;
            break;
        case 4:
            printf("[Buzzer Demo] 5. BMO_SOUND_ERROR (Descending Low Buzz: F3-D3)\r\n");
            BUZZER_PlaySound(&g_bsp.buzzer, BMO_SOUND_ERROR);
            buzzer_step = 5;
            break;
        case 5:
            printf("[Buzzer Demo] 6. BMO_SOUND_BEEP_SHORT (Crisp UI Click 2.5 kHz)\r\n");
            BUZZER_PlaySound(&g_bsp.buzzer, BMO_SOUND_BEEP_SHORT);
            buzzer_step = 6;
            break;
        case 6:
            printf("[Buzzer Demo] 7. BMO_SOUND_MATCH_START (Referee Whistle 3.0 kHz)\r\n");
            BUZZER_PlaySound(&g_bsp.buzzer, BMO_SOUND_MATCH_START);
            buzzer_step = 0;
            break;
        }
    }
    */

    /* MPU-6050 IMU integration loop (commented for buzzer test) */
    /*
    static uint32_t last_imu_tick = 0;
    static uint32_t last_imu_print_tick = 0;
    if (g_bsp.imu_ok && (now - last_imu_tick >= 20)) {
        float dt = (float)(now - last_imu_tick) / 1000.0f;
        last_imu_tick = now;
        BSP_Update(dt);

        if (now - last_imu_print_tick >= 200) {
            last_imu_print_tick = now;
            printf("[IMU] Yaw: %7.2f deg | Rate: %6.2f deg/s | Accel[g]: X=%5.2f Y=%5.2f Z=%5.2f | T: %.1f C\r\n",
                   g_bsp.imu.yaw, (g_bsp.imu.gyro_z - g_bsp.imu.gyro_z_offset),
                   g_bsp.imu.accel_x, g_bsp.imu.accel_y, g_bsp.imu.accel_z,
                   g_bsp.imu.temperature);
        }
    }
    */

    /* NeoPixel 12-LED Ring Process (commented for buzzer test) */
    /*
    static uint32_t last_led_tick = 0;
    if (g_bsp.neopixel_ok && (now - last_led_tick >= 500)) {
        last_led_tick = now;
        leds_process();
    }
    */

    /* Power Telemetry (BQ27220 Fuel Gauge & BQ25896 Charger) every 1000 ms */
    /*
    static uint32_t last_power_tick = 0;
    if (now - last_power_tick >= 1000) {
        last_power_tick = now;
        if (g_bsp.fuel_gauge_ok) {
            BQ27220_Update(&g_bsp.fuel_gauge);
            printf("[Power] BQ27220 Fuel Gauge -> SOC: %3u%% | V: %4u mV | I: %5d mA | T: %4.1f C | Rem: %4u mAh\r\n",
                   g_bsp.fuel_gauge.soc_percent,
                   g_bsp.fuel_gauge.voltage_mv,
                   g_bsp.fuel_gauge.current_ma,
                   g_bsp.fuel_gauge.temperature_c,
                   g_bsp.fuel_gauge.remaining_capacity_mah);
        }
        if (g_bsp.charger_ok) {
            BQ25896_Update(&g_bsp.charger);
            const char *chrg_str = (g_bsp.charger.charge_status == BQ_CHRG_DONE) ? "DONE" :
                                   (g_bsp.charger.charge_status == BQ_CHRG_FAST_CHARGE) ? "FAST" :
                                   (g_bsp.charger.charge_status == BQ_CHRG_PRECHARGE) ? "PRE" : "IDLE";
            printf("[Power] BQ25896 Charger    -> VBUS: %4u mV | VBAT: %4u mV | Ichg: %4u mA | Chg: %s | PG: %d\r\n",
                   g_bsp.charger.vbus_mv,
                   g_bsp.charger.vbat_mv,
                   g_bsp.charger.ichg_ma,
                   chrg_str,
                   g_bsp.charger.power_good);
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
