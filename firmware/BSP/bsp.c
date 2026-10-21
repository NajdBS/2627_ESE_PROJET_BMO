/*
 * bsp.c
 *
 * Board Support Package implementation for BMO Robot.
 * Encapsule l'etat materiel dans le singleton prive `bsp_ctx`, gere la tache
 * temps-reel d'acquisition `bsp_task` et la tache d'affichage DMA `bsp_display_task`.
 */

#include "bsp.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "app.h"
#include "debounce.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "drv8833.h"
#include "encoder.h"
#include "servo.h"
#include "neopixel.h"
#include "vl53l0x.h"
#include "apds9960.h"
#include "mpu6050.h"
#include "ydlidar_x2.h"
#include "buzzer.h"
#include "bq27220.h"
#include "bq25896.h"
#include "ble.h"

#define BSP_OLED_WIDTH          128
#define BSP_OLED_HEIGHT         64

static const SSD1306_Font_t *const OLED_FONT = &Font_6x8;

#define BSP_TASK_STACK_DEPTH    512     /* in 32-bit words */
#define BSP_POLL_PERIOD_MS      10      /* real-time 10 ms period (100 Hz) */
#define BSP_DISPLAY_STACK_DEPTH 256     /* in 32-bit words */

#define BSP_HEARTBEAT_PERIOD_MS 500
#define BSP_TOF_PERIOD_MS       100
#define BSP_COLOR_PERIOD_MS     200
#define BSP_POWER_PERIOD_MS     1000
#define BSP_WD_RESET_PERIOD_MS  10000   /* BQ25896 charger watchdog refresh every 10 s */

/* Internal BSP Context (private singleton encapsulated in bsp.c) */
typedef struct {
    TaskHandle_t      task;             /* periodic sensor & input scanning task */
    TaskHandle_t      display_task;     /* display task: notified by I2C DMA completion ISR */
    SemaphoreHandle_t display_request;  /* given by bsp_oled_show(), taken by display task */

    /* Button debouncing state machines */
    debounce_t        deb_user_btn;
    debounce_t        deb_pwr_btn;

    /* Actuators & Audio */
    drv8833_t         motor_left;
    drv8833_t         motor_right;
    servo_t           gripper;
    buzzer_t          buzzer;

    /* Power Management (I2C1) */
    bq27220_t         fuel_gauge;
    bq25896_t         charger;

    /* Wireless Communication BLE (USART3) */
    ble_t             ble;

    /* Sensors */
    encoder_t         enc_left;
    encoder_t         enc_right;
    mpu6050_t         imu;
    vl53l0x_t         tof_left;
    vl53l0x_t         tof_right;
    /* vl53l0x_t      tof_center; */    /* Optional 3rd center ToF (EXT_PB13 -> 0x32) */
    apds9960_t        color;
    ydlidar_x2_t      lidar;

    /* Cached sensor telemetry */
    uint16_t          tof_left_mm;
    uint16_t          tof_right_mm;
    apds9960_data_t   color_data;

    /* Subsampling timestamp counters */
    uint32_t          last_heartbeat_ms;
    uint32_t          last_tof_ms;
    uint32_t          last_color_ms;
    uint32_t          last_power_ms;
    uint32_t          last_wd_reset_ms;

    /* Hardware presence & health flags */
    bool              display_present;
    bool              motors_ok;
    bool              encoders_ok;
    bool              gripper_ok;
    bool              neopixel_ok;
    bool              imu_ok;
    bool              tof_left_ok;
    bool              tof_right_ok;
    /* bool           tof_center_ok; */
    bool              color_ok;
    bool              lidar_ok;
    bool              buzzer_ok;
    bool              fuel_gauge_ok;
    bool              charger_ok;
    bool              ble_ok;
} bsp_ctx_t;

static bsp_ctx_t bsp_ctx;

static void bsp_task(void *unused);
static void bsp_process(void);
static void bsp_display_task(void *unused);

/* Initialize Board Support Package hardware and create periodic scanning task */
void bsp_init(unsigned int task_priority) {
    bsp_ctx_t *ctx = &bsp_ctx;
    memset(ctx, 0, sizeof(bsp_ctx_t));

    printf("\r\n========================================\r\n");
    printf("   BMO SYSTEM - BOARD SUPPORT PACKAGE   \r\n");
    printf("========================================\r\n");

    /* --- 1. Power management --- */
    /* BQ25896 charger & 5V boost | Periph: I2C1 | Addr: 0x6B | Pins: PB8 SCL, PB9 SDA */
    if (BQ25896_Init(&ctx->charger, &hi2c1) == HAL_OK) {
        ctx->charger_ok = true;
        printf("[BSP] BQ25896 Charger (I2C1 0x%02X): OK (VBUS: %u mV, VBAT: %u mV)\r\n",
               ctx->charger.i2c_addr >> 1,
               ctx->charger.vbus_mv,
               ctx->charger.vbat_mv);
    } else {
        printf("[BSP] BQ25896 Charger (I2C1): Not detected\r\n");
    }

    /* BQ27220 fuel gauge | Periph: I2C1 | Addr: 0x55 | Pins: PB8 SCL, PB9 SDA */
    if (BQ27220_Init(&ctx->fuel_gauge, &hi2c1) == HAL_OK) {
        ctx->fuel_gauge_ok = true;
        printf("[BSP] BQ27220 Fuel Gauge (I2C1 0x55): OK (SOC: %u%%, %u mV)\r\n",
               ctx->fuel_gauge.soc_percent,
               ctx->fuel_gauge.voltage_mv);
    } else {
        printf("[BSP] BQ27220 Fuel Gauge (I2C1 0x55): Not detected\r\n");
    }

    /* --- 2. Actuators & Safe-state --- */
    /* NeoPixel 12-LED ring | Periph: TIM15_CH1 | Addr: N/A | Pins: PB14 */
    np_led_set_all_RGB(0, 0, 0);
    np_led_render();
    HAL_Delay(10);
    ctx->neopixel_ok = true;
    printf("[BSP] NeoPixel (TIM15 CH1 / PB14): OK\r\n");

    /* DRV8833 dual DC motors | Periph: TIM3_CH1..CH4 | Addr: N/A | Pins: PA6, PA7, PB0, PB1 */
    if (htim3.Instance != NULL) {
        DRV8833_Init(&ctx->motor_left, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, false);
        DRV8833_Init(&ctx->motor_right, &htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, true);
        DRV8833_Coast(&ctx->motor_left);
        DRV8833_Coast(&ctx->motor_right);
        ctx->motors_ok = true;
        printf("[BSP] Motors DRV8833 (TIM3 CH1..CH4): OK (Coast)\r\n");
    } else {
        printf("[BSP] Motors DRV8833: TIM3 not initialized\r\n");
    }

    /* Servo gripper | Periph: TIM8_CH1 | Addr: N/A | Pins: PC6 */
    if (SERVO_Init(&ctx->gripper, &htim8, TIM_CHANNEL_1) == HAL_OK) {
        ctx->gripper_ok = true;
        SERVO_Release(&ctx->gripper);
        printf("[BSP] Servo Gripper (TIM8 CH1 / PC6): OK (Released)\r\n");
    } else {
        printf("[BSP] Servo Gripper (TIM8 CH1 / PC6): Init error\r\n");
    }

    /* --- 3. User inputs & Feedback --- */
    /* Push buttons debouncing | Periph: GPIO | Addr: N/A | Pins: PC13 USER_BTN, PC14 PWR_BUTTON */
    debounce_init(&ctx->deb_user_btn);
    debounce_init(&ctx->deb_pwr_btn);

    /* Passive buzzer | Periph: TIM17_CH1 | Addr: N/A | Pins: PB5 */
    if (BUZZER_Init(&ctx->buzzer, &htim17, TIM_CHANNEL_1)) {
        ctx->buzzer_ok = true;
        printf("[BSP] Buzzer (TIM17 CH1 / PB5): OK\r\n");
        BUZZER_PlaySound(&ctx->buzzer, BMO_SOUND_BOOT);
    }

    /* --- 4. Sensors --- */
    /* Quadrature encoders | Periph: TIM2 (32-bit), TIM4 (16-bit) | Addr: N/A | Pins: PA0/PA1, PB6/PB7 */
    Encoder_Init(&ctx->enc_left, &htim2, ENCODER_DEFAULT_CPR_WHEEL, ENCODER_DEFAULT_WHEEL_DIAM_MM, false);
    Encoder_Init(&ctx->enc_right, &htim4, ENCODER_DEFAULT_CPR_WHEEL, ENCODER_DEFAULT_WHEEL_DIAM_MM, true);
    ctx->encoders_ok = true;
    printf("[BSP] Encoders (TIM2 / TIM4): OK\r\n");

    /* MPU-6050 6-DOF IMU | Periph: I2C3 | Addr: 0x68 | Pins: PC8 SCL, PC9 SDA */
    if (MPU6050_Init(&ctx->imu, &hi2c3)) {
        ctx->imu_ok = true;
        printf("[BSP] MPU-6050 IMU (I2C3 0x68): OK\r\n");
        if (MPU6050_Calibrate(&ctx->imu, 150)) {
            printf("[BSP] MPU-6050 IMU: Calibrated (Gyro Z offset = %.2f deg/s)\r\n",
                   ctx->imu.gyro_z_offset);
        }
    } else {
        printf("[BSP] MPU-6050 IMU (I2C3 0x68): Not detected\r\n");
    }

    /* VL53L0X ToF distance sensors | Periph: I2C3 | Addr: 0x30, 0x31 (opt 0x32) | Pins: PC8 SCL, PC9 SDA, PC4/PC5/PB13 XSHUT */
    ctx->tof_left_mm = 65535;
    ctx->tof_right_mm = 65535;
    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_TOF_R_GPIO_Port, XSHUT_TOF_R_Pin, GPIO_PIN_RESET);
    /* HAL_GPIO_WritePin(EXT_PB13_GPIO_Port, EXT_PB13_Pin, GPIO_PIN_RESET); */ /* Center ToF XSHUT */
    HAL_Delay(10);

    /* Left ToF -> 0x30 */
    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&ctx->tof_left, &hi2c3)) {
            VL53L0X_SetAddress(&ctx->tof_left, VL53L0X_ADDR_LEFT_8BIT);
            VL53L0X_StartContinuous(&ctx->tof_left, 0);
            ctx->tof_left_ok = true;
            printf("[BSP] VL53L0X Left (I2C3 0x30): OK\r\n");
        }
    }
    if (!ctx->tof_left_ok) {
        printf("[BSP] VL53L0X Left (I2C3 0x30): Not detected\r\n");
    }

    /* Right ToF -> 0x31 */
    HAL_GPIO_WritePin(XSHUT_TOF_R_GPIO_Port, XSHUT_TOF_R_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&ctx->tof_right, &hi2c3)) {
            VL53L0X_SetAddress(&ctx->tof_right, VL53L0X_ADDR_RIGHT_8BIT);
            VL53L0X_StartContinuous(&ctx->tof_right, 0);
            ctx->tof_right_ok = true;
            printf("[BSP] VL53L0X Right (I2C3 0x31): OK\r\n");
        }
    }
    if (!ctx->tof_right_ok) {
        printf("[BSP] VL53L0X Right (I2C3 0x31): Not detected\r\n");
    }

    /* Optional Center ToF -> 0x32 (uncomment if hardware installed)
    HAL_GPIO_WritePin(EXT_PB13_GPIO_Port, EXT_PB13_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&ctx->tof_center, &hi2c3)) {
            VL53L0X_SetAddress(&ctx->tof_center, 0x32 << 1);
            VL53L0X_StartContinuous(&ctx->tof_center, 0);
            ctx->tof_center_ok = true;
            printf("[BSP] VL53L0X Center (I2C3 0x32): OK\r\n");
        }
    }
    if (!ctx->tof_center_ok) {
        printf("[BSP] VL53L0X Center (I2C3 0x32): Not detected\r\n");
    }
    */

    /* APDS-9960 color & proximity sensor | Periph: I2C3 | Addr: 0x39 | Pins: PC8 SCL, PC9 SDA */
    if (HAL_I2C_IsDeviceReady(&hi2c3, APDS9960_I2C_ADDR_8BIT, 2, 50) == HAL_OK) {
        if (APDS9960_Init(&ctx->color, &hi2c3)) {
            ctx->color_ok = true;
            printf("[BSP] APDS-9960 Color/Prox (I2C3 0x39): OK\r\n");
        }
    } else {
        printf("[BSP] APDS-9960 Color/Prox (I2C3 0x39): Not detected\r\n");
    }

    /* YDLIDAR X2 scanner | Periph: USART1 | Addr: N/A | Pins: PB6 TX / PB7 RX (or PC4/PC5) */
    YDLIDAR_X2_Init(&ctx->lidar, &huart1);
    ctx->lidar_ok = true;
    printf("[BSP] YDLIDAR X2 (USART1): OK\r\n");

    /* --- 5. Wireless communication --- */
    /* Bluetooth BLE module | Periph: USART3 | Addr: N/A | Pins: PB10 TX, PB11 RX, PA10 BLE_EN */
    if (BLE_Init(&ctx->ble, &huart3, BLE_EN_GPIO_Port, BLE_EN_Pin) == HAL_OK) {
        ctx->ble_ok = true;
        printf("[BSP] Bluetooth BLE (USART3 / PA10): OK\r\n");
    }

    /* --- 6. FreeRTOS task --- */
    BaseType_t ok = xTaskCreate(bsp_task, "BSP", BSP_TASK_STACK_DEPTH,
            NULL, task_priority, &ctx->task);
    configASSERT(ok == pdPASS);
}

/* Initialize OLED display hardware, must be called in main() before app_init() */
void bsp_init_display(unsigned int task_priority) {
    bsp_ctx_t *ctx = &bsp_ctx;

    /* SSD1306 OLED display | Periph: I2C2 | Addr: 0x3C | Pins: PA8 SCL, PA9 SDA */
    if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c2, SSD1306_I2C_ADDR, 3, 100)) {
        ctx->display_present = true;
        /* Before scheduler starts: HAL_Delay works because HAL timebase
         * (TIM6) is at NVIC priority 0, never masked by FreeRTOS.
         * ssd1306_Init() sends initialization commands only (no pixels). */
        ssd1306_Init();
        printf("[BSP] OLED Display (SSD1306 on I2C2): OK\r\n");
    } else {
        printf("[BSP] OLED Display (I2C2): Not detected\r\n");
    }

    /* Binary semaphore: multiple requests during an ongoing DMA transfer
     * merge into a single refresh (only the latest framebuffer state matters) */
    ctx->display_request = xSemaphoreCreateBinary();
    configASSERT(ctx->display_request != NULL);
    vQueueAddToRegistry(ctx->display_request, "display request");

    BaseType_t ok = xTaskCreate(bsp_display_task, "DISPLAY", BSP_DISPLAY_STACK_DEPTH,
            NULL, task_priority, &ctx->display_task);
    configASSERT(ok == pdPASS);
}

/* Display task: waits for refresh requests, then transmits framebuffer over DMA */
static void bsp_display_task(void *unused) {
    (void)unused;
    bsp_ctx_t *ctx = &bsp_ctx;

    for (;;) {
        xSemaphoreTake(ctx->display_request, portMAX_DELAY);    /* Blocked waiting for app */
        if (ctx->display_present) {
            ssd1306_UpdateScreen();
        }
    }
}

/* Overrides the weak blocking version in ssd1306 driver: driver manages protocol,
 * BSP provides non-blocking DMA transport with task notification synchronization. */
void ssd1306_WriteData(uint8_t *buffer, size_t size) {
    configASSERT(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);

    if (HAL_I2C_Mem_Write_DMA(&hi2c2, SSD1306_I2C_ADDR, 0x40, 1, buffer, (uint16_t)size) != HAL_OK) {
        return;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    /* Blocked during DMA: CPU remains 100% free */
}

/* Called from I2C2_EV_IRQHandler upon DMA transfer completion & STOP condition */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C2) {
        return;
    }
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(bsp_ctx.display_task, &woken);
    portYIELD_FROM_ISR(woken);
}

/* In case of I2C error, wake display task so it does not remain deadlocked */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_MemTxCpltCallback(hi2c);
}

/* Periodic real-time BSP task (100 Hz / 10 ms rate) */
static void bsp_task(void *unused) {
    (void)unused;
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        bsp_process();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(BSP_POLL_PERIOD_MS));
    }
}

/*
 * FUTURE OPTIMIZATION ROADMAP - HARDWARE INTERRUPT (EXTI) INTEGRATION
 * The following EXTI pins are configured in CubeMX for event-driven optimization:
 * - MOT_nFAULT   (PC0  / EXTI0)     : Instant PWM shutdown on motor stall / over-current (<1 us)
 * - MPU_INT      (PB4  / EXTI4)     : IMU raw data ready notification
 * - INT_TOF_L    (PB10 / EXTI15_10) : Left ToF distance measurement complete
 * - INT_TOF_R    (PC14 / EXTI15_10) : Right ToF distance measurement complete
 * - CHARGER_INT  (PC12 / EXTI15_10) : BQ25896 USB plug/unplug & charge fault alert
 * - COLOR_INT    (PC15 / EXTI15_10) : APDS-9960 can detected by threshold without I2C polling
 * - FG_GPOUT     (PD2  / EXTI2)     : BQ27220 critical low-battery hardware shutdown alert
 * Currently, periodic 100 Hz polling is used for stability and deterministic bringup.
 */
static void bsp_process(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    uint32_t now = HAL_GetTick();
    const float dt = (float)BSP_POLL_PERIOD_MS / 1000.0f;

    /* 1. Push button sampling and debouncing */
    bool user_pressed = (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET);
    if (debounce_process(&ctx->deb_user_btn, user_pressed, now)) {
        app_post_event((app_event_t){.type = EVT_BTN_USER_CLICK, .data = 0});
    }

    bool pwr_pressed = (HAL_GPIO_ReadPin(PWR_BUTTON_GPIO_Port, PWR_BUTTON_Pin) == GPIO_PIN_RESET);
    if (debounce_process(&ctx->deb_pwr_btn, pwr_pressed, now)) {
        app_post_event((app_event_t){.type = EVT_BTN_PWR_CLICK, .data = 0});
    }

    /* 2. High-rate 100 Hz updates: Encoders, IMU, Buzzer */
    if (ctx->encoders_ok) {
        Encoder_Update(&ctx->enc_left, dt);
        Encoder_Update(&ctx->enc_right, dt);
    }

    if (ctx->imu_ok) {
        MPU6050_Update(&ctx->imu, dt);
    }

    if (ctx->buzzer_ok) {
        BUZZER_Process(&ctx->buzzer, now);
    }

    /* 3. Inbound BLE command parser */
    if (ctx->ble_ok && BLE_HasCommand(&ctx->ble)) {
        char cmd_str[BLE_CMD_MAX_LEN] = {0};
        ble_command_type_t cmd_type = BLE_GetCommand(&ctx->ble, cmd_str, sizeof(cmd_str));
        app_post_event((app_event_t){.type = EVT_BLE_CMD, .data = (int32_t)cmd_type});
    }

    /* 4. Heartbeat LED (500 ms) */
    if ((int32_t)(now - ctx->last_heartbeat_ms) >= BSP_HEARTBEAT_PERIOD_MS) {
        ctx->last_heartbeat_ms = now;
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    }

    /* 5. Dual VL53L0X ToF range measurement (100 ms) */
    if ((int32_t)(now - ctx->last_tof_ms) >= BSP_TOF_PERIOD_MS) {
        ctx->last_tof_ms = now;

        if (ctx->tof_left_ok) {
            ctx->tof_left_mm = VL53L0X_ReadDistanceContinuous(&ctx->tof_left, NULL);
        }
        if (ctx->tof_right_ok) {
            ctx->tof_right_mm = VL53L0X_ReadDistanceContinuous(&ctx->tof_right, NULL);
        }

        uint16_t min_tof = ctx->tof_left_mm;
        if (ctx->tof_right_mm < min_tof) {
            min_tof = ctx->tof_right_mm;
        }
        if (min_tof < 150) {
            app_post_event((app_event_t){.type = EVT_TOF_OBJECT, .data = (int32_t)min_tof});
        }
    }

    /* 6. APDS-9960 Color sensor read (every 200 ms) */
    if (ctx->color_ok && (int32_t)(now - ctx->last_color_ms) >= BSP_COLOR_PERIOD_MS) {
        ctx->last_color_ms = now;
        (void)APDS9960_ReadData(&ctx->color, &ctx->color_data);
    }

    /* 7. Battery & Charger monitoring (BQ27220 & BQ25896 every 1000 ms) */
    if ((int32_t)(now - ctx->last_power_ms) >= BSP_POWER_PERIOD_MS) {
        ctx->last_power_ms = now;

        if (ctx->fuel_gauge_ok) {
            (void)BQ27220_Update(&ctx->fuel_gauge);
            if (ctx->fuel_gauge.terminate_discharge || ctx->fuel_gauge.sys_down) {
                app_post_event((app_event_t){
                    .type = EVT_BATTERY_LOW,
                    .data = (int32_t)ctx->fuel_gauge.voltage_mv
                });
            }
        }
        if (ctx->charger_ok) {
            (void)BQ25896_Update(&ctx->charger);
        }
    }

    /* 8. Refresh BQ25896 I2C watchdog timer (every 10 s to avoid WD timeout) */
    if (ctx->charger_ok && (int32_t)(now - ctx->last_wd_reset_ms) >= BSP_WD_RESET_PERIOD_MS) {
        ctx->last_wd_reset_ms = now;
        (void)BQ25896_ResetWatchdog(&ctx->charger);
    }
}

/* HSV to RGB color space conversion (h: 0..359, s/v: 0..255) */
static void hsv_to_rgb(int h, int s, int v, uint8_t *r, uint8_t *g, uint8_t *b) {
    h = ((h % 360) + 360) % 360;
    if (s <= 0) {
        *r = *g = *b = (uint8_t)v;
        return;
    }

    int region = h / 60;
    int rem = h % 60;
    int p = (v * (255 - s)) / 255;
    int q = (v * (255 - (s * rem) / 60)) / 255;
    int t = (v * (255 - (s * (60 - rem)) / 60)) / 255;

    switch (region) {
    case 0: *r = (uint8_t)v; *g = (uint8_t)t; *b = (uint8_t)p; break;
    case 1: *r = (uint8_t)q; *g = (uint8_t)v; *b = (uint8_t)p; break;
    case 2: *r = (uint8_t)p; *g = (uint8_t)v; *b = (uint8_t)t; break;
    case 3: *r = (uint8_t)p; *g = (uint8_t)q; *b = (uint8_t)v; break;
    case 4: *r = (uint8_t)t; *g = (uint8_t)p; *b = (uint8_t)v; break;
    default: *r = (uint8_t)v; *g = (uint8_t)p; *b = (uint8_t)q; break;
    }
}

uint32_t bsp_get_tick_ms(void) {
    return HAL_GetTick();
}

void bsp_status_led_set(bool on) {
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bsp_status_led_toggle(void) {
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
}

void bsp_neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    np_led_set_all_RGB(r, g, b);
    np_led_render();
}

void bsp_neopixel_set_hsv(int hue, int sat, int val) {
    uint8_t r, g, b;
    hsv_to_rgb(hue, sat, val, &r, &g, &b);
    np_led_set_all_RGB(r, g, b);
    np_led_render();
}

void bsp_neopixel_clear(void) {
    np_led_clear();
}

void bsp_leds_process(void) {
    static uint8_t state = 0;

    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

    switch (state) {
    case 0: /* RED */
        np_led_set_all_RGB(30, 0, 0);
        np_led_render();
        state = 1;
        break;
    case 1: /* GREEN */
        np_led_set_all_RGB(0, 30, 0);
        np_led_render();
        state = 2;
        break;
    case 2: /* BLUE */
        np_led_set_all_RGB(0, 0, 30);
        np_led_render();
        state = 3;
        break;
    case 3: /* WHITE */
        np_led_set_all_RGB(25, 25, 25);
        np_led_render();
        state = 4;
        break;
    case 4: /* YELLOW */
        np_led_set_all_RGB(30, 20, 0);
        np_led_render();
        state = 5;
        break;
    case 5: /* PURPLE */
        np_led_set_all_RGB(30, 0, 30);
        np_led_render();
        state = 6;
        break;
    case 6: /* CYAN */
        np_led_set_all_RGB(0, 30, 30);
        np_led_render();
        state = 7;
        break;
    case 7: /* OFF */
        np_led_clear();
        state = 0;
        break;
    default:
        state = 0;
        break;
    }
}

void bsp_oled_clear(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->display_present) {
        return;
    }
    ssd1306_Fill(Black);
}

void bsp_oled_fill_rect(int x, int y, int w, int h, bool on) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->display_present || w <= 0 || h <= 0) {
        return;
    }

    int x1 = x < 0 ? 0 : x;
    int y1 = y < 0 ? 0 : y;
    int x2 = x + w - 1;
    int y2 = y + h - 1;
    if (x2 >= BSP_OLED_WIDTH) {
        x2 = BSP_OLED_WIDTH - 1;
    }
    if (y2 >= BSP_OLED_HEIGHT) {
        y2 = BSP_OLED_HEIGHT - 1;
    }
    if (x1 > x2 || y1 > y2) {
        return;
    }

    ssd1306_FillRectangle((uint8_t)x1, (uint8_t)y1, (uint8_t)x2, (uint8_t)y2, on ? White : Black);
}

void bsp_oled_draw_string(int x, int y, const char *str, bool on) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->display_present) {
        return;
    }
    if (x < 0 || y < 0 || x >= BSP_OLED_WIDTH || y >= BSP_OLED_HEIGHT) {
        return;
    }

    char buf[32];
    size_t n = strlen(str);
    if (n >= sizeof(buf)) {
        n = sizeof(buf) - 1;
    }
    memcpy(buf, str, n);
    buf[n] = '\0';

    ssd1306_SetCursor((uint8_t)x, (uint8_t)y);
    ssd1306_WriteString(buf, *OLED_FONT, on ? White : Black);
}

/* Ne fait plus le transfert : demande un rafraichissement a la tache
 * d'affichage et revient tout de suite (run-to-completion de l'app). */
void bsp_oled_show(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (ctx->display_request != NULL) {
        xSemaphoreGive(ctx->display_request);
    }
}

void bsp_motors_set_speed(int16_t left_permille, int16_t right_permille) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->motors_ok) {
        return;
    }
    DRV8833_SetSpeed(&ctx->motor_left, left_permille);
    DRV8833_SetSpeed(&ctx->motor_right, right_permille);
}

void bsp_motors_brake(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->motors_ok) {
        return;
    }
    DRV8833_Brake(&ctx->motor_left);
    DRV8833_Brake(&ctx->motor_right);
}

void bsp_motors_coast(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->motors_ok) {
        return;
    }
    DRV8833_Coast(&ctx->motor_left);
    DRV8833_Coast(&ctx->motor_right);
}

void bsp_gripper_grab(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (ctx->gripper_ok) {
        SERVO_Grip(&ctx->gripper);
    }
}

void bsp_gripper_release(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (ctx->gripper_ok) {
        SERVO_Release(&ctx->gripper);
    }
}

void bsp_gripper_set_angle(uint16_t angle_deg) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (ctx->gripper_ok) {
        SERVO_SetAngle(&ctx->gripper, angle_deg);
    }
}

void bsp_buzzer_play(uint8_t sound_id) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (ctx->buzzer_ok && sound_id <= (uint8_t)BMO_SOUND_MATCH_START) {
        BUZZER_PlaySound(&ctx->buzzer, (bmo_sound_t)sound_id);
    }
}

void bsp_buzzer_stop(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (ctx->buzzer_ok) {
        BUZZER_Stop(&ctx->buzzer);
    }
}

void bsp_get_tof_distances(uint16_t *left_mm, uint16_t *right_mm) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (left_mm != NULL) {
        *left_mm = ctx->tof_left_mm;
    }
    if (right_mm != NULL) {
        *right_mm = ctx->tof_right_mm;
    }
}

float bsp_get_imu_yaw(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    return ctx->imu_ok ? ctx->imu.yaw : 0.0f;
}

uint16_t bsp_get_battery_mv(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (ctx->fuel_gauge_ok && ctx->fuel_gauge.voltage_mv > 0) {
        return ctx->fuel_gauge.voltage_mv;
    }
    if (ctx->charger_ok) {
        return ctx->charger.vbat_mv;
    }
    return 0;
}

uint8_t bsp_get_battery_soc(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    return ctx->fuel_gauge_ok ? (uint8_t)ctx->fuel_gauge.soc_percent : 0;
}

void bsp_ble_send(const char *msg) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (ctx->ble_ok && msg != NULL) {
        BLE_SendString(&ctx->ble, msg);
    }
}

/**
  * @brief  Route UART RX complete events to peripheral drivers.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (huart->Instance == USART3) {
        if (ctx->ble_ok) {
            BLE_RxCallback(&ctx->ble);
        }
    }
}

/*
 * Optional EXTI Interrupt ISR Handler (Template for future optimization)
 * Activates when EXTI interrupts are unmasked in CubeMX NVIC.
 *
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
        case MOT_nFAULT_Pin:
            // Fast shutdown: brake motors immediately
            DRV8833_Brake(&bsp_ctx.motor_left);
            DRV8833_Brake(&bsp_ctx.motor_right);
            break;
        case CHARGER_INT_Pin:
            // Charger status/fault change
            break;
        case COLOR_INT_Pin:
            // Can optical proximity detection
            break;
        default:
            break;
    }
}
*/
