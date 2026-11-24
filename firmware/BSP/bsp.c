/*
 * bsp.c
 *
 * Board Support Package implementation for BMO Robot.
 * Encapsule l'etat materiel dans le singleton prive `bsp_ctx`, gere la tache
 * temps-reel d'acquisition `bsp_task` et la tache d'affichage DMA `bsp_display_task`.
 */

#include "bsp.h"

#include <math.h>
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
#include "pid.h"

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

/* Asservissement en vitesse des roues (PI), execute a 100 Hz dans bsp_process().
 * Gains de depart A REGLER sur le robot : commencer par KI = 0, augmenter KP
 * jusqu'a une reponse rapide sans oscillation, puis ajouter KI pour annuler
 * l'erreur statique. Commande en pour mille (-1000..+1000) vers le DRV8833. */
#define BSP_SPEED_KP            2.0f    /* [pour mille / (mm/s)]       */
#define BSP_SPEED_KI            10.0f   /* [pour mille / (mm/s) / s]   */
#define BSP_SPEED_KD            0.0f    /* D non utilise : mesure bruitee */
#define BSP_SPEED_LOG           1       /* 1 = trace de reglage sur le port serie, 0 = rien */
#define BSP_SPEED_CMD_MAX       1000.0f /* limite de la commande du PI [pour mille], 1000 = 100 % PWM ;
                                         * a baisser ICI (pas apres le PID) pour garder l'anti-windup */

/* Reglages qui dependent du robot (0 = desactive).
 * Anticipation mesuree le 24/11/2026, roues en l'air, boost 5 V : commande en regime permanent
 * G/D = 606/628, 720/727, 797/805, 871/879, 939/948 pour mille a 50, 100, 150, 200, 250 mm/s ;
 * 1000 pour mille donne 292/285 mm/s. Droite des moindres carres : 551 + 1,61 x vitesse.
 * On prend 540 + 1,6 x vitesse, un peu sous le besoin : le PI ajoute le reste, sans depassement. */
#define BSP_SPEED_ACCEL_MM_S2   500.0f  /* rampe de consigne [mm/s2] : 0 -> 150 mm/s en 0,3 s ; limite le courant de
                                         * demarrage et le patinage. Mettre 0 pour regler KP/KI sur des echelons */
#define BSP_SPEED_KFF           1.6f    /* anticipation : pente [pour mille / (mm/s)] */
#define BSP_SPEED_FF_OFFSET     540.0f  /* anticipation : decalage [pour mille] (zone morte : depart vers 600) */

/* Commandes de test du PI au terminal (VCP ST-Link 115200 bauds, ou BLE), une par ligne :
 *   V 150    consigne des deux roues en mm/s (V -100 : marche arriere, V 0 : roue libre)
 *   F / B    consigne + / - BSP_SPEED_TEST_STEP mm/s
 *   H        (ou STOP) arret : roue libre, consigne 0
 *   KP 3     KI 15 : change le gain des deux roues tout de suite, sans recompiler
 *   KF 1.6   FO 540 : change l'anticipation (pente, decalage) tout de suite
 *   ?        affiche les reglages
 * Traitees dans le BSP : la machine d'etat n'est pas modifiee (elle ignore V, F, B, KP, KI, ?).
 * A remettre a 0 apres le reglage (sinon une commande F en match ferait bouger le robot). */
#define BSP_SPEED_TEST          1
#define BSP_SPEED_TEST_STEP     50.0f   /* pas de F / B [mm/s] */
#define BSP_SPEED_TEST_MAX      600.0f  /* consigne de test max [mm/s] */

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

    /* Speed control loop (PI), run by bsp_task, set by the APP task */
    pid_ctrl_t        pid_left;
    pid_ctrl_t        pid_right;
    volatile float    target_left_mm_s;   /* consigne roue gauche [mm/s] */
    volatile float    target_right_mm_s;  /* consigne roue droite [mm/s] */
    volatile bool     speed_loop_on;      /* boucle active ; remise a false par tout arret */
    float             ramp_left_mm_s;     /* consigne apres rampe, suivie par le PI gauche [mm/s] */
    float             ramp_right_mm_s;    /* consigne apres rampe, suivie par le PI droit [mm/s] */

    /* Power Management (I2C1) */
    bq27220_t         fuel_gauge;
    bq25896_t         charger;

    /* Wireless BLE (USART3) & Debug VCP Command Parser (USART2) */
    ble_t             ble;
#if (BMO_DEBUG_ENABLE != 0)
    ble_t             vcp_cmd;
    bool              vcp_cmd_ok;
#endif

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
    bmo_can_color_t   detected_color;
    uint16_t          lidar_target_dist_mm;
    int16_t           lidar_target_angle_deg;
    uint16_t          lidar_target_width_mm;
    bool              lidar_target_found;

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
        (void)BQ25896_DisableWatchdog(&ctx->charger);
        (void)BQ25896_SetBoostMode(&ctx->charger, true);
        ctx->charger_ok = true;
        printf("[BSP] BQ25896 Charger (I2C1 0x%02X): OK (VBUS: %u mV, VBAT: %u mV, Boost 5V: ON)\r\n",
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
        DRV8833_Init(&ctx->motor_left, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, true);
        DRV8833_Init(&ctx->motor_right, &htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, false);
        DRV8833_Coast(&ctx->motor_left);
        DRV8833_Coast(&ctx->motor_right);
        /* Reveille le DRV8833 : MX_GPIO_Init() met nSLEEP (PA4) a 0, et a 0 le driver reste en veille,
         * sorties coupees (SLVSAR1E p.3 : "Logic high to enable device"). Les entrees sont deja
         * en roue libre (Coast) : rien ne bouge au reveil. */
        HAL_GPIO_WritePin(MOT_nSLEEP_GPIO_Port, MOT_nSLEEP_Pin, GPIO_PIN_SET);
        ctx->motors_ok = true;
        PID_Init(&ctx->pid_left,  BSP_SPEED_KP, BSP_SPEED_KI, BSP_SPEED_KD, -BSP_SPEED_CMD_MAX, BSP_SPEED_CMD_MAX);
        PID_Init(&ctx->pid_right, BSP_SPEED_KP, BSP_SPEED_KI, BSP_SPEED_KD, -BSP_SPEED_CMD_MAX, BSP_SPEED_CMD_MAX);
        PID_SetFeedforward(&ctx->pid_left,  BSP_SPEED_KFF, BSP_SPEED_FF_OFFSET);
        PID_SetFeedforward(&ctx->pid_right, BSP_SPEED_KFF, BSP_SPEED_FF_OFFSET);
        ctx->speed_loop_on = false;
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
    Encoder_Init(&ctx->enc_left, &htim2, ENCODER_DEFAULT_CPR_WHEEL, ENCODER_DEFAULT_WHEEL_DIAM_MM, true);
    Encoder_Init(&ctx->enc_right, &htim4, ENCODER_DEFAULT_CPR_WHEEL, ENCODER_DEFAULT_WHEEL_DIAM_MM, false);
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

    /* VL53L0X ToF Sensors Architecture:
     * - Center ToF (PB13 / EXT_PB13 -> 0x32): Dedicated Can / Gripper sensor inside jaw.
     * - Left Cliff ToF (PC2 / XSHUT_TOF_L -> 0x30): Floor sensor pointing down on front-left.
     * - Right Cliff ToF (PC3 / XSHUT_TOF_R -> 0x31): Floor sensor pointing down on front-right.
     *
     * CURRENT TEST BENCH SETUP:
     * Only 1 physical ToF is connected right now on PC2 (tof_left).
     * We initialize and use it to emulate the Center Can/Gripper sensor for this bench test.
     */
    ctx->tof_left_mm = 65535;
    ctx->tof_right_mm = 65535;
    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_TOF_R_GPIO_Port, XSHUT_TOF_R_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    /* 1. Sensor connected to PC2 (Addr: 0x30) - Currently used as Can/Gripper test sensor */
    HAL_GPIO_WritePin(XSHUT_TOF_L_GPIO_Port, XSHUT_TOF_L_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&ctx->tof_left, &hi2c3)) {
            VL53L0X_SetAddress(&ctx->tof_left, VL53L0X_ADDR_LEFT_8BIT);
            VL53L0X_StartContinuous(&ctx->tof_left, 0);
            ctx->tof_left_ok = true;
            printf("[BSP] ToF on PC2 (I2C3 0x30) [Acting as Center/Gripper]: OK\r\n");
        }
    }
    if (!ctx->tof_left_ok) {
        printf("[BSP] ToF on PC2 (I2C3 0x30): Not detected\r\n");
    }

    /* 2. Right ToF -> PC3 / Addr: 0x31 */
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

    /* 3. Future Dedicated Center Can/Gripper ToF -> PB13 / Addr: 0x32 (Uncomment when installed)
    HAL_GPIO_WritePin(EXT_PB13_GPIO_Port, EXT_PB13_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (HAL_I2C_IsDeviceReady(&hi2c3, VL53L0X_DEFAULT_ADDRESS_8BIT, 2, 50) == HAL_OK) {
        if (VL53L0X_Init(&ctx->tof_center, &hi2c3)) {
            VL53L0X_SetAddress(&ctx->tof_center, 0x32 << 1);
            VL53L0X_StartContinuous(&ctx->tof_center, 0);
            ctx->tof_center_ok = true;
            printf("[BSP] ToF Center Gripper (I2C3 0x32 / PB13): OK\r\n");
        }
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

    /* YDLIDAR X2 scanner | Periph: USART1, TIM16_CH1 (PA12 PWM) | Pins: PC5 RX */
    if (htim16.Instance != NULL) {
        HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 65);
    }
    YDLIDAR_X2_Init(&ctx->lidar, &huart1);
    ctx->lidar.angle_offset_deg = BSP_LIDAR_ANGLE_OFFSET_DEG;
    ctx->lidar_ok = true;
    printf("[BSP] YDLIDAR X2 (USART1): OK\r\n");

    /* --- 5. Wireless & VCP command interfaces --- */
    /* Bluetooth BLE module | Periph: USART3 | Addr: N/A | Pins: PB10 TX, PB11 RX, PA10 BLE_EN */
    if (BLE_Init(&ctx->ble, &huart3, BLE_EN_GPIO_Port, BLE_EN_Pin) == HAL_OK) {
        ctx->ble_ok = true;
        printf("[BSP] Bluetooth BLE (USART3 / PA10): OK\r\n");
    }
#if (BMO_DEBUG_ENABLE != 0)
    /* PC VCP command listener | Periph: USART2 | Commands: MAP, START, STOP */
    if (BLE_Init(&ctx->vcp_cmd, &huart2, NULL, 0) == HAL_OK) {
        ctx->vcp_cmd_ok = true;
        printf("[BSP] VCP Command Shell (USART2): Ready (send MAP, START, STOP)\r\n");
    }
#endif

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
typedef struct {
    uint16_t best_can_dist;
    int16_t  best_can_angle;
    uint16_t best_can_width;
    uint16_t best_any_dist;
    int16_t  best_any_angle;
    uint16_t best_any_width;
} lidar_scan_res_t;

static void bsp_eval_cluster(uint16_t start_deg, uint16_t first_step, uint16_t last_step,
                             uint16_t min_d, uint16_t hit_count, lidar_scan_res_t *res) {
    if (hit_count < 2 || min_d < BSP_LIDAR_MIN_DIST_MM || min_d > BSP_LIDAR_MAX_DIST_MM) {
        return;
    }

    uint16_t span_deg   = (last_step - first_step) + 1;
    uint16_t width_mm   = (uint16_t)((float)min_d * (float)span_deg * 0.01745329f + 0.5f);
    uint16_t center_u16 = (uint16_t)((start_deg + (first_step + last_step) / 2) % 360);
    int16_t  center_deg = (center_u16 > 180) ? ((int16_t)center_u16 - 360) : (int16_t)center_u16;
    int16_t  abs_deg    = (center_deg < 0) ? -center_deg : center_deg;

    if (abs_deg > BSP_LIDAR_SCAN_HALF_DEG) {
        return;
    }

    if (min_d < res->best_any_dist) {
        res->best_any_dist  = min_d;
        res->best_any_angle = center_deg;
        res->best_any_width = width_mm;
    }

    bool is_can = (width_mm >= BSP_LIDAR_CAN_MIN_WIDTH_MM) &&
                  (width_mm <= BSP_LIDAR_CAN_MAX_WIDTH_MM);
    if (is_can && min_d < res->best_can_dist) {
        res->best_can_dist  = min_d;
        res->best_can_angle = center_deg;
        res->best_can_width = width_mm;
    }
}

/* Segment 360-deg LiDAR point cloud into clusters and isolate cylindrical cans from arena walls */
static void bsp_lidar_cluster_scan(bsp_ctx_t *ctx) {
    lidar_scan_res_t res = {
        .best_can_dist = 65535, .best_can_angle = 0, .best_can_width = 0,
        .best_any_dist = 65535, .best_any_angle = 0, .best_any_width = 0
    };

    /* 1. Find a clean angular seam (empty ray or depth jump) so no wall is cut at 0 deg */
    uint16_t start_deg = 0;
    for (uint16_t i = 0; i < 360; i++) {
        uint16_t prev  = (i == 0) ? 359 : (i - 1);
        uint16_t d_cur = ctx->lidar.distances[i];
        uint16_t d_prv = ctx->lidar.distances[prev];
        if (d_cur < BSP_LIDAR_MIN_DIST_MM) d_cur = 0;
        if (d_prv < BSP_LIDAR_MIN_DIST_MM) d_prv = 0;
        uint16_t diff  = (d_cur > d_prv) ? (d_cur - d_prv) : (d_prv - d_cur);
        if (d_cur == 0 || d_prv == 0 || diff > BSP_LIDAR_CLUSTER_JUMP_MM) {
            start_deg = i;
            break;
        }
    }

    /* 2. Walk 360 degrees (step == 360 flushes the final open cluster) */
    bool     in_cluster = false;
    uint16_t first_step = 0;
    uint16_t last_step  = 0;
    uint16_t last_d     = 0;
    uint16_t min_d      = 65535;
    uint16_t hit_count  = 0;
    uint8_t  gap_count  = 0;

    for (uint16_t step = 0; step <= 360; step++) {
        uint16_t d = (step < 360) ? ctx->lidar.distances[(start_deg + step) % 360] : 0;
        if (d < BSP_LIDAR_MIN_DIST_MM) d = 0;

        if (step < 360 && d > 0) {
            if (!in_cluster) {
                in_cluster = true;
                first_step = step;
                last_step  = step;
                last_d     = d;
                min_d      = d;
                hit_count  = 1;
                gap_count  = 0;
            } else {
                uint16_t diff = (d > last_d) ? (d - last_d) : (last_d - d);
                if (diff <= BSP_LIDAR_CLUSTER_JUMP_MM) {
                    last_step = step;
                    last_d    = d;
                    if (d < min_d) min_d = d;
                    hit_count++;
                    gap_count = 0;
                } else {
                    /* Depth discontinuity (e.g. can edge in front of arena wall) */
                    bsp_eval_cluster(start_deg, first_step, last_step, min_d, hit_count, &res);
                    first_step = step;
                    last_step  = step;
                    last_d     = d;
                    min_d      = d;
                    hit_count  = 1;
                    gap_count  = 0;
                }
            }
        } else if (in_cluster) {
            if (step < 360 && gap_count < BSP_LIDAR_MAX_GAP_DEG) {
                gap_count++;
            } else {
                bsp_eval_cluster(start_deg, first_step, last_step, min_d, hit_count, &res);
                in_cluster = false;
            }
        }
    }

    /* 3. Publish closest valid can (or closest ignored obstacle for live diagnostics) */
    if (res.best_can_dist <= BSP_LIDAR_MAX_DIST_MM) {
        ctx->lidar_target_found     = true;
        ctx->lidar_target_dist_mm   = res.best_can_dist;
        ctx->lidar_target_angle_deg = res.best_can_angle;
        ctx->lidar_target_width_mm  = res.best_can_width;
        app_post_event((app_event_t){.type = EVT_LIDAR_TARGET, .data = (int32_t)res.best_can_dist});
    } else {
        ctx->lidar_target_found     = false;
        ctx->lidar_target_dist_mm   = (res.best_any_dist <= BSP_LIDAR_MAX_DIST_MM) ? res.best_any_dist : 0;
        ctx->lidar_target_angle_deg = res.best_any_angle;
        ctx->lidar_target_width_mm  = res.best_any_width;
    }
}

/* Un pas de la boucle de vitesse pour une roue (100 Hz) : rampe, arret propre, PI.
 * Retourne la commande en pour mille pour le DRV8833. */
static float bsp_speed_wheel_step(pid_ctrl_t *pid, float *ramp, float target, float measure, float dt) {
    /* 1. Rampe : la consigne suivie bouge d'au plus ACCEL * dt par cycle (desactivee si 0) */
    if (BSP_SPEED_ACCEL_MM_S2 > 0.0f) {
        float step = BSP_SPEED_ACCEL_MM_S2 * dt;
        float diff = target - *ramp;
        *ramp += (diff > step) ? step : (diff < -step) ? -step : diff;
    } else {
        *ramp = target;
    }

    /* 2. Consigne nulle : roue libre et PI remis a zero (pas de regulation autour de 0,
     *    donc pas de vibration ; avec la rampe, la roue a deja ralenti en suivant la consigne) */
    if (*ramp == 0.0f) {
        PID_Reset(pid);
        return 0.0f;
    }

    /* 3. Correcteur PI (+ anticipation si reglee) */
    return PID_Update(pid, *ramp, measure, dt);
}

#if BSP_SPEED_TEST
static float bsp_speed_test_mm_s = 0.0f;   /* derniere consigne de test [mm/s] */

/* Affiche x >= 0 avec 2 decimales, en entiers : pas de printf("%f") dans la tache BSP,
 * car le formatage des float de newlib passe par malloc */
#define BSP_X100(x)  (long)((x) * 100.0f + 0.5f) / 100, (long)((x) * 100.0f + 0.5f) % 100

/* Lit un nombre decimal simple ("150", "-100", "2.5") ; evite strtof, trop gros pour la Flash */
static bool bsp_parse_number(const char *s, float *out) {
    float sign = 1.0f, val = 0.0f, scale = 0.0f;
    bool digit = false;
    while (*s == ' ' || *s == '\t' || *s == '=') {
        s++;
    }
    if (*s == '-' || *s == '+') {
        sign = (*s == '-') ? -1.0f : 1.0f;
        s++;
    }
    for (; *s != '\0' && *s != ' ' && *s != '\t'; s++) {
        if (*s >= '0' && *s <= '9') {
            digit = true;
            if (scale == 0.0f) {
                val = val * 10.0f + (float)(*s - '0');
            } else {
                val += (float)(*s - '0') * scale;
                scale *= 0.1f;
            }
        } else if ((*s == '.' || *s == ',') && scale == 0.0f) {
            scale = 0.1f;
        } else {
            return false;
        }
    }
    if (!digit) {
        return false;
    }
    *out = sign * val;
    return true;
}

/* Commandes de test du PI (voir BSP_SPEED_TEST), appelee par bsp_process() pour chaque ligne recue */
static void bsp_speed_test_command(bsp_ctx_t *ctx, ble_command_type_t type, const char *str) {
    char up[BLE_CMD_MAX_LEN];
    size_t n = 0;
    float v = ctx->speed_loop_on ? bsp_speed_test_mm_s : 0.0f;   /* boucle coupee : on repart de 0 */

    while (*str == ' ' || *str == '\t') {
        str++;
    }
    for (; str[n] != '\0' && n < sizeof(up) - 1; n++) {
        up[n] = (str[n] >= 'a' && str[n] <= 'z') ? (char)(str[n] - ('a' - 'A')) : str[n];
    }
    up[n] = '\0';

    if (type == BLE_CMD_STOP) {
        bsp_speed_test_mm_s = 0.0f;
        bsp_motors_coast();
        printf("[PID] Arret : roue libre\r\n");
        return;
    }
    if (type == BLE_CMD_MANUAL_FWD || type == BLE_CMD_MANUAL_BWD) {
        v += (type == BLE_CMD_MANUAL_FWD) ? BSP_SPEED_TEST_STEP : -BSP_SPEED_TEST_STEP;
    } else if (type != BLE_CMD_CUSTOM) {
        return;                                     /* START, MAP... : pour l'application */
    } else if (up[0] == '?') {
        printf("[PID] KP %ld.%02ld KI %ld.%02ld KD %ld.%02ld | KFF %ld.%02ld OFFSET %ld | rampe %ld mm/s2 | max %ld | consigne %ld mm/s | boucle %s\r\n",
               BSP_X100(ctx->pid_left.kp), BSP_X100(ctx->pid_left.ki), BSP_X100(ctx->pid_left.kd),
               BSP_X100(ctx->pid_left.kff), (long)ctx->pid_left.ff_offset,
               (long)BSP_SPEED_ACCEL_MM_S2, (long)BSP_SPEED_CMD_MAX,
               (long)v, ctx->speed_loop_on ? "ON" : "OFF");
        return;
    } else if (up[0] == 'K' && (up[1] == 'P' || up[1] == 'I')) {
        float k;
        if (!bsp_parse_number(&up[2], &k) || k < 0.0f) {
            printf("[PID] Valeur invalide : %s\r\n", str);
            return;
        }
        if (up[1] == 'P') {
            ctx->pid_left.kp = k;
            ctx->pid_right.kp = k;
        } else {
            ctx->pid_left.ki = k;                   /* l'integrale garde sa valeur : pas de saut */
            ctx->pid_right.ki = k;
        }
        printf("[PID] KP %ld.%02ld KI %ld.%02ld\r\n", BSP_X100(ctx->pid_left.kp), BSP_X100(ctx->pid_left.ki));
        return;
    } else if ((up[0] == 'K' && up[1] == 'F') || (up[0] == 'F' && up[1] == 'O')) {
        float k;
        if (!bsp_parse_number(&up[2], &k) || k < 0.0f) {
            printf("[PID] Valeur invalide : %s\r\n", str);
            return;
        }
        float kff = (up[0] == 'K') ? k : ctx->pid_left.kff;
        float off = (up[0] == 'F') ? k : ctx->pid_left.ff_offset;
        PID_SetFeedforward(&ctx->pid_left, kff, off);
        PID_SetFeedforward(&ctx->pid_right, kff, off);
        printf("[PID] KFF %ld.%02ld OFFSET %ld\r\n", BSP_X100(kff), (long)off);
        return;
    } else if (up[0] == 'V') {
        if (!bsp_parse_number(&up[1], &v)) {
            printf("[PID] Valeur invalide : %s\r\n", str);
            return;
        }
    } else {
        return;                                     /* autre texte : ignore */
    }

    if (v > BSP_SPEED_TEST_MAX) {
        v = BSP_SPEED_TEST_MAX;
    } else if (v < -BSP_SPEED_TEST_MAX) {
        v = -BSP_SPEED_TEST_MAX;
    }
    bsp_speed_test_mm_s = v;
    bsp_motors_set_target_speed(v, v);
    printf("[PID] Consigne %ld mm/s (KP %ld.%02ld, KI %ld.%02ld)\r\n", (long)v,
           BSP_X100(ctx->pid_left.kp), BSP_X100(ctx->pid_left.ki));
}
#endif

static void bsp_process(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    uint32_t now = HAL_GetTick();
    const float dt = (float)BSP_POLL_PERIOD_MS / 1000.0f;

    /* 1. Push button sampling and debouncing (USER_BTN on PC13) */
    bool user_pressed = (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET);
    if (debounce_process(&ctx->deb_user_btn, user_pressed, now)) {
        printf("[BTN] USER_BTN (PC13) Clicked! Posting EVT_BTN_USER_CLICK\r\n");
        app_post_event((app_event_t){.type = EVT_BTN_USER_CLICK, .data = 0});
    }

    bool pwr_pressed = (HAL_GPIO_ReadPin(PWR_BUTTON_GPIO_Port, PWR_BUTTON_Pin) == GPIO_PIN_RESET);
    if (debounce_process(&ctx->deb_pwr_btn, pwr_pressed, now)) {
        app_post_event((app_event_t){.type = EVT_BTN_PWR_CLICK, .data = 0});
    }

    /* 2. High-rate 100 Hz updates: Encoders, IMU, Buzzer, LiDAR */
    if (ctx->encoders_ok) {
        Encoder_Update(&ctx->enc_left, dt);
        Encoder_Update(&ctx->enc_right, dt);
    }

    /* 2b. Boucle de vitesse PI des roues (100 Hz) : mesure encodeur -> PID -> PWM */
    if (ctx->speed_loop_on && ctx->motors_ok && ctx->encoders_ok) {
        float cmd_l = bsp_speed_wheel_step(&ctx->pid_left,  &ctx->ramp_left_mm_s,
                                           ctx->target_left_mm_s,  ctx->enc_left.speed_mm_s,  dt);
        float cmd_r = bsp_speed_wheel_step(&ctx->pid_right, &ctx->ramp_right_mm_s,
                                           ctx->target_right_mm_s, ctx->enc_right.speed_mm_s, dt);

        /* Section critique : si un arret (ou plus tard l'ISR nFAULT) coupe la boucle pendant
         * le calcul, on n'ecrit pas la PWM, sinon les moteurs repartiraient apres le freinage. */
        taskENTER_CRITICAL();
        if (ctx->speed_loop_on) {
            DRV8833_SetSpeed(&ctx->motor_left,  (int16_t)cmd_l);
            DRV8833_SetSpeed(&ctx->motor_right, (int16_t)cmd_r);
        }
        taskEXIT_CRITICAL();

#if BSP_SPEED_LOG
        /* Trace pour le reglage (port serie ST-Link, 115200 bauds), toutes les 50 ms :
         * SPD,consigne_G,mesure_G,commande_G,consigne_D,mesure_D,commande_D  (mm/s et pour mille) */
        static uint8_t log_div = 0;
        if (++log_div >= 5) {
            log_div = 0;
            printf("SPD,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                   (long)ctx->target_left_mm_s,  (long)ctx->enc_left.speed_mm_s,  (long)cmd_l,
                   (long)ctx->target_right_mm_s, (long)ctx->enc_right.speed_mm_s, (long)cmd_r);
        }
#endif
    }

    if (ctx->imu_ok) {
        MPU6050_Update(&ctx->imu, dt);
    }

    if (ctx->buzzer_ok) {
        BUZZER_Process(&ctx->buzzer, now);
    }

    if (ctx->lidar_ok) {
        YDLIDAR_X2_Process(&ctx->lidar);
    }

    /* 3. Inbound BLE & VCP command parsers */
    if (ctx->ble_ok && BLE_HasCommand(&ctx->ble)) {
        char cmd_str[BLE_CMD_MAX_LEN] = {0};
        ble_command_type_t cmd_type = BLE_GetCommand(&ctx->ble, cmd_str, sizeof(cmd_str));
#if BSP_SPEED_TEST
        bsp_speed_test_command(ctx, cmd_type, cmd_str);
#endif
        app_post_event((app_event_t){.type = EVT_BLE_CMD, .data = (int32_t)cmd_type});
    }
#if (BMO_DEBUG_ENABLE != 0)
    if (ctx->vcp_cmd_ok && BLE_HasCommand(&ctx->vcp_cmd)) {
        char cmd_str[BLE_CMD_MAX_LEN] = {0};
        ble_command_type_t cmd_type = BLE_GetCommand(&ctx->vcp_cmd, cmd_str, sizeof(cmd_str));
        printf("[VCP] Command: '%s'\r\n", cmd_str);
#if BSP_SPEED_TEST
        bsp_speed_test_command(ctx, cmd_type, cmd_str);
#endif
        app_post_event((app_event_t){.type = EVT_BLE_CMD, .data = (int32_t)cmd_type});
    }
#endif

    /* 4. Heartbeat LED (500 ms) */
    if ((int32_t)(now - ctx->last_heartbeat_ms) >= BSP_HEARTBEAT_PERIOD_MS) {
        ctx->last_heartbeat_ms = now;
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    }

    /* 5a. LiDAR 360-deg Angular Clustering (Can vs Wall discrimination, every 200 ms) */
    static uint32_t last_lidar_scan_ms = 0;
    if (ctx->lidar_ok && (int32_t)(now - last_lidar_scan_ms) >= 200) {
        last_lidar_scan_ms = now;
        bsp_lidar_cluster_scan(ctx);
    }

    /* 5b. ToF Distance Sensors (100 ms)
     * Target Architecture:
     * - Center ToF (PB13): Dedicated Can / Gripper sensor inside the jaw.
     * - Left ToF (PC2) & Right ToF (PC3): Downward-facing Cliff sensors for table edge detection.
     *
     * CURRENT BENCH TEST EMULATION:
     * Since only 1 sensor is wired right now (on PC2 / tof_left), it acts as the Can/Gripper sensor.
     * Once the center sensor is plugged into PB13, swap ctx->tof_left_mm for ctx->tof_center_mm below.
     */
    if ((int32_t)(now - ctx->last_tof_ms) >= BSP_TOF_PERIOD_MS) {
        ctx->last_tof_ms = now;

        if (ctx->tof_left_ok) {
            ctx->tof_left_mm = VL53L0X_ReadDistanceContinuous(&ctx->tof_left, NULL);
        }
        if (ctx->tof_right_ok) {
            ctx->tof_right_mm = VL53L0X_ReadDistanceContinuous(&ctx->tof_right, NULL);
        }

        /* 1. Can / Gripper close-range detection (< BSP_TOF_GRIPPER_MM) */
        uint16_t min_tof = 65535;
        if (ctx->tof_left_ok && ctx->tof_left_mm > 0 && ctx->tof_left_mm < min_tof) {
            min_tof = ctx->tof_left_mm;
        }
        if (ctx->tof_right_ok && ctx->tof_right_mm > 0 && ctx->tof_right_mm < min_tof) {
            min_tof = ctx->tof_right_mm;
        }

        if (min_tof < BSP_TOF_GRIPPER_MM) {
            /* Decoupled Active Object: post ToF event; consumed in SEARCH or APPROACH */
            app_post_event((app_event_t){.type = EVT_TOF_OBJECT, .data = (int32_t)min_tof});
        }
    }

    /* 6. APDS-9960 Color sensor read (every 150 ms) */
    if (ctx->color_ok && (int32_t)(now - ctx->last_color_ms) >= 150) {
        ctx->last_color_ms = now;
        if (APDS9960_ReadData(&ctx->color, &ctx->color_data)) {
            bmo_can_color_t color = APDS9960_ClassifyColor(&ctx->color_data);
            ctx->detected_color = color;

            if (color == BMO_CAN_COLOR_RED ||
                color == BMO_CAN_COLOR_GREEN ||
                color == BMO_CAN_COLOR_BLUE) {
                /* Decoupled Active Object: post periodic color event; FSM consumes it in VERIFY_COLOR */
                app_post_event((app_event_t){.type = EVT_CAN_COLOR, .data = (int32_t)color});
            }
        }
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
    ctx->speed_loop_on = false;     /* commande directe : on coupe la boucle */
    DRV8833_SetSpeed(&ctx->motor_left, left_permille);
    DRV8833_SetSpeed(&ctx->motor_right, right_permille);
}

/* Consigne de vitesse en boucle fermee (mm/s). La boucle PI tourne dans bsp_task.
 * Appelee par la tache APP (priorite 2), lue par la tache BSP (priorite 3) :
 * section critique pour que les deux consignes et le depart de la boucle
 * soient pris en compte ensemble, au meme cycle de 10 ms. */
void bsp_motors_set_target_speed(float left_mm_s, float right_mm_s) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->motors_ok || !ctx->encoders_ok) {
        return;
    }
    if (!isfinite(left_mm_s) || !isfinite(right_mm_s)) {
        return;                         /* consigne invalide (NaN, infini) : ignoree */
    }
    taskENTER_CRITICAL();
    if (!ctx->speed_loop_on) {
        /* Depart sans a-coup : la rampe part de la vitesse mesuree,
         * et l'integrale reprend la PWM deja appliquee (0 apres brake / coast) */
        ctx->ramp_left_mm_s  = ctx->enc_left.speed_mm_s;
        ctx->ramp_right_mm_s = ctx->enc_right.speed_mm_s;
        PID_Preset(&ctx->pid_left,  (float)ctx->motor_left.current_speed,  ctx->ramp_left_mm_s);
        PID_Preset(&ctx->pid_right, (float)ctx->motor_right.current_speed, ctx->ramp_right_mm_s);
    }
    ctx->target_left_mm_s  = left_mm_s;
    ctx->target_right_mm_s = right_mm_s;
    ctx->speed_loop_on = true;
    taskEXIT_CRITICAL();
}

void bsp_motors_brake(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->motors_ok) {
        return;
    }
    ctx->speed_loop_on = false;     /* commande directe : on coupe la boucle */
    DRV8833_Brake(&ctx->motor_left);
    DRV8833_Brake(&ctx->motor_right);
}

void bsp_motors_coast(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (!ctx->motors_ok) {
        return;
    }
    ctx->speed_loop_on = false;     /* commande directe : on coupe la boucle */
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

bool bsp_get_lidar_target(uint16_t *dist_mm, int16_t *angle_deg) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (dist_mm != NULL) {
        *dist_mm = ctx->lidar_target_dist_mm;
    }
    if (angle_deg != NULL) {
        *angle_deg = ctx->lidar_target_angle_deg;
    }
    return ctx->lidar_target_found;
}

uint16_t bsp_get_lidar_target_width(void) {
    return bsp_ctx.lidar_target_width_mm;
}

const uint16_t* bsp_get_lidar_distances(float *scan_hz) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (scan_hz != NULL) {
        *scan_hz = ctx->lidar.scan_frequency_hz;
    }
    return ctx->lidar.distances;
}

uint8_t bsp_get_detected_color(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    return (uint8_t)ctx->detected_color;
}

const char* bsp_get_color_name(void) {
    bsp_ctx_t *ctx = &bsp_ctx;
    return APDS9960_ColorToString(ctx->detected_color);
}

void bsp_get_color_raw(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c, uint8_t *prox) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (r != NULL) *r = ctx->color_data.red;
    if (g != NULL) *g = ctx->color_data.green;
    if (b != NULL) *b = ctx->color_data.blue;
    if (c != NULL) *c = ctx->color_data.clear;
    if (prox != NULL) *prox = ctx->color_data.proximity;
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
#if (BMO_DEBUG_ENABLE != 0)
    else if (huart->Instance == USART2) {
        if (ctx->vcp_cmd_ok) {
            BLE_RxCallback(&ctx->vcp_cmd);
        }
    }
#endif
}

/**
  * @brief  Recover interrupt-driven UART RX (USART3 BLE / USART2 VCP) after ORE/FE/NE error.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    bsp_ctx_t *ctx = &bsp_ctx;
    if (huart->Instance == USART3) {
        if (ctx->ble_ok) {
            BLE_ErrorCallback(&ctx->ble);
        }
    }
#if (BMO_DEBUG_ENABLE != 0)
    else if (huart->Instance == USART2) {
        if (ctx->vcp_cmd_ok) {
            BLE_ErrorCallback(&ctx->vcp_cmd);
        }
    }
#endif
}

/*
 * Optional EXTI Interrupt ISR Handler (Template for future optimization)
 * Activates when EXTI interrupts are unmasked in CubeMX NVIC.
 *
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
        case MOT_nFAULT_Pin:
            // Fast shutdown: brake motors immediately
            bsp_ctx.speed_loop_on = false;   // stop the PI loop, otherwise it would restart the motors
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
