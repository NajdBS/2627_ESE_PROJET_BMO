/*
 * bsp.h
 *
 * Board Support Package facade for BMO Robot.
 * No HAL headers or low-level register definitions are exposed to application layer.
 */

#ifndef BSP_H
#define BSP_H

#include <stdbool.h>
#include <stdint.h>

/* =========================================================================
 * LIDAR & TOF ARENA PERCEPTION PARAMETERS
 * Target cans: Slim 25cL (diam 53 mm, H 134 mm) & Sleek 33cL (diam 58 mm, H 145 mm)
 * Wide objects (> 95 mm: arena walls, legs, obstacles) are automatically rejected.
 * ========================================================================= */
#define BSP_LIDAR_SCAN_HALF_DEG     180     /* 180 = full 360-deg scan around robot */
#define BSP_LIDAR_ANGLE_OFFSET_DEG  0       /* Chassis mounting angle offset so 0 deg = robot front */
#define BSP_LIDAR_MIN_DIST_MM       100     /* Min LiDAR range in mm (filters cover glare < 10 cm) */
#define BSP_LIDAR_MAX_DIST_MM       1000    /* Max target lock distance on arena table (mm) */
#define BSP_LIDAR_MAP_ZOOM_MM       500     /* 2D OLED map vertical radius in mm (smaller = more zoom) */

#define BSP_LIDAR_CLUSTER_JUMP_MM   70      /* Depth jump (mm) splitting two adjacent objects */
#define BSP_LIDAR_MAX_GAP_DEG       2       /* Bridge up to 2 missing degrees inside one surface */
#define BSP_LIDAR_CAN_MIN_WIDTH_MM  15      /* Min apparent chord width of a can (mm) */
#define BSP_LIDAR_CAN_MAX_WIDTH_MM  95      /* Max apparent chord width (> 95 mm = wall, ignored) */

#define BSP_TOF_GRIPPER_MM          80      /* ToF threshold (mm) where can is inside gripper jaws */

/*
 * Initialize Board Support Package hardware and launch periodic sensor/input task.
 * Called in main() prior to vTaskStartScheduler().
 */
void bsp_init(unsigned int task_priority);

/*
 * Initialize OLED display hardware and launch asynchronous DMA display refresh task.
 * Called prior to app_init() so display framebuffer is ready.
 */
void bsp_init_display(unsigned int task_priority);

/* System uptime clock in milliseconds */
uint32_t bsp_get_tick_ms(void);

/* Status LED and NeoPixel 12-LED ring controls */
void bsp_status_led_set(bool on);
void bsp_status_led_toggle(void);
void bsp_neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void bsp_neopixel_set_hsv(int hue, int sat, int val);
void bsp_neopixel_clear(void);
void bsp_leds_process(void);

/* OLED 128x64 display primitives */
void bsp_oled_clear(void);
void bsp_oled_fill_rect(int x, int y, int w, int h, bool on);
void bsp_oled_draw_string(int x, int y, const char *str, bool on);
void bsp_oled_show(void);

/* Actuators: DRV8833 dual DC motors, SER0039 gripper servo, buzzer */
void bsp_motors_set_speed(int16_t left_permille, int16_t right_permille);
/* Consigne de vitesse en boucle fermee (PI a 100 Hz), en mm/s par roue.
 * Consigne 0 : roue libre (avec la rampe BSP_SPEED_ACCEL_MM_S2 : ralentissement progressif, puis roue libre).
 * bsp_motors_set_speed(), bsp_motors_brake() et bsp_motors_coast() coupent la boucle. */
void bsp_motors_set_target_speed(float left_mm_s, float right_mm_s);
void bsp_motors_brake(void);
void bsp_motors_coast(void);
void bsp_gripper_grab(void);
void bsp_gripper_release(void);
void bsp_gripper_set_angle(uint16_t angle_deg);

/* Buzzer sound effects identifiers */
typedef enum {
    BSP_SOUND_NONE = 0,
    BSP_SOUND_BOOT,          /* Ascending power-on arpeggio */
    BSP_SOUND_CAN_FOUND,     /* Positive detection double-beep */
    BSP_SOUND_GRIP_SUCCESS,  /* Victory mini-fanfare */
    BSP_SOUND_CAN_DROPPED,   /* Mission complete chime */
    BSP_SOUND_ERROR,         /* Descending error buzz */
    BSP_SOUND_BEEP_SHORT,    /* Crisp UI click */
    BSP_SOUND_MATCH_START    /* Long referee whistle */
} bsp_sound_t;

/* Wireless BLE / VCP command identifiers forwarded via EVT_BLE_CMD */
typedef enum {
    BSP_BLE_CMD_NONE = 0,
    BSP_BLE_CMD_START,       /* "START" or "S" */
    BSP_BLE_CMD_STOP,        /* "STOP", "HALT", "IDLE", or "H" */
    BSP_BLE_CMD_RESET,       /* "RESET" or "RST" */
    BSP_BLE_CMD_MAP,         /* "MAP" or "M" (force persistent 2D LiDAR map view) */
    BSP_BLE_CMD_MANUAL_FWD,  /* "FWD" or "F" */
    BSP_BLE_CMD_MANUAL_BWD,  /* "BWD" or "B" */
    BSP_BLE_CMD_MANUAL_LFT,  /* "LFT" or "L" */
    BSP_BLE_CMD_MANUAL_RGT,  /* "RGT" or "R" */
    BSP_BLE_CMD_CUSTOM
} bsp_ble_cmd_t;

void bsp_buzzer_play(uint8_t sound_id);
void bsp_buzzer_stop(void);

/* Sensor, power management, and BLE accessors */
void bsp_get_tof_distances(uint16_t *left_mm, uint16_t *right_mm);
bool bsp_get_lidar_target(uint16_t *dist_mm, int16_t *angle_deg);
uint16_t bsp_get_lidar_target_width(void);
const uint16_t* bsp_get_lidar_distances(float *scan_hz);
uint8_t bsp_get_detected_color(void);
const char* bsp_get_color_name(void);
void bsp_get_color_raw(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c, uint8_t *prox);
float bsp_get_imu_yaw(void);
uint16_t bsp_get_battery_mv(void);
uint8_t bsp_get_battery_soc(void);
void bsp_ble_send(const char *msg);

#endif /* BSP_H */
