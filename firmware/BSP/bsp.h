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
void bsp_motors_brake(void);
void bsp_motors_coast(void);
void bsp_gripper_grab(void);
void bsp_gripper_release(void);
void bsp_gripper_set_angle(uint16_t angle_deg);
void bsp_buzzer_play(uint8_t sound_id);
void bsp_buzzer_stop(void);

/* Sensor, power management, and BLE accessors */
void bsp_get_tof_distances(uint16_t *left_mm, uint16_t *right_mm);
float bsp_get_imu_yaw(void);
uint16_t bsp_get_battery_mv(void);
uint8_t bsp_get_battery_soc(void);
void bsp_ble_send(const char *msg);

#endif /* BSP_H */
