#ifndef __BMO_SCREEN_H__
#define __BMO_SCREEN_H__

#include <stdint.h>


/*
 * BMO Facial Expression States
 * Procedural geometric faces rendered on the 128x64 OLED display.
 */
typedef enum {
    BMO_FACE_NORMAL = 0,   /* Default face with procedural auto-blink */
    BMO_FACE_LOOKING,      /* Eyes tracking left/right during LiDAR search */
    BMO_FACE_HAPPY,        /* Joyful arch eyes with wide smile (can detected) */
    BMO_FACE_SHOCK,        /* Wide eyes with small pupils (obstacle / cliff alert) */
    BMO_FACE_WINK,         /* Playful wink with smirk */
    BMO_FACE_TELEMETRY,    /* Diagnostics: battery voltage, collected cans, state */
    BMO_FACE_FULL_BODY,    /* Adventure Time BMO full-body pixel art */
    BMO_FACE_LIDAR_RADAR   /* Real-time 2D mini-radar & distance display */
} bmo_face_t;

/* Initialize BMO screen state (OLED hardware is initialized by bsp_init_display) */
void BMO_Screen_Init(void);

/* Set the current facial expression */
void BMO_Screen_SetFace(bmo_face_t face);

/* Return the currently active face */
bmo_face_t BMO_Screen_GetFace(void);

/* Update telemetry values shown on the dashboard screen */
void BMO_Screen_SetTelemetry(float vbat, uint8_t cans, const char* state_str);

/* Update live LiDAR telemetry and mini-radar visualization */
void BMO_Screen_SetLidarData(float scan_hz, uint16_t fwd_mm, uint16_t rgt_mm, uint16_t bck_mm, uint16_t lft_mm, uint16_t min_mm, uint16_t min_deg);

/*
 * Render current BMO face into the framebuffer and request a non-blocking
 * refresh from bsp_display_task via bsp_oled_show().
 */
void BMO_Screen_Update(uint32_t now_ms);


#endif /* __BMO_SCREEN_H__ */
