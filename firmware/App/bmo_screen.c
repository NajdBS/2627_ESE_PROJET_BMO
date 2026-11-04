#include "bmo_screen.h"
#include "bmo_bitmaps.h"
#include "bsp.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Current active emotion displayed on the robot */
static bmo_face_t current_face = BMO_FACE_NORMAL;

/* Auto-blink timer state for the default face */
static uint32_t last_blink_time = 0;
static uint8_t is_blinking = 0;

/* Eye tracking sweep parameters for search mode */
static int8_t look_offset = 0;
static int8_t look_dir = 1;
static uint32_t last_look_time = 0;

/* Cached telemetry values for the diagnostic screen */
static float telem_vbat = 3.92f;
static uint8_t telem_cans = 0;
static char telem_state[16] = "IDLE";

/* Cached 2D LiDAR radar map state */
static const uint16_t *radar_distances = NULL;
static uint16_t radar_max_range_mm = 1000;
static bool     radar_can_locked = false;
static uint16_t radar_obj_dist_mm = 0;
static int16_t  radar_obj_angle_deg = 0;
static uint16_t radar_obj_width_mm = 0;

void BMO_Screen_SetLidarData(float scan_hz, uint16_t fwd_mm, uint16_t rgt_mm, uint16_t bck_mm, uint16_t lft_mm, uint16_t min_mm, uint16_t min_deg) {
    (void)scan_hz; (void)fwd_mm; (void)rgt_mm; (void)bck_mm; (void)lft_mm;
    radar_obj_dist_mm = min_mm;
    radar_obj_angle_deg = (int16_t)min_deg;
}

void BMO_Screen_SetRadarMap(const uint16_t *distances_360,
                            uint16_t max_range_mm,
                            bool can_locked,
                            uint16_t obj_dist_mm,
                            int16_t obj_angle_deg,
                            uint16_t obj_width_mm,
                            uint16_t tof_mm) {
    (void)tof_mm;
    radar_distances     = distances_360;
    radar_max_range_mm  = (max_range_mm > 100) ? max_range_mm : 1000;
    radar_can_locked    = can_locked;
    radar_obj_dist_mm   = obj_dist_mm;
    radar_obj_angle_deg = obj_angle_deg;
    radar_obj_width_mm  = obj_width_mm;
}

/* Status overlay drawn around animated faces (header bar + bottom line) */
static char    status_line[22] = "";
static uint8_t status_progress = BMO_SCREEN_PROGRESS_HIDDEN;

void BMO_Screen_SetStatusLine(const char *text) {
    snprintf(status_line, sizeof(status_line), "%s", (text != NULL) ? text : "");
}

void BMO_Screen_SetProgress(uint8_t percent) {
    status_progress = (percent > 100 && percent != BMO_SCREEN_PROGRESS_HIDDEN) ? 100 : percent;
}

static void draw_header_bar_text(const char *left_text) {
    char buf[12];
    ssd1306_FillRectangle(0, 0, 127, 9, White);
    ssd1306_SetCursor(2, 1);
    ssd1306_WriteString((char *)left_text, Font_6x8, Black);
    snprintf(buf, sizeof(buf), "CAN:%u", telem_cans);
    ssd1306_SetCursor((uint8_t)(126 - 6 * strlen(buf)), 1);
    ssd1306_WriteString(buf, Font_6x8, Black);
}

/* Header: inverted bar with state name (left) and can counter (right).
 * Footer: centered status text, or a progress bar + text when progress is active. */
static void draw_status_overlay(void) {
    draw_header_bar_text(telem_state);

    /* Footer (y 54..63) */
    size_t len = strlen(status_line);
    if (status_progress != BMO_SCREEN_PROGRESS_HIDDEN) {
        /* Progress frame on the left 50 px, text on the right */
        ssd1306_DrawRectangle(1, 55, 50, 62, White);
        uint8_t w = (uint8_t)((46u * status_progress) / 100u);
        if (w > 0) {
            ssd1306_FillRectangle(3, 57, (uint8_t)(3 + w - 1), 60, White);
        }
        if (len > 12) len = 12;
        char txt[13];
        memcpy(txt, status_line, len);
        txt[len] = '\0';
        ssd1306_SetCursor(55, 55);
        ssd1306_WriteString(txt, Font_6x8, White);
    } else if (len > 0) {
        uint8_t x = (uint8_t)((128 - 6 * len) / 2);
        ssd1306_SetCursor(x, 55);
        ssd1306_WriteString(status_line, Font_6x8, White);
    }
}

/* Radar scale style:
 *  1 = Full graduated bottom ruler (... 60 | 40 | 20 | 0 | 20 | 40 | 60 cm)
 *  0 = Polar 8-dot distance rings + compact corner scale bar */
#define BMO_RADAR_RULER_STYLE   1

#if (BMO_RADAR_RULER_STYLE != 0)
/* Compact 3x5 pixel micro-digits (5 rows, 3 bits per row) for precision ruler ticks */
static const uint8_t micro_digits_3x5[10][5] = {
    {0x7, 0x5, 0x5, 0x5, 0x7}, /* 0 */
    {0x2, 0x6, 0x2, 0x2, 0x7}, /* 1 */
    {0x7, 0x1, 0x7, 0x4, 0x7}, /* 2 */
    {0x7, 0x1, 0x7, 0x1, 0x7}, /* 3 */
    {0x5, 0x5, 0x7, 0x1, 0x1}, /* 4 */
    {0x7, 0x4, 0x7, 0x1, 0x7}, /* 5 */
    {0x7, 0x4, 0x7, 0x5, 0x7}, /* 6 */
    {0x7, 0x1, 0x1, 0x2, 0x2}, /* 7 */
    {0x7, 0x5, 0x7, 0x5, 0x7}, /* 8 */
    {0x7, 0x5, 0x7, 0x1, 0x7}  /* 9 */
};

static void draw_micro_digit(int x, int y, uint8_t d) {
    if (d > 9) return;
    for (int r = 0; r < 5; r++) {
        uint8_t row = micro_digits_3x5[d][r];
        for (int c = 0; c < 3; c++) {
            if ((row & (1u << (2 - c))) && (x + c >= 0) && (x + c < 128)) {
                ssd1306_DrawPixel((uint8_t)(x + c), (uint8_t)(y + r), White);
            }
        }
    }
}

/* Draw 5x4 lowercase 'm' glyph at (x, y+1..y+4) for metric "1m" label */
static void draw_micro_m(int x, int y) {
    static const uint8_t glyph_m[4] = {0x1A, 0x15, 0x15, 0x15}; /* ##.#. / #.#.# */
    for (int r = 0; r < 4; r++) {
        uint8_t row = glyph_m[r];
        for (int c = 0; c < 5; c++) {
            if ((row & (1u << (4 - c))) && (x + c >= 0) && (x + c < 128)) {
                ssd1306_DrawPixel((uint8_t)(x + c), (uint8_t)(y + 1 + r), White);
            }
        }
    }
}

/* Draw ruler distance label centered at cx: "1m", "2m" at whole meters, else cm ("0", "20".."80") */
static void draw_micro_ruler_label_centered(int cx, int y, unsigned int cm) {
    if (cm > 0u && (cm % 100u) == 0u && (cm / 100u) <= 9u) {
        /* 3px digit + 1px gap + 5px 'm' = 9px total width */
        int x = cx - 4;
        draw_micro_digit(x, y, (uint8_t)(cm / 100u));
        draw_micro_m(x + 4, y);
        return;
    }
    char s[6];
    int n = snprintf(s, sizeof(s), "%u", cm);
    if (n <= 0) return;
    int total_w = n * 4 - 1;
    int x = cx - total_w / 2;
    for (int i = 0; i < n; i++) {
        draw_micro_digit(x + i * 4, y, (uint8_t)(s[i] - '0'));
    }
}
#endif

/* Render full-screen 128x54 2D LiDAR map with true-scale ruler and FOV indicator */
static void draw_lidar_radar_map(void) {
    char hdr[24];
    if (radar_can_locked && radar_obj_dist_mm > 0) {
        unsigned int dist_cm = (unsigned int)((radar_obj_dist_mm + 5u) / 10u);
        snprintf(hdr, sizeof(hdr), "CAN %umm@%ucm", radar_obj_width_mm, dist_cm);
    } else if (radar_obj_dist_mm > 0) {
        snprintf(hdr, sizeof(hdr), "WALL DETECTED");
    } else {
        snprintf(hdr, sizeof(hdr), "SEARCHING...");
    }
    draw_header_bar_text(hdr);

    const int mcx = 64;
#if (BMO_RADAR_RULER_STYLE != 0)
    const int mcy = 35;          /* 48 cm front range (up) and 36 cm rear range above ruler */
    const int max_plot_y = 53;
#else
    const int mcy = 37;
    const int max_plot_y = 63;
#endif
    const int mrad_y = 25;
    const float px_per_mm = (float)mrad_y / (float)radar_max_range_mm;
    const float deg_to_rad = 0.01745329f;
    const uint16_t step_mm = (radar_max_range_mm <= 600) ? 200 : 500;

#if (BMO_RADAR_RULER_STYLE == 0)
    /* Style 0: Subtle 8-dot polar distance rings at 20 cm and 40 cm */
    static const float cos45 = 0.70710678f;
    for (int ring = 1; ring <= 2; ring++) {
        int r_px = (int)((float)(step_mm * ring) * px_per_mm + 0.5f);
        int d_px = (int)((float)r_px * cos45 + 0.5f);
        if (mcy - r_px >= 11) ssd1306_DrawPixel((uint8_t)mcx, (uint8_t)(mcy - r_px), White);
        if (mcy + r_px <= 63) ssd1306_DrawPixel((uint8_t)mcx, (uint8_t)(mcy + r_px), White);
        if (mcx - r_px >= 1)  ssd1306_DrawPixel((uint8_t)(mcx - r_px), (uint8_t)mcy, White);
        if (mcx + r_px <= 126) ssd1306_DrawPixel((uint8_t)(mcx + r_px), (uint8_t)mcy, White);
        if (mcy - d_px >= 11 && mcx + d_px <= 126) ssd1306_DrawPixel((uint8_t)(mcx + d_px), (uint8_t)(mcy - d_px), White);
        if (mcy - d_px >= 11 && mcx - d_px >= 1)   ssd1306_DrawPixel((uint8_t)(mcx - d_px), (uint8_t)(mcy - d_px), White);
        if (mcy + d_px <= 63 && mcx + d_px <= 126) ssd1306_DrawPixel((uint8_t)(mcx + d_px), (uint8_t)(mcy + d_px), White);
        if (mcy + d_px <= 63 && mcx - d_px >= 1)   ssd1306_DrawPixel((uint8_t)(mcx - d_px), (uint8_t)(mcy + d_px), White);
    }
#endif

    /* Optional FOV cone rays if search sector < 360 deg */
#if (BSP_LIDAR_SCAN_HALF_DEG < 180)
    {
        float fov_rad = (float)BSP_LIDAR_SCAN_HALF_DEG * deg_to_rad;
        int fx = (int)(sinf(fov_rad) * 14.0f);
        int fy = (int)(cosf(fov_rad) * 14.0f);
        ssd1306_Line((uint8_t)mcx, (uint8_t)mcy, (uint8_t)(mcx - fx), (uint8_t)(mcy - fy), White);
        ssd1306_Line((uint8_t)mcx, (uint8_t)mcy, (uint8_t)(mcx + fx), (uint8_t)(mcy - fy), White);
    }
#endif

    /* Directional chevron for BMO pointing forward (up = 0 deg) */
    ssd1306_Line(mcx, (uint8_t)(mcy - 3), mcx - 2, (uint8_t)(mcy + 2), White);
    ssd1306_Line(mcx, (uint8_t)(mcy - 3), mcx + 2, (uint8_t)(mcy + 2), White);
    ssd1306_Line(mcx - 1, (uint8_t)(mcy + 1), mcx + 1, (uint8_t)(mcy + 1), White);

    /* Plot all 360 live LiDAR echoes across the viewport (ignoring <10cm cover glare) */
    if (radar_distances != NULL) {
        for (int deg = 0; deg < 360; deg++) {
            uint16_t d = radar_distances[deg];
            if (d >= BSP_LIDAR_MIN_DIST_MM) {
                float rad = (float)deg * deg_to_rad;
                int px = (int)((float)mcx + sinf(rad) * (float)d * px_per_mm + 1000.5f) - 1000;
                int py = (int)((float)mcy - cosf(rad) * (float)d * px_per_mm + 1000.5f) - 1000;
                if (px >= 1 && px <= 126 && py >= 11 && py <= max_plot_y) {
                    ssd1306_DrawPixel((uint8_t)px, (uint8_t)py, White);
                }
            }
        }
    }

    /* Highlight closest detected can with a r=3 circular reticle */
    if (radar_can_locked && radar_obj_dist_mm > 0) {
        float rad = (float)radar_obj_angle_deg * deg_to_rad;
        int tx = (int)((float)mcx + sinf(rad) * (float)radar_obj_dist_mm * px_per_mm + 1000.5f) - 1000;
        int ty = (int)((float)mcy - cosf(rad) * (float)radar_obj_dist_mm * px_per_mm + 1000.5f) - 1000;
        if (tx < 4)   tx = 4;
        if (tx > 123) tx = 123;
        if (ty < 14)  ty = 14;
        if (ty > max_plot_y - 3) ty = max_plot_y - 3;
        ssd1306_DrawCircle((uint8_t)tx, (uint8_t)ty, 3, White);
    }

#if (BMO_RADAR_RULER_STYLE != 0)
    /* Style 1: Full graduated true-scale bottom ruler (1m | 80 | 60 | 40 | 20 | 0 | 20 | 40 | 60 | 80 | 1m) */
    ssd1306_FillRectangle(0, 54, 127, 63, Black);
    ssd1306_Line(2, 56, 125, 56, White);

    /* Center 0 tick aligned directly under BMO (x = 64) */
    ssd1306_Line(mcx, 54, mcx, 57, White);
    draw_micro_ruler_label_centered(mcx, 59, 0);

    const uint16_t half_step_mm = step_mm / 2u;
    for (int k = 1; k <= 12; k++) {
        int offset_px = (int)((float)(half_step_mm * (uint16_t)k) * px_per_mm + 0.5f);
        int xl = mcx - offset_px;
        int xr = mcx + offset_px;
        bool is_major = ((k & 1) == 0);
        uint8_t tick_top = is_major ? 54 : 55;
        unsigned int cm = (unsigned int)((step_mm * (unsigned int)(k / 2)) / 10u);

        if (xl >= 3) {
            ssd1306_Line((uint8_t)xl, tick_top, (uint8_t)xl, 57, White);
            if (is_major && xl >= 10) {
                draw_micro_ruler_label_centered(xl, 59, cm);
            }
        }
        if (xr <= 124) {
            ssd1306_Line((uint8_t)xr, tick_top, (uint8_t)xr, 57, White);
            if (is_major && xr <= 118) {
                draw_micro_ruler_label_centered(xr, 59, cm);
            }
        }
    }
#else
    /* Style 0: Bottom-left compact scale bar (|---| 20cm) + Bottom-right FOV badge (360°) */
    int ruler_px = (int)((float)step_mm * px_per_mm + 0.5f);
    if (ruler_px < 6)  ruler_px = 6;
    if (ruler_px > 24) ruler_px = 24;
    char scale_lbl[10];
    snprintf(scale_lbl, sizeof(scale_lbl), "%ucm", (unsigned)(step_mm / 10u));

    uint8_t lbl_x = (uint8_t)(4 + ruler_px);
    uint8_t box_end_x = (uint8_t)(lbl_x + 6 * strlen(scale_lbl));
    ssd1306_FillRectangle(0, 54, box_end_x, 63, Black);
    ssd1306_Line(2, 60, (uint8_t)(2 + ruler_px), 60, White);
    ssd1306_Line(2, 58, 2, 62, White);
    ssd1306_Line((uint8_t)(2 + ruler_px), 58, (uint8_t)(2 + ruler_px), 62, White);
    ssd1306_SetCursor(lbl_x, 55);
    ssd1306_WriteString(scale_lbl, Font_6x8, White);

    char fov_lbl[8];
    snprintf(fov_lbl, sizeof(fov_lbl), "%u", (unsigned)(BSP_LIDAR_SCAN_HALF_DEG * 2));
    uint8_t fov_w = (uint8_t)(6 * strlen(fov_lbl));
    uint8_t fov_x = (uint8_t)(123 - fov_w);
    ssd1306_FillRectangle((uint8_t)(fov_x - 1), 54, 127, 63, Black);
    ssd1306_SetCursor(fov_x, 55);
    ssd1306_WriteString(fov_lbl, Font_6x8, White);
    ssd1306_DrawRectangle(124, 55, 125, 56, White);
#endif
}

void BMO_Screen_Init(void) {
    current_face = BMO_FACE_NORMAL;
    last_blink_time = 0;
    is_blinking = 0;
    look_offset = 0;
}

void BMO_Screen_SetFace(bmo_face_t face) {
    current_face = face;
    is_blinking = 0;
    look_offset = 0;
}

bmo_face_t BMO_Screen_GetFace(void) {
    return current_face;
}

void BMO_Screen_SetTelemetry(float vbat, uint8_t cans, const char* state_str) {
    telem_vbat = vbat;
    telem_cans = cans;
    if (state_str != NULL) {
        snprintf(telem_state, sizeof(telem_state), "%s", state_str);
    }
}

/* Helper to render BMO's subtle upward-curving smile */
static void draw_mouth_smile(uint8_t cx, uint8_t cy) {
    ssd1306_Line(cx - 7, cy + 12, cx + 7, cy + 12, White);
    ssd1306_DrawPixel(cx - 8, cy + 11, White);
    ssd1306_DrawPixel(cx + 8, cy + 11, White);
    ssd1306_DrawPixel(cx - 9, cy + 10, White);
    ssd1306_DrawPixel(cx + 9, cy + 10, White);
}

/* =========================================================================
 * 10-SECOND STARTUP SPLASH DANCE ANIMATION (BMO_FACE_FULL_BODY)
 * Animates BMO's 3D chassis bounce/sway, waving arms, kicking legs,
 * dynamic facial expressions, and floating musical notes at 10 FPS.
 * ========================================================================= */
#define BMO_SPLASH_DANCE_ENABLE     1

#if (BMO_SPLASH_DANCE_ENABLE != 0)
static void blit_splash_region(int x0, int y0, int x1, int y1, int dx, int dy) {
    for (int y = y0; y <= y1; y++) {
        int dst_y = y + dy;
        if (dst_y < 0 || dst_y >= 64) continue;
        const uint8_t *row_ptr = &bmo_splash_bitmap[y * 16];
        for (int x = x0; x <= x1; x++) {
            if (row_ptr[x >> 3] & (0x80u >> (x & 7))) {
                int dst_x = x + dx;
                if (dst_x >= 0 && dst_x < 128) {
                    ssd1306_DrawPixel((uint8_t)dst_x, (uint8_t)dst_y, White);
                }
            }
        }
    }
}

static void draw_thick_limb(int x0, int y0, int x1, int y1) {
    if (x0 < 0) x0 = 0; else if (x0 > 127) x0 = 127;
    if (x1 < 0) x1 = 0; else if (x1 > 127) x1 = 127;
    if (y0 < 0) y0 = 0; else if (y0 > 63)  y0 = 63;
    if (y1 < 0) y1 = 0; else if (y1 > 63)  y1 = 63;
    ssd1306_Line((uint8_t)x0, (uint8_t)y0, (uint8_t)x1, (uint8_t)y1, White);
    if (y0 + 1 <= 63 && y1 + 1 <= 63) {
        ssd1306_Line((uint8_t)x0, (uint8_t)(y0 + 1), (uint8_t)x1, (uint8_t)(y1 + 1), White);
    }
}

static void draw_music_note(int x, int y) {
    if (x < 0 || x + 4 > 127 || y < 0 || y + 5 > 63) return;
    ssd1306_Line((uint8_t)(x + 2), (uint8_t)y, (uint8_t)(x + 2), (uint8_t)(y + 5), White);
    ssd1306_DrawPixel((uint8_t)(x + 3), (uint8_t)y, White);
    ssd1306_DrawPixel((uint8_t)(x + 4), (uint8_t)(y + 1), White);
    ssd1306_FillRectangle((uint8_t)x, (uint8_t)(y + 4), (uint8_t)(x + 2), (uint8_t)(y + 5), White);
}

static void draw_dancing_bmo_splash(uint32_t now_ms) {
    /* 8-step dance groove (200 ms per beat = 1.6 s full cycle) */
    static const int8_t body_dx_tbl[8] = { 0,  1,  1,  0,  0, -1, -1,  0};
    static const int8_t body_dy_tbl[8] = { 1, -2, -3, -1,  1, -2, -3, -1};

    uint8_t step = (uint8_t)((now_ms / 200u) % 8u);
    int dx = body_dx_tbl[step];
    int dy = body_dy_tbl[step];

    /* 1. Draw right-side banner ("BMO! ROBOT ONLINE") steady */
    blit_splash_region(68, 0, 127, 63, 0, 0);

    /* 2. Draw BMO 3D body chassis (x = 6..58, y = 5..51) with bounce & sway */
    blit_splash_region(6, 5, 58, 51, dx, dy);

    /* Erase static tucked left arm from 3D side panel while preserving seam & "BMO" logo */
    ssd1306_FillRectangle((uint8_t)(23 + dx), (uint8_t)(22 + dy), (uint8_t)(25 + dx), (uint8_t)(24 + dy), Black);
    ssd1306_FillRectangle((uint8_t)(16 + dx), (uint8_t)(24 + dy), (uint8_t)(21 + dx), (uint8_t)(27 + dy), Black);
    ssd1306_Line((uint8_t)(22 + dx), (uint8_t)(22 + dy), (uint8_t)(22 + dx), (uint8_t)(28 + dy), White);

    /* 3. Animate BMO's screen face (interior: x = 27..53, y = 10..28 + offset) */
    ssd1306_FillRectangle((uint8_t)(27 + dx), (uint8_t)(10 + dy), (uint8_t)(53 + dx), (uint8_t)(28 + dy), Black);
    int lx = 34 + dx;
    int rx = 46 + dx;
    int ey = 15 + dy;
    int mx = 40 + dx;
    int my = 22 + dy;

    if (step == 1 || step == 2 || step == 6) {
        /* Joyful arch eyes (^ ^) during high jumps */
        ssd1306_Line((uint8_t)(lx - 2), (uint8_t)(ey + 1), (uint8_t)lx, (uint8_t)(ey - 1), White);
        ssd1306_Line((uint8_t)lx, (uint8_t)(ey - 1), (uint8_t)(lx + 2), (uint8_t)(ey + 1), White);
        ssd1306_Line((uint8_t)(rx - 2), (uint8_t)(ey + 1), (uint8_t)rx, (uint8_t)(ey - 1), White);
        ssd1306_Line((uint8_t)rx, (uint8_t)(ey - 1), (uint8_t)(rx + 2), (uint8_t)(ey + 1), White);
        if (step == 2) {
            /* Singing 'o' mouth */
            ssd1306_DrawCircle((uint8_t)mx, (uint8_t)my, 2, White);
        } else {
            /* Big open 'D' laugh */
            ssd1306_Line((uint8_t)(mx - 4), (uint8_t)(my - 1), (uint8_t)(mx + 4), (uint8_t)(my - 1), White);
            ssd1306_Line((uint8_t)(mx - 3), (uint8_t)(my + 2), (uint8_t)(mx + 3), (uint8_t)(my + 2), White);
            ssd1306_Line((uint8_t)(mx - 4), (uint8_t)my, (uint8_t)(mx - 4), (uint8_t)(my + 1), White);
            ssd1306_Line((uint8_t)(mx + 4), (uint8_t)my, (uint8_t)(mx + 4), (uint8_t)(my + 1), White);
        }
    } else if (step == 3) {
        /* Playful wink (; o) + smirk */
        ssd1306_Line((uint8_t)(lx - 2), (uint8_t)ey, (uint8_t)(lx + 2), (uint8_t)ey, White);
        ssd1306_FillRectangle((uint8_t)(rx - 1), (uint8_t)(ey - 1), (uint8_t)(rx + 1), (uint8_t)(ey + 1), White);
        ssd1306_Line((uint8_t)(mx - 2), (uint8_t)(my + 1), (uint8_t)(mx + 3), (uint8_t)(my + 1), White);
        ssd1306_DrawPixel((uint8_t)(mx + 4), (uint8_t)my, White);
    } else if (step == 7) {
        /* Happy blink (- -) + smile */
        ssd1306_Line((uint8_t)(lx - 2), (uint8_t)ey, (uint8_t)(lx + 2), (uint8_t)ey, White);
        ssd1306_Line((uint8_t)(rx - 2), (uint8_t)ey, (uint8_t)(rx + 2), (uint8_t)ey, White);
        ssd1306_Line((uint8_t)(mx - 3), (uint8_t)(my + 1), (uint8_t)(mx + 3), (uint8_t)(my + 1), White);
        ssd1306_DrawPixel((uint8_t)(mx - 4), (uint8_t)my, White);
        ssd1306_DrawPixel((uint8_t)(mx + 4), (uint8_t)my, White);
    } else {
        /* Eyes tracking dance direction (step 0: right, step 4 & 5: left) */
        int look = (step == 0) ? 1 : -1;
        ssd1306_FillRectangle((uint8_t)(lx - 1 + look), (uint8_t)(ey - 1), (uint8_t)(lx + 1 + look), (uint8_t)(ey + 1), White);
        ssd1306_FillRectangle((uint8_t)(rx - 1 + look), (uint8_t)(ey - 1), (uint8_t)(rx + 1 + look), (uint8_t)(ey + 1), White);
        ssd1306_Line((uint8_t)(mx - 3), (uint8_t)(my + 1), (uint8_t)(mx + 3), (uint8_t)(my + 1), White);
        ssd1306_DrawPixel((uint8_t)(mx - 4), (uint8_t)my, White);
        ssd1306_DrawPixel((uint8_t)(mx + 4), (uint8_t)my, White);
    }

    /* 4. Animate Left & Right Arms waving in rhythm */
    int lsx = 23 + dx, lsy = 24 + dy;
    int rsx = 58 + dx, rsy = 27 + dy;

    /* Left arm poses: shoulder -> elbow -> waving hand */
    if (step == 2 || step == 5 || step == 6) {
        /* Left arm high up (\o/) */
        draw_thick_limb(lsx, lsy, 8 + dx, 18 + dy);
        draw_thick_limb(8 + dx, 18 + dy, 2, 11 + dy);
        ssd1306_DrawPixel(1, (uint8_t)(10 + dy), White);
        ssd1306_DrawPixel(4, (uint8_t)(10 + dy), White);
    } else if (step == 1 || step == 3 || step == 7) {
        /* Left arm mid-wave out */
        draw_thick_limb(lsx, lsy, 10 + dx, 25 + dy);
        draw_thick_limb(10 + dx, 25 + dy, 2, 21 + dy);
        ssd1306_DrawPixel(1, (uint8_t)(20 + dy), White);
        ssd1306_DrawPixel(1, (uint8_t)(23 + dy), White);
    } else {
        /* Left arm down / groove */
        draw_thick_limb(lsx, lsy, 13 + dx, 29 + dy);
        draw_thick_limb(13 + dx, 29 + dy, 3, 33 + dy);
    }

    /* Right arm poses: shoulder -> elbow -> waving hand */
    if (step == 1 || step == 2 || step == 6) {
        /* Right arm high up (\o/) */
        draw_thick_limb(rsx, rsy, 62 + dx, 22 + dy);
        draw_thick_limb(62 + dx, 22 + dy, 65, 14 + dy);
        ssd1306_DrawPixel(64, (uint8_t)(13 + dy), White);
        ssd1306_DrawPixel(66, (uint8_t)(14 + dy), White);
    } else if (step == 3 || step == 5 || step == 7) {
        /* Right arm mid-wave */
        draw_thick_limb(rsx, rsy, 63 + dx, 27 + dy);
        draw_thick_limb(63 + dx, 27 + dy, 66, 22 + dy);
    } else {
        /* Right arm on hip */
        draw_thick_limb(rsx, rsy, 63 + dx, 33 + dy);
        draw_thick_limb(63 + dx, 33 + dy, 60 + dx, 39 + dy);
    }

    /* 5. Animate Left & Right Legs (kick, squat, mid-air jump) */
    int lhx = 30 + dx, rhx = 48 + dx, hy = 51 + dy;
    if (step == 0 || step == 4) {
        /* Knee-bend squat (< >) */
        draw_thick_limb(lhx, hy, lhx - 3, hy + 4);
        draw_thick_limb(lhx - 3, hy + 4, lhx, hy + 8);
        ssd1306_Line((uint8_t)(lhx - 2), (uint8_t)(hy + 8), (uint8_t)(lhx + 2), (uint8_t)(hy + 8), White);

        draw_thick_limb(rhx, hy, rhx + 3, hy + 4);
        draw_thick_limb(rhx + 3, hy + 4, rhx, hy + 8);
        ssd1306_Line((uint8_t)(rhx - 1), (uint8_t)(hy + 8), (uint8_t)(rhx + 3), (uint8_t)(hy + 8), White);
    } else if (step == 1) {
        /* Plant left leg, high-kick right leg! */
        draw_thick_limb(lhx, hy, lhx, hy + 9);
        ssd1306_Line((uint8_t)(lhx - 2), (uint8_t)(hy + 9), (uint8_t)(lhx + 2), (uint8_t)(hy + 9), White);

        draw_thick_limb(rhx, hy, rhx + 9, hy + 4);
        ssd1306_Line((uint8_t)(rhx + 9), (uint8_t)(hy + 4), (uint8_t)(rhx + 10), (uint8_t)(hy + 2), White);
    } else if (step == 5) {
        /* High-kick left leg, plant right leg! */
        draw_thick_limb(lhx, hy, lhx - 9, hy + 4);
        ssd1306_Line((uint8_t)(lhx - 9), (uint8_t)(hy + 4), (uint8_t)(lhx - 10), (uint8_t)(hy + 2), White);

        draw_thick_limb(rhx, hy, rhx, hy + 9);
        ssd1306_Line((uint8_t)(rhx - 1), (uint8_t)(hy + 9), (uint8_t)(rhx + 3), (uint8_t)(hy + 9), White);
    } else if (step == 2 || step == 6) {
        /* Joyful mid-air jump (/ \) */
        draw_thick_limb(lhx, hy, lhx - 5, hy + 6);
        ssd1306_DrawPixel((uint8_t)(lhx - 6), (uint8_t)(hy + 5), White);

        draw_thick_limb(rhx, hy, rhx + 5, hy + 6);
        ssd1306_DrawPixel((uint8_t)(rhx + 6), (uint8_t)(hy + 5), White);
    } else {
        /* Groove toe-tap */
        int slant = (step == 3) ? 2 : -2;
        draw_thick_limb(lhx, hy, lhx + slant, hy + 8);
        ssd1306_Line((uint8_t)(lhx + slant - 1), (uint8_t)(hy + 8), (uint8_t)(lhx + slant + 2), (uint8_t)(hy + 8), White);

        draw_thick_limb(rhx, hy, rhx + slant, hy + 8);
        ssd1306_Line((uint8_t)(rhx + slant - 1), (uint8_t)(hy + 8), (uint8_t)(rhx + slant + 2), (uint8_t)(hy + 8), White);
    }

    /* 6. Floating musical eighth-notes rising smoothly at 10 FPS on the right */
    uint32_t tick10 = now_ms / 100u;
    int ny1 = 52 - (int)(tick10 % 44u);
    int ny2 = 52 - (int)((tick10 + 22u) % 44u);
    int nx_sway = ((tick10 & 2u) != 0u) ? 1 : 0;
    draw_music_note(116 + nx_sway, ny1);
    draw_music_note(122 - nx_sway, ny2);
}
#endif

void BMO_Screen_Update(uint32_t now_ms) {
    /* Prevent buffer corruption while DMA hardware is actively transmitting */
    if (ssd1306_IsBusy()) {
        return;
    }

    /* Screen center coordinates (128x64 display) */
    const uint8_t cx = 64;
    const uint8_t cy = 32;

    ssd1306_Fill(Black);

    /* Automatic eye blink logic: closes eyes for ~140ms every 2.8s */
    if (current_face == BMO_FACE_NORMAL) {
        if (!is_blinking && (now_ms - last_blink_time > 2800)) {
            is_blinking = 1;
            last_blink_time = now_ms;
        } else if (is_blinking && (now_ms - last_blink_time > 140)) {
            is_blinking = 0;
            last_blink_time = now_ms;
        }
    }

    switch (current_face) {
        case BMO_FACE_NORMAL:
            if (is_blinking) {
                /* Horizontal bar eyes during blink */
                ssd1306_FillRectangle(39, cy - 5, 51, cy - 3, White);
                ssd1306_FillRectangle(77, cy - 5, 89, cy - 3, White);
            } else {
                /* Standard circular solid eyes */
                ssd1306_FillCircle(45, cy - 4, 5, White);
                ssd1306_FillCircle(83, cy - 4, 5, White);
            }
            draw_mouth_smile(cx, cy);
            break;

        case BMO_FACE_LOOKING:
            /* Smooth horizontal eye sweep for LiDAR target scanning */
            if (now_ms - last_look_time > 70) {
                last_look_time = now_ms;
                look_offset += look_dir;
                if (look_offset >= 6 || look_offset <= -6) {
                    look_dir = -look_dir;
                }
            }
            ssd1306_FillCircle(45 + look_offset, cy - 4, 5, White);
            ssd1306_FillCircle(83 + look_offset, cy - 4, 5, White);
            ssd1306_DrawCircle(cx, cy + 12, 3, White); /* Curious "o" mouth */
            break;

        case BMO_FACE_HAPPY:
            /* Inverted V arch eyes (^ ^) */
            ssd1306_Line(39, cy - 2, 45, cy - 7, White);
            ssd1306_Line(45, cy - 7, 51, cy - 2, White);
            ssd1306_Line(77, cy - 2, 83, cy - 7, White);
            ssd1306_Line(83, cy - 7, 89, cy - 2, White);

            /* Wide open semicircular smile */
            ssd1306_FillCircle(cx, cy + 8, 8, White);
            ssd1306_FillRectangle(cx - 9, cy, cx + 9, cy + 8, Black);
            ssd1306_Line(cx - 8, cy + 8, cx + 8, cy + 8, White);
            break;

        case BMO_FACE_SHOCK:
            /* Wide outer circle with small inner pupil */
            ssd1306_DrawCircle(45, cy - 5, 7, White);
            ssd1306_FillCircle(45, cy - 5, 2, White);
            ssd1306_DrawCircle(83, cy - 5, 7, White);
            ssd1306_FillCircle(83, cy - 5, 2, White);
            ssd1306_DrawCircle(cx, cy + 13, 5, White); /* Surprised "O" mouth */
            break;

        case BMO_FACE_WINK:
            /* Left eye closed, right eye open */
            ssd1306_FillRectangle(39, cy - 5, 51, cy - 3, White);
            ssd1306_FillCircle(83, cy - 4, 5, White);
            /* Asymmetrical smirk mouth */
            ssd1306_Line(cx - 4, cy + 12, cx + 8, cy + 12, White);
            ssd1306_DrawPixel(cx + 9, cy + 11, White);
            ssd1306_DrawPixel(cx + 10, cy + 10, White);
            break;

        case BMO_FACE_TELEMETRY: {
            char buf[32];
            ssd1306_SetCursor(12, 4);
            ssd1306_WriteString("== BMO ROBOT ==", Font_7x10, White);

            int v_int = (int)telem_vbat;
            int v_dec = (int)((telem_vbat - (float)v_int) * 100.0f);
            if (v_dec < 0) {
                v_dec = -v_dec;
            }
            snprintf(buf, sizeof(buf), "BAT : %d.%02d V", v_int, v_dec);
            ssd1306_SetCursor(4, 20);
            ssd1306_WriteString(buf, Font_7x10, White);

            snprintf(buf, sizeof(buf), "CANS: %d / 3", telem_cans);
            ssd1306_SetCursor(4, 34);
            ssd1306_WriteString(buf, Font_7x10, White);

            snprintf(buf, sizeof(buf), "ETAT: %s", telem_state);
            ssd1306_SetCursor(4, 48);
            ssd1306_WriteString(buf, Font_7x10, White);
            break;
        }

        case BMO_FACE_FULL_BODY:
#if (BMO_SPLASH_DANCE_ENABLE != 0)
            draw_dancing_bmo_splash(now_ms);
#else
            ssd1306_DrawBitmap(0, 0, bmo_splash_bitmap, 128, 64, White);
#endif
            break;

        case BMO_FACE_LIDAR_RADAR:
            draw_lidar_radar_map();
            break;

        default:
            current_face = BMO_FACE_NORMAL;
            break;
    }

    /* Header/footer status overlay on animated faces only (not on full-screen pages) */
    if (current_face <= BMO_FACE_WINK) {
        draw_status_overlay();
    }

    /* Request asynchronous refresh from bsp_display_task (run-to-completion) */
    bsp_oled_show();
}
