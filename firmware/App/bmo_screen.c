#include "bmo_screen.h"
#include <stdio.h>
#include <string.h>

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

void BMO_Screen_Init(void) {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    last_blink_time = 0;
    is_blinking = 0;
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

        default:
            current_face = BMO_FACE_NORMAL;
            break;
    }

    /* Start asynchronous non-blocking DMA transfer to OLED */
    ssd1306_UpdateScreen_DMA();
}
