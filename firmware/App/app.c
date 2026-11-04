/*
 * app.c
 *
 * Application Active Object implementation for BMO Robot.
 * Aucune dependance directe envers STM32 HAL : tout passe par bsp.h.
 */

#include "app.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "bsp.h"
#include "app_timers.h"
#include "bmo_screen.h"

#define APP_TASK_STACK_DEPTH        512     /* en mots de 32 bits */
#define APP_EVENT_QUEUE_LENGTH      16

/*
 * Constantes originales dans le TD du cours :
 * #define APP_UI_TIMER_ID         0
 * #define APP_MISSION_TIMER_ID    1
 * Remplacees par l'enum app_timer_id_t (APP_TIMER_ANIM, APP_TIMER_STATE) dans app_timers.h.
 */

#define APP_UI_TICK_MS              100     /* Procedural animation / radar map refresh rate (10 Hz) */
#define APP_APPROACH_TIMEOUT_MS     8000    /* Return to SEARCH if ToF not reached within 8 s */
#define APP_COLOR_VERIFY_MS         5000    /* Color analysis window */

/* =========================================================================
 * SEARCH DISPLAY MODE (Option 2)
 *  1 = Show real-time 2D LiDAR radar map (walls + locked can) during SEARCH
 *  0 = Show procedural BMO_FACE_LOOKING eye-sweep animation during SEARCH
 * ========================================================================= */
#define BMO_SEARCH_SHOW_RADAR_MAP   1

#if (BMO_SEARCH_SHOW_RADAR_MAP != 0)
    #define APP_SEARCH_FACE         BMO_FACE_LIDAR_RADAR
#else
    #define APP_SEARCH_FACE         BMO_FACE_LOOKING
#endif

/* Consecutive 200 ms LiDAR scans required to confirm a can lock (4 x 200 ms = 0.8 s) */
#define APP_LIDAR_LOCK_STREAK       4

/* BLE + USB Terminal 2D ASCII map streaming (2 lines/100ms tick + 800ms pause between frames) */
#define APP_TERM_MAP_PAUSE_TICKS    8
#define APP_TERM_MAP_LINES_PER_TICK 2

/* White ring-light used as a lamp for the APDS-9960 during VERIFY_COLOR.
 * Tune brightness here (0..255 per channel): raise if colors read too dark, lower if saturated. */
#define APP_COLOR_LIGHT_LEVEL       50

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_MAP_VIEW,         /* Persistent 2D LiDAR map viewer (entered via MAP command) */
    APP_STATE_SEARCH,           /* 360-deg LiDAR can clustering */
    APP_STATE_APPROACH,         /* Approaching locked LiDAR can until ToF < 80 mm */
    APP_STATE_VERIFY_COLOR,     /* Can in gripper (<80 mm): APDS-9960 color analysis */
    APP_STATE_GRABBING,         /* Close gripper jaws and hold */
    APP_STATE_RELEASE,          /* Open gripper jaws, deposit can */
    APP_STATE_ERROR_LOW_BAT
} app_state_t;

typedef struct {
    TaskHandle_t  task;         /* kept in object (useful for notifications) */
    QueueHandle_t events;       /* private queue of the active object */
    app_timers_t  timers;

    app_state_t   state;
    uint32_t      next_ui_tick_ms;
    uint32_t      state_enter_ms;
    uint16_t      step_counter;
    uint8_t       log_tick_div;
    uint8_t       map_tx_line;
    uint8_t       lidar_lock_streak;
    uint8_t       color_match_streak;
    int32_t       last_color_candidate;
    uint8_t       cans_collected;
    int32_t       can_color;    /* Last confirmed color (0 = NONE/unknown, 1 R, 2 G, 3 B) */
} app_t;

/* Singleton prive de l'application */
static app_t app;

static void app_task(void *unused);
static void app_handle_event(app_event_t *evt);
static void app_enter_search(void);
static void app_enter_idle(void);
static void app_enter_map_view(void);
static void app_enter_verify_color(uint32_t now, int32_t tof_mm);

#if (BMO_DEBUG_ENABLE != 0)
static const char* app_color_bilingual_str(int32_t color_id) {
    switch (color_id) {
        case 1:  return "RED (ROUGE)";
        case 2:  return "GREEN (VERT)";
        case 3:  return "BLUE (BLEU)";
        default: return "UNKNOWN";
    }
}
#endif

/* =========================================================================
 * COMPACT 34-COL 2D ASCII RADAR MAP (BLE Smartphone + PC HTerm)
 * Streamed at 2 lines per 100 ms tick (~68 B/tick) so BLE 20-byte GATT
 * packets never overflow module UART buffers and lines never wrap on phones.
 * ========================================================================= */
#define TERM_MAP_COLS           31      /* c = 0..30 (-100 cm .. +100 cm, 3 cols = 20 cm) */
#define TERM_MAP_ROWS           8       /* r = 0..7  (+80 cm front .. -60 cm rear, 1 row = 20 cm) */
#define TERM_MAP_CX             15      /* Center column (0 cm) */
#define TERM_MAP_CY             4       /* Center row (0 cm) */
#define TERM_MAP_TOTAL_LINES    (TERM_MAP_ROWS + 5)

static char term_map_lines[TERM_MAP_TOTAL_LINES][52];

static void app_send_map_line(const char *s) {
    printf("%s", s);        /* USB VCP (HTerm) when BMO_DEBUG_ENABLE == 1 */
    bsp_ble_send(s);        /* BLE UART (Serial Bluetooth Terminal on phone) */
}

static void app_build_ascii_radar_frame(uint16_t frame_id, const uint16_t *lidar_map,
                                        bool can_found, uint16_t lidar_d, int16_t lidar_a,
                                        uint16_t lidar_w, uint16_t tof_mm)
{
    char grid[TERM_MAP_ROWS][TERM_MAP_COLS + 1];
    const float deg_to_rad = 0.01745329f;

    /* 1. Initialize border walls and 20 cm reference crosshairs */
    for (int r = 0; r < TERM_MAP_ROWS; r++) {
        for (int c = 0; c < TERM_MAP_COLS; c++) {
            char ch = ' ';
            if (c == 0 || c == TERM_MAP_COLS - 1) {
                ch = '+';
            } else if (r == TERM_MAP_CY) {
                ch = ((c % 3) == 0) ? '+' : '.';
            } else if (c == TERM_MAP_CX) {
                ch = '.';
            }
            grid[r][c] = ch;
        }
        grid[r][TERM_MAP_COLS] = '\0';
    }

    /* 2. Plot all 360 live LiDAR echoes ('#') inside interior (ignoring <10cm cover glare) */
    if (lidar_map != NULL) {
        for (int deg = 0; deg < 360; deg++) {
            uint16_t d = lidar_map[deg];
            if (d >= BSP_LIDAR_MIN_DIST_MM) {
                float rad = (float)deg * deg_to_rad;
                float x_mm = sinf(rad) * (float)d;
                float y_mm = cosf(rad) * (float)d;
                int c = (int)((float)TERM_MAP_CX + (x_mm * 3.0f / 200.0f) + 1000.5f) - 1000;
                int r = (int)((float)TERM_MAP_CY - (y_mm / 200.0f) + 1000.5f) - 1000;
                if (c >= 1 && c <= TERM_MAP_COLS - 2 && r >= 0 && r < TERM_MAP_ROWS) {
                    grid[r][c] = '#';
                }
            }
        }
    }

    /* 3. Place BMO forward marker '^' at (0, 0) */
    grid[TERM_MAP_CY][TERM_MAP_CX] = '^';

    /* 4. Highlight locked CAN target as '(@)' */
    if (can_found && lidar_d > 0) {
        float rad = (float)lidar_a * deg_to_rad;
        int tc = (int)((float)TERM_MAP_CX + (sinf(rad) * (float)lidar_d * 3.0f / 200.0f) + 1000.5f) - 1000;
        int tr = (int)((float)TERM_MAP_CY - (cosf(rad) * (float)lidar_d / 200.0f) + 1000.5f) - 1000;
        if (tc >= 1 && tc <= TERM_MAP_COLS - 2 && tr >= 0 && tr < TERM_MAP_ROWS) {
            if (!(tr == TERM_MAP_CY && tc == TERM_MAP_CX)) {
                grid[tr][tc] = '@';
            }
            if (tc > 1 && !(tr == TERM_MAP_CY && (tc - 1) == TERM_MAP_CX)) {
                grid[tr][tc - 1] = '(';
            }
            if (tc < TERM_MAP_COLS - 2 && !(tr == TERM_MAP_CY && (tc + 1) == TERM_MAP_CX)) {
                grid[tr][tc + 1] = ')';
            }
        }
    }

    /* 5. Format atomic 13-line frame (<= 34 visible chars/line for smartphone screens) */
    snprintf(term_map_lines[0], sizeof(term_map_lines[0]),
             "\r\n=== BMO MAP #%u (stop = exit) ===\r\n", frame_id);

    if (can_found && lidar_d > 0) {
        snprintf(term_map_lines[1], sizeof(term_map_lines[1]),
                 "CAN %umm @ %ucm (%+d deg) ToF:%u\r\n",
                 lidar_w, (unsigned)((lidar_d + 5u) / 10u), (int)lidar_a, tof_mm);
    } else if (lidar_d > 0) {
        snprintf(term_map_lines[1], sizeof(term_map_lines[1]),
                 "WALL %ucm (W:%umm) | ToF:%umm\r\n",
                 (unsigned)((lidar_d + 5u) / 10u), lidar_w, tof_mm);
    } else {
        snprintf(term_map_lines[1], sizeof(term_map_lines[1]),
                 "SEARCHING... | ToF:%umm\r\n", tof_mm);
    }

    snprintf(term_map_lines[2], sizeof(term_map_lines[2]),
             "  +--+--+--+--+--+--+--+--+--+--+\r\n");

    for (int r = 0; r < TERM_MAP_ROWS; r++) {
        int y_cm = (TERM_MAP_CY - r) * 20;
        if (y_cm < 0) y_cm = -y_cm;
        snprintf(term_map_lines[3 + r], sizeof(term_map_lines[3 + r]),
                 "%2d%s\r\n", y_cm, grid[r]);
    }

    snprintf(term_map_lines[3 + TERM_MAP_ROWS], sizeof(term_map_lines[3 + TERM_MAP_ROWS]),
             "  +--+--+--+--+--+--+--+--+--+--+\r\n");
    snprintf(term_map_lines[4 + TERM_MAP_ROWS], sizeof(term_map_lines[4 + TERM_MAP_ROWS]),
             " 1m 80 60 40 20  0  20 40 60 80 1m\r\n");
}

/* Ring color after analysis: R/G/B for a confirmed can, Yellow when color is NONE/unknown */
static void app_show_can_color(int32_t color_id) {
    switch (color_id) {
        case 1:  bsp_neopixel_set_rgb(50, 0, 0);  break;   /* Red */
        case 2:  bsp_neopixel_set_rgb(0, 50, 0);  break;   /* Green */
        case 3:  bsp_neopixel_set_rgb(0, 0, 50);  break;   /* Blue */
        default: bsp_neopixel_set_rgb(45, 35, 0); break;   /* Yellow: unknown */
    }
}

static const char* app_color_short_str(int32_t color_id) {
    switch (color_id) {
        case 1:  return "RED";
        case 2:  return "GREEN";
        case 3:  return "BLUE";
        default: return "UNKNOWN";
    }
}

static bool app_is_start_event(const app_event_t *evt) {
    return (evt->type == EVT_BTN_USER_CLICK) ||
           (evt->type == EVT_BLE_CMD && evt->data == BSP_BLE_CMD_START);
}

static bool app_is_stop_event(const app_event_t *evt) {
    return (evt->type == EVT_BTN_USER_CLICK) ||
           (evt->type == EVT_BLE_CMD && evt->data == BSP_BLE_CMD_STOP);
}

static void app_enter_search(void) {
    app_timer_stop(&app.timers, APP_TIMER_STATE);
    app.state = APP_STATE_SEARCH;
    app.step_counter = 0;
    app.log_tick_div = 0;
    app.map_tx_line = 0;
    app.lidar_lock_streak = 0;
    bsp_gripper_release();
    bsp_neopixel_set_rgb(0, 15, 35);        /* Cyan blue: search mode */
    BMO_Screen_SetFace(APP_SEARCH_FACE);
    bsp_ble_send("[BMO] -> SEARCH\r\n");

    /* --- Motor & Buzzer commands (commented out for bench test without motors) ---
     * bsp_motors_set_speed(150, -150);        // In-place rotation to scan 360 deg
     * bsp_buzzer_play(BSP_SOUND_MATCH_START);
     */
}

static void app_enter_idle(void) {
    app_timer_stop(&app.timers, APP_TIMER_STATE);
    app.state = APP_STATE_IDLE;
    app.map_tx_line = 0;
    app.lidar_lock_streak = 0;
    bsp_neopixel_clear();
    bsp_gripper_release();
    bsp_motors_coast();
    BMO_Screen_SetFace(BMO_FACE_NORMAL);
    bsp_ble_send("[BMO] -> IDLE\r\n");
}

static void app_enter_map_view(void) {
    app_timer_stop(&app.timers, APP_TIMER_STATE);
    app.state = APP_STATE_MAP_VIEW;
    app.step_counter = 0;
    app.log_tick_div = APP_TERM_MAP_PAUSE_TICKS;
    app.map_tx_line = 0;
    app.lidar_lock_streak = 0;
    bsp_motors_coast();
    bsp_gripper_release();
    bsp_neopixel_set_rgb(0, 20, 20);        /* Soft teal: passive map viewer */
    BMO_Screen_SetFace(BMO_FACE_LIDAR_RADAR);
}

static void app_enter_verify_color(uint32_t now, int32_t tof_mm) {
    app.state = APP_STATE_VERIFY_COLOR;
    app.state_enter_ms = now;
    app.step_counter = 0;
    app.log_tick_div = 0;
    app.color_match_streak = 0;
    app.last_color_candidate = 0;
    app.can_color = 0;
    bsp_motors_brake();
    /* Constant white lamp for the APDS-9960 during the whole analysis */
    bsp_neopixel_set_rgb(APP_COLOR_LIGHT_LEVEL, APP_COLOR_LIGHT_LEVEL, APP_COLOR_LIGHT_LEVEL);
    BMO_Screen_SetFace(BMO_FACE_LOOKING);
    app_timer_start(&app.timers, APP_TIMER_STATE, now + APP_COLOR_VERIFY_MS);
    printf("[FSM] -> VERIFY_COLOR (ToF at %ld mm < %d mm, analyzing APDS-9960 for 5s...)\r\n",
           (long)tof_mm, BSP_TOF_GRIPPER_MM);
}

void app_init(unsigned int task_priority) {
    app_timers_init(&app.timers);

    app.state = APP_STATE_IDLE;
    app.cans_collected = 0;
    app.step_counter = 0;
    app.log_tick_div = 0;
    app.lidar_lock_streak = 0;
    app.color_match_streak = 0;
    app.last_color_candidate = 0;

    BMO_Screen_Init();
    /* Display BMO Full-Body robot artwork at startup */
    BMO_Screen_SetFace(BMO_FACE_FULL_BODY);
    BMO_Screen_Update(0);

    /* Objet actif : la file et la tache sont creees par l'objet lui-meme */
    app.events = xQueueCreate(APP_EVENT_QUEUE_LENGTH, sizeof(app_event_t));
    configASSERT(app.events != NULL);
    vQueueAddToRegistry(app.events, "APP events");

    BaseType_t ok = xTaskCreate(app_task, "APP", APP_TASK_STACK_DEPTH,
            NULL, task_priority, &app.task);
    configASSERT(ok == pdPASS);
}

/* Timeout 0 : on ne bloque jamais. Les callbacks des timers postent ici
 * depuis la tache timer de FreeRTOS : s'ils bloquaient sur une file pleine,
 * ils bloqueraient tous les autres timers. */
bool app_post_event(app_event_t evt) {
    if (app.events == NULL) {
        return false;
    }
    return xQueueSend(app.events, &evt, 0) == pdPASS;
}

bool app_post_event_from_isr(app_event_t evt) {
    if (app.events == NULL) {
        return false;
    }
    BaseType_t woken = pdFALSE;
    BaseType_t ok = xQueueSendFromISR(app.events, &evt, &woken);
    portYIELD_FROM_ISR(woken);  /* bascule tout de suite si l'app est plus prioritaire */
    return ok == pdPASS;
}

/* Boucle de l'objet actif : le seul point de blocage est xQueueReceive */
static void app_task(void *unused) {
    (void)unused;   /* app est un singleton : pas besoin de passer l'objet */
    app_event_t evt;

    /* Demarre le timer periodique d'animation de l'ecran BMO */
    uint32_t now = bsp_get_tick_ms();
    app.next_ui_tick_ms = now + APP_UI_TICK_MS;
    app_timer_start(&app.timers, APP_TIMER_ANIM, app.next_ui_tick_ms);

    /* Hold BMO full-body startup splash screen for 10.0 seconds before showing normal face */
    app_timer_start(&app.timers, APP_TIMER_STATE, now + 10000);

    for (;;) {
        /* Les timers postent EVT_TIMER depuis la tache timer de FreeRTOS :
         * l'application peut attendre indefiniment sur sa file. */
        xQueueReceive(app.events, &evt, portMAX_DELAY);
        app_handle_event(&evt);
    }
}

/* Run-to-completion: handler executes to completion and never blocks */
static void app_handle_event(app_event_t *evt) {
    uint32_t now = bsp_get_tick_ms();

    /* 1. Global high-priority event: Low Battery */
    if (evt->type == EVT_BATTERY_LOW) {
        if (app.state != APP_STATE_ERROR_LOW_BAT) {
            app.state = APP_STATE_ERROR_LOW_BAT;
            bsp_motors_brake();
            bsp_neopixel_set_rgb(40, 0, 0);
            BMO_Screen_SetFace(BMO_FACE_SHOCK);
            BMO_Screen_Update(now);
        }
        return;
    }

    /* 2. Periodic BMO procedural face / 2D radar map refresh (10 Hz, drift-free) */
    if (evt->type == EVT_TIMER && evt->data == APP_TIMER_ANIM) {
        app.next_ui_tick_ms += APP_UI_TICK_MS;
        if ((int32_t)(now - app.next_ui_tick_ms) > 0) {
            app.next_ui_tick_ms = now + APP_UI_TICK_MS; /* resynchronize */
        }
        app_timer_start(&app.timers, APP_TIMER_ANIM, app.next_ui_tick_ms);

        float vbat = (float)bsp_get_battery_mv() / 1000.0f;
        const char *state_str = "IDLE";
        if (app.state == APP_STATE_MAP_VIEW) {
            state_str = "MAP";
            BMO_Screen_SetFace(BMO_FACE_LIDAR_RADAR);
        } else if (app.state == APP_STATE_SEARCH) {
            state_str = "SEARCH";
            BMO_Screen_SetFace(APP_SEARCH_FACE);
        } else if (app.state == APP_STATE_APPROACH)     state_str = "APPROACH";
        else if (app.state == APP_STATE_VERIFY_COLOR)   state_str = "COLOR";
        else if (app.state == APP_STATE_GRABBING)       state_str = "GRAB";
        else if (app.state == APP_STATE_RELEASE)        state_str = "RELEASE";
        else if (app.state == APP_STATE_ERROR_LOW_BAT)  state_str = "LOW_BAT";

        /* Live sensor snapshot (shared by OLED UI, 2D radar map, and console logs) */
        uint16_t tof_l = 0, tof_r = 0, lidar_d = 0;
        int16_t lidar_a = 0;
        bool can_found = bsp_get_lidar_target(&lidar_d, &lidar_a);
        uint16_t lidar_w = bsp_get_lidar_target_width();
        bsp_get_tof_distances(&tof_l, &tof_r);

        if (!can_found) {
            app.lidar_lock_streak = 0;
        }

        float scan_hz = 0.0f;
        const uint16_t *lidar_map = bsp_get_lidar_distances(&scan_hz);
        BMO_Screen_SetRadarMap(lidar_map, BSP_LIDAR_MAP_ZOOM_MM,
                               can_found, lidar_d, lidar_a, lidar_w, tof_l);

        int32_t rem_ms = (int32_t)APP_COLOR_VERIFY_MS - (int32_t)(now - app.state_enter_ms);
        if (rem_ms < 0) rem_ms = 0;

        /* OLED bottom status line, per state (used on animated faces) */
        char line[32];
        uint8_t progress = BMO_SCREEN_PROGRESS_HIDDEN;
        switch (app.state) {
            case APP_STATE_IDLE:
                snprintf(line, sizeof(line), "BTN=Start | MAP=2D");
                break;
            case APP_STATE_MAP_VIEW:
            case APP_STATE_SEARCH:
                if (can_found)       snprintf(line, sizeof(line), "%umm @ %ucm", lidar_w, (unsigned)((lidar_d + 5u) / 10u));
                else if (lidar_d > 0) snprintf(line, sizeof(line), "WALL DETECTED");
                else                 snprintf(line, sizeof(line), "SEARCHING...");
                break;
            case APP_STATE_APPROACH:
                if (tof_l < 2000) snprintf(line, sizeof(line), "ToF %umm (<%d)", tof_l, BSP_TOF_GRIPPER_MM);
                else              snprintf(line, sizeof(line), "Can %ucm -> ToF...", (unsigned)((lidar_d + 5u) / 10u));
                break;
            case APP_STATE_VERIFY_COLOR:
                progress = (uint8_t)(100 - (rem_ms * 100) / APP_COLOR_VERIFY_MS);
                snprintf(line, sizeof(line), "%s", bsp_get_color_name());
                break;
            case APP_STATE_GRABBING:
                snprintf(line, sizeof(line), "Grip: %s", app_color_short_str(app.can_color));
                break;
            case APP_STATE_RELEASE:
                snprintf(line, sizeof(line), "Dropped! Total %u", app.cans_collected);
                break;
            default:
                snprintf(line, sizeof(line), "Battery LOW!");
                break;
        }
        BMO_Screen_SetStatusLine(line);
        BMO_Screen_SetProgress(progress);

        BMO_Screen_SetTelemetry(vbat, app.cans_collected, state_str);
        BMO_Screen_Update(now);

        /* State-specific telemetry: BLE+USB streamed 2D ASCII radar in MAP_VIEW, 300ms logs elsewhere */
        if (app.state == APP_STATE_MAP_VIEW) {
            if (app.map_tx_line == 0) {
                if (++app.log_tick_div >= APP_TERM_MAP_PAUSE_TICKS) {
                    app.log_tick_div = 0;
                    app.step_counter++;
                    app_build_ascii_radar_frame(app.step_counter, lidar_map, can_found,
                                                lidar_d, lidar_a, lidar_w, tof_l);
                    for (uint8_t n = 0; n < APP_TERM_MAP_LINES_PER_TICK && app.map_tx_line < TERM_MAP_TOTAL_LINES; n++) {
                        app_send_map_line(term_map_lines[app.map_tx_line++]);
                    }
                }
            } else {
                for (uint8_t n = 0; n < APP_TERM_MAP_LINES_PER_TICK && app.map_tx_line < TERM_MAP_TOTAL_LINES; n++) {
                    app_send_map_line(term_map_lines[app.map_tx_line++]);
                }
                if (app.map_tx_line >= TERM_MAP_TOTAL_LINES) {
                    app.map_tx_line = 0;
                }
            }
        } else if (++app.log_tick_div >= 3) {
            app.log_tick_div = 0;

            if (app.state == APP_STATE_SEARCH) {
                app.step_counter++;
                if (can_found) {
                    printf("[SEARCH #%u] CAN (%u/%u) -> %u mm @ %u cm | Angle: %+4d deg\r\n",
                           app.step_counter, app.lidar_lock_streak, APP_LIDAR_LOCK_STREAK,
                           lidar_w, (unsigned)((lidar_d + 5u) / 10u), (int)lidar_a);
                } else if (lidar_d > 0) {
                    printf("[SEARCH #%u] WALL DETECTED (ignored) -> Dist: %4u mm | Angle: %+4d deg | Width: %3u mm\r\n",
                           app.step_counter, lidar_d, (int)lidar_a, lidar_w);
                } else {
                    printf("[SEARCH #%u] Scanning 360 (%d..%d mm): no echo\r\n",
                           app.step_counter, BSP_LIDAR_MIN_DIST_MM, BSP_LIDAR_MAX_DIST_MM);
                }
            } else if (app.state == APP_STATE_APPROACH) {
                app.step_counter++;
                printf("[APPROACH #%u] ToF: %4u mm (Trigger < %d mm) | LiDAR Can: %4u mm (%+d deg)\r\n",
                       app.step_counter, tof_l, BSP_TOF_GRIPPER_MM, lidar_d, (int)lidar_a);
            } else if (app.state == APP_STATE_VERIFY_COLOR) {
                uint16_t r = 0, g = 0, b = 0, c = 0;
                uint8_t prox = 0;
                bsp_get_color_raw(&r, &g, &b, &c, &prox);

                app.step_counter++;
                printf("[VERIFY_COLOR #%u | %ld.%lds rem] Color: %-5s (R:%3u G:%3u B:%3u C:%3u Prox:%3u) | ToF: %3u mm\r\n",
                       app.step_counter,
                       (long)(rem_ms / 1000),
                       (long)((rem_ms % 1000) / 100),
                       bsp_get_color_name(),
                       r, g, b, c, prox, tof_l);
            }
        }
        return;
    }

    /* 3. Global command: "MAP" forces persistent 2D LiDAR Map View from any state */
    if (evt->type == EVT_BLE_CMD && evt->data == BSP_BLE_CMD_MAP) {
        if (app.state != APP_STATE_ERROR_LOW_BAT) {
            app_enter_map_view();
            BMO_Screen_Update(now);
            printf("[FSM] -> MAP_VIEW (Persistent 2D LiDAR Map - send STOP or press BTN to exit)\r\n");
        }
        return;
    }

    /* 4. Main BMO Mission State Machine */
    switch (app.state) {
        case APP_STATE_IDLE:
            if (evt->type == EVT_TIMER && evt->data == APP_TIMER_STATE) {
                BMO_Screen_SetFace(BMO_FACE_NORMAL);
            } else if (app_is_start_event(evt)) {
                app_enter_search();
                printf("[FSM] IDLE -> SEARCH (Autonomous 360 LiDAR can hunt started)\r\n");
            }
            break;

        case APP_STATE_MAP_VIEW:
            /* Passive 2D map viewer: no automatic transitions until user sends STOP/BTN or START */
            if (app_is_stop_event(evt)) {
                app_enter_idle();
                printf("[FSM] MAP_VIEW -> IDLE\r\n");
            } else if (evt->type == EVT_BLE_CMD && evt->data == BSP_BLE_CMD_START) {
                app_enter_search();
                printf("[FSM] MAP_VIEW -> SEARCH (Autonomous mission started)\r\n");
            }
            break;

        case APP_STATE_SEARCH:
            if (app_is_stop_event(evt)) {
                app_enter_idle();
                printf("[FSM] SEARCH -> IDLE (Stopped by user/BLE)\r\n");
            } else if (evt->type == EVT_LIDAR_TARGET) {
                app.lidar_lock_streak++;
                if (app.lidar_lock_streak >= APP_LIDAR_LOCK_STREAK) {
                    uint16_t lidar_d = 0;
                    int16_t lidar_a = 0;
                    bsp_get_lidar_target(&lidar_d, &lidar_a);

                    app.state = APP_STATE_APPROACH;
                    app.step_counter = 0;
                    app.log_tick_div = 0;
                    bsp_neopixel_set_rgb(35, 18, 0);        /* Orange: can locked, approaching */
                    BMO_Screen_SetFace(BMO_FACE_NORMAL);

                    /* --- Motor & Buzzer commands during Approach ---
                     * bsp_motors_set_speed(250, 250);
                     * bsp_buzzer_play(BSP_SOUND_CAN_FOUND);
                     */
                    app_timer_start(&app.timers, APP_TIMER_STATE, now + APP_APPROACH_TIMEOUT_MS);

                    printf("[FSM] SEARCH -> APPROACH (Can LOCKED: %u mm @ %u cm, %+d deg)\r\n",
                           bsp_get_lidar_target_width(), (unsigned)((lidar_d + 5u) / 10u), (int)lidar_a);
                }
            }
            break;

        case APP_STATE_APPROACH:
            if (app_is_stop_event(evt)) {
                app_enter_idle();
                printf("[FSM] APPROACH -> IDLE (Stopped by user/BLE)\r\n");
            } else if (evt->type == EVT_TOF_OBJECT) {
                app_enter_verify_color(now, evt->data);
            } else if (evt->type == EVT_TIMER && evt->data == APP_TIMER_STATE) {
                app_enter_search();
                printf("[FSM] APPROACH -> SEARCH (Timeout: target not reached within 8s)\r\n");
            }
            break;

        case APP_STATE_VERIFY_COLOR:
            if (app_is_stop_event(evt)) {
                app_enter_idle();
                printf("[FSM] VERIFY_COLOR -> IDLE (Stopped by user/BLE)\r\n");
            } else if (evt->type == EVT_CAN_COLOR) {
                /* Require at least 2 printed telemetry lines and 2 consecutive identical color matches */
                if (app.step_counter >= 2) {
                    if (evt->data == app.last_color_candidate) {
                        app.color_match_streak++;
                    } else {
                        app.last_color_candidate = evt->data;
                        app.color_match_streak = 1;
                    }
                }
                if (app.color_match_streak >= 2) {
                    app_timer_stop(&app.timers, APP_TIMER_STATE);
                    app.state = APP_STATE_GRABBING;
                    app.can_color = evt->data;
                    bsp_motors_brake();
                    bsp_gripper_grab();
                    app_show_can_color(app.can_color);     /* Ring: confirmed R / G / B */
                    BMO_Screen_SetFace(BMO_FACE_HAPPY);

                    // bsp_buzzer_play(BSP_SOUND_GRIP_SUCCESS);

                    app_timer_start(&app.timers, APP_TIMER_STATE, now + 1500);
                    printf("[FSM] VERIFY_COLOR -> GRABBING (Can color: %s confirmed! Gripper grab)\r\n",
                           app_color_bilingual_str(evt->data));
                }
            } else if (evt->type == EVT_TIMER && evt->data == APP_TIMER_STATE) {
                /* 5s color timeout fallback: grab can based on ToF proximity, color unknown */
                app.state = APP_STATE_GRABBING;
                app.can_color = 0;
                bsp_motors_brake();
                bsp_gripper_grab();
                app_show_can_color(app.can_color);         /* Ring: Yellow = color unknown */
                BMO_Screen_SetFace(BMO_FACE_HAPPY);

                // bsp_buzzer_play(BSP_SOUND_GRIP_SUCCESS);

                app_timer_start(&app.timers, APP_TIMER_STATE, now + 1500);
                printf("[FSM] VERIFY_COLOR -> GRABBING (Proximity confirmed, color UNKNOWN -> Yellow. Gripper grab)\r\n");
            }
            break;

        case APP_STATE_GRABBING:
            if (app_is_stop_event(evt)) {
                app_enter_idle();
                printf("[FSM] GRABBING -> IDLE (Stopped by user/BLE)\r\n");
            } else if (evt->type == EVT_TIMER && evt->data == APP_TIMER_STATE) {
                app.cans_collected++;
                app.state = APP_STATE_RELEASE;
                bsp_gripper_release();
                bsp_neopixel_set_rgb(35, 0, 35);           /* Magenta / Purple */
                BMO_Screen_SetFace(BMO_FACE_WINK);

                /* --- Motors & Buzzer during Release ---
                 * bsp_motors_set_speed(-150, -150);
                 * bsp_buzzer_play(BSP_SOUND_CAN_DROPPED);
                 */

                app_timer_start(&app.timers, APP_TIMER_STATE, now + 1500);
                printf("[FSM] GRABBING -> RELEASE (Gripper open, can deposited. Total: %u)\r\n", app.cans_collected);
            }
            break;

        case APP_STATE_RELEASE:
            if (app_is_stop_event(evt)) {
                app_enter_idle();
                printf("[FSM] RELEASE -> IDLE (Stopped by user/BLE)\r\n");
            } else if (evt->type == EVT_TIMER && evt->data == APP_TIMER_STATE) {
                app_enter_search();
                printf("[FSM] RELEASE -> SEARCH (Resuming 360 LiDAR scan for next can...)\r\n");
            }
            break;

        case APP_STATE_ERROR_LOW_BAT:
            bsp_motors_brake();
            // bsp_buzzer_play(BSP_SOUND_ERROR);
            break;
    }
}
