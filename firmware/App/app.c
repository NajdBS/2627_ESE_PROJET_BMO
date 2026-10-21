/*
 * app.c
 *
 * Application Active Object implementation for BMO Robot.
 * Aucune dependance directe envers STM32 HAL : tout passe par bsp.h.
 */

#include "app.h"

#include <stddef.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "bsp.h"
#include "app_timers.h"
#include "bmo_screen.h"

#define APP_TASK_STACK_DEPTH    512     /* en mots de 32 bits */
#define APP_EVENT_QUEUE_LENGTH  16

/*
 * Constantes originales dans le TD du cours :
 * #define APP_UI_TIMER_ID         0
 * #define APP_MISSION_TIMER_ID    1
 * Remplacees par l'enum app_timer_id_t (APP_TIMER_ANIM, APP_TIMER_STATE) dans app_timers.h.
 */

#define APP_UI_TICK_MS          100     /* Procedural animation refresh rate (10 Hz) */

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_SEARCH,
    APP_STATE_GRABBING,
    APP_STATE_ERROR_LOW_BAT
} app_state_t;

typedef struct {
    TaskHandle_t  task;         /* garde dans l'objet (utile pour les notifications) */
    QueueHandle_t events;       /* file privee de l'objet actif */
    app_timers_t  timers;

    app_state_t   state;
    uint32_t      next_ui_tick_ms;
    uint8_t       cans_collected;
} app_t;

/* Singleton prive de l'application */
static app_t app;

static void app_task(void *unused);
static void app_handle_event(app_event_t *evt);

void app_init(unsigned int task_priority) {
    app_timers_init(&app.timers);

    app.state = APP_STATE_IDLE;
    app.cans_collected = 0;

    BMO_Screen_Init();
    BMO_Screen_SetFace(BMO_FACE_NORMAL);
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

    for (;;) {
        /* Les timers postent EVT_TIMER depuis la tache timer de FreeRTOS :
         * l'application peut attendre indefiniment sur sa file. */
        xQueueReceive(app.events, &evt, portMAX_DELAY);
        app_handle_event(&evt);
    }
}

/* Run-to-completion : un handler va jusqu'au bout et ne bloque jamais */
static void app_handle_event(app_event_t *evt) {
    uint32_t now = bsp_get_tick_ms();

    /* 1. Evenement global prioritaire : Batterie faible */
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

    /* 2. Rafraichissement periodique de l'animation du visage BMO (sans derive) */
    if (evt->type == EVT_TIMER && evt->data == APP_TIMER_ANIM) {
        app.next_ui_tick_ms += APP_UI_TICK_MS;
        if ((int32_t)(now - app.next_ui_tick_ms) > 0) {
            app.next_ui_tick_ms = now + APP_UI_TICK_MS; /* resynchronisation */
        }
        app_timer_start(&app.timers, APP_TIMER_ANIM, app.next_ui_tick_ms);

        float vbat = (float)bsp_get_battery_mv() / 1000.0f;
        BMO_Screen_SetTelemetry(vbat, app.cans_collected,
                                (app.state == APP_STATE_SEARCH) ? "SEARCH" :
                                (app.state == APP_STATE_GRABBING) ? "GRAB" : "IDLE");
        BMO_Screen_Update(now);
        return;
    }

    /* 3. Machine a etats principale de BMO */
    switch (app.state) {
        case APP_STATE_IDLE:
            if (evt->type == EVT_BTN_USER_CLICK ||
                (evt->type == EVT_BLE_CMD && evt->data == 1)) { /* BLE_CMD_START */
                app.state = APP_STATE_SEARCH;
                bsp_neopixel_set_rgb(0, 30, 15);
                BMO_Screen_SetFace(BMO_FACE_LOOKING);
                bsp_ble_send("ACK:MATCH_START\r\n");
            }
            break;

        case APP_STATE_SEARCH:
            if (evt->type == EVT_BTN_USER_CLICK ||
                (evt->type == EVT_BLE_CMD && evt->data == 2)) { /* BLE_CMD_STOP */
                app.state = APP_STATE_IDLE;
                bsp_motors_brake();
                bsp_neopixel_clear();
                BMO_Screen_SetFace(BMO_FACE_NORMAL);
                bsp_ble_send("ACK:STOP\r\n");
            } else if (evt->type == EVT_TOF_OBJECT) {
                app.state = APP_STATE_GRABBING;
                bsp_motors_brake();
                bsp_gripper_grab();
                bsp_neopixel_set_rgb(0, 40, 0);
                BMO_Screen_SetFace(BMO_FACE_HAPPY);
                app_timer_start(&app.timers, APP_TIMER_STATE, now + 800);
            }
            break;

        case APP_STATE_GRABBING:
            if (evt->type == EVT_TIMER && evt->data == APP_TIMER_STATE) {
                app.cans_collected++;
                app.state = APP_STATE_IDLE;
                BMO_Screen_SetFace(BMO_FACE_WINK);
            }
            break;

        case APP_STATE_ERROR_LOW_BAT:
            break;
    }
}
