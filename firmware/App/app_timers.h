#ifndef APP_TIMERS_H
#define APP_TIMERS_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "timers.h"     /* timers FreeRTOS (d'ou le renommage en app_timers) */

#include "events.h"

/*
 * Service de timers de l'app, reimplemente au-dessus des timers logiciels
 * FreeRTOS.
 *
 * Chaque timer est arme avec une echeance (un instant, pas une duree) --
 * comme un timer Linux, pas un intervalle qui se repete tout seul. Il
 * tire une fois puis se desarme (timer FreeRTOS "one-shot"). Pour du
 * periodique, c'est a l'appelant de rearmer en reponse a EVT_TIMER, avec
 * la nouvelle echeance de son choix (typiquement l'ancienne echeance +
 * une periode).
 *
 * A l'echeance, le callback s'execute dans la tache timer de FreeRTOS
 * (daemon), pas dans l'app : il se contente de poster EVT_TIMER{.data = id}
 * dans la file de l'app. Plus besoin de scruter les timers : la tache de
 * l'app peut bloquer indefiniment sur sa file.
 */

/*
 * Code original du TD4 RTOS :
 * #define APP_TIMER_COUNT 4
 */

/* BMO Application Timer Identifiers */
typedef enum {
    APP_TIMER_ANIM = 0,    /* Procedural facial animation tick (10 Hz) */
    APP_TIMER_HUD,         /* Telemetry & radar screen update tick */
    APP_TIMER_STATE,       /* State machine non-blocking delay / timeout */
    APP_TIMER_COUNT        /* Total allocated software timers */
} app_timer_id_t;

typedef struct {
    TimerHandle_t handles[APP_TIMER_COUNT];
} app_timers_t;

/* Cree les timers FreeRTOS (dans le tas). A appeler avant d'armer un timer. */
void app_timers_init(app_timers_t *timers);

/* Arme (ou rearme) le timer `id` pour qu'il tire a `deadline_ms` (horloge
 * bsp_get_tick_ms). Ne fait rien si `id` est hors bornes. */
void app_timer_start(app_timers_t *timers, int id, uint32_t deadline_ms);

/* Desarme le timer `id`. Ne fait rien si `id` est hors bornes.
 * Attention : un EVT_TIMER deja poste dans la file de l'app peut encore
 * arriver apres l'arret. */
void app_timer_stop(app_timers_t *timers, int id);

#endif /* APP_TIMERS_H */
