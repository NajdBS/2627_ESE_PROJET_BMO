#include "app_timers.h"

#include "app.h"
#include "bsp.h"

/* Callback des timers FreeRTOS. Il s'execute dans le contexte de la tache
 * timer (daemon), pas dans celui de l'app : il doit etre court et ne
 * jamais bloquer (il retarderait tous les autres timers). Il se contente
 * de poster un evenement : le traitement se fait dans l'app. */
static void app_timer_callback(TimerHandle_t handle) {
    int id = (int)(intptr_t)pvTimerGetTimerID(handle);
    app_post_event((app_event_t){.type = EVT_TIMER, .data = id});   /* timeout 0 */
}

void app_timers_init(app_timers_t *timers) {
    for (int i = 0; i < APP_TIMER_COUNT; i++) {
        /* One-shot (pdFALSE), periode provisoire : fixee a chaque start.
         * L'identifiant du timer est range dans le timer lui-meme. */
        timers->handles[i] = xTimerCreate("APP", 1, pdFALSE,
                (void *)(intptr_t)i, app_timer_callback);
        configASSERT(timers->handles[i] != NULL);
    }
}

void app_timer_start(app_timers_t *timers, int id, uint32_t deadline_ms) {
    if (id < 0 || id >= APP_TIMER_COUNT) {
        return;
    }
    /* Echeance -> duree. Comparaison signee sur la difference : sure au
     * wraparound. Une echeance deja depassee tire au prochain tick. */
    int32_t delay_ms = (int32_t)(deadline_ms - bsp_get_tick_ms());
    TickType_t delay = (delay_ms > 0) ? pdMS_TO_TICKS(delay_ms) : 0;
    if (delay == 0) {
        delay = 1;  /* une periode nulle est interdite */
    }

    /* Commande envoyee a la tache timer via sa file. Timeout 0 : l'app ne
     * bloque jamais (run-to-completion). Si la file de commandes est
     * pleine, la commande est perdue : on le signale. */
    BaseType_t ok = xTimerChangePeriod(timers->handles[id], delay, 0);  /* change + demarre */
    configASSERT(ok == pdPASS);
}

void app_timer_stop(app_timers_t *timers, int id) {
    if (id < 0 || id >= APP_TIMER_COUNT) {
        return;
    }
    BaseType_t ok = xTimerStop(timers->handles[id], 0);
    configASSERT(ok == pdPASS);
}
