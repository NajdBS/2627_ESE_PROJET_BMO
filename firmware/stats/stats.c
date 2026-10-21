/*
 * stats.c
 *
 * Objet "stats" : affiche des statistiques toutes les secondes.
 *
 * Pourquoi une tache dediee, et pas un handler de l'app ?
 * - printf sur l'UART, en polling : des dizaines de ms. Dans un handler de
 *   l'app, ca casserait le run-to-completion.
 * - vTaskList() et vTaskGetRunTimeStats() suspendent le scheduler pendant
 *   leur execution : ce sont des outils de debug, a garder en priorite basse.
 */

#include "stats.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#define STATS_STACK_DEPTH 256   /* en mots de 32 bits */
#define STATS_PERIOD_MS   1000

static char stats_buffer[512];

static void stats_task(void *unused) {
    (void)unused;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STATS_PERIOD_MS));

        /* Uptime calcule depuis le tick : pas de derive, meme si la tache
         * est retardee par des taches plus prioritaires */
        unsigned long uptime_s = (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ);
        printf("\r\n--- uptime: %lu s ---\r\n", uptime_s);

        /* 1 -- Le tas : taille, libre, minimum jamais atteint */
        printf("Heap: total=%u B, free=%u B, min_ever_free=%u B\r\n",
               (unsigned int)configTOTAL_HEAP_SIZE,
               (unsigned int)xPortGetFreeHeapSize(),
               (unsigned int)xPortGetMinimumEverFreeHeapSize());

        /* 2 -- Les piles : high water mark de chaque tache */
#if (configUSE_TRACE_FACILITY == 1) && (configUSE_STATS_FORMATTING_FUNCTIONS == 1)
        printf("Task\t\tState\tPrio\tStack\tNum\r\n");
        vTaskList(stats_buffer);
        printf("%s", stats_buffer);
#endif

        /* 3 -- Le temps CPU */
#if (configGENERATE_RUN_TIME_STATS == 1) && (configUSE_STATS_FORMATTING_FUNCTIONS == 1)
        printf("Task\t\tAbs Time\t%% Time\r\n");
        vTaskGetRunTimeStats(stats_buffer);
        printf("%s", stats_buffer);
#endif
    }
}

void stats_init(unsigned int task_priority) {
    BaseType_t ok = xTaskCreate(stats_task, "STATS", STATS_STACK_DEPTH,
            NULL, task_priority, NULL);
    configASSERT(ok == pdPASS);
}

/* -------------------------------------------------------------------------
 * Compteur de temps pour les statistiques d'execution.
 * Actif une fois GENERATE_RUN_TIME_STATS active dans CubeMX, et TIM7
 * configure a 10 kHz (prescaler = 17000-1), ARR = 0xFFFF.
 * ------------------------------------------------------------------------- */
#if (configGENERATE_RUN_TIME_STATS == 1)

#include "tim.h"

void configureTimerForRunTimeStats(void) {
    HAL_TIM_Base_Start(&htim7);
}

/* TIM7 compte sur 16 bits, FreeRTOS veut 32 bits. La difference sur 16 bits
 * reste correcte quand le compteur deborde, a condition d'etre appelee au
 * moins une fois par debordement (6,5 s a 10 kHz) : FreeRTOS l'appelle a
 * chaque changement de contexte, et la BSP se reveille toutes les 10 ms. */
unsigned long getRunTimeCounterValue(void) {
    static uint16_t prev = 0;
    static uint32_t total = 0;

    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim7);
    total += (uint16_t)(cnt - prev);
    prev = cnt;
    return total;
}

#endif /* configGENERATE_RUN_TIME_STATS */
