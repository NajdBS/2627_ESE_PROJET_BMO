/*
 * stats.h
 *
 * Objet "stats" : une tache de priorite basse qui affiche des statistiques
 * sur la console (UART2) toutes les secondes.
 */

#ifndef STATS_STATS_H_
#define STATS_STATS_H_

/* A appeler dans le main, avant vTaskStartScheduler(). Comme la BSP et
 * l'app, l'objet cree sa propre tache ; le main choisit la priorite. */
void stats_init(unsigned int task_priority);

#endif /* STATS_STATS_H_ */
