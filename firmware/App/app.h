/*
 * app.h
 *
 * Application Active Object interface for BMO Robot.
 */

#ifndef APP_H
#define APP_H

#include <stdbool.h>

#include "events.h"

/* Fonction a appeler dans le main, avant vTaskStartScheduler().
 * L'app est un objet actif : app_init cree sa file d'evenements et sa
 * propre tache. La priorite est choisie par le main : c'est une decision
 * systeme (qui passe avant qui), pas une decision de l'app. */
void app_init(unsigned int task_priority);

/* Interface publique de l'objet actif : seul point d'entree vers sa file
 * d'evenements (privee). Ne bloque jamais (timeout 0) : renvoie false si
 * la file est pleine.
 * - app_post_event          : depuis une tache
 * - app_post_event_from_isr : depuis une interruption */
bool app_post_event(app_event_t evt);
bool app_post_event_from_isr(app_event_t evt);

#endif /* APP_H */
