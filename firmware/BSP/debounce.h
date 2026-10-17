#ifndef BSP_DEBOUNCE_H
#define BSP_DEBOUNCE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Anti-rebond, sous forme de machine a etats a deux etats -- une par
 * ligne (USER_BTN, PWR_BUTTON). Purement cote BSP : contrairement a
 * la file d'evenements ou aux timers logiciels, elle n'a pas besoin
 * du service de timers de l'app, juste d'une horloge brute, donc
 * elle reste entierement locale a bsp.c.
 */
typedef enum {
    DEBOUNCE_STABLE,
    DEBOUNCE_COOLDOWN,
} debounce_state_t;

typedef struct {
    debounce_state_t state;
    uint32_t cooldown_start_ms;
    bool was_pressed; /* etat lu au tour precedent, pour detecter le front */
} debounce_t;

void debounce_init(debounce_t *d);

/*
 * A appeler une fois par tour de scrutation avec l'etat brut actuellement
 * lu sur la broche (pressed = true si le bouton est actuellement enfonce).
 * Detecte lui-meme le front (transition relache -> presse depuis le tour
 * precedent) et le filtre. Renvoie true si cet appui doit etre accepte
 * comme evenement (premier front vu en DEBOUNCE_STABLE) ; false sinon
 * (pas de front, ou rebond pendant DEBOUNCE_COOLDOWN).
 */
bool debounce_process(debounce_t *d, bool pressed, uint32_t now_ms);

#endif /* BSP_DEBOUNCE_H */
