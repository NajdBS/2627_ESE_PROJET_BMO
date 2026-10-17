#include "debounce.h"

#define DEBOUNCE_PERIOD_MS 200

void debounce_init(debounce_t *d) {
    d->state = DEBOUNCE_STABLE;
    d->cooldown_start_ms = 0;
    d->was_pressed = false;
}

bool debounce_process(debounce_t *d, bool pressed, uint32_t now_ms) {
    bool raw_edge = pressed && !d->was_pressed;
    d->was_pressed = pressed;

    switch (d->state) {
        case DEBOUNCE_STABLE:
            if (!raw_edge) {
                return false;
            }
            /* Entree en DEBOUNCE_COOLDOWN : ce front est accepte, les
             * suivants seront ignores tant que la periode n'est pas ecoulee. */
            d->state = DEBOUNCE_COOLDOWN;
            d->cooldown_start_ms = now_ms;
            return true;

        case DEBOUNCE_COOLDOWN:
            /* Soustraction non signee : sure au wraparound de now_ms, meme
             * principe que les timers logiciels. */
            if (now_ms - d->cooldown_start_ms >= DEBOUNCE_PERIOD_MS) {
                d->state = DEBOUNCE_STABLE;
            }
            return false; /* jamais accepte tant qu'on est en cooldown */
    }
    return false;
}
