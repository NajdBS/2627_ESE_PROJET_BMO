/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   Passive Buzzer driver with musical notes & BMO sound effects
  ******************************************************************************
  */

#include "buzzer.h"
#include <string.h>

#define BUZZER_TIMER_CLOCK_HZ  1000000UL  /* 1 MHz tick (Prescaler = 170-1) */

/* Predefined Melodies for BMO */
static const buzzer_note_t s_sound_boot[] = {
    { NOTE_C5,  80 },
    { NOTE_E5,  80 },
    { NOTE_G5,  80 },
    { NOTE_C6, 200 }
};

static const buzzer_note_t s_sound_can_found[] = {
    { NOTE_G5,       70 },
    { NOTE_SILENCE,  30 },
    { NOTE_C6,      150 }
};

static const buzzer_note_t s_sound_grip_success[] = {
    { NOTE_C5,  80 },
    { NOTE_G5,  80 },
    { NOTE_C6, 100 },
    { NOTE_E6, 250 }
};

static const buzzer_note_t s_sound_can_dropped[] = {
    { NOTE_C6, 100 },
    { NOTE_G5, 180 }
};

static const buzzer_note_t s_sound_error[] = {
    { NOTE_F3,      120 },
    { NOTE_SILENCE,  30 },
    { NOTE_D3,      250 }
};

static const buzzer_note_t s_sound_beep_short[] = {
    { 2500, 50 }
};

static const buzzer_note_t s_sound_match_start[] = {
    { 3000, 400 }
};

bool BUZZER_Init(buzzer_t *dev, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (!dev || !htim) {
        return false;
    }

    memset(dev, 0, sizeof(buzzer_t));
    dev->htim = htim;
    dev->channel = channel;

    /* Ensure output starts in silence */
    __HAL_TIM_SET_COMPARE(dev->htim, dev->channel, 0);
    HAL_TIM_PWM_Start(dev->htim, dev->channel);

    dev->is_initialized = true;
    dev->is_playing = false;

    return true;
}

void BUZZER_SetTone(buzzer_t *dev, uint16_t freq_hz)
{
    if (!dev || !dev->is_initialized) {
        return;
    }

    if (freq_hz == 0 || freq_hz < 20) {
        /* Silence: 0% duty cycle */
        __HAL_TIM_SET_COMPARE(dev->htim, dev->channel, 0);
    } else {
        uint32_t period = BUZZER_TIMER_CLOCK_HZ / freq_hz;
        if (period < 2) period = 2;
        if (period > 65535) period = 65535;

        __HAL_TIM_SET_AUTORELOAD(dev->htim, period - 1);
        __HAL_TIM_SET_COMPARE(dev->htim, dev->channel, period / 2);
    }
}

void BUZZER_Stop(buzzer_t *dev)
{
    if (!dev || !dev->is_initialized) {
        return;
    }

    BUZZER_SetTone(dev, 0);
    dev->is_playing = false;
    dev->current_melody = NULL;
    dev->melody_length = 0;
    dev->current_note_idx = 0;
}

void BUZZER_PlayMelody(buzzer_t *dev, const buzzer_note_t *notes, uint16_t length)
{
    if (!dev || !dev->is_initialized || !notes || length == 0) {
        return;
    }

    dev->current_melody = notes;
    dev->melody_length = length;
    dev->current_note_idx = 0;
    dev->is_playing = true;

    /* Play first note immediately */
    BUZZER_SetTone(dev, notes[0].freq_hz);
    dev->note_start_tick = HAL_GetTick();
    dev->note_duration_ms = notes[0].duration_ms;
}

void BUZZER_PlaySound(buzzer_t *dev, bmo_sound_t sound)
{
    switch (sound) {
    case BMO_SOUND_BOOT:
        BUZZER_PlayMelody(dev, s_sound_boot, sizeof(s_sound_boot) / sizeof(s_sound_boot[0]));
        break;
    case BMO_SOUND_CAN_FOUND:
        BUZZER_PlayMelody(dev, s_sound_can_found, sizeof(s_sound_can_found) / sizeof(s_sound_can_found[0]));
        break;
    case BMO_SOUND_GRIP_SUCCESS:
        BUZZER_PlayMelody(dev, s_sound_grip_success, sizeof(s_sound_grip_success) / sizeof(s_sound_grip_success[0]));
        break;
    case BMO_SOUND_CAN_DROPPED:
        BUZZER_PlayMelody(dev, s_sound_can_dropped, sizeof(s_sound_can_dropped) / sizeof(s_sound_can_dropped[0]));
        break;
    case BMO_SOUND_ERROR:
        BUZZER_PlayMelody(dev, s_sound_error, sizeof(s_sound_error) / sizeof(s_sound_error[0]));
        break;
    case BMO_SOUND_BEEP_SHORT:
        BUZZER_PlayMelody(dev, s_sound_beep_short, sizeof(s_sound_beep_short) / sizeof(s_sound_beep_short[0]));
        break;
    case BMO_SOUND_MATCH_START:
        BUZZER_PlayMelody(dev, s_sound_match_start, sizeof(s_sound_match_start) / sizeof(s_sound_match_start[0]));
        break;
    case BMO_SOUND_NONE:
    default:
        BUZZER_Stop(dev);
        break;
    }
}

void BUZZER_Process(buzzer_t *dev, uint32_t now_ms)
{
    if (!dev || !dev->is_initialized || !dev->is_playing || !dev->current_melody) {
        return;
    }

    if (now_ms - dev->note_start_tick >= dev->note_duration_ms) {
        dev->current_note_idx++;

        if (dev->current_note_idx < dev->melody_length) {
            /* Next note */
            BUZZER_SetTone(dev, dev->current_melody[dev->current_note_idx].freq_hz);
            dev->note_start_tick = now_ms;
            dev->note_duration_ms = dev->current_melody[dev->current_note_idx].duration_ms;
        } else {
            /* Melody ended */
            BUZZER_Stop(dev);
        }
    }
}
