/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   Passive Buzzer driver with musical notes & BMO sound effects
  ******************************************************************************
  */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Musical Note Frequencies (Hz) */
#define NOTE_SILENCE 0
#define NOTE_C3      131
#define NOTE_D3      147
#define NOTE_E3      165
#define NOTE_F3      175
#define NOTE_G3      196
#define NOTE_A3      220
#define NOTE_B3      247
#define NOTE_C4      262
#define NOTE_D4      294
#define NOTE_E4      330
#define NOTE_F4      349
#define NOTE_G4      392
#define NOTE_A4      440
#define NOTE_B4      494
#define NOTE_C5      523
#define NOTE_D5      587
#define NOTE_E5      659
#define NOTE_F5      698
#define NOTE_G5      784
#define NOTE_A5      880
#define NOTE_B5      988
#define NOTE_C6      1047
#define NOTE_D6      1175
#define NOTE_E6      1319
#define NOTE_F6      1397
#define NOTE_G6      1568
#define NOTE_A6      1760
#define NOTE_B6      1976
#define NOTE_C7      2093

/**
  * @brief Predefined BMO robot sound effects
  */
typedef enum {
    BMO_SOUND_NONE = 0,
    BMO_SOUND_BOOT,          /* Ascending power-on arpeggio (C5-E5-G5-C6) */
    BMO_SOUND_CAN_FOUND,     /* Positive detection double-beep (G5-C6) */
    BMO_SOUND_GRIP_SUCCESS,  /* Victory mini-fanfare (C5-G5-C6-E6) */
    BMO_SOUND_CAN_DROPPED,   /* Mission complete chime (C6-G5) */
    BMO_SOUND_ERROR,         /* Descending error buzz (F3-D3) */
    BMO_SOUND_BEEP_SHORT,    /* Crisp UI click (2500 Hz, 50 ms) */
    BMO_SOUND_MATCH_START    /* Long referee whistle (3000 Hz, 400 ms) */
} bmo_sound_t;

/**
  * @brief Musical note structure
  */
typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
} buzzer_note_t;

/**
  * @brief Buzzer device handle
  */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    bool is_initialized;
    bool is_playing;

    /* Non-blocking melody playback state */
    const buzzer_note_t *current_melody;
    uint16_t melody_length;
    uint16_t current_note_idx;
    uint32_t note_start_tick;
    uint32_t note_duration_ms;
} buzzer_t;

/**
  * @brief  Initialize buzzer driver with specified timer and PWM channel.
  * @param  dev: Pointer to buzzer handle.
  * @param  htim: Timer handle (e.g. &htim17).
  * @param  channel: PWM channel (e.g. TIM_CHANNEL_1).
  * @retval true if initialized successfully.
  */
bool BUZZER_Init(buzzer_t *dev, TIM_HandleTypeDef *htim, uint32_t channel);

/**
  * @brief  Set raw tone frequency (50% duty cycle square wave).
  * @param  dev: Pointer to buzzer handle.
  * @param  freq_hz: Frequency in Hz (0 for silence).
  */
void BUZZER_SetTone(buzzer_t *dev, uint16_t freq_hz);

/**
  * @brief  Silence buzzer and cancel any playing melody immediately.
  * @param  dev: Pointer to buzzer handle.
  */
void BUZZER_Stop(buzzer_t *dev);

/**
  * @brief  Start playing a predefined BMO sound effect non-blockingly.
  * @param  dev: Pointer to buzzer handle.
  * @param  sound: Sound effect identifier.
  */
void BUZZER_PlaySound(buzzer_t *dev, bmo_sound_t sound);

/**
  * @brief  Start playing a custom array of musical notes non-blockingly.
  * @param  dev: Pointer to buzzer handle.
  * @param  notes: Pointer to array of buzzer_note_t.
  * @param  length: Number of notes in melody.
  */
void BUZZER_PlayMelody(buzzer_t *dev, const buzzer_note_t *notes, uint16_t length);

/**
  * @brief  Periodic update function to advance melody notes non-blockingly.
  *         Call regularly from while(1) or BSP_Update().
  * @param  dev: Pointer to buzzer handle.
  * @param  now_ms: Current system tick (HAL_GetTick()).
  */
void BUZZER_Process(buzzer_t *dev, uint32_t now_ms);

#endif /* BSP_BUZZER_H */
