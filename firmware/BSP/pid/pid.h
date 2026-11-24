/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Correcteur PI(D) numerique avec saturation, anti-windup
  *          et anticipation (feedforward) optionnelle.
  *          Utilise pour l'asservissement en vitesse des moteurs BMO.
  ******************************************************************************
  */
#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp;           /* gain proportionnel                       */
    float ki;           /* gain integral   [1/s]                    */
    float kd;           /* gain derive     [s]                      */
    float kff;          /* anticipation : commande par unite de consigne (0 = pas d'anticipation) */
    float ff_offset;    /* anticipation : commande de demarrage (frottements), 0 = rien */
    float out_min;      /* commande min (ex. -1000 pour mille)      */
    float out_max;      /* commande max (ex. +1000 pour mille)      */
    float d_filter;     /* filtre de la derivee, 0..1 (0 = pas de filtre) */

    float integral;     /* somme des erreurs (etat interne)         */
    float prev_meas;    /* mesure precedente (pour la derivee)      */
    float d_state;      /* derivee filtree                          */
    bool  first_run;
} pid_ctrl_t;

void  PID_Init(pid_ctrl_t *pid, float kp, float ki, float kd, float out_min, float out_max);
/* Anticipation : commande = kff * consigne + ff_offset * signe(consigne) + correction PID */
void  PID_SetFeedforward(pid_ctrl_t *pid, float kff, float ff_offset);
void  PID_Reset(pid_ctrl_t *pid);
/* Depart sans a-coup : remet a zero, puis l'integrale reprend la commande deja appliquee */
void  PID_Preset(pid_ctrl_t *pid, float current_output, float setpoint);
/* setpoint et measure dans la meme unite (ex. mm/s), dt en secondes */
float PID_Update(pid_ctrl_t *pid, float setpoint, float measure, float dt);

#endif /* PID_H */
