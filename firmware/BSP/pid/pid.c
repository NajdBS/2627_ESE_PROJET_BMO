/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Correcteur PI(D) numerique avec saturation, anti-windup
  *          et anticipation (feedforward) optionnelle
  ******************************************************************************
  * Principe (appele a periode fixe dt) :
  *   erreur   = consigne - mesure
  *   FF       = kff * consigne + ff_offset * signe(consigne)   (anticipation, 0 si non reglee)
  *   P        = kp * erreur
  *   I       += ki * erreur * dt        (bloque si la commande sature : anti-windup)
  *   D        = -kd * d(mesure)/dt      (derivee sur la mesure : pas de pic quand la consigne change)
  *   commande = FF + P + I + D, limitee entre out_min et out_max
  */
#include "pid.h"
#include <stddef.h>

static float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

/* Commande "a vide" estimee pour la consigne : la boucle ne corrige que l'ecart restant */
static float feedforward(const pid_ctrl_t *pid, float setpoint) {
    float ff = pid->kff * setpoint;
    if (setpoint > 0.0f) {
        ff += pid->ff_offset;
    } else if (setpoint < 0.0f) {
        ff -= pid->ff_offset;
    }
    return ff;
}

void PID_Init(pid_ctrl_t *pid, float kp, float ki, float kd, float out_min, float out_max) {
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = 0.0f;
    pid->ff_offset = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->d_filter = 0.7f;
    PID_Reset(pid);
}

void PID_SetFeedforward(pid_ctrl_t *pid, float kff, float ff_offset) {
    if (pid == NULL) {
        return;
    }
    pid->kff = kff;
    pid->ff_offset = ff_offset;
}

void PID_Reset(pid_ctrl_t *pid) {
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_state = 0.0f;
    pid->first_run = true;
}

void PID_Preset(pid_ctrl_t *pid, float current_output, float setpoint) {
    if (pid == NULL) {
        return;
    }
    PID_Reset(pid);
    /* Si la consigne vaut la mesure (erreur nulle), la premiere commande vaut current_output */
    pid->integral = clampf(current_output - feedforward(pid, setpoint), pid->out_min, pid->out_max);
}

float PID_Update(pid_ctrl_t *pid, float setpoint, float measure, float dt) {
    if (pid == NULL || !(dt > 0.0f)) {
        return 0.0f;
    }

    float error = setpoint - measure;

    /* Anticipation (0 tant que kff et ff_offset ne sont pas regles) */
    float ff = feedforward(pid, setpoint);

    /* Terme proportionnel */
    float p = pid->kp * error;

    /* Terme derive, calcule sur la mesure et filtre (le bruit des encodeurs est amplifie par D) */
    float d = 0.0f;
    if (pid->kd != 0.0f && !pid->first_run) {
        float raw_d = -(measure - pid->prev_meas) / dt;
        pid->d_state = pid->d_filter * pid->d_state + (1.0f - pid->d_filter) * raw_d;
        d = pid->kd * pid->d_state;
    }
    pid->prev_meas = measure;
    pid->first_run = false;

    /* Terme integral candidat */
    float new_integral = pid->integral + pid->ki * error * dt;
    float out = ff + p + new_integral + d;

    /* Anti-windup par integration conditionnelle :
     * si la commande sature ET que l'erreur pousse encore dans le meme sens,
     * on n'accumule pas l'integrale. */
    if ((out > pid->out_max && error > 0.0f) || (out < pid->out_min && error < 0.0f)) {
        out = ff + p + pid->integral + d;    /* integrale gelee */
    } else {
        pid->integral = clampf(new_integral, pid->out_min, pid->out_max);
    }

    return clampf(out, pid->out_min, pid->out_max);
}
