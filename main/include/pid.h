#ifndef PID_H
#define PID_H

#include <math.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float tau; // derivative low-pass filter time constant
    float i;   // integrator state
    float d;   // derivative filter state
    float e_prev;
    float T;   // sample period (seconds)
    float min;
    float max;
} pid_state_t;

static inline void pid_reset(pid_state_t *pid) {
    pid->i = 0.0f;
    pid->d = 0.0f;
    pid->e_prev = 0.0f;
}

static inline float pid_update(pid_state_t *pid, float setpoint, float measurement) {
    float e = setpoint - measurement;

    // Proportional
    float p = pid->kp * e;

    // Dynamic integral limits for anti-windup
    float i_max = pid->max - p;
    if (i_max < 0.0f)
        i_max = 0.0f;
    float i_min = pid->min - p;
    if (i_min > 0.0f)
        i_min = 0.0f;

    // Trapezoidal integration
    pid->i += pid->ki * pid->T * (e + pid->e_prev) / 2.0f;
    if (pid->i > i_max)
        pid->i = i_max;
    if (pid->i < i_min)
        pid->i = i_min;

    // Tamed derivative with low-pass filter
    if (pid->tau > 0.0f) {
        pid->d = pid->kd * 2.0f / (2.0f * pid->tau + pid->T) * (e - pid->e_prev) + (2.0f * pid->tau - pid->T) / (2.0f * pid->tau + pid->T) * pid->d;
    }

    pid->e_prev = e;

    float output = p + pid->i + pid->d;
    return fmaxf(pid->min, fminf(pid->max, output));
}

#endif
