/* Thin C wrapper so Python (ctypes) can use core/pid.c without knowing
   the layout of pid_ctrl_t. Python only ever holds an opaque pointer. */
#include <stdlib.h>
#include "pid.h"

/* Create a controller on the heap; returns NULL if out of memory. */
pid_ctrl_t *pid_shim_new(float kp, float ki, float kd,
                         float out_min, float out_max,
                         float integ_min, float integ_max,
                         float d_filter_alpha)
{
    pid_ctrl_t *p = malloc(sizeof *p);
    if (p == NULL) return NULL;
    pid_init(p, kp, ki, kd, out_min, out_max);
    /* Set the fields pid_init doesn't take as arguments */
    p->integ_min = integ_min;
    p->integ_max = integ_max;
    p->d_filter_alpha = d_filter_alpha;
    return p;
}

float pid_shim_update(pid_ctrl_t *p, float setpoint, float measurement, float dt)
{
    return pid_update(p, setpoint, measurement, dt);
}

void pid_shim_reset(pid_ctrl_t *p) { pid_reset(p); }

void pid_shim_free(pid_ctrl_t *p) { free(p); }

