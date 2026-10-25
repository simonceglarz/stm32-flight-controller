#ifndef ROLL_CTRL_H
#define ROLL_CTRL_H

#include "pid.h"

/* Gains for the cascaded roll controller (values from SIL tuning, sim/roll_sim.py). */
typedef struct {
    float kp_angle;      /* (deg/s) per deg: angle error -> rate setpoint */
    float rate_limit;    /* deg/s, max rate the outer loop may request */
    float kp_rate;       /* motor cmd per (deg/s) */
    float ki_rate;       /* motor cmd per (deg/s * s) */
    float kd_rate;       /* motor cmd per (deg/s^2) */
    float d_alpha;       /* D-term low-pass, 0..1 (higher = smoother) */
    float integ_limit;   /* |rate integrator| max, motor-cmd units */
} roll_gains_t;

typedef struct {
    pid_ctrl_t angle;    /* outer loop */
    pid_ctrl_t rate;     /* inner loop */
} roll_ctrl_t;

/* Default gains from SIL: 10 / 0.005 / 0.1 / 0.0001, alpha 0.95 */
extern const roll_gains_t ROLL_GAINS_DEFAULT;

void  roll_ctrl_init(roll_ctrl_t *c, const roll_gains_t *g);
void  roll_ctrl_reset(roll_ctrl_t *c);

/* One control step. Returns the differential motor command, clamped to -1..1. */
float roll_ctrl_step(roll_ctrl_t *c, float angle_sp_deg,
                     float angle_meas_deg, float rate_meas_dps, float dt);

#endif