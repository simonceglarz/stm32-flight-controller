#include "roll_ctrl.h"

const roll_gains_t ROLL_GAINS_DEFAULT = {
    .kp_angle    = 10.0f,
    .rate_limit  = 500.0f,
    .kp_rate     = 0.005f,
    .ki_rate     = 0.1f,
    .kd_rate     = 0.0001f,
    .d_alpha     = 0.95f,
    .integ_limit = 0.3f,
};

void roll_ctrl_init(roll_ctrl_t *c, const roll_gains_t *g)
{
    /* Outer: P only. Output is a rate setpoint, limited to +/-rate_limit. */
    pid_init(&c->angle, g->kp_angle, 0.0f, 0.0f, -g->rate_limit, g->rate_limit);

    /* Inner: full PID. Output is the motor command, -1..1. */
    pid_init(&c->rate, g->kp_rate, g->ki_rate, g->kd_rate, -1.0f, 1.0f);
    c->rate.integ_min = -g->integ_limit;
    c->rate.integ_max =  g->integ_limit;
    c->rate.d_filter_alpha = g->d_alpha;
}

void roll_ctrl_reset(roll_ctrl_t *c)
{
    pid_reset(&c->angle);
    pid_reset(&c->rate);
}

float roll_ctrl_step(roll_ctrl_t *c, float angle_sp_deg,
                     float angle_meas_deg, float rate_meas_dps, float dt)
{
    float rate_sp = pid_update(&c->angle, angle_sp_deg, angle_meas_deg, dt);
    return pid_update(&c->rate, rate_sp, rate_meas_dps, dt);
}