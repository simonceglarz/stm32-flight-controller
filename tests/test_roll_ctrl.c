#include <math.h>
#include <stdio.h>
#include "roll_ctrl.h"

static int fails = 0;

static void check(const char *name, float got, float exp, float tol)
{
    int ok = fabsf(got - exp) <= tol;
    printf("%s: %s: got %f, expected %f\n", ok ? "PASS" : "FAIL", name, (double)got, (double)exp);
    if (!ok) fails++;
}

int main(void)
{
    roll_ctrl_t c;
    /* P-only gains so outputs are hand-computable */
    roll_gains_t g = { .kp_angle = 10.0f, .rate_limit = 500.0f, .kp_rate = 0.005f,
                       .ki_rate = 0.0f, .kd_rate = 0.0f, .d_alpha = 0.0f, .integ_limit = 0.3f };

    /* 1. At setpoint and not moving -> zero command */
    roll_ctrl_init(&c, &g);
    check("level, still", roll_ctrl_step(&c, 0.0f, 0.0f, 0.0f, 0.001f), 0.0f, 1e-6f);

    /* 2. 2 deg error, not moving: rate_sp = 10*2 = 20 dps -> cmd = 0.005*20 = 0.1 */
    roll_ctrl_init(&c, &g);
    check("2 deg error", roll_ctrl_step(&c, 2.0f, 0.0f, 0.0f, 0.001f), 0.1f, 1e-6f);

    /* 3. Already rotating at the requested rate -> zero command (inner loop satisfied) */
    roll_ctrl_init(&c, &g);
    check("rate matched", roll_ctrl_step(&c, 2.0f, 0.0f, 20.0f, 0.001f), 0.0f, 1e-6f);

    /* 4. Huge error saturates at +1 */
    roll_ctrl_init(&c, &g);
    check("saturates", roll_ctrl_step(&c, 90.0f, 0.0f, 0.0f, 0.001f), 1.0f, 1e-6f);

    /* 5. Sign: tilted past setpoint -> negative command */
    roll_ctrl_init(&c, &g);
    check("sign", roll_ctrl_step(&c, 0.0f, 2.0f, 0.0f, 0.001f), -0.1f, 1e-6f);

    return fails ? 1 : 0;
}