/*
 * test_comp_filter.c — Host-side unit tests for core/comp_filter.c.
 *
 * Build & run from the repo root:
 *   gcc -Wall -Wextra -std=c11 -Icore core/comp_filter.c tests/test_comp_filter.c -o tests/build/test_comp_filter -lm
 *   ./tests/build/test_comp_filter
 */

#include <stdio.h>
#include <math.h>
#include "comp_filter.h"

static int failures = 0;

//Like the PID tests, but hte tolrange is a parametr: integrating over
//1000 steps accumulates tiny float rounding, so some tests need a looser margin
static void check(const char *name, float got, float expected, float tol)
{
    if (fabsf(got - expected) > tol){
        printf("FAIL %-30s got %f, expected %f\n", name, got, expected);
        failures++;
    } else {
        printf("pass %s\n", name);
    }
}

int main(void)
{
    comp_filter_t f;
    const float dt = 0.001f;          /* 1 kHz, the flight controller's loop rate.  */
    const float GYRO_ONLY = 1.0e6f;   /* Huge tau -> alpha ~= 1 -> accel ignored.
                                         Isolates the gyro path for testing.       */
    float out = 0.0f;

    /* 1. Accelerometer angle math: equal gravity on both axes = 45 deg; level = 0. */
    check("accel angle 45 deg", cf_accel_angle_deg(1.0f, 1.0f), 45.0f, 1e-3f);
    check("accel angle level",  cf_accel_angle_deg(0.0f, 1.0f),  0.0f, 1e-3f);

    /* 2. First update seeds the angle from the accelerometer (no slow start). */
    cf_init(&f, 0.47f);
    check("first update seeds angle", cf_update(&f, 0.0f, 30.0f, dt), 30.0f, 1e-4f);

    /* 3. Gyro integration: 10 deg/s for 1000 steps of 1 ms = 1 s -> 10 deg. */
    cf_init(&f, GYRO_ONLY);
    cf_update(&f, 0.0f, 0.0f, dt);                 /* seed at 0 */
    for (int i = 0; i < 1000; i++) {
        out = cf_update(&f, 10.0f, 0.0f, dt);
    }
    check("gyro integrates to angle", out, 10.0f, 1e-2f);

    /* 4. Bias removal: a still gyro reading 2 deg/s with bias calibrated
          to 2 must NOT drift (without calibration: 2 deg after 1 s). */
    cf_init(&f, GYRO_ONLY);
    float still[100];
    for (int i = 0; i < 100; i++) still[i] = 2.0f;
    cf_calibrate_bias(&f, still, 100);
    cf_update(&f, 0.0f, 0.0f, dt);                 /* seed at 0 */
    for (int i = 0; i < 1000; i++) {
        out = cf_update(&f, 2.0f, 0.0f, dt);
    }
    check("bias removed, no drift", out, 0.0f, 1e-3f);

    /* 5. Drift correction: estimate starts at 0, sensor is truly tilted 20 deg
          (accel says 20, gyro 0). After 5 time constants (~2.35 s) the
          estimate should be within ~1% of 20. */
    cf_init(&f, 0.47f);
    cf_update(&f, 0.0f, 0.0f, dt);                 /* seed at 0 */
    for (int i = 0; i < 2350; i++) {
        out = cf_update(&f, 0.0f, 20.0f, dt);
    }
    check("converges to sustained tilt", out, 20.0f, 0.2f);

    /* 6. Vibration rejection: a single 1 ms accel spike to 90 deg (a jolt)
          moves the estimate by only (1 - alpha) * 90 = dt/(tau+dt) * 90 ~= 0.19 deg. */
    cf_init(&f, 0.47f);
    cf_update(&f, 0.0f, 0.0f, dt);                 /* seed at 0 */
    out = cf_update(&f, 0.0f, 90.0f, dt);
    check("single spike barely moves angle", out, 0.191f, 1e-3f);

    /* 7. Bad dt keeps the current estimate unchanged. */
    check("dt <= 0 keeps angle", cf_update(&f, 50.0f, 50.0f, 0.0f), out, 1e-6f);

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}