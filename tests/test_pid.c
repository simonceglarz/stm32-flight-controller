/*
 * test_pid.c — Host-side unit tests for core/pid.c.
 * Runs on the PC, not the STM32. Each test isolates ONE behaviour by
 * setting the other gains to zero, so a failure points to exactly one term.
 *
 * Build & run from the repo root:
 *   gcc -Wall -Wextra -std=c11 -Icore core/pid.c tests/test_pid.c -o tests/build/test_pid -lm
 *   ./tests/build/test_pid
 */

#include <stdio.h>      //For printf() in the unit test   
#include<math.h>        //fabsf(abs value of float)
#include "pid.h"        //Our own header: struct and func prototypes

static int failures = 0;  //Count of failed tests, decidees to exit code

//Compare a result against an expected value.
//Floats are compared with a small tolerance because they are never exact.
static void check(const char*name, float got, float expected)
{
    float tol = 0.0001f;  //Allowable error
    if (fabsf(got - expected) > tol) {
        printf("FAIL: %s: got %f, expected %f\n", name, got, expected);
        failures++;
    } else {
        printf("PASS: %s: got %f, expected %f\n", name, got, expected);
    }
}

int main(void)
{
    pid_ctrl_t pid; //One controller, re-initialized for each test
    float out = 0.0f;  //Output from the controller

    //1. P term alone
    //error = 10 - 4 = 6; output = kp*error = 2*6=12
    pid_init(&pid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f); //kp=2, ki=0, kd=0
    check("P term", pid_update(&pid, 10.0f, 4.0f, 0.001f), 12.0f);

    //2. Output clamp.
    //error = 10, kp = 1000 -> raw output 10000, but the limit is 100.
    pid_init(&pid, 1000.0f, 0.0f, 0.0f, -100.0f, 100.0f); //kp=1000, ki=0, kd=0
    check("Output clamp", pid_update(&pid, 10.0f, 0.0f, 0.001f), 100.0f);

    //3. Integral accumulation and anti-windup. The integrator should accumulate error over time, but not exceed the limits.
    pid_init(&pid, 0.0f, 1.0f, 0.0f, -100.0f, 100.0f); //kp=0, ki=1, kd=0
    for (int i = 0; i < 10; i++ ) {
        out =pid_update(&pid, 1.0f, 0.0f, 0.1f); //error = 10, dt = 0.1, so integrator += 1 each step
    }
    check("Integral accumulation", out, 1.0f); //After 10 steps integal should be 1.0 (1.0 * 10 * 0.1)
     /* 4. Anti-windup.
       1000 steps would reach 100, but the integrator limit is 5. */
    pid_init(&pid, 0.0f, 1.0f, 0.0f, -5.0f, 5.0f);
    for (int i = 0; i < 1000; i++) {
        out = pid_update(&pid, 1.0f, 0.0f, 0.1f);
    }
    check("integrator clamp", out, 5.0f);

    /* 5. No derivative kick.
       The setpoint jumps 0 -> 50 but the measurement stays at 5.
       D uses the measurement only, so it must output 0. */
    pid_init(&pid, 0.0f, 0.0f, 1.0f, -100.0f, 100.0f);
    pid_update(&pid, 0.0f, 5.0f, 0.01f);          /* first call: sets prev = 5 */
    check("no derivative kick", pid_update(&pid, 50.0f, 5.0f, 0.01f), 0.0f);

    /* 6. D brakes a rising measurement.
       Measurement 0 -> 0.1 in 0.01 s = rate +10/s. D output = -kd * 10 = -10. */
    pid_init(&pid, 0.0f, 0.0f, 1.0f, -100.0f, 100.0f);
    pid_update(&pid, 0.0f, 0.0f, 0.01f);          /* first call: sets prev = 0 */
    check("derivative sign", pid_update(&pid, 0.0f, 0.1f, 0.01f), -10.0f);

    /* 7. A bad time step is harmless: dt = 0 must return 0. */
    check("dt <= 0 returns 0", pid_update(&pid, 10.0f, 0.0f, 0.0f), 0.0f);

    /* Summary. A non-zero exit code lets automated checks (GitHub CI) detect
       failures without reading the text. */
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}