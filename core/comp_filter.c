/*
 * comp_filter.c — Implementation of the complementary filter in comp_filter.h.
 */

 #include <math.h>      //atan2f
 #include "comp_filter.h"  //Our own header: struct and func prototypes

 #define RAD_TO_DEG 57.29578f  /*pi: atan2f returns radians, we want degrees. */

 void cf_init(comp_filter_t *f, float tau_s)
 {
    f->tau_s         = tau_s;
    f->gyro_bias_dps = 0.0f;      /* No bias known until calibration runs.            */
    f->angle_deg     = 0.0f;
    f->initialized   = false;     /* First update will seed the angle from the accel.*/
 }

 void cf_calibrate_bias(comp_filter_t *f, const float *gyro_samples_dps, int n)
 {
    if (n<=0) return;

    float sum = 0.0f;
    for (int i = 0; i<n;i++) {
        sum += gyro_samples_dps[i]; //While perfectly still, any reading is pure bias.

    }
    f->gyro_bias_dps = sum / (float)n;  //Average the samples to get the bias
                                        //leaving only the pure bias
 }

 float cf_accel_angle_deg(float a_side, float a_down)
 {
    //Gravity splits between the 2 axes as the asaner tilts:
    //level->all on a_down ; tilted 45deg -> equal on both.
    //atan of the two gives the tilt angle, and handles all four
    //quadrants correctly (plain atan(a/b) cant tell 45 deg from -135deg).
    //Units cancle, so a_side and a_down can be in g or m/s^2.
    return atan2f(a_side, a_down) * RAD_TO_DEG;  //atan2f returns radians, we want degrees.
 }

 float cf_update(comp_filter_t *f, float gyro_dps, float accel_angle_deg, float dt)
{
    //Bad time step: keep the current estimate rathenr than corrupt it
    if (dt <= 0.0f) return f->angle_deg;

    //First update: there is no previous angle to integrate from, so start
    //from the accelerometer's angle. Otherwise the estimate would begin at 0
    //and take several time constants to converge after boot
    if (!f->initialized) {
        f->angle_deg = accel_angle_deg;  //Seed the filter with the accel angle
        f->initialized = true;            //Next update will integrate from this angle
        return f->angle_deg;
    }

    //Gyro path: remove the bies, then integreate rate x time into angle;
    //Accurate over short timescales; drifts over long ones
    float rate_dps = gyro_dps - f->gyro_bias_dps;  //Remove the bias from the raw gyro reading
    float gyro_angle = f->angle_deg + rate_dps *dt;  //Integrate the gyro rate into an angle

    //Blend weight form the time constant; alpha = tau/(tau + dt)
    //Computer every update, so the filter behaves the same if the loop
    //rate chanfes or a cycle runs late. (alpha adpats to the real dt)
    //At tau = 0.47 s and dt = 1ms, alpha ~=0.998
    float alpha = f->tau_s / (f-> tau_s + dt);

    //Complementary blend: mostly the gyro (smooth, responsive but drifts),
    //plus a small pull toward the accelerometer each step (no drift)
    f->angle_deg = alpha * gyro_angle + (1.0f -alpha) * accel_angle_deg;

    return f->angle_deg;
}