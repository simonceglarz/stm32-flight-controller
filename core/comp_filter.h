/*
 * comp_filter.h — Portable single-axis complementary filter.
 *
 * Fuses two imperfect angle sources:
 *   - gyroscope: rotation RATE; integrating it gives a smooth, fast angle
 *                that slowly DRIFTS (small errors accumulate forever).
 *   - accelerometer: angle from the direction of gravity; never drifts,
 *                but NOISY and fooled by vibration and acceleration.
 * The filter trusts the gyro over short timescales and the accelerometer
 * over long ones. One instance per axis (roll, pitch).
 */

#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include <stdbool.h>    //Gives us the bool type

typedef struct {
    //Configuration
        float tau_s;        //Time constant in seconds: how long th efilter takes
                            //to pull a drifting gyro angle back to the accel angle.
                            //Bigger = trusts the gyro longer (smoother, slower, drift correction
                            //Independent of loop rate
    float gyro_bias_dps;    //Gyro reading when perfectly still (deg/s) Every gyro has a small offset
                            //It's subtracted each update so it doesnt integreate into a 
                            //growing angle error
    //State
    float angle_deg;        //The filter's current best estimate of the angle (degrees)  
    bool initialized;       //True after the first update, false before. The first update
                            //uses the accel angle to initialize the filter's state.  

}comp_filter_t;

//Set the time constant, zero the bias, clear the state
void cf_init(comp_filter_t *cf, float tau_s);

//Average n gyri samples taken while stationar and store the result as the bias
//Call at boot the wthe vehicel still.
void cf_calibrate_bias(comp_filter_t *cf, const float *gyro_samples_dps, int n);

//Tilt angle (degrees from two accelerometer axes) and gyro rate (deg/s) are passed in each update.
//a_side: axis that picks up gravity as you tilt (e.g. ay for roll)
//a_down axis that sees full gravity when level (usually az
float cf_accel_angle_deg(float a_side, float a_down);

//Run one filter step. Returns the new angle estimate (degrees)
//gyrio_dps     =raw gyro rate for this axis (deg/s)
//accel_angle_deg = angle from cf_accel_angle_deg()
//dt        = seconds since the previous call (0.001 at 1kHz)
float cf_update(comp_filter_t *f, float gyro_dps, float accel_angle_deg, float dt);

#endif // COMP_FILTER_H