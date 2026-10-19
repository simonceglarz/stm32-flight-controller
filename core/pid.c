// pid.c - Implementation of the portabl PID controller declared in pid.h

#include "pid.h"    //Our own header: struct and func prototypes

//Limit x ot the range [lo, hi]
//'static' = private to this file: nothing outside pid.c can call it.

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;  //If below the floor, return the floor
    if (x> hi)  return hi;  //If abov the ceiling, return the ceiling
    return x;               
}

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;   //Store the gains
    pid->ki = ki;
    pid->kd = kd; 

    pid->out_min = out_min; //Store actuator limits
    pid->out_max = out_max; 

    //Default: the integrator alone may never demand more than the actuator can 
    //deliver. Can be tighetened if the I term needs less authoerity
    pid->integ_min = out_min;
    pid->integ_max = out_max;

    pid->d_filter_alpha = 0.0f; //Star with no F filtering, add it when the real sensor 
                                //nise shows it is needed

    pid_reset(pid);  //Start from a clean slate  
}

void pid_reset(pid_ctrl_t *pid)
{
    pid->integrator         =0.0f;  //Forget the accumulated error.
    pid->prev_measurement   =0.0f;  //No valid previous mrasurment anymor
    pid->d_filtered         =0.0f;  //Clear the derivative filter's memeory
    pid->first_update       =true;  //Next update must the derivative
                                    //Because prev_measurement isnt real yet
}

float pid_update (pid_ctrl_t *pid, float setpoint, float measurement, float dt)
{
    //A bard time step must never produce a big command
    //dt <= would make the integral wrong and divide by zero in the derivative.
    if (dt <= 0.0f) return 0.0f;
    
    //Compute the error between where we want to be and where we are
    float error = setpoint - measurement;

    //Proportional term: how much to react to the current error
    float p_term = pid->kp * error;

    //Integral term: how much to react to the accumulated error over time
    pid->integrator += pid->ki * error * dt;  //Add this step's contribution to the running sum
    pid->integrator = clampf(pid->integrator, pid->integ_min, pid->integ_max); //Anti-windup

    float i_term = pid->integrator;  //The I term is just the integrator itself

    //Derivative term: how much to react to how fast things are changing
    float d_raw;
    if (pid->first_update) {
        d_raw = 0.0f;  //No previous measurement yet, so no derivative
        pid->first_update = false;  //Next time we will have a previous measurement
    } else {
        d_raw = (measurement - pid->prev_measurement) / dt;  //Rate of change of measurement
    }
    
    //Apply a low-pass filter to the derivative term if needed
    pid->d_filtered = pid->d_filter_alpha * pid->d_filtered + (1.0f - pid->d_filter_alpha) * d_raw;
    
    float d_term = pid->kd * (-pid->d_filtered);  //Negative because we want to counteract the change

    //Remember this measurement for the next update
    pid->prev_measurement = measurement;

    //Combine all three terms and clamp the output to actuator limits
    float output = p_term + i_term + d_term;
    output = clampf(output, pid->out_min, pid->out_max);

    return output;
}
