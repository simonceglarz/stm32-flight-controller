/* pid.h - Portable PID controller
*
* Plain C so the same cod ecompiles on a PC (unit tests, simulation)
* and on the STM32 (flight firmware). Time step dt is always passed
* in by the caller, never read from a hardare timer, for the saem reason
*/

#ifndef PID_H   //include safeguard is header was already included
#define PID_H
#include <stdbool.h>    //Gives us the bool type

/* One controller's complete configuaration and memory 
* You creat one of these per control loop(e.g one for roll angle)
* one for roll rare), so every loop has its own independent state.
*/

typedef struct {

    // Gains: how strongly each term reacts
    float kp;       //Proportional gain: reacts to the error NOW
    float ki;       //Integral gain: reacts to error ACCUMULATED over time.
    float kd;       //Derivative gain: react to how fast things change.

    //Limits: what the actuator can actually do 
    float out_min;      //Lowest command output may ever produce
    float out_max;      //Highest command  output may ever produce
    float integ_min;    //Lowest value the integrator may hold (anti-windup)
    float integ_max;    //Highest value integrator may hold

    //Tuning
    float d_filter_alpha;   //0-1 smoothing for the D term. 0 = no filtering
                            //closer to 1 = smoother but slower to react
    
    
    //State what the controller remembers between calls
    float integrator;       //Running sum of ki*error*dt(The I term itself)
    float prev_measurement; //last call's measurement, needed to compute rate
    float d_filtered;       //last call's filtered derivatine (the filter's memory)
    bool first_update;      //True until the first update; no previous measurement
                            //No previous measurement exists yet
} pid_ctrl_t;       //Named pid_Ctrl_t because Linux already defines pid_t(process IF),
                    //Which could clash in PC builds

//Set gains and output limits, then clear all state. Call once at startup.
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float out_min, float out_max);

//Clear only the state (keeps gains/limits). Call on every disarm or fault so a stale integrator
//never carries into the next run.
void pid_reset(pid_ctrl_t *pid);

// Run one control step. Retunr the command to send to the actuator.
// setpoint     = where you want to eb (e.g. 0 degrees tilt)
// measurement  = where you are now (e.g. filtered IMU angle)
// dt           = seconds since the previous call (0.001 at 1kHz)
float pid_update(pid_ctrl_t *pid, float setpoint, float measurement, float kd);

#endif //PID_H