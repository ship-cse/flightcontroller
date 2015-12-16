/* 
 * File:   pid.c
 * Author: Kevin Dederer
 * Comments: controlling logic for the pid loops
 * Revision history: 
 */

#include "config.h"

#define MAX (4650)       // max output value for functions
#define MIN (2650)        // min output value for functions
#define HOVER (3400)      // speed at which craft will hover (approx.))
#define PID_DT (.01) // the frequency at which the pid loops are executed
#define PID_FACTOR (12)
/*
 * PID - controls the engine speed for the front left engine.
 * @param location - struct with all of the location data. (user and actual)
 * @param engine - pointer to struct with all of the pid necessary engine values
 * @param p_data - struct containing the pid parameters.
 */
void pid(location_data *location, struct e_data *engine, pid_data *p_data)
{
    float error, pitch_error, roll_error, yaw_error, p, i, d;
    
    pitch_error = (location->user.pitch - location->actual.pitch) * engine->pitch_sign;
    roll_error = (location->user.roll - location->actual.roll) * engine->roll_sign;
    yaw_error = (location->user.accel_z - location->actual.accel_z);
    error = (pitch_error + roll_error + yaw_error);
    p = p_data->kp * error;
    engine->total += error * PID_DT;
    i = engine->total * p_data->ki;
    d = p_data->kd * (error - engine->last) / PID_DT;
    engine->last = error;
    engine->pid_out = (p + i + d);
}

/* translation - manipulates the engine speed data to be within the desired range
 * @param *engine - pointer to the struct of the engine being modified.
 */
void translation(struct e_data *engine)
{
    float sgn = (engine->pid_out < 0) ? -1 : 1;
    float temp = sgn * pow(abs(engine->pid_out),2.4) * PID_FACTOR + HOVER;
    temp = (temp > MAX) ? MAX : temp;
    temp = (temp < MIN) ? MIN : temp;
    engine->speed = temp;
}

/*
 * PID CONTROL FUNCTION - callable by main to run the 4 pid functions with a single call
 * @param location - struct with all of the location data. (user and actual)
 * @param engine - struct with all of the pid necessary engine values
 */
void pid_control_function(location_data *location, engine_data *engine)
{
    pid_data p_data = {3.15, 1.85, 0.90};

    pid(location, &engine->e1, &p_data);
    pid(location, &engine->e2, &p_data);
    pid(location, &engine->e3, &p_data);
    pid(location, &engine->e4, &p_data);
    translation(&engine->e1);
    translation(&engine->e2);
    translation(&engine->e3);
    translation(&engine->e4);
}
