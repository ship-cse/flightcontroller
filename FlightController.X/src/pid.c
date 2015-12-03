/* 
 * File:   pid.c
 * Author: Kevin Dederer
 * Comments: controlling logic for the pid loops
 * Revision history: 
 */

#include "config.h"

#define MAX (290)       // max output value for functions
#define MIN (22)        // min output value for functions
#define HOVER (140)      // speed at which craft will hover (approx.))
#define PID_DT (2.0/1000.0) // the frequency at which the pid loops are executed
#define MAX_STEP (20)   // the maximum allowable increase in speed

/*
 * PID - controls the engine speed for the front left engine.
 * @param location - struct with all of the location data. (user and actual)
 * @param engine - pointer to struct with all of the pid necessary engine values
 * @param p_data - struct containing the pid parameters.
 */
void pid(location_data *location, volatile struct e_data *engine, pid_data *p_data)
{
    float error, pitch_error, roll_error, yaw_error, p, i, d;
    
    pitch_error = (location->user.pitch - location->actual.pitch) * engine->pitch_sign;
    roll_error = (location->user.roll - location->actual.roll) * engine->roll_sign;
    yaw_error = (location->actual.yaw - location->user.yaw) * engine->yaw_sign;
    error = (pitch_error + roll_error + yaw_error);
    p = p_data->kp * error;
    engine->total += error * PID_DT;
    i = engine->total * p_data->ki;
    d = p_data->kd * (error - engine->last) / PID_DT;
    engine->last = error;
    engine->speed = (p + i + d);
}

/* translation - manipulates the engine speed data to be within the desired range
 * @param *engine - pointer to the struct of the engine being modified.
 */
void translation(struct e_data *engine)
{
    int temp = engine->last_speed + MAX_STEP;
    engine->speed /= OFFSET;
    engine->speed = (engine->speed > temp) ? temp : engine->speed;
    engine->speed = (engine->speed > MAX) ? MAX : engine->speed;
    engine->speed = (engine->speed < MIN) ? MIN : engine->speed;
}

/*
 * PID CONTROL FUNCTION - callable by main to run the 4 pid functions with a single call
 * @param location - struct with all of the location data. (user and actual)
 * @param engine - struct with all of the pid necessary engine values
 */
void pid_control_function(location_data *location, volatile engine_data *engine)
{
    pid_data p_data = {5.0, 0.3, 3.0};

    pid(location, &engine->e1, &p_data);
    pid(location, &engine->e2, &p_data);
    pid(location, &engine->e3, &p_data);
    pid(location, &engine->e4, &p_data);
}
