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
 * E1 PID - controls the engine speed for the front left engine.
 * @param actual - struct with all of the location data. (user and actual)
 * @param engine - pointer to struct with all of the pid necessary engine values
 * @param p_data - struct containing the pid parameters.
 */
void e1_pid(location_data location, engine_data *engine, pid_data p_data)
{
    int diff_x = 0, diff_y = 0, diff_z = 0, sum_diff, diff_pitch, diff_roll, diff_yaw, p_out, 
            i_out, d_out, output;
    
    diff_x = (location.actual.x - location.user.x) * p_data.offset;
    diff_y = (location.actual.y - location.user.y) * p_data.offset;
    diff_z = location.user.z - location.actual.z;
    diff_pitch = location.user.pitch - location.actual.pitch;
    diff_roll = location.user.roll - location.actual.roll;
    diff_yaw = location.actual.yaw - location.user.yaw;
    sum_diff = (diff_x + diff_y + diff_z + diff_pitch + diff_roll + diff_yaw)/OFFSET;
    p_out = p_data.kp * sum_diff;
    engine->e1.total += sum_diff * PID_DT;
    i_out = engine->e1.total * p_data.ki;
    d_out = p_data.kd * (sum_diff - engine->e1.last) / PID_DT;
    engine->e1.last = sum_diff;
    output = p_out + i_out + d_out;
    output /= 10;
    output += HOVER;
    if(output > (engine->e1.speed + MAX_STEP))
    {
        engine->e1.speed += MAX_STEP;
    }
    if(output > MAX)
    {
        engine->e1.speed =  MAX;
    }
    else if(output < MIN)
    {
        engine->e1.speed =  MIN;
    }
    else 
    {
        engine->e1.speed = output;
    }
}

/*
 * E2 PID - controls the engine speed for the front right engine.
 * @param location.actual - struct with all of the location data. (location.user and location.actual)
 * @param engine - struct with all of the pid necessary engine values
 * @param p_data - struct containing the pid parameters
 */
void e2_pid(location_data location, engine_data *engine, pid_data p_data)
{
    int diff_x = 0, diff_y = 0, diff_z = 0, sum_diff, diff_pitch, diff_roll, diff_yaw, p_out, 
            i_out, d_out, output;
    
    diff_x = (location.actual.x - location.user.x) * p_data.offset;
    diff_y = (location.actual.y - location.user.y) * p_data.offset;
    diff_z = location.user.z - location.actual.z;
    diff_pitch = location.user.pitch - location.actual.pitch;
    diff_roll = location.actual.roll - location.user.roll;
    diff_yaw = location.user.yaw - location.actual.yaw;
    sum_diff = (diff_x + diff_y + diff_z + diff_pitch + diff_roll + diff_yaw) / OFFSET;
    p_out = p_data.kp * sum_diff;
    engine->e2.total += sum_diff * PID_DT;
    i_out = engine->e2.total * p_data.ki;
    d_out = p_data.kd * (sum_diff - engine->e2.last) / PID_DT;
    engine->e2.last = sum_diff;
    output = p_out + i_out + d_out;
    output /= 10;
    output += HOVER;
    if(output > (engine->e2.speed + MAX_STEP))
    {
        engine->e2.speed += MAX_STEP;
    }
    if(output > MAX)
    {
        engine->e2.speed= MAX;
    }
    else if(output < MIN)
    {
        engine->e2.speed = MIN;
    }
    else
    {
        engine->e2.speed = output;
    }
}

/*
 * E3 PID - controls the engine speed for the back right engine.
 * @param location.actual - struct with all of the location data. (location.user and location.actual)
 * @param engine - struct with all of the pid necessary engine values
 * @param p_data - struct containing the pid parameters
 */
void e3_pid(location_data location, engine_data *engine, pid_data p_data)
{
    int diff_x = 0, diff_y = 0, diff_z = 0, sum_diff, diff_pitch, diff_roll, diff_yaw, p_out, 
            i_out, d_out, output;
    
    diff_x = (location.user.x - location.actual.x) * p_data.offset;
    diff_y = (location.actual.y - location.user.y) * p_data.offset;
    diff_z = location.user.z - location.actual.z;
    diff_pitch = location.actual.pitch - location.user.pitch;
    diff_roll = location.actual.roll - location.user.roll;
    diff_yaw = location.actual.yaw - location.user.yaw;
    sum_diff = (diff_x + diff_y + diff_z + diff_pitch + diff_roll + diff_yaw) / OFFSET;
    p_out = p_data.kp * sum_diff;
    engine->e3.total += sum_diff * PID_DT;
    i_out = engine->e3.total * p_data.ki;
    d_out = p_data.kd * (sum_diff - engine->e3.last) / PID_DT;
    engine->e3.last = sum_diff;
    output = p_out + i_out + d_out;
    output /= 10;
    output += HOVER;
    if(output > (engine->e3.speed + MAX_STEP))
    {
        engine->e3.speed += MAX_STEP;
    }
    if(output > MAX)
    {
        engine->e3.speed = MAX;
    }
    else if(output < MIN)
    {
        engine->e3.speed = MIN;
    }
    else
    {
        engine->e3.speed = output;    
    }
}

/*
 * E4 PID - controls the engine speed for the back left engine.
 * @param location.actual - struct with all of the location data. (location.user and acual)
 * @param engine - struct with all of the pid necessary engine values
 * @param p_data - struct containing the pid parameters
 */
void e4_pid(location_data location, engine_data *engine, pid_data p_data)
{
    int diff_x = 0, diff_y = 0, diff_z = 0, sum_diff, diff_pitch, diff_roll, diff_yaw, p_out, 
            i_out, d_out, output;
    
    diff_x = (location.user.x - location.actual.x) * p_data.offset;
    diff_y = (location.actual.y - location.user.y) * p_data.offset;
    diff_z = location.user.z - location.actual.z;
    diff_pitch = location.actual.pitch - location.user.pitch;
    diff_roll = location.user.roll - location.actual.roll;
    diff_yaw = location.user.yaw - location.actual.yaw;
    sum_diff = (diff_x + diff_y + diff_z + diff_pitch + diff_roll + diff_yaw) / OFFSET;
    p_out = p_data.kp * sum_diff;
    engine->e4.total += sum_diff * PID_DT;
    i_out = engine->e4.total * p_data.ki;
    d_out = p_data.kd * (sum_diff - engine->e4.last) / PID_DT;
    engine->e4.last = sum_diff;
    output = p_out + i_out + d_out;
    output /= 10;
    output += HOVER;
    if(output > (engine->e4.speed + MAX_STEP))
    {
        engine->e4.speed += MAX_STEP;
    }    
    if(output > MAX)
    {
        engine->e4.speed = MAX;
    }
    else if(output < MIN)
    {
        engine->e4.speed = MIN;
    }
    else
    {
        engine->e4.speed = output;
    }
}

/*
 * PID CONTROL FUNCTION - callable by main to run the 4 pid functions with a single call
 * @param location - struct with all of the location data. (user and actual)
 * @param engine - struct with all of the pid necessary engine values
 */
void pid_control_function(location_data location, engine_data *engine)
{
    pid_data p_data = {5.0, 0.3, 3.0, 0.0};
    p_data.offset = cos(location.actual.yaw / OFFSET * RAD);

    e1_pid(location, engine, p_data);
    e2_pid(location, engine, p_data);
    e3_pid(location, engine, p_data);
    e4_pid(location, engine, p_data);
}
