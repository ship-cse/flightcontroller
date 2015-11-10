/* 
 * File:   pid.c
 * Author: Kevin Dederer
 * Comments: controlling logic for the pid loops
 * Revision history: 
 */

#include "config.h"

//TODO comment what these are for - the RAD  I get, but the rest?
//TODO You are doing everything floating point - its going to kill your performance 
#define OFFSET (10000.0)
#define RAD (M_PI / 180.0)
#define DT (1.0/500.0)
#define HOVER (65)
#define MAX (116 - HOVER)
#define MIN (59 - HOVER)

//TODO why are these global?  This is not acceptable.  You CAN create a struct to hold them, but then pass the struct to the functions
float diff_x, diff_y, diff_z, sum_diff, diff_pitch, diff_roll, diff_yaw, p_out, e1_total,
        i_out, d_out, e1_last, e1_out, kp = 1.0, kd = 0, ki = 0, e2_total, e2_last, e3_total,
        e3_last, e4_total, e4_last, offset, e2_out, e3_out, e4_out;


//TODO The cost of making a function call increases with each variable - wrap this up into a struct, pass a pointer to the struct

/*
 * E1 PID - controls the engine speed for the front left engine.
 * @param real_pitch - the current pitch of the craft.
 * @param real_roll - the current roll of the craft.
 * @param real_yaw - the current yaw of the craft.
 * @param location_x - the current x location of the craft.
 * @param location_y - the current y location of the craft.
 * @param location_z - the current z location of the craft.
 * @param set_pitch - the desired pitch of the craft.
 * @param set_roll - the desired roll of the craft.
 * @param set_yaw - the desired yaw of the craft.
 * @param set_x - the desired x location of the craft.
 * @param set_y - the desired y location of the craft.
 * @param set_z - the desired z location of the craft.
 */
int e1_pid(int real_pitch, int real_roll, int real_yaw, int location_x,
        int location_y, int location_z, int set_pitch, int set_roll,
        int set_yaw, int set_x, int set_y, int set_z)
{
    diff_x = (location_x - set_x) * offset;             //TODO Ok, I get this
    diff_y = (set_y - location_y) * offset;
    diff_z = set_z - location_z;
    diff_pitch = set_pitch - real_pitch;                //TODO I even get this
    diff_roll = set_roll - real_roll;
    diff_yaw = real_yaw - set_yaw;
    //TODO This I don't get - X,Y,Z, pitch,roll, and yaw are 6 different values - why isn't this a 6-dimensional vector?
    sum_diff = diff_x + diff_y + diff_z + diff_pitch + diff_roll + diff_yaw;
    p_out = kp * sum_diff;
    e1_total += sum_diff;
    i_out = e1_total * ki;
    d_out = kd * (e1_last * sum_diff) / DT;
    e1_last = sum_diff;
    e1_out = p_out + i_out + d_out;
    if(e1_out > MAX)
        return MAX + HOVER;
    if(e1_out < MIN)
        return MIN + HOVER;
    
    return e1_out + HOVER;
}

/*
 * E2 PID - controls the engine speed for the front right engine.
 * @param real_pitch - the current pitch of the craft.
 * @param real_roll - the current roll of the craft.
 * @param real_yaw - the current yaw of the craft.
 * @param location_x - the current x location of the craft.
 * @param location_y - the current y location of the craft.
 * @param location_z - the current z location of the craft.
 * @param set_pitch - the desired pitch of the craft.
 * @param set_roll - the desired roll of the craft.
 * @param set_yaw - the desired yaw of the craft.
 * @param set_x - the desired x location of the craft.
 * @param set_y - the desired y location of the craft.
 * @param set_z - the desired z location of the craft.
 */
int e2_pid(int real_pitch, int real_roll, int real_yaw, int location_x,
        int location_y, int location_z, int set_pitch, int set_roll,
        int set_yaw, int set_x, int set_y, int set_z)
{
    diff_x = (location_x - set_x) * offset;
    diff_y = (location_y - set_y) * offset;
    diff_z = set_z - location_z;
    diff_pitch = set_pitch - real_pitch;
    diff_roll = real_roll - set_roll;
    diff_yaw = set_yaw - real_yaw;
    sum_diff = diff_x + diff_y + diff_z + diff_pitch + diff_roll + diff_yaw;
    p_out = kp * sum_diff;
    e2_total += sum_diff;
    i_out = e2_total * ki;
    d_out = kd * (e2_last * sum_diff) / DT;
    e2_last = sum_diff;
    e2_out = p_out + i_out + d_out;
    if(e2_out > MAX)
        return MAX + HOVER;
    if(e2_out < MIN)
        return MIN + HOVER;
    
    return e2_out + HOVER;
}

/*
 * E3 PID - controls the engine speed for the back right engine.
 * @param real_pitch - the current pitch of the craft.
 * @param real_roll - the current roll of the craft.
 * @param real_yaw - the current yaw of the craft.
 * @param location_x - the current x location of the craft.
 * @param location_y - the current y location of the craft.
 * @param location_z - the current z location of the craft.
 * @param set_pitch - the desired pitch of the craft.
 * @param set_roll - the desired roll of the craft.
 * @param set_yaw - the desired yaw of the craft.
 * @param set_x - the desired x location of the craft.
 * @param set_y - the desired y location of the craft.
 * @param set_z - the desired z location of the craft.
 */
int e3_pid(int real_pitch, int real_roll, int real_yaw, int location_x,
        int location_y, int location_z, int set_pitch, int set_roll,
        int set_yaw, int set_x, int set_y, int set_z)
{
    diff_x = (set_x - location_x) * offset;
    diff_y = (location_y - set_y) * offset;
    diff_z = set_z - location_z;
    diff_pitch = real_pitch - set_pitch;
    diff_roll = real_roll - set_roll;
    diff_yaw = real_yaw - set_yaw;
    sum_diff = diff_x + diff_y + diff_z + diff_pitch + diff_roll + diff_yaw;
    p_out = kp * sum_diff;
    e3_total += sum_diff;
    i_out = e3_total * ki;
    d_out = kd * (e3_last * sum_diff) / DT;
    e3_last = sum_diff;
    e3_out = p_out + i_out + d_out;
    if(e3_out > MAX)
        return MAX + HOVER;
    if(e3_out < MIN)
        return MIN + HOVER;
    
    return e3_out + HOVER;
}

/*
 * E4 PID - controls the engine speed for the back left engine.
 * @param real_pitch - the current pitch of the craft.
 * @param real_roll - the current roll of the craft.
 * @param real_yaw - the current yaw of the craft.
 * @param location_x - the current x location of the craft.
 * @param location_y - the current y location of the craft.
 * @param location_z - the current z location of the craft.
 * @param set_pitch - the desired pitch of the craft.
 * @param set_roll - the desired roll of the craft.
 * @param set_yaw - the desired yaw of the craft.
 * @param set_x - the desired x location of the craft.
 * @param set_y - the desired y location of the craft.
 * @param set_z - the desired z location of the craft.
 */
int e4_pid(int real_pitch, int real_roll, int real_yaw, int location_x,
        int location_y, int location_z, int set_pitch, int set_roll,
        int set_yaw, int set_x, int set_y, int set_z)
{
    diff_x = (set_x - location_x) * offset;
    diff_y = (location_y - set_y) * offset;
    diff_z = set_z - location_z;
    diff_pitch = real_pitch - set_pitch;
    diff_roll = set_roll - real_roll;
    diff_yaw = set_yaw - real_yaw;
    sum_diff = diff_x + diff_y + diff_z + diff_pitch + diff_roll + diff_yaw;
    p_out = kp * sum_diff;
    e4_total += sum_diff;
    i_out = e4_total * ki;
    d_out = kd * (e4_last * sum_diff) / DT;
    e4_last = sum_diff;
    e4_out = p_out + i_out + d_out;
    e4_out /= 1000;
    if(e4_out > MAX)
        return MAX + HOVER;
    if(e4_out < MIN)
        return MIN + HOVER;
    
    return e4_out + HOVER;
}

//TODO - see above comment about structs... this is really fragile code here.
/*
 * PID CONTROL FUNCTION - callable by main to run the 4 pid functions with a single call
 * @param *real_pitch - pointer to the current pitch of the craft.
 * @param *real_roll - pointer to the current roll of the craft.
 * @param *real_yaw - pointer to the current yaw of the craft.
 * @param *location_x - pointer to the current x location of the craft.
 * @param *location_y - pointer to the current y location of the craft.
 * @param *location_z - pointer to the current z location of the craft.
 * @param *set_pitch - pointer to the desired pitch of the craft.
 * @param *set_roll - pointer to the desired roll of the craft.
 * @param *set_yaw - pointer to the desired yaw of the craft.
 * @param *set_x - pointer to the desired x location of the craft.
 * @param *set_y - pointer to the desired y location of the craft.
 * @param *set_z - pointer to the desired z location of the craft.
 * @param *e1 - pointer to the e1_pulse_time variable in main.
 * @param *e2 - pointer to the e2_pulse_time variable in main.
 * @param *e3 - pointer to the e3_pulse_time variable in main.
 * @param *e4 - pointer to the e4_pulse_time variable in main.
 */
void pid_control_function(int *real_pitch, int *real_roll, int *real_yaw,
        int *set_pitch, int *set_roll, int *set_yaw, int *location_x, int *location_y,
        int *location_z, int *set_x, int *set_y, int *set_z, int *e1, int *e2, int *e3,
        int *e4)
{
    int temp = *real_yaw;
    offset = cos((temp / OFFSET) * RAD);
    
    *e1 = e1_pid(*real_pitch, *real_roll, *real_yaw, *location_x, *location_y, 
            *location_z, *set_pitch, *set_roll, *set_yaw, *set_x, *set_y, *set_z);
    
    *e2 = e2_pid(*real_pitch, *real_roll, *real_yaw, *location_x, *location_y, 
            *location_z, *set_pitch, *set_roll, *set_yaw, *set_x, *set_y, *set_z);
    
    *e3 = e3_pid(*real_pitch, *real_roll, *real_yaw, *location_x, *location_y, 
            *location_z, *set_pitch, *set_roll, *set_yaw, *set_x, *set_y, *set_z);
    
    *e4 = e4_pid(*real_pitch, *real_roll, *real_yaw, *location_x, *location_y, 
            *location_z, *set_pitch, *set_roll, *set_yaw, *set_x, *set_y, *set_z);
}
