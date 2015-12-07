/* 
 * File:   pid.h
 * Author: Kevin Dederer
 * Comments: Header file for the pid loops
 * Revision history: 
 */

#ifndef PID_H
#define	PID_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/*
 * engine_data - contains a struct allowing a data set for each engine
 * e_data - struct containing the individual data for each engine
 * @param total - the accumulated difference for each engine from the pid loop
 * @param last - the last difference for each engine from the pid loop
 * @param speed - the current speed setting for each engine as determined by the pid loop
 * @param last_speed - the previous speed setting
 * @param pitch_sign - the sign value for the difference in pitch
 * @param roll_sign - the sign value for the difference in roll
 * @param yaw_sign - the sign value for the difference in yaw
 */
typedef struct
{
    struct e_data
    {
        float total;
        float last;
        int speed;
        float pid_out;
        int pitch_sign;
        int roll_sign;
        int yaw_sign;
    } e1, e2, e3, e4;
} engine_data;

/*
 * pid_data - struct containing all of the necessary values to operate the pid loops
 * @param kp - the peripheral gain constant.
 * @param ki - the integral gain constant.
 * @param kd - the derivative gain constant.
 * @param offset - the offset variable which is based off of the yaw
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_data;

/*
 * location_data - contains a struct with two values, the user values and the actual values
 * data - struct containing the pitch, roll, yaw, x, y, and z values for location
 */
typedef struct
{
    struct data
    {
        float pitch;
        float roll;
        float yaw;
    } user, actual;
} location_data;

void pid_control_function(location_data *location, engine_data *constant);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

