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
 */
typedef struct
{
    struct e_data
    {
        int total:32;
        int last:32;
        int speed:32;
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
    float offset;
} pid_data;

void pid_control_function(location_data location, engine_data *constant);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

