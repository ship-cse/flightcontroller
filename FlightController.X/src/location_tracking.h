/* 
 * File:   location_tracking.h
 * Author: Kevin Dederer
 * Comments: Header file for the location, velocity and attitude tracking 
 *              functions. 
 * Revision history: 
 */

#ifndef LOCATIONTRACKING_H
#define	LOCATIONTRACKING_H

#ifdef	__cplusplus
extern "C" {
#endif

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
        int x:32;
        int y:32;
        int z:32;        
    } user, actual;
} location_data;

void find_orientation_and_velocity(location_data *attitude, sensor_data lsm330);

#ifdef	__cplusplus
}
#endif

#endif	/* ORIENTATIONTRACKING_H */

