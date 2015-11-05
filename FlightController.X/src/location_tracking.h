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
    
void find_orientation_and_velocity(float *, float *, float *, int *, int *,
        int *, float *, float *, float *);
int get_location_x(int, int, int);
int get_location_y(int, int, int);
int get_location_z(int, int);

void get_scale();

#ifdef	__cplusplus
}
#endif

#endif	/* ORIENTATIONTRACKING_H */

