/* 
 * File:   pid.h
 * Author: Kevin Dederer
 * Comments: Header file for the pid loops
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include "config.h" // include processor files - each processor file is guarded.  

void pid_control_function(int *real_pitch, int *real_roll, int *real_yaw,
        int *set_pitch, int *set_roll, int *set_yaw, int *location_x, int *location_y,
        int *location_z, int *set_x, int *set_y, int *set_z, int *e1, int *e2, int *e3,
        int *e4);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

