/* 
 * File:   lsm330tr.h
 * Author: Kevin Dederer
 * Comments: Header file for the lsm330tr sensor, callable function definitions
 * Revision history: 
 */

//TODO ERROR - read about include guards and explain to me why this is totally wrong!!!!
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

//TODO why is this here - what depends on it?
#include <xc.h> // include processor files - each processor file is guarded.  

int read_accel( float *accel_x, float *accel_y, float *accel_z);
int read_gyro( float *pitch, float *roll, float *yaw);
int configure_lsm330tr();
int configure_lsm330tr_test();
void get_accel_scale(int *scale);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

//TODO you checked code in with a todo?
    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

