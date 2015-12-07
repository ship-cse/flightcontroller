/* 
 * File:   lsm330tr.h
 * Author: Kevin Dederer
 * Comments: Header file for the lsm330tr sensor, callable function definitions
 * Revision history: 
 */
 
#ifndef LSM330TR_H
#define	LSM330TR_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/*
 * sensor_data - struct containing the variables for the sensor output
 * @param accel_x - the acceleration in the x direction read from the sensor.
 * @param accel_y - the acceleration in the y direction read from the sensor.
 * @param accel_z - the acceleration in the z direction read from the sensor.
 * @param pitch - the pitch read from the sensor.
 * @param roll - the roll read from the sensor.
 * @param yaw - the yaw read from the sensor.
 */
typedef struct
{
    float accel_x;
    float accel_x_zero;
    float accel_y;
    float accel_y_zero;
    float accel_z;
    float accel_z_zero;
} sensor_data;
    
int read_accel(sensor_data *lsm330);
int configure_lsm330tr(sensor_data *lsm330);
     
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

