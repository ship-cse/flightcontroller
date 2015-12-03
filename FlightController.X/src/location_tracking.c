/* 
 * File:   location_tracking.c
 * Author: Kevin Dederer
 * Comments: main logic file for the location, velocity and location->actual tracking
 *              functions.
 * Revision history: 
 */

//    rotation_matrix[0][1] = cos(y) * sin(r) * sin(p) - sin(y) * cos(r); \
//    rotation_matrix[0][2] = cos(y) * sin(p) * cos(r) + sin(y) * sin(r); \
//    rotation_matrix[1][1] = sin(y) * sin(p) * sin(r) + cos(y) * cos(r); \
//    rotation_matrix[1][2] = sin(y) * sin(p) * cos(r) - cos(y) * sin(r); \

#include "config.h"
      
#define GRAVITY (9.8)       // decimal force of gravity in m/s^2
#define DEG (180.0 / M_PI)  // conversion from radians to degrees

/*
 * ZYX_MATRIX - rotation matrix for location tracking, includes only those values
 *              needed to track the location
 */
#define ZYX_MATRIX(y, p, r, rotation_matrix) { \
    rotation_matrix[0][0] = cos(y) * cos(p); \
    rotation_matrix[1][0] = sin(y) * cos(p); \
    rotation_matrix[2][0] = 0.0 - sin(p); \
    rotation_matrix[2][1] = cos(p) * sin(r); \
    rotation_matrix[2][2] = cos(p) * cos(r); } \


/*
 * GET SCALE - determines the accelerometer sensitivity setting to be used in 
 *              the complementary filter
 * @output - the integer value of the accelerometer's scale setting
 */
int set_scale()
{        
    return get_accel_scale();
}

/* COMPLEMENTARY FILTER - The function combines a large part of the gyro readings
 *                  with a small portion of the given angle from the accelerometer
 *                  readings to allow better accuracy. Scale is static so that it
 *                  will only update once because the accelerometer setting will not 
 *                  change during operation
 * @param a_x - acceleration on the x axis
 * @param a_y - acceleration on the y axis
 * @param a_z - acceleration on the z axis
 * @param *location - pointer to a struct containing integer values for the pitch, roll, and yaw data
 */
void complementary_filter(float a_x, float a_y, float a_z,
        location_data *location)
{
    static int scale = -1;
    if(scale == -1)
        scale = set_scale();

    float force_magnitude_approx = fabs(a_x) + fabs(a_y) + fabs(a_z);
    a_z = (a_z > 1) ? 1 : a_z;
    a_z = (a_z < -1) ? -1 : a_z;
    if(force_magnitude_approx > .5 && force_magnitude_approx < (scale))
    {
        int pitch_acc = atan2f(a_x, a_z) * DEG * OFFSET;
        location->actual.pitch *= 0.95 + 0.05 * pitch_acc;

        int roll_acc = 0.0 - atan2f(a_y, a_z) * DEG * OFFSET;
        location->actual.roll *= 0.95  + 0.05 * roll_acc;
    }
}

/*
 * FIND LOCATION X - tracks the x location of the drone
 * @param pitch - the current pitch of the drone.
 * @param roll - the current roll of the drone.
 * @param yaw - the current yaw of the drone.
 * @param v_x - the velocity in the x direction.
 * @param v_y - the velocity in the y direction.
 * @param v_z - the velocity in the z direction.
 * @return the x location of the drone in 10ths of millimeters
 */
int find_location_x(float pitch, float roll, float yaw, float v_x, float v_y, float v_z)
{
    float value_x = v_x * cos(yaw) * cos(pitch);
    float value_y = v_y  * sin(yaw) * cos(roll);
    float value_z = v_z * (cos(yaw) * sin(pitch) * -1 +
        sin(yaw) * sin(roll));
    
    return (OFFSET * (value_x + value_y + value_z));
}

/*
 * FIND LOCATION Y - tracks the y location of the drone
 * @param pitch - the current pitch of the drone.
 * @param roll - the current roll of the drone.
 * @param yaw - the current yaw of the drone.
 * @param v_x - the velocity in the x direction.
 * @param v_y - the velocity in the y direction.
 * @param v_z - the velocity in the z direction.
 * @return the y location of the drone in 10ths of millimeters
 */
int find_location_y(float pitch, float roll, float yaw, float v_x, float v_y, float v_z)
{
    float value_y = v_y * cos(yaw) * cos(roll);
    float value_x = v_x * sin(yaw) * cos(pitch) * -1;
    float value_z = v_z * (cos(yaw) * sin(roll) + 
        sin(yaw) * sin(pitch));
    
    return (OFFSET * (value_y + value_x + value_z));
}

/*
 * FIND LOCATION Z - tracks the z location of the drone
 * @param pitch - the current pitch of the drone.
 * @param roll - the current roll of the drone.
 * @param v_x - the velocity in the x direction.
 * @param v_y - the velocity in the y direction.
 * @param v_z - the velocity in the z direction.
 * @return the z location of the drone in 10ths of millimeters
 */
int find_location_z(float pitch, float roll, float v_x, float v_y, float v_z)
{
    float value_z =  v_z * cos(pitch) * cos(roll);
    float value_x = v_x * sin(pitch);
    float value_y = v_y * sin(roll) * -1;
    
    return (OFFSET * (value_z + value_x + value_y));
}

/*
 * FIND PITCH - calculates the updated pitch using a rotation matrix
 * @param *pitch - a pointer to the current pitch value
 * @param rotation_matrix - the matrix containing the necessary rotation information.
 * @return the integer pitch value
 * @see ZYX_MATRIX(y, p, r)
 */
int find_pitch(int pitch, float rotation_matrix[3][3])
{
    pitch += (atan2f((0.0-rotation_matrix[2][0]), 
            sqrt(pow(rotation_matrix[2][1],2) + pow(rotation_matrix[2][2],2))) * 
            OFFSET * DEG);
    pitch = (pitch <= 1800000) ? pitch : 1800000 - pitch;
    return (pitch > -1800000) ? pitch : 1800000 + pitch;
    
}

/*
 * FIND ROLL - calculates the updated roll using a rotation matrix
 * @param *roll - a pointer to the current roll value
 * @param rotation_matrix - the matrix containing the necessary rotation information.
 * @return the integer roll value
 * @see ZYX_MATRIX(y, p, r)
 */
int find_roll(int roll, float rotation_matrix[3][3])
{
    roll += (atan2f(rotation_matrix[2][1], rotation_matrix[2][2]) * DEG * 
            OFFSET);
    roll = (roll <= 1800000) ? roll : roll - 3600000;
    return (roll > -1800000) ? roll : roll + 3600000;
}
    
/*
 * FIND YAW - calculates the updated yaw using a rotation matrix
 * @param *yaw - a pointer to the current yaw value
 * @param rotation_matrix - the matrix containing the necessary rotation information.
 * @return the integer yaw value
 * @see ZYX_MATRIX(y, p, r)
 */
int find_yaw(int yaw, float rotation_matrix[3][3])
{
    yaw += (atan2f(rotation_matrix[1][0], rotation_matrix[0][0]) * DEG * 
            OFFSET);
    yaw = (yaw <= 1800000) ? yaw : yaw - 3600000;
    return (yaw > -1800000) ? yaw : yaw + 3600000;
}

/*
 * FIND VELOCITY X - tracks the velocity in the x direction of the craft.
 *                  if the craft is level it cannot be moving in the x direction
 *                  so if the pitch is < +- 1 degree it resets the velocity to 0.
 *                  it also compensates for the acceleration of gravity if pitched
 * @param accel_x - the acceleration in the x direction
 * @param pitch - the current pitch of the drone
 * @param velocity_x - the integer velocity value for the x direction
 * @return - the updated integer value for the velocity in the x direction
 */
int find_velocity_x(float accel_x, float pitch, int velocity_x)
{
    float temp = fabs(sin(pitch));
    
    if(pitch < -0.01745)
    {
        accel_x += temp;
    }
    else if (pitch > 0.01745)
    {
        accel_x -= temp;
    }
    else
    {
        velocity_x = 0;
    }
    
    velocity_x += accel_x * GRAVITY * OFFSET;
    
    return velocity_x;
}

/*
 * FIND VELOCITY Y - tracks the velocity in the y direction of the craft.
 *                  if the craft is level it cannot be moving in the y direction
 *                  so if the roll is < +- 1 degree it resets the velocity to 0.
 *                  it also compensates for the acceleration of gravity if rolled
 * @param accel_y - the acceleration in the y direction
 * @param roll - the current roll of the drone
 * @param velocity_y - the integer value for the velocity in the y direction.
 * @return - the updated integer value for the velocity in the y direction.
 */
int find_velocity_y(float accel_y, float roll, int velocity_y)
{
    float temp = fabs(sin(roll));
    
    if(roll < -0.01745)
    {
        accel_y += temp;
    }
    else if(roll > 0.01745)
    {
        accel_y -= temp;
    }
    else
    {
        velocity_y = 0;
    }
    
    accel_y = (fabs(accel_y) < .05) ? 0 : accel_y;
    
    return velocity_y + accel_y * GRAVITY * OFFSET;
}

/*
 * FIND VELOCITY Z - tracks the velocity in the z direction of the craft.
 *                  it will compensate for gravity based off of the pitch and 
 *                  roll of the craft. If it is level and registering approximately
 *                  1 g it will reset the velocity to 0.
 * @param accel_z - the acceleration in the z direction
 * @param pitch - the current pitch of the drone
 * @param roll - the current roll of the drone
 * @param velocity_z - the integer value of the velocity in the z direction.
 * @return - the updated integer value for the velocity in the z direction.
 */
int find_velocity_z(float accel_z, float pitch, float roll, int velocity_z)
{
    if(fabs(pitch) < 0.01745 && fabs(roll) < 0.01745 && fabs(accel_z) < 1.1000 &&
            fabs(accel_z) > .9000)
        velocity_z = 0;
    
    accel_z -= cos(fabs(pitch)) * cos(fabs(roll));

    accel_z = (fabs(accel_z) < .05) ? 0 : accel_z;
    
    return velocity_z + accel_z * GRAVITY  * OFFSET;    
}

/*
 * FIND ORIENTATION AND VELOCITY - the main logic function for the location tracking
 *                  file. This method is called by the main method and calls all of the
 *                  other methods. Data is manipulated as needed to be passed to those
 *                  methods in order to cut out duplicate equations in each method.
 * @param *location - pointer to the struct containing the integer values for
 *                  the x, y, z location and the pitch, roll and yaw
 * @param lsm330 - struct containing the float values read from the sensor
 *                  (accel_x, accel_y, accel_z, pitch, roll, and yaw)
 */
void find_orientation_and_velocity(location_data *location, sensor_data lsm330)
{
    static int velocity_x, velocity_y, velocity_z;
    float p = lsm330.pitch, r = lsm330.roll, y = lsm330.yaw, a_x = lsm330.accel_x,
            a_y = lsm330.accel_y, a_z = lsm330.accel_z;
    float rotation_matrix[3][3];
    
    a_x = (fabs(a_x) < .05) ? 0 : a_x * DT;
    a_y = (fabs(a_y) < .05) ? 0 : a_y * DT;
    a_z = (fabs(a_z) < .05) ? 0 : a_z * DT;
    p = (fabs(p) < .1) ? 0 : p * RAD * DT;
    r = (fabs(r) < .1) ? 0 : r * RAD * DT;
    y = (fabs(y) < .05) ? 0 : y * RAD * DT;
    
    ZYX_MATRIX(y, p, r, rotation_matrix);

    location->actual.roll = find_roll(location->actual.roll, rotation_matrix);
    location->actual.yaw = find_yaw(location->actual.yaw, rotation_matrix);
    location->actual.pitch = find_pitch(location->actual.pitch, rotation_matrix);
    
    complementary_filter(lsm330.accel_x, lsm330.accel_y, lsm330.accel_z, location);
    
    float r_p = (float)(location->actual.pitch * RAD / OFFSET);
    float r_r = (float)(location->actual.roll * RAD / OFFSET);
    float r_y = (float)(location->actual.yaw * RAD / OFFSET);
    
    float v_x = find_velocity_x(a_x, r_p, velocity_x) / OFFSET;
    float v_y = find_velocity_y(a_y, r_r, velocity_y) / OFFSET;
    float v_z = find_velocity_z(a_z, r_p, r_r, velocity_z) / OFFSET;
        
    location->actual.x = find_location_x(r_p, r_r, r_y, v_x, v_y, v_z);
    location->actual.y = find_location_y(r_p, r_r, r_y, v_x, v_y, v_z);
    location->actual.z = find_location_z(r_p, r_r, v_x, v_y, v_z);    
}