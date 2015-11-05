/* 
 * File:   location_tracking.c
 * Author: Kevin Dederer
 * Comments: main logic file for the location, velocity and attitude tracking
 *              functions.
 * Revision history: 
 */

#include "config.h"

#define RAD (M_PI / 180.0)
#define GRAVITY (9.8)
#define OFFSET (10000.0)
#define DEG (180.0 / M_PI)
#define DT (1.0/500.0)

#define ZYX_MATRIX(y, p, r) { \
    rotation_matrix[0][0] = cos(y) * cos(p); \
    rotation_matrix[0][1] = cos(y) * sin(r) * sin(p) - sin(y) * cos(r); \
    rotation_matrix[0][2] = cos(y) * sin(p) * cos(r) + sin(y) * sin(r); \
    rotation_matrix[1][0] = sin(y) * cos(p); \
    rotation_matrix[1][1] = sin(y) * sin(p) * sin(r) + cos(y) * cos(r); \
    rotation_matrix[1][2] = sin(y) * sin(p) * cos(r) - cos(y) * sin(r); \
    rotation_matrix[2][0] = 0.0 - sin(p); \
    rotation_matrix[2][1] = cos(p) * sin(r); \
    rotation_matrix[2][2] = cos(p) * cos(r); } \

            
PRIVATE int velocity_x = 0 , velocity_y = 0, velocity_z = 0, scale;
PRIVATE float rotation_matrix[3][3] = {0};
PRIVATE int x_location = 0, y_location = 0, z_location = 0;

/*
 * COMPLEMENTARY FILTER - The function combines a large part of the gyro readings
 *                  with a small portion of the given angle from the accelerometer
 *                  readings to allow better accuracy
 * @param a_x - acceleration on the x axis
 * @param a_y - acceleration on the y axis
 * @param a_z - acceleration on the z axis
 * @param *real_pitch - pointer to the calculated pitch value
 * @param *real_roll - pointer to the calculated roll value
 * @param *real_yaw - pointer to the calculated yaw value
 */
void complementary_filter(float a_x, float a_y, float a_z,
        int *real_pitch, int *real_roll, int *real_yaw)
{
    int p = *real_pitch, r = *real_roll, y = *real_yaw;

    float force_magnitude_approx = abs(a_x) + abs(a_y) + abs(a_z);
    a_z = (a_z > 1.0) ? 1.0 : a_z;
    a_z = (a_z < -1.0) ? -1.0 : a_z;
    if(force_magnitude_approx > .5 && force_magnitude_approx < (scale))
    {
        float pitch_acc = atan2f(a_x, a_z) * DEG * OFFSET;
        *real_pitch = 0.95 * p + 0.05 * pitch_acc;

        float roll_acc = 0.0 - atan2f(a_y, a_z) * DEG * OFFSET;
        *real_roll = 0.95 * r + 0.05 * roll_acc;
    }
}

/*
 * FIND LOCATION X - tracks the x location of the drone
 * @param real_pitch - the current pitch of the drone.
 * @param real_roll - the current roll of the drone.
 * @param real_yaw - the current yaw of the drone.
 * @return the x location of the drone in 10ths of millimeters
 */
int find_location_x(int real_pitch, int real_roll, int real_yaw)
{
    float yaw = real_yaw / OFFSET, pitch = real_pitch / OFFSET, 
            roll = real_roll / OFFSET;
    float v_x = velocity_x / OFFSET, v_y = velocity_y / OFFSET,
            v_z = velocity_z / OFFSET;
    
    int value_x = v_x * cos(yaw * RAD) * cos(pitch * RAD);
    int value_y = v_y  * sin(yaw * RAD) * cos(roll * RAD);
    int value_z = v_z * (cos(yaw * RAD) * (sin(pitch * RAD) * -1 +
        sin(yaw * RAD) * sin(roll * RAD)));
    
    return value_x + value_y + value_z;
}

/*
 * FIND LOCATION Y - tracks the y location of the drone
 * @param real_pitch - the current pitch of the drone.
 * @param real_roll - the current roll of the drone.
 * @param real_yaw - the current yaw of the drone.
 * @return the y location of the drone in 10ths of millimeters
 */
int find_location_y(int real_pitch, int real_roll, int real_yaw)
{
    float yaw = real_yaw / OFFSET, pitch = real_pitch / OFFSET, 
            roll = real_roll / OFFSET;
    float v_x = velocity_x / OFFSET, v_y = velocity_y / OFFSET,
            v_z = velocity_z / OFFSET;

    int value_y = v_y * cos(yaw * RAD) * cos(roll * RAD);
    int value_x = v_x * sin(yaw * RAD) * cos(pitch * RAD) * -1;
    int value_z = v_z * ((cos(yaw * RAD) * sin(roll * RAD)) + 
        (sin(yaw * RAD) * sin(pitch * RAD)));
    
    return value_y + value_x + value_z;
}

/*
 * FIND LOCATION Z - tracks the z location of the drone
 * @param real_pitch - the current pitch of the drone.
 * @param real_roll - the current roll of the drone.
 * @return the z location of the drone in 10ths of millimeters
 */
int find_location_z(int real_pitch, int real_roll)
{
    float pitch = real_pitch / OFFSET, roll = real_roll / OFFSET;
    float v_x = velocity_x / OFFSET, v_y = velocity_y / OFFSET,
            v_z = velocity_z / OFFSET;
    
    int value_z =  v_z * cos(pitch * RAD) * cos(roll * RAD);
    int value_x = v_x * sin(pitch * RAD);
    int value_y = v_y * sin(roll * RAD) * -1;
    
    return value_z + value_x + value_y;
}

/*
 * FIND PITCH - calculates the updated pitch using a rotation matrix
 * @param *real_pitch - a pointer to the current pitch value
 * @return none
 * @see ZYX_MATRIX(y, p, r)
 */
find_pitch(int *real_pitch)
{
    *real_pitch += (atan2f((0.0-rotation_matrix[2][0]), 
            sqrt(pow(rotation_matrix[2][1],2) + pow(rotation_matrix[2][2],2))) * 
            OFFSET * DEG * DT);
    *real_pitch = (*real_pitch <= 1800000) ? *real_pitch : *real_pitch - 3600000;
    *real_pitch = (*real_pitch > -1800000) ? *real_pitch : *real_pitch + 3600000;
}

/*
 * FIND ROLL - calculates the updated roll using a rotation matrix
 * @param *real_roll - a pointer to the current roll value
 * @return none
 * @see ZYX_MATRIX(y, p, r)
 */
find_roll(int *real_roll)
{
    *real_roll += (atan2f(rotation_matrix[2][1], rotation_matrix[2][2]) * DEG * 
            OFFSET * DT);
    *real_roll = (*real_roll <= 1800000) ? *real_roll : *real_roll - 3600000;
    *real_roll = (*real_roll > -1800000) ? *real_roll : *real_roll + 3600000;
}
    
/*
 * FIND YAW - calculates the updated yaw using a rotation matrix
 * @param *real_yaw - a pointer to the current yaw value
 * @return none
 * @see ZYX_MATRIX(y, p, r)
 */
find_yaw(int *real_yaw)
{
    *real_yaw += (atan2f(rotation_matrix[1][0], rotation_matrix[0][0]) * DEG * 
            OFFSET * DT);
    *real_yaw = (*real_yaw <= 1800000) ? *real_yaw : *real_yaw - 3600000;
    *real_yaw = (*real_yaw > -1800000) ? *real_yaw : *real_yaw + 3600000;
}

/*
 * FIND VELOCITY X - tracks the velocity in the x direction of the craft.
 *                  if the craft is level it cannot be moving in the x direction
 *                  so if the pitch is < +- 1 degree it resets the velocity to 0.
 *                  it also compensates for the acceleration of gravity if pitched
 * @param accel_x - the acceleration in the x direction
 * @param real_pitch - the current pitch of the drone
 */
void find_velocity_x(float accel_x, int real_pitch)
{
    float temp = pow(sin(real_pitch * RAD / OFFSET),2);
    
    if(real_pitch < -10000)
        accel_x += temp;
    else if (real_pitch > 10000)
        accel_x -= temp;
    else
        velocity_x = 0;
    
    velocity_x += accel_x * GRAVITY * OFFSET * DT;
}

/*
 * FIND VELOCITY Y - tracks the velocity in the y direction of the craft.
 *                  if the craft is level it cannot be moving in the y direction
 *                  so if the roll is < +- 1 degree it resets the velocity to 0.
 *                  it also compensates for the acceleration of gravity if rolled
 * @param accel_y - the acceleration in the y direction
 * @param real_roll - the current roll of the drone
 */
void find_velocity_y(float accel_y, int real_roll)
{
    float temp = pow(sin(real_roll * RAD / OFFSET),2);

    if(real_roll < -10000)
        accel_y += temp;
    else if(real_roll > 10000)
        accel_y -= temp;
    else
        velocity_y = 0;
    
    velocity_y += accel_y * GRAVITY * OFFSET * DT;
}

/*
 * FIND VELOCITY Z - tracks the velocity in the z direction of the craft.
 *                  it will compensate for gravity based off of the pitch and 
 *                  roll of the craft. If it is level and registering approximately
 *                  1 g it will reset the velocity to 0.
 * @param accel_z - the acceleration in the z direction
 * @param real_pitch - the current pitch of the drone
 * @param real_roll - the current roll of the drone
 */
void find_velocity_z(float accel_z, int real_pitch, int real_roll)
{
    if(abs(real_pitch) < 10000 && abs(real_roll) < 10000 && abs(accel_z) < 1.2 &&
            abs(accel_z) > 0.9)
        velocity_z = 0;
    
    accel_z -= pow(cos(real_pitch * RAD / OFFSET), 2) * pow(cos(real_roll * RAD / OFFSET),2);

    velocity_z += accel_z * GRAVITY * OFFSET * DT;    
}

/*
 * GET SCALE - determines the accelerometer sensitivity setting to be used in 
 *              the complementary filter
 */
void get_scale()
{
    get_accel_scale(&scale);
}

/*
 * FIND ORIENTATION AND VELOCITY - the main logic function for the location tracking
 *                  file. This method is called by the main method and calls all of the
 *                  other methods accept for the find_location_* methods.
 * @param *pitch - pointer to the current gyroscope reading for pitch
 * @param *roll - pointer to the current gyroscope reading for roll
 * @param *yaw - pointer to the current gyroscope reading for yaw
 * @param *real_pitch - pointer to the current value for pitch
 * @param *real_roll - pointer to the current value for roll
 * @param *real_yaw - pointer to the current value for yaw
 * @param *accel_x - pointer to the current accelerometer reading for the x axis
 * @param *accel_y - pointer to the current accelerometer reading for the y axis
 * @param *accel_z - pointer to the current accelerometer reading for the z axis
 */
void find_orientation_and_velocity(float *pitch, float *roll, float *yaw,
        int *real_pitch, int *real_roll, int *real_yaw, float *accel_x,
        float *accel_y, float *accel_z)
{
    float p = *pitch, r = *roll, y = *yaw;
    
    ZYX_MATRIX(y * RAD,p * RAD,r * RAD);
    
    find_roll(real_roll);
    find_yaw(real_yaw);
    find_pitch(real_pitch);
    
    complementary_filter(*accel_x, *accel_y, *accel_z, real_pitch, real_roll,
    real_yaw);
    
    find_velocity_x(*accel_x, *real_pitch);
    find_velocity_y(*accel_y, *real_roll);
    find_velocity_z(*accel_z, *real_pitch, *real_roll);
}