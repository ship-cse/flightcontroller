/* 
 * File:   lsm330tr.c
 * Author: Kevin Dederer
 * Comments: logic file for the lsm330tr sensor, includes register address 
 *              definitions, configuration bit definitions, register structs,
 *              and the main logic for configuring and reading the sensor.
 * Revision history: 
 */

#include "config.h"

// Device Addresses
#define LSM330_DEV_ACCEL (0b0011110)
#define LSM330_DEV_GYRO  (0b1101010)

//Accelerometer Register Addresses
#define LSM330_REG_CTRL4A (0x23)
#define LSM330_REG_CTRL5A (0x20)
#define LSM330_REG_CTRL6A (0x24)
#define LSM330_REG_CTRL7A (0x25)
#define LSM330_REG_OUTS1A (0x5F)
#define LSM330_ACC_FIFO_CTRL (0x2E)
#define LSM330_ACC_FIFO_SRC (0x2F)
#define LSM330_ACC_OFFX (0x10)
#define LSM330_ACC_OFFY (0x11)
#define LSM330_ACC_OFFZ (0x12)
#define LSM330_REG_STATUS_A (0x27)

//Accelerometer Register Values
// who am I used for verifying connection
#define LSM330_WHOAMI_VALA (0b01000000)

// Output data rates
#define LSM330_ACC_ODR_OFF    (0b0000)
#define LSM330_ACC_ODR_3HZ    (0b0001)
#define LSM330_ACC_ODR_6HZ    (0b0010)
#define LSM330_ACC_ODR_12HZ   (0b0011)
#define LSM330_ACC_ODR_25HZ   (0b0100)
#define LSM330_ACC_ODR_50HZ   (0b0101)
#define LSM330_ACC_ODR_100HZ  (0b0110)
#define LSM330_ACC_ODR_400HZ   (0b0111)
#define LSM330_ACC_ODR_800HZ   (0b1000)
#define LSM330_ACC_ODR_1600HZ  (0b1001)

// Sensitivity scaling values
#define LSM330_ACCEL_SCALE_2G (4.0/65536.0) 
#define LSM330_ACCEL_SCALE_4G (8.0/65536.0) 
#define LSM330_ACCEL_SCALE_6G (12.0/65536.0)
#define LSM330_ACCEL_SCALE_8G (16.0/65536.0)
#define LSM330_ACCEL_SCALE_16G (32.0/65536.0)

// Bandwith frequency
#define LSM33_ACC_BW_50HZ (0b11)
#define LSM33_ACC_BW_200HZ (0b10)
#define LSM33_ACC_BW_400HZ (0b01)
#define LSM33_ACC_BW_800HZ (0b00)

// Sensitivity setting
#define LSM330_ACC_SETG_2G (0b000)
#define LSM330_ACC_SETG_4G (0b001)
#define LSM330_ACC_SETG_6G (0b010)
#define LSM330_ACC_SETG_8G (0b011)
#define LSM330_ACC_SETG_16G (0b100)

// Axis offset values
#define OFFSETX_A (0x03)
#define OFFSETY_A (0x04)
#define OFFSETZ_A (0x03)

// Output registers X, Y, Z
#define LSM330_REG_OUT_X_L (0x28)
#define LSM330_REG_OUT_X_H (0x29)
#define LSM330_REG_OUT_Y_L (0x2A)
#define LSM330_REG_OUT_Y_H (0x2B)
#define LSM330_REG_OUT_Z_L (0x2C)
#define LSM330_REG_OUT_Z_H (0x2D)

// read all outputs register address
#define LSM330_REG_OUT_MULTIPLE (0xA8)

// who am I register address
#define LSM330_REG_WHOAMI (0x0f)

// FIFO setting values
#define LSM330_FIFO_BYPASS (0b000)
#define LSM330_FIFO_FIFO (0b001)
#define LSM330_FIFO_STREAM (0b010)
#define LSM330_FIFO_STREAM_TO_FIFO (0b011)
#define LSM330_FIFO_BYPASS_TO_STREAM (0b100)

/*
 * ATTITUDE CORRECTION - negates the values passed into it to align with desired
 *                  coordinate system.
 * @param value - the value to be negated.
 * @return the negated value; 
 */
#define ATTITUDE_CORRECTION(value) \
{ value = value * -1; \
} \

// accelerometer control register 4 structure
typedef union {
    struct {
        uint8_t byte;
    };
    struct {
        int strt:1;     // soft reset bit - 1 enabled, 0 disabled (default)
        int :1;     
        int vfilt:1;    // vector filter enable bit - 1 enabled, 0 disabled (default)
        int int1_en:1;  // interrupt on int1_A pin - 1 enabled, 0 disabled (default)
        int int2_en:1;  // interrupt on int2_a pin - 1 enabled, 0 disabled (default)
        int iel:1;      // interrupt signal latching - 1 interrupt signal pulsed, 
                        //0 - interrupt signal latched (default)
        int iea:1;      // interrupt signal polarity - 1 interrupt active high,
                        //0 interrupt active low (default)
        int dren:1;     // DRDY signal enable on int1_A - 1 DRDY signal routed to 
                        //int1_A, 0 DRDY signal disabled (default)
    };
} lsm_reg_ctrl4_a_t;

// accelerometer control register 5 structure
typedef union {
    struct {
        uint8_t byte;
    };
    
    struct {
        int xen:1;  // enable x axis readings bit - 1 enabled (default), 0 disabled
        int yen:1;  // enable y axis readings bit - 1 enabled (default), 0 disabled
        int zen:1;  // enable z axis readings bit - 1 enabled (defautl), 0 disabled
        int bdu:1;  // block data update bit - 1 output registers not updated until 
                    // both LSB and MSB are read, 0 continuous update (default)
        int odr:4;  // 4 bit output data ready setting. @see LSM_ACCEL_ODR_100HZ 
    };
} lsm_reg_ctrl5_a_t;

// accelerometer control register 6 structure
typedef union {
    struct {
        uint8_t byte;
    };
    struct {
        int sim:1;      // SPI mode selection - ignore for i2c use
        int :2;         
        int fscale:3;   // 3 bit setting for sensitivity. @see LSM330_ACCEL_SCALE_2G
        int bw:2;       // 2 bit anti-aliasing filter bandwidth @see LSM330_ACC_BW_50HZ
    };
} lsm_reg_ctrl6_a_t;

// accelerometer control register 7 structure
typedef union {
    struct {
        uint8_t byte;
    };
    struct {
        int boot_int:1;     // boot interrupt on int2_A - 1 enabled, 0 disabled (default)
        int p1_overrun:1;   // fifo overrun indicator on int1_A - 1 enabled, 0 disabled (default)
        int p1_wtm:1;       // fifo watermark level reached indicator on int1_A 
                            // 1 enabled,  0 disabled (default)
        int p1_empty:1;     // fifo empty indicator on int1_A - 1 enabled, 0 disabled (default)
        int add_inc:1;      // auto increment address for multiple register reads
                            // 1 enabled 0 disabled (default)
        int wtm_en:1;       // fifo watermark enable bit - 1 enabled, 0 disabled (default)
        int fifo_en:1;      // fifo enable bit - 1 enabled, 0 disabled (default)
        int boot:1;         // force reboot - 1 Active cleared when finished, 0 (default)
    };
} lsm_reg_ctrl7_a_t;

// shared fifo control register structure
typedef union {
    struct {
        uint8_t byte;
    };
    struct {
        int wtmp:5;     // 5 bit watermark level setting
        int fmode:3;    // 3 bit fifo mode setting @see LSM330_FIFO_BYPASS
    };
} lsm_reg_fifo_ctrl_t;

// accelerometer fifo source register structure
typedef union {
    struct {
        uint8_t byte;
    };
    struct {
        int fss:5;          // fifo stored data level register
        int empty:1;        // fifo empty bit: 1 fifo empty, 0 fifo not empty
        int ovrn_fifo:1;    // fifo overrun indicator: 1 fifo overrun, 0 fifo not overrun
        int wtm:1;          // watermark status bit: 1 watermark level reached, 0 watermark not reached
    };
} lsm_reg_fifo_src_a_t;

// shared status register structure
typedef union {
    struct {
        uint8_t byte;
    };
    struct {
        int xda:1;      // data ready on x
        int yda:1;      // data ready on y
        int zda:1;      // data ready on z
        int zyxda:1;    // data ready on zyx
        int xor:1;      // data overrun on x
        int yor:1;      // data overrun on y
        int zor:1;      // data overrun on z
        int zyxor:1;    // data overrun on zyx
    };
} lsm_reg_status_t;

PRIVATE float accel_sensitivity, gyro_sensitivity;
PRIVATE uint8_t accel_scale;

/*
 * CHECK WHO AMI - verifies that the i2c communication is working with the sensor
 * @return 0 if working properly, -1 if an error has occurred.
 */
int check_who_ami( )
{
    int rc;
    uint8_t byte;
          
    rc = lsm330_read_reg(LSM330_DEV_ACCEL, LSM330_REG_WHOAMI, &byte);
    if (rc < 0) return -1;
    
    if (byte != LSM330_WHOAMI_VALA) return -1;
    
    rc = lsm330_read_reg(LSM330_DEV_GYRO, LSM330_REG_WHOAMI, &byte);
    if (rc < 0) return -1;
}

enum accel_sensitivity_level
{
    G2 = LSM330_ACC_SETG_2G,
    G4 = LSM330_ACC_SETG_4G, 
    G6 = LSM330_ACC_SETG_6G, 
    G8 = LSM330_ACC_SETG_8G, 
    G16 = LSM330_ACC_SETG_16G,
}; 

/*
 * SET ACCEL SENSITIVITY - sets the appropriate value in the accel_sensitivity 
 *                      variable to be applied to the raw sensor data.
 * @param x - the 3 digit integer of the accelerometer scale setting.
 * @return 0 if the setting was set correctly, -1 if nothing registered.
 */
set_accel_sensitivity(uint8_t sensitivity)
{
    switch(sensitivity)
    {
        case G2:
            accel_sensitivity = LSM330_ACCEL_SCALE_2G;
            accel_scale = 2;
            break;
        case G4:
            accel_sensitivity = LSM330_ACCEL_SCALE_4G;
            accel_scale = 4;
            break;
        case G6:
            accel_sensitivity = LSM330_ACCEL_SCALE_6G;
            accel_scale = 6;
            break;
        case G8:
            accel_sensitivity = LSM330_ACCEL_SCALE_8G;
            accel_scale = 8;
            break;
        case G16:
            accel_sensitivity = LSM330_ACCEL_SCALE_16G;
            accel_scale = 16;
            break;
        default:
            accel_sensitivity = LSM330_ACCEL_SCALE_2G;
            break;
    }
}

/*
 * SOFT RESET - reset the device to ensure a consistent starting point 
 * @return 0 if operating correctly, -1 if an error occurs
 */
int soft_reset()
{
    lsm_reg_ctrl4_a_t accel_ctrl4;
    accel_ctrl4.byte = 0;
    accel_ctrl4.strt = 1;
    
    int rc = 0;

    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL4A, accel_ctrl4.byte);
    if (rc < 0) return -1;
    
    WriteCoreTimer(0);
    while(ReadCoreTimer() < ( 40e6));

    return 0;
}

/*
 * READ ACCEL - performs the register reads on the accelerometer, combines the 
 *              high and low values and multiplies the result by the sensitivity
 * @param *lsm330 - pointer to the struct containing the variables for acceleration
 *                  on all 3 axes.
 * @return 0 if all reads were successfully completed, -1 if a failure occurs.
 */
int read_accel(sensor_data *lsm330)
{
    lsm_reg_status_t accel_status;
    int rc;
    uint8_t buff[6] = {0};
    int16_t ival;    

    while (1) {
        rc = lsm330_read_reg(LSM330_DEV_ACCEL, LSM330_REG_STATUS_A, &accel_status.byte);
        if (rc < 0) return -1;

        if (!accel_status.zyxda) continue;
        if (!accel_status.xda) continue;
        if (!accel_status.yda) continue;
        if (!accel_status.zda) continue;
        break;
    }

    rc = lsm330_read_multiple_reg(LSM330_DEV_ACCEL, LSM330_REG_OUT_MULTIPLE, buff);
    if(rc < 0) return -1;
    
    ival = (((int16_t) buff[1]) << 8 | (uint16_t) buff[0]);
    lsm330->accel_x = ival * accel_sensitivity;
    
    ival = (((int16_t) buff[3]) << 8 | (uint16_t) buff[2]);
    lsm330->accel_y = ival * accel_sensitivity;
    
    ival = ((int16_t) buff[5]) << 8 | (uint16_t) buff[4];
    lsm330->accel_z = ival * accel_sensitivity;
    
    return 0;
}

/*
 * SET ZERO OFFSET - reads the sensor 100 times while level and calculates the
 *              average to be added to each reading while operating.
 * @param *lsm330 - pointer to the lsm330 struct to read the sensor and set the
 *                  zero offset variables.
 */
void set_zero_offset(sensor_data *lsm330)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int i;
    
    for(i = 0; i < 100; i++)
    {
        read_accel(lsm330);
        sum_x += lsm330->accel_x;
        sum_y += lsm330->accel_y;
        sum_z += lsm330->accel_z;
    }
    lsm330->accel_x_zero = 0.0 - (sum_x / 100.0);
    lsm330->accel_y_zero = 0.0 - (sum_y / 100.0);
    lsm330->accel_z_zero = 1.0 - (sum_z / 100.0);    
}


/*
 * CONFIGURE LSM330TR - callable function by main to call for a new configuration
 * @return 0 if the device is configured successfully, -1 if a failure occurs.
 */
int configure_lsm330tr(sensor_data *lsm330)
{
    lsm_reg_ctrl4_a_t accel_ctrl4;
    lsm_reg_ctrl5_a_t accel_ctrl5;
    lsm_reg_ctrl6_a_t accel_ctrl6;
    lsm_reg_ctrl7_a_t accel_ctrl7;
    lsm_reg_fifo_ctrl_t accel_fifo_ctrl;
    
    if(check_who_ami() < 0) return;
    
    if(soft_reset() < 0) return -1;
    
#ifndef TEST
        // @see lsm_reg_ctrl4_a_t for details
        accel_ctrl4.byte = 0;
        accel_ctrl4.iea = 1;    // interrupt level high since pulled down
        
        // @see lsm_reg_ctrl5_a_t for details
        accel_ctrl5.byte = 0;
        accel_ctrl5.xen = 1;    // enable x accelerometer readings
        accel_ctrl5.yen = 1;    // enable y accelerometer readings
        accel_ctrl5.zen = 1;    // enable z accelerometer readings
        accel_ctrl5.bdu = 1;    // wait to update until low and high registers are read
        accel_ctrl5.odr = LSM330_ACC_ODR_100HZ;  // closest output data rate to 500hz
        
        // @see lsm_reg_ctrl6_a_t for details
        accel_ctrl6.byte = 0;
        accel_ctrl6.bw = LSM33_ACC_BW_200HZ;        // set bandwith filter frequency - best results
        accel_ctrl6.fscale = LSM330_ACC_SETG_8G;    // set the sensitivity, 8G was efficient
        
        // @see lsm_reg_ctrl7_a_t for details
        accel_ctrl7.byte = 0;
        accel_ctrl7.add_inc = 1;    // enable auto-increment. allows multiple reads without 
                                    // resending read commands
        
        // @see lsm_reg_fifo_ctrl_t for details
        accel_fifo_ctrl.byte = 0;

        if(lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL4A, accel_ctrl4.byte) < 0) return -1;
        if(lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL5A, accel_ctrl5.byte) < 0) return -1;
        if(lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL6A, accel_ctrl6.byte) < 0) return -1;
        if(lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL7A, accel_ctrl7.byte) < 0) return -1;
        if(lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_ACC_FIFO_CTRL, accel_fifo_ctrl.byte) < 0) return -1;
    
#else
        accel_ctrl5.byte = 0x8F;
        rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL5A, accel_ctrl5.byte);
        if(rc < 0) return -1;
        accel_ctrl6.byte = 0;
        accel_ctrl6.bw = LSM33_ACC_BW_200HZ;
        accel_ctrl6.fscale = LSM330_ACC_SETG_8G;
        rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL6A, accel_ctrl6.byte);
        if(rc < 0) return -1;
        accel_ctrl7.byte = 0;
        accel_ctrl7.add_inc = 1;
        rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL7A, accel_ctrl7.byte);
        if(rc < 0) return -1;
#endif    
        
    set_accel_sensitivity(accel_ctrl6.fscale);   
   
    set_zero_offset(lsm330);
    
    return 0;
}