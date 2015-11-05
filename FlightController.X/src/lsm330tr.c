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

// Gyroscope Register Addresses
#define LSM330_REG_CTRL1G (0x20)
#define LSM330_REG_CTRL2G (0x21)
#define LSM330_REG_CTRL3G (0x22)
#define LSM330_REG_CTRL4G (0x23)
#define LSM330_REG_CTRL5G (0x24)
#define LSM330_GYRO_FIFO_CTRL (0x2E)
#define LSM330_REG_STATUS_G (0x27)

// Gyroscope Register Values
#define LSM330_WHOAMI_VALG (0b11010100)

// Gyroscope output data rates
#define LSM330_GYRO_DR_95HZ (0b00)
#define LSM330_GYRO_DR_190HZ (0b01)
#define LSM330_GYRO_DR_380HZ (0b10)
#define LSM330_GYRO_DR_760HZ (0b11)

// Gyroscope bandwidth levels
#define LSM330_GYRO_BW_LVL1 (0b00)
#define LSM330_GYRO_BW_LVL2 (0b01)
#define LSM330_GYRO_BW_LVL3 (0b10)
#define LSM330_GYRO_BW_LVL4 (0b11)

// Gyroscope high pass filter mode settings
#define LSM330_GYRO_HPF_NORMAL_RESET (0b00)
#define LSM330_GYRO_HPF_REFERENCE (0b01)
#define LSM330_GYRO_HPF_NORMAL (0b10)
#define LSM330_GYRO_HPF_AUTORESET (0b11)

// Gyroscope high pass filter level settings
#define LSM330_GYRO_HPF_LVL1 (0b0000)
#define LSM330_GYRO_HPF_LVL2 (0b0001)
#define LSM330_GYRO_HPF_LVL3 (0b0010)
#define LSM330_GYRO_HPF_LVL4 (0b0011)
#define LSM330_GYRO_HPF_LVL5 (0b0100)
#define LSM330_GYRO_HPF_LVL6 (0b0101)
#define LSM330_GYRO_HPF_LVL7 (0b0110)
#define LSM330_GYRO_HPF_LVL8 (0b0111)
#define LSM330_GYRO_HPF_LVL9 (0b1000)
#define LSM330_GYRO_HPF_LVL10 (0b1001)

// Gyroscope sensitivity scaling values
#define LSM330_GYRO_SCALE_250DPS (500.0/65536.0) 
#define LSM330_GYRO_SCALE_500DPS (1000.0/65536.0) 
#define LSM330_GYRO_SCALE_2000DPS (4000.0/65536.0) 

// Gyroscope select sensitivity
#define LSM330_GYRO_SETDPS_250DPS (0b00)
#define LSM330_GYRO_SETDPS_500DPS (0b01)
#define LSM330_GYRO_SETDPS_2000DPS (0b10)


// Shared Register values/ addresses


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

// Gyroscope control register 1 structure
typedef union {
    
    struct {
        uint8_t byte;
    };
    
    struct {
        int xen:1;  // x axis enable bit - 1 enabled (default), 0 disabled
        int yen:1;  // y axis enable bit - 1 enabled (default), 0 disabled
        int zen:1;  // z axis enable bit - 1 enabled (default), 0 disabled
        int pd:1;   // power down mode enable - 1 Normal or sleep mode, 0 power down mode
        int bw:2;   // 2 bit bandwith selection - use LSM330_GYRO_BW_LVL1
        int dr:2;   // 2 bit output data rate @see LSM330_GYRO_DR_90HZ
    };

} lsm_reg_ctrl1_g_t;

// Gyroscope control register 2 structure
typedef union {
    
    struct {
        uint8_t byte;
    };
    
    struct {
        int hpcf:4;     // 4 bit high pass filter cutoff frequency @see LSM330_GYRO_HPCF_LVL1
        int hpm:2;      // 2 bit high pass filter mode @see LSM330_GYRO_HPM_NORMAL
        int lvlen:1;    // level sensitive trigger enable - 0 disabled (default), 1 enabled
        int extr:1;     // edge sensitive trigger enable - 0 external trigger disabled (default), 
                        // 1 external trigger enabled
    };

} lsm_reg_ctrl2_g_t;

// Gyroscope control register 3 structure
typedef union {
    
    struct {
        uint8_t byte;
    };
    
    struct {
        int i2_empty:1;     // fifo empty interrupt on int2_G - 1 enabled, 0 disabled (default)
        int i2orun:1;       // fifo overrun interrupt on int2_G - 1 enabled, 0 disabled (default)
        int i2wtm:1;        // watermark enable on int2_G - 1 enabled, 0 disabled (default)
        int i2drdy:1;       // data ready on int2_G - 1 enabled, 0 disabled (default)
        int pp_od:1;        // push/pull or open drain - 1 open drain, 0 push/pull (default)
        int h_lactive:1;    // interrupt active config on int1_G - 1 low, 0 high (default)
        int i1boot:1;       // boot status available on int1_G pin - 1 enabled, 0 disabled (default)
        int i1int1:1;       // interrupt enable on in1_G pin - 1 enabled, 0 disabled (default)
    };

} lsm_reg_ctrl3_g_t;

// Gyroscope control register 4 structure
typedef union {
    
    struct {
        uint8_t byte;
    };
    
    struct {
        int sim:1;      // SPI configuration select - not used with i2c
        int :3;
        int fs:2;       // gyroscope scale selection @see LSM330_GYRO_SETDPS_250DPS
        int ble:1;      // big/little endian select - 1 MSB at lower address, 0 MSB at higher 
                        // address (default)
        int bdu:1;      // block data update - 1 update after LSB and MSB have been read, 
                        // 0 continuous update (default)
    };

} lsm_reg_ctrl4_g_t;

// Gyroscope control register 5 structure
typedef union {
    
    struct {
        uint8_t byte;
    };
    
    struct {
        int out_sel:2;  // out selection configuration
        int int1_sel:2; // INT1 selection configuration
        int hpen:1;     // high pass filter enable bit - 1 enabled, 0 disabled (default)
        int :2;
        int fifoen:1;   // fifo enable bit - 1 enabled, 0 disabled (default)
        int boot:1;     // reboot memory content - 1 reboot memory, 0 Normal mode (default)
    };

} lsm_reg_ctrl5_g_t;

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

lsm_reg_ctrl4_a_t accel_ctrl4;
lsm_reg_ctrl5_a_t accel_ctrl5;
lsm_reg_ctrl6_a_t accel_ctrl6;
lsm_reg_ctrl7_a_t accel_ctrl7;
lsm_reg_fifo_ctrl_t accel_fifo_ctrl;
lsm_reg_fifo_src_a_t accel_fifo_src;
lsm_reg_status_t accel_status;

lsm_reg_ctrl1_g_t gyro_ctrl1;
lsm_reg_ctrl2_g_t gyro_ctrl2;
lsm_reg_ctrl3_g_t gyro_ctrl3;
lsm_reg_ctrl4_g_t gyro_ctrl4;
lsm_reg_ctrl5_g_t gyro_ctrl5;
lsm_reg_fifo_ctrl_t gyro_fifo_ctrl;

double accel_sensitivity, gyro_sensitivity;

/*
 * ATTITUDE CORRECTION - negates the values passed into it to align with desired
 *                  coordinate system.
 * @param value - the value to be negated.
 * @return the negated value; 
 */
float attitude_correction(float value)
{
    return value * -1.0;
}

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
    
    if (byte != LSM330_WHOAMI_VALG) return -1;
}

/*
 * SET ACCEL SENSITIVITY - sets the appropriate value in the accel_sensitivity 
 *                      variable to be applied to the raw sensor data.
 * @param x - the 3 digit integer of the accelerometer scale setting.
 * @return 0 if the setting was set correctly, -1 if nothing registered.
 */
int set_accel_sensitivity(int x)
{
    switch(x)
    {
        case 0:
            accel_sensitivity = LSM330_ACCEL_SCALE_2G;
            break;
        case 1:
            accel_sensitivity = LSM330_ACCEL_SCALE_4G;
            break;
        case 2:
            accel_sensitivity = LSM330_ACCEL_SCALE_6G;
            break;
        case 3:
            accel_sensitivity = LSM330_ACCEL_SCALE_8G;
            break;
        case -4:
            accel_sensitivity = LSM330_ACCEL_SCALE_16G;
            break;
        default:
            return - 1;
            break;
    }
    return 0;
}

/*
 * SET GYRO SENSITIVIYT - sets the appropriate value in the gyro_sensitivity
 *                  variable to be applied to the raw sensor data
 * @param x - the 2 digit integer representation of the current scale setting
 * @return 0 if sensitivity is set correctly, -1 if nothing registered
 */
int set_gyro_sensitivity(int x)
{
    switch(x)
    {
        case 0:
            gyro_sensitivity = LSM330_GYRO_SCALE_250DPS;
            break;
        case 1:
            gyro_sensitivity = LSM330_GYRO_SCALE_500DPS;
            break;
        case 3:
            gyro_sensitivity = LSM330_GYRO_SCALE_2000DPS;
            break;
        default:
            return -1;
            break;
    }
    return 0;
}

/*
 * CONFIGURE ACCEL - sets the configuration registers to the desired specifications
 * @return 0 if configured successfully, -1 if an error has occurred.
 */
int configure_accel(void)
{
    int rc;

    // @see lsm_reg_ctrl4_a_t for details
    accel_ctrl4.byte = 0;
    accel_ctrl4.iea = 1;
    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL4A, accel_ctrl4.byte);
    if (rc < 0) return -1;
   
    // @see lsm_reg_ctrl5_a_t for details
    accel_ctrl5.byte = 0;
    accel_ctrl5.xen = 1;
    accel_ctrl5.yen = 1;
    accel_ctrl5.zen = 1;
    accel_ctrl5.bdu = 1;
    accel_ctrl5.odr = LSM330_ACC_ODR_800HZ;
    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL5A, accel_ctrl5.byte);
    if(rc < 0) return -1;
    
    // @see lsm_reg_ctrl6_a_t for details
    accel_ctrl6.byte = 0;
    accel_ctrl6.bw = LSM33_ACC_BW_200HZ;
    accel_ctrl6.fscale = LSM330_ACC_SETG_8G;
    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL6A, accel_ctrl6.byte);
    if(rc < 0) return -1;
    
    // @see lsm_reg_ctrl7_a_t for details
    accel_ctrl7.byte = 0;
    accel_ctrl7.add_inc = 1;
    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_REG_CTRL7A, accel_ctrl7.byte);
    if(rc < 0) return -1;
    
    // @see lsm_reg_fifo_ctrl_t for details
    accel_fifo_ctrl.byte = 0;
    accel_fifo_ctrl.fmode = LSM330_FIFO_STREAM;
    accel_fifo_ctrl.wtmp = 0b11111;
    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_ACC_FIFO_CTRL, accel_fifo_ctrl.byte);
    if(rc < 0) return -1;
    
    // set offset for x axis accelerometer reading
    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_ACC_OFFX, OFFSETX_A);
    if(rc < 0) return -1;
    
    // set offset for y axis accelerometer reading
    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_ACC_OFFY, OFFSETY_A);
    if(rc < 0) return -1;
    
    // set offset for z axis acceleromter reading
    rc = lsm330_write_reg(LSM330_DEV_ACCEL, LSM330_ACC_OFFZ, OFFSETZ_A);
    if(rc < 0) return -1;

    // set the sensitivity value for the accelerometer
    rc = set_accel_sensitivity(accel_ctrl6.fscale);
    if(rc < 0) return -1;
}

/*
 * CONFIGURE GYRO sets the configuration registers to the desired settings
 * @return 0 if the gyro is successfully configured, -1 if an error occurred.
 */
int configure_gyro(void)
{
    int rc;
    
    // @see lsm330_reg_ctrl1_g_t for specifications
    gyro_ctrl1.byte = 0;
    gyro_ctrl1.xen = 1;
    gyro_ctrl1.yen = 1;
    gyro_ctrl1.zen = 1;
    gyro_ctrl1.pd = 1;
    gyro_ctrl1.dr = LSM330_GYRO_DR_760HZ;
    gyro_ctrl1.bw = LSM330_GYRO_BW_LVL4;
    rc = lsm330_write_reg(LSM330_DEV_GYRO, LSM330_REG_CTRL1G, gyro_ctrl1.byte);
    if(rc < 0) return -1;    

    // @see lsm330_reg_ctrl2_g_t for specifications
    gyro_ctrl2.byte = 0;
    gyro_ctrl2.hpcf = LSM330_GYRO_HPF_LVL10;
    gyro_ctrl2.hpm = LSM330_GYRO_HPF_NORMAL_RESET;
    gyro_ctrl2.lvlen = 1;
    rc = lsm330_write_reg(LSM330_DEV_GYRO, LSM330_REG_CTRL2G, gyro_ctrl2.byte);
    if(rc < 0) return -1;    
    
    // @see lsm330_reg_ctrl3_g_t for specifications    
    gyro_ctrl3.byte = 0;
    gyro_ctrl3.pp_od = 1;
    rc = lsm330_write_reg(LSM330_DEV_GYRO, LSM330_REG_CTRL3G, gyro_ctrl3.byte);
    if(rc < 0) return -1;    

    // @see lsm330_reg_ctrl4_g_t for specifications
    gyro_ctrl4.byte = 0;
    gyro_ctrl4.bdu = 1;
    gyro_ctrl4.fs = LSM330_GYRO_SETDPS_250DPS;
    rc = lsm330_write_reg(LSM330_DEV_GYRO, LSM330_REG_CTRL4G, gyro_ctrl4.byte);
    if(rc < 0) return -1;    
    
    // @see lsm330_reg_ctrl5_g_t for specifications
    gyro_ctrl5.byte = 0;
    gyro_ctrl5.int1_sel = 0b11;
    gyro_ctrl5.out_sel = 0b01;
    gyro_ctrl5.hpen = 0;
    rc = lsm330_write_reg(LSM330_DEV_GYRO, LSM330_REG_CTRL5G, gyro_ctrl5.byte);
    if(rc < 0) return -1;    

    // @see lsm330_reg_fifo_ctrl_t for specifications
    gyro_fifo_ctrl.byte = 0;
    gyro_fifo_ctrl.fmode = LSM330_FIFO_FIFO;
    gyro_fifo_ctrl.wtmp = 0b11111;
    rc = lsm330_write_reg(LSM330_DEV_GYRO, LSM330_GYRO_FIFO_CTRL, gyro_fifo_ctrl.byte);
    if(rc < 0) return -1;

    // set the sensitivity value for the gyroscope
    rc = set_gyro_sensitivity(gyro_ctrl4.fs);
    if(rc < 0) return -1;
}

/*
 * SOFT RESET - reset the device to ensure a consistent starting point 
 * @return 0 if operating correctly, -1 if an error occurs
 */
int soft_reset( )
{
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
 * CONFIGURE LSM330TR - callable function by main to call for a new configuration
 * @return 0 if the device is configured successfully, -1 if a failure occurs.
 */
int configure_lsm330tr()
{
    int rc = 0;
    
    rc = check_who_ami();
    if (rc < 0) return;
    
    rc = soft_reset();
    if(rc < 0) return -1;
    
    rc = configure_accel();
    if(rc < 0) return -1;
    
    rc = configure_gyro();
    if(rc < 0) return -1;
    
    return 0;
}

/*
 * CONFIGURE LSM330TR TEST - configures the sensor to be validated
 * @return 0 if the device is configured successfully, -1 if a failure occurs.
 */
int configure_lsm330tr_test()
{
    int rc = 0;
    
    rc = check_who_ami();
    if (rc < 0) return;
    
    rc = soft_reset();
    if(rc < 0) return -1;

    gyro_ctrl1.byte = 0xFF;
    rc = lsm330_write_reg(LSM330_DEV_GYRO, LSM330_REG_CTRL1G, gyro_ctrl1.byte);
    if(rc < 0) return -1;
    gyro_ctrl4.byte = 0;
    gyro_ctrl4.bdu = 1;
    gyro_ctrl4.fs = LSM330_GYRO_SETDPS_250DPS;
    rc = lsm330_write_reg(LSM330_DEV_GYRO, LSM330_REG_CTRL4G, gyro_ctrl4.byte);
    if(rc < 0) return -1; 
    
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
    
    rc = set_accel_sensitivity(accel_ctrl6.fscale);
    if(rc < 0) return -1;
    
    rc = set_gyro_sensitivity(gyro_ctrl4.fs);
    if(rc < 0) return -1;
    
    return 0;
}

/*
 * READ ACCEL - performs the register reads on the accelerometer, combines the 
 *              high and low values and multiplies the result by the sensitivity
 * @param *accel_x - pointer to the accel_x address in main
 * @param *accel_y - pointer to the accel_y address in main
 * @param *accel_z - pointer to the accel_z address in main
 * @return 0 if all reads were successfully completed, -1 if a failure occurs.
 */
int read_accel( float *accel_x, float *accel_y, float *accel_z)
{
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
    
    ival = ((int16_t) buff[1]) << 8 | (uint16_t) buff[0];
    *accel_x = (float) ival * accel_sensitivity;
    
    ival = ((int16_t) buff[3]) << 8 | (uint16_t) buff[2];
    *accel_y = (float) ival * accel_sensitivity;
    
    ival = ((int16_t) buff[5]) << 8 | (uint16_t) buff[4];
    *accel_z = (float) ival * accel_sensitivity;
    
    *accel_y = attitude_correction(*accel_y);

    return 0;
}

/*
 * READ GYRO - performs the register reads for gyroscope output, combines the
 *              high and low values and multiplies it by the sensitivity. 
 * @param *pitch - pointer to the pitch address in main
 * @param *roll - pointer to the roll address in main
 * @param *yaw - pointer to the yaw address in main
 * @return 0 if all reads were successfully completed, -1 if a failure occurs.
 */
int read_gyro( float *pitch, float *roll, float *yaw)
{
    int rc;
    uint8_t buff[6] = {0};
    int16_t ival;
 
    lsm_reg_status_t gyro_status;

    while (1) {
        rc = lsm330_read_reg(LSM330_DEV_GYRO, LSM330_REG_STATUS_G, &gyro_status.byte);
        if (rc < 0) return -1;

        if (!gyro_status.zyxda) continue;
        if (!gyro_status.xda) continue;
        if (!gyro_status.yda) continue;
        if (!gyro_status.zda) continue;
        break;
    }
    
    rc = lsm330_read_multiple_reg(LSM330_DEV_GYRO, LSM330_REG_OUT_MULTIPLE, buff);
    if(rc < 0) return -1;
    
    ival = (((int16_t) buff[1]) << 8 | (uint16_t) buff[0]);
    *roll = (float) ival * gyro_sensitivity;
    
    ival = (((int16_t) buff[3]) << 8 | (uint16_t) buff[2]);
    *pitch = (float) ival * gyro_sensitivity;
    
    ival = (((int16_t) buff[5]) << 8 | (uint16_t) buff[4]);
    *yaw = (float) ival * gyro_sensitivity;
    
    *pitch = attitude_correction(*pitch);
    
    return 0;
}

/*
 * GET ACCEL SCALE - passes the accelerometer setting to location_tracking.c
 * @param *scale - pointer to the scale variable in location_tracking.c
 */
void get_accel_scale(int *scale)
{
    int x = accel_ctrl6.fscale;
    
    switch(x)
    {
        case 0:
            *scale = 2;
            break;
        case 1:
            *scale = 4;
            break;
        case 2:
            *scale = 6;
            break;
        case 3:
            *scale = 8;
            break;
        case -4:
            *scale = 16;
            break;
        default:
            *scale = 2;
            break;
    }
}