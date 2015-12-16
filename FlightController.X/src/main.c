/* 
 * File:   main.c
 * Author: Kevin Dederer
 * Comments: main program file to run the base logic
 * Revision history: 
 */

#include "config.h"

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = ON              // USB PLL Enable (Enabled)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))

// DEVCFG0
#pragma config DEBUG = ON              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (ICE EMUC1/EMUD1 pins shared with PGC1/PGD1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// Bit operations to drive or sink current on PWM pins
#define PULSEON() \
{   PORTE = PORTE | 0b1111 << 1; E1ON = TRUE; E2ON = TRUE; E3ON = TRUE; E4ON = TRUE; }\

#define PULSEOFF() \
{   PORTE = PORTE & 0b0000 << 1; E1ON = FALSE; E2ON = FALSE; E3ON = FALSE; E4ON = FALSE; }\

#define PULSEE1OFF() \
{   PORTE = PORTE & 0b1011 << 1; E1ON = FALSE; }\

#define PULSEE2OFF() \
{   PORTE = PORTE & 0b1110 << 1; E2ON = FALSE; }\

#define PULSEE3OFF() \
{   PORTE = PORTE & 0b1101 << 1; E3ON = FALSE; }\

#define PULSEE4OFF() \
{   PORTE = PORTE & 0b0111 << 1; E4ON = FALSE; }\

#define DELAY(x) \
{   int t; for(t = 0;t<x;t++) _nop();} \

typedef struct
{
    float filter_x[51];
    float out_x;
    float filter_y[51];
    float out_y;
    float filter_z[51];
    float out_z;
} filter_table;

enum timer_state
{
    on, off, update
};

#define BASE (0xbd03D000)
volatile int *flash = (int *) BASE;
#define SET_HIGH (4800)
#define SET_LOW (2450)

//#define TEST_SENSOR
//#define CALIBRATE

PRIVATE engine_data engine = {{0,0,2500,0.0,1,1},{0,0,2500,0.0,1,-1},
                    {0,0,2500,0.0,-1,-1},{0,0,2500,0.0,-1,1}};
        
/*
 * __ISR() Timer1Handler() - performs the pulse width modulation functionality for
 *                      the four motors
 * 
 *  variables are static so that they will be remembered for the next interrupt.
 *  
 *  if u_enable is false it is calibrate mode and will only turn on and off the PWM pins
 *  if u_enable is true it allows the sensor to be read as well as location tracking and PID control for each engine
 */
void __ISR(_TIMER_1_VECTOR, IPL7SRS) Timer1Handler(void)
{
    mT1ClearIntFlag();
    static int  E1ON = FALSE, E2ON = FALSE, E3ON = FALSE, E4ON = FALSE;
    int counter = ReadTimer2() + 5;
    
    if(counter < 5000)
    {
#ifdef CALIBRATE
        if(engine.e1.speed < counter)
            PULSEOFF();
#else
        if(E1ON && engine.e1.speed < counter)
            PULSEE1OFF();
        if(E2ON && engine.e2.speed < counter)
            PULSEE2OFF();
        if(E3ON && engine.e3.speed < counter)
            PULSEE3OFF();
        if(E4ON && engine.e4.speed < counter)
            PULSEE4OFF();
#endif
    }
    else
    {
        PULSEON();
        WriteTimer2(0);
    }   
}

/*
 * INIT HARDWARE - initialize and configure the hardware for use
 */
int init_hardware(sensor_data *lsm330)
{
    uint32_t pb_clk = SYSTEMConfig( GetSystemClock(), SYS_CFG_ALL);    
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    INTEnableSystemMultiVectoredInt();

    PORTSetPinsDigitalOut(IOPORT_E, BIT_1 | BIT_2 | BIT_3 | BIT_4);
    PORTE = 0;

    if(configure_lsm330tr(lsm330) < 0) return -1;

    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, T1_TICK);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_7);
    OpenTimer2(T2_ON | T2_PS_1_32, T2_TICK);
    
    PR1 = 40;            // timer 1 interrupt timing    
    
#ifdef CALIBRATE            // if defined will calibrate the speed controllers to 
    engine.e1.speed = SET_HIGH;   // desired range of operation
    engine.e2.speed = SET_HIGH;
    engine.e3.speed = SET_HIGH;
    engine.e4.speed = SET_HIGH;
    DELAY(40000000);
    engine.e1.speed = SET_LOW;
    engine.e2.speed = SET_LOW;
    engine.e3.speed = SET_LOW;
    engine.e4.speed = SET_LOW;
    DELAY(800000);
    #undef CALIBRATE
#endif

    return 0;
}

/*
 * GET ATTITUDE - calculates the pitch and roll of the drone
 * @param *actual - struct containing the actual orientation of the drone
 * @param *lsm330 - struct containing the sensor read outs
 */
void get_attitude(struct data *actual, sensor_data *lsm330)
{
    actual->pitch = atan2f(lsm330->accel_x, lsm330->accel_z);
    actual->roll = atan2f(lsm330->accel_y, lsm330->accel_z);
}

/*
 * filter - passes the sensor data through to eliminate noise from the engines
 * @param input - the sensor reading
 * @param array - the previous readings from the desired axis
 * @return the filtered value for the given axis.
 */
float filter(float input, float array[51])
{
    int i;
    float sum = 0;
    const int BL = 52;
    const float B[52] = {
  -5.259049844545e-06,-7.003930036877e-05,-0.0002192722383055,-0.0005355247742648,
  -0.001106251801568,-0.002029005291553, -0.00338859076657,-0.005229924529025,
  -0.007526104921771, -0.01014756651919,  -0.0128394666295, -0.01521479586034,
   -0.01676963522277, -0.01692404043199, -0.01508771661046, -0.01074440475192,
  -0.003544265753318, 0.006610225397195,  0.01950065554854,  0.03456535328037,
     0.0509181437564,  0.06741588729194,  0.08276960160501,  0.09568589113223,
     0.1050195933373,   0.1099150922913,   0.1099150922913,   0.1050195933373,
    0.09568589113223,  0.08276960160501,  0.06741588729194,   0.0509181437564,
    0.03456535328037,  0.01950065554854, 0.006610225397195,-0.003544265753318,
   -0.01074440475192, -0.01508771661046, -0.01692404043199, -0.01676963522277,
   -0.01521479586034,  -0.0128394666295, -0.01014756651919,-0.007526104921771,
  -0.005229924529025, -0.00338859076657,-0.002029005291553,-0.001106251801568,
  -0.0005355247742648,-0.0002192722383055,-7.003930036877e-05,-5.259049844545e-06
};
    
    for(i = 0; i < BL; i++)
    {
        if(i == BL-1)
        {
            array[0] = input;
        }
        else
        {
            array[BL-(i+1)] = array[BL-(i+2)];
        }
        sum += array[BL-(i+1)] * B[i];
    }
    return sum;
}

/*
 * MAIN -initializes the hardware, configures the software and then loops at 100hz
 *      to read the sensor, determine orientation and call the pid function.
 *      
 */
int main(int argc, char** argv)
{   
#ifdef TEST_SENSOR
    int rc = 0;
    rc += configure_lsm330tr(&lsm330);
    rc += read_accel(&accel_x, &accel_y, &accel_z);
    rc += read_gyro(&pitch, &roll, &yaw);
    if(accel_x > .1 || accel_x < -.1) rc--;
    if(accel_y > .1 || accel_y < -.1) rc--;
    if(accel_z > 1.1 || accel_z < -1.1) rc--;
    if(pitch > .1 || pitch < -.1) rc--;
    if(roll > .1 || roll < -.1) rc--;
    if(yaw > .1 || yaw < -.1) rc--;
    if(rc < 0)
        _nop();
    else
        _nop();
#else
    static sensor_data lsm330;
    location_data location = {{0,0,0},{0,0,0}};
    float filter_x[52], filter_y[52], filter_z[52];

    if(init_hardware(&lsm330) < 0) return(EXIT_SUCCESS);

    
    DELAY(40000000);    // delay to allow all escs to turn on 
    location.user.accel_z = 1.0;
    
    int i;
    for(i = 0; i < 52; i++) // fill fir filter values
    {
        read_accel(&lsm330);
        filter(lsm330.accel_x + lsm330.accel_x_zero, filter_x);
        filter(lsm330.accel_y + lsm330.accel_y_zero, filter_y);
        filter(lsm330.accel_z + lsm330.accel_z_zero, filter_z);        
    }
#define CALIBRATE
    while(engine.e1.speed < 2800)  // engine ramp up
    {
        engine.e1.speed += 50;
        DELAY(800000);
    }
#undef CALIBRATE
    
    while(1)
    {
        WriteCoreTimer(0);
        read_accel(&lsm330);
        lsm330.accel_x = filter(lsm330.accel_x,filter_x) + lsm330.accel_x_zero;
        lsm330.accel_y = filter(lsm330.accel_y,filter_y) + lsm330.accel_y_zero;
        lsm330.accel_z = filter(lsm330.accel_z,filter_z) + lsm330.accel_z_zero;
        location.actual.accel_z = lsm330.accel_z;
        get_attitude(&location.actual,&lsm330);
        pid_control_function(&location, &engine);
        while(ReadCoreTimer() < 400000){}
    }
#endif
    return (EXIT_SUCCESS);
}