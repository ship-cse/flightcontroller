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

#define PULSEE2ON() \
{   PORTE = PORTE | 0b0001 << 1; E2ON = TRUE; }\

#define PULSEE1ON() \
{   PORTE = PORTE | 0b0100 << 1; E1ON = TRUE; }\

#define DELAY(x) \
{   int t; for(t = 0;t<x;t++) _nop();} \

enum timer_state
{
    on, off, update
};

#define SET_500HZ (630)
#define SET_400HZ (788)
#define SET_100HZ (315)
#define SET_HIGH (292)
#define SET_LOW (0)
#define DEG (180.0 / M_PI)  // conversion from radians to degrees

//#define TEST_SENSOR
//#define CALIBRATE

PRIVATE int calibrate, u_enable = FALSE;
PRIVATE engine_data engine = {{0,0,2500,2500,0.0,1,1,-1},{0,0,2500,2500,0.0,1,-1,1},{0,0,2500,2500,0.0,-1,-1,-1},{0,0,2500,2500,0.0,-1,1,1}};
        static int  E1ON = FALSE, E2ON = FALSE, E3ON = FALSE, E4ON = FALSE, timer = 0;
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

    int counter = ReadTimer2();
    
    if(counter < 5000)
    {
        counter += 5;
        if(E1ON && engine.e1.speed < counter)
            PULSEE1OFF();
        if(E2ON && engine.e2.speed < counter)
            PULSEE2OFF();
        if(E3ON && engine.e3.speed < counter)
            PULSEE3OFF();
        if(E4ON && engine.e4.speed < counter)
            PULSEE4OFF();
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
int init_hardware()
{
    int rc;

    uint32_t pb_clk = SYSTEMConfig( GetSystemClock(), SYS_CFG_ALL);    
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    INTEnableSystemMultiVectoredInt();

    PORTSetPinsDigitalOut(IOPORT_E, BIT_1 | BIT_2 | BIT_3 | BIT_4);
    PORTE = 0;

    rc = configure_lsm330tr(0);
    if(rc < 0) return -1;

//    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, T1_TICK);
//    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_7);
//    OpenTimer2(T2_ON | T2_PS_1_32, T2_TICK);
    
    PR1 = 5;            // timer 1 interrupt timing: will interrupt every 1ms
    
#ifdef CALIBRATE            // if defined will calibrate the speed controllers to 
    calibrate = SET_HIGH;   // desired range of operation
    DELAY(40000000);
    calibrate = SET_LOW;
    DELAY(800000);
#endif
    DELAY(40000000);
//    calibrate = 20;               // ramp up code for debugging
//    while(calibrate < 130)
//    {
//        calibrate += 10;
//        DELAY(8000000);
//    }
    
    u_enable = TRUE;           // enables reading the sensor, tracking location and PID controls
    return 0;
}

void get_attitude(struct data *actual, sensor_data *lsm330)
{
    actual->pitch = atan2f(lsm330->accel_x, lsm330->accel_z);
    actual->roll = atan2f(lsm330->accel_y, lsm330->accel_z);
}
        
/*
 * MAIN -initializes the hardware and then loops to keep the program running.
 *      all functionality is interrupt driven.
 */
int main(int argc, char** argv)
{   
#ifdef TEST_SENSOR
    int rc = 0;
    rc += configure_lsm330tr(1);
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

    if(init_hardware() < 0) return(EXIT_SUCCESS);
    static sensor_data lsm330;
    location_data location = {{0,0,0,0,0,0},{0,0,0,0,0,0}};
    
    while(1)
    {
        WriteCoreTimer(0);

        read_accel(&lsm330);
        get_attitude(&location.actual, &lsm330);
        pid_control_function(&location, &engine);
        while(ReadCoreTimer() < 400000){}
    }
#endif
    return (EXIT_SUCCESS);
}