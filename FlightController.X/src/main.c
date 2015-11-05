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

#define PULSEON() \
{   PORTE = 0b1111 << 1; E1ON = TRUE; E2ON = TRUE; E3ON = TRUE; E4ON = TRUE; }\

#define PULSEE2OFF() \
{   PORTE = PORTE & 0b1110 << 1; E2ON = FALSE; }\

#define PULSEE3OFF() \
{   PORTE = PORTE & 0b1011 << 1; E3ON = FALSE; }\

#define PULSEE1OFF() \
{   PORTE = PORTE & 0b1101 << 1; E1ON = FALSE; }\

#define PULSEE1ON() \
{   PORTE = PORTE | 0b0010 << 1; E1ON = TRUE; }\

#define PULSEE4OFF() \
{   PORTE = PORTE & 0b0111 << 1; E4ON = FALSE; }\

#define DELAY(x) \
{   int t; for(t = 0;t<x;t++) _nop();} \

//#define TEST_SENSOR

PRIVATE float accel_x, accel_y, accel_z, pitch, roll, yaw;

PRIVATE int timer_counter = 0, E1ON, E2ON = 0, E3ON = 0, E4ON = 0, 
        e1_pulse_time, e2_pulse_time, e3_pulse_time, e4_pulse_time, 
        real_pitch = 0, real_roll = 0, real_yaw = 0, location_x = 0, 
        location_y = 0, location_z = 0;

/*
 * __ISR() Timer1Handler() - performs the pulse width modulation functionality for
 *                      the four motors
 */
void __ISR(_TIMER_1_VECTOR, IPL2AUTO) Timer1Handler(void)
{
    mT1ClearIntFlag();
    TMR1 = 0x00;    

    if(timer_counter == 118)
    {
                PULSEON();
                timer_counter = 0;
    }
    else
    {
        switch(E1ON)
        {
            case FALSE:
                break;
            case TRUE:
                if(e1_pulse_time < timer_counter)
                    PULSEE1OFF();
                break;
        }
        switch(E2ON)
        {
            case FALSE:
                break;
            case TRUE:
                if(e2_pulse_time < timer_counter)
                    PULSEE2OFF();
                break;
        }
        switch(E3ON)
        {
            case FALSE:
                break;
            case TRUE:
                if(e3_pulse_time < timer_counter)
                    PULSEE3OFF();
                break;
        }
        switch(E4ON)
        {
            case FALSE:
                break;
            case TRUE:
                if(e4_pulse_time < timer_counter)
                    PULSEE4OFF();
                break;
        }
    }
    timer_counter++;
}

/* 
 * __ISR() Timer2Handler(void) - performs the i2c bus functionality to read the sensors
 *                  as well as call the tracking functions with the updated data.
 */
void __ISR(_TIMER_2_VECTOR, IPL3AUTO) Timer2Handler(void)
{     
    mT2ClearIntFlag();
    TMR2 = 0x00;
    int rc;

    rc = read_accel(&accel_x, &accel_y, &accel_z);
    if (rc < 0) return;

    rc = read_gyro(&pitch, &roll, &yaw);
    if (rc < 0) return;

    find_orientation_and_velocity(&pitch, &roll, &yaw, &real_pitch, 
                    &real_roll, &real_yaw, &accel_x, &accel_y, &accel_z);

    location_x += find_location_x(real_pitch, real_roll, real_yaw);
    location_y += find_location_y(real_pitch, real_roll, real_yaw);
    location_z += find_location_z(real_pitch, real_roll, real_yaw);
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

    rc = configure_lsm330tr();
    if(rc < 0) return -1;
    
    get_scale();
//    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, T1_TICK);
//    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, T2_TICK);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_3);

//    PR1 = 6;
    PR2 = 630;
   
    return 0;
}

/*
 * MAIN - continually runs the PID controllers to ensure the proper
 *      engine speed for the four motors
 */
int main(int argc, char** argv)
{   
    int rc = 0, set_roll = 0, set_yaw = 0, set_pitch = 0, set_x = 0, set_y = 0,
            set_z = 15000;
    
#ifdef TEST_SENSOR
    rc += configure_lsm330tr_test();
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
    
    rc = init_hardware();
    if(rc < 0) return(EXIT_SUCCESS);
        
    while(1) 
    { 
        pid_control_function(&real_pitch, &real_roll, &real_yaw, &set_pitch,
                &set_roll, &set_yaw, &location_x, &location_y, &location_z,
                &set_x, &set_y, &set_z, &e1_pulse_time, &e2_pulse_time, 
                &e3_pulse_time, &e4_pulse_time);
    }
#endif
    return (EXIT_SUCCESS);

}