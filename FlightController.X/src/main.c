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
#define SET_HIGH (4800)
#define SET_LOW (2500)

//#define TEST_SENSOR
//#define CALIBRATE

PRIVATE engine_data engine = {{0,0,2500,0.0,1,1,-1},{0,0,2500,0.0,1,-1,1},{0,0,2500,0.0,-1,-1,-1},{0,0,2500,0.0,-1,1,1}};
        static int  E1ON = FALSE, E2ON = FALSE, E3ON = FALSE, E4ON = FALSE;
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

    int counter = ReadTimer2() + 5;
    
    if(counter < 5000)
    {
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
int init_hardware(sensor_data *lsm330)
{
    int rc;

    uint32_t pb_clk = SYSTEMConfig( GetSystemClock(), SYS_CFG_ALL);    
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    INTEnableSystemMultiVectoredInt();

    PORTSetPinsDigitalOut(IOPORT_E, BIT_1 | BIT_2 | BIT_3 | BIT_4);
    PORTE = 0;

    rc = configure_lsm330tr(lsm330);
    if(rc < 0) return -1;

    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, T1_TICK);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_7);
    OpenTimer2(T2_ON | T2_PS_1_32, T2_TICK);
    
    PR1 = 40;            // timer 1 interrupt timing: will interrupt every 1ms
    
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
#endif
    

    
    return 0;
}

void get_attitude(struct data *actual, sensor_data *lsm330)
{
    actual->pitch = atan2f(lsm330->accel_x, lsm330->accel_z);
    actual->roll = atan2f(lsm330->accel_y, lsm330->accel_z);
}

float filter(float input, float array[51])
{
    int i;
    float sum = 0;
    const int BL = 51;
    const float B[51] = {
  0.0004505121978624,  0.00153912705778, 0.003484973853022, 0.005993557189185,
   0.008153435576056, 0.008592952740711,  0.00606548279898,0.0002846755213859,
  -0.007422979046305,  -0.0141538572038, -0.01642740882862, -0.01192407685345,
  -0.001193164875279,  0.01168481560342,  0.02021320609519,  0.01829842335945,
   0.003718245377588, -0.01945112740089, -0.04099016948947, -0.04779531970777,
   -0.02927442676121,  0.01733488350917,  0.08424518003038,   0.1546060172782,
     0.2080043895553,   0.2279136247956,   0.2080043895553,   0.1546060172782,
    0.08424518003038,  0.01733488350917, -0.02927442676121, -0.04779531970777,
   -0.04099016948947, -0.01945112740089, 0.003718245377588,  0.01829842335945,
    0.02021320609519,  0.01168481560342,-0.001193164875279, -0.01192407685345,
   -0.01642740882862,  -0.0141538572038,-0.007422979046305,0.0002846755213859,
    0.00606548279898, 0.008592952740711, 0.008153435576056, 0.005993557189185,
   0.003484973853022,  0.00153912705778,0.0004505121978624
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

//float pitch[100];
//float roll[100];
//int e1[100];
//int e2[100];
//int e3[100];
//int e4[100];
float accel_x[2][100], accel_y[2][100],accel_z[2][100];
float output[4][100];
        // matlab results for pid with values in pitch and roll
//        float matlab[4][37] = {{0,-239.548796030712,-243.480499236680,231.683033424286,
//                -243.487567820150,-7.87283118989602,-486.975135640300,303.652732327824,
//                308.885578491153,314.121566247136,239.511096918869,243.442800124836,
//                -231.720732536130,243.449868708307,7.83513207805294,486.937436528457,
//                -303.690431439667,-308.923277602996,-314.159265358979,-239.548796030712,
//                -243.480499236680,710.780625485710,243.473430653209,-471.238898038469,
//                0.0,0.0,0.0,0.0,239.548796030712,243.480499236680,-710.780625485710,
//                -243.473430653209,471.238898038469,0.0,0.0,0.0,0.0},
//                {0,-239.548796030712,-243.480499236680,710.780625485710,243.473430653209,
//                -471.238898038469,0,0,0,0,239.548796030712,243.480499236680,-710.780625485710,
//                -243.473430653209,471.238898038469,0,0,0,0,-239.548796030712,-243.480499236680,
//                231.683033424286,-243.487567820150,-7.87283118989602,-486.975135640300,
//                303.652732327824,308.885578491153,314.121566247136,239.511096918869,
//                243.442800124836,-231.720732536130,243.449868708307,7.83513207805294,
//                486.937436528457,-303.690431439667,-308.923277602996,-314.159265358979},                
//                {0,239.548796030712,243.480499236680,-231.683033424286,243.487567820150,
//                7.87283118989602,486.975135640300,-303.652732327824,-308.885578491153,
//                -314.121566247136,-239.511096918869,-243.442800124836,231.720732536130,
//                -243.449868708307,-7.83513207805294,-486.937436528457,303.690431439667,
//                308.923277602996,314.159265358979,239.548796030712,243.480499236680,
//                -710.780625485710,-243.473430653209,471.238898038469,0.0,0.0,0.0,0.0,
//                -239.548796030712,-243.480499236680,710.780625485710,243.473430653209,
//                -471.238898038469,0.0,0.0,0.0,0.0},
//                {0,239.548796030712,243.480499236680,-710.780625485710,-243.473430653209,
//                471.238898038469,0,0,0,0,-239.548796030712,-243.480499236680,710.780625485710,
//                243.473430653209,-471.238898038469,0,0,0,0,239.548796030712,243.480499236680,
//                -231.683033424286,243.487567820150,7.87283118989602,486.975135640300,
//                -303.652732327824,-308.885578491153,-314.121566247136,-239.511096918869,
//                -243.442800124836,231.720732536130,-243.449868708307,-7.83513207805294,
//                -486.937436528457,303.690431439667,308.923277602996,314.159265358979}};
   
        float pitch[37] = {0,M_PI/4,M_PI/2,0,0,M_PI/4,M_PI/2,M_PI/3,M_PI/6,0,-M_PI/4,-M_PI/2,0,0,-M_PI/4,-M_PI/2,-M_PI/3,-M_PI/6,0,
                M_PI/4,M_PI/2,0,0,M_PI/4,M_PI/2,M_PI/3,M_PI/6,0,-M_PI/4,-M_PI/2,0,0,-M_PI/4,-M_PI/2,-M_PI/3,-M_PI/6,0};
        float roll[37] = {0,0,0,M_PI/4,M_PI/2,M_PI/4,M_PI/2,M_PI/3,M_PI/6,0,0,0,-M_PI/4,-M_PI/2,-M_PI/4,-M_PI/2,-M_PI/3,-M_PI/6,0,0,
                0,-M_PI/4,-M_PI/2,-M_PI/4,-M_PI/2,-M_PI/3,-M_PI/6,0,0,0,M_PI/4,M_PI/2,M_PI/4,M_PI/2,M_PI/3,M_PI/6,0};
        int count = 0;
        float e1_difference = 0, e2_difference=0, e3_difference=0, e4_difference=0;
        float diff_x[2], diff_y[2], diff_z[2];
        float filtdiff[3], nofiltdiff[3];
        int i;

    float min=10000,max=0;
/*
 * MAIN -initializes the hardware and then loops to keep the program running.
 *      all functionality is interrupt driven.
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
    if(init_hardware(&lsm330) < 0) return(EXIT_SUCCESS);
    location_data location = {{0,0,0},{0,0,0}};
    DELAY(40000000);
    float filter_x[51], filter_y[51], filter_z[51];

    while(engine.e1.speed < 2700)
    {
        engine.e1.speed += 50;
        engine.e2.speed += 50;
        engine.e3.speed += 50;
        engine.e4.speed += 50;
        DELAY(8000000);
    }
    DELAY(100000);

//    while(1)
//    {
//        WriteCoreTimer(0);
//
//        read_accel(&lsm330);
//        get_attitude(&location.actual, &lsm330);
//        pitch[i] = location.actual.pitch;
//        roll[i] = location.actual.roll;
//        pid_control_function(&location, &engine);
//        e1[i] = engine.e1.speed;
//        e2[i] = engine.e2.speed;
//        e3[i] = engine.e3.speed;
//        e4[i] = engine.e4.speed;
//        while(ReadCoreTimer() < 400000){}
//        i = (i<100) ? ++i : 0;
//    }
//    _nop();
    while(i<51)
    {
        WriteCoreTimer(0);
        read_accel(&lsm330);
        filter(lsm330.accel_x, filter_x);
        filter(lsm330.accel_y, filter_y);
        filter(lsm330.accel_z, filter_z);
        i++;
    }
    i = 0;
    while(1)
    {
        WriteCoreTimer(0);
        read_accel(&lsm330);
        lsm330.accel_x = filter(lsm330.accel_x, filter_x) + lsm330.accel_x_zero;
        lsm330.accel_y = filter(lsm330.accel_y, filter_y) + lsm330.accel_y_zero;
        lsm330.accel_z = filter(lsm330.accel_z, filter_z) + lsm330.accel_z_zero;
        location.actual.pitch = atan2f(lsm330.accel_x, lsm330.accel_z);
        location.actual.roll = atan2f(lsm330.accel_y, lsm330.accel_z);
//        location.actual.pitch = -1.414172;//*OFFSET;
//        location.actual.roll = -1.415259;//*OFFSET;
        pid_control_function(&location, &engine);
        output[0][i] = engine.e1.pid_out;
        output[1][i] = engine.e2.pid_out;
        output[2][i] = engine.e3.pid_out;
        output[3][i] = engine.e4.pid_out;
        i++;
        if(i == 100)
        {
//            for(i = 0; i < 100; i++)
//            {
//                float temp = min(output[0][i],output[1][i]);
//                float temp2 = min(output[3][i],output[2][i]);
//                temp = min(temp,temp2);
//                min = min(temp,min);
//                temp = max(output[0][i],output[1][i]);
//                temp2 = max(output[3][i],output[2][i]);
//                temp = max(temp,temp2);
//                max = max(temp,max);        
//            }
            i = 0;
            _nop();
        }
        while(ReadCoreTimer() < 400000){}
    }
    count = ReadCoreTimer();
    

    

    
//    for(i = 0; i < 100; i++)
//    {
//        diff_x[0] += accel_x[0][i];
//        diff_x[1] += accel_x[1][i];
//        diff_y[0] += accel_y[0][i];
//        diff_y[1] += accel_y[1][i];
//        diff_z[0] += 1.0 - accel_z[0][i];
//        diff_z[1] += 1.0 - accel_z[1][i];
//    }
//    nofiltdiff[0] = diff_x[0] / 100.0;
//    nofiltdiff[1] = diff_y[0] / 100.0;
//    nofiltdiff[2] = diff_z[0] / 100.0;
//    filtdiff[0] = diff_x[1] / 100.0;
//    filtdiff[1] = diff_y[1] / 100.0;
//    filtdiff[2] = diff_z[1] / 100.0;
//    
//    for(i = 0;i<37;i++)
//    {
//        float temp;
//        temp = fabs(matlab[0][i] - output[0][i]);///fabs(matlab[0][i]) * 100.0;
//        if(temp != 0)
//        {
//            temp /= fabs(matlab[0][i]);
//            temp *= 100.0;
//        }
//        e1_difference += temp;
//        temp = fabs(matlab[1][i] - output[1][i]);///fabs(matlab[0][i]) * 100.0;
//        if(temp != 0)
//        {
//            temp /= fabs(matlab[1][i]);
//            temp *= 100.0;
//        }
//        e2_difference += temp;
//        temp = fabs(matlab[2][i] - output[2][i]);///fabs(matlab[0][i]) * 100.0;
//        if(temp != 0)
//        {
//            temp /= fabs(matlab[2][i]);
//            temp *= 100.0;
//        }
//        e3_difference += temp;
//        temp = fabs(matlab[3][i] - output[3][i]);///fabs(matlab[0][i]) * 100.0;
//        if(temp != 0)
//        {
//            temp /= fabs(matlab[3][i]);
//            temp *= 100.0;
//        }
//        e4_difference += temp;
//    }
//    e1_difference /= 37.0;
//    e2_difference /= 37.0;
//    e3_difference /= 37.0;
//    e4_difference /= 37.0;
    _nop();


#endif
    return (EXIT_SUCCESS);
}