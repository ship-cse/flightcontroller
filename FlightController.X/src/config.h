/* 
 * File:   config.h
 * Author: tbriggs, Kevin Dederer
 * Comments: main configuration file, includes all necessary "#include" files
 *           for the project.
 * Revision history: 
 */

#ifndef CONFIG_H
#define	CONFIG_H


#ifdef	__cplusplus
extern "C" {
#endif
    
//TODO remove extraneous .h references
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <p32xxxx.h>
#include <xc.h>
#include <plib.h>
#include <peripheral/system.h>
#include <errno.h>
#include <sys/appio.h>
#include <math.h>
#include "i2c.h"
#include "lsm330tr.h"   
#include "location_tracking.h"
#include "pid.h"


#define SYS_FREQ (80000000L)
#define PB_DIV 8
#define PRESCALE 256
#define TOGGLES_PER_SEC 1
#define T1_TICK (SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define T2_TICK (SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define	GetSystemClock()      (80000000ul)
#define	GetPeripheralClock()  (10000000ul) // (GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock() (GetSystemClock())

#define I2C_CLOCK_FREQ (400000)
    
#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */
