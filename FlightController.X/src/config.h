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
    
#include <stdlib.h>
#include <stdint.h>
#include <p32xxxx.h>
#include <xc.h>
#include <plib.h>
#include <peripheral/system.h>
#include <errno.h>
#include <math.h>
#include "i2c.h"
#include "lsm330tr.h"  
#include "pid.h"

#define SYS_FREQ (80000000L)
#define PB_DIV 8
#define PRESCALE 256
#define TOGGLES_PER_SEC 1
#define T1_TICK (SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define T2_TICK (SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define	GetSystemClock()      (80000000ul)
#define	GetPeripheralClock()  (10000000ul) 
#define	GetInstructionClock() (GetSystemClock())

#define OFFSET (10000.0)    // decimal place shift
#define RAD (M_PI / 180.0)  // conversion from degrees to radians
#define DT (1/100.0)  // change in time between sensor readings
#define DT_OFFSET (100) // decimal shift * time step ***** Change if DT, OFFSET or sample rate change
#define I2C_CLOCK_FREQ (400000)

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */
