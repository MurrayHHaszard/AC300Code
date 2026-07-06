/* ------------------------------------------------------------
Title:          digital.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for digital Input (switches)

Changes:

------------------------------------------------------------ */
#ifndef _DIGITAL_H
#define _DIGITAL_H

/*
Operating Modes, defined by Inputs
*/
#ifdef MH_XXX	// MHH:15/07/2025
// Operating Modes
typedef enum
{
    MANUAL  = 0,
    FEATHER,		// 1
    HOLD,			// 2
    CRUISE,			// 3
    CLIMB,			// 4
    TAKEOFF,		// 5
//    MAP,
	REVERSE,		// 6 MHH:05/02/2019
	STARTUP			// 7 MHH:18/11/2017
} OpMode;
#endif
// Operating Modes
typedef enum 
{
    MANUAL  = 0,	// in	cnt	 ix
	FEATHER,		// 2	(2)	 1, or 0  (0) extended
    HOLD,			// 4	(3)	 2
    CRUISE,			// 8	(4)  3
    CLIMB,			// 16	(5)  4
    TAKEOFF,		// 32	(6)  5
	REVERSE,		// 2    (2)  7 (same pos as 5 pos feather)
	BETA,			// 2	(2)	 8 (same pos as 5 pos feather)
	INVALID,
 	STARTUP			//           10
} OpMode;

void Auto_cycle_check(void);
OpMode Auto_operating_mode(void);


// Initialisation call. Required for Manual/Auto sense
// which uses interrupts
void initDigital (void);

// repeated call, scans input lines and determines current Mode
void updateOperatingMode (void);

// accessor function
OpMode operatingMode (void);

// manual keys pressed
// defined as a bitmap because they are not necessarily mutually exclusive

#define MANUAL_KEY_FEATHER 0x01
#define MANUAL_KEY_COARSE  0x02
#define MANUAL_KEY_FINE    0x04

// accessor function
BYTE manualKeys (void);

#if BETA_VERSION
BYTE betaModePending (void);
#endif

bool Reverse_front_switch_on(void);

#endif

