/* ------------------------------------------------------------
Title:          control.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for PI control block

Changes:

------------------------------------------------------------ */

#ifndef _CONTROL_H
#define _CONTROL_H

/*
Control States, determined by mode and PID cycle
*/

// Control States
typedef enum
{
    C_IDLE = 0,
    C_FINER,
    C_COARSER,
    C_BRAKE,
	C_BL_CONTROL
} ControlState;
extern ControlState cState;	// For SIG100
// initialisation
void initControl (void);

// repeated call, PI control cycle
void controlCycle (void);

// state accessor function
ControlState controlState (WORD *errorVal);

// returns current set speed (to 1dp)
WORD currentSetSpeed (void);

// returns current measured speed (to 1dp)
WORD currentActualSpeed (void);

// reset the hold set speed to the cruise set speed
void resetHoldSpeed (void);

void setSetSpeed(WORD val);

void Set_cState(ControlState new_cstate,int ifrom);

#endif

