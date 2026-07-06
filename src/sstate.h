/* ------------------------------------------------------------
Title:          sstate.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for system state monitor

Changes:

------------------------------------------------------------ */
#ifndef _SSTATE_H
#define _SSTATE_H

/*
System States, determined by mode and state analysis
*/

// System States are defined as a bit map, because there can be multiple
// states existing at once (ie STOP_COARSE & RUN_COARSE when feathering)
#define S_IDLE          (WORD)0x0000

#define S_RUN_FINE      (WORD)0x0001
#define S_RUN_COARSE    (WORD)0x0002
#define S_RUN_FEATHER   (WORD)0x0004
#define S_RUN_REVERSE   (WORD)0x0008

#define S_STOP_FINE     (WORD)0x0010
#define S_STOP_COARSE   (WORD)0x0020
#define S_STOP_FEATHER  (WORD)0x0040
#define S_STOP_REVERSE  (WORD)0x0080

#define S_ERROR_OPEN    (WORD)0x0100
#define S_ERROR_CURRENT (WORD)0x0200
#define S_ERROR_SLIPRING  (WORD)0x0400	// MHH:11/01/2021
#define S_ERROR_POSITION (WORD)0x0800	// MHH:26/05/2025


// Auto test state machine states
typedef enum 
{
    testIdle = 0,
    testFineFine,
    testFineCoarse,
    testFineFeather,
    testCoarseFine,
    testCoarseCoarse,
    testCoarseFeather,
    testFeatherFine,
    testFeatherCoarse,
    testFeatherFeather,
    testFinish,
    testFeatherReverse
} TestState;

// Initialisation
void initSystemState (void);

// repeated call, eiher forces a state check or monitors state if motor in use
void updateSystemState (void);

// accessor function
WORD systemState (void);

// Are we in a test
TestState motorTestState (void);

// indicate a feather failure
void setUnfeatherFail (void);

// set to unfeathertest
void reverseFeatherTest (void);


// check in unfeather state
BYTE isUnfeathered (void);

void clearUnfeatherCheck( void);

extern BYTE State_fast_result;
extern BYTE State_fast_test;		// Use as an interlock when a fast test in progress

BYTE State_NextFastTest(void);
void State_StartFastTest(void);

void setStateBits (WORD ifrom,WORD bits);
void resetStateBits (WORD ifrom,WORD bits);
extern bool AC200_status_test;

#endif
