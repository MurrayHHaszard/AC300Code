/* ------------------------------------------------------------
Title:          sstate.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        System State sensing code

Changes:

------------------------------------------------------------ */

#include <string.h>
#include <stdio.h>

#include "ac210_global.h"

#include "global.h"
#include "sstate.h"
#include "analog.h"
#include "digital.h"
#include "param.h"
#include "feather.h"
#include "drive.h"
#include "control.h"
#include "manifold.h"
#if REMOTE_VERSION
#include "remote.h"
#endif
#include "ac210_sig100.h"

/*
    System state sensing has several potential "feeds".
    In manual mode.
    The drive state is set directly by the switches. The stop state
    can be partially assumed by a combination of switch state and
    motor current, otherwise if the switches are idle then we use
    the autosense code below.
    In Auto mode.
    If the motor is being driven, then, as above, we can monitor current
    and detect a partial state. If the motor drive is Idle, then we
    use an auto test as below.
    An auto test is automatically cancelled if the motor starts driving
    whether in auto or manual mode. This is detected by an out of spec
    return reading from the sense circuitry, and /or a change of drive state
    during a test sequence. (DRIVE overrides TEST)

    The Auto test is a state machine which takes 9 calls to updateMotorState
    to complete. This is because it places a high voltage on each line in sequence,
    and then monitors current on each of the 3 low legs, providing a 3x3 matrix
    of results which are then used to determine motor state.
*/

// Auto test port definitions
#ifndef AC210_PORT
#define HI_OUTPUT P9
#define HI_OUTPUT_DIR PD9

#define LO_OUTPUT P8
#define LO_OUTPUT_DIR PD8

#define OUTPUT_MASK 0x07
#endif

// loops until an end stop condition is set
#define STOP_DELAY_COUNT 10
#define STOP_MOTOR_CURRENT	20	// Was 50

// Storage
// Main state
WORD currentState;

// Auto test matrix
BYTE autoResults[3][3];

// Auto test state
TestState tState;

// Auto test counter
BYTE testCounter;

// counter for detecting end stop condition.
// required to delay an end stop detection so as to avoid spurious
// end stop condition indication  index 0 = FINE, index 1 = COARSE, index 2 = FEATHER

BYTE endStopCount[3];

// local procedures
void startTest (void);
void progressTest (void);
void cancelTest (int ifrom);
void setOutput  (TestState ts,int ifrom);
//void setStateBits (WORD bits);
//void resetStateBits (WORD bits);

//----------------------------------------------------------------------------------------
#ifdef AC2_TEST		// AC2_TEST
BYTE lastResults[3][3];
void AC2_TEST_ini_state(void)
{
	int i,j;
	for (i=0; i<3; i++) {
        for (j=0; j<3; j++) {
            lastResults[i][j]= 5;
        }
    }
}

void testingCall (void)
{
	BYTE t=Trace("testingCall");
    if (tState == testIdle)
    {
        startTest();
    }
    else {
        progressTest();
    }
    TraceReturn(t,"");
}
#endif
//------------------------------------------------------------------------------------------

// Are we in a test
TestState motorTestState (void)
{
    return (tState);
}
/*
bool Testing_state(void)
{
	if(tState >testIdle && tState < testFinish)
	{
		return true;
	}
	return false;
}
*/

BYTE State_fast_test;		// Use as an interlock when a fast test in progress
void StateTestSlipRingsStart(void);
void StateTestSlipRingsFinish(void);
bool ADC_is_slipring_test_finished(void);

void Set_tState(TestState new_tstate,int ifrom)
{
	if(new_tstate != tState)
	{
		if(new_tstate == testFeatherReverse)
		{
			DPRINTF("Set_tState:testFeatherReverse, ifrom:%d\r\n",ifrom);
		}
	}
	tState = new_tstate;
}

// Initialisation, starts an auto test.
void initSystemState (void)
{

    // set auto test output ports up
    // unprotect port 9 direction
#ifndef AC210_PORT
	PRCR |= 0x04;
    PD9  |= OUTPUT_MASK;
    PD8  |= OUTPUT_MASK;
#endif

    testCounter=0;

//    tState = testIdle;
    Set_tState(testIdle,100);
    currentState = S_IDLE;

    endStopCount[0] = endStopCount[1] = endStopCount[2] = 0;

//    wasBeingDriven = 0;
//    startTest ();
}

// repeated call (every tick),
// either forces a state check or monitors state if motor in use

extern bool Drive_wait_flag;
void Drive_set_temp_max_pwm(WORD temp_max_pwm);
void Drive_reset_temp_max_pwm(void);
extern WORD Drive_last_temp_max_pwm;
static BYTE overCurrentCount;
//static BYTE overCurrentCount2;

extern bool Sig100_connected;	// Should put in header

void updateSystemState (void)
{
    char buf[30];
    BYTE keys, beingDriven;
    WORD motorCurrent;
    WORD oMode;
    WORD oldState;
    MotorDriveState mds;
    FeatherState fs;
    WORD driveDir = 0;

    if(Drive_wait_flag)		// MHH:04/07/2019. See if this fixes problem
    {
    	DPRINTF("Drive_wait\r\n");
    	return;
    }
    if(Sig100_connected)
    {
    	SIG100_check_zone();
    }

    // save the old state
    oldState = currentState;

    // cancel the overcurrent error state for this run through
    // and the driving states....
    resetStateBits (5000,S_ERROR_CURRENT | S_ERROR_POSITION | S_RUN_FINE | S_RUN_COARSE | S_RUN_FEATHER | S_RUN_REVERSE);


    if(Hub_pos_error)		// MHH:26/05/2025
    {
    	 currentState |= S_ERROR_POSITION;
    }
    // retrieve all the state info we need
    oMode = operatingMode();
    keys = manualKeys();
    mds = driveState ();
    fs = featherState();


    // and get motor current in mA
    motorCurrent = scaledValue (A_MOTOR_CURRENT);

    // are we being driven at this instant
    beingDriven = (((oMode == MANUAL) && (keys & ~MANUAL_KEY_FEATHER)) ||
                   (mds == MD_BETA_EXIT) ||
                   ((oMode == FEATHER) && (fs == F_ACTIVE)) ||
                   ((oMode == FEATHER) && (fs == F_REVERSING)) ||
                   ((oMode > FEATHER) && (oMode <= TAKEOFF) && (mds != MD_IDLE) && (mds < MD_BETA)));


    if((oMode > FEATHER) && (oMode <= TAKEOFF))	// MHH:22/03/2017. Because was starting state test while braking
    {
    	if(mds == MD_FINE_BRAKE || mds == MD_COARSE_BRAKE)
    	{
    		beingDriven = 2;
    	}
    }
    if(Control_type == CT_REVERSE)		// This test from version 4.02 as state check was interrupting drive.
    {
    	if(mds != MD_IDLE) beingDriven = 3;
    }
    if(Drive_wait_flag || motorCurrent > 0)		// MHH:03/11/2018
    {
    	beingDriven = 4;
    }
    if (beingDriven)
    {

        // What we need to do here is determine which direction we are being driven
        // this is then used below to set states

        if (oMode == MANUAL)
        {
#if (BETA_VERSION || REVERSING_VERSION)
            if((Control_type == CT_BETA) || (Control_type == CT_REVERSE))
            {
                if (keys & MANUAL_KEY_FINE)
                {
    #if REVERSING_VERSION
                    if(
                      ((Control_type == CT_REVERSE) && (keys & MANUAL_KEY_FEATHER)) ||
                      ((Control_type != CT_REVERSE) && (fs == F_BETA))
					  )
    #endif
                    {
//                        driveDir = S_RUN_FEATHER;
                        driveDir = S_RUN_REVERSE;
                    }
                    else
                    {
                        driveDir = S_RUN_FINE;
                    }
                }
                else
                {
                    if ((keys & MANUAL_KEY_COARSE) || (mds == MD_BETA_EXIT))
                    // Beta exit happens in manual mode now (artificially forced)
                    {
                        driveDir = S_RUN_COARSE;
                    }
                }
            }
            else
            {
                // Normal (non reversing) Version
                if (keys & MANUAL_KEY_FINE)
                {
                    driveDir = S_RUN_FINE;
                }
                else
                {
                    if (keys & MANUAL_KEY_COARSE)
                    {
                        if (keys & MANUAL_KEY_FEATHER)
                        {
                            driveDir = S_RUN_FEATHER;
                        }
                        else
                        {
                            driveDir = S_RUN_COARSE;
                        }
                    }
                }
            }
//#else
#endif
        }
        else
        {
            // Auto mode
#if (REVERSING_VERSION || BETA_VERSION)
//            if((Control_type == CT_REVERSE) || (Control_type == CT_BETA))
            if((Feather_mode == FM_REVERSING) || (Feather_mode == FM_BETA))
            {
#ifdef MH_OLD_STATE
            	if (mds == MD_FINER)
                {
                    driveDir = S_RUN_FINE;
                }
                else if (mds == MD_FEATHER)
                {
                    driveDir = S_RUN_FEATHER;
                }
                else
                {
                    // this includes mds == MD_BETA_EXIT  and mds == MD_FEATHER_REVERSE
                    driveDir = S_RUN_COARSE;
                }
#else
            	switch(mds)
            	{
            	default:
            		break;
            	case MD_FINER:
                    driveDir = S_RUN_FINE;
                    break;
            	case MD_COARSER:
            		driveDir = S_RUN_COARSE;
            		break;
#ifdef MH_XXX
            	case MD_FEATHER_REVERSE:
                	driveDir = S_RUN_FINE;
            		break;
            	case MD_FEATHER:
            		driveDir = S_RUN_FEATHER;
            		break;
#endif
            	case MD_REVERSE:
            		driveDir = S_RUN_REVERSE;
            		break;
            	case MD_REVERSE_REVERSE:
            		driveDir = S_RUN_COARSE;
            		break;
            	}
#endif
            }
            else
            {
#ifdef MH_OLD_STATE
                if ((mds == MD_FINER) || (mds == MD_FEATHER_REVERSE))
                {
                    driveDir = S_RUN_FINE;
                }
                else if (mds == MD_FEATHER)
                {
                    driveDir = S_RUN_FEATHER;
                }
                else
                {
                    driveDir = S_RUN_COARSE;
                }
#else
            	switch(mds)
            	{
            	default:
            		break;
            	case MD_FINER:
            		driveDir = S_RUN_FINE;
            		break;

            	case MD_FEATHER_REVERSE:
#ifdef MH_XXX
            		if(Feather_mode == FM_UNREVERSING)
//            		if(Control_type == CT_REVERSE)
                    {
                		driveDir = S_RUN_COARSE;
                    }
                    else
                    {
                    	driveDir = S_RUN_FINE;
                    }
#else
            		driveDir = S_RUN_FINE;
#endif
                    break;
            	case MD_COARSER:
            		driveDir = S_RUN_COARSE;
            		break;
            	case MD_FEATHER:
#ifdef MH_XXX
            		if(Feather_mode == FM_REVERSING)
 //           		if(Control_type == CT_REVERSE)
                    {
                		driveDir = S_RUN_REVERSE;
                    }
                    else
                    {
                		driveDir = S_RUN_FEATHER;
                    }
#else
            		driveDir = S_RUN_FEATHER;
#endif
            		break;
            	}
#endif

            }
//#else
#endif
        }
        // Halt any auto test in progress
        cancelTest (200);

        WORD mc_current_limit = getParameter(MC_CURRENT_LIMIT);
//        WORD mc_max_pwm = getParameter(MC_MAX_PWM);

//#define MH_NEW_PWM	// MHH:25/07/2023
#ifdef MH_NEW_PWM
        if(ps.parms[BL_ENABLED] != 1)	// MHH:24/07/2023
        {
        	if(Drive_last_temp_max_pwm < mc_max_pwm)	// and already throttling back
        	{
        		int current_no_pwm = (motorCurrent * mc_max_pwm)/Drive_last_temp_max_pwm;
        		if(current_no_pwm > mc_current_limit)	// if not scaled back, are we over current limit?
        		{
        			if(overCurrentCount2++ > 15)		// As overCurrentCount was 10 before we started throttling back
        			{
        				overCurrentDir = driveDir;
        				currentState |= S_ERROR_CURRENT;
        			}
        		}
        		else
        		{
        			overCurrentCount2 = 0;
        		}
        	}
        	else
        	{
        		overCurrentCount2 = 0;
        	}
        }
#endif

        if (motorCurrent > mc_current_limit || Hub_not_ready_error)
//        if (motorCurrent > getParameter(MC_CURRENT_LIMIT))
        {

        	// current over MOTOR_CURRENT_LIMIT
            if (++overCurrentCount >= 20)	// MHH:16/09/2025. Test!!!
//            if (++overCurrentCount >= 25)
            {
                overCurrentCount--;
                currentState |= S_ERROR_CURRENT;
//                overCurrentDir = driveDir;
                if (overCurrentTrip == 0)
                {
                    Set_overcurrent_dir(driveDir);	// MHH:16/09/2025
                	overCurrentTrip = 1;
                    // save the state we are in for later
#if BETA_VERSION
                    if(Control_type == CT_BETA)
                    {
                        if (fs == F_BETA)
                        {
                        	betaOverCurrent = 1;
                        	clearBetaDriveMode();
                        }
                    }
#endif
                }

                if ((oMode > FEATHER) && (mds > MD_IDLE))
                {
                    shutdownDrive ();
                }
                else if ((oMode == FEATHER) ||
                         ((oMode > FEATHER) &&
                          ((mds < MD_IDLE)||(mds == MD_BETA_EXIT))))
                {
                    // feathering OR reversing feather OR reversing BETA
                    shutdownFeather ();
                }
            }
        }
        else if (overCurrentTrip)
        {
            // Stay in this state once tripped
        	// (only in non manual modes OR BETA mode)

            if (oMode != MANUAL)
            {
                currentState |= S_ERROR_CURRENT;
            }
            else
            {

#if BETA_VERSION
                if((Control_type != CT_BETA) || (betaOverCurrent == 0))
                {
            		overCurrentTrip = 0;
            		overCurrentCount = 0;
                }
                if(Control_type == CT_BETA)
                {
                	if (overCurrentTrip)
                	{
                		// A real overcurrent in BETA mode should still be shown
                		currentState |= S_ERROR_CURRENT;
                	}
                }
#endif
/*
#if BETA_VERSION
            	// In beta mode, do not reset the overcurrent
            	if (betaOverCurrent == 0)
            	{

#endif
            		overCurrentTrip = 0;
            		overCurrentCount = 0;
#if BETA_VERSION
            	}
            	if (overCurrentTrip)
            	{
            		// A real overcurrent in BETA mode should still be shown
            		currentState |= S_ERROR_CURRENT;
            	}
#endif
*/
            }
        }
//        else if (motorCurrent <  lastSpeed/15)	// MHH: 21/10/2015 (was 50)
//        else if (motorCurrent <  50)	// MHH: 27/11/2015, back to original as Globe not recognising stop. May need to distinguish between Maxon and Globe.
//        else if (motorCurrent <  40)	// MHH: 14/12/2015, Try 40, 50 causing problems with maxon
        else if (motorCurrent <  25 && (Sig100_connected == false))	// MHH: 05/11/2018
        {
            // If it is zero (less than 50 mA), then all we can say here is that
            // the end stop has been reached..
            overCurrentCount = 0;
            switch (driveDir)
            {
            case S_RUN_FEATHER:
                setStateBits (1000,S_RUN_FEATHER);
                if (++endStopCount[2] >= STOP_DELAY_COUNT)
                {
                    endStopCount[2] = STOP_DELAY_COUNT;
                    setStateBits (1100,S_STOP_FEATHER);
                }
                break;

            case S_RUN_REVERSE:
                setStateBits (1200,S_RUN_REVERSE);
                if (++endStopCount[2] >= STOP_DELAY_COUNT)
                {
                    endStopCount[2] = STOP_DELAY_COUNT;
                    setStateBits (1300,S_STOP_REVERSE);
                }
                break;

            case S_RUN_FINE:
                setStateBits (1400,S_RUN_FINE);
                if (++endStopCount[0] >= STOP_DELAY_COUNT)
                {
                    endStopCount[0] = STOP_DELAY_COUNT;
#ifdef MH_DEBUG_FINE_STOP
                   	if(((currentState & S_STOP_FINE) == 0) && (debug & DEBUG_FINE_STOP))
                   	{
						sprintf(buf,"S_STOP_FINE[1], MC=%d\r\n",motorCurrent);
                    	txDebug(buf);
					}
#endif
#ifdef AC210_PORT
//                   	AC210_SIG60_set_stop(S_STOP_FINE);
#endif
                   	setStateBits (1500,S_STOP_FINE);
                }
                break;

            case S_RUN_COARSE:
                setStateBits (1600,S_RUN_COARSE);
#ifdef AC210_PORT
#ifdef MH_SIG60
                if(AC210_SIG60_motor_active())
                {
                	break;
                }
#endif
#endif
                if (++endStopCount[1] >= STOP_DELAY_COUNT)
                {
                    endStopCount[1] = STOP_DELAY_COUNT;
#ifdef AC210_PORT
//                   	AC210_SIG60_set_stop(S_STOP_COARSE);
#endif
                    setStateBits (1700,S_STOP_COARSE);
                }
                break;
            }
        }
        else
        {
            // normal operation.. Current is OK
            overCurrentCount = 0;
            if(Sig100_connected == false)
            {
                resetStateBits (5100,S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER | S_STOP_REVERSE);
            }
            endStopCount[0] = endStopCount[1] = endStopCount[2] = 0;

            setStateBits (1800,driveDir);
        }
    }
    else	// Not beingDriven
    {
        if (
        	 ((oMode != MANUAL) && overCurrentTrip)
#if BETA_VERSION
        	|| ((Control_type == CT_BETA) && (oMode == MANUAL) && overCurrentTrip && betaOverCurrent)
#endif
           )

        {
            currentState |= S_ERROR_CURRENT;
        }
        else
        {
            overCurrentCount = 0;
            if(State_fast_test)	// MH:05/07/2019
            {
            	return;
            }
            if(Sig100_connected)
            {
//            	SIG100_check_zone();
            	return;
            }

            // Not being driven.. Maybe start or continue an auto test
            if (!tState)
            {
                // start an auto test if we are at the start of a second
            	BYTE ticks = timerTicks();
            	if (ticks == 0 && State_fast_test == FALSE)
                {
                   startTest ();
                }
            	else
            	{
#ifdef MH_SLIPRING_STATE_LOGIC
            		if(ticks == 25)	// Spread test over 4 ticks
            		{
            			StateTestSlipRingsStart();
            		}
#endif
/*
            		if(ADC_is_slipring_test_finished())
            		{
            			StateTestSlipRingsFinish();
            		}
*/
            	}
            }
            else if (tState)
            {
                // progress the auto test
                progressTest ();
            }
        }
    }

    // Debug output if required
    if ((debug & DEBUG_SYSTEM_STATE) && (currentState != oldState))
    {
        sprintf (buf, "State  -> %X\r\n", currentState);
        txDebug (buf);
    }
}
//-------------------------------------------------------------------------------------------
// Used by Xoar2 overcurrent reset instruction
void State_reset_overcurrent_trip(void)
{
    overCurrentTrip = 0;
    overCurrentCount = 0;
}

// accessor function
WORD systemState (void)
{
    return (currentState);
}

// Unfeather Failure
void setUnfeatherFail (void)
{

}

// Auto test code
void startTest (void)
{
    BYTE i,j, nonFeather;
//    PRINTF("startTest\r\n");
//	BYTE t=Trace("startTest");
//    DPRINTF("\r\nstartTest:");	// MHH:15/01/2026

    nonFeather = 0;
    if (!isFeatheringProp() && !isBetaProp())
    {
        nonFeather = 1;
    }
    // clear storage
    for (i=0; i<3; i++)
    {
        for (j=0; j<3; j++)
        {
            if (nonFeather && ((i == 2) || (j == 2)))
            {
                // fill feather related states with the OPEN state if non-feathering
                autoResults[i][j]= STATE_OPEN;
            }
            else
            {
                autoResults[i][j]= STATE_OK;
            }
        }
    }
    // Turn off Auto Motor Drive
    disableMotorDrive ();

    // set state
//    tState = testFineFine;
    Set_tState(testFineFine,200);
    // and outputs
    setOutput (tState,100);
//    TraceReturn(t,"");
}
//------------------------------------------------------------------------------
// Assist with finding if FINE and COARSE stops have been hit for positioning,
void StateFastTest(TestState test_state)
{
	if(tState)		// test in progress?
	{
        cancelTest (300);
	}

    disableMotorDrive ();
    State_fast_test = TRUE;
	setOutput(test_state,200);
}
//------------------------------------------------------------------------------
void StateStartCoarseStopTest(void)
{
	StateFastTest(testCoarseFine);
}
//------------------------------------------------------------------------------
void StateStartFineStopTest(void)
{
	StateFastTest(testFineCoarse);
}
//------------------------------------------------------------------------------
#ifdef MH_SLIPRING_STATE_LOGIC
TestState Last_state_set;
void StateTestSlipRingsStart(void)
{
	if(Last_state_set == testIdle || Last_state_set == testFinish)		// Make sure status pins not being used
	{
		disableMotorDrive ();			// May not be needed
	    ADC_slipring_test_ini(SLIPRING_TEST_STATE);
	}
	else
	{
		mh_debug();
	}
}
#endif
//------------------------------------------------------------------------------
/*
void StateTestSlipRingsFinish(void)
{
//    SlipRing_test_state = sr_test_idle;
	if(Last_state_set != testCoarseFine)
	{
		mh_debug();
	}

	setOutput(0);	// Note: Test may not work if at Coarse stop
}
*/
//------------------------------------------------------------------------------
// This is called 1 tick later
extern WORD Analog_raw;
int StateReadStopTest(void)
{
	int result;
	result = FALSE;
	if(scaledValue (A_MOTOR_STATE) == STATE_OPEN) result = TRUE;
//	txDebugVal("Analog_raw",Analog_raw);
//    tState = testIdle;
    Set_tState(testIdle,300);
    setOutput (tState,300);
	State_fast_test = FALSE;
	return result;
}
//==============================================================================
// These routines for remote reversing.
TestState State_fast_test_t[]=
{
		testFineCoarse,
		testCoarseFine,
		testFeatherFine,	// Feather stop test
		testFeatherCoarse,	// Reverse stop test
		testFinish,
};
BYTE State_fast_result;
int State_fast_index;
TestState Fast_tstate;
//------------------------------------------------------------------------------
BYTE State_NextFastTest(void)
{
	if(Drive_wait_flag)
	{
		mh_debug();
	}
	if(Sig100_connected) return true;

	if(tState != Fast_tstate)
	{
		mh_debug();
	}
	if(State_fast_test == false)	// in case was cancelled
	{
		return true;
	}
	if(tState == testFinish)
	{
		txDebug("State_NextFastTest:E1\r\n");
		return false;
	}
	if(scaledValue (A_MOTOR_STATE) == STATE_OPEN)	// Set bit to indicate STOP
	{
		State_fast_result |= (1 << State_fast_index);
	}
	State_fast_index++;
//	tState = State_fast_test_t[State_fast_index];
	Fast_tstate = State_fast_test_t[State_fast_index];
	Set_tState(Fast_tstate,400);

//	Fast_tstate = tState;
	if(tState == testFinish)
	{
		State_fast_test = false;
//		tState = testIdle;
		Set_tState(testIdle,500);
	    setOutput (tState,400);
	    return true;
	}
    setOutput (tState,500);
    return false;
}
//------------------------------------------------------------------------------
void State_StartFastTest(void)
{
	if(Drive_wait_flag)
	{
		mh_debug();
	}
	if(Sig100_connected) return;

//	DPRINTF("\r\nState_StartFastTest:\r\n");
	State_fast_index = 0;
	State_fast_result = 0;
	if(tState)		// test in progress?
	{
        cancelTest (400);
	}
//	tState = State_fast_test_t[State_fast_index];
//	Fast_tstate = tState;
	Fast_tstate = State_fast_test_t[State_fast_index];
	Set_tState(Fast_tstate,600);

    disableMotorDrive ();
    State_fast_test = TRUE;
	setOutput(tState,600);
}
//------------------------------------------------------------------------------
#ifdef AC2_TEST		// AC2_TEST

static void AC2_TEST_state(void)
{
	BYTE i,j,error;
    char buf[50];
    BYTE t=Trace("AC2_TEST_state");
    error = 0;
	for (i=0; i<3; i++) {
	    for (j=0; j<3; j++) {
	        if (autoResults[i][j] != lastResults[i][j]) {
	            error++;
	        }
	    }
	}
	if (error) {
		           //            1         2         3         4         5
                   //  012345678901234567890123456789012345678901234567890
	    sprintf (buf, "Fine %d,%d,%d, Coarse %d,%d,%d, Feather %d,%d,%d",
	               autoResults[0][0],
	               autoResults[0][1],
	               autoResults[0][2],
	               autoResults[1][0],
	               autoResults[1][1],
	               autoResults[1][2],
	               autoResults[2][0],
	               autoResults[2][1],
	               autoResults[2][2] );
	    txDebug (buf);
	}
	for (i=0; i<3; i++) {
	    for (j=0; j<3; j++) {
	        lastResults[i][j] = autoResults[i][j];
	    }
	}

	// and go to idle
//	tState = testIdle;
	Set_tState(testIdle,700);

	// enable auto drive
	enableMotorDrive ();
	TraceReturn(t,"");
}
#endif
//------------------------------------------------------------------------------
#ifdef AC210_PORT
//#define MH_DISPLAY_RAW
#ifdef MH_DISPLAY_RAW
WORD rawResults[3][3];
#endif
#endif
bool AC200_status_test;
void progressTest (void)
{
    // Here we analyse results of last state,
    // then step it on....
    BYTE modState,i,j,error, nonFeather;
    bool open_circuit;
    char buf[30];


	BYTE t = Trace("progressTest");
    WORD mh_state_bits = currentState;

    nonFeather = 0;
    if (!isFeatheringProp() && !isBetaProp())
    {
        nonFeather = 1;
    }
    // use state as an index into results array
    modState = tState-1;

    if (tState < testFinish)
    {
        // save result of last state test
    	BYTE result = scaledValue (A_MOTOR_STATE);
        autoResults[modState/3][modState%3] = result;
//        DPRINTF("=%d, rawval = %d\r\n",result,Analog_raw);
//       DPRINTF("=%d,",result);	// MHH:15/01/2026
//        autoResults[modState/3][modState%3] = scaledValue (A_MOTOR_STATE);
#ifdef MH_DISPLAY_RAW
        rawResults[modState/3][modState%3] = Analog_raw;
#endif
        // increment state
        if (nonFeather)
        {
            // skip all feather related states
            if (tState == testFineCoarse)
            {
//                tState += 2;
                Set_tState(tState + 2,800);
            }
            else if (tState == testCoarseCoarse)
            {
//                tState += 5;
                Set_tState(tState + 5,900);
            }
            else
            {
//                tState++;
                Set_tState(tState + 1,1000);
            }
        }
        else
        {
//            tState++;
            Set_tState(tState + 1,1100);
        }
        setOutput (tState,700);
    }
    else if (tState == testFinish)
    {
        // must be at state testFinish.. Analyse results
        // If any overange results exist, can the results
#ifdef AC2_TEST		// AC2_TEST
        if(ac2_test_flag == AC2_TEST_ON)
        {
        	AC2_TEST_state();
        	TraceReturn(t,"AC2_TEST_state");
        	return;
        }
#endif
        error = 0;
        for (i=0; i<3; i++)
        {
            for (j=0; j<3; j++)
            {
                if (autoResults[i][j] == STATE_ERROR)
                {
                    error++;
                }
            }
        }
        if (!error)
        {
#ifdef MH_DISPLAY_RAW
            AC210_display_auto_results();
#endif

            if (debug & DEBUG_STATE_MATRIX)
            {
                sprintf (buf, "Fine  [%d][%d][%d]\r\n", autoResults[0][0],
                                                        autoResults[0][1],
                                                        autoResults[0][2]);
                txDebug (buf);
                sprintf (buf, "Corse [%d][%d][%d]\r\n", autoResults[1][0],
                                                        autoResults[1][1],
                                                        autoResults[1][2]);
                txDebug (buf);
                sprintf (buf, "Feath [%d][%d][%d]\r\n", autoResults[2][0],
                                                        autoResults[2][1],
                                                        autoResults[2][2]);
                txDebug (buf);

            }
            // clear the state we are going to set up

            resetStateBits (5200,S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER | S_STOP_REVERSE | S_ERROR_OPEN);
            AC200_status_test = true;

            // set all the same leg results to open
            // ie finefine, coarsecoarse, featherfeather
            for (i=0; i<3; i++)
            {
                autoResults[i][i] = STATE_OPEN;
            }

            // this lets us do a global test for Motor Open Circuit. Because
            // the coarse and feather outputs are tied at the motor connection
            // we must test specifically ALL required states.

            open_circuit = false;
            if ((autoResults[0][1] == STATE_OPEN) &&     // fine_coarse
            	(autoResults[0][2] == STATE_OPEN) &&	 // fine_feather
				(autoResults[1][0] == STATE_OPEN))   	 // coarse_fine
            {
            	if(autoResults[2][0] == STATE_OPEN)      // feather_fine
            		open_circuit = true;
            	if(autoResults[1][2] == STATE_OPEN)		 // coarse_feather
            		open_circuit = true;
            }
            if((Control_type == CT_REVERSE) || (Control_type == CT_BETA))
            {
                if((autoResults[1][0] == STATE_OPEN) &&     // coarse
                   (autoResults[1][2] == STATE_OPEN) &&
                   (autoResults[2][0] == STATE_OPEN) &&     // and feather
                   (autoResults[2][1] == STATE_OPEN))
                    		open_circuit = true;

            }
            else
            {
                if((nonFeather == 0) &&                     // only for feather version
                 (autoResults[0][1] == STATE_OPEN) &&     // fine
                 (autoResults[0][2] == STATE_OPEN) &&
                 (autoResults[2][0] == STATE_OPEN) &&     // and feather
                 (autoResults[2][1] == STATE_OPEN))
							open_circuit = true;

            }
            if(open_circuit)
/*
            if (((autoResults[0][1] == STATE_OPEN) &&     // fine_coarse
                 (autoResults[0][2] == STATE_OPEN) &&     // fine_feather
                 (autoResults[1][0] == STATE_OPEN) &&     // coarse_fine
                 (autoResults[2][0] == STATE_OPEN)) ||    // feather_fine

                ((autoResults[0][1] == STATE_OPEN) &&     // fine
                 (autoResults[0][2] == STATE_OPEN) &&
                 (autoResults[1][0] == STATE_OPEN) &&     // and coarse
                 (autoResults[1][2] == STATE_OPEN)) ||
#if (REVERSING_VERSION || BETA_VERSION)
                ((autoResults[1][0] == STATE_OPEN) &&     // coarse
                 (autoResults[1][2] == STATE_OPEN) &&
                 (autoResults[2][0] == STATE_OPEN) &&     // and feather
                 (autoResults[2][1] == STATE_OPEN)))
#else
                ((nonFeather == 0) &&                     // only for feather version
                 (autoResults[0][1] == STATE_OPEN) &&     // fine
                 (autoResults[0][2] == STATE_OPEN) &&
                 (autoResults[2][0] == STATE_OPEN) &&     // and feather
                 (autoResults[2][1] == STATE_OPEN)))
#endif
*/
            {
                // the definition of open motor
                currentState |= S_ERROR_OPEN;
#ifdef MH_DISPLAY_RAW
                AC210_display_auto_results();
#endif
            }
            else
            {
                // set each state..
                // Fine....
                if ((autoResults[0][1] == STATE_OPEN) &&
                    (autoResults[0][2] == STATE_OPEN))
                {
                    setStateBits (1900,S_STOP_FINE);
#ifdef MH_DISPLAY_RAW
                AC210_display_auto_results();
#endif
                }
                else
                {
                    endStopCount[0] = 0;
                }
                // Coarse....
                if ((autoResults[1][0] == STATE_OPEN) &&
                    (autoResults[1][2] == STATE_OPEN))
                {
                    setStateBits (2000,S_STOP_COARSE);
                }
                else
                {
                    endStopCount[1] = 0;
                }
                // Feather....
                if (!nonFeather)
                {
#if BETA_VERSION
                	if(Control_type == CT_BETA)
                	{
                		if ((autoResults[1][1] == STATE_OPEN) &&
                				(autoResults[2][1] == STATE_OPEN) &&
								(featherState() == F_BETA))
                		{
                			// Beta mode stuffs the codes
                			setStateBits (2100,S_STOP_FEATHER);
                			resetStateBits (5300,S_STOP_FINE);
                		}
                		else if ((autoResults[2][0] == STATE_OPEN) &&
                				(autoResults[2][1] == STATE_OPEN))
                		{
                			// Not in Beta mode, or in Beta Exit
                			resetStateBits (5400,S_STOP_FINE);
                			setStateBits (2200,S_STOP_FEATHER);

                		}
                	}
                	else
                	{
                		if ((autoResults[2][0] == STATE_OPEN) &&
                				(autoResults[2][1] == STATE_OPEN))
                		{
//                			setStateBits (S_STOP_FEATHER);
                			// only want feather stop indicated
#if (REVERSING_VERSION)
                			if(Control_type == CT_REVERSE)
                			{
                    			setStateBits (2300,S_STOP_REVERSE);
                				resetStateBits (5500,S_STOP_FINE);
                			}
                			else
                			{
                    			setStateBits (2400,S_STOP_FEATHER);
                				resetStateBits (5600,S_STOP_COARSE);
                			}
                			//#else
#endif

                		}
#endif
                		else
                		{
                			endStopCount[2] = 0;
                		}
                	}
                }
            }
        }

        // and go to idle
//        tState = testIdle;
        Set_tState(testIdle,1200);
        if(Control_type == CT_REVERSE)
        {
            if(RC_X_last_action)
            {
//            	PRINTF("last_action:%d, currentState:%04x\r\n",(WORD)RC_X_last_action,currentState);
            	if((currentState & (S_STOP_REVERSE | S_STOP_FINE)) == 0)
    	        {
    	        	Set_Zone(RC_S_ZONE_NORMAL);
    	        }
    	        else
    	        {
    	        	Set_Zone(RC_S_ZONE_REVERSE);
    	        }
            	RC_X_last_action = 0;
            }
            else
            {
            	if((currentState & (S_STOP_REVERSE | S_STOP_FINE)) == 0)
    	        {
    	        	Set_Zone(RC_S_ZONE_NORMAL);
    	        }
            	else
            	{
                	if(currentState & S_STOP_REVERSE)
                	{
        	        	Set_Zone(RC_S_ZONE_REVERSE);
                	}
            	}
            }
        }

        // enable auto drive
        enableMotorDrive ();
    }

    if(mh_state_bits & S_STOP_REVERSE)
    {
    	if((currentState & S_STOP_REVERSE) == false)
    	{
    		mh_debug();
    	}
    }

    TraceReturn(t,"");
}


/*
#if BETA_VERSION
                	if ((autoResults[1][1] == STATE_OPEN) &&
                		(autoResults[2][1] == STATE_OPEN) &&
                		(featherState() == F_BETA))
                	{
                		// Beta mode stuffs the codes
                		setStateBits (S_STOP_FEATHER);
                		resetStateBits (S_STOP_FINE);
                	}
                	else if ((autoResults[2][0] == STATE_OPEN) &&
                             (autoResults[2][1] == STATE_OPEN))
                	{
                		// Not in Beta mode, or in Beta Exit
                		resetStateBits (S_STOP_FINE);
                		setStateBits (S_STOP_FEATHER);

                	}
#else
                    if ((autoResults[2][0] == STATE_OPEN) &&
                        (autoResults[2][1] == STATE_OPEN))
                    {
                        setStateBits (S_STOP_FEATHER);
                        // only want feather stop indicated
#if (REVERSING_VERSION)
                        if(Control_type == CT_REVERSE)
                        {
                            resetStateBits (S_STOP_FINE);
                        }
                        else
                        {
                            resetStateBits (S_STOP_COARSE);
                        }
//#else
#endif

                    }
#endif
                    else
                    {
                        endStopCount[2] = 0;
                    }
                }

            }
        }



        // and go to idle
        tState = testIdle;

        // enable auto drive
        enableMotorDrive ();
    }
}
*/
#ifdef AC210_PORT
#ifdef MH_DISPLAY_RAW
void AC210_display_auto_results(void)
{
    printf ("Fine  [%d][%d][%d]\r\n", autoResults[0][0],
                                            autoResults[0][1],
                                            autoResults[0][2]);
    printf ("Corse [%d][%d][%d]\r\n", autoResults[1][0],
                                            autoResults[1][1],
                                            autoResults[1][2]);
    printf ("Feath [%d][%d][%d]\r\n", autoResults[2][0],
                                            autoResults[2][1],
                                            autoResults[2][2]);
    printf ("RawFine  [%d][%d][%d]\r\n", rawResults[0][0],
                                            rawResults[0][1],
                                            rawResults[0][2]);
    printf ("RawCorse [%d][%d][%d]\r\n", rawResults[1][0],
                                            rawResults[1][1],
                                            rawResults[1][2]);
    printf ("RawFeath [%d][%d][%d]\r\n", rawResults[2][0],
                                            rawResults[2][1],
                                            rawResults[2][2]);

}
#endif
#endif

void cancelTest (int ifrom)
{
//	DPRINTF("\r\ncancelTest:[%d]\r\n",ifrom);
    // set state
	State_fast_test = false;
	if (tState != testFeatherReverse)
    {
//        tState = testIdle;
        Set_tState(testIdle,1300);
        // and outputs
        setOutput (tState,800);
        enableMotorDrive ();
    }
}
#ifdef MH_XXX
const char *TestDesc[]={"Idle","FineFine","FineCoarse","FineFeather",	// 0,1,2,3
"CoarseFine","CoarseCoarse","CoarseFeather",                            // 4,5,6
"FeatherFine","FeatherCoarse","FeatherFeather",							// 7,8,9
"Finish",																// 10
"FeatherReverse",														// 11. What pin does this correspond to?
};
#endif
const char *TestDesc[]={"Idle","FiFi","FiCo","FiFe",	// 0,1,2,3
"CoFi","CoCo","CoFe",                            // 4,5,6
"FeFi","FeCo","FeFe",							// 7,8,9
"Finish",																// 10
"FeRe",				  						// 11. What pin does this correspond to?
};
void setOutput (TestState ts,int ifrom)
{
#ifdef MH_SLIPRING_STATE_LOGIC
	Last_state_set = ts;
#endif
#ifdef AC210_PORT
    p_ZeroStatePins();
#ifdef MH_XXX    // MHH:15/01/2026
    int ix = (int)ts;
    char *desc = "?Unknown";
    if(ix >= 0 && ix <= 11)
    {
    	desc = TestDesc[ix];
    }
    if(ifrom != 700)  DPRINTF("\r\nsetOutput:[%d]",ifrom)
   	DPRINTF(":%d (%s)",ix,desc);
#endif
//    DPRINTF("setOutput[%d]: tx=%d %s,",ifrom,ix,desc);
    if ((ts > testIdle) && (ts < testFinish))
    {
        p_SetStatePins((int)ts);
    }
#else
	BYTE maskHi, maskLo;

// turn off all outputs, Hi & Lo
	HI_OUTPUT &= ~OUTPUT_MASK;
	LO_OUTPUT &= ~OUTPUT_MASK;
    if ((ts > testIdle) && (ts < testFinish))
    {
        // valid test mode... set new outputs
        maskHi = (0x01 << ((ts-1)/3));
        maskLo = (0x01 << ((ts-1)%3));
        HI_OUTPUT |= maskHi;
        LO_OUTPUT |= maskLo;
    }
#endif
}
#ifdef MH_DEBUG_STOP_REVERSE
#define MH_CHECK_STATE_MAX	8
WORD mh_state;
WORD mh_check_state_t[MH_CHECK_STATE_MAX];
WORD mh_check_state_ix;
void mh_check_state(WORD ifrom)
{
	bool mh_bit_on = ((mh_state & S_STOP_REVERSE) != 0);
	bool cs_bit_on = ((currentState & S_STOP_REVERSE) != 0);
	if(cs_bit_on != mh_bit_on)
	{
		mh_check_state_t[mh_check_state_ix++] = ifrom;
		mh_check_state_ix%= MH_CHECK_STATE_MAX;
		if(ifrom < 5000 && AC200_status_test)
		{
			if(cs_bit_on == false)
			{
				mh_debug();
			}
		}
	}
	mh_state = currentState;
}
#else
void mh_check_state(WORD ifrom){}
#endif
#ifdef MH_XXX		// MHH:15/01/2026
void mh_show_state(WORD ifrom,WORD bits, WORD stop_bit,char *desc)
{
	if(bits & stop_bit) DPRINTF(";STOP_%s[%d]",desc,ifrom);
}
#endif
void setStateBits (WORD ifrom,WORD bits)
{
/* Debug code to see where STOP_FINE was being set when current was not zero. 03/11/2018
    if(bits & (S_STOP_FINE | S_STOP_COARSE))
    {
        WORD motorCurrent = scaledValue (A_MOTOR_CURRENT);
        if(motorCurrent > 0)
        {
        	MyBreak();
        }
    }
*/
	WORD newstate;
	newstate = (currentState | bits);
	currentState = newstate;
#ifdef MH_XXX	// MHH:31/01/2026
//	currentState |= bits;
	char *desc = "Y";
	if(newstate == currentState) desc = "N";

//	if(newstate == currentState) return;
	currentState = newstate;
	DPRINTF("[%02d,%s]set:[%d],%04x,%04x\r\n",sysTick%100,desc,ifrom,newstate,bits);
#endif
#ifdef MH_XXX		// MHH:15/01/2026
	 mh_show_state(ifrom,bits, S_STOP_FINE,"FINE");
	 mh_show_state(ifrom,bits, S_STOP_COARSE,"COARSE");
	 mh_show_state(ifrom,bits, S_STOP_FEATHER,"FEATHER");
	 mh_show_state(ifrom,bits, S_STOP_REVERSE,"REVERSE");

	if(bits & S_STOP_REVERSE)	// MHH:Debug! 22/10/22
	{
		mh_check_state(ifrom);
	}
#endif
	if(RC_X_last_action == RC_X_WAIT_FINISH) //MHH:03/04/2018
	{
		if(bits & (S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER | S_STOP_REVERSE))
		{
			RC_X_last_action = 0;
		}
	}
//#define MH_DEBUG_STATE
#ifdef MH_DEBUG_STATE
    if(currentState != mh_state)
    {
//    	if(currentState & (S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER | S_STOP_REVERSE))
//    	if(currentState & S_RUN_FINE)
    	if((currentState & 0x70) == 0x70)
    	{
        	mh_debug();
    	}
    	mh_state = currentState;
    }
#endif
}
void resetStateBits (WORD ifrom,WORD bits)
{
	WORD newstate;
	newstate = (currentState & ~bits);
	currentState = newstate;

	//	currentState &= ~bits;
#ifdef MH_XXX	// MHH:31/01/2026
	char *desc = "Y";
	if(newstate == currentState) desc = "N";
	currentState = newstate;
	DPRINTF("[%02d,%s]reset:[%d],%04x,%04x\r\n",sysTick%100,desc,ifrom,newstate,bits);
#endif
}

// set to unfeathertest
void reverseFeatherTest (void)
{
    // can now test for not feathered by setting the coarse sense on high
    // (or the fine sense in a reversing scenario)
    // and waiting for it to go low..... (ie set state coarsecoarse)
    // In Beta version we DO NOT USE THIS, AS the beta relay affects the sampling !!!!!


#if (REVERSING_VERSION || BETA_VERSION)
	// MHH: 20/03/2018. Since Beta does not use this routine, then no point in testing for beta, so remove test
//    if((Control_type == CT_REVERSE) || (Control_type == CT_BETA))

    if(Control_type == CT_REVERSE)
    {
        setOutput (testFineFine,900);
    }
    else
    {
        setOutput (testCoarseCoarse,1000);
    }
//#else
#endif
//    tState = testFeatherReverse;
    Set_tState(testFeatherReverse,1400);
}


// check in unfeather state
BYTE isUnfeathered (void)
{
//	return STATE_OK;		// MHH:22/03/2018: Test only!!!!!
	return (scaledValue (A_MOTOR_STATE));
}

void clearUnfeatherCheck( void)
{
//    tState = testIdle;
    Set_tState(testIdle,1500);
    setOutput (tState,1100);
}
