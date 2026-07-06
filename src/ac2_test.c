/*
 * ac2_test.c
 *
 *  Created on: 25/08/2017
 *      Author: Murray
 */


/* ------------------------------------------------------------
Title:          main.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   22/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller test software
Version:        1.0
Platform:       M16/62
Comment:        Main program loop implementation

Changes:

------------------------------------------------------------ */

#include <stdio.h>
#include <string.h>

#include "global.h"
#include "analog.h"
#include "comms.h"
#include "digital.h"
#include "leds.h"
#include "param.h"
#include "rpm.h"
#include "sstate.h"
#include "control.h"
#include "drive.h"
#include "feather.h"
#include "log.h"


void dumpCurrentMode (void);		// In digital.c
void setModeDebug ( BYTE on);
void stopTheFeather (void);			// feather.c
void startTheFeather (void);
void setDrive ( ControlState control);	// drive.c
void Set_Prop_RPM(WORD mag_rpm);	// comms.c
void cancelTest (int ifrom);				// sstate.c
void testingCall (void);
void AC2_TEST_ini_state(void);
void AC2_check_tstate(int ifrom);

//const WORD version = 402;

typedef enum {
    tIdle = 0,
    tTestingLEDS,
    tTestingModes,
    tTestingCurrentZero,
    tTestingCurrentGain,
    tTestingDrive,
    tTestingRPM,
    tTestingState,
    tTestingOther,
    tTestFinished
} HTestState;

const char htName[10][20] = {
    "Space Bar to Start",
    "Testing LEDS",
    "Testing Modes",
    "Current Offset",
    "Current Gain",
    "Testing Drive",
    "Testing RPM",
    "Testing State",
    "Testing Misc",
    "Tests Complete"
};
/*
    Version Information
    V 1.00. First release. Test software is based on the real stuff because everything
            has to be set up properly anyway..
      1.01  Added dimmer testing and code to Misc tests
      1.02  Modified to detect PCB version, modified watchdog operation
      1.03  Added current offset and gain operation

*/

void setLedState ( LedIndex led, BYTE state) {
    ledState[led] = state;
}

//volatile BYTE waiting;
// ticks counts up to 49, (1 seconds worth) so we can flash at 1 Hz
//BYTE timerTick;
//WORD debug;
WORD tCount, tDelay, tTest;

HTestState htState;

//-----------------------------------------------------------------------
static void Set_Debug_Bitmask(BYTE on,WORD bitmask)
{
	if(on)
		debug |= bitmask;
	else
		debug &= ~bitmask;
}

void setModeDebug ( BYTE on)
{
	Set_Debug_Bitmask(on,DEBUG_SWITCH_STATE);
}

void setRPMDebug ( BYTE on)
{
	Set_Debug_Bitmask(on,DEBUG_SPEED_INPUT);
}

void setVoltageDebug (BYTE on)
{
	Set_Debug_Bitmask(on,DEBUG_ANALOG_VOLTAGE);
}

void setDimmerDebug (BYTE on)
{
	Set_Debug_Bitmask(on,DEBUG_DIMMER);
}
//------------------------------------------------------------------------
BYTE currentTest (void)
{
    return ((BYTE)htState);
}

void cancelAnyTest (void) {
    cancelTest(100);
}
//------------------------------------------------------------------------
static void Turn_all_leds_off(void)
{
	for(BYTE i=1;i<LED_RELAY;i++)
	{
		setLedState (i, LED_OFF);
	}
}
//------------------------------------------------------------------------
static void Turn_red_leds_on(void)
{
	for(BYTE i=2;i<LED_RELAY;i+=2)
	{
		setLedState (i, LED_ON);
	}
}
//------------------------------------------------------------------------
static void Turn_green_leds_on(void)
{
	for(BYTE i=1;i<LED_RELAY;i+=2)
	{
		setLedState (i, LED_ON);
	}
}
//------------------------------------------------------------------------
static void Turn_all_leds_on(void)
{
	Turn_red_leds_on();
	Turn_green_leds_on();
}
//------------------------------------------------------------------------
void incrementTest (void)
{
	BYTE t = Trace("incrementTest");

    if (htState == tTestFinished)
    {
        htState = tIdle;
    }
    else {
        htState++;
    }
    txDebug (&htName[htState][0]);

    switch (htState)
    {
    case tIdle:
        // turn off LEDS
    	Turn_all_leds_off();
        tCount = tDelay = tTest = 0;
        break;

    case tTestingLEDS:
        tCount = tDelay = tTest = 0;
        break;

    case tTestingModes:
        // Turn off LEDS
    	Turn_all_leds_off();
        dumpCurrentMode();
        tCount = tDelay = tTest = 0;
        AC2_check_tstate(200);
        break;

    case tTestingCurrentZero:
        // Turn off debug strings
        setModeDebug (0);
        tCount = tDelay = tTest = 0;
        // turn off drive
        stopTheFeather();
        setDrive (C_IDLE);
        break;

    case tTestingCurrentGain:
        // turn on drive
        setDrive (C_COARSER);
        break;

    case tTestingDrive:
        setDrive (C_IDLE);
        tCount = tDelay = tTest = 0;
        break;

    case tTestingRPM:
        // turn off drive
        stopTheFeather();
        setDrive (C_IDLE);
        // RPM to 6000
        Set_Prop_RPM(6000);
//        TA1 = 2499;
        tCount = tDelay = tTest = 0;
        break;

    case tTestingState:
        // turn off debug
//        TraceVal = 1;
        setRPMDebug (0);
        AC2_TEST_ini_state();		// sstate.c
        tCount = tDelay = tTest = 0;
        break;

    case tTestingOther:
        cancelAnyTest();
        tCount = tDelay = tTest = 0;
        break;

    case tTestFinished:
        setVoltageDebug(0);
        setDimmerDebug (0);
        tCount = tDelay = tTest = 0;
        break;
    }

	TraceReturn(t,"");
}

// Flags an overcurrent state. reset by switching to manual
// and back to an auto mode
//BYTE overCurrentTrip = 0;
//BYTE overCurrentDir = 0;
BYTE ac2_test_flag;

// ------------------------------------------------------------
char PrintBuffer[80];
void Printf(void)
{
	txDebug(PrintBuffer);
}
// ------------------------------------------------------------
void AC2_TEST_start(void)
{
	WORD pcb_version;
	htState = tTestFinished;

    sprintf(PrintBuffer,"\r\nAC200 Test Software V1.%02d\r\n",SUBVERSION);
	Printf();

	pcb_version = pcbVersion();
#ifdef AC210_PORT
//    sprintf(PrintBuffer,"Hardware Version: %d%c PCB\r\n",pcb_version,AC210_hardware_version);
    sprintf(PrintBuffer,"Hardware Version: %d%c PCB\r\n",pcb_version/100,pcb_version%100);
#else
    sprintf(PrintBuffer,"Hardware Version: %d PCB\r\n",pcb_version);
#endif
	Printf();
    ac2_test_flag = AC2_TEST_ON;
    LogData.enabled = false;			// MHH:03/03/2018. Turn off logging.
    incrementTest();
    cancelAnyTest();		// 26/01/2018
    AC2_check_tstate(100);
}
//-----------------------------------------------------------------------------------------------
BYTE AC2_TEST_get_command(void);		// comms.c
#define TERMINATOR 0x20
#define INCREASE 0x2B					// '+'
#define DECREASE 0x2D					// '-'
#define CMD_RETURN		13
#define CMD_LINEFEED	10
#define REBOOT		'!'


static void AC2_TEST_checkForCommand (void) {
    BYTE command;


    // command processor. Called every time through the
    // main loop, and responds to message after it arrives
    command = AC2_TEST_get_command();
    if (command) {
    	if(command == REBOOT)
    	{

            txDebug ("Rebooting AC200...");
#ifdef AC210_PORT
    		wait_ms(100);		// Time for message to go
    		AC210_reboot();
//   			NVIC_SystemReset();			// Should not pass here
#else
			Delay_ms(100);
			M16_Reboot();
#endif
   			return;		// But just in case..

    	}


        if (command == TERMINATOR)
        {
            incrementTest ();
            return;
        }
        switch (htState)		// MHH:25/06/2018: Change logic to only advance drive if RETURN or LF
        {
        default:
        	return;

        case tTestingCurrentZero:
            // Current offset
            AC2_TEST_setCurrentOffset (((command == INCREASE) ? 1 : -1));
            break;

        case tTestingCurrentGain:
            AC2_TEST_setCurrentGain (((command == INCREASE) ? 1 : -1));
            break;

        case tTestingDrive:
        	if(command == CMD_RETURN || command == CMD_LINEFEED)
        	{
        		tDelay = 3;			// Force next drive
        	}
        	break;
        }

/*
        else if (currentTest() == 3)
        {
            // Current offset
            AC2_TEST_setCurrentOffset (((command == INCREASE) ? 1 : -1));
        }
        else if (currentTest() == 4)
        {
            // current gain
            AC2_TEST_setCurrentGain (((command == INCREASE) ? 1 : -1));
        }
*/
        // reset counter and flag
//        commandAvailable = 0;
//        rxCount = 0;
        // re-enable receiver
//        U1C1 |= 0x04;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
//void setLEDS (BYTE state);

static void AC2_TEST_updateLEDS(void)
{
    WORD cnt, mask;
    WORD control = 0; // all off

    // calculate the actual state at this point in time
    // set the update mask for the first LED
    mask = 0x01; // Diagnostic LED
    // get timer tick count
    // do the diagnostic & 3 tricolour LEDS

    for (cnt=0; cnt < LED_RELAY; cnt++)		// For each LED. 0 = Diagnostic
    {
    	switch (ledState[cnt])
        {
        case LED_OFF:
            break;

        case LED_ON:
            control |= mask;
            break;
        }
        //update mask for next LED
        mask <<= 1;
    }
    setLEDS (control);
}
//----------------------------------------------------------------------------------------------------------------------------------
static void Set_led_colour(LedIndex led, WORD colour)
{
	LedIndex led_green = led;
	LedIndex led_red = led+1;
	Turn_all_leds_off();
    switch (colour)
    {
    case 0:
    	setLedState (led_green, LED_ON);
        break;
    case 1:
    	setLedState (led_green, LED_ON);
    	setLedState (led_red, LED_ON);
        break;
    case 2:
    	setLedState (led_red, LED_ON);
        break;
    }
}
//------------------------------------------------------------------
#ifdef MH_DEBUG_RPM
extern WORD Set_prop_rpm;		// Defined in comms.c Debug check
int RPM_test_count;			// Debug
int AC2_check_rpm;
#endif
void AC2_TEST_stage(void)
{
    char buf[45];
//    WORD rpm;
    WORD cgain;
    int  coffset;
//    BYTE lc = LED_OFF;
//    BYTE lccount= 0;
	BYTE tCountmax=3;

    BYTE t = Trace("AC2_TEST_stage");
	if(AC210_4leds)
	{
		tCountmax=4;
//		lccountmax = 7;
	}


    // sequencing of tests
    AC2_TEST_checkForCommand();
    // detect mode changes
    updateOperatingMode ();
    AC2_TEST_updateLEDS ();


    // led toggle 1 Hz

/*	MHH:09/04/2019. I don't think this code does anything as lccount is set to zero on entry to routine
    if (!timerTick) {
        lccount++;
        if (lccount == 5) {
            lccount = 0;
            diagnosticDisplay (lc);
            if (!lc) {
                lc++;
            }
            else {
                lc <<= 1;
            }
            if (lc == 0x10) {
                lc = 0;
            }
        }
    }
*/
    if (!timerTick)
    {
        tTest++;
    }
    // Do the tests
    switch (htState)
    {
    case tIdle:
        break;

    case tTestingLEDS:
        if (!timerTick)
        {
            tCount++;
            tCount %= 4;
            switch (tCount)
            {
            case 0:
                // leds off
                txDebug ("LEDS off");
                Turn_all_leds_off();
                break;

            case 1:
                // leds on red
                txDebug ("LEDS RED");
                Turn_all_leds_off();
                Turn_red_leds_on();
                break;

            case 2:
                // leds on green
                txDebug ("LEDS GREEN");
                Turn_all_leds_off();
                Turn_green_leds_on();
                break;

            case 3:
                // leds on orange
                txDebug ("LEDS ORANGE");
                Turn_all_leds_on();
                break;
            }
        }
        break;

    case tTestingModes:
        setModeDebug (1);
        break;

    case tTestingCurrentZero:
        if (!timerTick)
        {
            // get motor current and dump it
            tTest = scaledValue (A_MOTOR_CURRENT);
            coffset = getCurrentOffset();
            sprintf(buf, "Zero (offset is %d) Current = %d", coffset, tTest);
            txDebug (buf);
        }
        break;

    case tTestingCurrentGain:
        if (!timerTick)
        {
            // get motor current and dump it
            tTest = scaledValue (A_MOTOR_CURRENT);
            cgain = getCurrentGain();
            sprintf(buf, "Gain (gain is %u) Current = %d", cgain, tTest);
            txDebug (buf);
        }
        break;

    case tTestingDrive:
        if (!timerTick)
        {
            // get motor current and dump it
            tTest = scaledValue (A_MOTOR_CURRENT);
            sprintf(buf, "Motor Current = %d, RETURN for next Drive", tTest);
            txDebug (buf);
            if (tDelay >= 3)		// MHH: 25/06/2018. Now set if CMD == return or LF.
//            if (++tDelay >= 3)
            {
                tDelay = 0;
                tCount++;
                tCount %= 6;
                switch (tCount)
                {
                case 0:
                    stopTheFeather();
                case 2:
                case 4:
                    setDrive (C_IDLE);
                    txDebug ("Drive Off");
                    break;
                case 1:
                    setDrive (C_COARSER);
                    txDebug ("Driving Coarse");
                    break;
                case 3:
                    setDrive (C_FINER);
                    txDebug ("Driving Fine");
                    break;
                case 5:
                    startTheFeather();
                    txDebug ("Driving Feather");
                    break;

                }
            }
        }

        break;

    case tTestingRPM:
        setRPMDebug (1);
        if (!timerTick)
        {
//        	magSpeed ((WORD *)&rpm);
        	Get_RPM();	// Note: Debug in routine is used to output stuff, rpm not used
#ifdef MH_DEBUG_RPM
        	AC2_check_rpm = 9999;
            if(RPM_test_count++ != 0)
            {
            	switch(tCount)
            	{
            	case 0:
            		AC2_check_rpm = 6000;
            		break;
            	case 1:
            		AC2_check_rpm = 3000;
            		break;
            	case 2:
            		AC2_check_rpm = 1500;
            		break;
            	}
            	if(AC2_check_rpm != rpm/10 && tDelay >0)
            	{
            		AC2_check_rpm = AC2_check_rpm;		// dummy for breakpoint
            	}
            }
#endif
            if (++tDelay >= 2)
            {
                tDelay = 0;
                tCount++;
                tCount %= 3;
                if (tCount == 0)
                {
                    Set_Prop_RPM(6000);
//                    TA1 = 2499;
                }
                else if (tCount == 1)
                {
                    Set_Prop_RPM(3000);
//                    TA1 = 4999;
                }
                else if (tCount == 2)
                {
                    Set_Prop_RPM(1500);
//                    TA1 = 9999;
                }
            }
        }
        break;

    case tTestingState:

        testingCall();
        break;

    case tTestingOther:
        setVoltageDebug(1);
        setDimmerDebug (1);
        // leds on to see effect of dimmer
        Turn_all_leds_on();
        if (!timerTick)
        {
            tTest = scaledValue (A_SUPPLY);
            tTest = scaledValue (A_DIMMER);
//            tTest = scaledValue (A_SPARE);		// inches of mercury. Not used at present
        }
        break;

    case tTestFinished:
        if ((timerTick % 25) == 0)
        {
            tCount++;
            tCount %= tCountmax;
            if (!tCount)
            {
                tDelay++;
                tDelay %=3;
            }
        }

        switch (tCount)
        {
        case 0:
        	Set_led_colour(LED_COARSE_G, tDelay);
/*
            Turn_all_leds_off();
            switch (tDelay)
            {
            case 0:
            	setLedState (LED_COARSE_G, LED_ON);
                break;
            case 1:
            	setLedState (LED_COARSE_G, LED_ON);
            	setLedState (LED_COARSE_R, LED_ON);
                break;
            case 2:
            	setLedState (LED_COARSE_R, LED_ON);
                break;
            }
*/
            break;
        case 1:
        	Set_led_colour(LED_FINE_G, tDelay);
            break;

        case 2:
        	Set_led_colour(LED_FEATHER3_G, tDelay);
            break;

        case 3:		// Only if 4 leds
        	Set_led_colour(LED_FEATHER4_G, tDelay);
            break;
        }

        break;
    }
    TraceReturn(t,"");
}
//------------------------------------------------------------
void AC2_check_tstate(int ifrom)
{
	TestState ts;
    ts = motorTestState ();

    if (ts != testIdle)
    {
        sprintf(PrintBuffer,"AC2_check_tstate:%d\r\n",ifrom);
    	Printf();
    	Printf();		// To be sure
    }
}

