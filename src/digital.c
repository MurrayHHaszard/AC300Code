/* ------------------------------------------------------------
Title:          digital.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   29/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Digital Input scanning and accessor routines

Changes:

------------------------------------------------------------ */

#include <string.h>
#include <stdio.h>

#include "global.h"
#include "drive.h"
#include "digital.h"
#include "sstate.h"
#include "leds.h"
#include "control.h"
#include "feather.h"
#include "param.h"
#include "rpm.h"
#include "analog.h"
#if REMOTE_VERSION
#include "remote.h"
#endif

#define NOMODE 0xFFFF

#ifndef AC210_PORT

// port used
#define MODEPORT    P4
#define MODEDIRPORT PD4

// Manual mode bit input
#define MANUAL_MODE_PORT P1
#define MANUAL_MODE_MASK 0x80
#endif
// mode mask
#define MODEMASK 0x003F

// manual key mask
#define MANUAL_KEY_MASK 0x01C0

/*
   NOTE.. The reversing version does not alter any internal modes. Internally
   it is still in Feather, and FeatherReverse is still triggered in the same way
*/

const char keyString[3][8] = {		// Will deal with FE/BE/RE later
    "Key  FE",
    "Key  CO",
    "Key  FI"
};

const char modeString[7][20] = {
    "Mode -> Manual\r\n",
    "Mode -> Feather\r\n",	// Will deal with FE/BE/RE later
    "Mode -> Hold\r\n",
    "Mode -> Cruise\r\n",
    "Mode -> Climb\r\n",
    "Mode -> TakeOff\r\n",
    "Mode -> Reverse\r\n"
};

// Data Storage
// "debounced" versions
WORD opMode;
WORD keysDown;

WORD newMode;
BYTE newModeCount;

// "raw" versions of inputs
WORD lastKeysDown;


// Half beta mode storage
// Set to 0 normally, 1 if speed checks passed and key sequence passed
// set to 0xFF if attempting to go into beta mode when over speed
BYTE halfWayToBeta = 0;

#if DEMO_VERSION
void setDemoDrive (BYTE dir, BYTE ticks);
#endif


// Initialisation call.
void initDigital (void)
{

    // As all these are inputs, we don't have to initialise any ports

//    opMode = MANUAL;
    opMode = STARTUP;		// MHH:18/11/2017
    keysDown = 0;
    lastKeysDown = 0;
    newMode = NOMODE;
    newModeCount = 0;
    // and thats it
}

#if BETA_VERSION
BYTE betaModePending (void)
{
    return (halfWayToBeta);
}
#endif

//--------------------------------------------------------------------------------------------
//#ifdef XOAR_VERSION
// Similar logic to below, except do not mask out TO,CLIMB,CRUISE,HOLD if MANUAL true
BYTE XoarGetSwitches(void)
{
	WORD localInputs;
	BYTE bit,match,i;
	BYTE rv=0;
	BYTE ln,rn;
#ifdef AC210_PORT
    localInputs = p_GetAC200_Switches(false);	// Don't mask rotary switch bits for Xoar
#else
    // get inputs (active low so invert)
    localInputs = ~MODEPORT;
    // and shift in the MANUAL mode bit at the bottom end

    localInputs <<= 1;

    if (!(MANUAL_MODE_PORT & MANUAL_MODE_MASK))
    {
//        localInputs &= MANUAL_KEY_MASK;
        localInputs |= 0x0001;
    }
#endif
    ln = 1;		// auto
    if(localInputs & 1) ln = 2;	// manual
/*
    The new bit sequence in localInput is:
    Bit 0	Empty, will be used for Manual Switch
    Bit 1	Rotary Feather
    Bit 2	Rotary Hold
    Bit 3	Rotary Cruise
    Bit 4	Rotary Climb
    Bit 5 	Rotary Takeoff
    Bit 6	Feather Switch			// maps to MANUAL_KEY_FEATHER
    Bit 7	Coarse Sense			// maps to MANUAL_KEY_COARSE
    Bit 8	Fine Sense				// maps to MANUAL_KEY_FINE
*/
    bit = (1<<5);	// should correspond to takeoff
    match = 0;
    for(i=1;i<6;i++) {
    	if(bit & localInputs) {
    		match = i;
    		break;
    	}
    	bit >>= 1;
    }
    rn = match;
    rv = (ln << 4) | rn;
    return rv;
}


#ifdef AC2_TEST		// Used by AC2_TEST
void dumpCurrentMode (void) {
    txDebug2 (modeString[opMode]);
}
#endif
//#endif

// repeated call, scans input lines and determines current Mode
// also updates manual keys pressed

// Mode changes are not acted on for 0.5 second.. This is to allow for
// a change from MANUAL to CRUISE without incurring motor activity

//-----------------------------------------------------------------------------
// This routine added to handle beta/reversing integration
const far char *Get_modeString(int m)
{
    const far char *desc;

    // if this is a new state, log it
	desc = modeString[m];

	if(m == 1)
	{
	    if(Control_type == CT_REVERSE) desc = "Mode -> Reverse\r\n";
        if(Control_type == CT_BETA) desc = "Mode -> Beta\r\n";
	}
	return desc;
}
//-----------------------------------------------------------------------------
WORD AC200_debounce_input(WORD localInputs)
{
	static WORD AC200_inputs;
	static WORD last_inputs;
	if(localInputs == last_inputs)		// Only update switch value if it repeats over 2 cycles. 31/01/2018
	{
		AC200_inputs = last_inputs;
	}
	last_inputs = localInputs;
	return AC200_inputs;
}
//------------------------------------------------------------------------------
WORD Map_cnt_to_mode2(int cnt)
{
	WORD cw =  getParameter(AC_CONTROL_WORD);
	if(getParameter(LED4_ENABLE) != 1)		// No extra mode?
	{
		if(cnt < 2) return INVALID;
		if(cnt > 6) return INVALID;
		if(cnt > 2) return (cnt - 1);
		// If we drop through then cnt = 2. This could mean Feather, Beta or Reverse, depending on cw settings.

		if(cw & CW_BETA_REVERSING)
		{
			if(cw & CW_NON_BETA_REVERSING) return INVALID;
			return BETA;
		}
		if(cw & CW_NON_BETA_REVERSING)
		{
			if(cw & CW_BETA_REVERSING) return INVALID;
			return REVERSE;
		}
		if(cw & CW_NON_FEATHERING)
		{
			return INVALID;
		}
		return FEATHER;

	}

	// If we drop through then we have an extra LED and position in mode switch.

	if(cnt == 0) return FEATHER;
	if(cnt < 2) return INVALID;
	if(cnt > 6) return INVALID;

	if(cnt > 2) return (cnt - 1);
	// If we drop through then cnt = 2. This could mean Beta or Reverse, depending on cw settings.
	if(cw & CW_BETA_REVERSING)
	{
		if(cw & CW_NON_BETA_REVERSING) return INVALID;
		return BETA;
	}

	if(cw & CW_NON_BETA_REVERSING)
	{
		if(cw & CW_BETA_REVERSING) return INVALID;
		return REVERSE;
	}
	return INVALID;
}
WORD Rotary_switch_mode;
WORD Map_localinputs_to_mode(WORD localinputs)
{
    // the mode is the bit position
	int cnt=0;
	WORD li = localinputs;
    while (li)
    {
        li >>= 1;
        cnt++;
    }
	Rotary_switch_mode = Map_cnt_to_mode2(cnt);
	if(localinputs & 1) return MANUAL;
	return Rotary_switch_mode;
}
//-----------------------------------------------------------------------------
void Set_fstate(FeatherState new_fstate);
void updateOperatingMode (void)
{
    char buf[40];
    const far char *desc;
    WORD localInputs, manKeys, mask, cnt;
    WORD localMode;
    TestState ts;
    WORD state;
    BYTE switchToBeta = 0;
    BYTE featheringProp;
    WORD rpm;


#if MAP_VERSION
    mapEnabled = manifoldPressureEnabled();
#else
//    mapEnabled = 0;
#endif

    featheringProp = isFeatheringOrReversingProp() || isBetaProp();
#ifdef AC210_PORT
    localInputs = p_GetAC200_Switches(false);
#else
    // get inputs (active low so invert)
    localInputs = ~MODEPORT;
    // and shift in the MANUAL mode bit at the bottom end

    localInputs <<= 1;

    if (!(MANUAL_MODE_PORT & MANUAL_MODE_MASK))
    {
        localInputs &= MANUAL_KEY_MASK;
        localInputs |= 0x0001;
    }
#endif

    localInputs = AC200_debounce_input(localInputs);

    // manual keys are always updated
    manKeys = localInputs & MANUAL_KEY_MASK;
    // shift into lower bit position
    manKeys >>= 6;

    // Fiddle with the bits for a non-feathering version
    // dont allow a feather selection at the rotary switch,
    // and mask out the feather manual switch
    // this code modified, feather can become MAP
    if (!featheringProp)
    {
        manKeys &= ~MANUAL_KEY_FEATHER;
    }


    // Ignore the testing during a motor state test, however,
    // as this does weird things to the switch monitoring
    ts = motorTestState ();

    if ((ts == testIdle) || (ts == testFinish))
    {

        for (mask = MANUAL_KEY_FEATHER,cnt=0; cnt < 3; cnt++)
        {
            // do the test.......
            if (((manKeys & mask)^(keysDown & mask)))
            {
                // Change.
                if (debug & DEBUG_SWITCH_STATE)
                {
                    // if this is a new state, log it
                	desc = keyString[cnt];

/*
 *
 * #if REVERSING_VERSION
    "Key  RE",
#elif BETA_VERSION
    "Key  BE",
#else
 *
 *
 */
                	if(cnt == 0)
                	{
                	    if(Control_type == CT_REVERSE) desc = "Key  RE";
                        if(Control_type == CT_BETA) desc = "Key  BE";
                	}

                    sprintf (buf, "%s -> %d\r\n",
                             desc,
                             ((manKeys & mask) ? 1 : 0) );
                    txDebug (buf);
                }
                keysDown &= ~mask;
                keysDown |= (manKeys & mask);
            }
            else
            {
                // There is a change,leave the original as is for now

            }
            mask <<= 1;
        }

        // update lastKeysDown
        lastKeysDown = manKeys;
    }


    // keep just the mode bits
    localInputs &= MODEMASK;


    // the mode is the bit position
#ifdef MH_XXX
    cnt=0;
    while (localInputs)
    {
        localInputs >>= 1;
        cnt++;
    }
#endif
//    if(getParameter (RC_BOARD) == 1)	// MHH:30/09/2023. Special Remote Control Board?
    if(AC210_remote_control_board)    // MHH:03/11/2023
    {
    	localMode = HOLD;
    }
    else
    {
    	/*
    	 *  if FeatherAndReverseProp then
    	 *    if new most anti_clockwise position then
    	 *       position = feather
    	 *    else
    	 *       if old_feather_position then
    	 *         position = reverse
    	 *       end if
    	 *    end if
    	 *  end if
    	 *
    	 */
    	localMode = Map_localinputs_to_mode(localInputs);
    	if(localMode == INVALID) return;

    }
#ifdef MH_REVERSE_RELAY
    AC210_check_start_reverse();
#endif


    // Fiddle with the mode.. Rules are
    // 1. If feathering && MAP, then HOLD == MAP, CRUISE == HOLD
    // 2. If non feathering and MAP, then FEATHER == MAP
    // 3. Beta mode doesnt mix with MAP at the moment

#if REVERSING_VERSION
    if(Control_type == CT_REVERSE)
    {
        // we don't want feather mode to be feather mode
		if(RemoteCommsType == REMOTE_COMMS_LEGACY)	// MHH:01/09/2018
		{
	    	if (localMode == FEATHER)
	        {
	            if (remoteMode() > REMOTE_IDLE)
	            {
	                localMode = HOLD;
	            }
	        }
		}
    }
#endif

#if BETA_VERSION

    // New system for getting into mode.
    // 1. Switch to Beta
    // 2. Trigger Beta switch, this gets to halfway mode
    // 3. Switch to Manual, this goes into full beta

//    if (isBetaProp())
#ifdef MH_XXX    // MHH:17/07/2025
    if(Rotary_switch_mode != BETA)
    {
    	if (fState == F_BETA_IDLE) Set_fstate(F_IDLE);
    }
#endif
//    if(Rotary_switch_mode == BETA)
    if(opMode != BETA)
    {
    	if (fState == F_BETA_IDLE) Set_fstate(F_IDLE);
    }
    else
//    if(opMode == BETA)
    {
//        if ((opMode == FEATHER) &&		// MHH:15/07/2025

    	if (fState == F_IDLE) Set_fstate(F_BETA_IDLE);
    	if(opMode == BETA)	// MHH:08/04/2026
    	{
            if (getParameter(BETA_CHK_RPM))
            {
                int maxrpm;
                if(dState == MD_BETA)
                {
                	maxrpm = getParameter(BETA_MAX_RPM_ENGAGED);
                }
                else
                {
                	maxrpm = getParameter(BETA_MAX_RPM);
                }
                rpm = Get_RPM();
                if(rpm >= maxrpm) LED_overlay_flags |= LED_OVERLAY_FLAG_BETA_MAX_RPM;
            }
    	}

    	if ((opMode == BETA) &&
            ((halfWayToBeta == 0) || (halfWayToBeta == 0xFF)) &&
            (manKeys & MANUAL_KEY_FEATHER))
        {
        	// Check the speed if required
//    		LED_overlay_flags &= ~LED_OVERLAY_FLAG_BETA_MAX_RPM;
            if (getParameter(BETA_CHK_RPM))
            {
                // Get current RPM
                rpm = Get_RPM();

//                if (rpm < getParameter(BETA_MAX_RPM)*RPMFACTOR)

                if (rpm < getParameter(BETA_MAX_RPM))
                {
                	halfWayToBeta = 1;
                }
                else
                {
//                	LED_overlay_flags |= LED_OVERLAY_FLAG_BETA_MAX_RPM;
                	halfWayToBeta = 0xFF;
                }
            }
            else
            {
            	halfWayToBeta = 1;
            }

        }
        else if ((localMode == MANUAL) &&
                 (opMode == BETA) &&		// MHH:15/07/2025
                 (halfWayToBeta == 1))
        {
            // set mode to BETA after a couple of
            // configurable checks
            switchToBeta = 0;
            if (getParameter(BETA_CHK_SWITCH))
            {
                //
                if (betaSwitchOn())
                {
                    switchToBeta += 1;
                }
            }
            else
            {
                switchToBeta += 1;
            }
            if (getParameter(BETA_CHK_RPM))
            {
                // Get current RPM
            	rpm = Get_RPM();

//                if (rpm < getParameter(BETA_MAX_RPM)*RPMFACTOR)
                if (rpm < getParameter(BETA_MAX_RPM))
                {
                    switchToBeta += 1;
                }
            }
            else
            {
                switchToBeta += 1;
            }
            if (switchToBeta == 2)
            {
                Feather_mode = FM_BETA;
            	enterBetaMode();
            }
            // immediate change
            if (debug & DEBUG_SWITCH_STATE)
            {
                // if this is a new state, log it
                desc = Get_modeString(localMode);
            	txDebug (desc);
            }
            newMode = NOMODE;
            newModeCount = 0;
            // Reset overcurrent trip to clear display
            if (betaOverCurrent == 0)
            {
                overCurrentTrip = 0;
            }
            opMode = localMode;
            halfWayToBeta = 0;
        }
        else if (((halfWayToBeta == 1) || (halfWayToBeta == 0xFF)) &&
                 (opMode == BETA) &&
                 (localMode != BETA))
        {
            halfWayToBeta = 0;
        }
    }


    // and check for exit
    if(opMode == MANUAL)
    {
        if(localMode == BETA)
        {
        	mh_debug();
        }

    	if ((localMode != MANUAL) && (featherState() == F_BETA))
        {
            // Need to trigger the exit
            exitBetaMode();
            // but don't change state yet
            localMode = opMode;
        }
        else if ((localMode != MANUAL) && (featherState() == F_BETA_EXIT))
        {
            // need to avoid state change until reverse beta has finished
            localMode = opMode;
        }
   }

#endif
    // mode changes are delayed for 0.5 second, UNLESS we are changing to manual

    if ((localMode == MANUAL) && (localMode != opMode))
    {

#if REMOTE_VERSION
        // switching to manual will force termination of remote mode
        setRemoteLockout();
#endif
        // immediate change
        if (debug & DEBUG_SWITCH_STATE)
        {
            // if this is a new state, log it
            desc = Get_modeString(localMode);
            txDebug (desc);
        }
        newMode = NOMODE;
        newModeCount = 0;
        // Reset overcurrent trip to clear display
#if BETA_VERSION
        if(Control_type == CT_BETA)
        {
            // only if not a beta overcurrent
            if (betaOverCurrent == 0)
            	overCurrentTrip = 0;
        }
        else
        {
        	overCurrentTrip = 0;
        }
#endif
/*
        // only if not a beta overcurrent
        if (betaOverCurrent == 0)
        {
#endif
        	overCurrentTrip = 0;
#if BETA_VERSION
        }
#endif
*/
        opMode = localMode;
//        Reverse_position = false;
#if DEMO_VERSION
        // stop demo drive now
        setDemoDrive (0,0);
#endif
    }
    else if (localMode != opMode)
    {
        // delayed change
        if (localMode != newMode)
        {
            newMode = localMode;
            if (!newModeCount)
            {
                newModeCount = 25;
            }
        }
    }
    else
    {
        // no change..
        newMode = NOMODE;
        newModeCount = 0;
    }

    // Check to see if we are updating a new mode yet
    if (newModeCount)
    {
        if (--newModeCount == 0)
        {
            if (newMode != NOMODE)
            {

#if DEMO_VERSION
                if ((newMode != MANUAL) && (opMode != MANUAL) &&
                    (newMode !=FEATHER) && (opMode != FEATHER))
                {
                    if (newMode > opMode)
                    {
                        // switching down.. go fine
                        setDemoDrive (1,100);
                    }
                    else
                    {
                        // switching up.. go coarse
                        setDemoDrive (2,100);
                    }
                }
#endif
                if (debug & DEBUG_SWITCH_STATE)
                {
                    // if this is a new state, log it
                    desc = Get_modeString(newMode);
                    txDebug2 (desc);
                }
                // Reset Overcurrent Trip flag if changing from manual back to
                // an auto mode
                if ((opMode == MANUAL) && (newMode != MANUAL))
                {
#if REMOTE_VERSION
                    // clear our remote override
                    clearRemoteLockout();
#endif
#if BETA_VERSION
                    if(Control_type == CT_BETA)
                    {
                        // only if not a beta overcurrent
                        if (betaOverCurrent == 0)
                        {
                        	overCurrentTrip = 0;
                        	Set_overcurrent_dir(0);
//                        	overCurrentDir = 0;
                        }
                    }
                    else
                    {
                    	overCurrentTrip = 0;
                    	Set_overcurrent_dir(0);
//                    	overCurrentDir = 0;
                    }
#endif
/*

                    if (betaOverCurrent == 0)
                    {
#endif
                    	overCurrentTrip = 0;
                    	overCurrentDir = 0;
#if BETA_VERSION
                    }
#endif
*/
                }
//#if (BETA_VERSION == 0)
//                if(Control_type != CT_BETA)
                {
                    // Check to see whether we need to exit a feathered or reversed prop
                    // Beta mode exit is done in feather.c
            		state = systemState();
                	if(Sig100_connected)
                	{
                		if(newMode != MANUAL)
                		{
                			if(opMode == FEATHER && newMode != FEATHER)
                			{
                				if ((state & S_STOP_FEATHER) || (state & S_STOP_COARSE) || Feather_mode == FM_FEATHERING)
                				{
                					//                                    if(Get_RPM()< getParameter (MIN_ENGINE_SPEED))	// MHH:11/08/2021, looks unnecessary.
                					Feather_mode = FM_UNFEATHERING;
                					doReverseFeather();
                				}
                			}
                			if(opMode == REVERSE && newMode != REVERSE)
                			{
                				if ((state & S_STOP_REVERSE) || (state & S_STOP_FINE) || Feather_mode == FM_REVERSING)
                				{
                					Feather_mode = FM_UNREVERSING;
                					doReverseFeather();
                				}
                			}
                		}
                        // Modify the state
                         opMode = newMode;
                         newMode = NOMODE;
                         return;
                	}
                	bool exit_state=false;
//                	WORD feather_mode = Get_param_Feather_mode();
//                	if(feather_mode == FM_FEATHERING && newMode != FEATHER) exit_state=true;
                	if(opMode == FEATHER && newMode != FEATHER) exit_state=true;
//                	if(Control_type == CT_FEATHERING && newMode != FEATHER) exit_state=true;
//                	if(feather_mode == FM_REVERSING && newMode != REVERSE) exit_state=true;
                	if(opMode == REVERSE && newMode != REVERSE) exit_state=true;


                	if ((newMode != MANUAL) && (exit_state))
                		//                    if ((newMode != MANUAL) && (newMode != FEATHER))
                	{
                		if (featheringProp)
                		{
                			//                			{
                			if(Control_type == CT_REVERSE)
                			{
                				if(In_Reverse_Zone())	// MHH:03/04/2018
                				{
                					Feather_mode = FM_UNREVERSING;
                					doReverseFeather();
                				}
                			}
                			else
                			{
                				if ((state & S_STOP_FEATHER) || (state & S_STOP_COARSE))
                				{
                					//                                    if(Get_RPM()< getParameter (MIN_ENGINE_SPEED))	// MHH:11/08/2021, looks unnecessary.
                					Feather_mode = FM_UNFEATHERING;
                					doReverseFeather();
                				}

                			}

                		}
                		/*
                		 * Replace commented logic.
                            if ( (state & S_STOP_FEATHER) ||
    #if REVERSING_VERSION
                                 (state & S_STOP_FINE)
    #else
                                 (state & S_STOP_COARSE)
    #endif
                                                         )
                            {
                                doReverseFeather();
                            }
                		 */
                		//              		}
                	}

                }
                //#endif
                // Modify the state
                opMode = newMode;
                newMode = NOMODE;
            }
        }
    }
    // all done
}

// accessor function
OpMode operatingMode (void)
{
    return (opMode);
}

// manual keys pressed
BYTE manualKeys (void)
{
    return (keysDown);
}

bool Reverse_front_switch_on(void)
{
	return ((keysDown & MANUAL_KEY_FEATHER) != 0);
}

