/* ------------------------------------------------------------
Title:          feather.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Feathering control and state code

Changes:


------------------------------------------------------------ */

#include <string.h>
#include <stdio.h>

#include "global.h"
#include "analog.h"
#include "feather.h"
#include "digital.h"
#include "sstate.h"
#include "drive.h"
#include "rpm.h"
#include "param.h"
#include "leds.h"
#include "remote.h"



const char fStateName[7][10] = {
    "Idle",
    "Active",
    "Complete",
    "Reversing",
    "BetaIdle",
    "Beta",
    "BetaExit"
};

typedef enum
{
    uCheckState0 = 0,
    uDrive1,
    uCheckState1,
    uDrive2
} UnfeatherState;

// data storage
FeatherState fState;

// for getting back out of feather. Drive for ?? seconds max..
WORD reverseTimer, reverseTimerSet;
UnfeatherState reverseState;

BYTE feather;

BYTE inBetaHold = 0;

// local procedures
void startFeather (void);
void stopFeather (void);
void reverseFeather (UnfeatherState rState);
void reverseFeatherProc (void);
void reverseBetaProc (void);

// initialisation call

//----------------------------------------------------------------------------
void Set_fstate(FeatherState new_fstate)
{
	if(fState == new_fstate) return;

	if(new_fstate == F_IDLE)
	{
		fState = F_IDLE;		// debug line
	}
	fState = new_fstate;
}
//----------------------------------------------------------------------------
void initFeather (void)
{
	feather =  isFeatheringOrReversingProp() || isBetaProp();
	Set_fstate(F_IDLE);

#ifdef MH_XXX  	// MHH:16/07/2025

	feather =  isFeatheringOrReversingProp() || isBetaProp();
#if BETA_VERSION
    if(Control_type == CT_BETA)
    {
        Set_fstate(F_BETA_IDLE);
    }
    else
    {
        Set_fstate(F_IDLE);
    }
//#elsefState FM_ dState Feather_mode
#endif
#endif
}
//--------------------------------------------------------------------------
static void Debug_feather_state(void)
{
    char buf[30];
    const char far *fstate_name;
	fstate_name = fStateName[fState];
    switch(Control_type)
    {
    case CT_REVERSE:
        sprintf (buf,"Reverse-> %s\r\n",fstate_name);
        break;

    case CT_BETA:
        sprintf (buf,"Beta   -> %s\r\n",fstate_name);
        break;

    default:
        sprintf (buf,"Feather-> %s\r\n",fstate_name);
        break;
    }
    txDebug (buf);
}
//-----------------------------------------------------------------------------------
BYTE In_Reverse_Zone(void)
{
	WORD state;
	if(Control_type != CT_REVERSE) return false;

	state = systemState();
	if(state & S_STOP_FEATHER)
	{
		return true;
	}

	if(state & S_STOP_FINE)		// MHH:16/07/2019
	{
		return true;
	}

	if(state & S_STOP_REVERSE)		// MHH:16/07/2019
	{
		return true;
	}


/*
	if(state & S_STOP_FINE)
	{
		if((LED_overlay_flags & LED_OVERLAY_FLAG_REVERSE_ZONE) == false)	// Debug!!!
	    {
	    	return false;
	    }
		return true;
	}
*/
    if(LED_overlay_flags & LED_OVERLAY_FLAG_REVERSE_ZONE)
    {
    	return true;
    }
    return false;
}
//-----------------------------------------------------------------------------------
BYTE Reverse_engage_rpm_too_high(void)
{
    WORD cSpeed;
    WORD rpm;

    if (getParameter(BETA_CHK_RPM) == false)
    {
    	return false;
    }

	WORD engage_max_rpm = getParameter(BETA_MAX_RPM);
	if(engage_max_rpm == 0)
	{
		return false;
	}
//    magSpeed ((WORD*)&cSpeed);
    cSpeed = Get_RPM();
//    rpm = (WORD)cSpeed/RPMFACTOR;		// Get rid of decimal place
    rpm = (WORD)cSpeed;		// Get rid of decimal place
    if(rpm > engage_max_rpm)
    {
		LED_error = LED_ERROR_REVERSE;
    	return true;
    }
	return false;
}
//-----------------------------------------------------------------------------------
BYTE Reverse_RPM_too_high(void)		// MHH:16/03/2018
{
    if(Control_type != CT_REVERSE) return false;

    return Reverse_engage_rpm_too_high();
}
//--------------------------------------------------------------------------
//bool Feather_and_Reverse_Prop=true;		// MHH:05/02/2019
//bool Reverse_position;
BYTE Reverse_position = FEATHER;		// Default
#ifdef MH_REVERSE_RELAY
bool Reverse_relay_on;

// Only called if zone is set to normal

void Reverse_relay_set_control_type(void)
{
	WORD cw = getParameter (AC_CONTROL_WORD);
	if(cw & CW_NON_FEATHERING)
	{
		Control_type = CT_NON_FEATHERING;
	}
	else
	{
		Control_type = CT_FEATHERING;
	}
}
//--------------------------------------------------------------------------
void AC210_reverse_relay_off(void)
{
	if(Reverse_relay_on)
	{
//		Board_LED_Set(7,false);		// Set reverse relay off
		AC210_RELAY_Set(false);

/*
 *  Changing Control_type to REVERSING when relay is on, then back to FEATHER or NON_FEATHER when relay off.
 *
 */
		Reverse_relay_set_control_type();
//				Control_type = CT_FEATHERING;
		Reverse_relay_on = false;
	}
}
//--------------------------------------------------------------------------
void AC210_check_reverse_relay(void)
{
   	if(isReverseRelay())
   	{
   		if(operatingMode() != Reverse_position)
   		{
   			AC210_reverse_relay_off();
   		}
   	}
}
//--------------------------------------------------------------------------
BYTE Reverse_enable_pin(void)
{
    if(AC210_4leds)
    {
    	WORD dval = scaledValue (A_DIMMER);
        if(dval < 8)
        {
        	return false;	// At least 8V
        }
        return true;
    }

    // If we drop through then assume using REVERSE switch on front panel.

	return Reverse_front_switch_on();
}
//--------------------------------------------------------------------------
typedef enum
{
    RS_START=0,
    RS_TEST_ENABLE,
	RS_RELAY_ON,
	RS_RELAY_ENABLED,
	RS_DISABLED
} RS_state;

RS_state Reverse_state;
uint16_t RS_tick_count;
uint16_t RS_try_count;
void AC210_check_start_reverse(void)
{
	if(isReverseRelay() == false)
	{
		return;
	}
	if(operatingMode() != Reverse_position)
	{
		Reverse_state = RS_START;

		if(operatingMode() == MANUAL)
		{
   			AC210_reverse_relay_off();
		}

		if(fState == F_IDLE)
		{
			if((systemState() & S_STOP_FINE) == 0)
			{
		    	Set_Zone(RC_S_ZONE_NORMAL);
			}
		}
		RS_tick_count = 0;
		if(operatingMode() != MANUAL)	// MHH:02/12/2022. Added this test because feather LED went RED when manual feather key on in manual mode
		{
			if(Reverse_enable_pin())
			{
				LED_overlay_flags |= LED_OVERLAY_FLAG_REVERSE_BUTTON_ON;
			}
		}
		return;
	}
	switch(Reverse_state)
	{
	case RS_START:
		if(Reverse_enable_pin() == 0)	// Check off for a fifth of a second at start of sequence
		{
			if(RS_tick_count++ > 10)
			{
				Reverse_state = RS_TEST_ENABLE;
				RS_tick_count = 0;
				RS_try_count = 0;
			}
		}
		else
		{
			Reverse_state = RS_DISABLED;
		}
		return;

	case RS_TEST_ENABLE:
		if(Reverse_enable_pin())
		{
			if(Reverse_engage_rpm_too_high() == false)
			{
				LED_overlay_flags |= LED_OVERLAY_FLAG_REVERSE_BUTTON_ON;
				if(RS_tick_count++ > 10)	// Must be on for at least one fifth of a second.
				{
					AC210_RELAY_Set(true);
//					Board_LED_Set(7,true);		// Set reverse relay on
					Reverse_state = RS_RELAY_ON;
					RS_tick_count = 0;
					RS_try_count = 0;
				}
			}
		}
		else
		{
			if(RS_tick_count > 0)
			{
				RS_tick_count = 0;
				if(RS_try_count++ > 3)
				{
					Reverse_state = RS_DISABLED;
				}
			}
		}
		return;

	case RS_RELAY_ON:
		if(Reverse_enable_pin() == false)
		{
			if(RS_tick_count++ > 10)
			{
				Reverse_state = RS_RELAY_ENABLED;
				Control_type = CT_REVERSE;	// Hopefully will allow normal reverse logic to work
				Reverse_relay_on = true;
				RS_tick_count = 0;
			}
		}
		else
		{
			if(RS_tick_count > 0)
			{
				RS_tick_count = 0;
				if(RS_try_count++ > 3)
				{
					AC210_reverse_relay_off();
					Reverse_state = RS_DISABLED;
				}
			}
		}
		return;

	case RS_RELAY_ENABLED:
/*
 *
 * This logic was for toggling out of reverse using reverse switch, but getting too complicated.
 * If we do implement, will need to ensure a wait state before getting back to start.
 *
		if(RS_tick_count < 100)		// minimum of 2 secs
		{
			RS_tick_count++;
		}
		else
		{
			if(Reverse_enable_pin())	// Pin toggled?
			{
            	if(In_Reverse_Zone())	// MHH:03/04/2018
            	{
                    doReverseFeather();
            	}
            	else
            	{
            		AC210_reverse_relay_off();	// MHH:22/03/2019
            	}
			}
			Reverse_state = RS_START;
		}
*/
		return;

	case RS_DISABLED:
		return;
	}

/*
//   	if(Feather_and_Reverse_Prop)
	if(isReverseRelay())
   	{
		if(operatingMode() == Reverse_position)
   		{
   			if(Control_type != CT_REVERSE)
   			{
   				if(Reverse_enable_pin())
   				{
   					if(Reverse_engage_rpm_too_high())
   					{
   						return;
   					}
					AC210_RELAY_Set(true);
//   					Board_LED_Set(7,true);		// Set reverse relay on
   					Control_type = CT_REVERSE;	// Hopefully will allow normal reverse logic to work
   					Reverse_relay_on = true;
   				}
   			}
   		}
   	}
*/
}
#else
void AC210_check_reverse_relay(void){};
void AC210_reverse_relay_off(void){};
#endif
//--------------------------------------------------------------------------
BYTE AutoGyro_enable_pin(void)
{
    WORD dval = scaledValue (A_DIMMER);
    if(dval < 8)
    {
    	return false;	// At least 8V
    }
    return true;
}
//--------------------------------------------------------------------------
BYTE AutoGyro_mode_change(void)
{
	if (operatingMode() == FEATHER) return false;

    if (operatingMode() == MANUAL)
    {
        stopFeather();
        Set_fstate(F_IDLE);
    }
    else
    {
        // turn off the FET
        stopFeather();
        // and reverse  with check first
        reverseFeather(0);
    }
    return true;
}
//--------------------------------------------------------------------------
BYTE AutoGyro_reverse_enabled(void)
{
	if(AutoGyro_enable_pin() == false)
	{
//		LED_error = LED_ERROR_REVERSE;
		return false;
	}
	return true;
}
//--------------------------------------------------------------------------
BYTE AutoGyro_wait_keys(void)
{
	if (manualKeys() & MANUAL_KEY_FEATHER)
    {
    	if(Reverse_RPM_too_high())
    	{
    		return false;
    	}
#ifdef MH_ENABLE_PIN_OLD
    	if(AutoGyro_reverse_enabled() == false) return false;
#endif
    	//		AutoGyro_state = AUTOGYRO_REVERSING;
    	// start feathering
//        Feather_mode = FM_IDLE;
        Feather_mode = FM_REVERSING;
        startFeather();
        return true;
    }
	return false;
}
//--------------------------------------------------------------------------
BYTE AutoGyro_reverse_control(void)
{
/*
 *
#define AUTOGYRO_IDLE		0
#define AUTOGYRO_REVERSING	1
#define AUTOGYRO_REVERSING_STOPPED 2
#define AUTOGYRO_WAIT_KEYS	3
#define AUTOGYRO_LATCHED	4
#define AUTOGYRO_TAKEOFF	5
#define AUTOGYRO_EXIT		6
 *
 */
	OpMode mode;
	BYTE autogyro_enable_pin = AutoGyro_enable_pin();

	// Use remote LED signal - technically the rotor-controller is remote.

	if(autogyro_enable_pin)
	{
		LED_overlay_flags |= LED_OVERLAY_FLAG_AUTOGYRO_ACTIVE;
	}
	else
	{
		LED_overlay_flags &= ~LED_OVERLAY_FLAG_AUTOGYRO_ACTIVE;
	}
	if((AutoGyro_state == AUTOGYRO_LATCHED) || (AutoGyro_state == AUTOGYRO_LATCHED_PIN))
	{
		LED_overlay_flags |= LED_OVERLAY_FLAG_AUTOGYRO_LATCHED;
	}
	else
	{
		LED_overlay_flags &= ~LED_OVERLAY_FLAG_AUTOGYRO_LATCHED;
	}

    if (systemState() & S_STOP_COARSE)	// This should never happen, but to be safe.
    {
    	if((fState != F_IDLE) || (AutoGyro_state != AUTOGYRO_IDLE))	// MHH: 02/04/2018. Extra test
    	{
        	Set_fstate(F_IDLE);
        	Set_Zone(RC_S_ZONE_NORMAL);
            reverseTimer = 0;
            cancelFeatherDrive();
            clearUnfeatherCheck();
            AutoGyro_state = AUTOGYRO_IDLE;
    	}
    }

	switch(AutoGyro_state)
	{
	case AUTOGYRO_IDLE:
        if (operatingMode() == FEATHER)
        {
        	if(AutoGyro_wait_keys())
        	{
        		AutoGyro_state = AUTOGYRO_REVERSING;
        	}
//            if (systemState() & S_STOP_FEATHER)		// Can get here on a restart
            if (systemState() & S_STOP_REVERSE)		//MHH:19/08/2019 Can get here on a restart
            {
        		AutoGyro_state = AUTOGYRO_REVERSING_STOPPED;
            	RC_X_last_action = RC_X_REVERSE_FINE;

        		// feather stop set
                stopFeather();
                Set_fstate(F_COMPLETE);
            }
        }
        break;

	case AUTOGYRO_REVERSING:
		if(AutoGyro_mode_change())
		{
			AutoGyro_state = AUTOGYRO_EXIT;
			break;
		}
#ifdef MH_ENABLE_PIN_OLD
    	if(autogyro_enable_pin == false)
    	{
    		stopFeather();
    		AutoGyro_state = AUTOGYRO_WAIT_KEYS;	// No, wait for keys
    		break;
    	}
#endif
//        if (systemState() & S_STOP_FEATHER)
        if (systemState() & S_STOP_REVERSE)		//MHH:19/08/2019 Can get here on a restart
        {
    		AutoGyro_state = AUTOGYRO_REVERSING_STOPPED;
        	RC_X_last_action = RC_X_REVERSE_FINE;

    		// feather stop set
            stopFeather();
            Set_fstate(F_COMPLETE);
        }
        break;

        // Here because stopped. Need to know if stopped because reverse key not pressed, or reached reverse stop

	case AUTOGYRO_REVERSING_STOPPED:
		if(AutoGyro_mode_change())
		{
			AutoGyro_state = AUTOGYRO_EXIT;
			break;
		}
#ifdef MH_ENABLE_PIN_OLD
    	if(autogyro_enable_pin == false)
    	{
    		stopFeather();
    		AutoGyro_state = AUTOGYRO_WAIT_KEYS;	// No, wait for keys
    		break;
    	}
#endif

    	// fState is F_ACTIVE here, not sure what is turning it on.
    	// Try setting to F_COMPLETE, but this may assume STOP_FEATHER.
    	Set_fstate(F_COMPLETE);	// Is this setting

		if(RC_X_last_action) break;		// Waiting for a state test to finish

//        if ((systemState() & S_STOP_FEATHER) == 0)	// Are we at reverse stop?
      if ((systemState() & S_STOP_REVERSE) == 0)		//MHH:19/08/2019 Can get here on a restart
      {
    		AutoGyro_state = AUTOGYRO_WAIT_KEYS;	// No, wait for keys
    		break;
        }

        // Looks like we got to REVERSE stop.

		AutoGyro_state = AUTOGYRO_AT_REVERSE_STOP;
        stopFeather();
        Set_fstate(F_COMPLETE);		// Just in case feather logic is looking for it.
        break;

	case AUTOGYRO_WAIT_KEYS:
		if(AutoGyro_mode_change())
		{
			AutoGyro_state = AUTOGYRO_EXIT;
			break;
		}
#ifdef MH_ENABLE_PIN_OLD
		if(autogyro_enable_pin == false)
    	{
    		stopFeather();
    		break;
    	}
#endif
		if(AutoGyro_wait_keys())
    	{
    		AutoGyro_state = AUTOGYRO_REVERSING;
    	}
		break;

// OK, we are at the reverse stop. Now wait for the manual reverse key to stop being pressed, then we are in first latch state

	case AUTOGYRO_AT_REVERSE_STOP:
		if(AutoGyro_mode_change())
		{
			AutoGyro_state = AUTOGYRO_EXIT;
			break;
		}
		if ((manualKeys() & MANUAL_KEY_FEATHER) == false)
		{
    		AutoGyro_state = AUTOGYRO_LATCHED;
		}
		break;

	case AUTOGYRO_UNLATCHED:
		if ((manualKeys() & MANUAL_KEY_FEATHER) == false)
		{
    		AutoGyro_state = AUTOGYRO_WAIT_KEYS;
		}
		break;


	case AUTOGYRO_LATCHED:
		if (manualKeys() & MANUAL_KEY_FEATHER)
		{
    		AutoGyro_state = AUTOGYRO_UNLATCHED;
    		break;
		}
    	if(autogyro_enable_pin)
    	{
    		AutoGyro_state = AUTOGYRO_LATCHED_PIN;
    	}
		break;

	case AUTOGYRO_LATCHED_PIN:
    	if(autogyro_enable_pin)
    	{
    		break;
    	}

// If we drop through then we want to un-latch.

		mode = operatingMode();

		if(mode == FEATHER)
		{
    		AutoGyro_state = AUTOGYRO_AT_REVERSE_STOP;
        	break;
		}

		// Could make it just take-off or climb....
		if (mode == TAKEOFF || mode == CLIMB || mode == CRUISE)
        {
    		AutoGyro_state = AUTOGYRO_TAKEOFF;
        }
        break;


// In AutoGyro take off state, the only difference with AutoGyro idle state is that we are not sure if we have passed the
// FINE stop. If it is still actively going COARSE under AUTO->TAKEOFF control then no point in checking stop states.
// If it is in drive idle state, we could test to see if fine stop is on. If not, then we are out of reverse zone.
//

	case AUTOGYRO_TAKEOFF:
		AutoGyro_state = AUTOGYRO_WAIT_FINISH;
		Set_fstate(F_COMPLETE);
		break;

	case AUTOGYRO_EXIT:	// Here from Mode change but not latched
		AutoGyro_state = AUTOGYRO_WAIT_FINISH;
//		Set_fstate(F_ACTIVE;
		Set_fstate(F_COMPLETE);		// Acts straight away. F_ACTIVE delays a few seconds.
//    	RC_X_last_action = RC_X_REVERSE_COARSE;		// Try and trigger reverse_zone change
        break;

	case AUTOGYRO_WAIT_FINISH:
		if((fState == F_IDLE) && (RC_X_last_action == 0))
		{
			AutoGyro_state = AUTOGYRO_IDLE;
			break;
		}
    	RC_X_last_action = RC_X_WAIT_FINISH;		// Try and trigger reverse_zone change
		return false;

/*
	case AUTOGYRO_AT_REVERSE_STOP:
		if(AutoGyro_mode_change())
		{
			AutoGyro_state = AUTOGYRO_EXIT;
			break;
		}
    	if(autogyro_enable_pin)
    	{
    		AutoGyro_state = AUTOGYRO_LATCHED;
            Set_fstate(F_COMPLETE;		// Just in case feather logic is looking for it.
    	}
		break;
*/
	}
	if(fState == F_REVERSING)	// MHH:02/04/2018. In case going out of reverse, we want exiting reverse tests
	{
		return false;
	}
	return true;
}
//--------------------------------------------------------------------------
// repeated call to detect state of feathering
//WORD Feather_trace=1;
//static WORD Feather_current=0;		// MHH:20/05/2025
//static WORD Feather_state=0;
extern uint8_t BL_flags;
extern bool BL_feather_when_error;
void featherControl (void)
{
//    WORD trace_val;
	WORD cSpeed;
	WORD current;
	WORD state;
    FeatherState fs = fState;

    if (!feather)
        return;

    if(AutoGyro_reverse)
    {
    	if(AutoGyro_reverse_control())
    	{
    		return;
    	}
    }
//    trace_val = Set_trace_val(Feather_trace);

    Reverse_position = FEATHER;		// default
//    if(isFeatherAndReverseProp()) Reverse_position = REVERSE;

    switch (fState)
    {
    case F_IDLE:
#ifdef MH_XXX
    case F_BETA_IDLE:			// MHH:14/01/2025
    	if(fState == F_BETA_IDLE && Sig100_connected == false)
    	{
    		break;
    	}
    	if(isFeatheringProp() == false)
    	{
//    		Restore_trace_val(trace_val);
    		return;
    	}
#endif
        if (operatingMode() == FEATHER)
        {
            if (manualKeys() & MANUAL_KEY_FEATHER)
            {
            	{
            		Feather_mode = FM_FEATHERING;
            	}
            	// start feathering
//                Feather_mode = FM_IDLE;
                startFeather();
            }
            break;
        }
        if (operatingMode() == REVERSE)	// MHH:16/07/2025
        {
            if (manualKeys() & MANUAL_KEY_FEATHER)
            {
//            	if(Control_type == CT_REVERSE)
            	{
                	if(Reverse_RPM_too_high())
                	{
      //          		Restore_trace_val(trace_val);
                		return;
                	}

            		Feather_mode = FM_REVERSING;
            	}
            	// start feathering
//                Feather_mode = FM_IDLE;
                startFeather();
            }
        }
        break;

    case F_ACTIVE:
        if (operatingMode() != FEATHER)
        {
            // stop immediately, mode change........
            if (operatingMode() == MANUAL)
            {
                stopFeather();
                Set_fstate(F_IDLE);
            }
            else
            {
                // turn off the FET
                stopFeather();
                // and reverse  with check first
                reverseFeather(uCheckState0);
            }
            break;
        }
    	if(Hub_pos_error)		// MHH:20/05/2025. Should only get this if brushless
    	{
#ifdef MH_XXX
    		state = systemState();
    		if(state != Feather_state)
    		{
    			Feather_state = state;
    			DPRINTF("FS=%d\r\n",state);
    		}
#endif
    		state = BL_flags << 4;
    		if (state & S_STOP_COARSE)
            {
            	current = scaledValue (A_MOTOR_CURRENT);
#ifdef MH_XXX
            	if(current != Feather_current)
            	{
            		Feather_current = current;
            		if(current > 0)		// Debug only
            		{
            			DPRINTF("FC=%d\r\n",current);
            		}
            	}
#endif
            	if(current > 2500)	// Same as Hub uses for calibrating
            	{
            		// May need to set feather stop here or special LED flag if we want to get the correct feather LED

            		stopFeather();		// Same as if hit stop feather
            		Set_fstate(F_COMPLETE);
            		BL_feather_when_error = true;
            	}
            }
            break;
    	}
        if (systemState() & S_STOP_FEATHER)
        {
            // feather stop set
            stopFeather();
            Set_fstate(F_COMPLETE);
        }
        else if (systemState() & S_STOP_REVERSE)
        {
            // feather stop set
            stopFeather();
            Set_fstate(F_COMPLETE);
        }
        break;

    case F_COMPLETE:
        if (operatingMode() != FEATHER )
        {
            // coming out of feather;
            reverseFeather(uDrive1);
         }
        break;

    case F_REVERSING:
    	BL_feather_when_error = false;
        reverseFeatherProc();
        break;

//#ifdef MH_XXX		// MHH:14/01/2025. If brushless, may be able to feather
    case F_BETA_IDLE:
        // Do Nothing
        break;
//#endif

    case F_BETA:
        // monitor exit condition
        // we exit when mode changes to != MANUAL && != FEATHER
#if BETA_VERSION
        if(Control_type == CT_BETA)
        {
        	// Monitor speed, if it goes above limit, drop relay out until it goes back
        	// within limits
            if (getParameter(BETA_MAX_RPM_ENGAGED))
            {
                // Get current RPM
            	cSpeed = Get_RPM();
                // If over, drop out relay
                if ((inBetaHold == 0) && (cSpeed > (getParameter(BETA_MAX_RPM_ENGAGED))))	// MHH:20/04/2022
//                if ((inBetaHold == 0) && (cSpeed > (getParameter(BETA_MAX_RPM_ENGAGED)*10)))
                {
                	inBetaHold = 1;
                	setBetaHoldMode();
//                	LED_overlay_flags |= LED_OVERLAY_FLAG_BETA_MAX_RPM;
                }
                // if under, re-engage
                if ((inBetaHold == 1) && (cSpeed < (getParameter(BETA_MAX_RPM_ENGAGED))))	// MHH:20/04/2022
//                if ((inBetaHold == 1) && (cSpeed < (getParameter(BETA_MAX_RPM_ENGAGED)*10)))
                {
                	inBetaHold = 0;
                	clearBetaHoldMode();
 //               	LED_overlay_flags &= ~LED_OVERLAY_FLAG_BETA_MAX_RPM;
                }
            }
        }
        else
        {
            Set_fstate(F_IDLE);
        }
//#else
#endif
        break;

    case F_BETA_EXIT:
        // Call the reversing proc
#if BETA_VERSION
        if(Control_type == CT_BETA)
        {
            reverseBetaProc();
        }
        else
        {
            Set_fstate(F_IDLE);
        }
//#else
#endif
        break;

    }

    if (debug & DEBUG_FEATHER)
    {
        if (fs != fState)
        {
/*
#if REVERSING_VERSION
            sprintf (buf,"Reverse-> %s\r\n",&fStateName[fState][0]);
#elif BETA_VERSION
            sprintf (buf,"Beta   -> %s\r\n",&fStateName[fState][0]);
#else
            sprintf (buf,"Feather-> %s\r\n",&fStateName[fState][0]);
#endif
            txDebug (buf);
*/
        	Debug_feather_state();
        }
    }
//	Restore_trace_val(trace_val);
}

// state accessor
FeatherState featherState (void)
{
    return (fState);
}

// Overcurrent shutdown
void shutdownFeather (void)
{
    BYTE t = Trace("shutdownFeather");
	stopFeather();
#if BETA_VERSION
//	if(Get_param_Feather_mode() == FM_BETA)
	if(fState == F_BETA || fState == F_BETA_EXIT)
//	if(Control_type == CT_BETA)
    {
        clearBetaDriveMode();
        Set_fstate(F_BETA_IDLE);
    }
    else
    {
        Set_fstate(F_IDLE);
    }
//#else
#endif
    TraceReturn(t,"");
}

#if REVERSING_VERSION
// a couple of specials for the remote control




FeatherMode Feather_mode;

void remoteStartReverse (void)
{
    Feather_mode = FM_REVERSING;
	startFeather();
    Set_fstate(F_IDLE);
}

void remoteStopReverse (void)
{
    shutdownFeather();
}

void remoteStartReverseReverse (void)
{
    Feather_mode = FM_UNREVERSING;
	doReverseFeather();
}
void remoteStartFeather (void)
{
    Feather_mode = FM_FEATHERING;
	startFeather();
    Set_fstate(F_IDLE);
}

void remoteStopFeather (void)
{
    shutdownFeather();
}

void remoteStartFeatherReverse (void)
{
    Feather_mode = FM_UNFEATHERING;
	doReverseFeather();
}
#endif

//-----------------------------------------------------------------------------------
void startFeather (void)
{
    if (!feather)
        return;
    // set state
    Set_fstate(F_ACTIVE);
//    Write_param_Feather_mode(Feather_mode);
    setDriveToFeather ();
}

void stopFeather (void)
{
	BYTE t = Trace("stopFeather");
	// turn off quick..
//    Feather_mode = FM_IDLE;
	cancelFeatherDrive ();
    TraceReturn(t,"");
}

#ifdef AC2_TEST		// AC2_TEST
void startTheFeather (void) {

//    Feather_mode = FM_IDLE;


// MHH: Only called from AC2_TEST, so assume FM_FEATHERING
#ifdef MH_XXX
	if(Control_type == CT_REVERSE)
	{

		Feather_mode = FM_REVERSING;
	}
	else
	{
		Feather_mode = FM_FEATHERING;
	}
#endif
	Feather_mode = FM_FEATHERING;
	startFeather();
	enableMotorDrive();
}

void stopTheFeather (void) {
	BYTE t = Trace("stopTheFeather");
    stopFeather();
    disableMotorDrive();
    TraceReturn(t,"");
}
#endif

void reverseFeather (UnfeatherState rState)
{
	BYTE t = Trace("reverseFeather");
    // set up control so we are driving toward fine
    stopFeather();
    if (!feather)
        return;
#if BETA_VERSION

//    if(Control_type == CT_BETA)
    if(Feather_mode == FM_REVERSE_BETA)
    {
        Set_fstate(F_BETA_EXIT);
        reverseState = rState;
        if ((rState != uCheckState0) && (rState != uCheckState1))
        {
            reverseFeatherDrive();
        }
    }
    else
    {
        Set_fstate(F_REVERSING);
        reverseState = rState;
        if ((rState != uCheckState0) && (rState != uCheckState1))
        {
            reverseFeatherTest();
            reverseFeatherDrive();
        }
    }
//#else
#endif

    reverseTimer = ((rState != uCheckState0) ?
                    getParameter(FR_TIME_LIMIT) * 50 :
                    (50 - timerTicks())  + 130);
    reverseTimerSet = reverseTimer;
    TraceReturn(t,"");
}


void doReverseFeather (void)
{
//	WORD trace_val;
	//    char buf[25];

    if (!feather)
        return;

    if(AutoGyro_reverse)
    {
		if((AutoGyro_state == AUTOGYRO_LATCHED)||(AutoGyro_state == AUTOGYRO_LATCHED_PIN))
    	{
    		return;
    	}
    }
//    trace_val = Set_trace_val(Feather_trace);


    reverseFeather(uDrive1);
    if (debug & DEBUG_FEATHER)
    {


/*
#if REVERSING_VERSION
        sprintf (buf,"Reverse-> %s\r\n",&fStateName[fState][0]);
#elif BETA_VERSION
        sprintf (buf,"Beta   -> %s\r\n",&fStateName[fState][0]);
#else
        sprintf (buf,"Feather-> %s\r\n",&fStateName[fState][0]);
#endif
        txDebug (buf);
*/
    	Debug_feather_state();
    }
//	Restore_trace_val(trace_val);

}

// in unfeather cycle in the 1 second pause
BYTE waitingForStateCheck(void)
{   BYTE rval;

#if BETA_VERSION
	if(Control_type == CT_BETA)
	{
	    rval = ((fState == F_BETA_EXIT) &&
	            ((reverseState == uCheckState0) || (reverseState == uCheckState1)));
	}
	else
	{
	    rval = ((fState == F_REVERSING) &&
	            ((reverseState == uCheckState0) || (reverseState == uCheckState1)));
	}

//#else
#endif
    return (rval);
}

extern WORD currentState;

void reverseFeatherProc (void)
{
    WORD cSpeed;
//    BYTE t = Trace("reverseFeatherProc");

    // getting back out of feather is controlled with a system of 2
    // interlocks.
    // 1. A timer that limits drive for x seconds
    //    checks the coarse stop, and retries once
    // 2. Fine stop detection

    // if we are reversing a reversing prop, then this must be
    // 1. A timer that limits drive for x seconds
    //    checks the fine stop, and retries once
    // 2. Coarse stop detection


    // Put the reverse feather stuff here to make it easier to maintain

    bool stop_passed = false;

    if(Sig100_connected)
    {
		if(Feather_mode == FM_UNFEATHERING)
		{
			stop_passed = ((currentState & (S_STOP_COARSE | S_STOP_FEATHER)) == 0);
		}
		if(Feather_mode == FM_UNREVERSING)
		{
        	stop_passed = ((currentState & (S_STOP_FINE | S_STOP_REVERSE)) == 0);
		}
    }
    else
    {
    	stop_passed =  ( ((reverseState == uDrive1) || (reverseState == uDrive2)) &&
    	         (reverseTimerSet - reverseTimer > 25) &&      // allow 0.5 sec min drive time
    	         (isUnfeathered() == STATE_OPEN));
    }

/*
    if ( ((reverseState == uDrive1) || (reverseState == uDrive2)) &&
         (reverseTimerSet - reverseTimer > 25) &&      // allow 0.5 sec min drive time
         (isUnfeathered() == STATE_OPEN))
*/
    if(stop_passed)
    {
        // stop immediately, stop passed
        Set_fstate(F_IDLE);
    	Set_Zone(RC_S_ZONE_NORMAL);
        reverseTimer = 0;
        cancelFeatherDrive();
        clearUnfeatherCheck();

 //       TraceReturn(t,"stop passed");
        return;
    }

    cSpeed = Get_RPM();

    /*
    if (
#if (REVERSING_VERSION == 0)
        ( cSpeed > (getParameter (MIN_ENGINE_SPEED) *10)) ||
#endif
        ( operatingMode() == MANUAL) ||
        ( operatingMode() == FEATHER))
*/

    OpMode op_mode = operatingMode();
    WORD test = 0;
    MotorDriveState ds = driveState();
    if(ds != MD_FEATHER_REVERSE)	// MHH:27/07/2019
    {
        if(op_mode != REVERSE)	// MHH:17/07/2019. This should allow going from FEATHER to REVERSE and exiting FEATHER
        {
//            if((Control_type != CT_REVERSE) && (cSpeed > (getParameter (MIN_ENGINE_SPEED) *RPMFACTOR))) test=1;
//            if((Control_type != CT_REVERSE) && (cSpeed > (getParameter (MIN_ENGINE_SPEED)))) test=1;
//            if((Control_type != CT_REVERSE) && (cSpeed > (getParameter (MIN_ENGINE_SPEED)))) test=1;
            if((Feather_mode != FM_UNREVERSING) && (cSpeed > (getParameter (MIN_ENGINE_SPEED)))) test=1;	// MHH:15/11/2023. Not sure??
// MHH:10/12/2024. Remove maximum speed test at Russell's request.
        }
    }
    if(op_mode == MANUAL) test=2;
    if((op_mode == FEATHER) && (Control_type == CT_FEATHERING)) test=3;

//    if(Control_type == CT_FEATHERING)	// MHH:11/08/2021. When coming out of feather, if RPM > minimum, do ordinary FINE logic.


#ifdef MH_UNFEATHER_RPM_TEST    	// MHH:13/12/2024

    if(Feather_mode == FM_UNFEATHERING)	// MHH:11/08/2021. When coming out of feather, if RPM > minimum, do ordinary FINE logic.
    {
    	if(cSpeed > (getParameter (MIN_ENGINE_SPEED)))
    	{
    		test = 4;
    	}
    }
#endif

    if(test)
/*
    if (((Control_type != CT_REVERSE) && (cSpeed > (getParameter (MIN_ENGINE_SPEED) *10))) ||
        ( operatingMode() == MANUAL) ||
//        ( operatingMode() == FEATHER))
        ( (operatingMode() == FEATHER) && (Control_type == CT_FEATHERING)))	// MHH:13/02/2019
*/
    {
        // stop immediately,
        Set_fstate(F_IDLE);
        reverseTimer = 0;
        cancelFeatherDrive();
        clearUnfeatherCheck();
        return;
    }
    if(Sig100_connected == false)	// MHH:13/12/2024
    {
        switch (reverseState)
        {
        case uCheckState0:
            // For internal switch.. check state first
            if (--reverseTimer == 0)
            {
                // check for coarse/feather stop. If still set, do reverse
/*
#if REVERSING_VERSION
                if ((systemState() & S_STOP_FINE) ||
#else
                if ((systemState() & S_STOP_COARSE) ||
#endif
*/
                if(
                	((Control_type == CT_REVERSE) && (systemState() & S_STOP_FINE)) ||
                    ((Control_type != CT_REVERSE) && (systemState() & S_STOP_COARSE)) ||

                    (systemState() & S_STOP_FEATHER))
                {
                    // reverse required
                    reverseFeather(uDrive1);
                }
                else
                {
                    // past the stop
                    Set_fstate(F_IDLE);
                    cancelFeatherDrive();
                    clearUnfeatherCheck();
                }
            }
            break;

        case uDrive1:

            if (--reverseTimer == 0)
            {
                // increment before we stop drive, otherwise
                // state sense thinks this is a fine stop due
                // to falling current
                reverseState = uCheckState1;
                // stop driving, but don't exit state yet
                cancelFeatherDrive();
                clearUnfeatherCheck();
                // delay is at least 2 samples of the state
                reverseTimer = (50 - timerTicks())  + 65;
            }
            break;

        case uCheckState1:
            if (--reverseTimer == 0)
            {
                // check for coarse/feather stop. If still set, try one more cycle
/*
#if REVERSING_VERSION
                if ((systemState() & S_STOP_FINE) ||
#else
                if ((systemState() & S_STOP_COARSE) ||
#endif
*/
                if(
//                 	((Control_type == CT_REVERSE) && (systemState() & S_STOP_FINE)) ||
//                  ((Control_type != CT_REVERSE) && (systemState() & S_STOP_COARSE)) ||
                 	((Feather_mode == FM_UNREVERSING) && (systemState() & S_STOP_FINE)) ||
                     ((Feather_mode == FM_UNFEATHERING) && (systemState() & S_STOP_COARSE)) ||
                    (systemState() & S_STOP_FEATHER))
                {
                    // still more required
                    reverseFeather(uDrive2);
                }
                else
                {
                    // past the stop
                    Set_fstate(F_IDLE);
                    cancelFeatherDrive();
                    clearUnfeatherCheck();
                }
            }
            break;

        case uDrive2:

            if (--reverseTimer == 0)
            {
                // stop driving, had 2 goes now
                cancelFeatherDrive();
                clearUnfeatherCheck();
                Set_fstate(F_IDLE);
                reverseState = uCheckState0;
            }
            break;

        }
    }
//    TraceReturn(t,"");
}

#if BETA_VERSION

void reverseBetaProc (void)
{
    // getting back out of Beta
    // Drive until fine stop passed
    // drive for x seconds past the stop
    // Coarse stop detection as a backup

    // Put the reverse Beta stuff here to make it easier to maintain

    if(Sig100_connected)
    {
    	if((currentState & (S_STOP_FINE | S_STOP_REVERSE)) == 0)
    	{
            Set_fstate(F_BETA_IDLE);
            reverseTimer = 0;
            cancelFeatherDrive();
            clearBetaDriveMode();
    	}
    	else
    	{
    	   	Feather_mode = FM_REVERSE_BETA;
    		dState = MD_BETA_EXIT;
    	}
    	return;
    }
   	if( ((reverseState == uDrive1) || (reverseState == uDrive2)) &&
    	         (reverseTimerSet - reverseTimer > 25) &&   // allow 0.5 sec min drive time
    	         (systemState() & S_STOP_COARSE))
    {
        // stop immediately, stop passed
        //txDebug ("STOP IN DRIVE, state check\r\n");
        Set_fstate(F_BETA_IDLE);
        reverseTimer = 0;
        cancelFeatherDrive();
        clearBetaDriveMode();
        return;
    }
   	Feather_mode = FM_REVERSE_BETA;
    switch (reverseState)
    {
    case uCheckState0:
        // For internal switch.. check state first
        if (--reverseTimer == 0)
        {
            // check for coarse/feather stop. If still set, do reverse
            if ((systemState() & S_STOP_FINE) ||
                (systemState() & S_STOP_FEATHER))
            {
                // reverse required
                reverseFeather(uDrive1);
            }
            else
            {
                // past the stop
                //txDebug ("STOP IN CHECK 0\r\n");
                Set_fstate(F_BETA_IDLE);
                cancelFeatherDrive();
                clearBetaDriveMode();
            }
        }
        break;

    case uDrive1:

        if (--reverseTimer == 0)
        {
            // increment before we stop drive, otherwise
            // state sense thinks this is a fine stop due
            // to falling current
            reverseState = uCheckState1;
            // stop driving, but don't exit state yet
            cancelFeatherDrive();
            // delay is at least 2 samples of the state
            reverseTimer = (50 - timerTicks())  + 65;
        }
        break;

    case uCheckState1:
        if (--reverseTimer == 0)
        {
            // check for coarse/feather stop. If still set, try one more cycle
            if ((systemState() & S_STOP_FINE) ||
                (systemState() & S_STOP_FEATHER))
            {
                // still more required
                reverseFeather(uDrive2);
            }
            else
            {
                // past the stop
                //txDebug ("STOP IN CHECK 1\r\n");
                Set_fstate(F_BETA_IDLE);
                cancelFeatherDrive();
                clearBetaDriveMode();
            }
        }
        break;

    case uDrive2:

        if (--reverseTimer == 0)
        {
            // stop driving, had 2 goes now
            //txDebug ("STOP IN DRIVE 2\r\n");
            cancelFeatherDrive();
            clearBetaDriveMode();
            Set_fstate(F_BETA_IDLE);
            reverseState = uCheckState0;
        }
        break;
    }
}

void enterBetaMode (void)
{
    char buf[30];

    Set_fstate(F_BETA);
    inBetaHold = 0;
    if (debug & DEBUG_FEATHER)
    {
        sprintf (buf,"Beta   -> %s\r\n",&fStateName[fState][0]);
        txDebug (buf);
    }
    setBetaDriveMode();
}

void exitBetaMode (void)
{
	// We want to release the relay here, as it  interferes with state sampling
	clearBetaDriveModeForReverse();
	Feather_mode = FM_REVERSE_BETA;
    reverseFeather (uCheckState0); // used to check if past stops
}

#endif
