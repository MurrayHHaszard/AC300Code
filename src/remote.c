/* ------------------------------------------------------------
Title:          remote.c

Copyright:      Airmaster Ltd, December 2000

Date created:   01/07/2002
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Remote control code

Changes:
//NO_FEATHER_BIT
------------------------------------------------------------ */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "global.h"
#include "remote.h"

#include "ac210_sig100.h"
#include "param.h"
#include "rpm.h"
#include "digital.h"
#include "comms.h"
#include "control.h"
#include "sstate.h"
#include "feather.h"
#include "drive.h"
#include "analog.h"
#include "leds.h"
#include "xoar.h"

void RemotePos_ControlCycle(void);
int AC200Enc_version;

#if REMOTE_VERSION

// state letters, Idle,Ground,Flight,Lostcoms
const char rsChar[7]="LIGFRC";


typedef enum
{
    rcOK = 0,
    rcLockout,
    rcBadCommand,
    rcBadSetpoint,
    rcBadTime,
    rcBadRPM,
    rcBadStop
}RCVal;

void RC_S_ControlCycle(void);		// maybe put in remote.h



#ifdef MH_REMOTE_BRAKE		// Think this was just when testing braking.
WORD AT_BRAKE_remote_cnt;	// Just used in remote.c to allow time to get msg to control.c
#endif

// externs.these are declared in control.c
extern long setSpeed;

#if REVERSING_VERSION
// these are in feather.c
extern void remoteStartReverse (void);
extern void remoteStopReverse (void);
extern void remoteStartReverseReverse (void);
extern void remoteStartFeather (void);
extern void remoteStopFeather (void);
extern void remoteStartFeatherReverse (void);


void startFeather (void);
#endif

// local variables
RemoteState remoteState;
int remoteComCounter;

// flight mode
WORD remoteSetSpeed;
WORD remoteSetPoint[10];

// ground mode
BYTE            controlReversing;
BYTE			controlFeathering;
ControlState    controlDirection;
BYTE            controlStop;

BYTE Xoar_feather;
BYTE Xoar_reverse;
BYTE Xoar_instruction;
//BYTE Xoar_next_instruction;
//WORD Xoar_next_parameter;

// change mode in remote, single point of change
void changeRemoteMode (RemoteState newMode)
{
	remoteState = newMode;
}

//initialise remote operation
void initRemoteMode (void)
{
    int i;

//    remoteState = REMOTE_IDLE;
    changeRemoteMode(REMOTE_IDLE);
    for (i=0; i< 10; i++)
    {
        remoteSetPoint[i] = 0;
    }
    remoteSetSpeed = 0;
    controlReversing = 0;
    controlDirection = C_IDLE;
    controlStop = ' ';
    remoteComCounter = 0;
}

int iremoteState (void)	// So we can access value externally
{
    return (int)(remoteState);
}
// are we in remote mode
RemoteState remoteMode (void)
{
    return (remoteState);
}

// lockout remote mode
void setRemoteLockout (void)
{
    if (remoteState == REMOTE_GROUND_MODE)
    {
        // just in case
#if REVERSING_VERSION
        if(Control_type == CT_REVERSE)
        {
        	if (controlReversing)
            {
                remoteStopReverse();
                controlReversing = 0;
            }
        }
#endif
    }
    changeRemoteMode (REMOTE_LOCKOUT);
    remoteComCounter = 0;
}

// clear remote lockout
void clearRemoteLockout (void)
{
    changeRemoteMode (REMOTE_IDLE);
}

// get control direction for ground mode
WORD remoteControlDirection(void)
{
    if (remoteState == REMOTE_GROUND_MODE)
    {
        return ((WORD)controlDirection);
    }
    else
    {
        return ((WORD)C_IDLE);
    }
}

BYTE remoteReverseActive(void)
{
    return (controlReversing);
}
//-----------------------------------------------------------------------
// the control cycle call
WORD remoteControlCycle_count;
bool remoteXoar2SetAngle(void);
void remoteControlCycle (void)
{
    WORD sysState;
    RemoteState ors = remoteState;


    RemoteCommsType = getParameter(REMOTE_COMMS_TYPE);
	if(RemoteCommsType == REMOTE_COMMS_STANDARD)
	{
		RC_S_ControlCycle();
		return;
	}

	remoteControlCycle_count++;

    if (remoteState == REMOTE_LOCKOUT)
    {
        return;
    }

   if(remoteXoar2SetAngle())
   {
	   return;
   }

    if (remoteComCounter && AT_Remote==0)
    {
        // decrement by 1 only, as this gets called 50 times per second
//        if (--remoteComCounter == 0)
        if (--remoteComCounter <= 0)	// MHH:09/12/2022
        {
        	if(Xoar_instruction)
        	{
        		controlDirection = C_IDLE;
        		return;
        	}

        	// lost comms.. change state
        	if (remoteState > REMOTE_IDLE)
            {
                // not if we are in a reverse_reverse cycle
                if (remoteState != REMOTE_REVERSE_REVERSE && remoteState != REMOTE_FEATHER_REVERSE)
                {
                    changeRemoteMode (REMOTE_IDLE);
                }
            }
        }
    }

    if (remoteState == REMOTE_IDLE)
    {
        return;
    }

    // need to update control variables
    switch (remoteState)
    {
    default:		// MHH: 16/11/16. Xpresso compiler error, remote lockout not handled
    	break;

    case REMOTE_GROUND_MODE:
        {
            if (controlStop != ' ')
            {
                sysState = systemState();
                if ( ((controlStop == 'C') && (sysState & S_STOP_COARSE)) ||
                     ((controlStop == 'F') && (sysState & S_STOP_FINE))
#if REVERSING_VERSION
                     ||
					 ((Control_type == CT_REVERSE) &&
                     ((controlStop == 'R') && (sysState & S_STOP_REVERSE)))
#endif
                     ||
					 ((Control_type == CT_FEATHERING) &&
                     ((controlStop == 'f') && (sysState & S_STOP_FEATHER)))

                )
                {
                    controlDirection = C_IDLE;
                    controlStop = ' ';
#if REVERSING_VERSION
                    if(Control_type == CT_REVERSE)
                    {
                        if (controlReversing)
                        {
                            remoteStopReverse();
                            controlReversing = 0;
                        }
                    }
                    if(Control_type == CT_FEATHERING)
                    {
                        if (controlFeathering)
                        {
                            remoteStopFeather();
                            controlFeathering = 0;
                        }
                    }
#endif
                }
                else	// MHH:21/04/2022. If reversing and RPM >= Max_run_rpm then send error and wait for RPM to come down or a new command
                {
                    if(Control_type == CT_REVERSE)
                    {
                        if (controlReversing)
                        {
                    		if (getParameter(BETA_CHK_RPM))	// MHH:22/04/2022 Don't allow reverse if RPM too high
                    		{
                    			WORD rpm = Get_RPM();
                                if (rpm >= getParameter(BETA_MAX_RPM_ENGAGED))
                        		{
                                	remoteStopReverse();
                                    controlReversing = 0;
                                    Xoar_reverse = false;	// ??
                        			Xoar2_set_error(XOAR_ERROR_RPM_GT_BETA_MAX_RUN);
                        			return;
                        		}
                    		}
                        }
                    }
                }
            }
            break;
        }

    case REMOTE_FLIGHT_MODE:
        {
//            setSpeed = (long)(remoteSetSpeed*10);
//            setSetSpeed ((long)(remoteSetSpeed*RPMFACTOR));	// MHH:20/03/2018
            setSetSpeed (remoteSetSpeed);	// MHH:20/03/2018
            break;
        }

#if REVERSING_VERSION
    case REMOTE_REVERSE_REVERSE:
        {
            // wait for the reverse_reverse cycle to complete
            sysState = systemState();
            if (!(sysState & S_STOP_REVERSE) && !(sysState & S_STOP_FINE))
            {
                if ((featherState() == F_IDLE) && (driveState() == MD_IDLE))
                {
                    // this will then be updated via the comms
                    changeRemoteMode (REMOTE_IDLE);
                    Xoar_reverse = false;
                }
            }
            break;
        }
#endif
    case REMOTE_FEATHER_REVERSE:
        {
            // wait for the reverse_reverse cycle to complete
            sysState = systemState();
            if (!(sysState & S_STOP_FEATHER) && !(sysState & S_STOP_COARSE))
            {
                if ((featherState() == F_IDLE) && (driveState() == MD_IDLE))
                {
                    // this will then be updated via the comms
                    changeRemoteMode (REMOTE_IDLE);
                    Xoar_feather = false;
/*
                    if(Xoar_next_instruction)
                    {
                    	XoarProcessCommand3(Xoar_next_instruction,Xoar_next_parameter);
                    	Xoar_next_instruction = 0;
                    }
*/
                }
            }
            break;
        }

    case REMOTE_LOST_COMMS:
        {
            // What we do here is stop controlling in ground mode
            if (ors == REMOTE_GROUND_MODE)
            {
                controlDirection = C_IDLE;
                controlStop = ' ';
#if REVERSING_VERSION
                if(Control_type == CT_REVERSE)
                {
                    if (controlReversing)
                    {
                        remoteStopReverse();
                        controlReversing = 0;
                    }
                }
#endif
            }
            else if (ors == REMOTE_FLIGHT_MODE)
            {
                // and What here.set to 0 to stop controlling...........
                remoteSetSpeed = 0;
            }
            break;
        }
    }
}

// remote command processing routines

void updateRemoteCounter (void)
{
    // increment a counter, the remoteControlCycle call decrements this
    // and if it ever gets to 0 of below, then we are deemed to have lost comms
    // because the call above happens 50 times per second, but we only expect
    // 10 polls per second, we add 5 here
    if (remoteState != REMOTE_LOCKOUT)
    {
        if (remoteComCounter <= 100)
        {
            remoteComCounter += 10;
        }
    }
}

WORD convertRPM (char *num, BYTE *res)
{
    BYTE cnt, tmp, setZero;
    WORD rpm = 0;

    setZero = 0;
    for (cnt = 0; cnt < 4; cnt++)
    {
        tmp = (*num++) - '0';
        if (tmp <= 9)
        {
            rpm *= 10;
            rpm += tmp;
        }
        else
        {
            rpm = 0;
            break;
        }
        if ((cnt == 3) && (rpm == 0))
        {
            setZero = 1;
        }
    }
    if (rpm)
    {
        // check against min and max
        if ( (rpm > getParameter(MAX_ENGINE_SPEED)) ||
             (rpm < getParameter(MIN_ENGINE_SPEED)) )
        {
            rpm = 0;
        }
    }
    *res = setZero;

    return (rpm);
}

RCVal programSetpoint (char *cmd)
{
    // Psnnnn    P=program, s=setpointnumber,nnnn=rpm
    BYTE sp, validZero;
    WORD rpm;
    RCVal rval = rcBadSetpoint;

    cmd++;
    sp = *cmd++ - '0';
    if (sp < 10)
    {
        rpm = convertRPM(cmd, &validZero);
        if (rpm || validZero)
        {
            remoteSetPoint[sp] = rpm;
            rval = rcOK;
        }
        else
        {
            rval = rcBadRPM;
        }
    }
    return (rval);
}

RCVal groundMode (char *cmd)
{
    // GDd       G=GroundMode, D=Drive, d=(C)oarse, (F)ine, (R)everse, or (I)dle,
    // GSs       G=GroundMode, s=MoveToStop, s=stop (CFRI)
    BYTE Reverse, stop;
    ControlState cs;
    RCVal rval= rcOK;

    Reverse = 0;
    stop = ' ';
    cs = C_IDLE;
    cmd++;
    switch (*cmd)
    {
    case 'D':
        {
            // drive control
            cmd++;
            switch (*cmd)
            {
            case 'F':
                {
                    cs = C_FINER;
                    Reverse =0;
                    break;
                }
            case 'C':
                {
                    cs = C_COARSER;
                    Reverse = 0;
                    break;
                }
            case 'I':
                {
                    cs = C_IDLE;
                    Reverse = 0;
                    break;
                }

#if REVERSING_VERSION
            case 'R':
                if(Control_type == CT_REVERSE)
                {
                    cs = C_FINER;
                    Reverse = 1;
                    break;
                }
                else
                {
                    rval = rcBadCommand;
                    break;
                }
#endif
            default:
                {
                    rval = rcBadCommand;
                    break;
                }
            }
            // update control
            if (rval == rcOK)
            {
                controlStop = ' ';
                controlDirection = C_IDLE;
#if REVERSING_VERSION
                if(Control_type == CT_REVERSE)
                {
                    if (controlReversing && !Reverse)
                    {
                        remoteStopReverse();
                    }
                    else if (Reverse && !controlReversing)
                    {
                        remoteStartReverse();
                    }
                }
#endif
                controlDirection = cs;
                controlReversing = Reverse;
            }
            break;
        }

    case 'S':
        {
            // drive to stop
            cmd++;
            switch (*cmd)
            {
            case 'C':
                {
                    cs = C_COARSER;
                    stop = 'C';
                    Reverse = 0;
                    break;
                }
            case 'F':
                {
                    cs = C_FINER;
                    stop = 'F';
                    Reverse = 0;
                    break;
                }
#if REVERSING_VERSION
            case 'R':
                if(Control_type == CT_REVERSE)
                {
                    cs = C_FINER;
                    stop = 'R';
                    Reverse = 1;
                    break;
                }
                else
                {
                    rval = rcBadStop;
                    break;
                }
#endif
            case 'I':
                {
                    // just wait for stop to be hit
                    break;
                }
            default:
                {
                    rval = rcBadStop;
                    break;
                }
            }
            // update control
            if ((rval == rcOK) && (*cmd != 'I') && (remoteState != REMOTE_REVERSE_REVERSE))
            {
                controlStop = stop;
                controlDirection = C_IDLE;
#if REVERSING_VERSION
                if(Control_type == CT_REVERSE)
                {
                    if (controlReversing && !Reverse)
                    {
                        remoteStopReverse();
                    }
                    else if (Reverse && !controlReversing)
                    {
                        remoteStartReverse();
                    }
                }
#endif
                controlDirection = cs;
                controlReversing = Reverse;
            }
            break;
        }

    default:
        {
            rval = rcBadCommand;
            break;
        }
    }

    if (rval == rcOK)
    {
        // check mode if successful, if we have triggered a reverse_reverse cycle, let it finish
        if ((remoteState != REMOTE_GROUND_MODE)
#if REVERSING_VERSION
        		&& ((Control_type == CT_REVERSE) && (remoteState != REMOTE_REVERSE_REVERSE))
#endif
        		)
        {
            changeRemoteMode (REMOTE_GROUND_MODE);
        }
    }
    return (rval);
}

RCVal flightMode (char *cmd)
{
    // FI        F=Flightmode, I=Idle
    // FPn       F=FlightMode, P=setpoint, n=setpoint number
    // FSnnnn    F=Flightmode, S=speed, nnnn=rpm

    WORD speed;
    BYTE sp, validZero;
    WORD state;
    RCVal rval= rcOK;

    cmd++;
    speed = 0;
    switch (*cmd)
    {
    case 'I':
        {
            //stop controlling
            speed = 0;
            break;
        }
    case 'P':
        {
            // control to set point
            cmd++;
            sp =  *cmd - '0';
            if (sp < 10)
            {
                speed = remoteSetPoint[sp];
            }
            else
            {
                rval = rcBadSetpoint;
            }
            break;
        }
    case 'S':
        {
            // control to given rpm
            cmd++;
            speed = convertRPM(cmd, &validZero);
            if (!speed && !validZero)
            {
                rval = rcBadRPM;
            }
            break;
        }
    default:
        {
            rval = rcBadCommand;
            break;
        }
    }

    if (rval == rcOK)
    {
        // set speed
        remoteSetSpeed = speed;
        // check mode if successful. if we have triggered a reverse_reverse then wait till
        // it finishes
//        if ((remoteState != REMOTE_FLIGHT_MODE) && (remoteState != REMOTE_REVERSE_REVERSE))	// This line looks redundant.
//        {
            if (remoteState == REMOTE_GROUND_MODE)
            {
                // just in case
#if REVERSING_VERSION
                if(Control_type == CT_REVERSE)
                {
                    if (controlReversing)
                    {
                        remoteStopReverse();
                    }
                    state = systemState();
                    if ((state & S_STOP_REVERSE)|| (state & S_STOP_FINE))
                    {
                        remoteStartReverseReverse();
                        changeRemoteMode (REMOTE_REVERSE_REVERSE);
                        // need to exit here
                        return (rval);
                    }
                }
#endif
                // if we are in the reversed range, we need to back out
#if REVERSING_VERSION
#endif
            }
            changeRemoteMode (REMOTE_FLIGHT_MODE);
//        }
    }

    return (rval);
}

void returnStatus (void)
{
    // reurn to a valid command is RCmsdnnnnn<LF>
    // m is current remote state(LIGFC),s= stop status, d = drive status, nnnnn = rpm to 1dp
    char buf[30];
    ULONG sp;
    WORD wsp;
    BYTE  ss;
    BYTE  ds;
    WORD st = systemState();

//    magSpeed ((WORD *)&wsp);
    wsp = Get_RPM();
//    sp = (sp * 10) /RPMFACTOR;	// MHH:20/11/2020
    sp = (wsp * 10);	// MHH:20/11/2020
    ss = ((st >> 4) & 0x07) | 0x30;
    if (st & S_ERROR_OPEN)
    {
        ss = 0x37;
    }
    if (st & S_RUN_FINE)
    {
        ds = 'F';
    }
    else if (st & S_RUN_COARSE)
    {
        ds = 'C';
    }
#if REVERSING_VERSION
    else if ((Control_type == CT_REVERSE) && (st & S_RUN_REVERSE))
    {
        ds = 'R';
    }
#endif
    else
    {
        ds = 'I';
    }
    sprintf (buf, "RC%c%c%c%05lu\n", rsChar[remoteState],ss,ds,sp );
    txDebug (buf);
}

void errorReturn (RCVal err)
{
    // return is RCEn, E=Error, n = error number
    char buf[10];
    sprintf (buf, "RCE%c\n", (char)(err+'0'));
    txDebug (buf);
}

// a remote command has arrived
/*
    Commands are
    RCPsnnnn    P=program, s=setpointnumber,nnnn=rpm
    RCGDd       G=GroundMode, D=Drive, d=(C)oarse, (F)ine, or (I)dle
    RCGSs       G=GroundMode, s=MoveToStop, s=stop (CFRI)
    RCFI        FlightMode, Idle
    RCFPn       F=FlightMode, P=setpoint, n=setpoint number
    RCFSnnnn    F=Flightmode, S=speed, nnnn=rpm
    RCE         E= exit remote mode
    If no commands arrive for 1 second, then system will revert to what......
*/

void processRemoteCommand (char *cmd)
{
    // at this stage we already know the first 2 letters were RC
    // so they dont get here
    RCVal rv;

    if (remoteState == REMOTE_LOCKOUT)
    {
        returnStatus();
        return;
    }
    switch (*cmd)
    {
    case 'P':
        {
            // program a setpoint
            rv = programSetpoint (cmd);
            if (rv == rcOK)
            {
                updateRemoteCounter();
                returnStatus ();
            }
            else
            {
                errorReturn(rv);
            }
            break;
        }
    case 'G':
        {
            rv = groundMode (cmd);
            if (rv == rcOK)
            {
                updateRemoteCounter();
                returnStatus ();
            }
            else
            {
                errorReturn(rv);
            }
            break;
        }
    case 'F':
        {
            rv = flightMode (cmd);
            if (rv == rcOK)
            {
                updateRemoteCounter();
                returnStatus ();
            }
            else
            {
                errorReturn(rv);
            }
            break;
        }
    case 'E':
        {
            if (remoteState == REMOTE_GROUND_MODE)
            {
                // just in case
#if REVERSING_VERSION
                if(Control_type == CT_REVERSE)
                {
                    if (controlReversing)
                    {
                        remoteStopReverse();
                    }
                }
#endif
            }
            changeRemoteMode (REMOTE_IDLE);
            remoteComCounter = 0;
            break;
        }
    default :
        {
            // error
            errorReturn(rcBadCommand);
            break;
        }
    }
}
//--------------------------------------------------------
//#if XOAR_VERSION > 0
void XoarSetRemoteMode(void)
// called from main.c
{
    OpMode          mode;
    mode = operatingMode ();
	if(S_Pkt.Mode == SERIAL_MODE_XOAR && mode == HOLD) {
// Don't really want REMOTE_FLIGHT_MODE unless we have a command packet.
//        changeRemoteMode (REMOTE_FLIGHT_MODE);
	} else {
        changeRemoteMode (REMOTE_IDLE);
	}
}
//--------------------------------------------------------
void XoarReverseCode(WORD instruction_parameter)
{
/*
 * Note: To make Xoar FEATHER + REVERSE work will need to have similar logic to AC210_check_start_reverse() in feather.c except
 *       no need to check reverse_enable_pin. Something like:
 *
 *       AC210_RELAY_Set(true);
 *	 	 Reverse_state = RS_RELAY_ENABLED;
 *		 Control_type = CT_REVERSE;	// Hopefully will allow normal reverse logic to work
 *
 *		 and then, after out of reverse command restore to original status. Something like:
 *
 * 	 	 AC210_reverse_relay_off();
 * 	 	 Reverse_state = RS_DISABLED;
 *
 */

	if(Control_type != CT_REVERSE)
	{
		Xoar2_set_error(XOAR_ERROR_NOT_REVERSING_TYPE);
		return;
	}

	switch(instruction_parameter)
	{
	default:
		Xoar2_set_error(XOAR_ERROR_INVALID_PARAMETER);
		return;

	case 0:
		if(Xoar_reverse && controlReversing == false)	// Already in feather state?
		{
			Xoar2_set_error(XOAR_ERROR_ALREADY_REVERSING);
			return;
		}
		if(controlReversing)
		{
			Xoar2_set_error(XOAR_ERROR_ALREADY_FEATHERING);
			return;
		}

		if (getParameter(BETA_CHK_RPM))	// MHH:22/04/2022 Don't allow reverse if RPM too high
		{
			WORD rpm = Get_RPM();
            if (rpm >= getParameter(BETA_MAX_RPM))
    		{
    			Xoar2_set_error(XOAR_ERROR_RPM_GT_BETA_MAX_ENGAGE);
    			return;
    		}
		}

        remoteStartReverse();
	    changeRemoteMode (REMOTE_GROUND_MODE);
        controlStop = 'R';
        controlDirection = C_IDLE;
        controlReversing = true;
        Xoar_reverse = true;
        return;

	case 1:
        if (controlReversing)
        {
            remoteStopReverse();
        }
        if(remoteState != REMOTE_REVERSE_REVERSE)	// Already getting out of reverse?
        {
            WORD state = systemState();
            if ((state & S_STOP_REVERSE)|| (state & S_STOP_FINE))
            {
                remoteStartReverseReverse();
                changeRemoteMode (REMOTE_REVERSE_REVERSE);
            }
            else
            {
    			Xoar2_set_error(XOAR_ERROR_NOT_REVERSE_STATE);
            }
        }
        return;
	}
}
//--------------------------------------------------------
void XoarFeatherCode(WORD instruction_parameter)
{
	if(Control_type != CT_FEATHERING)
	{
		Xoar2_set_error(XOAR_ERROR_NOT_FEATHERING_TYPE);
		return;
	}

	switch(instruction_parameter)
	{
	default:
		Xoar2_set_error(XOAR_ERROR_INVALID_PARAMETER);
		return;

	case 0:
		if(Xoar_feather)	// Already in feather state?
		{
			Xoar2_set_error(XOAR_ERROR_ALREADY_FEATHERING);
			return;
		}
		if(controlFeathering)
		{
			Xoar2_set_error(XOAR_ERROR_ALREADY_FEATHERING);
			return;
		}
        remoteStartFeather();
	    changeRemoteMode (REMOTE_GROUND_MODE);
        controlStop = 'f';
        controlDirection = C_IDLE;
        controlFeathering = true;
        Xoar_feather = true;
        return;

	case 1:
        if (controlFeathering)
        {
            remoteStopFeather();
        }
        if(remoteState != REMOTE_FEATHER_REVERSE)	// Already getting out of feather?
        {
            WORD state = systemState();
            if ((state & S_STOP_FEATHER)|| (state & S_STOP_COARSE))
            {
                remoteStartFeatherReverse();
                changeRemoteMode (REMOTE_FEATHER_REVERSE);
            }
            else
            {
    			Xoar2_set_error(XOAR_ERROR_NOT_FEATHER_STATE);
            }
        }
        return;
	}
}
//--------------------------------------------------------
void XoarSetDe_ice(WORD instruction_parameter)
{
	WORD deicer_enable = getParameter(DEICER_ENABLE);
	switch(deicer_enable)
	{
	default:
		Xoar2_set_error(XOAR_ERROR_DEICER_NOT_ENABLED);
		return;

	case 1:
	case 2:
		break;
	}

	switch(instruction_parameter)
	{
	default:
		Xoar2_set_error(XOAR_ERROR_INVALID_PARAMETER);
		return;

	case 0:		// Switch Off
		if(deicer_enable == 2)
		{
			writeParameter ( DEICER_ENABLE, 1);
		}
		break;

	case 1:		// Switch ON
		if(deicer_enable == 1)
		{
			writeParameter ( DEICER_ENABLE, 2);
		}
		break;
	}
	AC210_DeIcer_check();
}
//--------------------------------------------------------
void State_reset_overcurrent_trip(void);
WORD Xoar2_set_angle;
void XoarSetAngle(WORD instruction_parameter)
{
	if(instruction_parameter > 900)
	{
		Xoar2_set_error(XOAR_ERROR_INVALID_PARAMETER);

	}

	Xoar2_set_angle = instruction_parameter;
}
void XoarProcessCommand2(BYTE instruction_code,WORD instruction_parameter)
{
    OpMode          mode;
    mode = operatingMode ();
    if(mode != HOLD)  {	// To be sure
        changeRemoteMode (REMOTE_IDLE);
        remoteSetSpeed = 0;
        Xoar2_set_error(XOAR_ERROR_NOT_HOLD_MODE);
        return;
    }
    switch(instruction_code)
    {
    case X2_INSTRUCTION_CODE_RESET_OVERCURRENT:
    	if(instruction_parameter == 1)
    	{
    		if(overCurrentTrip == 0)
    		{
    	        Xoar2_set_error(XOAR_ERROR_NO_CURRENT_OVERLOAD);
    		}
    		else
    		{
        		State_reset_overcurrent_trip();
    		}
    	}
    	else
    	{
    		Xoar2_set_error(XOAR_ERROR_INVALID_PARAMETER);
    	}
    	return;

    case X2_INSTRUCTION_CODE_FEATHER:
    	XoarFeatherCode(instruction_parameter);
    	return;

    case X2_INSTRUCTION_CODE_REVERSE:
    	XoarReverseCode(instruction_parameter);
    	return;

    case X2_INSTRUCTION_CODE_SET_DEICE:	// MHH:15/03/2021
    	XoarSetDe_ice(instruction_parameter);
    	return;
    }

#ifdef MH_OLD_INSTRUCTION_CODE
    if(instruction_code == X2_INSTRUCTION_CODE_RESET_OVERCURRENT)	// MHH:05/03/2020
    {
    	if(instruction_parameter == 1)
    	{
    		if(overCurrentTrip == 0)
    		{
    	        Xoar2_set_error(XOAR_ERROR_NO_CURRENT_OVERLOAD);
    		}
    		else
    		{
        		State_reset_overcurrent_trip();
    		}
    	}
    	else
    	{
    		Xoar2_set_error(XOAR_ERROR_INVALID_PARAMETER);
    	}
    	return;
    }
    if(instruction_code == X2_INSTRUCTION_CODE_FEATHER)
    {
    	XoarFeatherCode(instruction_parameter);
    	return;
    }
    if(instruction_code == X2_INSTRUCTION_CODE_REVERSE)
    {
    	XoarReverseCode(instruction_parameter);
    	return;
    }
#endif
    if(Xoar_feather)
    {
        Xoar2_set_error(XOAR_ERROR_IN_FEATHER_STATE);
        return;
    }
    if(Xoar_reverse)
    {
        Xoar2_set_error(XOAR_ERROR_IN_REVERSE_STATE);
        return;
    }
    Xoar_instruction = instruction_code;

	switch(instruction_code)
	{
	default:
		Xoar2_set_error(XOAR_ERROR_INVALID_CODE);
		return;

	case X2_INSTRUCTION_CODE_AUTO_SPEED:
	    changeRemoteMode (REMOTE_FLIGHT_MODE);
	    remoteSetSpeed = instruction_parameter;
	    // MHH:09/03/2022. Xoar were setting setspeed to zero, which was causing problems for control logic.
	    if(remoteSetSpeed > getParameter(MAX_HOLD_SPEED))remoteSetSpeed = getParameter(MAX_HOLD_SPEED);
	    if(remoteSetSpeed < getParameter(MIN_HOLD_SPEED))remoteSetSpeed = getParameter(MIN_HOLD_SPEED);
		break;
	case X2_INSTRUCTION_CODE_MAN_COARSE:
	    changeRemoteMode (REMOTE_GROUND_MODE);
		controlDirection = C_COARSER;
		remoteComCounter = instruction_parameter/20;	// Convert ms to cycles
	    remoteSetSpeed = 0;
		break;
	case X2_INSTRUCTION_CODE_MAN_FINE:
	    changeRemoteMode (REMOTE_GROUND_MODE);
		controlDirection = C_FINER;
		remoteComCounter = instruction_parameter/20;	// Convert ms to cycles
	    remoteSetSpeed = 0;
		break;

	case X2_INSTRUCTION_CODE_SET_ANGLE:
		if(Sig100_connected == false)
		{
			Xoar2_set_error(XOAR_ERROR_CODE_NOT_IMPLEMENTED);
			break;
		}
		if(instruction_parameter > 900)
		{
			Xoar2_set_error(XOAR_ERROR_INVALID_PARAMETER);
			break;
		}
		Xoar2_set_angle = instruction_parameter;
		break;
	}
//	setSetSpeed(remoteSetSpeed);
//	setSetSpeed(remoteSetSpeed*RPMFACTOR);	// MHH:10/07/2019
	setSetSpeed(remoteSetSpeed);	// MHH:10/07/2019
}
//#endif
//--------------------------------------------------------
static WORD save_remote_type;
static WORD save_AT_Remote;
void Remote_init_tune(void)
{
	save_remote_type = getParameter(REMOTE_COMMS_TYPE);
	save_AT_Remote = AT_Remote;

	setTempParameter(REMOTE_COMMS_TYPE,REMOTE_COMMS_LEGACY);
	AT_Remote = 1;		// So no timeout if no new commands
	remoteSetSpeed = getParameter (SP_TAKEOFF);		// This will be target
	changeRemoteMode(REMOTE_FLIGHT_MODE);
//	remoteState = REMOTE_FLIGHT_MODE;
}
//--------------------------------------------------------
void Remote_end_tune(void)
{
// Restore old values

	setTempParameter(REMOTE_COMMS_TYPE,save_remote_type);
	AT_Remote = save_AT_Remote;

	remoteSetSpeed = 0;
	changeRemoteMode(REMOTE_IDLE);
//	remoteState = REMOTE_IDLE;
}
#endif
//int RemotePos_active=FALSE;
int RemotePos_fastbrake;
//==============================================================================================
#define RC_S_ERR_OK				0
#define RC_S_ERR_BAD_COMMAND	1
#define RC_S_ERR_BAD_VALUE		2
#define RC_S_ERR_NOT_HOLD_MODE	3
#define RC_S_ERR_NO_REVERSE		4
#define RC_S_ERR_NOT_IN_REVERSE	5
#define RC_S_ERR_BAD_CHECK_VALUE		6
#define RC_S_ERR_RPM_TOO_HIGH			7
#define RC_S_ERR_SIG100_NOT_CONNECTED	8
#define RC_S_ERR_NO_FEATHER				9
#define RC_S_ERR_ALREADY_FEATHERING		10
#define RC_S_ERR_ALREADY_FEATHERED		11
#define RC_S_ERR_ALREADY_UNFEATHERING	12
#define RC_S_ERR_NOT_FEATHER_STATE		13
#define RC_S_ERR_POSITION_ERROR			14

#define RC_S_MAX_KEYWORD_LEN	12

WORD RC_S_error;
WORD RC_S_active;

int RC_S_trace;
int RC_S_count;
int RC_S_DI_count;		// Data interval
bool RC_S_send_DI_data=true;		// Allow stopping data by sending RC_D=0

//WORD RC_S_SETRPM_val;
char RC_S_keyword[RC_S_MAX_KEYWORD_LEN+2];
int RC_S_keyval;
int RC_S_value;
int RC_S_speed_value;
bool RC_S_terminate_msg = true;
char *RC_S_bps;

extern BYTE codeEntered;	// defined in comms.c

BYTE RC_X_zone=RC_S_ZONE_NORMAL;
BYTE RC_X_last_action;


BYTE RC_S_restart_check;
BYTE RC_S_was_reversing;

typedef enum
{
//	RC_S_MODE_ZERO=0,	// MHH:03/11/2023. When testing RC board this value was zero initially
	RC_S_MODE_IDLE=1,
	RC_S_MODE_C_IDLE,	// 2
	RC_S_MODE_SETPOS,	// 3
	RC_S_MODE_GETPOS,	// 4
	RC_S_MODE_REVERSE_POS,	// 5
	RC_S_MODE_SETSPEED,	// 6
	RC_S_MODE_FINE,		// 7
	RC_S_MODE_COARSE,	// 8
	RC_S_MODE_REVERSE_FINE,	// 9
	RC_S_MODE_REVERSE_COARSE,	// 10
	RC_S_MODE_DATA,
	RC_S_MODE_WAIT_STATECHECK,
	RC_S_MODE_WAIT_BRAKE,
	RC_S_MODE_SET_DATA_INTERVAL,
	RC_S_MODE_SET_DATA_FIELDS,
	RC_S_MODE_REQUEST_ENC_VERSION,
//	RC_S_MODE_REQUEST_ENC_FLAGS,
	RC_S_MODE_RESET_OVERCURRENT_TRIP,
	RC_S_MODE_FEATHER,	// MHH:07/11/2023
	RC_S_MODE_UNFEATHER,
	RC_S_CODE,		// MHH:21/11/2023
	RC_S_DATA,
	RC_S_STOP,		// MHH:25/02/2025
} RC_S_MODE;
RC_S_MODE RC_S_mode;
RC_S_MODE RC_S_next_mode;
int RC_S_next_val;
float RC_S_target_angle;
void RC_S_Terminate(void);
void RC_S_Terminate2(void);
void RC_S_Send_status(WORD eval,char ctype);
void RC_S_Send_Data(WORD eval);
//------------------------------------------------------------------------------------
#ifdef MH_DEBUG_SET_MODE
#define IFROM_MAX		16
uint16_t Ifrom_ix;
static uint16_t Ifrom_t[IFROM_MAX];
RC_S_MODE Imode_t[IFROM_MAX];
void Set_RC_S_mode(RC_S_MODE imode,int ifrom)
{
	RC_S_mode = imode;
	Imode_t[Ifrom_ix] = imode;
	Ifrom_t[Ifrom_ix++]=ifrom;
	Ifrom_ix %= IFROM_MAX;
}
#else
BYTE RC_S_mode_byte;
void Set_RC_S_mode(RC_S_MODE imode,int ifrom)
{
//#define MH_DEBUG_SETMODE2
#ifdef MH_DEBUG_SETMODE2
	int mode = (int)imode;
	PRINTF("Set_RC_S_mode:%d,%d\r\n",mode,ifrom);
#endif
	RC_S_mode = imode;
	RC_S_mode_byte = (BYTE) imode;
}
#endif
//------------------------------------------------------------------------------------
char RC_S_position_request(void)
{
	if(RC_S_mode == RC_S_MODE_SETPOS) return 'P';
	if(RC_S_mode == RC_S_MODE_REVERSE_POS) return 'p';

	return 0;
}
//------------------------------------------------------------------------------------
void Set_Zone(BYTE zone)		// So we can catch changes.
{
//	if(RC_X_zone != zone)	/ MHH:16/07/2019. On restart zone will not be in REVERSE, so will not check reverse relay when getting out of reverse
	{
		switch(zone)	// This switch to allow debug breakpoints
		{
		default:
			RC_X_zone = zone;
			break;

		case RC_S_ZONE_NORMAL:
			RC_X_zone = zone;
			AC210_check_reverse_relay();
			break;

		case RC_S_ZONE_REVERSE:
			RC_X_zone = zone;
			break;
		}
	}
}
//--------------------------------------------------------------------------------
BYTE RC_S_mode_is_setspeed(void)	// Called from LEDS.C to see if to set Feather LED to orange
{
    OpMode op_mode;
	BYTE result = false;

    op_mode = operatingMode();
    if(op_mode == HOLD)
    {
    	if(RC_S_mode == RC_S_MODE_SETSPEED)
    	{
    		result = true;
    	}
    }
    return result;
}
//--------------------------------------------------------------------------------
#ifdef MH_REMOTE_LED_SHIFT		// MHH:26/11/2020. Not used.
BYTE RC_S_mode_is_active(void)	// Called from LEDS.C to see if to set shift LED display
{
    OpMode op_mode;
	BYTE result = false;

    op_mode = operatingMode();
    if(op_mode == HOLD)
    {
    	if(RC_S_mode >= RC_S_MODE_C_IDLE)
    	{
    		result = true;
    	}
    }
    return result;
}
#endif
//--------------------------------------------------------------------------------
void RC_S_param_setspeed(int val)
{
	if(RC_S_mode != hwf.RC_S_mode || val != hwf.RC_S_setspeed)
	{
		hwf.RC_S_mode = RC_S_MODE_SETSPEED;
		hwf.RC_S_setspeed = val;
		RC_S_write_param();	// Is there any possibility this could conflict with stats? Different page, and AC200User updates seem OK.
	}
}
//--------------------------------------------------------------------------------
void RC_S_param_idle(void)
{
	if(hwf.RC_S_mode != RC_S_MODE_IDLE)
	{
		hwf.RC_S_mode = RC_S_MODE_IDLE;
		hwf.RC_S_setspeed = 0;
		RC_S_write_param();
	}
}
//--------------------------------------------------------------------------------
void RC_S_param_direct(void)		// Here from direct commands such as FINE,COARSE,REVERSE,OUT_REVERSE
{
	RC_S_active = true;
	if(hwf.RC_S_mode != RC_S_MODE_C_IDLE)
	{
		hwf.RC_S_mode = RC_S_MODE_C_IDLE;
		hwf.RC_S_setspeed = 0;
		RC_S_write_param();
	}
}
//--------------------------------------------------------------------------------
void RC_S_change_mode(RemoteState new_mode)
{
//	remoteState = new_mode;
	changeRemoteMode(new_mode);
	if(new_mode == REMOTE_IDLE)
	{
		RC_S_param_idle();
	}
}
//--------------------------------------------------------------------------------
void RC_S_set_idle_mode(void)
{
	RC_S_change_mode (REMOTE_IDLE);
	Set_RC_S_mode(RC_S_MODE_IDLE,100);
//	RC_S_mode = RC_S_MODE_IDLE;
	remoteSetSpeed = 0;
}
//--------------------------------------------------------------------------------
static void RC_S_set_count_zero(void)
{
	RC_S_count = 0;
}
//--------------------------------------------------------------------------------
void RC_S_Setspeed(int val);
int RC_S_Check_In_Reverse(void);
void RC_S_handle_restart(void)
{
	int val;
	switch(hwf.RC_S_mode)
	{
	case RC_S_MODE_SETSPEED:
		val = hwf.RC_S_setspeed;		// Similar logic to SetSpeed

		if(val)		// Check valid val
		{
	        if ( (val > getParameter(MAX_ENGINE_SPEED)) ||
	             (val  < getParameter(MIN_ENGINE_SPEED)) )
	        {
	        	RC_S_set_idle_mode();		// Default to Hold speed
//	        	RC_S_Status(RC_S_ERR_BAD_VALUE);
	    		return;
	        }
		}
		if(RC_S_Check_In_Reverse())
		{
            remoteStartReverseReverse();
            RC_S_change_mode (REMOTE_REVERSE_REVERSE);
            RC_S_next_mode = RC_S_MODE_SETSPEED;
            RC_S_next_val  = val;
        	Set_RC_S_mode(RC_S_MODE_REVERSE_COARSE,200);
//            RC_S_mode = RC_S_MODE_REVERSE_COARSE;	// Try using this mode, may need a special one
//    		RC_S_Status(RC_S_ERR_OK);
            return;
		}
		RC_S_Setspeed(val);
//		RC_S_Status(RC_S_ERR_OK);
		return;

	case RC_S_MODE_C_IDLE:		// Here if we restart after a direct command
//		RC_S_count = 0;
		RC_S_set_count_zero();
        RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
    	Set_RC_S_mode(RC_S_MODE_C_IDLE,300);
//   		RC_S_mode = RC_S_MODE_C_IDLE;
        controlDirection = C_IDLE;
		return;

//	case RC_S_MODE_ZERO:		// MHH:03/11/2023. Because RC board is initially set to zero.
	case RC_S_MODE_IDLE:		// Here if we restart and remote was not active
    	RC_S_set_idle_mode();		// Default to Hold speed
    	return;
	}
}
//--------------------------------------------------------------------------------
void RC_S_Setspeed(int val)
{
	if(val == 0)
	{
    	RC_S_set_idle_mode();		// Default to Hold speed
	}
	else
	{
        RC_S_change_mode (REMOTE_FLIGHT_MODE);
        remoteSetSpeed = val;
    	Set_RC_S_mode(RC_S_MODE_SETSPEED,400);
//        RC_S_mode = RC_S_MODE_SETSPEED;
	}
	controlReversing = false;
}
//------------------------------------------------
// These routines copied from drive.c and adapted.
//Brake Reverse = L2 + L3 = 0x40 + 0x80 = 0xC0

#ifdef MH_OLD_SOFTSTART

#define L23 0xC0

void setBrakeDutyCycle(WORD speed);
extern WORD AT_BRAKE_val;
extern WORD AT_BRAKE_cnt;
extern WORD AT_SoftStop_val;	// In cycles
extern WORD BrakeDutyCycle,BrakeIncrement;
#define BRAKE_START_DUTY	50		// Test!!

void setDirection (BYTE dir);
void setTheSpeed (WORD speed);

//------------------------------------------------
void RS_S_StartBrake(void)
{
//	txDebug("SB\r\n");

//	setTheSpeed (0);	// MHH: 12/12/15
	setIdle();			// MHH:29/08/2018

	BrakeDutyCycle = BRAKE_START_DUTY;
	BrakeIncrement = 75;	// 50,125,200,255,255
	AT_BRAKE_cnt = AT_BRAKE_val;

	setBrakeDutyCycle(BrakeDutyCycle);	// Initial value
	setDirection(L23);		// Do not set direction before speed!
}
//------------------------------------------------
int RC_S_KeepBraking(void)
{
	if(AT_BRAKE_cnt <= 0)
	{
		return false;
	}

//	txDebug("KB\r\n");

	AT_BRAKE_cnt--;

	if(BrakeDutyCycle < 255)
	{
		BrakeDutyCycle += BrakeIncrement;	// Increment Duty Cycle
		BrakeDutyCycle = MIN(BrakeDutyCycle,255);
		setBrakeDutyCycle(BrakeDutyCycle);
	}
	return true;	// Don't change state
}
#endif
//--------------------------------------------------------------------------------
void RC_S_start_state_test(void)
{
	Set_RC_S_mode(RC_S_MODE_WAIT_STATECHECK,500);
//	RC_S_mode = RC_S_MODE_WAIT_STATECHECK;
    controlDirection = C_IDLE;
    State_StartFastTest();
}
//--------------------------------------------------------------------------------
int RC_setpos_count;
int RC_setpos_pos;
extern int Encoder_Pos;
void RC_S_brushless_terminate(void);
void RC_S_setpos(void)
{
	float angle = AC210_SIG100_angle();
    float adiff = fabs(RC_S_target_angle - angle);

    if(adiff >= 0.05F)
    {
    	RC_setpos_count = 0;
    	RC_setpos_pos = Encoder_Pos;
    	if(angle > RC_S_target_angle)
        {
            controlDirection = C_FINER;
        }
        else
        {
            controlDirection = C_COARSER;
        }
        return;
    }


    if(RC_setpos_pos == Encoder_Pos)
    {
    	if(RC_setpos_count++ >= 3)		// MHH:20/02/2025. Check value stable for at least 3 cycles
    	{
    		RC_S_Terminate();
    		RC_S_brushless_terminate();
    	}
    }
    else
    {
    	RC_setpos_pos = Encoder_Pos;
    	RC_setpos_count = 0;
    }
}
//--------------------------------------------------------------------------------
//void setStateBits (WORD bits);
extern uint8_t Soft_mode;		// ac210_drive.c
extern bool Drive_wait_flag;

void RC_S_brushless_terminate(void)	// MHH:20/02/2025
{
	Set_RC_S_mode(RC_S_MODE_C_IDLE,1302);
    RC_S_speed_value = 0;
    Set_cState(C_IDLE,1302);
    Set_dState(MD_IDLE,1302);
    if(RC_S_terminate_msg)
    {
		RC_S_Terminate2();
    }
    controlDirection = C_IDLE;
}

void RC_S_ControlCycle()
{
    WORD sysState;
    WORD data_interval;
    OpMode op_mode;

    // Turn remote flag bits off.

    LED_overlay_flags &= ~(LED_OVERLAY_FLAG_REMOTE_ACTIVE | LED_OVERLAY_FLAG_REVERSE_ZONE);

    op_mode = operatingMode();
    if(op_mode == STARTUP)
    {
    	return;
    }

    // What if we are in reverse zone and auto_mode is changed from HOLD???

    if(op_mode != HOLD)
	{
    	RC_S_restart_check = true;
		RC_S_change_mode (REMOTE_IDLE);
		Set_RC_S_mode(RC_S_MODE_IDLE,700);
//		RC_S_mode = RC_S_MODE_IDLE;
//		RC_S_param_idle();	// Done in RC_S_change_mode()
	    State_fast_test = false;
		return;
	}

    if(RC_S_restart_check == false)		// First time through?
    {
    	RC_S_restart_check = true;
    	RC_S_handle_restart();
    }


    if(RC_S_mode != RC_S_MODE_IDLE)
    {
        LED_overlay_flags = LED_OVERLAY_FLAG_REMOTE_ACTIVE;
#ifdef MH_XXX
        if(RC_X_zone == RC_S_ZONE_REVERSE)
		{
			LED_overlay_flags |= LED_OVERLAY_FLAG_REVERSE_ZONE;
		}
#endif
        codeEntered = true;			// Allow AT commands
    	data_interval = ps.parms[RC_DATA_INTERVAL];
		if((ps.parms[RC_MISC_OPTIONS] & 2)==0)	// tenths of a second or ticks?
		{
			data_interval *= 5;			// tenths
		}

        if(data_interval > 0)
        {
            if(RC_S_send_DI_data)		// MHH:24/02/2025
            {
            	if(++RC_S_DI_count >= data_interval)
                {
                	RC_S_DI_count = 0;
                	RC_S_Send_Data(0);
                }
            }
        }
    }

    sysState = systemState();

    /*
     * 	 Need to know if we are in Reverse Zone.
     *   It is a bit tricky because STOP bits are removed from sysState while moving, but then added after
     *   State Test. We do not want to say we are out of the reverse zone until we know a state test has completed.
	 *   So, when we are waiting for a command then we will set a flag to say that if a state test finishes then
	 *   we will use those results to see if FINE stop is off, in which case we are out of reverse zone.
     */

    switch(RC_S_mode)
	{
	default:		// Get rid of compiler errors
		return;

	case RC_S_MODE_SETPOS:
		RC_S_setpos();
		switch(controlDirection)
		{
		default:
			break;

		case C_FINER:
	       	if(sysState & S_STOP_FINE)
	       	{
	       		RC_S_Terminate();
	       		RC_S_brushless_terminate();
//	       		Set_RC_S_mode(RC_S_MODE_WAIT_BRAKE,900);
//	            controlDirection = C_IDLE;
	       	}
			break;

		case C_COARSER:
	       	if(sysState & S_STOP_COARSE)
	       	{
	       		RC_S_Terminate();
	       		RC_S_brushless_terminate();
//	       		Set_RC_S_mode(RC_S_MODE_WAIT_BRAKE,1000);
//	       		RC_S_mode = RC_S_MODE_WAIT_BRAKE;
//	            controlDirection = C_IDLE;
	       	}
			break;

		}
        return;

		case RC_S_MODE_REVERSE_POS:
//			txDebug("RP:");
			RC_S_setpos();
//			RC_S_reverse_pos();
			switch(controlDirection)
			{
			default:
				break;

			case C_IDLE:
//				txDebug("C_IDLE");
				if(controlReversing)
				{
//					txDebug(":StopReverse");
		   			controlReversing = false;
					remoteStopReverse();
				}
				//				setIdle();	// ??
				break;

			case C_FINER:
//				txDebug("C_FINER");
		        if(sysState & S_STOP_REVERSE)
		       	{
//		       		txDebug(":STOP_R");
		        	Set_Zone(RC_S_ZONE_REVERSE);
		    		RC_S_set_count_zero();
//		       		RC_S_count = 0;
		       		if(controlReversing)
		       		{
//		       			txDebug(":rStopR");
		       			controlReversing = false;
		       			remoteStopReverse();
		       		}
		       		RC_S_Terminate();
		       		Set_RC_S_mode(RC_S_MODE_WAIT_BRAKE,1100);
//		       		RC_S_mode = RC_S_MODE_WAIT_BRAKE;
		            controlDirection = C_IDLE;
		       		break;
		       	}
		       	if(sysState & S_STOP_FINE)
		       	{
		        	Set_Zone(RC_S_ZONE_REVERSE);
		       	}
		       	RC_S_was_reversing = true;
		       	if(controlReversing == false)
		       	{
		   			controlReversing = true;
//		       		txDebug(":rStartR");
		       		remoteStartReverse();
		       	}
		       	RC_X_last_action = RC_X_REVERSE_FINE;
				break;

			case C_COARSER:
//				txDebug("C_COARSER");
				if(controlReversing)
				{
//					txDebug(":rStopR");	// In case goes from FINE to COARSE
		   			controlReversing = false;
					remoteStopReverse();
				}

				if(sysState & S_STOP_COARSE)
		       	{
//		       		txDebug(":STOP_C");
		       		RC_S_Terminate();
		       		Set_RC_S_mode(RC_S_MODE_WAIT_BRAKE,1200);
//		       		RC_S_mode = RC_S_MODE_WAIT_BRAKE;
		            controlDirection = C_IDLE;
		       	}
				break;

			}
//			txDebug("\r\n");
	        return;

	case RC_S_MODE_IDLE:
//    	Set_Zone(RC_S_ZONE_NORMAL);
        return;

	case RC_S_MODE_WAIT_STATECHECK:
		if(State_NextFastTest())	// Finished?
		{
       		Set_RC_S_mode(RC_S_MODE_C_IDLE,1300);
//	   		RC_S_mode = RC_S_MODE_C_IDLE;
	        controlDirection = C_IDLE;

	        if(Sig100_connected == false)
	        {
				sysState |= (State_fast_result << 4);
	        }

			if(RC_X_last_action == RC_X_FEATHER)	// MHH:14/11/2023
			{
				sysState &= ~S_STOP_REVERSE;
			}
			else
			{
				if(RC_S_was_reversing)
				{
					RC_S_was_reversing = false;
					if(sysState & (S_STOP_REVERSE | S_STOP_FINE))	// For compatibility with normal State test
					{
						Set_Zone(RC_S_ZONE_REVERSE);
					}
				}
				if(sysState & S_STOP_REVERSE)	// For compatibility with normal State test
				{
	//				sysState &= ~S_STOP_FINE;
					sysState &= ~(S_STOP_FINE | S_STOP_FEATHER);	// MHH:05/07/2019
				}
			}
			setStateBits(100,sysState);
			State_fast_test = false;
			RC_S_Terminate2();
		}
		return;

	case RC_S_MODE_WAIT_BRAKE:
#ifdef MH_OLD_SOFTSTART
		if(AT_BRAKE_cnt == 0)	// Finished?
		{
			RC_S_start_state_test();
		}
#else

		if(Drive_wait_flag)
		{
			mh_debug();
			return;
		}
		if(Sig100_connected) Soft_mode = 0;
		if(Soft_mode == 0)	// Finished?
		{
			RC_S_start_state_test();
		}
#endif
		return;

    case RC_S_MODE_SETSPEED:
//        setSpeed = (long)(remoteSetSpeed*10);
//        setSetSpeed ((long)(remoteSetSpeed*RPMFACTOR));	// MHH:20/03/2018
        setSetSpeed (remoteSetSpeed);	// MHH:20/03/2018
        return;


     case RC_S_MODE_C_IDLE:
        	return;

     case RC_S_MODE_FINE:
       	if(sysState & S_STOP_FINE)
       	{
    		RC_S_set_count_zero();
//       		RC_S_count = 0;
       	}
       	break;

     case RC_S_MODE_COARSE:
       	if(sysState & S_STOP_COARSE)
       	{
    		RC_S_set_count_zero();
//       		RC_S_count = 0;
       	}
       	else
       	{
       		RC_X_last_action = RC_X_COARSE;
       	}
       	break;

     case RC_S_MODE_FEATHER:
        if(sysState & S_STOP_FEATHER)
       	{
//       		Set_Zone(RC_S_ZONE_REVERSE);
            remoteStopFeather();
       		RC_S_set_count_zero();
            controlFeathering = false;
       		break;
       	}
        RC_X_last_action = RC_X_FEATHER;		// Not sure if needed
        break;

     case RC_S_MODE_UNFEATHER:
    	 if(remoteState != REMOTE_FEATHER_REVERSE)

//    	 if((sysState & (S_STOP_COARSE | S_STOP_FEATHER)) == 0)
    	 {
    	 //       		Set_Zone(RC_S_ZONE_REVERSE);
    	 	RC_S_set_count_zero();
    	 	break;
    	 }
         RC_X_last_action = RC_X_UNFEATHER;		// Not sure if needed
         break;

     case RC_S_MODE_REVERSE_FINE:
        if(sysState & S_STOP_REVERSE)
       	{
       		Set_Zone(RC_S_ZONE_REVERSE);
       		RC_S_set_count_zero();
       		break;
       	}
       	if(sysState & S_STOP_FINE)
       	{
        	Set_Zone(RC_S_ZONE_REVERSE);
       	}
       	RC_S_was_reversing = true;
       	RC_X_last_action = RC_X_REVERSE_FINE;

// Reverse is a special case, we will do the count down here as it is desirable to brake, otherwise 1/10th second becomes more like
// half a second which makes control difficult.

#ifdef MH_OLD_SOFTSTART

       	if(RC_S_count > 1)
       	{
           	RC_S_count--;
           	return;
       	}
       	RC_S_count--;
       	if(RC_S_count == 0)
       	{
       		RC_S_Terminate();
       		RS_S_StartBrake();
       		return;
       	}
       	if(RC_S_KeepBraking())
       	{
       		return;
       	}
		remoteStopReverse();
		RC_S_start_state_test();

//   		RC_S_mode = RC_S_MODE_C_IDLE;
//        controlDirection = C_IDLE;
		return;
#else
		break;
#endif

     case RC_S_MODE_REVERSE_COARSE:

    	 // Do we want to stop when FINE reached? May need to use Peter's state logic to know if reached stop
    	 // Possibly stop if they use RC_O=S		??

   		// This logic copied from Peter's routine.
   		// Do we want to go to REMOTE_IDLE? This means it should setspeed to Hold speed

   		// wait for the reverse_reverse cycle to complete
//   		Set_Zone(RC_S_ZONE_REVERSE);	// Was setting REVERSE_ZONE when only at FINE stop.
   		RC_X_last_action = RC_X_REVERSE_COARSE;
   		sysState = systemState();
        if (!(sysState & S_STOP_REVERSE) && !(sysState & S_STOP_FINE))
        {
            if ((featherState() == F_IDLE) && (driveState() == MD_IDLE))
            {
            	Set_Zone(RC_S_ZONE_NORMAL);
                // this will then be updated via the comms
            	switch(RC_S_next_mode)
            	{
            	default:		// Keep compiler happy
            		break;

            	case RC_S_MODE_SETSPEED:	// Here from RC_S=nnn
    	       		Set_RC_S_mode(RC_S_MODE_SETSPEED,1500);
//            		RC_S_mode = RC_S_MODE_SETSPEED;
					RC_S_Setspeed(RC_S_next_val);
            		RC_S_next_mode = RC_S_MODE_IDLE;
            		return;

            	case RC_S_MODE_COARSE:		// Here from RC_C=nnn
                    controlReversing = false;
                    RC_S_change_mode (REMOTE_GROUND_MODE);
                    controlDirection = C_COARSER;
    	       		Set_RC_S_mode(RC_S_MODE_COARSE,1600);
//                    RC_S_mode = RC_S_MODE_COARSE;
            		RC_S_count = RC_S_next_val;
            		return;

            	case RC_S_MODE_C_IDLE:		// Here from RC_RF=S
                    controlReversing = false;
                    RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
            		RC_S_set_count_zero();
//            		RC_S_count = 0;
               		RC_S_Terminate();

               		Set_RC_S_mode(RC_S_MODE_WAIT_BRAKE,1650);
            //   		RC_S_mode = RC_S_MODE_WAIT_BRAKE;
                    controlDirection = C_IDLE;

//            		RC_S_start_state_test();
            		return;

            	}

            	// If we drop through then no next command so set to idle.

	       		Set_RC_S_mode(RC_S_MODE_IDLE,1700);
//            	RC_S_mode = RC_S_MODE_IDLE;
                RC_S_change_mode (REMOTE_IDLE);
            	return;								// and this one
            }
        }
        return;				// This is supposed to not stop

	}
   	if(RC_S_count > 1)
   	{
       	RC_S_count--;
   	}
   	else
   	{
   		if(controlReversing)
   		{
   			controlReversing = false;
   			remoteStopReverse();
   		}
   		RC_S_Terminate();

//	   		RC_S_mode = RC_S_MODE_C_IDLE;
		if(Sig100_connected)	// MHH:06/02/2025
		{
			RC_S_brushless_terminate();
#ifdef MH_XXX
	   		Set_RC_S_mode(RC_S_MODE_C_IDLE,1302);
	        RC_S_speed_value = 0;
	        Set_cState(C_IDLE,1302);
	        Set_dState(MD_IDLE,1302);
	        if(RC_S_terminate_msg)
	        {
				RC_S_Terminate2();
	        }
#endif
		}
		else
		{
	   		Set_RC_S_mode(RC_S_MODE_WAIT_BRAKE,1800);
		}

//		RC_S_start_state_test();
//   		RC_S_mode = RC_S_MODE_WAIT_BRAKE;
        controlDirection = C_IDLE;
   	}
   	return;
}
//----------------------------------------------------------------------------------------------
static uint32_t RC_usecs;
void RC_S_ms_display(void)
{
#ifdef AC210_PORT
	if(RC_S_trace == 0) return;

	uint32_t ms = AC210_ms_elapsed(&RC_usecs);
	AC210_ms_display("RC",ms);
#endif
}
//------------------------------------------------------------------------------------------------
int iround(float f)
{
	if(f >= 0)
	{
		f += 0.5F;
	}
	else
	{
		f -= 0.5F;
	}
	int ival = (int)f;
	return ival;
}
//CAN_MSG_T CAN_snd_status_msg
extern CAN_MSG_T CAN_snd_status_msg;
extern void CAN_send_status(void);
#define CAN_ANGLE_DIVISOR	10

void RC_S_send_CAN_data_fields(void)
{
	WORD motor_current;
	WORD voltage;
	WORD led_status;
	int16_t iangle;
	WORD val;

    CAN_snd_status_msg.ID = 720 + ps.parms[RC_PROPNUM];
    CAN_snd_status_msg.Type = 0;		// Not CAN_REMOTE_MSG


    /*
     * Simplest to send all values, not just requested. That way we know position.
     */

    val = scaledValue (A_MOTOR_CURRENT);
    if(val < 10) val = 0;
    val = (val/10) * 10;
    motor_current = val;


    CAN_snd_status_msg.Data[0] = (motor_current >> 8);	// Think in milli-amps
    CAN_snd_status_msg.Data[1] = motor_current & 255;

    val = scaledValue (A_SUPPLY);		// Note 1 implied decimal place
    voltage = val;
    CAN_snd_status_msg.Data[2] = (voltage >> 8);
    CAN_snd_status_msg.Data[3] = voltage & 255;

    led_status = XoarStatus;
    CAN_snd_status_msg.Data[4] = (led_status >> 8);
    CAN_snd_status_msg.Data[5] = led_status & 255;

    float angle = AC210_SIG100_angle();
//    angle = angle * CAN_ANGLE_DIVISOR + 0.5F;		// 1 implied decimal place
    angle = angle * CAN_ANGLE_DIVISOR;		// 1 implied decimal place
    iangle = (int16_t)iround(angle);
    CAN_snd_status_msg.Data[6] = (iangle >> 8);
    CAN_snd_status_msg.Data[7] = iangle & 255;

    CAN_snd_status_msg.DLC = 8;		// Message length = 8 bytes

    CAN_send_status();


}
void RC_send_data_fields(void)
{
	char field[20];
	WORD data_fields;
	WORD val;

	data_fields = getParameter(RC_DATA_FIELDS);
	if(data_fields & 1)	// Current?
	{
        val = scaledValue (A_MOTOR_CURRENT);
        if(val < 10) val = 0;
        val = (val/10) * 10;
		sprintf(field,",%d",val);
		Remote_reply(field);
	}
	if(data_fields & 2)	// Voltage?
	{
		if(Sig100_connected)
		{
			float fvoltage = SIG100_get_hub_voltage();
#ifdef MH_XXX
			float fvoltage = (float)(ADC_adjusted_voltage * 16);
			fvoltage /= 10.954;		// By experiment, should have an implied decimal place
#endif
			int voltage = (int)(fvoltage + 0.5);
			val = voltage;
		}
		else
		{
			val = scaledValue (A_SUPPLY);
		}

		sprintf(field,",%d.%d",val/10,val%10);
		Remote_reply(field);
	}
	if(data_fields & 4)	// LED_status?
	{
        val = XoarStatus;
		sprintf(field,",%d",val);
		Remote_reply(field);
	}
	if(data_fields & 8)	// position?
	{
        float angle = AC210_SIG100_angle();
		sprintf(field,",%3.2f",angle);
		Remote_reply(field);
	}
}
//-----------------------------------------------------------------------------------------------

BYTE RC_S_get_cmode(void)
{
	BYTE cmode;
	switch (RC_S_mode)
	{
	default:
		cmode = 'D';
		break;

	case RC_S_MODE_SETSPEED:
		cmode = 'S';
		break;

	case RC_S_MODE_IDLE:
		cmode = 'I';
		break;

	case RC_S_MODE_SETPOS:			// MHH:24/02/2025
	case RC_S_MODE_REVERSE_POS:
		cmode = 'P';
		break;

	case RC_S_MODE_C_IDLE:
		cmode = 'W';
		break;
	}
	return cmode;
}

void RC_S_Send_CAN_status2(WORD eval,char ctype)
{
//	BYTE stops;
	WORD status;
    WORD st;
	BYTE cmode;
	WORD actualspeed;
	WORD setspeed;

	cmode = RC_S_get_cmode();
#ifdef MH_XXX
	if(ctype == 'S')		// Status?
	{
		if((ps.parms[RC_MISC_OPTIONS] & 4) && (ps.parms[RC_DATA_INTERVAL] != 0))	// Short status && rc data interval != 0??
		{
			sprintf(msg,"RS_S:%d,%c",eval,cmode);
			Remote_reply(msg);
			return;
		}
	}
#endif
	actualspeed = currentActualSpeed ();
    st = systemState();
//    stops = ((st >> 4) & 0x07) + '0';
    status = (st >> 4);		// Ignore run state

//    RC_S_ms_display();
    setspeed = remoteSetSpeed;		// MHH:14/02/2018

    CAN_snd_status_msg.ID = 700 + ps.parms[RC_PROPNUM];
    CAN_snd_status_msg.Type = 0;		// Not CAN_REMOTE_MSG

	CAN_snd_status_msg.Data[0] = ctype;
	CAN_snd_status_msg.Data[1] = eval;
	CAN_snd_status_msg.Data[2] = cmode;
	CAN_snd_status_msg.Data[3] = status;
    uint16_t ival= 0;
    if(cmode != 'S')
    {
    	setspeed = 0;
    	if(Sig100_connected)	// MHH:07/02/2025. Send blade angle in setspeed position if not setspeed command
    	{
    	    float angle = AC210_SIG100_angle();
//    	    angle = angle * CAN_ANGLE_DIVISOR + 0.5F;		// 1 implied decimal place
    	    angle *=  CAN_ANGLE_DIVISOR;		// 1 implied decimal place
    	    ival = (uint16_t)iround(angle);		// MHH:28/04/2025
//   	   		sprintf(msg,"RS_%c:%d,%c,%d,%3.2f,%d",ctype,eval,cmode,status,angle,actualspeed);
    	}
    }
    else	// MHH:28/04/2025
    {
    	ival = setspeed;
    }
//    if(ival == 0) ival = setspeed;
    CAN_snd_status_msg.Data[4] = ival >> 8;		// Bigendian
    CAN_snd_status_msg.Data[5] = ival & 255;	// Bigendian
    CAN_snd_status_msg.Data[6] = actualspeed >> 8;
    CAN_snd_status_msg.Data[7] = actualspeed & 255;

    CAN_snd_status_msg.DLC = 8;		// Message length = 8 bytes

    CAN_send_status();
}

//------------------------------------------------------------------------------------------------
void RC_S_Send_status2(WORD eval,char ctype)
{
//	BYTE stops;
	WORD status;
    WORD st;
	BYTE cmode;
	WORD actualspeed;
	WORD setspeed;

	char msg[30],msg2[20];
	if(AC210_remote_control_port == 2)
	{
		RC_S_Send_CAN_status2(eval,ctype);
		return;
	}
	cmode = RC_S_get_cmode();
	if(ctype == 'S')		// Status?
	{
		if((ps.parms[RC_MISC_OPTIONS] & 4) && (ps.parms[RC_DATA_INTERVAL] != 0))	// Short status && rc data interval != 0??
		{
			sprintf(msg,"RS_S:%d,%c",eval,cmode);
			Remote_reply(msg);
			return;
		}
	}
	actualspeed = currentActualSpeed ();
    st = systemState();
//    stops = ((st >> 4) & 0x07) + '0';
    status = (st >> 4);		// Ignore run state

//    RC_S_ms_display();
    msg2[0] = 0;		// default is no data
    if(cmode == 'S')
    {
        setspeed = remoteSetSpeed;		// MHH:14/02/2018
    	sprintf(msg2,"%d",setspeed);
    }
    else
    {
    	if(Sig100_connected)	// MHH:07/02/2025. Send blade angle in setspeed position if not setspeed command
    	{
    		WORD data_fields = getParameter(RC_DATA_FIELDS);
    		float angle;
    		if(data_fields & 8)	// data position?
    		{
    			if(cmode == 'P')
    			{
        			angle = RC_S_target_angle;		// MHH:19/06/2025 Then use target angle, as per Xoar request
        			sprintf(msg2,"%3.2f",angle);
    			}
    		}
    		else
    		{
        		angle = AC210_SIG100_angle();
        		sprintf(msg2,"%3.2f",angle);
    		}
    	}
    }
    sprintf(msg,"RS_%c:%d,%c,%d,%s,%d",ctype,eval,cmode,status,msg2,actualspeed);

//	txDebug(msg);
	Remote_reply(msg);
}
//------------------------------------------------------------------------------------------------
void RC_S_Send_status(WORD eval,char ctype)
{

	RC_S_Send_status2(eval,ctype);
	if(AC210_remote_control_port != 2)
	{
		Remote_reply("\r\n");
	}
}
//------------------------------------------------------------------------------------------------
void RC_S_Send_Data(WORD eval)
{
	RC_S_Send_status2(eval,'D');

	if(AC210_remote_control_port == 2)
	{
		RC_S_send_CAN_data_fields();
	}
	else
	{
		RC_send_data_fields();
		Remote_reply("\r\n");
	}
}
//------------------------------------------------------------------------------------------------
void RC_S_Status(WORD eval)
{

//    long setspeed    = currentSetSpeed ();
	RC_S_Send_status(eval,'S');
	if(eval == 0)
	{
		codeEntered = 1;		// Allow AT commmands
//		RC_S_active = true;		// So on a restart in Hold mode we will continue
	}
}
//------------------------------------------------------------------------------------------------
void RC_S_Terminate2(void)
{
	RC_S_Send_status(0,'T');
}
//------------------------------------------------------------------------------------------------
void RC_S_Terminate(void)
{
//    RC_S_ms_display();
// May want to save elapsed time here.

	RC_S_active = false;
}
//------------------------------------------------------------------------------------------------
#define RC_S_GETVAL_ERR 32767
#define RC_S_GETVAL_S	32766
#define RC_S_NO_VAL		32765

#define RC_S_OK 	0
//-----------------------------------------------------------------------------------------------------
/*
 * idec = maximum decimal allowed
 * ier  = error output;
 */

#define FN_GETVAL_ERR		-1
int fn_getval(int idec,int icnt,int *ier)
{
	int len;
	int val = 0;
	int dec_count=0;
	bool dot_found = false;
	bool minus_found=false;
	char c;
	int err = 0;

	for (len=0;len<7;len++)
	{
		c = *RC_S_bps++;
		if(c >= '0' && c <= '9')
		{
			if(dot_found)
			{
				if(++dec_count > idec)
				{
					err = FN_GETVAL_ERR;
					break;
				}
			}
			val *= 10;
			val += (c - '0');
			continue;
		}
		if(c == '-')
		{
			if(len == 0 && idec > 0)	// must be first character of decimal number for angle
			{
				minus_found = true;
				continue;
			}
			err = FN_GETVAL_ERR;
			break;
		}
		if(c == '.')
		{
			if(idec > 0 && dot_found == false)
			{
				dot_found = true;
				continue;
			}
			err = FN_GETVAL_ERR;
			break;
		}
		if(c == 0)	// Terminator?
		{
			break;
		}
		if(c == ',')
		{
			if(icnt) err = FN_GETVAL_ERR;
			break;
		}
		if(c == ':')
		{
			break;
		}
	}

	*ier = err;
	if(err)
	{
		return 0;
	}
	for(int i=dec_count;i<idec;i++)	// Handle implied decimal place
	{
		val *= 10;
	}
	if(minus_found)
	{
		val = -val;
	}
	RC_S_bps--;			// Adjust ptr so we know what terminator was
#ifdef MH_XXX			// MHH:24/02/2025. In case of SETRPM=0 0 is a valid value.
	if(idec == 0)
	{
		if(val == 0)
		{
			val = RC_S_NO_VAL;
		}
	}
#endif
	return val;
}
//-----------------------------------------------------------------------------------------------------
int RC_S_getval(int idec)
{
	// OK, we have the keyword, now to extract the value. bps should point to first byte of val
	// Assume value must be no larger than 9999.
	// Deal with decimal places if we implement positioning to an angle

	int val,val2;
	int val_speed1=0;
	int val_speed2=0;
	char c;
	int err;

	RC_S_value = 0;

	c = RC_S_bps[0];		// Check for 'S'
	if(c == 'S')
	{
		if(RC_S_bps[1] == 0 || strcmp(RC_S_bps,"S,S") == 0)
		{
			RC_S_value = RC_S_GETVAL_S;
			return RC_S_OK;
		}
	}

	val = fn_getval(idec,0,&err);
	if(err)
	{
		return RC_S_ERR_BAD_VALUE;			// Invalid character
	}
	c = *RC_S_bps++;

/*
 *  MHH:06/02/2025. If c == ':' then assume a speed argument.
 */
	if(idec == 0)
	{
		if(c == ':')
		{
			val_speed1 = fn_getval(0,0,&err);
			c = *RC_S_bps++;
		}
	}
	if(c == ',')
	{
		val2 = fn_getval(idec,1,&err);
		if(err)
		{
			return RC_S_ERR_BAD_VALUE;			// Invalid character
		}
		c = *RC_S_bps++;
		if(idec == 0)
		{
			if(c == ':')
			{
				val_speed2 = fn_getval(0,0,&err);
				c = *RC_S_bps++;
			}
			if(val_speed1 != val_speed2)
			{

			}
		}
		if(val != val2)
		{
			return RC_S_ERR_BAD_CHECK_VALUE;
		}
		if(val_speed1 != 0)
		{
			if(val_speed1 < 1 || val_speed1 > 9)
			{
				return RC_S_ERR_BAD_VALUE;			// Invalid character
			}
			if(val_speed2 != 0)
			{
				if(val_speed1 != val_speed2)
				{
					return RC_S_ERR_BAD_VALUE;			// Invalid character
				}
			}
		}
	}
	if(c)
	{
		return RC_S_ERR_BAD_VALUE;			// Invalid terminator
	}
	if(val_speed1 > 0)
	{
		RC_S_speed_value = val_speed1;
	}
	RC_S_value = val;
	return RC_S_OK;
}
//------------------------------------------------------------------------------------------------
typedef struct
{
	char far *keyword;
	int keyval;
}RC_S_keywords_TD;

// In alphabetical order

static const RC_S_keywords_TD RC_S_keywords[]=
{
		{"C",RC_S_MODE_COARSE},
		{"D",RC_S_MODE_DATA},
		{"F",RC_S_MODE_FINE},
		{"STOP",RC_S_STOP},
		{"RC",RC_S_MODE_REVERSE_COARSE},
		{"RF",RC_S_MODE_REVERSE_FINE},
		{"S",RC_S_MODE_SETSPEED},
		{"SETRPM",RC_S_MODE_SETSPEED},
		{"SETPOS",RC_S_MODE_SETPOS},
		{"GETPOS",RC_S_MODE_GETPOS},
		{"RPOS",RC_S_MODE_REVERSE_POS},
		{"SET_DI",RC_S_MODE_SET_DATA_INTERVAL},
		{"SET_DF",RC_S_MODE_SET_DATA_FIELDS},
		{"ENCVER",RC_S_MODE_REQUEST_ENC_VERSION},
//		{"ENCFLAGS",RC_S_MODE_REQUEST_ENC_FLAGS},
		{"RESET_OC",RC_S_MODE_RESET_OVERCURRENT_TRIP},
		{"FEATHER",RC_S_MODE_FEATHER},
		{"UNFEATHER",RC_S_MODE_UNFEATHER},
		{"CODE",RC_S_CODE},
		{"DATA",RC_S_DATA},
		{"",0}
};


int RC_S_get_keyword(void)
{
	int len;
	char *bpd = RC_S_keyword;

	char far *kw_p;
	char c;
	int i;
	int cmp;

	if(*RC_S_bps++ != '_')
	{
		return RC_S_ERR_BAD_COMMAND;
	}

	// Extract keyword

	for(len=1;len<RC_S_MAX_KEYWORD_LEN;len++)
	{
		c = *RC_S_bps++;
		if(c == '=' || c == 0)
		{
			*bpd++ = 0;		// null terminate
			break;
		}
		if(c < 'A' || c > 'Z')		// Check valid character
		{
			if(c != '_')
			{
				return RC_S_ERR_BAD_COMMAND;
			}
		}
		*bpd++ = c;			// Valid character, save
	}
	RC_S_bps--;					// So we point to current byte


// Now translate keyword to value

	for(i=0;;i++)
	{
		kw_p = RC_S_keywords[i].keyword;
		if(*kw_p == 0)
		{
			return RC_S_ERR_BAD_COMMAND;		// End of list
		}
		cmp = strcmp(RC_S_keyword,kw_p);
		if(cmp == 0)		// Match?
		{
			RC_S_keyval = RC_S_keywords[i].keyval;
			return RC_S_OK;
		}
#ifdef MH_ALPHA_ORDER
		if(cmp < 0)		// Have we passed where it would be?
		{
			return RC_S_ERR_BAD_COMMAND;		// No match
		}
#endif
	}
}
//--------------------------------------------------------------------------------
int RC_S_Check_In_Reverse(void)
{
//	int state;

	if (remoteState == REMOTE_GROUND_MODE)
	{
	    // just in case
	    if(Control_type == CT_REVERSE)
	    {
	        if (controlReversing)
	        {
	            remoteStopReverse();
	        }

	        if(RC_X_zone == RC_S_ZONE_REVERSE)		// MHH:15/01/2026
#ifdef MH_XXX
	        	state = systemState();
	        if ((state & S_STOP_REVERSE)|| (state & S_STOP_FINE))
#endif
	        {
	            return true;
	        }
	    }
	    // if we are in the reversed range, we need to back out
	}
	return false;
}
//------------------------------------------------------------------------------------------------
int RC_S_fnval(int val)
{
    if(val == RC_S_NO_VAL) val = 1;	// Time-out in tenths of a second
	if(val == RC_S_GETVAL_S) val = 999;	// reasonable limit = 99.9 secs

	uint16_t rc_terminate_min = getParameter(RC_TERMINATE_MIN);
	RC_S_terminate_msg = (val >= rc_terminate_min);	// If tenths >= minimum value then send terminate message

	val *=  5;		// as 5 ticks every tenth of a sec
	val++;		// ???
	return val;
}
//------------------------------------------------------------------------------------------------
char Get_SIG100_speed_from_remote(void)
{

	switch(RC_S_mode)
	{
	default:
		return -1;

	case RC_S_MODE_FINE:
	case RC_S_MODE_COARSE:
	case RC_S_MODE_REVERSE_FINE:
	case RC_S_MODE_REVERSE_COARSE:
		return RC_S_speed_value;
	}
}
//------------------------------------------------------------------------------------------------
int fn_tenths_to_ticks(int val)
{
	return val * 5 + 1;
}
extern bool CAN_new_command;
extern CAN_MSG_T CAN_rcv_command_msg;

void RC_S_feather(void)
{
	RC_S_param_direct();
	if(Sig100_connected)
	{
		if(isFeatheringProp() == false)
		{
			RC_S_Status(RC_S_ERR_NO_FEATHER);
			return;
		}
	}
	else
	{
		if(Control_type != CT_FEATHERING)	// Note: Will need to modify for brushless
				{
			RC_S_Status(RC_S_ERR_NO_FEATHER);
			return;
				}
	}

	if(controlFeathering)
	{
		RC_S_Status(RC_S_ERR_ALREADY_FEATHERING);
		return;
	}

	WORD state = systemState();
	if (state & S_STOP_FEATHER)
	{
		RC_S_Status(RC_S_ERR_ALREADY_FEATHERED);
		return;

	}
	// Possible should do an RPM test here, similar to reverse.

	remoteStartFeather();
	//		startFeather();
	controlStop = 'f';
	controlDirection = C_IDLE;
	controlFeathering = true;
	//        Xoar_feather = true;

	RC_S_change_mode (REMOTE_GROUND_MODE);		// for now

	Set_RC_S_mode(RC_S_MODE_FEATHER,1810);	// MHH:10/06/2025
//	RC_S_mode = RC_S_MODE_FEATHER;
//	RC_S_count = 999;
	RC_S_count = 2000;		// MHH:16/01/2026. 1000/50 = 20 secs, not long enough for some very slow hubs. Try 2000 (40 secs)
	RC_S_Status(RC_S_ERR_OK);
}
void RC_S_unfeather(void)
{
    RC_S_param_direct();
    if(Sig100_connected)
    {
    	if(isFeatheringProp() == false)
    	{
    		RC_S_Status(RC_S_ERR_NO_FEATHER);
    		return;
    	}
    }
    else
    {
        if(Control_type != CT_FEATHERING)	// Note: Will need to modify for brushless
        {
			RC_S_Status(RC_S_ERR_NO_FEATHER);
			return;
        }
    }
    if(RC_S_mode == RC_S_MODE_UNFEATHER)	// Already getting out of feather?
    {
		RC_S_Status(RC_S_ERR_ALREADY_UNFEATHERING);
		return;

    }

    if (controlFeathering)
    {
        remoteStopFeather();
//        controlFeathering = false;	// MHH:16/01/2026. Because we stops were not active while feathering
    }

    WORD state = systemState();
    if ((state & S_STOP_FEATHER)|| (state & S_STOP_COARSE) || controlFeathering)
    {
        remoteStartFeatherReverse();
        RC_S_change_mode (REMOTE_FEATHER_REVERSE);		// for now
        Set_RC_S_mode(RC_S_MODE_UNFEATHER,1820);	// MHH:10/06/2025
//        RC_S_mode = RC_S_MODE_UNFEATHER;
//  		RC_S_count = 999;
  		RC_S_count = 2000;		// MHH:16/01/2026. 1000/50 = 20 secs, not long enough for some very slow hubs. Try 2000 (40 secs)
  		RC_S_Status(RC_S_ERR_OK);
    }
    else
    {
    	RC_S_Status(RC_S_ERR_NOT_FEATHER_STATE);
    }
    controlFeathering = false;

}
void RC_S_stop_move(void)
{
	RC_S_terminate_msg = false;
	RC_S_Terminate();
	if(Sig100_connected)	// MHH:06/02/2025
	{
		RC_S_brushless_terminate();
	}
	else
	{
		Set_RC_S_mode(RC_S_MODE_WAIT_BRAKE,1830);
	}

	controlDirection = C_IDLE;
	//		RC_S_set_count_zero();
	RC_S_Status(RC_S_ERR_OK);
}

void checkRemoteCANcommand(void)
{
	int val;
	int16_t val16;

	if(CAN_new_command == false) return;

	CAN_new_command = false;

	/*
	 *       CAN_transmit_rec.ID = CAN_ID_MOVE_FINE;
	      CAN_transmit_rec.Data[0] = CAN_hub_transmit_id;
	      int tenths = int.Parse(tb_move_fine_tenths.Text);
	      CAN_transmit_rec.Data[1] = (byte)tenths;
	      CAN_transmit_rec.Data[2] = CAN_motor_speed;
	      CAN_transmit_rec.DLC = 3;   // Data length = 3
	 */


	switch(CAN_rcv_command_msg.ID)
	{
	case 501:		// Move Fine
		RC_S_param_direct();
		RC_S_count = fn_tenths_to_ticks(CAN_rcv_command_msg.Data[1]);
		RC_S_speed_value =  CAN_rcv_command_msg.Data[2];
		RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
		controlDirection = C_FINER;
		Set_RC_S_mode(RC_S_MODE_FINE,5200);
		RC_S_Status(RC_S_ERR_OK);		// Possibly should check stops first?
		return;

	case 502:		// Move coarse
		RC_S_param_direct();
		controlReversing = false;
		RC_S_count = fn_tenths_to_ticks(CAN_rcv_command_msg.Data[1]);
		RC_S_speed_value =  CAN_rcv_command_msg.Data[2];
		RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
		controlDirection = C_COARSER;		// Not sure if needed
		Set_RC_S_mode(RC_S_MODE_COARSE,5400);
		RC_S_Status(RC_S_ERR_OK);
		return;

	case 503:		// Move Fine in reverse
		RC_S_param_direct();
		if(Control_type != CT_REVERSE)
		{
			RC_S_Status(RC_S_ERR_NO_REVERSE);
			return;
		}
		if(Reverse_RPM_too_high())
		{
			RC_S_Status(RC_S_ERR_RPM_TOO_HIGH);
			return;
		}
		controlReversing = true;
		RC_S_count = fn_tenths_to_ticks(CAN_rcv_command_msg.Data[1]);
		RC_S_speed_value =  CAN_rcv_command_msg.Data[2];
		remoteStartReverse();
		controlDirection = C_FINER;		// Not sure if needed
		RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
		Set_RC_S_mode(RC_S_MODE_REVERSE_FINE,5410);	// MHH:10/06/2025
//		RC_S_mode = RC_S_MODE_REVERSE_FINE;
		RC_S_Status(RC_S_ERR_OK);
		return;

	case 504:		// Move Coarse in reverse
		if(RC_S_Check_In_Reverse() == false)	// Similar logic to SetSpeed
		{
			RC_S_Status(RC_S_ERR_NOT_IN_REVERSE);
			return;
		}
		RC_S_param_direct();
//		if(val == RC_S_GETVAL_S)		// Going to a stop?
		if(CAN_rcv_command_msg.Data[1] == 255)	// Going to a stop?
		{
			remoteStartReverseReverse();
			RC_S_change_mode (REMOTE_REVERSE_REVERSE);
			RC_S_next_mode = RC_S_MODE_C_IDLE;
			RC_S_next_val  = 0;
			Set_RC_S_mode(RC_S_MODE_REVERSE_COARSE,2500);
			//            RC_S_mode = RC_S_MODE_REVERSE_COARSE;	// Try using this mode, may need a special one
			RC_S_Status(RC_S_ERR_OK);
			return;
		}
		RC_S_count = fn_tenths_to_ticks(CAN_rcv_command_msg.Data[1]);
		RC_S_speed_value =  CAN_rcv_command_msg.Data[2];
		RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
		controlDirection = C_COARSER;

		if (controlReversing)
		{
			controlReversing = false;			// as per existing logic??
			remoteStopReverse();
		}

		Set_RC_S_mode(RC_S_MODE_COARSE,2600);
		//        RC_S_mode = RC_S_MODE_COARSE;
		RC_S_Status(RC_S_ERR_OK);
		return;

	case 505:		// Stop move
		RC_S_stop_move();
		return;

	case 510:		// Setspeed
		val = (CAN_rcv_command_msg.Data[1] << 8) | CAN_rcv_command_msg.Data[2];
		if(val)		// Check valid val here, in case we need for next command
		{
			if ( (val > getParameter(MAX_ENGINE_SPEED)) ||
					(val  < getParameter(MIN_ENGINE_SPEED)) )
			{
				RC_S_Status(RC_S_ERR_BAD_VALUE);
				return;
			}
		}
		RC_S_param_setspeed(val);		// If we restart we want to get out of reverse and then setspeed.
		if(RC_S_Check_In_Reverse())
		{
			remoteStartReverseReverse();
			RC_S_change_mode (REMOTE_REVERSE_REVERSE);
			RC_S_next_mode = RC_S_MODE_SETSPEED;
			RC_S_next_val  = val;
			Set_RC_S_mode(RC_S_MODE_REVERSE_COARSE,2100);
			//            RC_S_mode = RC_S_MODE_REVERSE_COARSE;	// Try using this mode, may need a special one
			RC_S_Status(RC_S_ERR_OK);
			return;
		}
		RC_S_Setspeed(val);
		RC_S_Status(RC_S_ERR_OK);
		return;

	case 520:		// Set Pos
		val = (CAN_rcv_command_msg.Data[1] << 8) | CAN_rcv_command_msg.Data[2];
		if(Sig100_connected == false)
		{
			RC_S_Status(RC_S_ERR_SIG100_NOT_CONNECTED);
			return;
		}
#ifdef MH_XXX

		if(val == RC_S_NO_VAL)	// Was there an '='?
		{
			//	        angle = AC210_SIG60_angle();	// no, then just display value
			angle = AC210_SIG100_angle();
			sprintf(field,"%3.2f\r\n",angle);
			Remote_reply(field);
			return;
		}
#endif
		RC_S_target_angle = ((float)val)/CAN_ANGLE_DIVISOR;	// Two implied decimal places
		RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
		Set_RC_S_mode(RC_S_MODE_SETPOS,1900);
		controlDirection = C_IDLE;		// So we know it is first time
		RC_S_terminate_msg = true;
		RC_S_Status(RC_S_ERR_OK);		// Possibly should check stops first?
		return;

	case 521:	// Reverse Pos
//		val = (CAN_rcv_command_msg.Data[1] << 8) | CAN_rcv_command_msg.Data[2];
		val16 = (CAN_rcv_command_msg.Data[1] << 8) | CAN_rcv_command_msg.Data[2];		// MHH:28/04/2025
		val = val16;
    	if(Sig100_connected == false)
    	{
			RC_S_Status(RC_S_ERR_SIG100_NOT_CONNECTED);
    		return;
    	}

#ifdef MH_XXX
    	if(val == RC_S_NO_VAL)	// Was there an '='?
		{
	        angle = AC210_SIG100_angle();	// no, then just display value
			sprintf(field,"%3.2f\r\n",angle);
			Remote_reply(field);
			return;
		}
#endif
        RC_S_param_direct();
        if(Control_type != CT_REVERSE)
        {
			RC_S_Status(RC_S_ERR_NO_REVERSE);
			return;
        }
    	if(Reverse_RPM_too_high())
    	{
			RC_S_Status(RC_S_ERR_RPM_TOO_HIGH);
    		return;
    	}
        RC_S_target_angle = (float)(val/CAN_ANGLE_DIVISOR);	// Two implied decimal places
// Note: May want to check against fine and coarse stop values.
        RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
//        controlDirection = C_FINER;
   		Set_RC_S_mode(RC_S_MODE_REVERSE_POS,2000);
//        RC_S_mode = RC_S_MODE_REVERSE_POS;
        controlDirection = C_IDLE;		// So we know it is first time
        RC_S_terminate_msg = true;
        RC_S_Status(RC_S_ERR_OK);		// Possibly should check stops first?
		return;

	case 530:		// Feather
		RC_S_feather();
		return;

	case 531:		// Unfeather
		RC_S_unfeather();
		return;

	case 540:		// Get CTL data
		CAN_snd_status_msg.ID = 710 + ps.parms[RC_PROPNUM];
		CAN_snd_status_msg.Type = 0;		// Not CAN_REMOTE_MSG

		CAN_snd_status_msg.Data[0] = VERSION >> 8;
		CAN_snd_status_msg.Data[1] = VERSION & 255;
		CAN_snd_status_msg.Data[2] = ps.parms[AC_CONTROL_WORD] >> 8;
		CAN_snd_status_msg.Data[3] = ps.parms[AC_CONTROL_WORD] & 255;
		CAN_snd_status_msg.Data[4] = ps.parms[BL_HUB_SOFTWARE_VERSION] >> 8;
		CAN_snd_status_msg.Data[5] = ps.parms[BL_HUB_SOFTWARE_VERSION] & 255;
		CAN_snd_status_msg.Data[6] = 0;		// spare
		CAN_snd_status_msg.Data[7] = ps.parms[RC_DATA_FIELDS] & 255;

		CAN_snd_status_msg.DLC = 8;		// Message length = 8 bytes

		CAN_send_status();
		return;

	case 541:
//	case RC_S_MODE_DATA:		// Maybe a RC_D command?
		val = CAN_rcv_command_msg.Data[1];
		if(val == 0) RC_S_send_DI_data = false;
		if(val == 1) RC_S_send_DI_data = true;

		//RC_S_MODE_IDLE=1,
//		RC_S_MODE_C_IDLE
		if(remoteMode() == REMOTE_IDLE)
		{
			changeRemoteMode (REMOTE_GROUND_MODE);		// So it starts sending DI data
			Set_RC_S_mode(RC_S_MODE_C_IDLE,1860);
		}

		RC_S_Send_status(RC_S_ERR_OK,'D');
		return;


	case 550:	// Reset over-current trip
		val = CAN_rcv_command_msg.Data[1];
		if(val == 0)
		{
			State_reset_overcurrent_trip();
		}
		else	// Allow overcurrent trip to be set on for testing...
		{
			overCurrentTrip = 1;
		}
		RC_S_Status(RC_S_ERR_OK);
		return;

	}
}
static bool Pos_debug = false;
void process_RC_Standard (char *cmd)
{
    // at this stage we already know the first 2 letters were RC, it has been converted to upper-case, and null terminated
	char c;
	int val;
	int rval;
	char field[16];
	char dataline[80];
	float angle;
	RC_S_error=0;

	DPRINTF("C:%s\r\n",cmd);

//	DPRINTF("%d:%s\r\n",TimeInTicks,cmd);
//	PRINTF("%d:%s\r\n",TimeInTicks,cmd);

	RC_S_bps = cmd;
	rval = RC_S_get_keyword();
	if(rval)
	{
		RC_S_Status(rval);
		return;
	}

	c = *RC_S_bps++;
	if(c == '=')		// Did we find an '='?
	{
		int dec = 0;
//		if((RC_S_keyval == RC_S_MODE_SETPOS) || RC_S_keyval == RC_S_MODE_REVERSE_POS) dec = 1;
		if((RC_S_keyval == RC_S_MODE_SETPOS) || RC_S_keyval == RC_S_MODE_REVERSE_POS) dec = 2;
		rval = RC_S_getval(dec);
		if(rval)
		{
			RC_S_Status(rval);
			return;
		}
	}
	else
	{
		if(c == 0)
		{
			RC_S_value = RC_S_NO_VAL;
		}
		else
		{
			RC_S_Status(RC_S_ERR_BAD_COMMAND);
			return;
		}
	}

	// OK, at this stage we have extracted the keyword and value and speed if present.

	if(Hub_pos_error)
	{
		if((ps.parms[BL_DIAGNOSTICS] & P_BL_RC_ALLOW) == 0)
		{
			RC_S_Status(RC_S_ERR_POSITION_ERROR);
			return;
		}
	}


	val = RC_S_value;

	if(RC_S_keyval == RC_S_MODE_SETPOS && val == 2000)
	{
		Pos_debug = true;
	}
	if(Pos_debug)
	{
		if(RC_S_keyval != RC_S_MODE_SETPOS)
		{
			mh_debug();
		}
		if(val != 2000)
		{
			if(val != RC_S_NO_VAL)
			{
				mh_debug();
			}

		}
	}

	switch(RC_S_keyval)
	{
	case RC_S_CODE:
		Remote_reply("OK\r\n");
		return;

	case RC_S_DATA:
		  // Send comma delimited line
		//          1         2         3         4         5         6
		//0123456789012345678901234567890123456789012345678901234567890
        //VERSION=nnnn,AC200CW=nnnn,RCTY=nnnn,RCDI=nnnn,RCFE=nnnn
		// Note: Could use print buffer if short of storage...

		sprintf(dataline,"VERSION=%d,AC200CW=%d,RCTY=%d,RCDI=%d,RCFE=%d,\r\n",
//				(int)pcbVersion() * 1000 + version,
				(int)pcbVersion() * PCB_VERSION_MULTIPLIER + VERSION,	// MHH:15/05/2024
				ps.parms[AC_CONTROL_WORD],
				ps.parms[REMOTE_COMMS_TYPE],
				ps.parms[RC_DATA_INTERVAL],
				ps.parms[RC_DATA_FIELDS]);
		Remote_reply(dataline);
		Set_RC_S_mode(RC_S_MODE_C_IDLE,1850);	// So FEATHER LED will indicate connected
		return;


	case RC_S_MODE_GETPOS:
        angle = AC210_SIG100_angle();
		sprintf(field,"%3.2f\r\n",angle);
		Remote_reply(field);
		return;

	case RC_S_MODE_SET_DATA_INTERVAL:
		if(val == RC_S_NO_VAL) val = 0;
		setTempParameter(RC_DATA_INTERVAL,val);
        RC_S_Status(RC_S_ERR_OK);
		return;

	case RC_S_MODE_SET_DATA_FIELDS:
		if(val == RC_S_NO_VAL) val = 0;
		setTempParameter(RC_DATA_FIELDS,val);
        RC_S_Status(RC_S_ERR_OK);
		return;

	case RC_S_MODE_REQUEST_ENC_VERSION:
		AC200Enc_version = ps.parms[BL_HUB_SOFTWARE_VERSION];
		sprintf(field,"%d.%0d\r\n",AC200Enc_version/100,AC200Enc_version%100);
		Remote_reply(field);
		return;

#ifdef MH_XXX
	case RC_S_MODE_REQUEST_ENC_FLAGS:
		sprintf(field,"ENCFLAGS=%d\r\n",BL_flags);
		Remote_reply(field);
		return;
#endif

	case RC_S_MODE_RESET_OVERCURRENT_TRIP:
		if(val == RC_S_NO_VAL) val = 0;
		if(val == 0)
		{
			State_reset_overcurrent_trip();
		}
		else	// Allow overcurrent trip to be set on for testing...
		{
			overCurrentTrip = 1;
		}
		RC_S_Status(RC_S_ERR_OK);
		return;
	}

	if(operatingMode () != HOLD)
	{
		RC_S_Status(RC_S_ERR_NOT_HOLD_MODE);
		return;
	}


	RC_S_ms_display();
	switch(RC_S_keyval)
	{
	case RC_S_MODE_DATA:		// Maybe a RC_D command?

		if(val == 0) RC_S_send_DI_data = false;
		if(val == 1) RC_S_send_DI_data = true;

		//RC_S_MODE_IDLE=1,
//		RC_S_MODE_C_IDLE
		if(remoteMode() == REMOTE_IDLE)
		{
			changeRemoteMode (REMOTE_GROUND_MODE);		// So it starts sending DI data
			Set_RC_S_mode(RC_S_MODE_C_IDLE,1860);
		}

		RC_S_Send_status(RC_S_ERR_OK,'D');
		return;

	case RC_S_MODE_SETPOS:
#ifdef AC210_PORT
    	if(Sig100_connected == false)
    	{
			RC_S_Status(RC_S_ERR_SIG100_NOT_CONNECTED);
    		return;
    	}

		if(val == RC_S_NO_VAL)	// Was there an '='?
		{
//	        angle = AC210_SIG60_angle();	// no, then just display value
	        angle = AC210_SIG100_angle();
			sprintf(field,"%3.2f\r\n",angle);
			Remote_reply(field);
			return;
		}

		RC_S_target_angle = ((float)val)/100;	// Two implied decimal places
        RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
   		Set_RC_S_mode(RC_S_MODE_SETPOS,1900);
        controlDirection = C_IDLE;		// So we know it is first time
        RC_S_terminate_msg = true;
        RC_S_Status(RC_S_ERR_OK);		// Possibly should check stops first?

#endif
        return;

	case RC_S_MODE_REVERSE_POS:
#ifdef AC210_PORT
    	if(Sig100_connected == false)
    	{
			RC_S_Status(RC_S_ERR_SIG100_NOT_CONNECTED);
    		return;
    	}

    	if(val == RC_S_NO_VAL)	// Was there an '='?
		{
	        angle = AC210_SIG100_angle();	// no, then just display value
			sprintf(field,"%3.2f\r\n",angle);
			Remote_reply(field);
			return;
		}
        RC_S_param_direct();
        if(Control_type != CT_REVERSE)
        {
			RC_S_Status(RC_S_ERR_NO_REVERSE);
			return;
        }
    	if(Reverse_RPM_too_high())
    	{
			RC_S_Status(RC_S_ERR_RPM_TOO_HIGH);
    		return;
    	}
        RC_S_target_angle = ((float)val)/100;	// Two implied decimal place
// Note: May want to check against fine and coarse stop values.
        RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
//        controlDirection = C_FINER;
   		Set_RC_S_mode(RC_S_MODE_REVERSE_POS,2000);
//        RC_S_mode = RC_S_MODE_REVERSE_POS;
        controlDirection = C_IDLE;		// So we know it is first time
        RC_S_terminate_msg = true;
        RC_S_Status(RC_S_ERR_OK);		// Possibly should check stops first?
#endif

		return;

	case RC_S_MODE_SETSPEED:		// Maybe a RC_SETRPM command?
		if(val)		// Check valid val here, in case we need for next command
		{
	        if ( (val > getParameter(MAX_ENGINE_SPEED)) ||
	             (val  < getParameter(MIN_ENGINE_SPEED)) )
	        {
	    		RC_S_Status(RC_S_ERR_BAD_VALUE);
	    		return;
	        }
		}
    	RC_S_param_setspeed(val);		// If we restart we want to get out of reverse and then setspeed.
		if(RC_S_Check_In_Reverse())
		{
            remoteStartReverseReverse();
            RC_S_change_mode (REMOTE_REVERSE_REVERSE);
            RC_S_next_mode = RC_S_MODE_SETSPEED;
            RC_S_next_val  = val;
       		Set_RC_S_mode(RC_S_MODE_REVERSE_COARSE,2100);
//            RC_S_mode = RC_S_MODE_REVERSE_COARSE;	// Try using this mode, may need a special one
    		RC_S_Status(RC_S_ERR_OK);
            return;
		}
		RC_S_Setspeed(val);
		RC_S_Status(RC_S_ERR_OK);
		return;

	case RC_S_MODE_FINE:		// Fine?
        RC_S_param_direct();
		RC_S_count = RC_S_fnval(val);
        RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
        controlDirection = C_FINER;
   		Set_RC_S_mode(RC_S_MODE_FINE,2200);
//        RC_S_mode = RC_S_MODE_FINE;
        RC_S_Status(RC_S_ERR_OK);		// Possibly should check stops first?
        return;

	case RC_S_MODE_COARSE:		// Coarse?
        RC_S_param_direct();


#ifdef MH_RC_CHECK_IN_REVERSE   	// MHH:30/12/2023. This logic causing reverse_feather when testing brushless. Is it needed?
        // Also, even when switching to manual, was ignoring manual fine and coarse
		if(RC_S_Check_In_Reverse())	// Similar logic to SetSpeed
		{
            remoteStartReverseReverse();
            RC_S_change_mode (REMOTE_REVERSE_REVERSE);
            RC_S_next_mode = RC_S_MODE_COARSE;
            RC_S_next_val  = RC_S_fnval(val);
       		Set_RC_S_mode(RC_S_MODE_REVERSE_COARSE,2300);
//            RC_S_mode = RC_S_MODE_REVERSE_COARSE;	// Try using this mode, may need a special one
    		RC_S_Status(RC_S_ERR_OK);
            return;
		}
#endif
        controlReversing = false;
        RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
        controlDirection = C_COARSER;		// Not sure if needed
   		Set_RC_S_mode(RC_S_MODE_COARSE,2400);
//        RC_S_mode = RC_S_MODE_COARSE;
		RC_S_count = RC_S_fnval(val);
		RC_S_Status(RC_S_ERR_OK);
        return;

	case RC_S_MODE_FEATHER:
		RC_S_feather();
        return;


	case RC_S_MODE_UNFEATHER:
		RC_S_unfeather();
		return;

	case RC_S_MODE_REVERSE_FINE:		// Reverse

		RC_S_param_direct();
		if(Control_type != CT_REVERSE)
		{
			RC_S_Status(RC_S_ERR_NO_REVERSE);
			return;
		}
		if(Reverse_RPM_too_high())
		{
			RC_S_Status(RC_S_ERR_RPM_TOO_HIGH);
			return;
		}

		controlReversing = true;
		remoteStartReverse();

		controlDirection = C_FINER;		// Not sure if needed
		RC_S_change_mode (REMOTE_GROUND_MODE);		// for now

		Set_RC_S_mode(RC_S_MODE_REVERSE_FINE,2410);	// MHH:10/06/2025
//		RC_S_mode = RC_S_MODE_REVERSE_FINE;
		RC_S_count = RC_S_fnval(val);
		RC_S_Status(RC_S_ERR_OK);
		return;

	case RC_S_MODE_REVERSE_COARSE:	// Out of Reverse
		if(RC_S_Check_In_Reverse() == false)	// Similar logic to SetSpeed
		{
			RC_S_Status(RC_S_ERR_NOT_IN_REVERSE);
			return;
		}
		RC_S_param_direct();
        if(val == RC_S_GETVAL_S)		// Going to a stop?
        {
            remoteStartReverseReverse();
            RC_S_change_mode (REMOTE_REVERSE_REVERSE);
            RC_S_next_mode = RC_S_MODE_C_IDLE;
            RC_S_next_val  = 0;
       		Set_RC_S_mode(RC_S_MODE_REVERSE_COARSE,2500);
//            RC_S_mode = RC_S_MODE_REVERSE_COARSE;	// Try using this mode, may need a special one
    		RC_S_Status(RC_S_ERR_OK);
            return;
        }

		RC_S_count = RC_S_fnval(val);
        RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
        controlDirection = C_COARSER;

        if (controlReversing)
        {
            controlReversing = false;			// as per existing logic??
            remoteStopReverse();
        }

   		Set_RC_S_mode(RC_S_MODE_COARSE,2600);
//        RC_S_mode = RC_S_MODE_COARSE;
		RC_S_Status(RC_S_ERR_OK);
        return;

	case RC_S_STOP:
		RC_S_stop_move();
        return;

	}
	RC_S_Status(RC_S_ERR_BAD_COMMAND);
}
//------------------------------------------------------------------------
bool remoteXoar2SetAngle(void)	// MHH:08/04/2024
{
	if(Xoar_instruction != X2_INSTRUCTION_CODE_SET_ANGLE) return false;
	if(Sig100_connected == false) return false;
	if(RemoteCommsType != REMOTE_COMMS_XOAR2) return false;

	float target_angle = (float)Xoar2_set_angle;
	target_angle /= 10;
	RC_S_target_angle = target_angle;
	RC_S_change_mode (REMOTE_GROUND_MODE);		// for now
	Set_RC_S_mode(RC_S_MODE_SETPOS,1901);
	controlDirection = C_IDLE;		// So we know it is first time
//	return false;
	return true;	// MHH:15/05/2024. Need to test...
}
void mh_debug()
{

}



