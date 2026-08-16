/* ------------------------------------------------------------
Title:          Comms.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   22/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Serial port Handler and comms protocol

Changes:

------------------------------------------------------------ */
// CODE
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

#include "comms.h"

#include "ac210_sig100.h"
#include "ac210_global.h"
#include "ac210_logix.h"

#include "global.h"
#include "param.h"
#include "control.h"
#include "manifold.h"
//#if REMOTE_VERSION
#include "remote.h"
//#endif
#include "device.h"

// These 2 needed for AT_SYS function. Perhaps should move debug code to seperate file?

#include "sstate.h"
#include "digital.h"
#include "diags.h"
#include "log.h"
#include "analog.h"	// for A_MOTOR_CURRENT
#include "xoar.h"

/*
  Coms uses serial port 1. Default is 19200 Baud, 8 data 1 stop  no parity
  Handshaking is not used
*/

#define CODE 1L

//247627837L

#define TERMINATOR 0x0A

#define BAUD_RATE 19200

#ifndef AC210_PORT
#define BAUD_DIVISOR ((CLOCK / BAUD_RATE / 16)-1)
#define SER_1_TX_INT 19
#define SER_1_RX_INT 20
#define INT_PRIORITY 7
#endif

// serial port buffers. Tx buffer is circular
// rx buffer is message based, and pointers are reset to 0
// after receipt of a <CR><LF> combination
#define TX_BUF_SIZE 1024
#define RX_BUF_SIZE 50


char txBuf[TX_BUF_SIZE];
volatile BYTE txInPtr;
volatile BYTE txOutPtr;

char rxBuf[RX_BUF_SIZE];
volatile BYTE rxCount;

char rxBuf2[RX_BUF_SIZE];	// MHH:21/11/2023. For Remote control on port 1.
volatile BYTE rxCount2;

BYTE rx_Count;			// Working buffers so we can enable interrupts while processing command
char rx_Buf[RX_BUF_SIZE];

bool Sig100_hub_link;

// Flag for incoming messages
volatile BYTE commandAvailable;
BYTE codeEntered;
BYTE RemoteCommsType;	// defined in comms.c
WORD AT_Remote;			// remote comms only times out when this value zero.
						// Useful for testing when you do not want timeout for commands

WORD AT_flag_rpm_stop;	// Used by AC210_rpm.c to disable RPM logic
// local prototypes
void processCommand (void);
void send (const char  far *buf, BYTE numChar);

BYTE txBufferSpaceAvailable (void);
BYTE txBufferEmpty (void);
void txBufferPut (char ch);
char txBufferGet (void);
void txChar (char ch);

//-------------------------------------------------------------------------------------------------------------
// May work, need to test
#ifndef AC210_PORT
#pragma INTERRUPT rxInt
void rxInt (void);

#pragma INTERRUPT txInt
void txInt (void);

WORD p_BufferLen;
char p_Buffer[100];
//#define PRINTF() {p_BufferLen = sprintf(p_Buffer,__VA_ARGS__);txDebug(p_Buffer);}
//#define PRINTF(...) sprintf(p_Buffer,va_list)
//#define eprintf(args...) fprintf (stderr, args)

void PRINTF(const char _far *format,...)
//char _far *s;
//const char _far *format;
//va_list ap;
{
  	va_list ap;
  	va_start(ap,format);

	sprintf(p_Buffer,format,ap);
	txDebug(p_Buffer);
}
//extern void PRINTF(const char _far *format,...);
//PRINTF("I2C_read_page: result = %d\r\n",result);
12345

#endif
//---------------------------------------------------------------------------------------------------------------
#define AUTO_INDEX_MAX		8
#define AUTO_TAKEOFF		1
#define AUTO_CLIMB			2
#define AUTO_CRUISE			3
#define AUTO_HOLD			4
#define AUTO_FEATHER		5		// Will implement this last, if at all
#define AUTO_MANUAL			6		// Automatic in the sense it is not being controlled by switches
#define AUTO_LEDS			7		// MHH:27/01/2019
#define AUTO_UNUSED			8		// MHH:13/07/2023

#define AUTO_FINE			61
#define AUTO_COARSE			62

//#define AUTO_OK				0
//#define AUTO_ERROR			1

WORD Auto_flag;		// Used by LED routines
WORD Auto_throttle=AUTO_THROTTLE_UNUSED;
// Updated from Aux port routines in AC210_comms.c, can set with ATX_THROTTLE command too
// In fact, should show in AC200User!!

WORD Auto_index;
static WORD Auto_value;
static BYTE Auto_direction;		// Possibly for Fine/Coarse
static BYTE Auto_throttle_index_override;	// When too long in take-off
static WORD Auto_takeoff_secs;
static ULONG Auto_time_in_secs;
extern long setSpeed;
static const char *Auto_desc[]=
{
		"OFF",		// 0
		"Take-off",	// 1
		"Climb",	// 2
		"Cruise",	// 3
		"Hold",		// 4
		"Feather",	// 5
		"Manual",	// 6
		"LEDS",		// 7	// MHH:27/01/2019
};
//---------------------------------------------------------------------------------------------------------------
const uint8_t Map_throttle_ix[]=
{
		SL_T0,				// Single lever throttle setrpm
		SL_T1,
		SL_T2,
		SL_T3,
		SL_T4,
		SL_T5,
		SL_T6,
		SL_T7,
		SL_T8,
		SL_T9,

};
#define AUTO_CHANGE_VAL		1		// To avoid throttle sitting on (say) 95% and going back and forward between take-off and climb
static WORD Last_auto_throttle;
static WORD Last_cruise_setrpm;
WORD AC210_get_cruise_val(void)		// MHH:01/05/2023
{
	ParamIndex pix;
	WORD pval;
	WORD setrpm = 0;


	pix = (ParamIndex)Map_throttle_ix[0];	// In case have not set up cruise throttle settings in param.
	pval = ps.parms[pix];
	if(pval == 0)
	{
		setrpm = ps.parms[SP_CRUISE];
		return setrpm;
	}

	// This logic in case Throttle is close to boundary to avoid it going up and down. Only lags when throttle is decreasing.

	int throttle_ix = Auto_throttle/10;
	int last_throttle_ix = Last_auto_throttle/10;

	if(throttle_ix < last_throttle_ix && Auto_throttle >= Last_auto_throttle-AUTO_CHANGE_VAL)
	{
		return Last_cruise_setrpm;
	}
	int tix;

	// Scan back from throttle_ix until we find a non-zero entry.
	for(tix=throttle_ix;tix>=0;tix--)
	{
		pix = (ParamIndex)Map_throttle_ix[tix];
		pval = ps.parms[pix];
		if(pval != 0)
		{
			setrpm = pval;
			break;
		}
	}
	if(setrpm == 0)	// Could check it is within valid range too.
	{
		setrpm = ps.parms[SP_CRUISE];
	}
	Last_auto_throttle = Auto_throttle;
	Last_cruise_setrpm = setrpm;
	return setrpm;
}
//--------------------------------------------------------------------------------------------
bool Auto_LEDS_flag;
#define MH_AUTO
#ifdef MH_AUTO
BYTE Auto_set_mode(WORD value)
{
	WORD idx;
	WORD nval;
	const char *desc;
	char tmp[40];

	// Note: Value of zero turns auto mode off
	if(value == 0)
	{
		Auto_LEDS_flag=false;
		Auto_index = 0;
		Auto_value = 0;		// To be sure
		txDebug("*Auto:OFF\r\n");
		return ATX_OK;
	}
	if(value > AUTO_INDEX_MAX)
	{
		return ATX_ERROR;
	}
	if(value == AUTO_LEDS)	// Send Xoar LEDS value to comport if it changes.
	{
		Auto_LEDS_flag = true;
		return ATX_OK;
	}
	if(value <= AUTO_HOLD)
	{
		idx = value -1;	// ParameterIndex = Auto_index-1, as we deliberately have same format
		if(value == AUTO_HOLD) idx = SP_HOLD;	// Except for hold
		nval = getParameter (idx);

		if((nval >= getParameter(MIN_ENGINE_SPEED)) && (nval <= getParameter(MAX_ENGINE_SPEED)))
		{
			Auto_index = value;
			Auto_value = nval;
			desc = Auto_desc[Auto_index];
			sprintf(tmp,"*Auto:%s, setspeed:%d, throttle:%d\r\n",desc,Auto_value,Auto_throttle);
			txDebug(tmp);
			return ATX_OK;
		}
		return ATX_ERROR;
	}

	if(value == AUTO_MANUAL)
	{
		Auto_index = value;
		Auto_value = 0;		// May use this as a fine/coarse timer
		txDebug("*Auto:Manual\r\n");
		return ATX_OK;
	}

	// Need to code for MANUAL and FEATHER

	return ATX_ERROR;
}
#endif
//---------------------------------------------------------------------------------------------------------------
static WORD Auto_check_value(WORD value,WORD icode)
{
// Check value against max and minimum parameter values.

	WORD nval = value;

	switch(icode)
	{
	case AUTO_HOLD:
		if((nval >= getParameter(MIN_ENGINE_SPEED)) && (nval <= getParameter(MAX_ENGINE_SPEED)))
		{
			Auto_value = nval;
			return ATX_OK;
		}
		break;

	case AUTO_FINE:
		if(Auto_index == AUTO_MANUAL)
		{
			Auto_direction = 'F';
			Auto_value = value*5;		// convert from tenths to 50ths
			return ATX_OK;
		}
		break;

	case AUTO_COARSE:
		if(Auto_index == AUTO_MANUAL)
		{
			Auto_direction = 'C';
			Auto_value = value*5;		// convert from tenths to 50ths
			return ATX_OK;
		}
		break;

	}
	return ATX_ERROR;
}
//---------------------------------------------------------------------------------------------------------------
int Auto_setrpm;
void Auto_throttle_control(void)
{
	BYTE throttle_index;
	WORD takeoff_pct;
	WORD climb_pct;

	if(Auto_throttle >  200) return;		// In case unused

	takeoff_pct = getParameter(TC_TAKEOFF_PCT);
	climb_pct   = getParameter(TC_CLIMB_PCT);

	throttle_index = 0;

	if(Auto_throttle >= takeoff_pct)
	{
		Auto_setrpm = getParameter(SP_TAKEOFF);
		throttle_index = AUTO_TAKEOFF;
	}
	else
	{
		if(Auto_throttle >= climb_pct)
		{	// This logic to handle throttle boundaries so does not "hunt"
			if(Auto_index == AUTO_TAKEOFF && Auto_throttle >= takeoff_pct - AUTO_CHANGE_VAL)
			{
				throttle_index = AUTO_TAKEOFF;
			}
			else
			{
				throttle_index = AUTO_CLIMB;
			}
		}
		else
		{
			if(Auto_index == AUTO_CLIMB && Auto_throttle >= climb_pct - AUTO_CHANGE_VAL)
			{
				Auto_setrpm = getParameter(SP_CLIMB);
				throttle_index = AUTO_CLIMB;
			}
			else
			{
				Auto_setrpm = AC210_get_cruise_val();
				throttle_index = AUTO_CRUISE;
			}
		}
	}


#ifdef MH_XXX
	// e.g if in take-off already, and throttle boundary is 95%, then throttle will need to drop to 93% to go to climb.

	bool change_allowed = true;
	throttle_index = Auto_index;
	switch(Auto_index)
	{
	case AUTO_TAKEOFF:
		if(Auto_throttle >= takeoff_pct - AUTO_CHANGE_VAL) change_allowed = false;
		break;

	case AUTO_CLIMB:
		if(Auto_throttle < takeoff_pct)
		{
			if(Auto_throttle >= climb_pct - AUTO_CHANGE_VAL) change_allowed = false;
		}
		break;
	}
	if(change_allowed)
	{
		if(Auto_throttle >= takeoff_pct)
		{
			throttle_index = AUTO_TAKEOFF;			// Take-off
		}
		else if(Auto_throttle >= climb_pct)
		{
			throttle_index = AUTO_CLIMB;			// Climb
		}else
			throttle_index = AUTO_CRUISE;			// Cruise
	}
#endif
	BYTE set_index = throttle_index;
	if(Auto_throttle_index_override == AUTO_CLIMB)
	{
		if(throttle_index == AUTO_TAKEOFF)
		{
			set_index = AUTO_CLIMB;				// Too long in take-off, set climb
		}
		else
		{
			Auto_throttle_index_override = 0;	// Not in take-off, so can reset
		}
	}
	if(set_index == AUTO_TAKEOFF)				// Take-off mode?
	{
		if(Auto_time_in_secs != TimeInSeconds)
		{
			Auto_time_in_secs = TimeInSeconds;
			if(++Auto_takeoff_secs > getParameter(TC_TAKEOFF_MAX_SECS))
			{
				Auto_throttle_index_override = AUTO_CLIMB;	// Climb
				set_index = AUTO_CLIMB;
//				Auto_set_mode(set_index);
			}
		}
	}
	if(set_index != AUTO_TAKEOFF)
	{
		Auto_takeoff_secs = 0;
	}
	Auto_index = set_index;
	if(Auto_index == AUTO_TAKEOFF)
	{
		Auto_setrpm = getParameter(SP_TAKEOFF);
	}
	else
	{
		Auto_takeoff_secs = 0;
		if(Auto_index == AUTO_CLIMB)
		{
			Auto_setrpm = getParameter(SP_CLIMB);
		}
	}
	Auto_value = Auto_setrpm;

#ifdef MH__XXX
	if(set_index != Auto_index)	// Change?
	{
		Auto_time_in_secs = TimeInSeconds;
		Auto_takeoff_secs = 0;
		Auto_throttle_index_override = 0;
		Auto_set_mode(set_index);
	}
#endif
}
//---------------------------------------------------------------------------------------------------------------
#ifdef MH_XXX
#define TC_PORT_NONE		0
#define TC_PORT_CAN			1
#define TC_PORT_SERIAL		2
#endif
//#define AUX_SERIAL_UL			1
// Not all CAN and Serial to both operate together, as may want to communicate on both ports
//void AC210_check_aux_ports(void)
void AC210_get_throttle_from_engine(void)
{

#ifdef MH_XXX	// MHH: 01/05/2023. Moved to case:TC_PORT_SERIAL below
	if(getParameter(AUX_PORTS_SERIAL_CTL))
	{
		AC210_aux2_check_line();
	}
#endif
// Note: Currently CAN is handled by ISR, no need to poll at this point.
// We may decide to extract other information at some point, even if it does not relate to throttle control.

	WORD tc_port = getParameter(TC_PORT);
	WORD real_throttle_pct=0;
	switch(tc_port)
	{
	case TC_PORT_NONE:
		return;

	case TC_PORT_CAN:
		real_throttle_pct = AC210_can_get_throttle();
		break;

	case TC_PORT_SERIAL:
		AC210_aux2_check_line();
		real_throttle_pct = AC210_aux2_get_throttle();
		break;

	default:
//		PRINTF("AC210_check_aux_ports:Unknown tc_port\r\n");
		return;
	}


	if(real_throttle_pct >= AUTO_THROTTLE_TIMEOUT)	// Handle timeout
	{
		Auto_throttle = AUTO_THROTTLE_TIMEOUT;
		if(operatingMode() == HOLD)
		{
			Auto_flag = AUTO_THROTTLE_TIMEOUT;		// So we get an LED warning
		}
		return;
	}

/*
 *  MHH:30/01/2024. On UL, maximum throttle vale is 89% and idle throttle is 30%.
 *
 *  For Rotax 912, valid range for throttle is between 0 and 100.
 *  For Rotax 916, valid range for throttle is between 0 and 115.
 *
 */
	WORD max_100_pct = getParameter(TC_MAX_100_PCT);
	WORD min_0_pct = getParameter(TC_MIN_0_PCT);
	if(real_throttle_pct > max_100_pct) real_throttle_pct = max_100_pct;	// Ensure in valid range
	if(real_throttle_pct < min_0_pct) real_throttle_pct = min_0_pct;

#ifdef MH_XXX	// MHH:01/05/2025
	if(tc_port == TC_PORT_SERIAL)	// MHH:30/04/2025
	{	// Use this logic to map 30% to zero % and 89% to 100%
		WORD real_pct_range = max_100_pct - min_0_pct;		// If UL then = 89 - 30 = 59%
		WORD adjusted_throttle_pct = ((real_throttle_pct - min_0_pct) * 100 + real_pct_range/2) / real_pct_range;
		if(adjusted_throttle_pct > 100) adjusted_throttle_pct = 100;
		Auto_throttle = adjusted_throttle_pct;
	}
	else		// Rotax. Don't adjust percentage
	{
		Auto_throttle = real_throttle_pct;
	}
#else
	Auto_throttle = real_throttle_pct;
#endif
}
//---------------------------------------------------------------------------------------------------------------
extern WORD currentControl;
extern ControlState cState;
//WORD Auto_throttle_mode=1;
void Auto_cycle_check()
{
//	static BYTE save_xoar_status;
	Auto_flag = 0;

	WORD tc_port = getParameter(TC_PORT);
	if(tc_port != 1 && tc_port != 2) return;

	AC210_get_throttle_from_engine();	// MHH:13/07/2023. Want to display throttle in AC200User, even if not using

//#ifdef MH_CHECK_HOLD
//	if(operatingMode() != HOLD && ps.parms[AUX_PORTS_SERIAL_CTL] != 3)	// MHH:13/07/2023. For testing in diagnostics mode
	if(operatingMode() != HOLD)
	{
		Auto_flag = 0;		// check ports may change flag (shouldn't)
//		Auto_index = 0;		// 22/04/2017: Just after Drop-box release
		Auto_index = AUTO_UNUSED;		// MHH:13/07/2023.
		Auto_LEDS_flag=false;
		return;
	}
//#endif
//	AC210_get_throttle_from_engine();


#ifdef MH_XXX
	if(Auto_LEDS_flag)	// MHH:27/01/2019
	{
		if(XoarStatus != save_xoar_status)
		{
			save_xoar_status = XoarStatus;
			PRINTF("*LEDS=%d\r\n",(int)XoarStatus)
		}
	}
#endif

    if(Auto_flag == AUTO_THROTTLE_TIMEOUT)
    {
    	if(TimeInSeconds == 0)
    	{
//            setSetSpeed((long)getParameter (SP_HOLD) * RPMFACTOR);
            setSetSpeed(getParameter (SP_CRUISE));
    	}
    	Auto_index = 0;
    	return;
    }
#ifdef MH_XXX
    WORD tc_port = getParameter(TC_PORT);
    if(tc_port > 0 && tc_port <= 2)		// Serial or CAN?
	{
		Auto_throttle_control();
	}
#endif

	Auto_throttle_control();

	if(Auto_index <= AUTO_HOLD)
	{
//		setSpeed = (long)Auto_value * 10;
//		setSetSpeed((long)Auto_value * RPMFACTOR);
		setSetSpeed(Auto_value);
		Auto_flag = Auto_index;
		return;
	}
#ifdef MH_AUTO_MANUAL	// MHH:02/05/2023. Not using at this stage
// Need to consider what to do about Manual and Feather here

	if(Auto_index == AUTO_MANUAL)
	{
		Auto_flag = Auto_index;
//		setSpeed = 0;
//		setSetSpeed(0);		// MHH:28/01/2019
		if(Auto_value == 0)
		{
			Auto_direction = 0;
			Set_cState(C_IDLE);
            currentControl = 0;
            return;
		}
		if(Auto_direction == 'F')
		{
// Probably don't want to do this if at a stop.
			Set_cState(C_FINER);
//            currentControl = 255;
            currentControl = 	getParameter (MC_MAX_PWM);	//MHH:15/08/2018
            Auto_value--;		// decrement timer
            return;
		}
		if(Auto_direction == 'C')
		{
// Probably don't want to do this if at a stop.
			Set_cState(C_COARSER);
//            currentControl = 255;
            currentControl = 	getParameter (MC_MAX_PWM);	//MHH:15/08/2018
            Auto_value--;		// decrement timer
            return;
		}
	}
#endif
}
//---------------------------------------------------------------------------------------------------------------
BYTE Auto_check_manual(void)		// Called from control.c
{
	return (Auto_index == AUTO_MANUAL);
}
//---------------------------------------------------------------------------------------------------------------
/*
 *
 * typedef enum
{
    MANUAL  = 0,
    FEATHER,	// 1
    HOLD,		// 2
    CRUISE,		// 3
    CLIMB,		// 4
    TAKEOFF,	// 5
    MAP			// 6
} OpMode;
 *
 */

#ifdef MH_MAP_OPMODE
static BYTE Auto_map_opmode[]=
{
		0,			// 0.dummy entry
		5,			// 1.take_off
		4,			// 2.climb
		3,			// 3.cruise
		2,			// 4.hold
		1,			// 5.feather
		0			// 6.manual

};
#endif
OpMode Auto_operating_mode()
{
	OpMode op_mode = operatingMode();
#ifdef MH_AUTO_OP_MODE
	if(op_mode != HOLD)
	{
		return op_mode;
	}
	if(Auto_index <1 || Auto_index > AUTO_INDEX_MAX)
	{
		return op_mode;
	}
	op_mode = 6 - Auto_index;		// See map above
#endif
	return op_mode;
}
//---------------------------------------------------------------------------------------------

XOAR_SND_PKT_t X_SND_pkt;
XOAR_RCV_PKT_t X_RCV_pkt;

XOAR2_SND_PKT_t X2_SND_pkt;
XOAR2_RCV_PKT_t X2_RCV_pkt;

SERIAL_PKT_t S_Pkt;
BYTE XoarSendFlag;
BYTE Xoar2_error;

char DebugString[128];
//#endif
//--------------------------------------------------------------------------------------------------
// Initialisation
void initXoarError(void)
{

}
void initXoar(void)
{
//	int Xoar_pkt_size;
//#if XOAR_VERSION > 0

#if MH_XOAR_DEBUG > 0
	debug |= DEBUG_XOAR;				// Just for debugging initially
//	MH_Debug("initComms:");		// Comes before Reset[0], leave out for now
#endif
//    RemoteCommsType = REMOTE_COMMS_XOAR;	// Eventually will control from parameter file
    RemoteCommsType = getParameter(REMOTE_COMMS_TYPE);
    switch(RemoteCommsType)
    {
    default:
    	return;
    case REMOTE_COMMS_XOAR:
        S_Pkt.Len        = 10;
        break;
    case REMOTE_COMMS_XOAR2:
        S_Pkt.Len = sizeof(XOAR2_SND_PKT_t);
        if(sizeof(XOAR2_SND_PKT_t) != 12)
        	initXoarError();
//        Xoar_pkt_size = sizeof(XOAR2_RCV_PKT_t);
        if(sizeof(XOAR2_RCV_PKT_t) != 16)
        	initXoarError();
        break;
    }
    S_Pkt.Pkt_Head_0 = XOAR_HEAD_0;
	XoarCheckManual();
#ifdef MH_OLD_XOAR
    S_Pkt.Len        = 10;
	if(RemoteCommsType == REMOTE_COMMS_XOAR)
	{
		XoarCheckManual();
	}
#endif
}
//--------------------------------------------------------------------------------------------------
void initComms (void)
{
    commandAvailable = 0;
    codeEntered = 0;

    txInPtr = txOutPtr = 0;
    rxCount = 0;
}
//--------------------------------------------------------------------------------------------------
// output a debug string
void txDebug (const char far *mess)
{
    send (mess, strlen(mess));
    if(ac2_test_flag == AC2_TEST_ON)
    {
        send("\r\n",2);
    }
}
//--------------------------------------------------------------------------------------------------
// output a debug string
// Ignore AC2_TEST flag
void txDebug2(const char far *mess)
{
    send (mess, strlen(mess));
}
//--------------------------------------------------------------------------------------------------
#ifdef MH_XXX		// May use if find enough examples
void txDebugNL (const char far *mess)
{
    send (mess, strlen(mess));
    send("\r\n",2);
}
#endif

void txDebugVal(char far *mess,WORD w)
{
	char buf[20];
	txDebug(mess);
	sprintf(buf,":%d, %04xh\r\n",w,w);
	txDebug(buf);
}
//------------------------------------------------------------------------------------------------
WORD TraceVal=0;
WORD Trace_display=8;
#ifdef AC210_PORT
#define TRACE_PORT		1
#else
#define TRACE_PORT 0
#endif

BYTE TracePort = TRACE_PORT;			// Set default for hardware, but can change in debug for AC210
WORD TraceLevel;
char TraceDots[]=".................";
//WORD TraceMax = sizeof(TraceDots);
WORD TraceMax = 10;	// This MCU has limited stack!!
//-------------------------------------------------
static void Trace_puts(char far *mess)
{
	Board_UARTPutSTR(mess);
}
//-------------------------------------------------
static void Trace_send(char far *mess,BYTE len)
{
	send(mess,len);
}
//-------------------------------------------------
void Trace_check(void)
{
#ifdef AC210_PORT
	if(TraceVal == 0)
	{
		AC210_uart3_wait();
//		wait_ms(200);	// allow print buffer to be output
		return;
	}
#endif
}
//-------------------------------------------------
BYTE Trace(char far *mess)
{
//	Trace_check();
	if(TraceVal == 0)
	{
		return 0;
	}

	if(TraceLevel+1 >= TraceMax)
	{
		Trace_puts("Trace:TraceMax exceeded\r\n");
#ifdef AC210_PORT
		AC210_uart3_wait();
#endif
		return 0;
	}
	TraceLevel++;
	if(TraceLevel < Trace_display)
	{
		Trace_send(TraceDots,TraceLevel);
		Trace_send("[",1);
		Trace_puts(mess);
		Trace_puts("]\r\n");
	}
	return TraceLevel-1;
}
//-------------------------------------------------
void TraceReturn(BYTE t,char far *mess)
{
//	Trace_check();
	if(TraceVal == 0) return;

	if(TraceLevel < Trace_display)
	{
		Trace_send(TraceDots,TraceLevel);
		if(strlen(mess) == 0)
		{
			Trace_puts("[return]\r\n");;
		}
		else
		{
			Trace_puts("[return:");
			Trace_puts(mess);
			Trace_puts("]\r\n");
		}
	}
	if(TraceLevel == 0)
	{
		Trace_puts("TraceReturn:TraceLevel=0\r\n");
		return;
	}
	TraceLevel--;
	if(TraceLevel != t)	// to help catch any returns without TraceReturn()
	{
		Trace_puts("TraceReturn:TraceLevel != t\r\n");
#ifdef AC210_PORT
		AC210_uart3_wait();
#endif
		return;
	}
}
//-------------------------------------------------
void TraceMsg(char far *mess)
{
	if(TraceVal == 0) return;
	txDebug("[msg:");
	txDebug(mess);
	txDebug("]\r\n");
}
//---------------------------------------------
WORD BrakeDebug_val;
void BrakeDebug(char far *mess)
{
	if(BrakeDebug_val == 0) return;
	txDebug(mess);
}

void BrakeDebugVal(char far *mess,WORD w)
{
	if(BrakeDebug_val == 0) return;
	txDebugVal(mess,w);
}


// do the transmit thing for a whole buffer
// fill up the TX buffer really, although it
// does start the Tx ISR off if required
void send (const char far *buf, BYTE numChar)
{
	p_send(buf,numChar);
}

//-----------------------------------------------------------------------------------------------
void rxDisableInt(void)
{
	p_rxDisableInt();
}
//-----------------------------------------------------------------------------------------------
static void ResetReceiver(void);
#ifdef AC2_TEST
BYTE AC2_TEST_get_command(void)		// AC2_TEST
{
	BYTE rv;

#ifdef MH_XXX	// MHH:12/08/2026

			char ch = (char)b;
			rxProcessChar(ch);
	if(rxCount == 0) return 0;

	rv = rxBuf[0];
	// Probably only need to do this if CommandAvail
	ResetReceiver();
#else
	int b;
	b = PC_getc();
	if(b < 0) return 0;

	rv = (BYTE) b;
#endif
	return rv;
}
#endif
//-----------------------------------------------------------------------------------------------
bool Xoar_mode(void)
{
//	return(S_Pkt.Mode != SERIAL_MODE_ASCII);
	return(S_Pkt.Mode == SERIAL_MODE_XOAR);	// MHH:08/03/2025
}
//-----------------------------------------------------------------------------------------------
void AC200_SIG100_putchar(char ch);
const char Sig100_escape[8]="!@#Stop!";
uint8_t Sig100_escape_cnt;
bool Hublink_check_escape(unsigned char ch)
{
	if(ch == Sig100_escape[Sig100_escape_cnt])
	{
		if(Sig100_escape_cnt >= 7)
		{
			Sig100_hub_link = false;
			Sig100_escape_cnt = 0;
			PC_puts("OK\r\n");
			return true;
		}
		else
		{
			Sig100_escape_cnt++;
		}
	}
	else
	{
		Sig100_escape_cnt = 0;
	}
	return false;
}
//---------------------------------------------------------------------------------------------------------
void rxProcessChar(BYTE ch)
{
    // a character has arrived

    rxBuf[rxCount] = (char)ch;

	if(S_Pkt.Mode == SERIAL_MODE_ASCII) {

// This is where XOAR mode is triggered.
//		if((RemoteCommsType == REMOTE_COMMS_XOAR) || (RemoteCommsType == REMOTE_COMMS_XOAR2))
		if(ch)
		{
			if(ch == S_Pkt.Pkt_Head_0) {	// Pkt_Head_0 == 0 if not XOAR or XOAR2
				rxBuf[0] = ch;
				rxCount  = 1;
				S_Pkt.Mode = SERIAL_MODE_XOAR;
				S_Pkt.Pkt_TimeoutVal = XOAR_PACKET_TIMEOUT;	// Decremented every cycle
				S_Pkt.Mode_TimeoutVal = 150;	// 150 cycles = 3 secs
				XoarSendFlag = 1;
				return;
			}
		}
        if (rxCount < RX_BUF_SIZE-1)
        {
            rxCount++;
        }
        if (ch == TERMINATOR)
        {
        	rxDisableInt();
            // set flag
            commandAvailable++;
        }

        return;
    }

// If we drop through we are NOT in ASCII mode
	if(S_Pkt.Mode == SERIAL_MODE_CAN)
	{
		if(rxCount == 0)
		{	// If first byte must be head_0
			if(ch != S_Pkt.Pkt_Head_0) return;
			S_Pkt.Len = 5;
			S_Pkt.Pkt_TimeoutVal = XOAR_PACKET_TIMEOUT;	// Decremented every cycle
			S_Pkt.Mode_TimeoutVal = 150;	// 150 cycles = 3 secs
			rxCount++;
		}
		else
		{
			if(rxCount == 4)		// fifth byte ?
			{
				S_Pkt.Len = 5 + ch;		// Then calculate real length
			}
			if(++rxCount >= S_Pkt.Len)
			{

			   	commandAvailable++;	 // set flag
			}
		}
	}
	else
	{
		if(rxCount == 0) {	// If first byte must be head_0
			if(ch != S_Pkt.Pkt_Head_0) {
				return;
			}
			S_Pkt.Pkt_TimeoutVal = XOAR_PACKET_TIMEOUT;	// Decremented every cycle
			S_Pkt.Mode_TimeoutVal = 150;	// 150 cycles = 3 secs
		}
		if(++rxCount >= S_Pkt.Len) {
	    	rxDisableInt();
	           // set flag
	       	commandAvailable++;
		}
	}
}
//-----------------------------------------------------------------------------------------------
// Instead of modifying standard ISR routine, extra command from the ring buffer
void Get_data_from_RB(void)
{
	int b;
	for(;;)
	{
		b = PC_getc();
		if(b < 0) return;
		char ch = (char)b;
		rxProcessChar(ch);
		if(commandAvailable)	// MHH:07/02/2025
		{
			return;
		}
	}
}
//-----------------------------------------------------------------------------------------------
//#if XOAR_VERSION > 0
// Rx ISR handler
// Buffers chars until the terminator <lf> is received
// then sets a flag for the mainline call to a processing routine

//----------------------------------------------------------------------------------------
BYTE XoarChecksum(BYTE *pPkt,int len)
{
    BYTE *p_pkt;
    int i;
    int tot=0;
    p_pkt = pPkt;
    len--;
    for(i=0;i<len;i++) tot += *p_pkt++;
    return (BYTE)-tot;
}
//---------------------------------------------------------------------------------------------
void Xoar2_set_error(BYTE ecode)
{
	Xoar2_error = ecode;
}
//----------------------------------------------------------------
void XoarProcessCommand(void)
{
	BYTE c1,c2;
	WORD rpm;
	int i;

	Xoar_ticks_left = 1;		// Force immediate reply

#if MH_XOAR_DEBUG > 3
	MH_Debug("XoarProcessCommand:");
#endif
//	XoarDumpPkt(rxBuf);
// Note: Could possible send back error = 18 ??
// Eg, Could set error to 18, but if a subsequent pkt is OK then it would clear it?
// We don't know how often they are sending pkts.

	if(rxBuf[0] != XOAR_HEAD_0){
#ifdef MH_XOAR_PRINT
		printf("XoarProcessCommand:Head[0] %02x invalid\n\r",rxBuf[0]);
#endif
		return;
	}
	if(rxBuf[1] != XOAR_HEAD_1){
#ifdef MH_XOAR_PRINT
		printf("XoarProcessCommand:Head[1] %02x invalid\n\r",rxBuf[1]);
#endif
		return;
	}
#define X_SEND_PKT_LEN		10
	if(RemoteCommsType == REMOTE_COMMS_XOAR)
	{
		c1 = XoarChecksum((BYTE *)rxBuf,X_SEND_PKT_LEN);
		c2 = rxBuf[X_SEND_PKT_LEN-1];
		if(c1 != c2) {
	#ifdef MH_XOAR_PRINT
			printf("XoarProcessCommand:Bad checksum, %02x, %02x\n\r",c1,c2);
	#endif
			return;
		}
		memcpy((char *)&X_SND_pkt,(char *)rxBuf,X_SEND_PKT_LEN);
		rpm = X_SND_pkt.rpm_command[0];
		for(i=1;i<3;i++) {
			if(rpm != X_SND_pkt.rpm_command[i]) {
	#ifdef MH_XOAR_PRINT
				printf("XoarProcess:Command: rpm[0] != rpm[%d]\n\r",i);
	#endif
				return;
			}
		}
	// If we get this far, then looks OK.
	// Possibly should check against maximum and minimum allowed RPM, could check existing remote logic to see how that works.

	#if MH_XOAR_DEBUG > 3
		MH_Debug("XoarProcessCommand:Pkt OK");
	#endif
		XoarProcessCommand2(X2_INSTRUCTION_CODE_AUTO_SPEED,rpm);	// In remote.c
		return;
	}

// If we drop through then XOAR2 command.
	Xoar2_error = 0;

	c1 = XoarChecksum((BYTE *)rxBuf,X2_SEND_PKT_LEN);
	c2 = rxBuf[X2_SEND_PKT_LEN-1];
	if(c1 != c2) {
#ifdef MH_XOAR_PRINT
		printf("XoarProcessCommand:Bad checksum, %02x, %02x\n\r",c1,c2);
#endif
		Xoar2_set_error(XOAR_ERROR_BAD_CHECKSUM);
		return;
	}
	memcpy((char *)&X2_SND_pkt,(char *)rxBuf,X2_SEND_PKT_LEN);
	BYTE instruction_code   = X2_SND_pkt.instruction_code[0];
	WORD instruction_parameter = X2_SND_pkt.instruction_parameter[0];
	for(i=1;i<3;i++) {
		if(instruction_code != X2_SND_pkt.instruction_code[i]) {
#ifdef MH_XOAR_PRINT
			printf("XoarProcess:Command: rpm[0] != rpm[%d]\n\r",i);
#endif
			Xoar2_set_error(XOAR_ERROR_CODES_NOT_SAME);
			return;
		}
		if(instruction_parameter != X2_SND_pkt.instruction_parameter[i]) {
#ifdef MH_XOAR_PRINT
			printf("XoarProcess:Command: rpm[0] != rpm[%d]\n\r",i);
#endif
			Xoar2_set_error(XOAR_ERROR_PARAMETERS_NOT_SAME);
			return;
		}
	}
// If we get this far, then looks OK.
// Possibly should check against maximum and minimum allowed RPM, could check existing remote logic to see how that works.

#if MH_XOAR_DEBUG > 3
	MH_Debug("XoarProcessCommand:Pkt OK");
#endif


	XoarProcessCommand2(instruction_code,instruction_parameter);	// In remote.c
}
//---------------------------------------------------------------
//extern long setSpeed;
extern WORD remoteSetSpeed;
extern WORD remoteControlCycle_count;
WORD Xoar_prev_status;
void XoarSendPacket(void)
{
#if MH_XOAR_DEBUG > 3
	static WORD cnt;
    sprintf(DebugString,"XoarSendPacket:%d, setSpeed:%ld, remoteSetSpeed:%u,remoteState:%d,remoteCycleCount:%u",cnt++,setSpeed,remoteSetSpeed,iremoteState(),remoteControlCycle_count);
    MH_Debug(DebugString);
#endif

#define X_RCV_PKT_LEN	10
#define X2_RCV_PKT_LEN	16

    if(Xoar_prev_status == 22)
    {
    	if(XoarStatus != 22)
    	{
    		mh_debug();
    	}
    }
    Xoar_prev_status = XoarStatus;

    if(RemoteCommsType == REMOTE_COMMS_XOAR)
    {
        X_RCV_pkt.header[0] = XOAR_HEAD_0;
        X_RCV_pkt.header[1] = XOAR_HEAD_1;
//        X_RCV_pkt.rpm_command = currentSetSpeed()/RPMFACTOR;
//        X_RCV_pkt.rpm_current = currentActualSpeed()/RPMFACTOR;
        X_RCV_pkt.rpm_command = currentSetSpeed();
        X_RCV_pkt.rpm_current = currentActualSpeed();
        X_RCV_pkt.status_gear = XoarGetSwitches();
#define MH_MAP_SLIPRING_STATUS
#ifdef MH_MAP_SLIPRING_STATUS	// MHH:22/12/2021 Because Xoar have not updated their software to cope with slipring errors 23 and 24
        BYTE status = XoarStatus;
        if(status == 23) status = 1;
        if(status == 24) status = 2;
        X_RCV_pkt.status_controller = status;
#else
        X_RCV_pkt.status_controller = XoarStatus;
#endif
        X_RCV_pkt.reserved = 0;		// probably redundant
        X_RCV_pkt.checksum = XoarChecksum((BYTE *)&X_RCV_pkt,X_RCV_PKT_LEN);
        send ((char *)&X_RCV_pkt, X_RCV_PKT_LEN);
        return;
    }

    if(Xoar2_error == 0)
    {
    	if(overCurrentTrip)
    	{
    		Xoar2_error = XOAR_ERROR_CURRENT_OVERLOAD;
    	}
    }

    X2_RCV_pkt.header[0] = XOAR_HEAD_0;
    X2_RCV_pkt.header[1] = XOAR_HEAD_1;
//    X2_RCV_pkt.rpm_command = currentSetSpeed()/RPMFACTOR;
//    X2_RCV_pkt.rpm_current = currentActualSpeed()/RPMFACTOR;
    X2_RCV_pkt.rpm_command = currentSetSpeed();
    X2_RCV_pkt.rpm_current = currentActualSpeed();
    X2_RCV_pkt.status_gear = XoarGetSwitches();
#ifdef MH_MAP_SLIPRING_STATUS	// MHH:22/12/2021 Because Xoar have not updated their software to cope with slipring errors 23 and 24
    BYTE status = XoarStatus;
    if(status == 23) status = 1;
    if(status == 24) status = 2;
    X2_RCV_pkt.status_controller = status;
#else
    X2_RCV_pkt.status_controller = XoarStatus;
#endif
//    X2_RCV_pkt.status_controller = XoarStatus;
    X2_RCV_pkt.voltage =  scaledValue (A_SUPPLY)*10;
    X2_RCV_pkt.current = ((scaledValue (A_MOTOR_CURRENT)+50)/100);	// In tenths of an amp
    X2_RCV_pkt.angle = 0;
    if(Sig100_connected) // MHH:08/04/2024
    {
        float angle = AC210_SIG100_angle();
    	angle *= 10;	// Get ready to convert to integer with 1 implied decimal place
    	angle += 0.5;
    	X2_RCV_pkt.angle  = (int16_t) angle;
	}
   	X2_RCV_pkt.last_error = Xoar2_error;
    X2_RCV_pkt.spare = 0;
    X2_RCV_pkt.checksum = XoarChecksum((BYTE *)&X2_RCV_pkt,X2_RCV_PKT_LEN);
    send ((char *)&X2_RCV_pkt, X2_RCV_PKT_LEN);
}
//----------------------------------------------------------------------------------------------
// The idea is that if we have not received remainder of a packet in 3 cycles (60 ms)
// then reset counter as it has been lost.
void XoarCheckPacketTimeout(void)
{
	if(S_Pkt.Mode == SERIAL_MODE_ASCII) return;

	if(S_Pkt.Pkt_TimeoutVal) {
		S_Pkt.Pkt_TimeoutVal--;
		if(S_Pkt.Pkt_TimeoutVal == 0) {
			rxCount = 0;
		}
	}

// Check for mode timeout

#ifdef XOAR_MODE_TIMEOUT
	if(S_Pkt.Mode_TimeoutVal) {
		S_Pkt.Mode_TimeoutVal--;
		if(S_Pkt.Mode_TimeoutVal == 0) {
			rxCount = 0;
			S_Pkt.Mode = SERIAL_MODE_ASCII;
		}
	}
#endif

}
//----------------------------------------------------------------------------------------------
void XoarCheckManual(void)
{
	BYTE switches,ln,rn;

	if(XoarSendFlag) return;
	if(RemoteCommsType != REMOTE_COMMS_XOAR)
		if(RemoteCommsType != REMOTE_COMMS_XOAR2)
			return;

	switches = XoarGetSwitches();			// digital.c
	ln = (switches >> 4);
	rn = (switches & 0xf);	// right nybble

	if(ln == 2 && rn == 5) return;	// manual + feather? MHH:15/01/2026 Or could be reverse?

//	no.

	XoarSendFlag = 1;	// not manual+feather, then send packets once a second

	S_Pkt.Mode = SERIAL_MODE_XOAR;
	rxCount  = 0;
}
//#endif
//----------------------------------------------------------------------------------------------
#if MH_REMOTE_POS
void RemotePosCommand_Decode(char *cmd);
extern char RemotePos_rcv_command[];
int CheckRemotePosCommand(void)
{
    if (commandAvailable == 0) return FALSE;

    if (rxCount <= 3) return FALSE;


//	if(rxBuf[0] != 'P') return FALSE;
//	if(rxBuf[1] != ':') return FALSE;
	if(rxBuf[0] != AC200_H0) return FALSE;
	if(rxBuf[1] != AC200_H1) return FALSE;

    rxBuf[rxCount]=0;			// null terminate

	if(RemotePos_rcv_command[0])
	{
//		if(operatingMode() != MANUAL)	txDebug("CheckRemotePosCommand:Rcv buff not empty\r\n");
		txDebug("CheckRemotePosCommand:Rcv buff not empty\r\n");
	}
	strcpy(RemotePos_rcv_command,rxBuf);	// save
//	RemotePosCommand_Decode(rxBuf+6);		// Bypass header

//	TraceVal=1;
	RemotePosCommand_Decode(RemotePos_rcv_command+6);		// Bypass header
//	TraceVal=0;

    commandAvailable = 0;
    rxCount = 0;
            // re-enable receiver
    U1C1 |= 0x04;

	return TRUE;

}
#endif
//----------------------------------------------------------------------------------------------
#ifdef AC210_PORT
int PC_ReturnToContinue(void)
{
//  char str1[20];
	int c;

   PRINTF("\r\n<RETURN> to continue\r\n");	// AC200CLI needs CRLF at present
   AC210_uart0_wait();		// Wait for buffer to be output
   while(true)
   {
	   c = PC_getc();
	   if(c >= 0) break;
   }
#ifdef MH_123   // MHH:08/12/2023

   ResetReceiver();
   while(commandAvailable == 0);
   c = rxBuf[0];
   ResetReceiver();
#endif
   if(c == 'x' || c == 'X')
   {
	   c = 'X';
   }
   PC_puts("\r\n");
   wait_ms(100);	// wait for PC to output '>'
   return c;
}
#endif
//----------------------------------------------------------------------------------------------
static void ResetReceiver(void)
{
    commandAvailable = 0;
    rxCount = 0;
#ifdef AC210_PORT
    p_rxEnableInt();		// ac210_comms.c
#else
    // re-enable receiver
    U1C1 |= 0x04;
#endif
}
//----------------------------------------------------------------------------------------------
uint8_t commandAvailable2;
void rxProcessChar2(BYTE ch)
{
    // a character has arrived

    rxBuf2[rxCount2] = (char)ch;

    if (rxCount2 < RX_BUF_SIZE-1)
    {
        rxCount2++;
    }
    if (ch == TERMINATOR)
    {
        commandAvailable2++;
    }
}

//-----------------------------------------------------------------------------------------------
// Instead of modifying standard ISR routine, extra command from the ring buffer

void Format_rx_Buf(void)
{
   	rx_Buf[rx_Count]=0;
    // Null terminate, and convert to uppercase

	rx_Buf[rx_Count]=0;

	// replace any trailing spaces, \r\n with nulls MHH:16/11/17
	int len = rx_Count - 1;
	int cnt,b0;
	for(cnt=len;cnt>=0;cnt--)
	{
		b0 = rx_Buf[cnt];
		if(b0)
		{
			if(b0 == 13 || b0 == 10 || b0 == 32)
			{
				rx_Buf[cnt] = 0;
				rx_Count = cnt;
			}
			else
			{
				break;
			}
		}
	}
	for (cnt=0; cnt< rx_Count; cnt++)
    {
        rx_Buf[cnt] = toupper(rx_Buf[cnt]);
    }
}

void Get_data_from_RB2(void)
{
	int b;
	for(;;)
	{
		b = Aux2_getc();
		if(b < 0) return;
		char ch = (char)b;
		rxProcessChar2(ch);
		if(commandAvailable2) return;
	}
}

void Remote_reply(const char far *msg)
{
	if(AC210_remote_control_port == 1)
	{
		Aux2_puts2(msg);
	}
	else
	{
		txDebug(msg);
	}
}



void checkForCommand2(void)		// MHH:21/11/2023. Remote control on port 1?
{
	if(AC210_remote_control_port != 1) return;

	for(int i=0;i<10;i++)
	{
		commandAvailable2= 0;
	    Get_data_from_RB2();

	    if(commandAvailable2 == 0) return;

	    rx_Count = rxCount2;
	    memcpy(rx_Buf,rxBuf2,rxCount2);	// Copy to work buffer so we can enable interrupts
	    rxCount2 = 0;

	    Format_rx_Buf();


	    if(rx_Count < 3)
	    {
	        return;
	    }
	    char *p_rx = rx_Buf;
	    if(*p_rx++ != 'R') return;
		if(*p_rx++ != 'C') return;
		if(*p_rx != '_') return;

		 process_RC_Standard( &rx_Buf[2] );

	}
}

//----------------------------------------------------------------------------------------------
// command processor. Called every time through the
// main loop, and responds to message after it arrives


//--------------------------------------------------------------------------------------------
void Process_ascii_command(void)
{
   BYTE b0,b1;

   rx_Count = rxCount;
	memcpy(rx_Buf,rxBuf,rxCount);	// Copy to work buffer so we can enable interrupts

	ResetReceiver();		// Enable interrupts

    Format_rx_Buf();

#ifdef AC210_PORT
    Debug_ShowCommand(rx_Buf);
#endif


#if MH_REMOTE_POS
		if(CheckRemotePosCommand()) return;		// Note: Possible an older command will be overwritten. Thats OK.
#endif

    if(rx_Count < 2)
    {
//        ResetReceiver();
        txDebug("\r\n");
        return;
    }
    watchIt(WD_COMMS+120);

    b0 = rx_Buf[0];
    b1 = rx_Buf[1];

    switch(b0)
    {
    case 'A':
        // commands are based on the AT command set
        // at least it begins with AT, and is at least AT
        watchIt(WD_COMMS+130);

    	if(b1 == 'T')
    	{
//            DPRINTF("%s\r\n",rx_Buf);
    		processCommand ();
    	}
    	break;
//#define MH_PETER_DEVICE_LOGIC		// MHH:23/11/2020
#ifdef MH_PETER_DEVICE_LOGIC		// MHH:23/11/2020
    case 'D':
    	if(b1 == 'D')
    	{
            processDeviceData( rx_Buf);
            break;
    	}
    	if(b1 == 'V')
    	{
            // This is a remote device identifier
            // as it starts with DV
            processDeviceID( rx_Buf);
    	}
    	break;
#else
/*
    case '!':		// Not going to use std port now. MHH:24/11/2020

    	AC210_check_slave_command(rx_Buf);
    	break;
*/
#endif
    case 'R':
        watchIt(WD_COMMS+140);

    	RemoteCommsType = getParameter(REMOTE_COMMS_TYPE);
    	if(b1 == 'C')
    	{
    		if(RemoteCommsType == REMOTE_COMMS_LEGACY)
    		{
                // could well be a remote control command
                // as it starts with RC
            	    processRemoteCommand( &rx_Buf[2] );
    		}
    		if(AC210_remote_control_port == 0)	// MHH:21/11/2023
    		{
        		if(RemoteCommsType == REMOTE_COMMS_STANDARD)
        		{
                	    process_RC_Standard( &rx_Buf[2] );
        		}
    		}
    	}
		break;
    }
//    ResetReceiver();

}
//-----------------------------------------------------------------------------------------------------------
int CANProcessCommand(char *rxBuf,int rxCount);

void checkForCommand (void)
{
//    int cnt,len;

    watchIt(WD_COMMS+100);
//    return;		// MHH:29/05/2025 Debug!!!

    Get_data_from_RB();

    if (commandAvailable == 0) {
//#if XOAR_VERSION > 0
//		if((RemoteCommsType == REMOTE_COMMS_XOAR) || (RemoteCommsType == REMOTE_COMMS_XOAR2))

    	switch(RemoteCommsType)	// MHH:08/03/2025, so it will work for all binary modes
 		{
    	case REMOTE_COMMS_XOAR:
    	case REMOTE_COMMS_XOAR2:
    	case REMOTE_COMMS_CAN:
    		XoarCheckPacketTimeout();
    		break;
		}
//#endif
    	return;
    }
// If we drop through then a command is available

	if(S_Pkt.Mode == SERIAL_MODE_XOAR)
	{
		XoarProcessCommand();
		ResetReceiver();
		// and exit
		return;
	}
	if(S_Pkt.Mode == SERIAL_MODE_CAN)
	{
		for(;;)			// May be more than one command per Tick
		{
			ResetReceiver();
			int result = CANProcessCommand(rxBuf,rxCount);
			if(result != 0) return;		// error sending or end can_serial_mode from PC
		    Get_data_from_RB();
		    if (commandAvailable == 0)
		    {
		    	return;
		    }
		}
	}

    watchIt(WD_COMMS+110);
    for(int i=0;i<10;i++)
    {
        Process_ascii_command();
        Get_data_from_RB();

        if (commandAvailable == 0) return;
    }
}
//--------------------------------------------------------------------------------------------
// does what it says
void dumpAllParameters (void)
{
    WORD idx, value;
    int ivalue;
    char rs[50];

    // first the version
//    sprintf(rs, "VERSION=%d\r\n", ((int)pcbVersion() * 1000) + version);
    sprintf(rs, "VERSION=%d\r\n", ((int)pcbVersion() * PCB_VERSION_MULTIPLIER) + version);	// MHH:15/05/2024
    send(rs,strlen(rs));
    // then the parameters
    watchIt(WD_COMMS+20);
    for (idx=0; idx < MAX_P_INDEX; idx++)
    {
        value = getParameter(idx);
        sprintf(rs,"%s=%d\r\n",pName[idx],value);
        send(rs,strlen(rs));
    }
    watchIt(WD_COMMS+30);

    // Current sense offset and gain
    ivalue = getCurrentOffset();
    value = getCurrentGain();
    sprintf (rs,"COFF=%d\r\n", ivalue);
    send (rs, strlen(rs));
    sprintf (rs,"CGAN=%u\r\n", value);
    send (rs, strlen(rs));

    watchIt(WD_COMMS+40);

#ifdef AC210_PORT
    // And the fine and coarse stops

//    AC210_SIG60_send_stops();

#endif
    // and the lookup table values

/*    for (idx=0; idx < 256; idx++)
    {
        value = getTableValue (idx);
        sprintf (rs,"PILT%d=%d\r\n", idx,value);
        send (rs, strlen(rs));
    }
*/
    send("OK\r\n",4);
}

#ifdef MH_TABLE_LOGIC
void processTableUpdate ( BYTE query, BYTE value )
{
    // code to perform control table query and update
    // because it is outside the normal indexed parameter system
    // String is of format ATPILTnnn=xxx where nnn & xxx = 0-255
    // =xxx only present if query is zero, in which case value = xxx
    char far *wrk;
    char rs[15];
    BYTE tableValue;
    WORD indexValue = 0;

    if (!query)
    {
        wrk = strchr(rxBuf,'=');
        *wrk = 0;
    }
    // get the index value
    indexValue = atoi (&rxBuf[6]);
    // perform the function
    if (!query)
    {
        setTableValue ( (BYTE)indexValue, value);
        send ("OK\r\n",4);
    }
    else
    {
        tableValue = getTableValue ((BYTE)indexValue);
        sprintf(rs,"ATPILT%u=%u\r\n", indexValue,tableValue);
        send(rs,strlen(rs));
    }
}
#endif
//-------------------------------------------------------------------------
#define MH_ATRPM
#ifdef MH_ATRPM
WORD AT_RPMValue;
#ifdef AC210_PORT
//-----------------------------------------------------------------------
void Set_Prop_RPM_float(float mag_rpm)
{
	float usecs_per_rev = (60 * 1000000) / mag_rpm;	// Divide usecs in minute by revs in minute
	float usecs_per_int  = usecs_per_rev /2;				// Have to halve as cycle = time for half a rev.
	uint32_t usecs = (uint32_t)(usecs_per_int + 0.5);
	AC210_RIT_Set(usecs);
}
//-----------------------------------------------------------------------
WORD Set_prop_rpm;
void Set_Prop_RPM(WORD mag_rpm)
{
//	PRINTF("Set_Prop_RPM:%d\r\n",mag_rpm);	// !!! debug
	Set_Prop_RPM_float(mag_rpm);
	Set_prop_rpm = mag_rpm;
}
//-----------------------------------------------------------------------
BYTE Set_ATRPM(WORD rpm)
{

//	if(rpm < 1000 || rpm > 7000) return 1;	// MHH:07/02/2018

	if(rpm < 100)	rpm = 100;	// Avoid divide by 0
	AT_RPMValue = rpm;
	WORD ScalingFactorPercent = getParameter(SF_MAG_SPEED);	// Compensate for geared option (eg rotax)
//	WORD mag_rpm = (rpm * 100 + ScalingFactorPercent/2) / ScalingFactorPercent;
	float mag_rpm = (rpm * 100) / (float)ScalingFactorPercent;	// 07/02/2018
	Set_Prop_RPM_float(mag_rpm);
	return 0;
}
#else
#pragma INTERRUPT RPMTimer
void RPMTimer (void) {
    // test timer. Toggle digital input # 1
    P8 ^= 0x18;
}
//-----------------------------------------------------------------------
static BYTE ATRPM_initialised;
Init_ATRPM(void)
{
    // set up Digital Input # 1 & 2 as an output, and set up a timer to toggle it at
    // a rate of X so we can test the RPM input
    PD8 |= 0x18;

    // at 100 Hz == 6000 RPM
    // timer needs to trigger every 5 milliSeconds
    vector_table[22] = (unsigned long)RPMTimer;

    TA1MR = 0x80;     // /32, timer
    TA1 = 2499;		// Init at 6000 rpm
    TA1IC = 0x04;
    TABSR |= 0x02;
}
//-----------------------------------------------------------------------
void Set_Prop_RPM(WORD mag_rpm)
{
	if(ATRPM_initialised == FALSE)
	{
		ATRPM_initialised = TRUE;
		Init_ATRPM();
	}
	TA1 = (WORD)((15000000L/mag_rpm)-1);
}
//-----------------------------------------------------------------------
BYTE Set_ATRPM(WORD rpm)
{
	WORD ScalingFactorPercent;
	WORD mag_rpm;

	if(rpm < 1000 || rpm > 7000) return 1;

//	if(AT_RPMValue == 0) {	// First time?
//		Init_ATRPM();
//	}
	AT_RPMValue = rpm;
	ScalingFactorPercent = getParameter(SF_MAG_SPEED);	// Compensate for geared option (eg rotax)
	mag_rpm = (rpm * 100 + ScalingFactorPercent/2) / ScalingFactorPercent;

// Even if counting, should reload counter correctly next cycle
	Set_Prop_RPM(mag_rpm);
//	TA1 = (WORD)((15000000L/mag_rpm)-1);
	return 0;
}
#endif
#endif
//------------------------------------------------------------------------
typedef struct
{
	int e;
	char far *desc;
} XOAR_ERROR_TD;

static const XOAR_ERROR_TD XOAR_err[]=
{
		{0,"No error"},
		{1,"Pitch decreasing"},
		{2,"Pitch increasing"},
		{3,"Pitch increasing in feather"},
		{4,"No speed signal"},
		{5,"Fine pitch limit"},
		{6,"Coarse pitch limit"},
		{7,"Reverse/feather pitch limit"},
		{8,"Driving at fine pitch limit"},
		{9,"Driving at coarse pitch limit"},
		{10,"Driving at reverse/feather pitch limit"},
		{11,"Beta mode engage (step 1)"},
		{12,"Fine pitch limit override (beta option)"},
		{13,"Pitch decreasing in feather/reverse"},		// Note similar to 19??
		{14,"Over-current while pitch decreasing"},
		{15,"Over-current while pitch increasing"},
		{16,"Over-current while pitch increasing in feather"},
		{17,"Open circuit failure"},
		{18,"Controller software fault"},
		{19,"Pitch decreasing in reverse"},
		{20,"In reverse zone"},
		{21,"Unknown error"},
};


// Way to examine system variables
void AT_SYS(WORD value)
{
	WORD v;
	far char *desc,*desc2;

	desc2 = "";

	switch(value)
	{
		case 0:
			v = XoarStatus;
			desc = "XoarStatus";
			desc2 = XOAR_err[v].desc;
			break;

		case 1:
			v = BETA_VERSION;
			desc = "BETA_VERSION";
			break;

		case 2:
			v = systemState();
			desc = "systemState";
			break;

		case 3:
			v = operatingMode();
			desc = "operatingMode";
			if(v == MANUAL) 	desc2 = "[MANUAL]";
			if(v == FEATHER) 	desc2 = "[FEATHER]";
			if(v == HOLD) 		desc2 = "[HOLD]";
			if(v == CRUISE) 	desc2 = "[CRUISE]";
			if(v == CLIMB) 		desc2 = "[CLIMB]";
			if(v == TAKEOFF) 	desc2 = "[TAKEOFF]";
			break;

#ifdef MH_SIG60
		case 4:
			v = AC210_SIG60_input_errors();
			desc = "SIG60_input_errors";
			break;
#endif
		default:
			v = 0;
			desc = "Undefined";
			break;
	}
	sprintf(Ctl_pbuff,"%s%s = %d [%04x]\r\n",desc,desc2,v,v);
	txDebug(Ctl_pbuff);
	if(value == 2)		// SystemState?
	{
		if(v & S_RUN_FINE)txDebug("RUN_FINE set\r\n");
		if(v & S_RUN_COARSE)txDebug("RUN_COARSE set\r\n");
		if(v & S_RUN_REVERSE)txDebug("RUN_REVERSE/FEATHER set\r\n");
		if(v & S_STOP_FINE) txDebug("STOP_FINE set\r\n");
		if(v & S_STOP_COARSE) txDebug("STOP_COARSE set\r\n");
		if(v & S_STOP_REVERSE) txDebug("STOP_REVERSE/FEATHER set\r\n");
	}
}
//-------------------------------------------------------------------------
void AT_SYS_All(void)
{
	WORD v;
#ifdef AC210_PORT
    PRINTF("PCB version: 5%c\r\n",AC210_hardware_version);
#endif
	for(v=0;v<5;v++)
	{
		AT_SYS(v);
	}
}
//-------------------------------------------------------------------------
#define RX_KEYWORD_MAX		32
static int 	AT_KeywordLen;
static char AT_Keyword[RX_KEYWORD_MAX];
static void AT_KeywordSave(void)
{
	char c,*src,*dst;
	int i;
	AT_KeywordLen = 0;
	dst = AT_Keyword;
	src = rx_Buf + 2;

//	if(*src++ != 'A') return;	// we know it starts with AT
//	if(*src++ != 'T') return;
	c = 0;
	for(i=1;i<RX_KEYWORD_MAX-1;i++)
	{
		c = *src++;
		if(c < '0') c = 0;	// eg space
		if(c == '=') c = 0;
		*dst++ = c;
		if(c == 0) break;
	}
	if(c != 0) return;		// No trailing null within keyword max length - ignore
	if(i < 3) return;		// Length too short
	AT_KeywordLen = i;
}
//-------------------------------------------------------------------------
static bool AT_KeywordMatch(const char *string)
{
	int rv = strncmp(AT_Keyword,string,AT_KeywordLen);
	return (rv == 0);
}
//-------------------------------------------------------------------------
static bool rxBufMatch(const char *string)
{
	int slen = strlen(string);
	int rv = strncmp(rx_Buf,string,slen);
	return (rv ==0);
}
//-------------------------------------------------------------------------
#ifdef AC210_PORT
int AC210_Force_Hardfault(void);	// Test if watchdog handles this
void AC210_Flash_Wrap_Test(WORD val);
void Sig100_zero_enc_data(void);
extern uint8_t SIG100_display;
extern bool Hub_return_position;
extern uint8_t Sig100_init_status;
extern int Prop_rpm_1dec;
//-------------------------------------------------------------------------
static int ATX_command(BYTE query,WORD *p_value)
{
	WORD value = *p_value;
	// 0123
	// ATXx
	BYTE b3 =rx_Buf[3];
	uint16_t rv;

	switch(b3)
	{
	case 'A':
	    if (AT_KeywordMatch("XANGLE"))
	    {
	    	if(query)
		    {
		    	float angle = AC210_SIG100_angle();
		    	PRINTF("A=%3.2f\r\n",angle);
				return ATX_RETURN;
		    }
	    }
	    break;

	case 'B':
	    if (AT_KeywordMatch("XBOOT"))
	    {
			printf("Closing comport\r\n");
			AC210_SerialDeInit();
			wait_ms(100);
//			printf("Rebooting...\r\n");	// MHH:04/09/2023. Cannot printf if port is closed!
			DPRINTF("Rebooting...\r\n");
			wait_ms(100);
			AC210_reboot();
//			NVIC_SystemReset();			// Should not get passed here
	    }
	    if (AT_KeywordMatch("XBTEST"))
	    {
		    if(query) return ATX_ERROR;
	    	return AC210_board_test(value);
	    }
	    break;

	case 'C':
	    if (AT_KeywordMatch("XCALIBRATE"))
	    {
		    if(query) return ATX_ERROR;
		    switch(value)
		    {
		    case 1:
		    case 4:
				Hub_calibrate_pole_pairs = (uint8_t)value;
		    	break;

		    default:
				PRINTF("INVALID CODE\r\n");
				return ATX_RETURN;
		    }
#ifdef MH_FFF
			if(value != 2476)	// First 4 digits of password
			{
				PRINTF("INVALID CODE\r\n");
				return ATX_RETURN;
			}
#endif

			if(Prop_rpm_1dec != 0)	// MHH:18/12/2025
			{
				PRINTF("RPM not zero\r\n")
				return ATX_RETURN;
			}
			if(AC210_remote_control_board == false)
			{
				if(operatingMode () != MANUAL)
				{
					PRINTF("Must be in MANUAL mode\r\n")
					return ATX_RETURN;
				}
			}

			// Must put in a check for zero RPM, plus manual mode
			Hub_calibrate_state = CAL_START;
			Hub_calibrate_type = 0;
			Hub_calibrate_checkval = 12345;	// For safety
			return ATX_OK;
	    }
	    if (AT_KeywordMatch("XCANSERIAL"))
	    {
		    if(query) return ATX_ERROR;
		    if(value > 2)
		    {
		    	PRINTF("INVALID CODE\r\n");
		    	return ATX_RETURN;
		    }
		    CAN_mode = CAN_MODE_SERIAL;
		    if(value == 2)
		    {
		    	AC210_PC_ChangeBaud(115200);
//		    	PC_puts("012345678901234567890\r\n");
//		    	AC210_uart0_wait();	// This waits until output buffer is empty.
//		    	wait_ms(40);

		    }
		    S_Pkt.Mode = SERIAL_MODE_CAN;
		    S_Pkt.Pkt_Head_0 = 'C';
		    S_Pkt.Len        = 5;
			return ATX_OK;
	    }
	    break;

	case 'D':
	    if (AT_KeywordMatch("XDUPLICATE"))
	    {
		    txDebug("OK\r\n");
		    if(query) return ATX_ERROR;
		    AC210_ee_duplicate(value);
			return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XDVERIFY"))	// Data verify
	    {
		    txDebug("OK\r\n");
		    AC210_data_verify();		// MHH:31/12/2025
//		    AC210_logix_verify(2);
			return ATX_RETURN;
	    }
	    break;

	case 'E':
	    if (AT_KeywordMatch("XERASE"))
	    {
		    if(query) return ATX_ERROR;
		    if(AC210_logix_erase(value) == 0)
		    {
				return ATX_OK;
		    }
		    break;
	    }
	    if (AT_KeywordMatch("XERASESECTOR"))	// MHH:09/12/2023
	    {
		    if(query) return ATX_ERROR;
		    PRINTF("Erasing sector:%d\r\n",value);
		    AC210_ssp_flash_erase(value);
			return ATX_OK;
	    }
	    break;


	case 'F':
	    if (AT_KeywordMatch("XFIRST"))
	    {
		    if(query) return ATX_ERROR;
	    	AC210_range.first_run = value;
	        return ATX_OK;
	    }
	    if (AT_KeywordMatch("XFHEX"))
	    {
		    if(query) return ATX_ERROR;
	    	AC210_ssp_flash_hex(value);
	        return ATX_OK;
	    }
	    if (AT_KeywordMatch("XFLASH_WTEST"))	// MHH:09/12/2023
	    {
	    	if(query) return ATX_ERROR;
	    	AC210_ssp_write_test(value);
			return ATX_OK;
	    }
	    break;

	case 'H':
	    if (AT_KeywordMatch("XHIST"))
	    {
		    if(query) return ATX_ERROR;
		    txDebug("OK\r\n");
//			AC210_watchdog_active = false;
//		    wait_ms(20000);
		    AC210_PC_ChangeBaud(115200);
	    	AC210_eelog_history(value);
		    AC210_PC_ChangeBaud(19200);
			return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XHEX"))
	    {
		    if(query) return ATX_ERROR;
	    	eeprom_hex(value);
	        return ATX_OK;
	    }
	    if (AT_KeywordMatch("XHUBLINK"))
	    {
//		    DPRINTF("ATXHUBLINK\r\n");
	    	if(query) return ATX_ERROR;
			if(value != 2476)	// First 4 digits of password
			{
				PRINTF("INVALID CODE\r\n");
				return ATX_RETURN;
			}
			// Must put in a check for zero RPM.
			if(Sig100_connected == false)
			{
//				PRINTF(":A:ERR:Hub not connected\r\n");
//				DPRINTF("HUB_CONNECTED=FALSE\r\n");
				PRINTF("HUB_CONNECTED=FALSE\r\n");
//				Sig100_hub_link = true;	// MHH:06/12/2023. Should not have been set.
				Sig100_hub_link = true;	// MHH:09/06/2025. In case hub is in reboot state
				return ATX_RETURN;
			}
			Hublink_command_sent = false;
			PRINTF("HUB_CONNECTED=TRUE\r\n");
//			DPRINTF("HUB_CONNECTED=TRUE\r\n");
			Sig100_hub_link = true;
			return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XHUBINIT"))		// MHH:30/05/2023. Reset Hub initialisation so that AC200 will reload hub data
	    {
		    if(query) return ATX_ERROR;
			if(value != 2476)	// First 4 digits of password
			{
				PRINTF("INVALID CODE\r\n");
				return ATX_RETURN;
			}
			if(Sig100_connected == false)
			{
				PRINTF("HUB_CONNECTED=FALSE\r\n");
				return ATX_RETURN;
			}
			Sig100_init_status = 0;		// Will need to reload data from hub.
			return ATX_OK;
	    }
	    break;

	case 'I':
	    if (AT_KeywordMatch("XIBUILD"))
	    {
#define MH_NEED_CODE
#ifdef MH_NEED_CODE
	    	if(query) break;
	    	if(value == 2476)
	    	{
		    	txDebug("OK\r\n");
				AC210_logix_build();
				return ATX_RETURN;
	    	}
	    	if(value == 1234)	// MHH:09/08/2023
	    	{
		    	txDebug("OK\r\n");
				AC210_logctl_repair();
				return ATX_RETURN;

	    	}
			PRINTF(":A:ERR:Invalid code\r\n");
			return ATX_RETURN;
#endif
	    }
	    if (AT_KeywordMatch("XISHOW"))
	    {
		    txDebug("OK\r\n");
//		    if(query) return ATX_ERROR;
		    AC210_logix_show();
			return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XIVERIFY"))	// Index verify
	    {
		    txDebug("OK\r\n");
//		    AC210_logix_verify(1);
		    AC210_index_verify();	// MHH:31/12/2025
			return ATX_RETURN;
	    }
	    break;

	case 'L':
	    if (AT_KeywordMatch("XLAST"))
	    {
		    if(query) return ATX_ERROR;
	    	AC210_range.last_run = value;
		    txDebug("OK\r\n");
		    AC210_PC_ChangeBaud(115200);
	    	AC210_ee_load_range();
		    AC210_PC_ChangeBaud(19200);
			return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XLOAD"))
	    {
		    if(query) return ATX_ERROR;
		    txDebug("OK\r\n");
		    AC210_PC_ChangeBaud(115200);
	    	AC210_ee_load(value);
			AC210_watchdog_active = true;	// MHH:04/08/2023
		    AC210_PC_ChangeBaud(19200);
			return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XLOG"))
	    {
		    if(query) return ATX_ERROR;
	    	AC210_log_init();	// just in case
	    	Log_change_rate(value);
	        return ATX_OK;
	    }
	    if (AT_KeywordMatch("XLOGCTL"))
	    {
	    	txDebug("OK\r\n");
			AC210_display_logctl();
			return ATX_RETURN;
	    }
#ifdef MH_XXX	    // MHH:03/08/2023. Thinking about a new way of mapping run_number to position in ssp flash memory. See comments in AC210_display_logctl();
	    if (AT_KeywordMatch("XLOGHEXDUMP"))
	    {
	    	txDebug("OK\r\n");
			AC210_hexdump_logctl();
			return ATX_RETURN;
	    }
#endif
	    break;

	case 'N':
		if (AT_KeywordMatch("XNEWRUN"))
	    {
	    	Diags_new_run();
	        return ATX_OK;
	    }
		break;

	case 'P':
		if (AT_KeywordMatch("XPLOG"))
		{
			if(query) value = 0;
			AC210_plog_display(value);
			return ATX_RETURN;
		}
		if (AT_KeywordMatch("XPOSENC"))		// MHH:07/06/2026
		{
			if(query)
			{
				//				PRINTF("P=%d\r\n",Encoder_Pos);
			}
			else
			{
				rv = SIG100_set_position_encoder(value);
				PRINTF("T=%d\r\n",rv);
			}
			PRINTF_FLUSH;
			return ATX_RETURN;
		}
		if (AT_KeywordMatch("XPOSLGET"))
		{
			if(query)
			{
				PRINTF("P=%d\r\n",Encoder_Pos);
			}
			else
			{
				rv = SIG100_get_position_len(value*10);
				PRINTF("E=%d\r\n",rv);
			}
			PRINTF_FLUSH;
			return ATX_RETURN;
		}

		if (AT_KeywordMatch("XPOSLSET"))
		{
			if(query)
			{
//				PRINTF("P=%d\r\n",Encoder_Pos);
			}
			else
			{
				rv = SIG100_set_position_len(value*10);
				PRINTF("T=%d\r\n",rv);
			}
			PRINTF_FLUSH;
			return ATX_RETURN;
	    }
		if (AT_KeywordMatch("XPOSLSET2"))
	    {
			if(query)
			{
//				PRINTF("P=%d\r\n",Encoder_Pos);
			}
			else
			{
				rv = SIG100_set_position_len(value);
				PRINTF("T=%d\r\n",rv);
			}
			PRINTF_FLUSH;
			return ATX_RETURN;



	    }
		break;


	case 'R':
	    if (AT_KeywordMatch("XRESET"))
	    {
		    if(query) return ATX_ERROR;
	        eeprom_log_reset(value);
	        return ATX_OK;
	    }
	    if (AT_KeywordMatch("XRESTORE"))
	    {
	    	txDebug("OK\r\n");
	    	AC210_restore_flash();
	    	// May need to reboot? To Load param and diags rec?
	        return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XRVERIFY"))	// Run verify
	    {
		    txDebug("OK\r\n");
		    if(query) return ATX_ERROR;
			AC210_watchdog_active = false;				// MHH:04/08/2023
			AC210_ee_verify(value);
			AC210_watchdog_active = true;				// MHH:04/08/2023
			return ATX_RETURN;
	    }
	    break;

	case 'S':
	    if (AT_KeywordMatch("XSAVE"))
	    {
	    	txDebug("OK\r\n");
	    	AC210_save_flash();
	        return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XSERIAL"))
	    {
		    int v=-1;
		    if(query == false) v = value;
		    AC210_display_serial_ringbuffer_data(v);	// MHH:12/08/2023
	        return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("XSTOPRPM"))
	    {
		    if(query) return ATX_ERROR;
	    	AT_flag_rpm_stop = value;		// This allows setting on (1) or off (0)
	        return ATX_OK;
	    }
#ifdef MH_SIG60
	    if (AT_KeywordMatch("XSIG60"))
	    {
		    if(query) return ATX_ERROR;
		    SIG60_display = value;
	        return ATX_OK;
	    }
#endif
	    if (AT_KeywordMatch("XSIG100"))	// MHH:18/11/2022
	    {
	    	if(query) return ATX_ERROR;
	    	if(value == 0)
	    	{
	    		SIG100_display = 0;
	    		Hub_return_position = false;
	    	}
	    	if(value == 1) 	SIG100_display = value;
	    	if(value == 2) Hub_return_position = true;
	    	return ATX_OK;
	    }
	    break;

	case 'T':
	    if (AT_KeywordMatch("XTEST"))
	    {
	    	AC2_TEST_start();
            return ATX_RETURN;
	    }
		break;

	case 'V':
        if (AT_KeywordMatch("XVERBOSE"))
        {
            if (query)
            {
                *p_value = gVerbose;
            }
            else
            {
				gVerbose = value;
            }
            return ATX_OK;
        }
	    break;


	case 'W':
	    if (AT_KeywordMatch("XWRAP"))
	    {
		    if(query) return ATX_ERROR;
		    txDebug("OK\r\n");
	    	AC210_Flash_Wrap_Test(value);
			return ATX_RETURN;
	    }
	    break;

	case 'Z':	// MHH:23/07/2023
	    if (AT_KeywordMatch("XZERO_ENCDATA"))
	    {
		    if(query) return ATX_ERROR;
		    txDebug("OK\r\n");
		    Sig100_zero_enc_data();
			return ATX_RETURN;
	    }
	    break;


	case '_':
		if (AT_KeywordMatch("X_LEDS"))
	    {
		    txDebug("OK\r\n");
		    AC210_test_leds();
			return ATX_RETURN;
	    }
	    if (AT_KeywordMatch("X_THROTTLE"))
	    {
            if (query)
            {
                *p_value = Auto_throttle;
            }
            else
            {
            	Auto_throttle = value;
            	// A value outside valid range of 0 to 100 will disable
            	//            	Auto_throttle = MIN(100,Auto_throttle);
            }
            return ATX_OK;
	    }
	    break;
	}


#ifdef MH_XCHK
    if (AT_KeywordMatch("XCHK"))
    {
    	AC210_eelog_check(value);
        return ATX_OK;
    }
#endif
#ifdef MH_X_DISPLAY_DIAGS
    if (AT_KeywordMatch("XDIAG"))
    {
	    if(query) return ATX_ERROR;
    	AC210_ee_display_diags(value);
        return ATX_OK;
    }
#endif
	return ATX_ERROR;
}
#endif
//-------------------------------------------------------------------------
static void SendError(void)
{
    txDebug("ERROR\r\n");
}
//-------------------------------------------------------------------------
static void SendOK(void)
{
    txDebug("OK\r\n");
}
//-------------------------------------------------------------------------
static long Long_value;
static int Process_non_indexed_commands(BYTE query,WORD *p_value)
{
	BYTE b2;
    int  offsetValue = 0; // for current offset, must be signed
	char rs[50];
	WORD value = *p_value;

	b2 = rx_Buf[2];
	switch(b2)
	{
	case 'A':
        if (AT_KeywordMatch("ABORT"))
        {
            if (query) break;
        	Abort(WD_COMMS+value,"ATABORT");	// allows testing of Abort
        	return ATX_OK;
        }
	    if (AT_KeywordMatch("AC2TEST"))
	    {
	    	AC2_TEST_start();
            return ATX_RETURN;
	    }
        break;

	case 'B':
		if (AT_KeywordMatch( "BYE"))
		{
			// exit command. reset code flag
			codeEntered = 0;
//			query = 0;
			return ATX_OK;
		}
		break;

	case 'C':
        if (AT_KeywordMatch( "CGAN"))
        {
//        	currentSense = 2;
        	// current sense gain
        	if (query)
        	{
        		*p_value = getCurrentGain();
                // gain
                sprintf(rs, "ATCGAN=%u\r\n", value);
                txDebug(rs);
                return ATX_RETURN;
        	}
        	else
        	{
        		setCurrentGain (value);
                return ATX_OK;
        	}
        }
        if (AT_KeywordMatch( "COFF"))
        {
//            currentSense = 1;
            // Update current sense offset
            if (query)
            {
                offsetValue = getCurrentOffset();
                // offset
                sprintf(rs, "ATCOFF=%d\r\n", offsetValue);
                txDebug(rs);
                return ATX_RETURN;

            }
            else
            {
            	offsetValue = (int)Long_value;		// hopefully sign will be OK.
                setCurrentOffset (offsetValue);
                return ATX_OK;
            }
        }
        if (AT_KeywordMatch( "CTL"))
        {
            if (query)
            {
				value = getParameter(CTL_RPM_PER_TICK_X10);
				value /= 10;		// Because ATCTL is per tick
				*p_value = value;
				//                        value = Ctl_rpm_per_tick;
            }
            else
            {
				setParameter(CTL_RPM_PER_TICK_X10,value*10);
            }
            return ATX_OK;
        }
#ifdef MH_CTL_TUNE
        if (AT_KeywordMatch( "CTUNE"))
        {
            if (query)
            {
                *p_value = AT_Ctl_tune;
            }
            else
            {
            	AT_Ctl_tune = value;
				if(value)
				{
					Ctl_init_tune();
					Remote_init_tune();
				}
				else
				{
					Remote_end_tune();
				}
            }
			return ATX_OK;
        }
#endif
        break;


	case 'D':
		if (AT_KeywordMatch( "DEBUG"))
		{
			if (query)
			{
				*p_value = debug;
			}
			else
			{
				debug = value;
			}
			return ATX_OK;
		}
        if (AT_KeywordMatch("DIAGS"))
        {
            Stat_Rec.pc_prog = 2;
        	if (query) break;
        	// Because AC200Diagnostics is expecting OK first
        	if(value < 65530)
        	{
            	txDebug("OK\r\n");
            	Diags_AT(value);
            	return ATX_RETURN;
        	}
        	Diags_AT(value);
        	return ATX_OK;
        }
        if (AT_KeywordMatch("DIAGVER"))
        {
        	if (query) break;
        	Diags_version = value;
        	return ATX_OK;
        }
		break;

#ifdef MH_EEPROM_FAST_WRITE
	case 'E':
		if (AT_KeywordMatch("EEPROM"))
		{
			if (query) break;
			ParameterFastUpdate();	// in param.c
			Diags_param_update = true;
			// Should also update for current offset/gain, but leave for now
			return ATX_OK;
		}
		break;
#endif

#ifdef AC210_PORT
	case 'F':
		if (AT_KeywordMatch( "FLASH"))
		{
			if (query) break;
			if(value != 2476) break;	// First 4 digits of password

			AC210_flash();
			//                    		void AC210_SerialDeInit(void)
			DPRINTF("Closing comport\r\n");
			AC210_SerialDeInit();
//			wait_ms(100);
			DPRINTF("Rebooting...\r\n");
			wait_ms(100);
			AC210_reboot();
//			NVIC_SystemReset();			// Should not get passed here
		}
		break;
#endif

	case 'L':
#if CLI_LOG > 0
		if (AT_KeywordMatch( "LOG"))
		{
			if (query)
			{
				*p_value = CLI_log;
			}
			else
			{
				CLI_log = value;;
			}
        	return ATX_OK;
		}
#endif
		break;

	case 'M':
        if (AT_KeywordMatch("MONITOR"))
        {
            if (query) break;
            monitor = value;
			return ATX_OK;
        }
		break;

	case 'P':
        if (AT_KeywordMatch("PARM"))
        {
            // dump all parameters..
            if (!query) break;
            dumpAllParameters ();
            // and go straight back
            return ATX_RETURN;
        }
		if(AT_KeywordMatch("PILT"))	// Just ignore. This logic no longer used
		{
#ifdef MH_TABLE_LOGIC		// Ignore
			// Control Table lookup or query
			processTableUpdate (query, (BYTE)value);
#endif
			// return immediately
			return ATX_RETURN;
		}
#if AT_POKE > 0
		if (AT_KeywordMatch( "POKE"))
		{
			if (query)
			{
				*p_value = AT_POKE_val;
			}
			else
			{
				AT_POKE_val = value;;
			}
			return ATX_OK;
		}
#endif
		break;

	case 'R':
        if (AT_KeywordMatch( "REMOTE"))
        {
            if (query)
            {
                *p_value = AT_Remote;
            }
            else
            {
				AT_Remote = value;
            }
            return ATX_OK;
        }
		break;

	case 'S':
		if (AT_KeywordMatch("SERLOG"))
        {
            if (query)
            {
                *p_value = EEPROM_log;
            }
            else
            {
        		Log_serial_command(value);
//           		Log_change_rate(value);
            }
            return ATX_OK;
        }

        if (AT_KeywordMatch( "SYS"))
        {
            if (query)
            {
				AT_SYS_All();
            }
            else
            {
				AT_SYS(value);
            }
            return ATX_RETURN;
        }
        break;

	case 'T':
        if (AT_KeywordMatch( "TRACE"))
        {
            if (query)
            {
                *p_value = TraceVal;
            }
            else
            {
				TraceVal = value;;
            }
            return ATX_OK;
        }
        break;

	case 'U':
        if (AT_KeywordMatch( "UPMP"))
        {
            // Updated the manifold pressure points..

//        	rError = 0;
//            query = 0;
#if MAP_VERSION
            loadManifoldPressureLimits();
#endif
            break;		// Treat as error
        }
        break;

	case 'V':
		if (AT_KeywordMatch( "VERSION"))
		{
		    if(!query) return ATX_ERROR;

			//  sprintf(rs, "VERSION=%d.%02d\r\n", version / 100, version % 100);
		    int pcb_version = pcbVersion();
//			sprintf(rs, "VERSION=%ld\r\n", (pcb_version * 1000) + version);	// MHH:18/11/2016
/*
 *	Format
 *	PCBVersion + version
 *	999          99999
 *	572			 10101
 *	5 = Hardware number
 *	72 = 'H'
 *	10 = Version type (used to be used when separate compiles for BETA,REMOTE etc)
 *	101 = subversion. Previously limited to 2 bytes, with a maximum of 99.Now max 999.
 */
			sprintf(rs, "VERSION=%ld\r\n", (pcb_version * PCB_VERSION_MULTIPLIER) + version);	// MHH:15/05/2024
			send(rs,strlen(rs));
			return ATX_RETURN;
		}
		break;

	case 'W':
        if (AT_KeywordMatch("WATCHDOG"))
        {
            if (query == FALSE)
            {
            	switch(value)
            	{
            	case 1:
            		while(TRUE);	// infinite loop
            		break;			// not really

#ifdef AC210_PORT
            	case 2:
            		AC210_Force_Hardfault();	// Test HardFault_Handler
            		break;			// not really
#endif
            	}
            }
        }
        break;

	case 'Y':
        if (AT_KeywordMatch("YYMMDD"))
        {
            if (query) break;
            UpdateStatsDate(Long_value);
			return ATX_OK;
        }
        break;

	case '_':
        if (AT_KeywordMatch("_HMODE"))		// Hold mode
        {
            if (query)
            {
            	*p_value = Auto_index;
            	return ATX_OK;
            }
            return Auto_set_mode(value);
        }
		if (AT_KeywordMatch("_AUXRPM"))		// Generate RPM signal on AUX port for testing
		{
			if (query)
			{
				*p_value = AT_RPMValue;
				return ATX_OK;
			}
			else
			{
				return  Set_ATRPM(value);
				//                        TA1 = (WORD)((15000000L/value)-1);
			}
			break;
		}
		if (AT_KeywordMatch("_HOLD"))
        {
            if (query)
            {
                *p_value = Auto_value;
            	return ATX_OK;
            }
            return Auto_check_value(value,AUTO_HOLD);
        }
		if (AT_KeywordMatch("_COARSE"))
        {
            if (query) break;
            return Auto_check_value(value,AUTO_COARSE);
        }
		if (AT_KeywordMatch("_FINE"))
        {
            if (query) break;
            return Auto_check_value(value,AUTO_FINE);
        }
#ifdef MH_XXX
		if (AT_KeywordMatch("_NEWRUN"))
	    {
	    	Diags_new_run();
	        return ATX_OK;	//248
	    }
#endif
#ifdef MH_XXX		// MHH:08/04/2026
		if (AT_KeywordMatch("_REVERSE"))
		{
			if (query)
			{
				int rv = 0;
				if(Board_LED_Test(7)) rv=1;
				*p_value = rv;
				return ATX_OK;
			}
			if(value == 0)
			{
				AC210_RELAY_Set(false);
			}
			else
			{
				AC210_RELAY_Set(true);
			}
	        return ATX_OK;
		}
#endif
		if (AT_KeywordMatch("_RPM"))
		{
			if (query)
			{
				*p_value = currentActualSpeed();
				return ATX_OK;
			}
			break;
		}
		if (AT_KeywordMatch("_SETRTC"))
		{
            if (query) break;
            AC210_set_rtc(Long_value);
			return ATX_OK;
			break;
		}
		break;

#ifdef MH_XXX
		if (AT_KeywordMatch("_TAKEOFF"))
        {
            if (query)
            {
                *p_value = TraceVal;
            	return ATX_OK;
            }
            if(Auto_check_value(value,AUTO_TAKEOFF) == 0)
            {
            	return ATX_OK;
            }
            break;
        }
        if (AT_KeywordMatch("_CLIMB"))
        {
            if (query)
            {
                *p_value = TraceVal;
            	return ATX_OK;
            }
            if(Auto_check_value(value,AUTO_CLIMB) == 0)
            {
            	return ATX_OK;
            }
            break;
        }
        if (AT_KeywordMatch("_CRUISE"))
        {
        	if (query)
        	{
        		*p_value = TraceVal;
        		return ATX_OK;
        	}
        	if(Auto_check_value(value,AUTO_CRUISE) == 0)
        	{
        		return ATX_OK;
        	}
        	break;
        }
#endif
        break;

	}
	return ATX_ERROR;
}
//-------------------------------------------------------------------------
void processCommand (void)
{
    // determine what the command is, and what if any response there is to be
    BYTE rError = 0; // return flag to indicate ERROR
    BYTE query = 1;  // Command type is either a query or a set command
    int index = -1;  // the parameter index value
//    BYTE slen;
    char c;
    WORD value=0;    // either the incoming new value, or the return value
//    ULONG lvalue;
//    int  offsetValue = 0; // for current offset, must be signed
    char far *wrk;
    const char far *pname;
//    char far *wrksave=NULL;
//    BYTE currentSense;
    char rs[50];

    if(rx_Count <= 4)	// Allow AT command by itself
    {
    	SendOK();
    	return;
    }

//    currentSense = 0;
    // set query flag cos thats easy
    if ((wrk = strchr(rx_Buf, '='))!= 0)
    {
        // there is an = sign, so this is NOT a query
        query = 0;
        // point to first digit of new value
        wrk++;
//        wrksave = wrk;
        Long_value = atol(wrk);
        // get the new value
//        value = atoi (wrk);
        value = (WORD)Long_value;
//        offsetValue = (int)Long_value;	// offsetValue is signed
    }

    // check for a code number entry first

    if (rxBufMatch("ATCODE") && (wrk))
    {
    	Hub_return_position = false;
    	if(codeEntered == false)
    	{
#ifdef MH_XXX
    		if(value == 4)	// MHH:21/11/2023. Remote Control program?
    		{
    			if(AC210_remote_control_port == 0)	// MHH:21/11/2023)
    			{
    				value = CODE;
    			}
    		}
#endif
    		if (value == CODE)
            {
                codeEntered = 1;
                SendOK();
                Stat_Rec.pc_prog = 1;		// Default. Changed by ATDIAGS command
                return;
            }
    		if(value == 5)
    		{
    			if(ps.parms[REMOTE_COMMS_TYPE] == 5)
    			{
    				SendOK();
    				Stat_Rec.pc_prog = value;		// Another way of setting pc_prog
                    codeEntered = 1;
    				return;
    			}
    		}
    	}
    	else
    	{
    		if(value == 0)
    		{
    			codeEntered = 0;
    			SendOK();
    			return;
    		}
    		if(value >= 1 && value <= 3)
    		{
                SendOK();
                Stat_Rec.pc_prog = value;		// Another way of setting pc_prog
                return;
    		}
    	}
    }
    // the only response if the code is not entered is ERROR
    // with the exception of AT by itself

    if (!codeEntered)
    {
        SendError();
        return;
    }

    // a real command and the code is valid.. get the index
    wrk = &rx_Buf[2];
    c   = wrk[4];
    if(c == 0 || c == '=')	// Check it terminates with null or =
    {
    	c = wrk[0];
    	for (index = 0; index < MAX_P_INDEX; index++)
        {
        	pname = pName[index];
        	if(c == pname[0])
            {
            	if (strncmp(wrk, pname, 4) == 0)
                {
                    break;
                }
            }
    /*
        	if (!strncmp(wrk, &pName[index][0], 4))
            {
                break;
            }
    */
        }
        if (index == MAX_P_INDEX)
        {
            index = -1;
        }
    }
    else
    {
        index = -1;
    }

    // what to do now....
    // at this point, index will be -1, or a valid index
    // query will be 1 if this is a query command
    // or it will be 0 and value will be the new proposed value

    if(index > -1)
    {
    	if(query)
    	{
            // get the current value
            value = getParameter(index);
    	}
    	else
    	{
            // try to set the new value
            rError = setParameter (index, value);
    	}
    }
    else		// Index = -1, so not found in index
    {
    	AT_KeywordSave();
    	if(AT_KeywordLen <= 0)	// Should never happen
    	{
            SendError();
            return;
    	}

#ifdef AC210_PORT		// AC210 specific commands will start with "ATX"
    	if (rx_Buf[2] == 'X')
    	{
    		rError = ATX_command(query,&value);
    		if(rError == ATX_RETURN) return;
    	}
    	else
    	{
        	rError = Process_non_indexed_commands(query,&value);
        	if(rError == ATX_RETURN) return;
    	}
#else
    	rError = Process_non_indexed_commands(query,&value);
    	if(rError == ATX_RETURN) return;
#endif

    }

    // now decide what to send back......
    if (rError)
    {
        SendError();
        if ((index > -1) && (!query))
        {
            // get the current value
            value = getParameter(index);
            // and send it back
            sprintf(rs,"%s=%u\r\n", pName[index],value);
            send(rs,strlen(rs));
        }
    }
    else
    {
        if (!query)
        {
        	SendOK();
        }
        else
        {
        	// build and send the return string
        	if(index >= 0)
        	{
        		sprintf(rs,"%s=%u\r\n", pName[index],value);
        	}
        	else
        	{
        		sprintf(rs,"=%u\r\n",value);
        	}
        	txDebug(rs);
        }
    }
}
