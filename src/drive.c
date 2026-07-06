/* ------------------------------------------------------------
Title:          Drive.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Pitch Control Motor drive and state functions

Changes:

------------------------------------------------------------ */
#include <stdio.h>

#include "global.h"
#include "analog.h"
#include "drive.h"
#include "digital.h"
#include "param.h"
#include "sstate.h"
#include "control.h"
#if REMOTE_VERSION
#include "remote.h"
#endif
#include "feather.h"

/*
    Direction outputs are ACTIVE HIGH.
    Each transistor is driven directly from the processor pins
    Based on the following truth table

        Pin    Fine      Coarse    Feather     FReverse  OFF
    U1  71       H           L         L          H      L
    U2  72       L           H         L          L      L
    U3  73       L           L         H          L      L
    L1  75       L           H         H          L      L
    L2  76       H           L         L          L      L
    L3  77       L           L         L          H      L

    PWM 74

MHH:
U1 = 0x02
U2 = 0x04
U3 = 0x08

L1 = 0x20
L2 = 0x40
L3 = 0x80

FINE              = U1 + L2 = 0x02 + 0x40 = 0x42
COARSE            = U2 + L1 = 0x04 + 0x20 = 0x24

From Wiki H-Bridge:


Motor Right = S1 + S4 = U1 + L2 = 0x02 + 0x04 = 0x42 = FINE
Motor left  = S2 + S3 = L1 + U2 = 0x20 + 0x04 = 0x24 = COARSE
Brake Right = S2 + S4 = L1 + L2 = 0x60 = BRAKE_FINE
Brake left  = S1 + S3 = U1 + U2 = 0x06 = BRAKE_COARSE

Brake Reverse = L2 + L3 = 0x40 + 0x80 = 0xC0

S1 = U1
S2 = L1
S3 = U2
S4 = L2

Note: Since we can PWM L1 and L2, we should be able PWM brake?

*/

#ifdef AC210_PORT
// Consider putting all these porting elements into a single structure
//uint8_t p_ControlPort;
#define CONTROL_PORT p_ControlPort
#else

#define CONTROL_PORT     P7
#define CONTROL_PORT_DIR PD7

#define PWMBIT  0x10

#define PWM_REG        TA2
#define PWM_MODE       TA2MR
#define PWM_START_REG  TABSR
#define PWM_START_MASK 0x04

#define START_PULSE_REG  TA3
#define START_PULSE_MODE TA3MR
#define START_PULSE_START_REG TABSR
#define START_PULSE_TRIGGER   ONSF

#define START_PULSE_MASK 0x08
#endif

#define DRIVE_MASK              0xEE
#define DRIVE_MASK_BETA         0xE6

#define DRIVE_FINE              0x42
#define DRIVE_COARSE            0x24
#define DRIVE_IDLE				0
#define DRIVE_BRAKE				0x60

#define U1	2
#define U2	4
#define U12	(U1 | U2)

#define L1  0x20
#define L2  0x40
#define L12 (L1 | L2)

#define DRIVE_BRAKE_FINE_TEST	0x60
#define DRIVE_BRAKE_FINE		0x06	//???
#define DRIVE_BRAKE_COARSE		0x06
#define BRAKE_PWM_VAL			1	// Used by graph program to identify braking


//#define DRIVE_BRAKE_FINE		0x00
//#define DRIVE_BRAKE_COARSE		0x00

#define DRIVE_FEATHER           0x28
#define DRIVE_FEATHER_REVERSE   0x82
#define DRIVE_REVERSE           0x48
#define DRIVE_REVERSE_REVERSE   0x84

#define DRIVE_BETA_REVERSE      0x24
#define BETA_ON                 0x08

// Inverted PWM drive
#define MIN_DRIVE  0
#define MAX_DRIVE  255
//#define CYCLE_RATE 2		// Old rate = 20.8 kHz
#define CYCLE_RATE (0)		// new rate about 60 khz for Maxon

#define FEATHER_MAX_DRIVE 255

// max error allowed from PI control
#define MAX_ERROR  (MAX_DRIVE-MIN_DRIVE)

MotorDriveState dState;

BYTE lastDir = 0;

volatile BYTE inStartPulse = 0;
WORD afterStartPWMValue;

BYTE PWM_CycleRate=CYCLE_RATE;
#ifdef MH_ATSPEED
BYTE PWM_MaxSpeed=255;
#endif

BYTE AT_POKE_val=0;

// local procedures
//void setIdle (void);

void setDirection (BYTE dir);
void setTheSpeed (WORD speed);
//void StartRotating(WORD targetspeed,ControlState command);
//--------------------------------------------------------------------------------------
WORD SoftStartDelay;
void PWM_SoftStart(WORD targetspeed);
void PWM_SoftStop(void);
extern int Ctl_display;
int AT_Trace;
void loadParameters(void);
void AT_Poke(void)
{
	BYTE t = Trace("AT_Poke");
	switch(AT_POKE_val)
	{
	case 0:
//		txDebug("AT_Poke:Setting Idle\r\n");
		setIdle(100);
		break;

	case 1:
		txDebug("AT_Poke: Displaying pwm history\r\n");
		Ctl_display = 1;
		AT_POKE_val = 0;
		break;

	case 2:
		txDebug("AT_Poke:2 Trying loadParameter()\r\n");
		AT_Trace = 1;
		loadParameters();
		AT_Trace = 0;
		AT_POKE_val = 0;
		break;
	}
	TraceReturn(t,"");
}
//--------------------------------------------------------------------------------------
void triggerStartPulse (WORD pwm)
{
#ifdef AC210_PORT
    setTheSpeed (pwm);	// Ignore one shot for port
#else
    WORD spt;
    spt = getParameter (START_ONE_SHOT_MS) * 2000;
    if (spt)
    {
        inStartPulse = 1;
        afterStartPWMValue = pwm;
        setTheSpeed (255);
        // enable interrupt
        TA3IC = 0x04;
        // set one shot time
        START_PULSE_REG = spt;
        START_PULSE_START_REG |= START_PULSE_MASK;
        START_PULSE_TRIGGER |= START_PULSE_MASK;
    }
    else
    {
        setTheSpeed (pwm);
    }
#endif
}
#ifndef AC210_PORT
#pragma INTERRUPT driveTimer
void driveTimer (void)
{
    // Goes off after the one shot times out
    // set speed to real value
    setTheSpeed (afterStartPWMValue);
    // reset flag
    inStartPulse = 0;
    // disable the int
    TA3IC = 0;
    // stop one shot
    START_PULSE_START_REG &= ~START_PULSE_MASK;
}
#endif
void Set_drive_pins(uint32_t drive_pins);
// Initialisation call
void initDriveControl (void)
{
//	BYTE t = Trace("initDriveControl");
#ifdef AC210_PORT
    CONTROL_PORT &= ~DRIVE_MASK;
#ifdef MH_OLD_SOFTSTART
    p_SetDrivePins();
#else
    Set_drive_pins(CONTROL_PORT);
#endif
    setIdle (200);
    Set_dState(MD_IDLE,300);
#else
	// set up I/O
    // make sure outputs are all inactive..
    CONTROL_PORT &= ~DRIVE_MASK;

    // set PORT direction
    CONTROL_PORT_DIR |= (DRIVE_MASK | PWMBIT);

    // Motor drive is disabled with no direction set

    // Now set up PWM
    PWM_MODE = 0x27;    // 8 bit PWM, input freq = f1..

    // set a direction..
    setIdle ();

    // Start PWM
    PWM_START_REG |= PWM_START_MASK;

    Set_dState(MD_IDLE);


    vector_table[24] = (unsigned long)driveTimer;

    // set up timer TA3 as one shot f/8
    START_PULSE_MODE = 0x42;
#endif
//    TraceReturn(t,"");
}
//------------------------------------------------
#ifdef AC2_TEST		// AC2_TEST
void setDrive ( ControlState control)
{
	switch (control)
    {
    case C_IDLE:
        Drive_set_speed2(0);
        SetDrivePins(DRIVE_IDLE);
        break;

    case C_FINER:
        Drive_set_speed2(MAX_DRIVE);
        SetDrivePins(DRIVE_FINE);
        break;

    case C_COARSER:
        Drive_set_speed2(MAX_DRIVE);
        SetDrivePins(DRIVE_COARSE);
        break;
    case C_BRAKE:
    case C_BL_CONTROL:	// Just added to stop compiler error
    	break;
    }
}
#endif
//------------------------------------------------
extern bool BL_no_dstate_changes;
//#define MH_DISPLAY_DSTATE
#ifdef MH_DISPLAY_DSTATE
char *MD_desc[]=
{
		"FEATHER",
		"FEATHER_REVERSE",
		"IDLE",
		"FINER",
		"COARSER",
		"BETA",
		"BETA_HOLD",
		"BETA_EXIT",
		"FINE_BRAKE",
		"COARSE_BRAKE",
		"REVERSE",
		"REVERSE_REVERSE",
};
#endif
void Set_dState(MotorDriveState new_dstate,int ifrom)
{
	if(new_dstate == dState) return;

#ifdef MH_DISPLAY_DSTATE
	int ix = (int)new_dstate;
	char *desc = "?Unknown?";
	if(ix >= 0 && ix <= 11)
	{
		desc = MD_desc[ix];
	}

	DPRINTF("Set_dState:MD_%s from:%d\r\n",desc,ifrom);		// MHH:27/05/2024
#endif
//    WORD motorCurrent = scaledValue (A_MOTOR_CURRENT);

	// debug
#ifdef MH_XXX
    if (debug & DEBUG_MOTOR_DRIVE)
    {
        txDebug ("Drive  ->");
        switch (new_dstate)
        {
        case MD_FEATHER:
            txDebug("Feather\r\n");
            break;

        case MD_REVERSE:
        	txDebug("Reverse\r\n");
        	break;

        case MD_FEATHER_REVERSE:
        	txDebug ("Feather_Reverse\r\n");
            break;
        case MD_REVERSE_REVERSE:
        	txDebug ("Reverse_Reverse\r\n");
        	break;

        case MD_BETA:
            txDebug ("Beta_Idle\r\n");
            break;

        case MD_BETA_EXIT:
            txDebug ("Beta_Exit\r\n");
            break;

        case MD_BETA_HOLD:
            txDebug ("Beta_Hold\r\n");
            break;

        case MD_IDLE:
            txDebug("Idle\r\n");
            break;

        case MD_FINER:
            txDebug("Fine\r\n");
            break;

        case MD_COARSER:
            txDebug("Coarse\r\n");
            break;

        case MD_FINE_BRAKE:
            txDebug("Fine_Brake\r\n");
            break;

        case MD_COARSE_BRAKE:
        	txDebug("Coarse_Brake\r\n");
        	break;

        default:
        	txDebug("Set_dState:Unknown Drive State\r\n");
        	break;
        }
    }
#endif

    dState = new_dstate;
}
//------------------------------------------------
// repeated call for control
extern WORD currentControl;    // In control.c
extern uint8_t Soft_mode;		// MHH:29/05/2024
#define SOFTMODE_NONE		0
#define SOFTMODE_WAITING_SPEED	1
#define SOFTMODE_STARTING	2
#define SOFTMODE_BRAKING	3

void updateDriveControl (void)
{
	WORD opMode;
    ControlState command;
    WORD errorVal, minMotor, maxMotor, pwmVal;
    MotorDriveState newState = MD_IDLE;
    BYTE in_control;

    if (overCurrentTrip)
    {
        // bail out here if we tripped the overcurrent
        setIdle(300);
        return;
    }

    // get operating mode
    opMode = operatingMode();

    maxMotor = MAX_DRIVE;
    minMotor = MIN_DRIVE;

    // get the command from the PI control
    command = controlState (&errorVal);
    if(command == C_BL_CONTROL)	// MHH:16/05/2023. Need to check overcurrent logic...
    {
    	Set_cState(C_IDLE,2000);
    	return;
    }


    if(Ctl_rpm_per_tick_x10)	// MHH:24/02/2018. RemotePos_Active test redundant.
    {
    	pwmVal = errorVal;
    	pwmVal = MAX(pwmVal,minMotor);
    	pwmVal = MIN(pwmVal,maxMotor);	// probably limit to 254, not sure why
    }
    else
    {
    	// Scale the the error value...
    	// to produce a figure between minMotor & maxMotor
    	pwmVal = ((maxMotor - minMotor) * errorVal) / 255;
    	pwmVal += minMotor;
    }

    if (pwmVal > MAX_DRIVE)
    {
    	pwmVal = MAX_DRIVE;
    }

    switch (dState)
    {
    case MD_BETA:
    case MD_BETA_EXIT:
    case MD_BETA_HOLD:
    case MD_FEATHER:
    case MD_FEATHER_REVERSE:
    case MD_REVERSE:
    case MD_REVERSE_REVERSE:
    	newState = dState;;
//    	newState = MD_REVERSE_REVERSE;
		break;

    case MD_IDLE:
        // see if we are in control
		in_control = TRUE;
		if(opMode == MANUAL) in_control = FALSE;
		if(opMode == FEATHER)
		{
			in_control = FALSE;
			if(Control_type == CT_REVERSE)
			{
				switch(command)
				{
				default:
					break;

				case C_IDLE:
					if(lastDir != 0)
					{
						setIdle(400);
					}
		            break;

				case C_COARSER:
					setDirection(DRIVE_COARSE);
	                setTheSpeed (pwmVal);
	                newState = MD_COARSER;
	                break;
				}
                break;
			}
		}
		if(in_control)
		{
			// we are in control... Oh the POWER!!!
//DPRINTF("Soft_mode:%d\r\n",Soft_mode);
			if(Soft_mode != SOFTMODE_BRAKING)		// MHH:29/05/2024
			{
				if (command > C_IDLE)
				{
					// PI control wants us to do something
					// we can do it.., we really can...._STOP
					enableMotorDrive ();
					switch(command)
					{
					case C_FINER:
						newState = MD_FINER;
						if((currentState & S_STOP_FINE))	// MHH:14/01/2026. As state test uncertain if MOSFETs active
						{
							setIdle(500);
//							newState = MD_IDLE;
							break;
						}

						setDirection (DRIVE_FINE);
			   			setTheSpeed (pwmVal);
						break;

					case C_COARSER:
						newState = MD_COARSER;
						if((currentState & S_STOP_COARSE))	// MHH:14/01/2026
						{
							setIdle(600);
//							newState = MD_IDLE;
							break;
						}

						setDirection (DRIVE_COARSE);
			   			setTheSpeed (pwmVal);
						break;

					default:
						break;

					}
				}
			}
		}
		else
		{
			//			txDebug("drive:SetIdle\r\n");
			setIdle (700);
			newState = MD_IDLE;
		}
		break;

    case MD_FINER:
    case MD_COARSER:
#if BETA_VERSION
    	if ((isBetaProp()) && (opMode == FEATHER))
    	{
    		setIdle(800);
    	}
#endif
    	if(dState == MD_COARSER)	// MHH: 09/02/2018. COARSE LED was blinking very fast as COARSE drive toggled on and off under these conditions.
    	{
    		if((opMode == FEATHER) && (Control_type == CT_REVERSE))
    		{
    			if(command == C_COARSER)
    			{
    				newState = MD_COARSER;
    			}
    		}
    	}


    	if (opMode == MANUAL)
    	{
    		// shut it off immediately
    		// NB Feather control is separate function call
    		setIdle (900);
    		break;
    	}
    	if (opMode != FEATHER)	// What about REVERSE?
    	{
    		// check the command to see if we are still meant to be controlling.
    		switch(command)
    		{
    		case C_IDLE:
    			setIdle (1000);
    			newState = MD_IDLE;
    			break;
    		case C_FINER:
    			newState = MD_FINER;
    			if((currentState & S_STOP_FINE))	// MHH:14/01/2026. As state test uncertain if MOSFETs active
				{
					setIdle(1100);
					break;
				}
    			setDirection(DRIVE_FINE);
    			setTheSpeed (pwmVal);
    			break;
    		case C_COARSER:
    			newState = MD_COARSER;
				if((currentState & S_STOP_COARSE))	// MHH:14/01/2026
				{
					setIdle(1200);
					break;
				}
    			setDirection(DRIVE_COARSE);
    			setTheSpeed (pwmVal);
    			break;
			case C_BRAKE:
//DPRINTF("C_BRAKE:\r\n");
				setDirection (DRIVE_BRAKE);
				break;



    		default:		// Could be C_BL_CONTROL
    			break;
    		}
    	}
    	break;

    case MD_FINE_BRAKE:
    case MD_COARSE_BRAKE:
    	DPRINTF("??drive.c:MD_FINE_BRAKE\r\n");	// Error

    	break;

    }
    // set the new drive state..


    Set_dState(newState,400);

//    TraceReturn(t,"End");

}

// Overcurrent shutdown..
void shutdownDrive (void)
{
	BYTE t = Trace("shutdownDrive");
	setTheSpeed (MIN_DRIVE);
    disableMotorDrive();
    TraceReturn(t,"");
}

// state accessor
MotorDriveState driveState (void)
{
    return (dState);
}

// disable & enable functions
void disableMotorDrive (void)
{
#if BETA_VERSION
    if(Control_type == CT_BETA)
    {
        CONTROL_PORT &= ~DRIVE_MASK_BETA;
    }
    else
    {
        CONTROL_PORT &= ~DRIVE_MASK;
    }
//#else
#endif
    Set_drive_pins(CONTROL_PORT);
    lastDir = 0;
}

void enableMotorDrive (void)
{
    // Enable DRIVE
}

// set in feather mode
void setDriveToFeather (void)
{
//	Set_dState(MD_FEATHER,500);

#if REVERSING_VERSION
    // for reversing, we have to set direction to fine
    // so that sink transistor is on OK
#ifdef MH_XXX		// MHH:16/07/2025
    if(Feather_mode == FM_IDLE)	// MHH:14/11/2023
    {
        if(Control_type == CT_REVERSE)
        {
            setDirection (DRIVE_REVERSE);	// U3 + L2
        }
        else
            // for feathering, we have to set direction to coarse
            // so that sink transistor is on OK. ????
        {
            setDirection (DRIVE_FEATHER);	// U3 + L1
        }
    }
    else
#endif
    {
    	if(Feather_mode == FM_REVERSING)
    	{
    		Set_dState(MD_REVERSE,500);
            setDirection (DRIVE_REVERSE);	// U3 + L2
    	}
    	else
    	{
    		Set_dState(MD_FEATHER,510);
            setDirection (DRIVE_FEATHER);	// U3 + L1
    	}
    }
//#else
#endif
    if(ac2_test_flag == AC2_TEST_ON)	// MHH:20/09/2018
    {
        Drive_set_speed2(MAX_DRIVE);
    }
    else
    {
        setTheSpeed (MAX_DRIVE);
    }
#ifdef MH_NO_SET_DSTATE
    if (debug & DEBUG_MOTOR_DRIVE)
    {

#if REVERSING_VERSION
        if(Control_type == CT_REVERSE)
        {
        	txDebug ("Drive  ->Reverse\r\n");
        }
//#else
        else
        {
        	txDebug ("Drive  ->Feather\r\n");
        }
#endif
    }
#endif
}

void cancelFeatherDrive (void)
{
	BYTE t = Trace("cancelFeatherDrive");
	// Feather_mode
#if BETA_VERSION
	if(Feather_mode == FM_BETA || Feather_mode == FM_REVERSE_BETA)	// MHH:19/07/2025
//    if(Control_type == CT_BETA)
    {
        // leave state alone;
        setIdle(1300);
        if (debug & DEBUG_MOTOR_DRIVE)
        {
            txDebug ("Drive  ->Idle[BETA]\r\n");
        }
    }
    else
    {
//        Feather_mode = FM_IDLE;
    	Set_dState(MD_IDLE,600);
        setIdle(1400);
#ifdef MH_NO_SET_DSTATE
        if (debug & DEBUG_MOTOR_DRIVE)
        {
            txDebug ("Drive  ->Idle\r\n");
        }
#endif
    }
//#else
#endif
    TraceReturn(t,"");
}

#if BETA_VERSION
void setBetaDriveMode (void)
{
	setIdle(1500);
    Set_dState(MD_BETA,700);
    CONTROL_PORT |= BETA_ON;
#ifdef MH_NO_SET_DSTATE
    if (debug & DEBUG_MOTOR_DRIVE)
    {
        txDebug ("Drive  ->Beta(idle)\r\n");
    }
#endif
}

void clearBetaDriveMode(void)
{
	CONTROL_PORT &= ~BETA_ON;
    setIdle(1600);
    Set_dState(MD_IDLE,800);
#ifdef MH_NO_SET_DSTATE
    if (debug & DEBUG_MOTOR_DRIVE)
    {
        txDebug ("Drive  ->Idle\r\n");
    }
#endif
}

void clearBetaDriveModeForReverse (void)
{
    CONTROL_PORT &= ~BETA_ON;
}

// Release relay temporarily due to overspeed or overcurrent
void setBetaHoldMode (void)
{
    CONTROL_PORT &= ~BETA_ON;
    Set_dState(MD_BETA_HOLD,900);
#ifdef MH_NO_SET_DSTATE
    if (debug & DEBUG_MOTOR_DRIVE)
    {
        txDebug ("Drive  ->BetaHold\r\n");
    }
#endif
}

// Re-engage relay once speed comes back down / overcurrent timer times out
void clearBetaHoldMode (void)
{
    Set_dState(MD_BETA,1000);
    CONTROL_PORT |= BETA_ON;
#ifdef MH_NO_SET_DSTATE
    if (debug & DEBUG_MOTOR_DRIVE)
    {
        txDebug ("Drive  ->Beta(idle)\r\n");
    }
#endif
}
#endif
void reverseFeatherDrive (void)
{
#if BETA_VERSION
	if(fState == F_BETA || fState == F_BETA_EXIT)
    {
#ifdef MH_XXX
    	if(Sig100_connected)		// MHH:14/01/2025
        {
        	if(fState == F_REVERSING && Feather_mode == FM_UNFEATHERING)
        	{
        		Set_dState(MD_FEATHER_REVERSE,1090);
        	}
        	else
        	{
        		Set_dState(MD_BETA_EXIT,1095);
        	}
        }
        else
        {
        	Set_dState(MD_BETA_EXIT,1100);
        }
#endif
    	Set_dState(MD_BETA_EXIT,1100);
    }
    else
    {
        if(Feather_mode == FM_UNFEATHERING)
        {
        	Set_dState(MD_FEATHER_REVERSE,1200);
        }
        else
        {
        	if(Feather_mode == FM_UNREVERSING)
        	{
            	Set_dState(MD_REVERSE_REVERSE,1210);
        	}
        }
    }
#endif


/*
#if REVERSING_VERSION
    // need to go coarse
    setDirection (DRIVE_REVERSE_REVERSE);
#elif  BETA_VERSION
    // drive coarse
    setDirection (DRIVE_BETA_REVERSE);
#else
    // need to go fine
    setDirection (DRIVE_FEATHER_REVERSE);
#endif
*/
//	WORD feather_mode = Get_param_Feather_mode();
    switch(Feather_mode)		// MHH:17/07/2025. Not 100% sure about this. Need to know what we have been doing, instead of control type, since we
    							// can have beta and feather or reverse and feather
    {
    case FM_REVERSING:
    case FM_UNREVERSING:	// MHH:14/01/2026
        setDirection (DRIVE_REVERSE_REVERSE);
        break;
    case FM_BETA:
        setDirection (DRIVE_BETA_REVERSE);
        break;

    default:
        setDirection (DRIVE_FEATHER_REVERSE);
        break;
    }

#ifdef MH_XXX    // MHH:16/07/2025
    switch(Control_type)
    {
    case CT_REVERSE:
        setDirection (DRIVE_REVERSE_REVERSE);
        break;

    case CT_BETA:
        setDirection (DRIVE_BETA_REVERSE);
        break;

    default:
        setDirection (DRIVE_FEATHER_REVERSE);
        break;
    }
#endif
    setTheSpeed (MAX_DRIVE);

}

// local routines
void setDirection (BYTE dir)
{
	if(Sig100_connected) return;

	if (dir != lastDir)
    {
    	if (isBetaProp())
    	{
            CONTROL_PORT &= ~DRIVE_MASK_BETA;
    	}
    	else
    	{
            CONTROL_PORT &= ~DRIVE_MASK;
    	}
        CONTROL_PORT |= dir;
        Set_drive_pins(CONTROL_PORT);
        lastDir = dir;
    }
}
void Drive_set_speed(WORD speed);	// MHH:31/08/2018. New softstart logic
WORD lastSpeed = 0;
void setTheSpeed (WORD speed)
{
    WORD maxMotor = MAX_DRIVE;
	speed = MIN(speed,maxMotor);	// MHH:20/08/18 In case we missed any.

#ifdef AC210_PORT
	PWM_CycleRate = getParameter(PWM_RATE);
	p_SetCycleRate(PWM_CycleRate);
	Drive_set_speed(speed);
#endif
}
//------------------------------------------------------------------------------
void setBrakeDutyCycle(WORD speed)
{
	speed = MIN(254,speed);
	Drive_set_speed2(speed);		// MHH:25/05/2024
}
//------------------------------------------------------------------------------
extern BYTE wasReversing;

void setIdle (int ifrom)
{
	if(ifrom != 700)
	{
//		DPRINTF("setIdle:%d\r\n",ifrom);
	}
	disableMotorDrive();
    setTheSpeed (0);	// MHH: 12/12/15
}
