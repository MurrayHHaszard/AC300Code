/*
 * port_drive.c
 *
 *  Created on: 16/05/2015
 *      Author: Murray
 */

#include <cr_section_macros.h>
#include <string.h>
#include <stdlib.h>

#include "ac210_global.h"
#include "ac210_pwm.h"
#include "param.h"
#include "drive.h"
#include "remote.h"
#include "sstate.h"

#ifdef MH_YYY		// MHH:21/01/2026

//====================================================================================
/*

From drive.c:

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


A Feather circuit is different to a Reverse circuit.

In the feather circuit, the FEATHER and COARSE lines are joined, in the reverse the REVERSE and FINE lines are joined.

FEATHER           = U3 + L1 = 0x08 + 0x20 = 0x28	: Drive Feather_hi to Fine_lo, direction Feather/Coarse
FEATHER_REVERSE   = U1 + L3 = 0x02 + 0x80 = 0x82    : Drive Fine_hi to Feather_lo, direction Fine

REVERSE           = U3 + L2 = 0x08 + 0x40 = 0x48	: Drive Reverse_hi to Coarse_lo, direction Reverse/Fine
REVERSE_REVERSE   = U2 + L3 = 0x04 + 0x80 = 0x84	: Drive Coarse_hi to Reverse_lo, direction Coarse

Since Beta uses a relay and Manual mode, following drive code only moves out of Beta, in Coarse direction. Same code as Coarse
as the Reverse lines (U3 + L3) are not wired to motor.

BETA_REVERSE      = U2 + L1 = 0x04 + 0x20 = 0x24	: Drive Coarse_hi to Fine_lo, direction Coarse.
BETA_ON           = U3      = 0x08        = 0x08	: This turns on relay


Following defines from drive.c

#define DRIVE_FEATHER           0x28
#define DRIVE_FEATHER_REVERSE   0x82

#define DRIVE_REVERSE           0x48
#define DRIVE_REVERSE_REVERSE   0x84

#define DRIVE_BETA_REVERSE      0x24
#define BETA_ON                 0x08


From Wiki H-Bridge:

Motor Right = S1 + S4 = U1 + L2 = 0x02 + 0x04 = 0x42 = FINE
Motor left  = S2 + S3 = L1 + U2 = 0x20 + 0x04 = 0x24 = COARSE
Brake Right = S2 + S4 = L1 + L2 = 0x60 = BRAKE_FINE
Brake left  = S1 + S3 = U1 + U2 = 0x06 = BRAKE_COARSE


S1 = U1
S2 = L1
S3 = U2
S4 = L2

Note: Since we can PWM L1 and L2, we should be able PWM brake?

*/
#define DRIVE_U_PORT_NUM	PINPORT(AC210_J2_8_DRIVE_U1_PIND)		// Should be 1

#define DRIVE_U1_PIN	PINMASK(AC210_J2_8_DRIVE_U1_PIND)
#define DRIVE_U2_PIN	PINMASK(AC210_J2_10_DRIVE_U2_PIND)
#define DRIVE_U3_PIN	PINMASK(AC210_J2_11_DRIVE_U3_PIND)

#define DRIVE_U1_BIT	(1 << DRIVE_U1_PIN)
#define DRIVE_U2_BIT	(1 << DRIVE_U2_PIN)
#define DRIVE_U3_BIT	(1 << DRIVE_U3_PIN)

#define DRIVE_U_BIT_MASK	(DRIVE_U1_BIT | DRIVE_U2_BIT | DRIVE_U3_BIT)
LPC_GPIO_T *pLPC_DRIVE_PORT;

//----------------------------------------------------------------------
void AC210_DRIVE_Init(void)
{
// Set pin direction to output for DRIVE pins
	pLPC_DRIVE_PORT = LPC_GPIO + DRIVE_U_PORT_NUM;
	pLPC_DRIVE_PORT->DIR |= DRIVE_U_BIT_MASK;		// Set all drive pins to output
	pLPC_DRIVE_PORT->CLR |= DRIVE_U_BIT_MASK;		// Everything off
//	PWM_Init();
}
//----------------------------------------------------------------------
// This will just be used on DRIVE_U pins as DRIVE_L pins will use PWM control
void DRIVE_clear_u_pins(void)
{
	pLPC_DRIVE_PORT->CLR |= DRIVE_U_BIT_MASK;	// MHH:27/03/2019

}
void DRIVE_set_u_pins(uint32_t drive_pin_bits)
{
// Clear existing settings
//	pLPC_DRIVE_PORT->CLR |= DRIVE_U_BIT_MASK;	// MHH:27/03/2019
	pLPC_DRIVE_PORT->SET |= drive_pin_bits;
}
//----------------------------------------------------------------------
uint8_t p_ControlPort;		// This has bit settings from AC200 drive.c
#define U1	0x2
#define U2	0x4
#define U3	0x8

#define L1  0x20
#define L2  0x40
#define L3	0x80

#define U1L1	(U1|L1)
#define U2L2	(U2|L2)
#define U3L3	(U3|L3)

#define L12 (L1 | L2)		// Used for braking


#define L1_PWM_CHAN		3
#define L2_PWM_CHAN		2
#define L3_PWM_CHAN		1


//extern BYTE wasReversing;


//----------------------------------------------------------------------------------------------
static uint32_t DR_usecs;
void AC210_Drive_ms_display(uint32_t d)
{
	uint32_t ms = AC210_ms_elapsed(&DR_usecs);

	PRINTF("[DR:%d,%02x]\r\n",ms,d);
}
//-------------------------------------------------------------------------------
#define DRIVE_FINE				0x42	//        = U1 + L2 = 0x02 + 0x40 = 0x42
#define DRIVE_COARSE    		0x24	//        = U2 + L1 = 0x04 + 0x20 = 0x24
#define DRIVE_FEATHER           0x28
#define DRIVE_FEATHER_REVERSE   0x82

#define DRIVE_REVERSE           0x48
#define DRIVE_REVERSE_REVERSE   0x84

#define DRIVE_BRAKE				0x60

#define DRIVE_BETA_REVERSE      0x24
#define BETA_ON                 0x08

#define DRIVE_IDLE			0

#define MH_DISPLAY_DRIVE_PINS
#ifdef MH_DISPLAY_DRIVE_PINS
extern WORD lastSpeed;

void Display_drive_pins(uint32_t d)
{
	char *desc="Unknown";
	static uint32_t last_d;

	switch(d)
	{
	case DRIVE_IDLE:
		desc = "Idle";
		if((last_d == DRIVE_COARSE) || (last_d == DRIVE_FINE))
		{
			last_d = last_d;		// dummy for breakpoint.
		}
		break;
	case DRIVE_BRAKE:
		desc = "Brake";
		break;
	case DRIVE_FINE:
		desc = "Fine";
		break;
	case DRIVE_COARSE:
		desc = "Coarse";
		break;
	case DRIVE_FEATHER:
		desc = "Feather";
		break;
	case DRIVE_FEATHER_REVERSE:
		desc = "FeatherReverse";
		break;
	case DRIVE_REVERSE:
		desc = "Reverse";
		break;
	case DRIVE_REVERSE_REVERSE:
		desc = "ReverseReverse";
		break;
//	case DRIVE_BETA_REVERSE:	// Same as DRIVE_COARSE
//		desc = "BetaReverse";
//		break;
	case BETA_ON:
		desc = "BetaOn";
		break;
	}
	DPRINTF("Drive:%s,%d\r\n",desc,lastSpeed);
	DPRINTF_FLUSH;
//    AC210_uart0_wait();

	last_d = d;
}
#endif
//-------------------------------------------------------------------------------
static void Drive_invalid(uint32_t d)
{
	char buff[50];

	sprintf(buff,"Drive_invalid:d=%04x",d);
//	for(;;);
	Abort(AC210_SRC_DRIVE+5,buff);
}
//-------------------------------------------------------------------------------
static uint32_t d_save=1;
//static bool Reverse_active=false;
void p_SetDrivePins(void)
{
	uint32_t drive_pins=0;
	uint32_t d = p_ControlPort;


	if(d == d_save)
	{
		return;
	}
//	if(getParameter (BL_ENABLED) != 1) return;// MHH:05/04/2024. Had a Drive_invalid Abort when turning beta on in AC200User
	if(getParameter (BL_ENABLED) == 1) return;// MHH:08/05/2024. May have above test wrong!!

	d_save = d;

	uint8_t u = (d & 0xf);
	uint8_t l = (d & (0xf << 4));

	switch(u)
	{
	case U1:	// FINE
		if(l == L2)	// FINE->COARSE
		{
			break;
		}
		if(l == L3)	// FINE->FEATHER. Only valid when returning from FEATHER, relay should be off
		{
			break;
		}
		Drive_invalid(d);
		return;

	case U2:	// COARSE
		if(l == L1)	// COARSE->FINE
		{
			break;
		}
		if(l == L3)	// COARSE->FEATHER. Only valid when returning from REVERSE, so must be able to reverse. Relay may be on.
		{
			WORD cw = getParameter (AC_CONTROL_WORD);
			{
				if(cw & CW_NON_BETA_REVERSING) break;
			}
		}
		Drive_invalid(d);
		return;

	case U3:	// FEATHER
		if(l == L1)	// FEATHER->FINE only valid when Feathering
		{
			break;
		}
		if(l == L2)	// FEATHER->COARSE only valid when REVERSING
		{
			WORD cw = getParameter (AC_CONTROL_WORD);
			{
				if(cw & CW_NON_BETA_REVERSING) break;
			}
		}
		if(l == 0)	// BETA REVERSE?
		{
			WORD cw = getParameter (AC_CONTROL_WORD);
			if(cw & CW_BETA_REVERSING) break;
		}
		Drive_invalid(d);
		return;

	case 0:	// IDLE or Braking.
		if(l == 0)	// IDLE
		{
			break;
		}
		if(l == (L1 + L2))
		{
			break;
		}
		Drive_invalid(d);
		return;

	default:
		Drive_invalid(d);
		return;
	}

//	BYTE t = Trace("p_SetDrivePins");

//	AC210_Drive_ms_display(d);

	DRIVE_clear_u_pins();	// MHH:27/03/2019. Was being done in set before.

	PWM_Disable(0);		// Disable all PWM channels

	// map U bits

	if((d & U1) != 0)	drive_pins |= DRIVE_U1_BIT;
	if((d & U2) != 0)	drive_pins |= DRIVE_U2_BIT;
	if((d & U3) != 0)	drive_pins |= DRIVE_U3_BIT;

#define DRIVE_CHECK
#ifdef DRIVE_CHECK
	if((d & U1L1) == U1L1) Abort(AC210_SRC_DRIVE+10,"p_SetDrivePins:U1L1");
	if((d & U2L2) == U2L2) Abort(AC210_SRC_DRIVE+20,"p_SetDrivePins:U2L2");
	if((d & U3L3) == U3L3) Abort(AC210_SRC_DRIVE+30,"p_SetDrivePins:U3L3");
#endif

//	Display_drive_pins(d);
	DRIVE_set_u_pins(drive_pins);

	if((d & L1) != 0) PWM_Enable(L1_PWM_CHAN);	// MHH:27/03/2019. Move to after U pins set
	if((d & L2) != 0) PWM_Enable(L2_PWM_CHAN);
	if((d & L3) != 0) PWM_Enable(L3_PWM_CHAN);

	Display_drive_pins(d);
//	TraceReturn(t,"");
}
//----------------------------------------------------------------------------------------------------------
/* The idea is to look at what is currently happening and take action depending on request.
 *
 * If idle, and a request is made to go Fine, then set pins to FINE and softstart
 * if idle, and a request is made to go Coarse, then set pins to COARSE and softstart
 * if FINE, and a request to go idle, then set pins to fine_brake and brake.
 * if FINE, and a request to go Coarse, then fine_brake and then go coarse
 * Similarly going from Coarse to idle or fine.
 *
 * So, any moving command to idle will involve braking.
 * Any idle command to moving will involve a soft start.
 * Any moving command to a different moving command will involve braking then a soft start.
 *
 */

#define COMMAND_UNKNOWN	0

#define DIR_NONE		0
#define DIR_FINE		1
#define DIR_COARSE		2
#define DIR_OTHER		3		// e.g. FEATHER
#define DIR_UNKNOWN		4

#define REQ_IDLE		0
#define REQ_BRAKE		1
#define REQ_DRIVE		2

uint32_t Request_drive_pins=1;	// Invalid value to force ini.
uint8_t Current_dir;

//void setTheSpeed (WORD speed);
void setBrakeDutyCycle(WORD speed);	// In drive.c
extern WORD currentControl;    // In control.c


void SetDrivePins(uint32_t drive_pins)
{
	p_ControlPort = drive_pins;
	p_SetDrivePins();
}
//=======================================================================================================
uint8_t Soft_mode;
#define SOFTMODE_NONE		0
#define SOFTMODE_WAITING_SPEED	1
#define SOFTMODE_STARTING	2
#define SOFTMODE_BRAKING	3
//#define SOFTMODE_NO_SPEED_CONTROL		4
//-------------------------------------------------------------------------------------------------------
WORD Softstart_increment;
WORD Softstart_speed;
WORD Softstart_target_speed;

WORD Softmode_count;
/*
 *  1 = full speed
 *  2 = half speed. Only set if projected err is within 2% of target speed, and NCV first digit = 2. eg 2050
 *  3 = third speed Only set if projected err is within 2% of target speed, and NCV first digit = 3. eg 3050
 */


WORD Softmode_speed_val;		// MHH:27/05/2024.

//#define SOFTSTART_CYCLES	10

//#define SOFTSTART_CYCLES	10	// MHH:04/03/2020
#define SOFTSTART_CYCLES	2	// MHH:17/12/2020
//#define SOFTSTART_START_SPEED	50
#define SOFTSTART_START_SPEED	50	// MHH:17/12/2020

void Drive_softstart(uint32_t drive_pins)
{
	SetDrivePins(drive_pins);
//	Soft_mode = SOFTMODE_WAITING_SPEED;
	Soft_mode = SOFTMODE_STARTING;		// MHH:27/05/2024
	Softmode_count = 0;

#ifdef MH_OLD_SOFTSTART
	WORD target_speed = currentControl;
    WORD maxMotor = getParameter (MC_MAX_PWM);
	if(target_speed < maxMotor/5)
	{
		target_speed = maxMotor/5;
	}
	Softstart_target_speed = target_speed;
	Softstart_speed = target_speed/5;
	Softstart_increment = target_speed/SOFTSTART_CYCLES;
	setTheSpeed(Softstart_speed);	// Initial value
#endif
}
//-------------------------------------------------------------------------------
//WORD Drive_temp_max_pwm = 255;
//WORD Drive_last_temp_max_pwm = 255; Set_cState:

void Set_last_speed(WORD speed,int ifrom)	// Note could include tick.
{
	if(speed != lastSpeed)
	{
		lastSpeed = speed;
//		DPRINTF("Set_last_speed:%d from:%d,tick=%d\r\n",lastSpeed,ifrom,timerTick)
	}
}
void Drive_set_speed2(WORD speed,int ifrom)
{
//	speed = MIN(speed,Drive_temp_max_pwm);
#ifdef MH_DEBUG_SETSPEED
	static WORD last_speed;
	if(speed != last_speed)
	{
		last_speed = speed;
		DPRINTF("Drive_set_speed2:speed=%d from:%d\r\n",speed,ifrom);
	}
#endif
	p_SetDrivePWM(speed,0);
	Set_last_speed(speed,200);
//	lastSpeed = speed;		// This is used by logging.
//	Drive_last_temp_max_pwm = Drive_temp_max_pwm;
}
//-------------------------------------------------------------------------------
#ifdef MH_XXX		// MHH:25/05/2024. Looks redundant.
void Drive_set_temp_max_pwm(WORD temp_max_pwm)
{
	Drive_temp_max_pwm = temp_max_pwm;
	Drive_set_speed2(lastSpeed);
}
void Drive_reset_temp_max_pwm(void)
{
	Drive_temp_max_pwm = 255;
}
#endif
//-------------------------------------------------------------------------------------------------------
bool Drive_keep_starting(void)
{
	if(Softstart_speed >= Softstart_target_speed)		// Finished?
	{
		return true;
	}

	Softstart_speed = Softstart_target_speed;		// MHH:22/05/2024

//	Softstart_speed += Softstart_increment;	// Increment Duty Cycle
//	Softstart_speed = MIN(Softstart_target_speed,Softstart_speed);
	Drive_set_speed2(Softstart_speed,100);
	return false;
}
/*
 * MHH:27/05/2024.
 *
 * In order to implement slow speed for the PD control, we initialise here. Note that we will always start by giving a tick of full power
 *  The Softspeed_value is set in control.c and will have these values:
 *  	0 = full speed
 *  	1 = slow speed
 *  	2 = very slow speed
 *
 *  	We could try using PWM for slow and very slow, just to see what happens.
 *
 */
extern WORD setSpeed;
void Drive_softmode_start(void)		// MHH:27/05/2024
{
	WORD speed = 255;
//	static WORD save_setspeed;

/*
 *  MHH:14/01/2026 The purpose of this routine was a primitive speed control without using PWM as using PWM resulted in low torque.
 *  The idea was to reduce speed as actual speed was close to target speed.
 *  For now we will comment out....
 */


	if(remoteMode() != REMOTE_GROUND_MODE)	// MHH:29/05/2024. For Remote control.
	{
//		save_setspeed = setSpeed;
		switch (Softmode_speed_val)
		{
		default:
			break;
		case 2:
			if(Softmode_count & 1) speed = 0;
			break;
		case 3:
			if(Softmode_count %3) speed = 0;
			break;
		}
	}
	Softmode_count++;
//	if(Softmode_count++ & 1) speed = 0;
//	if(Softmode_count++ %3 == 0) speed = 255;
	Softstart_target_speed = speed;	// Not sure about this...
	Drive_set_speed2(Softstart_target_speed,200);		// Will this work??
}

static WORD Brake_duty_cycle;
static WORD Brake_increment;
static int Brake_count;
#define BRAKE_START_DUTY	50		// Test!!
//#define BRAKE_START_DUTY	20		// MHH:04/03/2020
#define BRAKE_CYCLES	16
//#define BRAKE_CYCLES	10		// MHH: 04/03/2020

void Drive_start_brake(void)
{
//	Drive_reset_temp_max_pwm();		// MHH:25/05/2024. Not sure why it I was doing this....
	Drive_set_speed2(0,300);				// MHH:25/05/2024
//	p_SetDrivePWM(0,0);				// MHH:25/05/2024. Try turning PWM to 0.
	SetDrivePins(DRIVE_BRAKE);
	Brake_duty_cycle  = BRAKE_START_DUTY;
//	Brake_increment  = 75;	// 50,125,200,255,255
	Brake_increment = ((256 - BRAKE_START_DUTY)/(BRAKE_CYCLES-3))+1;	// MHH:04/03/2020
	Brake_count = BRAKE_CYCLES;

	setBrakeDutyCycle(Brake_duty_cycle);	// Initial value
}
//-------------------------------------------------------------------------------------------------------
bool Drive_keep_braking(void)
{
	if(Brake_count <= 0)
	{
		setBrakeDutyCycle(0);	// MHH:27/05/2024
		return true;
	}

	Brake_count--;

	if(Brake_duty_cycle < 255)
	{
		Brake_duty_cycle += Brake_increment;	// Increment Duty Cycle
		Brake_duty_cycle = MIN(Brake_duty_cycle,255);
		setBrakeDutyCycle(Brake_duty_cycle);
	}
	return false;
}
// MHH:22/05/2024
//-------------------------------------------------------------------------------------------------------
extern WORD currentState;	// MHH:18/05/2024
WORD Check_current_control;
// Set_last_speed:
void Set_drive_pins2(uint32_t drive_pins)
{
	uint8_t dir;

#ifdef MH_DEBUG_CC

	if(currentControl != Check_current_control)		// Debug!!
	{
		Check_current_control = currentControl;
		DPRINTF("New cc=%d\r\n",currentControl);
	}
#endif
//	Drive_set_speed2(0);	// To be safe
	p_SetDrivePWM(0,0);		// MHH:25/05/2024
#ifdef MH_SLIPRING_STATE_LOGIC
	if(drive_pins != 0)
	{
		ADC_cancel_any_slipring_state_test();
	}
#endif
//		setTheSpeed(0);
//		SetDrivePins(0);

//	Soft_mode = SOFTMODE_NONE;		// MHH:29/05/2024


	if(drive_pins == BETA_ON)	// Special case - don't bother with braking.
	{
		Request_drive_pins = drive_pins;
		SetDrivePins(drive_pins);
#ifdef MH_XXX		// MHH:30/01/2024
	    WORD maxMotor = getParameter (MC_MAX_PWM);
	    WORD speed = 255;
		speed = MIN(speed,maxMotor);
#endif
		WORD speed = 255;
		p_SetDrivePWM(speed,0);
		Set_last_speed(0,100);	// MHH:25/05/2024
//		lastSpeed = 0;
		Current_dir = DIR_NONE;
		return;
	}
	dir = DIR_UNKNOWN;
	switch(drive_pins)
	{
	default:
		break;

	case DRIVE_FINE:
		dir = DIR_FINE;
		break;

	case DRIVE_COARSE:
		dir = DIR_COARSE;
		break;

	case DRIVE_IDLE:
		dir = DIR_NONE;
		break;

	case DRIVE_FEATHER_REVERSE:
	case DRIVE_REVERSE:
	case DRIVE_FEATHER:
	case DRIVE_REVERSE_REVERSE:
		dir = DIR_OTHER;
		break;

	}
	if(dir == DIR_UNKNOWN)
	{
		Abort(AC210_SRC_DRIVE+40,"Set_drive_pins2");	// MHH:28/05/2024
	}

	if(dir == DIR_OTHER)	// MHH:28/05/2024. No softstart or braking unless FINE or COARSE
	{
		SetDrivePins(drive_pins);
		Request_drive_pins = drive_pins;
		Current_dir = dir;
		return;
	}

#ifdef MH_YYY
	if(Softmode_in_control == false)	// MHH:29/05/2024. For Remote control. Same logic as above but test first
	{
		Drive_set_speed2(255,350);
//		SetDrivePins(drive_pins);
		Request_drive_pins = drive_pins;
		Current_dir = dir;
		return;
	}
#endif

/*
 * MHH:28/05/2024.
 *
 * 		if(stopped at a FINE or COARSE stop) then
 * 		    if(new drive_pins == DRIVE_IDLE) then
 * 		        Set drive to idle
 * 		    else		// Must be driving fine or coarse
 *			    initiate drive softstart
 *			endif
 *		else		// Not stopped at FINE or COARSE stop
 *		    if(current drive pins are DRIVE_FINE or DRIVE_COARSE) then
 *		        initiate braking		// We are here because the new drive pins are different to the current drive pins
 *		        Set Softmode to BRAKING	// As we have saved requested drive pins, we will be able to set them when braking has finished.
 *			endif
 *		endif
 *
 *
 */


	if(dir == DIR_NONE)
	{
//		if(Current_dir != DIR_NONE)
		if((Current_dir == DIR_FINE) || (Current_dir == DIR_COARSE)) // MHH:16/01/2026. Because was braking when should have been unfeathering
		{
			if((currentState & (S_STOP_FINE | S_STOP_COARSE)) == 0)// MHH:28/05/2024. Only brake if not at a stop
			{
				Drive_start_brake();
				Soft_mode = SOFTMODE_BRAKING;
			}
		}
	}
	else
	{
		Soft_mode = SOFTMODE_NONE;
		SetDrivePins(drive_pins);			// MHH:14/01/2028
#ifdef MH_XXX		// MHH:16/01/2026
		if(Soft_mode != SOFTMODE_BRAKING)	// MHH:29/05/2024
		{
			SetDrivePins(drive_pins);			// MHH:14/01/2026
//			Drive_softstart(drive_pins);		// MHH:28/05/2024
		}
#endif
	}
#ifdef MH_YYY
	if((currentState & (S_STOP_FINE | S_STOP_COARSE)))// MHH:28/05/2024. Only brake if not at a stop
	{
		if(dir == DIR_NONE)
		{
			SetDrivePins(drive_pins);
		}
		else
		{
			Drive_softstart(drive_pins);		// MHH:27/05/2024
		}
	}
	else
	{
		if(dir == DIR_NONE)
		{
			Drive_start_brake();
			Soft_mode = SOFTMODE_BRAKING;
		}
		else
		{
			Drive_softstart(drive_pins);		// MHH:27/05/2024
		}
	}
#endif
#ifdef MH_XXX		// MHH:28/05/2024
	if(Current_dir == DIR_NONE)	// MHH:18/05/2024
	{
		if(dir == DIR_NONE)
		{
			SetDrivePins(drive_pins);
		}
		else
		{
			Drive_softstart(drive_pins);		// MHH:27/05/2024
		}
	}
	else
	{
		if(currentState & (S_STOP_FINE | S_STOP_COARSE) == 0)// MHH:28/05/2024. Only brake if not at a stop
		{
			Drive_start_brake();
			Soft_mode = SOFTMODE_BRAKING;
		}
	}
#endif
	Request_drive_pins = drive_pins;
	Current_dir = dir;
}
//--------------------------------------------------------------------------
void Set_drive_pins(uint32_t drive_pins)
{
//#ifdef MH_YYY		// MHH:27/05/2024
	if(drive_pins == Request_drive_pins)
	{
		return;
	}
//#endif
	Set_drive_pins2(drive_pins);

}
//--------------------------------------------------------------------------
int Get_SIG100_command_from_drive_pins(void)
{
	int command = '.';		// default


	if(dState == MD_BETA_EXIT)	// Might have to put some more safeguards here...
	{
		command = '+';
		return command;
	}

	switch(Request_drive_pins)
	{
	case DRIVE_FINE:
	case DRIVE_FEATHER_REVERSE:
		command = '-';
		break;
	case DRIVE_COARSE:
		command = '+';
		break;
	case DRIVE_FEATHER:
		command = 'F';
		break;
/*
	case BETA_ON:
		command = 'R';
		break;
*/
	default:
		break;
	}
	return command;
}
//--------------------------------------------------------------------------
int Get_SIG100_command_from_dState(void)
{
	int command = '.';		// default

	switch(dState)
	{
	case MD_FINER:
		command = '-';
		break;

	case MD_FEATHER_REVERSE:
	case MD_REVERSE_REVERSE:
#ifdef MH_XXX		// MHH:16/07/2025
		if(Feather_mode == FM_IDLE)	// MHH:14/11/2023
		{
			if(Control_type == CT_REVERSE) command = '+';
			if(Control_type == CT_FEATHERING) command = '-';
		}
		else
#endif
		{
			if(Feather_mode == FM_UNREVERSING) command = '+';
			if(Feather_mode == FM_UNFEATHERING) command = '-';
		}
		break;

	case MD_COARSER:
		command = '+';
		break;

	case MD_BETA_EXIT:
		command = '+';
		break;

	case MD_FEATHER:
	case MD_REVERSE:
#ifdef MH_XXX		// MHH:16/07/2025
		if(Feather_mode == FM_IDLE)	// MHH:14/11/2023
		{
			if(Control_type == CT_REVERSE)
			{
				command = 'R';
				break;
			}
			if(Control_type == CT_FEATHERING)
			{
				command = 'F';
			}
		}
		else
#endif
		{
			if(Feather_mode == FM_REVERSING) command = 'R';
			if(Feather_mode == FM_FEATHERING) command = 'F';
		}
		break;
/*
	case BETA_ON:
		command = 'R';
		break;
*/
	default:
		break;
	}
	return command;
}
//--------------------------------------------------------------------------
bool SIG100_beta_mode_on(void)
{
	if(p_ControlPort == BETA_ON) return true;
	return false;
}
//--------------------------------------------------------------------------
// Performed every cycle
bool Drive_wait_flag=false;
int Drive_wait_count;
extern BYTE State_fast_test;		// Use as an interlock when a fast test in progress

void Drive_softcheck(void)
{
	if(Drive_wait_flag)
	{
		if(Drive_wait_count++ > 5)
		{
			Drive_wait_flag = false;
		}
	}

	switch(Soft_mode)
	{
	case SOFTMODE_NONE:

// Leave for now. Should stay at 50% pwm for test


	case SOFTMODE_WAITING_SPEED:
		return;

	case SOFTMODE_BRAKING:
/*
		if(State_fast_test)
		{
			mh_debug();
		}
*/
//		Drive_wait_flag = true;	// MHH:09/05/2024
		Drive_wait_count = 0;
		if(Drive_keep_braking())
		{
			Current_dir = DIR_NONE;
			Set_drive_pins2(Request_drive_pins);	// I think this should be IDLE
			Soft_mode = SOFTMODE_NONE;		// MHH:29/05/2024
		}
		return;

	case SOFTMODE_STARTING:
		Drive_set_speed2(Softstart_target_speed,200);
//		Drive_softmode_start();
//		Soft_mode = SOFTMODE_NONE;		// MHH:27/05/2024
#ifdef MH_YYY	// MHH:27/05/2024

		if(Drive_keep_starting())
		{
			Soft_mode = SOFTMODE_NONE;
		}
#endif
		return;
	}
}
//-------------------------------------------------------------------------------
void Drive_set_speed(WORD speed)	// Called from setTheSpeed() in drive.c
{
	Drive_set_speed2(speed,500);
#ifdef MH_XXX		// MHH:14/01/2026

	WORD target_speed;
//	Drive_reset_temp_max_pwm();		// MHH:25/05/2024. Not sure why I was doing this...

//	Soft_mode = SOFTMODE_NONE;		// MHH:27/05/2024

	switch(Soft_mode)
	{
	case SOFTMODE_WAITING_SPEED:
		target_speed = speed;
		Softstart_target_speed = target_speed;
//		Softstart_speed = target_speed/5;
		Softstart_speed = MIN(target_speed,SOFTSTART_START_SPEED);	// MHH:04/03/2020
		Softstart_increment = target_speed/SOFTSTART_CYCLES;
		Drive_set_speed2(Softstart_speed,400);
		Soft_mode = SOFTMODE_STARTING;
		break;

	case SOFTMODE_NONE:
		if(speed != Softstart_target_speed)
		{
			Drive_set_speed2(speed,500);
			Softstart_target_speed = speed;
		}
	}
#endif
}
#else
//====================================================================================
/*

From drive.c:

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


A Feather circuit is different to a Reverse circuit.

In the feather circuit, the FEATHER and COARSE lines are joined, in the reverse the REVERSE and FINE lines are joined.

FEATHER           = U3 + L1 = 0x08 + 0x20 = 0x28	: Drive Feather_hi to Fine_lo, direction Feather/Coarse
FEATHER_REVERSE   = U1 + L3 = 0x02 + 0x80 = 0x82    : Drive Fine_hi to Feather_lo, direction Fine

REVERSE           = U3 + L2 = 0x08 + 0x40 = 0x48	: Drive Reverse_hi to Coarse_lo, direction Reverse/Fine
REVERSE_REVERSE   = U2 + L3 = 0x04 + 0x80 = 0x84	: Drive Coarse_hi to Reverse_lo, direction Coarse

Since Beta uses a relay and Manual mode, following drive code only moves out of Beta, in Coarse direction. Same code as Coarse
as the Reverse lines (U3 + L3) are not wired to motor.

BETA_REVERSE      = U2 + L1 = 0x04 + 0x20 = 0x24	: Drive Coarse_hi to Fine_lo, direction Coarse.
BETA_ON           = U3      = 0x08        = 0x08	: This turns on relay


Following defines from drive.c

#define DRIVE_FEATHER           0x28
#define DRIVE_FEATHER_REVERSE   0x82

#define DRIVE_REVERSE           0x48
#define DRIVE_REVERSE_REVERSE   0x84

#define DRIVE_BETA_REVERSE      0x24
#define BETA_ON                 0x08


From Wiki H-Bridge:

Motor Right = S1 + S4 = U1 + L2 = 0x02 + 0x04 = 0x42 = FINE
Motor left  = S2 + S3 = L1 + U2 = 0x20 + 0x04 = 0x24 = COARSE
Brake Right = S2 + S4 = L1 + L2 = 0x60 = BRAKE_FINE
Brake left  = S1 + S3 = U1 + U2 = 0x06 = BRAKE_COARSE


S1 = U1
S2 = L1
S3 = U2
S4 = L2

Note: Since we can PWM L1 and L2, we should be able PWM brake?

*/
#define DRIVE_U_PORT_NUM	PINPORT(AC210_J2_8_DRIVE_U1_PIND)		// Should be 1

#define DRIVE_U1_PIN	PINMASK(AC210_J2_8_DRIVE_U1_PIND)
#define DRIVE_U2_PIN	PINMASK(AC210_J2_10_DRIVE_U2_PIND)
#define DRIVE_U3_PIN	PINMASK(AC210_J2_11_DRIVE_U3_PIND)

#define DRIVE_U1_BIT	(1 << DRIVE_U1_PIN)
#define DRIVE_U2_BIT	(1 << DRIVE_U2_PIN)
#define DRIVE_U3_BIT	(1 << DRIVE_U3_PIN)

#define DRIVE_U_BIT_MASK	(DRIVE_U1_BIT | DRIVE_U2_BIT | DRIVE_U3_BIT)
LPC_GPIO_T *pLPC_DRIVE_PORT;

//----------------------------------------------------------------------
void AC210_DRIVE_Init(void)
{
// Set pin direction to output for DRIVE pins
	pLPC_DRIVE_PORT = LPC_GPIO + DRIVE_U_PORT_NUM;
	pLPC_DRIVE_PORT->DIR |= DRIVE_U_BIT_MASK;		// Set all drive pins to output
	pLPC_DRIVE_PORT->CLR |= DRIVE_U_BIT_MASK;		// Everything off
//	PWM_Init();
}
//----------------------------------------------------------------------
// This will just be used on DRIVE_U pins as DRIVE_L pins will use PWM control
void DRIVE_clear_u_pins(void)
{
	pLPC_DRIVE_PORT->CLR |= DRIVE_U_BIT_MASK;	// MHH:27/03/2019

}
void DRIVE_set_u_pins(uint32_t drive_pin_bits)
{
// Clear existing settings
//	pLPC_DRIVE_PORT->CLR |= DRIVE_U_BIT_MASK;	// MHH:27/03/2019
	pLPC_DRIVE_PORT->SET |= drive_pin_bits;
}
//----------------------------------------------------------------------
uint8_t p_ControlPort;		// This has bit settings from AC200 drive.c
#define U1	0x2
#define U2	0x4
#define U3	0x8

#define L1  0x20
#define L2  0x40
#define L3	0x80

#define U1L1	(U1|L1)
#define U2L2	(U2|L2)
#define U3L3	(U3|L3)

#define L12 (L1 | L2)		// Used for braking


#define L1_PWM_CHAN		3
#define L2_PWM_CHAN		2
#define L3_PWM_CHAN		1


//extern BYTE wasReversing;


//----------------------------------------------------------------------------------------------
static uint32_t DR_usecs;
void AC210_Drive_ms_display(uint32_t d)
{
	uint32_t ms = AC210_ms_elapsed(&DR_usecs);

	PRINTF("[DR:%d,%02x]\r\n",ms,d);
}
//-------------------------------------------------------------------------------
#define DRIVE_FINE				0x42	//        = U1 + L2 = 0x02 + 0x40 = 0x42
#define DRIVE_COARSE    		0x24	//        = U2 + L1 = 0x04 + 0x20 = 0x24
#define DRIVE_FEATHER           0x28
#define DRIVE_FEATHER_REVERSE   0x82

#define DRIVE_REVERSE           0x48
#define DRIVE_REVERSE_REVERSE   0x84

#define DRIVE_BRAKE				0x60

#define DRIVE_BETA_REVERSE      0x24
#define BETA_ON                 0x08

#define DRIVE_IDLE			0

//#define MH_DISPLAY_DRIVE_PINS
#ifdef MH_DISPLAY_DRIVE_PINS
extern WORD lastSpeed;

void Display_drive_pins(uint32_t d)
{
	char *desc="Unknown";
	static uint32_t last_d;

	switch(d)
	{
	case DRIVE_IDLE:
		desc = "Idle";
		if((last_d == DRIVE_COARSE) || (last_d == DRIVE_FINE))
		{
			last_d = last_d;		// dummy for breakpoint.
		}
		break;
	case DRIVE_BRAKE:
		desc = "Brake";
		break;
	case DRIVE_FINE:
		desc = "Fine";
		break;
	case DRIVE_COARSE:
		desc = "Coarse";
		break;
	case DRIVE_FEATHER:
		desc = "Feather";
		break;
	case DRIVE_FEATHER_REVERSE:
		desc = "FeatherReverse";
		break;
	case DRIVE_REVERSE:
		desc = "Reverse";
		break;
	case DRIVE_REVERSE_REVERSE:
		desc = "ReverseReverse";
		break;
//	case DRIVE_BETA_REVERSE:	// Same as DRIVE_COARSE
//		desc = "BetaReverse";
//		break;
	case BETA_ON:
		desc = "BetaOn";
		break;
	}
	DPRINTF("Drive:%s,%d\r\n",desc,lastSpeed);
//    AC210_uart0_wait();

	last_d = d;
}
#endif
//-------------------------------------------------------------------------------
static void Drive_invalid(uint32_t d)
{
	char buff[50];

	sprintf(buff,"Drive_invalid:d=%04x",d);
//	for(;;);
	Abort(AC210_SRC_DRIVE+5,buff);
}
//-------------------------------------------------------------------------------
static uint32_t d_save=1;
//static bool Reverse_active=false;
void p_SetDrivePins(void)
{
	uint32_t drive_pins=0;
	uint32_t d = p_ControlPort;


	if(d == d_save)
	{
		return;
	}
	d_save = d;

	uint8_t u = (d & 0xf);
	uint8_t l = (d & (0xf << 4));

	switch(u)
	{
	case U1:	// FINE
		if(l == L2)	// FINE->COARSE
		{
			break;
		}
		if(l == L3)	// FINE->FEATHER. Only valid when returning from FEATHER, relay should be off
		{
			break;
		}
		Drive_invalid(d);
		return;

	case U2:	// COARSE
		if(l == L1)	// COARSE->FINE
		{
			break;
		}
		if(l == L3)	// COARSE->FEATHER. Only valid when returning from REVERSE, so must be able to reverse. Relay may be on.
		{
			WORD cw = getParameter (AC_CONTROL_WORD);
			if(cw & CW_NON_BETA_REVERSING) break;
		}
		Drive_invalid(d);
		return;

	case U3:	// FEATHER
		if(l == L1)	// FEATHER->FINE only valid when Feathering
		{
			break;
		}
		if(l == L2)	// FEATHER->COARSE only valid when REVERSING
		{
			WORD cw = getParameter (AC_CONTROL_WORD);
			if(cw & CW_NON_BETA_REVERSING) break;
		}
		if(l == 0)	// BETA REVERSE?
		{
			WORD cw = getParameter (AC_CONTROL_WORD);
			if(cw & CW_BETA_REVERSING) break;
		}
		Drive_invalid(d);
		return;

	case 0:	// IDLE or Braking.
		if(l == 0)	// IDLE
		{
			break;
		}
		if(l == (L1 + L2))
		{
			break;
		}
		Drive_invalid(d);
		return;

	default:
		Drive_invalid(d);
		return;
	}

//	BYTE t = Trace("p_SetDrivePins");

//	AC210_Drive_ms_display(d);

	DRIVE_clear_u_pins();	// MHH:27/03/2019. Was being done in set before.

	PWM_Disable(0);		// Disable all PWM channels

	// map U bits

	if((d & U1) != 0)	drive_pins |= DRIVE_U1_BIT;
	if((d & U2) != 0)	drive_pins |= DRIVE_U2_BIT;
	if((d & U3) != 0)	drive_pins |= DRIVE_U3_BIT;

#define DRIVE_CHECK
#ifdef DRIVE_CHECK
	if((d & U1L1) == U1L1) Abort(AC210_SRC_DRIVE+10,"p_SetDrivePins:U1L1");
	if((d & U2L2) == U2L2) Abort(AC210_SRC_DRIVE+20,"p_SetDrivePins:U2L2");
	if((d & U3L3) == U3L3) Abort(AC210_SRC_DRIVE+30,"p_SetDrivePins:U3L3");
#endif

//	Display_drive_pins(d);
	DRIVE_set_u_pins(drive_pins);

	if((d & L1) != 0) PWM_Enable(L1_PWM_CHAN);	// MHH:27/03/2019. Move to after U pins set
	if((d & L2) != 0) PWM_Enable(L2_PWM_CHAN);
	if((d & L3) != 0) PWM_Enable(L3_PWM_CHAN);

	//	Display_drive_pins(d);
//	TraceReturn(t,"");
}
//----------------------------------------------------------------------------------------------------------
/* The idea is to look at what is currently happening and take action depending on request.
 *
 * If idle, and a request is made to go Fine, then set pins to FINE and softstart
 * if idle, and a request is made to go Coarse, then set pins to COARSE and softstart
 * if FINE, and a request to go idle, then set pins to fine_brake and brake.
 * if FINE, and a request to go Coarse, then fine_brake and then go coarse
 * Similarly going from Coarse to idle or fine.
 *
 * So, any moving command to idle will involve braking.
 * Any idle command to moving will involve a soft start.
 * Any moving command to a different moving command will involve braking then a soft start.
 *
 */

#define COMMAND_UNKNOWN	0

#define DIR_NONE		0
#define DIR_FINE		1
#define DIR_COARSE		2
#define DIR_UNKNOWN		3

#define REQ_IDLE		0
#define REQ_BRAKE		1
#define REQ_DRIVE		2

uint32_t Request_drive_pins=1;	// Invalid value to force ini.
uint8_t Current_dir;

//void setTheSpeed (WORD speed);
void setBrakeDutyCycle(WORD speed);	// In drive.c
extern WORD currentControl;    // In control.c

//uint32_t Drivepins_after_brake;		// MHH:21/01/2026
uint32_t Drivepins;
void SetDrivePins(uint32_t drive_pins)
{
	p_ControlPort = drive_pins;
	p_SetDrivePins();
	Drivepins = drive_pins;
}
//=======================================================================================================
uint8_t Soft_mode;
#define SOFTMODE_NONE		0
#define SOFTMODE_WAITING_SPEED	1
#define SOFTMODE_STARTING	2
#define SOFTMODE_BRAKING	3
//-------------------------------------------------------------------------------------------------------
WORD Softstart_increment;
WORD Softstart_speed;
WORD Softstart_target_speed;

//#define SOFTSTART_CYCLES	10

//#define SOFTSTART_CYCLES	10	// MHH:04/03/2020
#define SOFTSTART_CYCLES	5	// MHH:21/01/2026
//#define SOFTSTART_START_SPEED	50
#define SOFTSTART_START_SPEED	50	// MHH:17/12/2020

#ifdef MH_YYY	// MHH:22/01/2026
void Drive_softstart(uint32_t drive_pins)
{
	SetDrivePins(drive_pins);
	Soft_mode = SOFTMODE_WAITING_SPEED;
#ifdef MH_OLD_SOFTSTART
	WORD target_speed = currentControl;
    WORD maxMotor = getParameter (MC_MAX_PWM);
	if(target_speed < maxMotor/5)
	{
		target_speed = maxMotor/5;
	}
	Softstart_target_speed = target_speed;
	Softstart_speed = target_speed/5;
	Softstart_increment = target_speed/SOFTSTART_CYCLES;
	setTheSpeed(Softstart_speed);	// Initial value
#endif
}
#endif
//-------------------------------------------------------------------------------
WORD Drive_temp_max_pwm = 255;
WORD Drive_last_temp_max_pwm = 255;
void Drive_set_speed2(WORD speed)
{
	speed = MIN(speed,Drive_temp_max_pwm);
	p_SetDrivePWM(speed,0);
	lastSpeed = speed;
	Drive_last_temp_max_pwm = Drive_temp_max_pwm;
}
//-------------------------------------------------------------------------------
void Drive_set_temp_max_pwm(WORD temp_max_pwm)
{
	Drive_temp_max_pwm = temp_max_pwm;
	Drive_set_speed2(lastSpeed);
}
void Drive_reset_temp_max_pwm(void)
{
	Drive_temp_max_pwm = 255;
}
//-------------------------------------------------------------------------------------------------------
bool Drive_keep_starting(void)
{
	if(Softstart_speed >= Softstart_target_speed)		// Finished?
	{
		return true;
	}

	Softstart_speed += Softstart_increment;	// Increment Duty Cycle
	Softstart_speed = MIN(Softstart_target_speed,Softstart_speed);
	Drive_set_speed2(Softstart_speed);
	return false;
}
//-------------------------------------------------------------------------------------------------------
static WORD Brake_duty_cycle;
static WORD Brake_increment;
static int Brake_count;
#define BRAKE_START_DUTY	50		// Test!!
//#define BRAKE_START_DUTY	20		// MHH:04/03/2020
#define BRAKE_CYCLES	5
//#define BRAKE_CYCLES	10		// MHH: 04/03/2020

void Drive_start_brake(void)
{
	Soft_mode = SOFTMODE_BRAKING;
	Drive_reset_temp_max_pwm();
	SetDrivePins(DRIVE_BRAKE);
	Brake_duty_cycle  = BRAKE_START_DUTY;
//	Brake_increment  = 75;	// 50,125,200,255,255
	Brake_increment = ((256 - BRAKE_START_DUTY)/(BRAKE_CYCLES-3))+1;	// MHH:04/03/2020
	Brake_count = BRAKE_CYCLES;

	setBrakeDutyCycle(Brake_duty_cycle);	// Initial value
}
//-------------------------------------------------------------------------------------------------------
bool Drive_keep_braking(void)
{
	if(Brake_count <= 0)
	{
		return true;
	}

	Brake_count--;

	if(Brake_duty_cycle < 255)
	{
		Brake_duty_cycle += Brake_increment;	// Increment Duty Cycle
		Brake_duty_cycle = MIN(Brake_duty_cycle,255);
		setBrakeDutyCycle(Brake_duty_cycle);
	}
	return false;
}
//-------------------------------------------------------------------------------------------------------
void Set_drive_pins2(uint32_t drive_pins)
{
//	Display_drive_pins(drive_pins);
	Drive_set_speed2(0);	// To be safe
#ifdef MH_SLIPRING_STATE_LOGIC
	if(drive_pins != 0)
	{
		ADC_cancel_any_slipring_state_test();
	}
#endif
//		setTheSpeed(0);
//		SetDrivePins(0);

	Soft_mode = SOFTMODE_NONE;
	switch(drive_pins)
	{
	case BETA_ON:
		Request_drive_pins = drive_pins;
		SetDrivePins(drive_pins);
		WORD maxMotor = getParameter (MC_MAX_PWM);
		WORD speed = 255;
		speed = MIN(speed,maxMotor);
		p_SetDrivePWM(speed,0);
		lastSpeed = 0;
		return;

	default:
		break;

	case DRIVE_FINE:
	case DRIVE_COARSE:
		SetDrivePins(drive_pins);
		Soft_mode = SOFTMODE_WAITING_SPEED;
		break;

	case DRIVE_FEATHER_REVERSE:
	case DRIVE_REVERSE:
	case DRIVE_FEATHER:
	case DRIVE_REVERSE_REVERSE:
	case DRIVE_IDLE:
		SetDrivePins(drive_pins);
		break;

	case DRIVE_BRAKE:
		Drive_start_brake();
		break;
	}

	Request_drive_pins = drive_pins;
}
//--------------------------------------------------------------------------
void Set_drive_pins(uint32_t drive_pins)
{
	if(drive_pins == Request_drive_pins)
	{
		return;
	}
	Set_drive_pins2(drive_pins);

}
//--------------------------------------------------------------------------
int Get_SIG100_command_from_drive_pins(void)
{
	int command = '.';		// default


	if(dState == MD_BETA_EXIT)	// Might have to put some more safeguards here...
	{
		command = '+';
		return command;
	}

	switch(Drivepins)
	{
	case DRIVE_FINE:
	case DRIVE_FEATHER_REVERSE:
		command = '-';
		break;
	case DRIVE_COARSE:
		command = '+';
		break;
	case DRIVE_FEATHER:
		command = 'F';
		break;
/*
	case BETA_ON:
		command = 'R';
		break;
*/
	default:
		break;
	}
	return command;
}
//--------------------------------------------------------------------------
/*
 * typedef enum
{
    MD_FEATHER = 0,
    MD_FEATHER_REVERSE,
    MD_IDLE,
    MD_FINER,
    MD_COARSER,
    MD_BETA,
    MD_BETA_HOLD,		// if speed goes over limit when in beta
    MD_BETA_EXIT,
    MD_FINE_BRAKE,
    MD_COARSE_BRAKE,
	MD_REVERSE,		//  MHH:18/07/2025
	MD_REVERSE_REVERSE
} MotorDriveState;
 *
 */
int Get_SIG100_command_from_dState(void)
{
	int command = '.';		// default

	switch(dState)
	{
#ifdef MH_XXX		// MHH:27/03/2026


	case MD_FINER:
		command = '-';
		break;

	case MD_FEATHER_REVERSE:
		if(Control_type == CT_REVERSE) command = '+';
		if(Control_type == CT_FEATHERING) command = '-';
		break;

	case MD_COARSER:
		command = '+';
		break;

	case MD_BETA_EXIT:
		command = '+';
		break;

	case MD_FEATHER:
		if(Control_type == CT_REVERSE)
		{
			command = 'R';
			break;
		}
		if(Control_type == CT_FEATHERING)
		{
			command = 'F';
		}
		break;
/*
	case BETA_ON:
		command = 'R';
		break;
*/
#else
	case MD_FEATHER:
		command = 'F';
		break;

	case MD_FINER:
	case MD_FEATHER_REVERSE:
		command = '-';
		break;

	case MD_COARSER:
	case MD_BETA_EXIT:
	case MD_REVERSE_REVERSE:
		command = '+';
		break;

	case MD_REVERSE:		//  MHH:18/07/2025
		command = 'R';
		break;

#endif
	default:
		break;
	}
	return command;
}
//--------------------------------------------------------------------------
bool SIG100_beta_mode_on(void)
{
	if(p_ControlPort == BETA_ON) return true;
	return false;
}
//--------------------------------------------------------------------------
// Performed every cycle
bool Drive_wait_flag;
int Drive_wait_count;
extern BYTE State_fast_test;		// Use as an interlock when a fast test in progress

void Drive_softcheck(void)
{
	if(Drive_wait_flag)
	{
		if(Drive_wait_count++ > 5)
		{
			Drive_wait_flag = false;
		}
	}
//	if(Drivepins == DRIVE_IDLE) Soft_mode = SOFTMODE_NONE;	// MHH:24/01/2026
	switch(Soft_mode)
	{
	case SOFTMODE_NONE:
	case SOFTMODE_WAITING_SPEED:
		return;

	case SOFTMODE_BRAKING:
/*
		if(State_fast_test)
		{
			mh_debug();
		}
*/
		Drive_wait_flag = true;
		Drive_wait_count = 0;
		if(Drive_keep_braking())
		{
//			Current_dir = DIR_NONE;
			Set_drive_pins2(DRIVE_IDLE);
		}
		return;

	case SOFTMODE_STARTING:
		if(Drive_keep_starting())
		{
			Soft_mode = SOFTMODE_NONE;
		}
		return;
	}
}
//-------------------------------------------------------------------------------
void Drive_set_speed(WORD speed)	// Called from setTheSpeed() in drive.c
{
	WORD target_speed;
	Drive_reset_temp_max_pwm();

	switch(Soft_mode)
	{
	case SOFTMODE_WAITING_SPEED:
		target_speed = speed;
		Softstart_target_speed = target_speed;
//		Softstart_speed = target_speed/5;
		Softstart_speed = MIN(target_speed,SOFTSTART_START_SPEED);	// MHH:04/03/2020
		Softstart_increment = target_speed/SOFTSTART_CYCLES;
		Drive_set_speed2(Softstart_speed);
		Soft_mode = SOFTMODE_STARTING;
		break;

	case SOFTMODE_NONE:
		if(speed != Softstart_target_speed)
		{
			Drive_set_speed2(speed);
			Softstart_target_speed = speed;
		}
	}
}

#endif
