/* ------------------------------------------------------------
Title:          control.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        PI control code

Changes:

------------------------------------------------------------ */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "global.h"
#include "control.h"
#include "param.h"
#include "rpm.h"
#include "digital.h"
#include "sstate.h"
#include "manifold.h"
#include "feather.h"
#if REMOTE_VERSION
#include "remote.h"
#endif
#include "device.h"
#include "drive.h"
#include "leds.h"
#include "log.h"

#include "ac210_sig100.h"

/*
    Hold and Cruise modes are sometimes combined..
    If manifold pressure is enabled and  the system has a feathering prop, then
    the hold functionality will apply to CRUISE mode, and MAP functionality will
    apply to HOLD mode.
    If manifold pressure is enabled on a non-feathering prop, the Feather mode will
    be interpreted as MAP.

    If MAP is functional, the Feather LED is used for MAP indications unless the
    mode is feather and we are driving.

*/

#define TABLE_MODE    0
#define PROP_MODE     1


#define MAX_CONTROL 255

void Set_current_control(WORD cval,int ifrom);

#if REVERSING_VERSION
extern void remoteStartReverse (void);
extern void remoteStopReverse (void);
#endif

ControlState cState;
//long actualSpeed;
WORD actualSpeed;

// these variables can be updated directly from the remote control module
//long setSpeed;
WORD setSpeed;

// end of above comment

WORD currentControl;    // this result..

#define RPM_HISTORY 50
#define CONTROL_HISTORY 2


BYTE wasReversing;

BYTE modifyHoldSpeed;


// ANy modified hold speed is kept here, as changes to it are only
// temporary, the initial value is reset on power up to CRUISE speed

WORD holdSpeed;

char controlDebugBuf[120];

// initialisation
void initControl (void)
{
//    BYTE cnt;

    Set_cState(C_IDLE,100);
//    currentControl = 0;
    Set_current_control(0,10);
    modifyHoldSpeed = 0;
    resetHoldSpeed();
    wasReversing = 0;
}

void resetHoldSpeed (void)
{
    // reset hold speed
    holdSpeed = getParameter (SP_HOLD);
}

// repeated call, control cycle


//#define MH_CONTROL
//#ifdef MH_CONTROL
//---------------------------------------------------------------------------------
//#define RPM_TOLERANCE	25

//int Ctl_rpm_per_tick = 12;		// May need to change this or have as a parameter
//int Ctl_rpm_per_tick = 10;		// Set via ATCTL=nn while testing. Maxon = 10, Globe=4 or 5
// Note: Moving to 1 dec place because a Globe with a non-geared motor is likely to have a rate of change
// of RPM about half of Rotax. Rotax and Globe has rate of change of about 8 RPM/tick, and optimum ATCTL value
// of about 5.
// Rotax and Maxon has a rate of change of about 16 RPM/tick, and an optimum ATCTL value of about 10.

// So, the rule of thumb is about 5/8 of RPM/tick.

// For a non geared engine with Globe we would be looking at rate of change of about 2000/5000 * 8 = 3.2 rpm/sec
// and therefore an optimum ATCTL value of 5/8 * 3.2 = 2, which does not allow fine tuning.

/*

Combination						RPM/Tick	RPM/Sec (x50)   ATCTL
-----------------------------------------------------------------------------------------------
for Globe and direct drive		3.2				160 		2.0
for Globe and Rotax             8.0				400 		5.0
for Maxon and Rotax            16.0				800 	   10.0

*/

//int Ctl_rpm_per_tick=0;
int Ctl_rpm_per_tick_x10 =100;		// Set via ATCTL=nn while testing. Maxon = 10.0, Globe=4.0 or 5.0

int Ctl_setspeed;
uint16_t Ctl_deadband;
int Ctl_actualspeed;
int Ctl_rpm_per_spread_tick;
int  Ctl_pwm_val;
//int Ctl_estimated_current_rpm=5000;	// Completely arbitrary - just interested in relative values
int Ctl_history_total_rpm_change;
int Ctl_display;
//int Ctl_last_pwm_tick;
int Ctl_verify_pipeline_tot;
int Ctl_verify_pipeline_tot5;

/*
 *  The plan is to use just the change in RPM for the last tick to use in PID control.
 */

extern int Prop_rpm_delta_prev;
extern int Prop_rpm_delta_per_tick;
extern int Prop_rpm_delta;
extern uint32_t Prop_rpm_delta_usecs;

extern int Engine_rpm_delta_per_tick_1dec;
extern int Engine_rpm_1dec;
extern int Engine_rpm_1dec_prev;
extern int Prop_int_rpm_1dec;


#ifdef MH_TRY_DYNAMIC_CTL_RATE
int Ctl_act_rpm_per_tick_fine;		// MHH:21/11/2020. An attempt at finding dynamic rpm/tick rate
int Ctl_act_rpm_per_tick_coarse;
int Ctl_pwm_ticks;
int Ctl_count_ticks;
int Ctl_start_rpm;
int Ctl_pipeline_pwm_ticks;
#endif

//int Ctl_pipeline_tick_change;
char Ctl_pbuff[256];
//#define MH_CTL_DEBUG2

#ifdef MH_CTL_DEBUG2
char Ctl_pbuff2[256];	// MHH:09/08/2018
#endif
//----------------------------------------------------------------------------------
//#define CTL_MIN_PWM_VAL			60		// Ignore pwm entries below this value
//#define CTL_MIN_PWM_VAL			180		// Ignore pwm entries below this value
//#define CTL_MIN_PWM_VAL			50		// Ignore pwm entries below this value: MHH:09/08/2018
//#define CTL_MIN_PWM_VAL			200		// Ignore pwm entries below this value: MHH:03/11/2018
#define CTL_MIN_PWM_VAL			1		// MHH:21/11/2020

//#define CTL_1000				1000	// For 3 decimal places
//#define CTL_100					100		// for 2 decimal places

/*
#define CTL_PWM_LAG_TICKS  		4
#define CTL_PWM_SPREAD_TICKS  32     // Number of ticks to spread effect of a power tick over
#define CTL_PWM_HIST_TOT	(CTL_PWM_LAG_TICKS + CTL_PWM_SPREAD_TICKS)
*/

#define CTL_PWM_LAG_TICKS  		4
//#define CTL_PWM_SPREAD_TICKS  32     // Number of ticks to spread effect of a power tick over
#define CTL_PWM_SPREAD_TICKS  16     // MHH:17/12/2020 Number of ticks to spread effect of a power tick over
#define CTL_PWM_HIST_TOT	(CTL_PWM_LAG_TICKS + CTL_PWM_SPREAD_TICKS)



#define CTL_MAX_PWM_HISTORY		(CTL_PWM_HIST_TOT+2)	// +2 to be safe for move.

extern WORD lastSpeed;
extern MotorDriveState dState;
struct CTL_PWM_HIST_struct
{
//	int head;
	int tbl[CTL_MAX_PWM_HISTORY];
} Ctl_pwm_hist;
//------------------------
//#define MH_CTL_TUNE
#ifdef MH_CTL_TUNE
#define CTL_DELTA_X				10	// How many ticks we look back to see rate of expected change vs actual
#define CTL_MAX_RPM_HISTORY		(CTL_DELTA_X+1)
struct CTL_RPM_HIST_struct
{
	int head;
	long tbl[CTL_MAX_RPM_HISTORY];	// long so we can add dec point
} Ctl_rpm_hist;
//} Ctl_rpm_hist,Ctl_estrpm_hist;
#endif
//----------------------------------------------------------------------------------
// Instead of ring buffer, just shuffle all entries one down then insert latest in position zero. Simpler.
//#ifdef MH_XXX
int Ctl_dir_count;	// MHH:17/12/2020
void Ctl_update_history4(void)
{
	int *ip,*ipend;

	BYTE t = Trace("Ctl_update_history4");

	Ctl_pwm_val = 0;
	int pwm = lastSpeed;

//	if(pwm < 254) pwm = pwm * 2;	// MHH:01/11/2018. Comment out as was not moving under load.
									// Consider ignoring entries below a minimum value.

	pwm = MIN(pwm,254);
	if(dState == MD_FINER)
	{
//		DPRINTF("Hist4:FI\r\n");
		Ctl_pwm_val = pwm;		// MHH:22/08/2018
	}
	else
	{
		if(dState == MD_COARSER)
		{
//			DPRINTF("Hist4:CO\r\n");
			Ctl_pwm_val = -pwm;		// MHH:22/08/2018
		}
	}
#ifdef MH_TRY_DYNAMIC_CTL_RATE
	Ctl_pwm_ticks += pwm;		// We may miss the first pwm value this way, need to consider....
	Ctl_count_ticks++;
#endif

#ifdef MH_CTL_DEBUG
	if(Ctl_pwm_val)
	{
		sprintf(Ctl_pbuff,"Ctl_pwm_val:%d\r\n",Ctl_pwm_val);
		txDebug(Ctl_pbuff);
	}
#endif


	if(Ctl_pwm_val != 0)
	{
		if(Ctl_dir_count++ < 1)		//MHH:17/12/2020. Try ignoring first 1 ticks for control purposes.
		{
			TraceReturn(t,"<2");
			return;
		}
	}

	ip = &Ctl_pwm_hist.tbl[CTL_PWM_HIST_TOT-2];
	ipend = &Ctl_pwm_hist.tbl[0];

	while(ip >= ipend)	// ripple move
	{
		*(ip+1) = *ip;
		ip--;
	}

	Ctl_pwm_hist.tbl[0] = Ctl_pwm_val;

	TraceReturn(t,"");
}
//#endif
//----------------------------------------------------------------------------------
//#ifdef MH_XXX
bool Ctl_table_cleared;
void Ctl_ini_tbl(void)
{
	if(Ctl_table_cleared) return;
	DPRINTF("Ctl_ini_tbl:\r\n");
	Ctl_table_cleared = true;
	Ctl_dir_count = 0;
	for(int i=0;i<CTL_MAX_PWM_HISTORY;i++)
	{
		Ctl_pwm_hist.tbl[i] = 0;
	}
//	Ctl_pwm_ticks = 0;
//	Ctl_count_ticks = 0;
//	Ctl_start_rpm = Get_RPM();
//	txDebug("Ctl_ini_tbl\r\n");
}
//#endif
//----------------------------------------------------------------------------------
#ifdef MH_CTL_TUNE
long Ctl_rpm_change_rate(struct CTL_RPM_HIST_struct * p_rpm_hist)
{

	int latest_ix,earliest_ix;
	long latest_val,earliest_val;
	long delta_val;
	long change_rate;

	int head   = p_rpm_hist->head;

	latest_ix = (head - 1);
	if(latest_ix < 0) latest_ix += CTL_MAX_RPM_HISTORY;

	earliest_ix = (latest_ix - CTL_DELTA_X);
	if(earliest_ix < 0) earliest_ix += CTL_MAX_RPM_HISTORY;

	latest_val = p_rpm_hist->tbl[latest_ix];
	earliest_val = p_rpm_hist->tbl[earliest_ix];

	delta_val = earliest_val - latest_val;
	change_rate = (delta_val + CTL_DELTA_X/2) / CTL_DELTA_X;
#ifdef MH_DEBUG_CTL_TUNE
	sprintf(Ctl_pbuff,"latest_ix=%d,earliest_ix=%d,latest_val=%ld,earliest_val=%ld,delta_val=%ld,change_rate=%ld\r\n",
					latest_ix,earliest_ix,latest_val,earliest_val,delta_val,change_rate);
	txDebug(Ctl_pbuff);
#endif

	return change_rate;
}
#endif
//----------------------------------------------------------------------------------
// Same logic as change3 but not using a ring buffer any more. Lowest entry is latest value
// Same logic as calc4 except try and remove Ctl_rpm_per_spread factor and just produce
// a number that represents a multiple of rpm_per_tick

//#ifdef MH_XXX
void Ctl_calculate_projected_rpm5(void)
//void Ctl_rpm_history_change4(int idisplay)
{
	int i;
	int pwm_val,abs_pwm_val;
//   	long rpm_change,tot_rpm_change;
//   	long rpm_change_255,tot_rpm_change_255;
	long tick_weighting_value,verify_pipeline_tot_255;
	long current_tick_val;
	long verify_pipeline_tot;

//	int check_weighting_value;

	verify_pipeline_tot_255 = 0;
	tick_weighting_value = CTL_PWM_SPREAD_TICKS;		// eg 32


	for(i=0;i<CTL_PWM_HIST_TOT;i++)
	{
        pwm_val = Ctl_pwm_hist.tbl[i];

		abs_pwm_val = ABS(pwm_val);
        if(abs_pwm_val > CTL_MIN_PWM_VAL)
        {
			current_tick_val = (pwm_val * tick_weighting_value);
			verify_pipeline_tot_255 += current_tick_val;

			if(Ctl_display > 1)
			{
				sprintf(Ctl_pbuff,"  i:%2d,pwm_val:%5d,tick_weighting_value:%ld,current_tick_val:%ld,verify_pipeline_tot_255:%ld\r\n",
				i,pwm_val,tick_weighting_value,current_tick_val,verify_pipeline_tot_255);
				txDebug(Ctl_pbuff);
			}
        }

		if(i >= CTL_PWM_LAG_TICKS)
		{
//			tick_weighting_value -= Ctl_rpm_per_spread_tick;
			tick_weighting_value--;
		}
    }
	if(tick_weighting_value != 0)
	{
		sprintf(Ctl_pbuff,"Ctl_rpm_history_change5: tick_weighting_value:%d, should be zero\r\n",tick_weighting_value);
		txDebug(Ctl_pbuff);

// Possibly should pause?

	}

// Note: If this works we may be able to just remove rounding, it is not that important.

//	Ctl_pipeline_pwm_ticks = abs(verify_pipeline_tot_255/CTL_PWM_SPREAD_TICKS);
	verify_pipeline_tot = ((verify_pipeline_tot_255 * Ctl_rpm_per_tick_x10)/255);
	verify_pipeline_tot /= CTL_PWM_SPREAD_TICKS;		// Get rid of weighting.
	Ctl_verify_pipeline_tot5 = (int)(verify_pipeline_tot/10);		// because Ctl_rpm_per_tick_x10 has 1 dec place

#ifdef MH_CTL4
	if(Ctl_verify_pipeline_tot5 != Ctl_verify_pipeline_tot && Ctl_verify_pipeline_tot5 +1 != Ctl_verify_pipeline_tot)
	{
		sprintf(Ctl_pbuff,"Ctl_verify_pipeline_tot:%d,Ctl_verify_pipeline_tot5:%d\r\n",Ctl_verify_pipeline_tot,Ctl_verify_pipeline_tot5);
		txDebug(Ctl_pbuff);
	}
#endif

    Ctl_history_total_rpm_change = Ctl_verify_pipeline_tot5;
//    Ctl_calculate_rpm_per_tick_rate();

	if(Ctl_display>1)
	{
		sprintf(Ctl_pbuff,"Ctl_verify_pipeline_tot:%d\r\n",Ctl_verify_pipeline_tot5);
		txDebug(Ctl_pbuff);
	}
	if(Ctl_display)
	{
//		setMainTimerLimit(100);		// DEBUG!!!!
	}
}
//#endif
//----------------------------------------------------------------------------------
char *Ctl_fmt_dec(long ldec1)
{
	static char string[16];
	int n,d;
	n  = (int)(ldec1/10);
	d  = (int)ABS(ldec1%10);
	sprintf(string," %5d.%01d",n,d);
	return string;
}
//----------------------------------------------------------------------------------
void Ctl_report_rate_change(void)	// Called from main();
{
}
//----------------------------------------------------------------------------------
extern bool Drive_wait_flag;
extern WORD currentState;
extern MotorDriveState dState;
uint8_t Ctl_brushless_speed;

int BL_projected_rpm;
MotorDriveState BL_dstate;
int BL_dstate_ifrom;

void Set_BL_dState(MotorDriveState newstate,int ifrom)
{
	Set_dState(newstate,100);
//	BL_no_dstate_changes = true;
	BL_dstate = newstate;
	BL_dstate_ifrom=ifrom;
}

//int Motor_position(void);

#define MANUAL_SWITCH_BIT	1
extern int Prop_rpm_delta_per_tick_1dec;

uint16_t BL_control;

int16_t BL_rpm_latent_change;
int BL_wait_count;
int BL_delta_zero_count;
int BL_calculated_rpm_change_1dec;
int BL_incr;
int BL_count;
int BL_lastdir;
bool BL_wait_ini;
uint16_t BL_actualspeed_base;

/*
 * Note: We currently use param value BL_CONTROL for rpm_change per tick,but we could calculate dynamically, but make sure calculated value within (say) 30%
 * of param value.
 *
 *
 */

//#ifdef MH_XXX
#define BL_DRIVE_TBL_MAX	50

int16_t BL_drive_percent[BL_DRIVE_TBL_MAX];
//#endif
#define BL_RPM_TBL_MAX		5		// one tenth of a second
uint8_t BL_rpm_ix = 0;
int16_t BL_rpm_t[BL_RPM_TBL_MAX];	// Use this table to get gradient of RPM and change of gradient

//int BL_last_speed_gradient;

extern int Prop_rpm_delta_per_tick_1dec;
extern int Prop_rpm_1dec;
int Prop_rpm_delta_change_per_tick_1dec;
int BL_count;
bool BL_added_to_tank;
bool BL_delta_rpm_on=false;
typedef enum
{
    BL_IDLE = 0,
	BL_COARSE,
	BL_FINE,
	BL_WAIT,
} BL_CTL_STATE;
BL_CTL_STATE BL_ctl_state;
//uint32_t BL_timer_count=0;

int BL_scale_percent=100;
uint8_t Get_speed_from_ratio(int motor_speed_pct)
{
	int gb_ratio = Enc_data.cal_motor_ratio;
	gb_ratio = (gb_ratio + 50) / 100;	// Round to integer

	motor_speed_pct = (motor_speed_pct * gb_ratio + 85)/ 190;	// Because 86 ratio is base line, with speeds of 10, 30 and 100
	if(Enc_data.hub_software_version < 151)
	{
		motor_speed_pct = ((motor_speed_pct)/10) * 10;	// To closest 10%
		if(motor_speed_pct == 0) motor_speed_pct = 100;
	}
	else
	{
		if(motor_speed_pct < 100)	// full speed is still full speed
		{
			motor_speed_pct = (motor_speed_pct * BL_scale_percent) / 100;
		}
	}
	if(motor_speed_pct < 10) motor_speed_pct = 10;
	if(motor_speed_pct > 100) motor_speed_pct = 100;
	return motor_speed_pct;
}
uint8_t Get_BL_speed_from_pct(int motor_speed_pct)
{
	uint8_t bl_speed;
	if(Enc_data.hub_software_version < 151)
	{
		bl_speed = (motor_speed_pct/10) + '0';
	}
	else
	{
		bl_speed = 128 + motor_speed_pct;
	}
	return bl_speed;
}
int16_t Engine_rpm_change_in_five_ticks;

#define SPEED_FAST_PCT		100
#define SPEED_MEDIUM_PCT	67
#define SPEED_SLOW_PCT		22

void Brushless_control(void)	// See if controlling speed helps
{
	int i;

//	Set_cState(C_BL_CONTROL,200);
	Set_cState(C_IDLE,201);		// MHH:30/01/2026

	WORD ac200_switches = p_GetAC200_Switches(false);	// This check because had situation where it was in manual mode but still doing automatic control!
	if(ac200_switches & MANUAL_SWITCH_BIT)
	{
// Can happen legitimately as takes a number of ticks to recognise manual.
		//		L2PRINTF("Brushless_control:MANUAL switch bit on!!!!\r\n");
		BL_ctl_state = BL_IDLE;
//        Set_BL_dState(MD_IDLE,10);	// MHH:30/01/2026
        return;
	}


#ifdef MH_XXX
	if(BL_timer_count != (TimerCount - 1))		// In case went into MANUAL mode
	{
		DPRINTF("!!BL_timer_count:%d\r\n",TimerCount-BL_timer_count-1);
		BL_count = 0;
	}
	BL_timer_count = TimerCount;
#endif

	int16_t drive_percent;
	int total_percent = 0;

	int engine_rpm_per_10tick = ps.parms[CTL_RPM_PER_TICK_X10];		// MHH:12/02/2025. rpm per tick * 10;
	//	int engine_rpm_per_10tick = ps.parms[CTL_RPM_PER_TICK_X10];		// MHH:12/02/2025. rpm per tick * 10

	BL_scale_percent = ps.parms[BL_SCALE_PCT];

	int estimated_engine_change_in_tank = 0;

	#define DEPRECIATE_PERCENT	5

	if(engine_rpm_per_10tick)
	{
		for(i=0;i<BL_DRIVE_TBL_MAX;i++)
		{
			drive_percent = BL_drive_percent[i];
			if(drive_percent)
			{
				total_percent += drive_percent;
				if(i >= 10)		// Depreciate table element?
				{
					if(drive_percent > 0)
					{
						drive_percent = (drive_percent * 96)/100;
					}
					else
					{
						drive_percent = (drive_percent * 96)/100;
					}
					BL_drive_percent[i] = drive_percent;
				}
			}
		}
		estimated_engine_change_in_tank = (total_percent * engine_rpm_per_10tick) / 10000;

		for(i=BL_DRIVE_TBL_MAX-1;i>0;i--)	// Shuffle all entries to right
		{
			BL_drive_percent[i] = BL_drive_percent[i-1];
		}
		BL_drive_percent[0] = 0;		// default
	}


//	int scaling_factor_percent = getParameter(SF_MAG_SPEED);

/*
 * Simulator, when set up for Rotax 915/916 and Hub with GR of 86, and Rotax GR of 2.54 has following statistics:
 * Moves 3.3 degrees in 23 Ticks, which is 0.14 degrees/Tick, or 7.7 degrees/sec. This was Manual control, so think BL=5000 rpm
 * Changes engine rpm at about 1600/sec, or 32/Tick, so BL_control = 320
 * Changes prop rpm at about 630/sec, or 13 prop rpm/Tick at full speed (5000 rpm)
 * So, 0.1 degrees of prop is about same as 10 prop rpm, or in 915 case 25.4 rpm
 *
 * Of course, we should be able to move BL motor as slowly as 500 rpm, or 0.014 degrees/tick.
 */
//	BL_control = 320;	// MHH:27/01/2025. Test!!

//	int prop_rpm_per_10tick = (engine_rpm_per_10tick * 100) / scaling_factor_percent;

//	int prop_setspeed = (Ctl_setspeed * 100) / scaling_factor_percent;
//	int prop_deadband = (Ctl_deadband * 100) / scaling_factor_percent;
//	int prop_adiff = abs(prop_setspeed - prop_deadband);
#ifdef MH_XXX
	if(prop_adiff < prop_deadband + 20)
	{
		if(abs(total_percent) < 2000)	// MHH:10/08/2025. Hopefully will help with fine positioning.
		{
			total_percent = 0;
		}
	}Prop_rpm_change_in_five_ticks
#endif


//	int prop_rpm = Prop_rpm_1dec / 10;
	int engine_rpm = Engine_rpm_1dec / 10;
	int16_t bl_oldest_rpm =  BL_rpm_t[BL_rpm_ix];
//	BL_rpm_t[BL_rpm_ix++] = prop_rpm;
	BL_rpm_t[BL_rpm_ix++] = engine_rpm;
	BL_rpm_ix %= BL_RPM_TBL_MAX;
//	int16_t prop_rpm_change_in_ten_ticks  = 0;
	Engine_rpm_change_in_five_ticks = (engine_rpm - bl_oldest_rpm);
	if(Engine_rpm_change_in_five_ticks > 120) Engine_rpm_change_in_five_ticks = 120;		// Have to fit inside a byte for logging
	if(Engine_rpm_change_in_five_ticks < -120) Engine_rpm_change_in_five_ticks = -120;		// Have to fit inside a byte for logging


//#define SPEED_FAST		10
//#define SPEED_MEDIUM	3
//#define SPEED_SLOW		1

	if(BL_delta_angle >= (-0.2F))			// Has blade been getting finer
	{
		int rpm_delta_change_limit = 50;
		if(BL_delta_rpm_on) rpm_delta_change_limit = 10;

		if(Engine_rpm_change_in_five_ticks >= rpm_delta_change_limit)		// MHH:30/01/2026
		{
			int rpm_margin = 200;
//			if(Engine_rpm_change_in_five_ticks >= 100) rpm_margin = 400;		// MHH:02/02/2026
			if(Engine_rpm_change_in_five_ticks >= 100) rpm_margin = 1000;		// MHH:05/02/2026
			if(BL_delta_rpm_on) rpm_margin = 1000;
			if(engine_rpm >= (Ctl_setspeed - rpm_margin)) 	// Ignore?
			{
				BL_delta_rpm_on = true;
				Ctl_brushless_speed = 0;		// No, maximum coarse
				Set_cState(C_COARSER,205);		// MHH:30/01/2026
				BL_ctl_state = BL_COARSE;		// Not sure this is needed.
				return;							// MHH: 02/02/2026.
			}
	//		return;		// OK, lets go maximum coarse then.
		}
	}
	BL_delta_rpm_on = false;


	int overspeed = ps.parms[BL_CONTROL];
	if(overspeed < 2000 || overspeed > 9000) overspeed = 0;
	if(overspeed != 0)
	{
		if((overspeed - engine_rpm) <= 200)
		{
			int motor_speed_pct = 0;
			int sw_option = (Ctl_setspeed - engine_rpm + 100) / 20;	// MHH:20/02/2026  So if engine_rpm == setspeed then sw_option = 100/20 = 5
			//			int sw_option = rpm_to_overspeed / 20;
			if(sw_option < 0) sw_option = 0;
			switch(sw_option)
			{
			case 0:		// 0-19		(5800 - 5781)
			case 1:		// 20-39 	(5780 - 5761)
			case 2:		// 40-59	(5760 - 5741) (Start of dead band)
				if(Engine_rpm_change_in_five_ticks >= -10) motor_speed_pct = SPEED_MEDIUM_PCT;
				if(Engine_rpm_change_in_five_ticks >= 0) motor_speed_pct = SPEED_FAST_PCT;
				break;

			case 3:		// 60-79	(5740 - 5721)
			case 4:		// 80-99 	(5720 - 5701)
				if(Engine_rpm_change_in_five_ticks >= 0) motor_speed_pct = SPEED_SLOW_PCT;
				if(Engine_rpm_change_in_five_ticks >= 5) motor_speed_pct = SPEED_MEDIUM_PCT;
				if(Engine_rpm_change_in_five_ticks >= 20) motor_speed_pct = SPEED_FAST_PCT;
				break;

			case 5:		// 100-119	(5700 - 5681) (On or below setspeed)
			case 6:		// 120-139	(5680 - 5661)
			case 7:		// 140-159	(5660 - 5641) (up to deadband)
//				if(Engine_rpm_change_in_five_ticks >= 10) motor_speed_pct = SPEED_SLOW_PCT;
				if(Engine_rpm_change_in_five_ticks >= 20) motor_speed_pct = SPEED_MEDIUM_PCT;
				if(Engine_rpm_change_in_five_ticks >= 30) motor_speed_pct = SPEED_FAST_PCT;

			default:	// 160-200	(5640 - 5600) (Below deadband)
				if(BL_delta_angle >= (-0.2F))
				{
//					if(Engine_rpm_change_in_five_ticks >= 10) motor_speed_pct = SPEED_SLOW_PCT;
					if(Engine_rpm_change_in_five_ticks >= 20) motor_speed_pct = SPEED_MEDIUM_PCT;
					if(Engine_rpm_change_in_five_ticks >= 30) motor_speed_pct = SPEED_FAST_PCT;
				}
				else
				{
					if(BL_delta_angle < (-0.5F))
					{
						if(Engine_rpm_change_in_five_ticks >= 50) motor_speed_pct = SPEED_MEDIUM_PCT;
					}
					else
					{
						if(Engine_rpm_change_in_five_ticks >= 40) motor_speed_pct = SPEED_MEDIUM_PCT;
					}
				}
			}
			if(motor_speed_pct != 0)
			{
//				Ctl_brushless_speed = Get_speed_from_ratio(motor_speed_pct);
				motor_speed_pct = Get_speed_from_ratio(motor_speed_pct);	// MHH:11/03/2026
//				Ctl_brushless_speed = motor_speed_pct + '0';
				Ctl_brushless_speed = Get_BL_speed_from_pct(motor_speed_pct);
				Set_cState(C_COARSER,205);		// MHH:30/01/2026
				BL_ctl_state = BL_COARSE;		// Not sure this is needed.
				return;							// MHH: 02/02/2026.
			}
		}
	}


	int estimated_engine_change_rpm = 0;

	/*
	 * MHH:10/02/2025. Only calculate estimated_prop_change_rpm if delta is >= than 1.8. A prop RPM change of about 100 rpm/sec = 2.0 delta value
	 *
	 */

// Example is Sling where RPM is increasing at about 62 RPM per ten ticks, or 620 with an implied decimal place.
// Dividing by 10 = 62, by 5 = 124
// prop_rpm at 5700 for Rotax 916 is 2244 rpm, and 3.5% of 2244 = about 80 rpm, and when error >= 3.5% pitch control speed = full speed
// Note that at gear ratio of 2.54 that 100 engine rpm = about 40 prop rpm, and 60 engine rpm about 24 prop rpm
/*
 *  Rotax 916 RPM	Prop RPM	% of 5700
 *  ----------------------------------------
 *  5800			2283
 *  5700			2244
 *  254				100			4.5%
 *  200				79			3.5%
 *  100				39			1.8%
 *  60				24			1.1%
 *
 */

//	int estimated_prop_change_rpm = prop_change_rpm_1dec / 10;
//	int estimated_prop_change_rpm = 0;	// Test!!

//	int prop_rpm = Prop_rpm_1dec / 10;
	int estimated_engine_rpm2 = engine_rpm + estimated_engine_change_rpm + estimated_engine_change_in_tank;

	int engine_difference = Ctl_setspeed - estimated_engine_rpm2;
	int engine_adiff = abs(engine_difference);

	BL_count++;

	if(estimated_engine_change_rpm)
	{
//		DPRINTF("%d S=%d,A=%d,E1=%d,E2=%d\r\n",BL_count,Ctl_setspeed,prop_rpm,estimated_engine_change_rpm,estimated_engine_change_in_tank);

	}
	int prop_deadband2 = Ctl_deadband;

	if(BL_ctl_state != BL_IDLE)		// MHH:13/08/2025. If not idle then 3/4 deadband to force
	{
		prop_deadband2 = (prop_deadband2 * 3) / 4;
	}
	if(engine_adiff < prop_deadband2)
	{
#ifdef MH_XX1	// MHH:30/01/2026
		BL_ctl_state = BL_IDLE;
	    Set_BL_dState(MD_IDLE,10);
#endif
	    return;
	}



#ifdef MH_XXX
	if(prop_rpm < (Ctl_setspeed - prop_deadband))	// MHH:18/12/2025. Try and capture situation where throttle is increasing rpm rapidly, but rpm < setspeed
	{
		if(prop_rpm_change_in_ten_ticks >= 100)
		{
#ifdef MH_XX1	// MHH:30/01/2026
			BL_ctl_state = BL_IDLE;
		    Set_BL_dState(MD_IDLE,10);
#endif
		    return;
		}
	}
#endif

/*
 * MHH:08/10/2025
 */
//#define FAST_LIMIT		260
//#define MEDIUM_LIMIT	150
//#define SLOW_LIMIT		80
	int slow_limit = 0;
	int fast_limit;
	int scaling_factor_percent = getParameter(SF_MAG_SPEED);
	int speed_per_diff=0;

	if(Enc_data.hub_software_version < 151)
	{
		if(scaling_factor_percent >= 200)
		{
			slow_limit = 160;
	//		medium_limit = 200;
			fast_limit = 400;
		}
		else
		{
			slow_limit = 80;
			fast_limit = 260;
		}
	}
	else
	{
		fast_limit = 260;
		speed_per_diff = 4;
		if(scaling_factor_percent >= 200)
		{
			fast_limit = 400;
			speed_per_diff = 6;
		}
	}


	int motor_speed_pct = SPEED_FAST_PCT;		// default
	bool speed_set = false;
#ifdef MH_XXX	// MHH:30/01/2026
	if((operatingMode () == TAKEOFF) && (prop_rpm > Ctl_setspeed))
	{
		if(prop_rpm_change_in_ten_ticks >= 100)		// Arbitrary start point, may need adjusting.Only interested in increasing RPM.
		{
			speed_set = true;
			if(engine_adiff < prop_deadband) motor_speed_pct = SPEED_MEDIUM;	// If larger than motor_speed_pct = 10
		}
		else
		{
			if(prop_rpm_change_in_ten_ticks >= 50)
			{
				if(prop_rpm > (Ctl_setspeed + 50))
				{
					speed_set = true;		// defaults to full speed
				}
			}
		}
	}
#endif
	if(Engine_rpm_change_in_five_ticks >= 25)
	{
		if(engine_rpm > (Ctl_setspeed + 50))
		{
			speed_set = true;		// defaults to full speed
		}
	}
	if(speed_set == false)
	{
		if(engine_adiff < fast_limit)
		{
			if(Enc_data.hub_software_version < 151)
			{
				motor_speed_pct = SPEED_MEDIUM_PCT;
				if(engine_adiff < slow_limit) motor_speed_pct = SPEED_SLOW_PCT;
			}
			else
			{
				motor_speed_pct = engine_adiff / speed_per_diff;
			}
//			Ctl_brushless_speed = Get_speed_from_ratio(motor_speed_pct);
		}
	}

	motor_speed_pct = Get_speed_from_ratio(motor_speed_pct);
//	Ctl_brushless_speed = motor_speed_pct + '0';
	Ctl_brushless_speed = Get_BL_speed_from_pct(motor_speed_pct);

	#ifdef MH_XXX	// MHH:11/03/202
	Ctl_brushless_speed = motor_speed_pct;
	if(Ctl_brushless_speed == 10)
	{
		Ctl_brushless_speed = 0;
	}
	else
	{
		Ctl_brushless_speed += '0';
	}
#endif
//	drive_percent = motor_speed_pct * 100;		// as full speed = 10 we get 1000. Min speed = 1, get 100
	drive_percent = motor_speed_pct * 10;		// as full speed = 100 we get 1000. Min speed = 10, get 100
	if(engine_difference > 0)		// Do we need to increase RPM?
	{

//		Set_BL_dState(MD_FINER,300);
		Set_cState(C_FINER,202);		// MHH:30/01/2026
		if((currentState & S_STOP_FINE) == 0)
		{
			BL_ctl_state = BL_FINE;
			BL_drive_percent[0] = drive_percent;
			BL_added_to_tank = true;
//			DPRINTF("I:F,S=%d,A=%d,D=%d\r\n",Ctl_setspeed,Ctl_actualspeed,drive_percent);
		}
	}
	else
	{
		Set_cState(C_COARSER,203);		// MHH:30/01/2026
//		Set_BL_dState(MD_COARSER,200);
		if((currentState & S_STOP_COARSE) == 0)
		{
			BL_ctl_state = BL_COARSE;
			BL_drive_percent[0] = -drive_percent;
			BL_added_to_tank = true;
//			DPRINTF("I:C,S=%d,A=%d,D=%d\r\n",Ctl_setspeed,Ctl_actualspeed,-drive_percent);
		}
	}
}


//#endif
ControlState Ctl_cstate;
WORD MH_last_setspeed;
//WORD Ctl_pwm_slow_count;
WORD Ctl_move_ticks;
ControlState Ctl_slow_cstate;

extern int Engine_rpm_delta_per_tick_1dec;
extern int Engine_rpm_1dec;
extern int Engine_rpm_1dec_prev;
extern int Engine_rpm_1dec_prev1;
extern int Engine_rpm_delta_1dec;

extern WORD Softmode_speed_val;		// MHH:27/05/2024.
extern WORD Softmode_count;			// MHH:29/05/2024

extern uint8_t Soft_mode;
#define SOFTMODE_NONE		0
#define SOFTMODE_WAITING_SPEED	1
#define SOFTMODE_STARTING	2
#define SOFTMODE_BRAKING	3

#ifdef MH_YYY
void Ctl_logic(void)
{
//	int projected_rpm,projected_err,abs_projected_err;
	int projected_rpm,projected_err;	// MHH:23/11/2020

	int deadband;
//	int rpm_tolerance;

//	BYTE t = Trace("Ctl_logic");

// Note: may need to take action if set speed changes and not in state zero.

// So if Ctl_rpm_per_tick = 16, and CTL_PWM_SPREAD_TICKS = 32, then we get (16 * 1000)/32 = 500 (3 decimal places)

// OK, we still need to think of the best way to use the values Ctl_estimated_current-rpm and Ctl_estimated_finish_rpm.
// At this stage they are not combined with actual_rpm, so not really given a chance to compare with reality.
// Perhaps we need to consider change? Eg, what if we saved all the projected rpm values in a table, then we could compare
// expected change with actual. Another approach is to have a value which reflects how much change the current pwm history
// holds. eg, add to it every time we have a power tick, and subtract from it how much is 'used up' in that tick at same time.
// The 'used up' portion is calculated from the Ctl_rpm_history_change function.

// OK, so we now have a value for what we think that the RPM change will be from the recent history of pwm changes, called
// Ctl_history_total_rpm_change. So we can add that to the current actual speed and get a projected speed.
// We should also be able to get a projected time for that speed, by knowing the oldest power tick and adding lag + spread to it.

// As far as having a set of estimated RPM speeds, we could compare that to the set of actual speeds to see if changes are more
// or less than expected. We would need to remember last calculated values for (say) up to a history of 10 (fifth of second).
// We could use the difference in rates to calculate a value to include when trying to work out what rpm will be at end of
// time period.

// This means that we may only need to store the same number of values of actual rpm, as we are really only interested in
// the value of change compared to expected change.

// So, logic might be something like:

/*
		Calculated expected_change_rpm over last 5 ticks
		Calculate actual_change_rpm over last 5 ticks.

		Calculate expected end rpm given pwm history.
		Adjust that value with any changes between expected_rpm and actual, taking into account time value returned from history check

		if expected_end_rpm less than set_rpm by more than tolerance
			do a power_tick towards target
			return
		end if

		if expected_end_rpm greater than target by more than tolerance then
			do a power tick towards target.
			return
		endif

*/
//    Ctl_rpm_per_spread_tick = (Ctl_rpm_per_tick_x10 * CTL_100) / CTL_PWM_SPREAD_TICKS;

//	Ctl_setspeed = (int)(setSpeed/RPMFACTOR);
//	Ctl_actualspeed = (int)(actualSpeed/RPMFACTOR);

    OpMode mode = operatingMode();
    if(mode == MANUAL)	// MHH:17/05/2023
    {
        Set_cState(C_IDLE,300);
        Set_dState(MD_IDLE,200);
//        currentControl = 0;	// Fasttrack
        Set_current_control(0,30);
//        BL_ctl_state = BL_IDLE;
        Ctl_move_ticks = 0;
        return;
    }


	Ctl_setspeed = (int)(setSpeed);
	Ctl_actualspeed = (int)(actualSpeed);


    deadband = getParameter (PI_DEAD_BAND);
    int deadband2 = getParameter(PI_DEAD_BAND2);	// MHH:28/03/2023
    if(deadband2 != 0)
    {
    	int pi_lo_rpm = getParameter(PI_LO_RPM);
    	int pi_hi_rpm = getParameter(PI_HI_RPM);
    	if(Ctl_setspeed > pi_lo_rpm && Ctl_setspeed < pi_hi_rpm)
    	{
    		deadband = deadband2;
    	}
    }
    if (mode == HOLD)
    {
    	if (isSlaveProp())
        {
    		if(isSlaveCommsActive())
    		{
//    			WORD slave_deadband = getParameter(CT_SLAVE_DEADBAND);

    			WORD slave_deadband = getSlaveDeadband();	// MHH:30/11/2020
    			slave_deadband = MIN(slave_deadband,deadband);

    			slave_deadband = MAX(slave_deadband,10);
    			deadband = slave_deadband;
    		}
        }
    }

	Ctl_deadband = deadband;		// This used for logging
    if(ps.parms[BL_ENABLED] == 1)
    {
    	Brushless_control();
        Set_current_control(0,40);
//    	currentControl = 0;	// To be sure...
		return;
    }



//    currentControl = 	getParameter (MC_MAX_PWM);	// default MHH:09/08/2018
//    currentControl = 255;		// MHH:30/01/2024
//    Set_current_control(255,50);	// MHH:22/05/2024

//    if(Drive_wait_flag == false)	// MHH:02/11/2018. FINE LED light was flashing Orange as it tried to go FINE when FINE STOP on.
// MHH:09/05/2024. Remove this test because it is possible that the current "bump" bug cause this.



	/*
	 *  MHH:10/05/2024. Because have added 'D' factor, look at putting a maximum on history change. Perhaps NCV/2 max?
	 */

#define MH_PD_ONLY
#ifdef MH_PD_ONLY		// MHH:18/05/2024
	projected_rpm = Ctl_actualspeed;
#endif


// Note we will need to convert Prop_rpm_delta_per_tick to engine rpm...
#ifdef MH_XXX
    if(abs(Prop_rpm_delta_per_tick) >1)
    {
        DPRINTF("A:%d,P:%d,D=%d,U=%d,r=%d,p=%d\r\n",Ctl_actualspeed,projected_rpm,Prop_rpm_delta_per_tick,Prop_rpm_delta_usecs,Prop_rpm_delta,Prop_rpm_delta_prev);
    }
#endif
/*
 * Note: This PD control could also be done using just prop rpm, not engine rpm, with limits converted to prop rpm limits
 */

    int delta_factor = 0;
	int ncv = getParameter(CTL_RPM_PER_TICK_X10);
	if(ncv < 1000) ncv = 1025;		// default
	int delta_weighting = ncv % 1000;
	int ncv_speed_option = ncv/1000;
	if(ncv_speed_option > 3) ncv_speed_option = 1;		// to be sure...

    if(abs(Engine_rpm_delta_per_tick_1dec) >= 20)
    {
    	//e.g. 1050 would mean 50% delta weighting. 1025 = 25% etc.
    	delta_factor = Engine_rpm_delta_per_tick_1dec * delta_weighting;	// eg * 25 / 100 = divide by 4
    	delta_factor /=100;			// finish weighting delta
        projected_rpm += delta_factor;
    }
	projected_err = Ctl_setspeed - projected_rpm;

//#define MH_PD_CTL_DEBUG
#ifdef MH_PD_CTL_DEBUG
//	if(abs(projected_err) >= deadband || cState != C_IDLE)
	{
        int millisecs = LogData.log_ticks * 20;
        int secs = millisecs/1000;
        int minutes = secs/60;
        secs %= 60;
        int ticks = LogData.log_ticks % 50;
        char dir = ' ';
        if(abs(projected_err) >= deadband)
        {
        	if(projected_err > 0)
        	{
        		dir = '-';
        	}
        	else
        	{
        		dir = '+';
        	}
        }
        DPRINTF("%d:%02d:%02d T:%d,A:%d,P=%d,D=%d,df=%d,PE=%d,%c\r\n",minutes,secs,ticks,Ctl_setspeed,
//        		Ctl_actualspeed,Prop_int_rpm_1dec,Engine_rpm_1dec,Engine_rpm_1dec_prev,Engine_rpm_delta_per_tick_1dec,
        		Ctl_actualspeed,projected_rpm,Engine_rpm_delta_per_tick_1dec,
        		delta_factor,projected_err,dir);
/*
 * int Engine_rpm_delta_per_tick_1dec;
int Engine_rpm_1dec;
int Engine_rpm_1dec_prev;
int Engine_rpm_1dec_prev1;
int Engine_rpm_delta_1dec;
 *
 */


        DPRINTF("    rpm=%d,rpm1=%d,rpm2=%d,rpmD=%d,rpmDt=%d\r\n",
        		Engine_rpm_1dec,Engine_rpm_1dec_prev,Engine_rpm_1dec_prev1,Engine_rpm_delta_1dec,Engine_rpm_delta_per_tick_1dec);

	}
#endif



	if(abs(projected_err) < deadband)
	{
		Set_cState(C_IDLE,400);
//        currentControl = 0;	// Fasttrack
        Set_current_control(0,60);
        Ctl_move_ticks = 0;
		return;
	}
	int pwm = 255;
	Softmode_speed_val = 1;			// Default;

/*
 *  ncv hi digit	max_count
 *  1				none
 *  2				9/2 = 4
 *  3				9/3 = 3
 *  4				9/4 = 2
 */

#ifdef MH_XXX				// MHH:14/01/2026
	if(Soft_mode == SOFTMODE_STARTING)
	{
		if(ncv_speed_option > 1)
		{
			int softmode_max_count = 9/ ncv_speed_option;
			if(Softmode_count > softmode_max_count)
			{
				if(abs(projected_err) < (Ctl_setspeed * 3)/100)	// projected_err < 3% setspeed?
				{
					Set_cState(C_IDLE,410);
			//        currentControl = 0;	// Fasttrack
			        Set_current_control(0,60);
			        Ctl_move_ticks = 0;
					return;
				}
			}
		}
	}
#endif
#ifdef MH_ZZZ

	if(ncv_speed_option > 1)		// Reduce speed?
	{
		if(abs(projected_err) < (Ctl_setspeed * 3)/100)	// projected_err < 3% setspeed?
//		if(abs(projected_err) < (Ctl_setspeed * 2)/100)	// projected_err < 2% setspeed?
		{
			Softmode_speed_val = ncv_speed_option;
		}
	}
#endif
	/*
	switch(Ctl_move_ticks++)
	{
	case 0:
		pwm = 100;
		break;
	case 1:
		pwm = 200;
		break;
	case 3:
		pwm = 255;
		break;
	case 4:
		pwm = 255;
		break;
	default:
		if(reduce_speed_when_close)
		{
			if((Ctl_move_ticks & 1) == 0)
			{
				pwm = 0;
			}
		}
	}
*/

//	int reduce_speed_zone_high = Ctl_setspeed + 200;
//	int reduce_speed_zone_low = Ctl_setspeed - deadband - one_pct_setspeed;
//#define NCV				200
//#define TARGET_NCV_SLOW		50
//#define NCV_SLOW_FACTOR		(NCV/TARGET_NCV_SLOW)
/*
 *  Note: We could add an extra parameter which governs speed for fast pitch change. It could be the pwm slow setting
 *  e.g. Could have a parameter in format abbb where a = % of max speed/10, and bbb = slow zone (say 200)
 *  Or, could be 2 parameters.
 */

#ifdef MH_YYY

#define PWM_SLOW		(64)
	if(reduce_speed_when_close)
	{
#ifdef MH_XXX

		if(abs(Ctl_setspeed - Ctl_actualspeed) < 200)		// arbitrary
		{
			if(Ctl_actualspeed > Ctl_setspeed)
			{
				if(delta_factor < 0)
				{
					pwm = PWM_SLOW;
	//				DPRINTF("Hi:Slow\r\n");
				}
			}
			else
			{
				if(delta_factor > 0)
				{
					pwm = PWM_SLOW;
	//				DPRINTF("Lo:Slow\r\n");
				}
			}
		}
		if(abs(Ctl_setspeed - Ctl_actualspeed) < 100)		// arbitrary
		{
			pwm = PWM_SLOW;
		}
#else
		if(cState != Ctl_slow_cstate)
		{
			Ctl_slow_cstate = cState;
			Ctl_pwm_slow_count = 0;
		}


		if(abs(Ctl_setspeed - Ctl_actualspeed) < 200)		// arbitrary
		{
			if(Ctl_pwm_slow_count++ > 0)
			{
				pwm = PWM_SLOW;
			}
		}
		else
		{
			Ctl_pwm_slow_count = 0;
		}
#endif
	}
#endif
//	currentControl = pwm;	// MHH:21/05/2024

    Set_current_control(pwm,70);
	//	currentControl = 255;	// default;
//	currentControl = 	getParameter (MC_MAX_PWM);	// default MHH:09/08/2018
#ifdef MH_PD_ONLY
	if(projected_err > 0)
	{
		Set_cState(C_FINER,500);
	}
	else
	{
		Set_cState(C_COARSER,600);
	}
#endif
}
#else
void Ctl_logic(void)
{
//	int projected_rpm,projected_err,abs_projected_err;
	int projected_rpm,projected_err;	// MHH:23/11/2020

	int deadband;
	int rpm_tolerance;

//	BYTE t = Trace("Ctl_logic");

// Note: may need to take action if set speed changes and not in state zero.

// So if Ctl_rpm_per_tick = 16, and CTL_PWM_SPREAD_TICKS = 32, then we get (16 * 1000)/32 = 500 (3 decimal places)

// OK, we still need to think of the best way to use the values Ctl_estimated_current-rpm and Ctl_estimated_finish_rpm.
// At this stage they are not combined with actual_rpm, so not really given a chance to compare with reality.
// Perhaps we need to consider change? Eg, what if we saved all the projected rpm values in a table, then we could compare
// expected change with actual. Another approach is to have a value which reflects how much change the current pwm history
// holds. eg, add to it every time we have a power tick, and subtract from it how much is 'used up' in that tick at same time.
// The 'used up' portion is calculated from the Ctl_rpm_history_change function.

// OK, so we now have a value for what we think that the RPM change will be from the recent history of pwm changes, called
// Ctl_history_total_rpm_change. So we can add that to the current actual speed and get a projected speed.
// We should also be able to get a projected time for that speed, by knowing the oldest power tick and adding lag + spread to it.

// As far as having a set of estimated RPM speeds, we could compare that to the set of actual speeds to see if changes are more
// or less than expected. We would need to remember last calculated values for (say) up to a history of 10 (fifth of second).
// We could use the difference in rates to calculate a value to include when trying to work out what rpm will be at end of
// time period.

// This means that we may only need to store the same number of values of actual rpm, as we are really only interested in
// the value of change compared to expected change.

// So, logic might be something like:

/*
		Calculated expected_change_rpm over last 5 ticks
		Calculate actual_change_rpm over last 5 ticks.

		Calculate expected end rpm given pwm history.
		Adjust that value with any changes between expected_rpm and actual, taking into account time value returned from history check

		if expected_end_rpm less than set_rpm by more than tolerance
			do a power_tick towards target
			return
		end if

		if expected_end_rpm greater than target by more than tolerance then
			do a power tick towards target.
			return
		endif

*/
//    Ctl_rpm_per_spread_tick = (Ctl_rpm_per_tick_x10 * CTL_100) / CTL_PWM_SPREAD_TICKS;

//	Ctl_setspeed = (int)(setSpeed/RPMFACTOR);
//	Ctl_actualspeed = (int)(actualSpeed/RPMFACTOR);

	Ctl_setspeed = (int)(setSpeed);
	Ctl_actualspeed = (int)(actualSpeed);


//	Ctl_actualspeed = 6000;	// Debug!!!!


//	Ctl_err = Ctl_setspeed - Ctl_actualspeed;	// Think they are held in tenths of a RPM

//    lastErrorValue = setSpeed - actualSpeed;	// Just in case we need it

    deadband = getParameter (PI_DEAD_BAND);
    int deadband2 = getParameter(PI_DEAD_BAND2);	// MHH:28/03/2023
    if(deadband2 != 0)
    {
    	int pi_lo_rpm = getParameter(PI_LO_RPM);
    	int pi_hi_rpm = getParameter(PI_HI_RPM);
    	if(Ctl_setspeed > pi_lo_rpm && Ctl_setspeed < pi_hi_rpm)
    	{
    		deadband = deadband2;
    	}
    }
    OpMode mode = operatingMode();
    if (mode == HOLD)
    {
    	if (isSlaveProp())
        {
    		if(isSlaveCommsActive())
    		{
//    			WORD slave_deadband = getParameter(CT_SLAVE_DEADBAND);

    			WORD slave_deadband = getSlaveDeadband();	// MHH:30/11/2020
    			slave_deadband = MIN(slave_deadband,deadband);

    			slave_deadband = MAX(slave_deadband,10);
    			deadband = slave_deadband;
    		}
        }
    }
	currentControl = 	getParameter (MC_MAX_PWM);	// default MHH:09/08/2018

	Ctl_deadband = deadband;		// This used for logging
    if(ps.parms[BL_ENABLED] == 1)
    {
    	Brushless_control();
        Set_current_control(0,40);
		return;
    }

    if(Drive_wait_flag == false)	// MHH:02/11/2018. FINE LED light was flashing Orange as it tried to go FINE when FINE STOP on.
    {
    	switch(dState)		// Maybe dState?
    	{
    	case MD_FINER:
    	    if(currentState & S_STOP_FINE)
    	    {
    	    	if(Ctl_actualspeed < Ctl_setspeed - deadband)
    	    	{
//    	    		TraceReturn(t,"S_STOP_FINE return");
    	    		return;
    	    	}
    	    }
    		break;

    	case MD_COARSER:
    	    if(currentState & S_STOP_COARSE)
    	    {
    	    	if(Ctl_actualspeed > Ctl_setspeed + deadband)
    	    	{
//    	    		TraceReturn(t,"S_STOP_COARSE return");
    	    		return;
    	    	}
    	    }
    		break;
    	default:
    		break;
    	}
    }

	if(cState == C_IDLE)
	{
//		Ctl_ini_tbl();		// Test if helps
	}
	else
	{
		Ctl_table_cleared = false;
	}
	Ctl_update_history4();
	Ctl_calculate_projected_rpm5();


#ifdef MH_EXTRA_CTL_LOGIC
	act_rpm_rate_change = Ctl_rpm_change_rate(&Ctl_rpm_hist);
	est_rpm_rate_change = Ctl_rpm_change_rate(&Ctl_estrpm_hist);
	diff_rpm_rate_change = act_rpm_rate_change - est_rpm_rate_change;

	est_finish_ticks = Ctl_last_pwm_tick;

	if(Ctl_last_pwm_tick == 0)		// Nothing in pipeline? Should be same as Ctl_history_total_rpm_change == 0
	{
		if(Ctl_history_total_rpm_change != 0)
		{
			txDebug("Ctl_logic: Ctl_last_pwm_tick == 0, Ctl_history_total_rpm_change != 0\r\n");
		}
	}
#endif


    projected_rpm = Ctl_actualspeed + Ctl_history_total_rpm_change;
	projected_err = Ctl_setspeed - projected_rpm;
	if(Ctl_display && (Ctl_history_total_rpm_change != 0))
	{
		sprintf(Ctl_pbuff,"Ctl_logic:Ctl_actualspeed:%5d,Ctl_setspeed:%5d,Ctl_history_change:%d,projected_rpm:%5d,projected_err:%5d\r\n",
			Ctl_actualspeed,Ctl_setspeed,Ctl_history_total_rpm_change,projected_rpm,projected_err);
		txDebug(Ctl_pbuff);
	}

// Note: Could take into account the difference in current rate of change between actual and estimated here

//#define MH_CTL_DEBUG
#ifdef MH_CTL_DEBUG
	sprintf(Ctl_pbuff,"%d,%d,%d,%d,%d,%d\r\n",
			Ctl_pwm_val,Ctl_actualspeed,Ctl_setspeed,Ctl_history_total_rpm_change,projected_rpm,projected_err);
	if(strcmp(Ctl_pbuff,Ctl_pbuff2))
	{
		strcpy(Ctl_pbuff2,Ctl_pbuff);
		txDebug(Ctl_pbuff);
	}
#endif



	rpm_tolerance = deadband;
	int rpm_min_ok;
	int rpm_max_ok;

#define MH_TRY_NEW_IDEA
#ifdef MH_TRY_NEW_IDEA
	switch(cState)
	{
	default:
		rpm_min_ok = Ctl_setspeed - rpm_tolerance;
		rpm_max_ok = Ctl_setspeed + rpm_tolerance;
		break;

	case C_FINER:
		rpm_min_ok = Ctl_setspeed - (rpm_tolerance/2);
		rpm_max_ok = Ctl_setspeed + rpm_tolerance;
/*
		if(projected_err <= 0 || (projected_err < (rpm_tolerance/2)))
		{
	        Set_cState(C_IDLE);
	        currentControl = 0;	// Fasttrack
			return;
		}
*/
        break;

	case C_COARSER:
		rpm_min_ok = Ctl_setspeed - rpm_tolerance;
		rpm_max_ok = Ctl_setspeed + rpm_tolerance/2;
/*
		if(projected_err >= 0 || (projected_err > (-rpm_tolerance/2)))
		{
	        Set_cState(C_IDLE);
	        currentControl = 0;	// Fasttrack
			return;
		}
*/
        break;
	}
	if(projected_rpm > rpm_min_ok && projected_rpm < rpm_max_ok)
	{
        if(cState == C_FINER || cState == C_COARSER)
        {
//        	SetDrivePins(0);
        	Set_cState(C_BRAKE,290);
//    		Set_cState(C_IDLE,290);
        }
        else
        {
        	Set_cState(C_IDLE,300);
        }
        currentControl = 0;	// Fasttrack
		return;
	}
#else


	abs_projected_err = ABS(projected_err);


// Try tighter tolerence while moving....??
//	rpm_tolerance = RPM_TOLERANCE;
//	if(dState == MD_IDLE) rpm_tolerance = deadband;



	if(abs_projected_err < rpm_tolerance)
	{
        Set_cState(C_IDLE);
        currentControl = 0;	// Fasttrack
//		TraceReturn(t,"Fasttrack");
		return;
	}
#endif


#ifdef MH_CTL_DEBUG2
	sprintf(Ctl_pbuff,"%d,%d,%d,%d,%d,%d\r\n",
			Ctl_pwm_val,Ctl_actualspeed,Ctl_setspeed,Ctl_history_total_rpm_change,projected_rpm,projected_err);
	if(strcmp(Ctl_pbuff,Ctl_pbuff2))
	{
		strcpy(Ctl_pbuff2,Ctl_pbuff);
		txDebug(Ctl_pbuff);
	}
#endif
	if(Drive_wait_flag)
	{
        Set_cState(C_IDLE,400);
//		txDebug("Brakereturn\r\n");
//		TraceReturn(t,"Brakereturn");
		return;
	}

//	currentControl = 255;	// default;
	currentControl = 	getParameter (MC_MAX_PWM);	// default MHH:09/08/2018

#ifdef MH_CTL_FINE_TUNE		// MHH:03/11/2018. Give it a minimum of maximum current for a few ticks.

	rpm_per_tick   = (Ctl_rpm_per_tick_x10 + 5)/10;

//#define MH_TRY_THIS	// MHH:22/08/2018
#ifdef MH_TRY_THIS
	if(abs_projected_err < rpm_per_tick*2)	// Keep maths short
	{
//		currentControl = (255 * abs_projected_err) / rpm_per_tick;
		currentControl = currentControl /2;
	}
#else
	if(abs_projected_err < rpm_per_tick)	// Keep maths short
	{
//		currentControl = (255 * abs_projected_err) / rpm_per_tick;
		currentControl = (currentControl * abs_projected_err) / rpm_per_tick;
	}
#endif

//#ifdef MH_CTL_DEBUG
//	if(currentControl < 255)
	if(currentControl == 0)		// Something wrong!!
	{
		sprintf(Ctl_pbuff,"CC:%d,CHT:%d,PRPM:%d,PER:%d\r\n",
		(int)currentControl,Ctl_history_total_rpm_change,projected_rpm,projected_err);
		txDebug(Ctl_pbuff);
	}
//#endif
#endif

	switch(cState)
	{
	case C_IDLE:
		if(projected_err > 0)
		{
			Set_cState(C_FINER,500);
		}
		else
		{
			Set_cState(C_COARSER,600);
		}
		break;
	case C_FINER:
		if(projected_err > 0)
		{
			Set_cState(C_FINER,700);
		}
		else
		{
			Set_cState(C_IDLE,800);
		}
		break;
	case C_COARSER:
		if(projected_err < 0)
		{
			Set_cState(C_COARSER,900);
		}
		else
		{
			Set_cState(C_IDLE,1000);
		}
		break;
	case C_BL_CONTROL:	// MHH:21/01/2026
		break;
	case C_BRAKE:	// dummy for compiler
		break;
	}

#ifdef MH_OLD_XXX
	if(projected_err > 0)
	{
		Set_cState(C_FINER);
	}
	else
	{
		Set_cState(C_COARSER);
	}
#endif
//	TraceReturn(t,"End");
}

#endif
//----------------------------------------------------------------------------------
#ifdef MH_CHECK_KEYS
void Debug_check_keys(BYTE keys)
{
	static BYTE check_keys;

	if(check_keys != keys)
	{
		printf("keys: %02x\r\n",keys);
		check_keys = keys;
	}

}
#endif
//------------------------------------------------------------------------------------
extern bool Xoar_mode(void);	// In comms.c

void Check_zone(void)
{
	if(Xoar_mode()) return;

	if(RC_X_zone == RC_S_ZONE_REVERSE)
	{
		LED_overlay_flags |= LED_OVERLAY_FLAG_REVERSE_ZONE;
	}
	else
	{
		LED_overlay_flags &= ~LED_OVERLAY_FLAG_REVERSE_ZONE;
	}
}
//------------------------------------------------------------------------------------
void Set_cState(ControlState new_cstate,int ifrom)	// MHH: 02/04/2018. So we know when cState is changing
{
#ifdef MH_XXX
	static int idle_cnt,fine_cnt,coarse_cnt,brake_cnt,other_cnt;
	char *state_desc;
#endif
	if(new_cstate == cState)
	{
		return;
	}

	cState = new_cstate;


#ifdef MH_XXX
	switch(cState)
	{
	case C_IDLE:
		state_desc="C_IDLE";
		idle_cnt++;
		break;
	case C_FINER:
		state_desc="C_FINER";
		fine_cnt++;
		break;
	case C_COARSER:
		state_desc="C_COARSER";
		coarse_cnt++;
		break;
	case C_BL_CONTROL:
		state_desc="C_BL_CONTROL";
		brake_cnt++;
		break;
	case C_BRAKE:
		state_desc="C_BRAKE";
		other_cnt++;
		break;
	}
	DPRINTF("Set_cState:%s,from:%d\r\n",state_desc,ifrom);
	if(Ctl_display)
	{
		txDebug(state_desc);
		txDebug("\r\n");
	}
#endif
}
//------------------------------------------------------------------------------------
extern BYTE Auto_check_manual(void);
//extern bool AC200_status_test;
void controlCycle (void)
{
    OpMode          mode;
#ifndef AC210_PORT
    WORD            propFactor, intFactor, diffFactor, deadband, sticks;
    long            err;
#endif
    WORD            holdSpeedTemp;
//    WORD            controlMode;
    ControlState    cs;
    long            control;
//    BYTE            keys, cnt;
    BYTE            keys;
#if MAP_VERSION
    PressureValue   mp;
#endif


	if(TimeInSeconds == 0)
	{
		return;		// Ignore for first second.
	}
#ifdef MH_XXX
	if(getParameter(AUX_PORTS_SERIAL_CTL) != AUX_PORT_SIG60)	// MHH:12/11/2022. Really SIG100, don't care if no status test for SIG100
	{
		if(AC200_status_test == false)	// MHH:28/023/20232. Think I should be testing if true...
		{
			return;
		}
	}
#endif

    actualSpeed = Get_RPM();		// MHH:22/12/2024
    Ctl_brushless_speed = 0;		// MHH:26/12/2024. Max speed is default.



//	if(fState != F_IDLE)	// MHH:13/12/2024
	if((fState != F_IDLE) && (fState != F_BETA_IDLE))	// MHH:22/12/2024
	{
        setSetSpeed(actualSpeed);			// MHH:19/07/2025
		return;
	}


	if(Auto_check_manual())
	{
//        magSpeed ((WORD*)&actualSpeed);	// MHH: 27/01/2019
//        actualSpeed = Get_RPM();			// MHH:22/12/2024
//        setspeed = actualSpeed;
        setSetSpeed(actualSpeed);
		return;
	}

	Check_zone();


#if MH_REMOTE_POS
	if(RemotePos_active)
	{
        magSpeed ((ULONG*)&actualSpeed);
        return;
	}
#endif

    Ctl_rpm_per_tick_x10 = getParameter(CTL_RPM_PER_TICK_X10);	// Check if new control logic
// Note: Following check probably not necessary.
    if(Ctl_rpm_per_tick_x10 < CTL_RPM_MIN || Ctl_rpm_per_tick_x10 > CTL_RPM_MAX) Ctl_rpm_per_tick_x10 = 100;	// MHH:15/08/2018 Set default 100


    cs = C_IDLE;
//    controlMode = PROP_MODE;
    mode = operatingMode ();
#ifdef MH_XXX
    if(mode == MANUAL)	// MHH:19/07/2025 dState
    {
    	Set_dState(MD_IDLE,20);
    	return;
    }
#endif

#ifndef AC210_PORT

    deadband = getParameter (PI_DEAD_BAND) * 10;
    propFactor = getParameter (PROP_FACTOR);    //(PI_GAIN)
    intFactor  = getParameter (INT_FACTOR);     //(PI_TIME_CONSTANT)
    diffFactor = getParameter (DIFF_FACTOR);    //(PI_DIFF_GAIN)
    sticks = getParameter (PI_SAMPLE_TICKS);    // temp use
    err = 0;
#endif
    control = 0;

    // Get current RPM
//    actualSpeed = Get_RPM();	// NHH:22/12/2024

#ifdef MH_CTL_TUNE
	if(AT_Ctl_tune)
	{
		Ctl_update_rpm_hist();
	}
#endif

#if REMOTE_VERSION
    // only fill in set speed here if not in remote
    // if in remote mode, then this has already been set
//    if (remoteMode() <= REMOTE_IDLE && Auto_flag == 0)	// MHH:05/07/2019 Note: < REMOTE_IDLE doesn't work!!!!
    if (remoteMode() <= REMOTE_IDLE && Auto_flag == 0)
    {
#endif

        // get the set speed...
        if (mode == TAKEOFF)
        {
//            setSpeed = (long)getParameter (SP_TAKEOFF) * 10;
//            setSetSpeed((long)getParameter (SP_TAKEOFF) * RPMFACTOR);
            setSetSpeed(getParameter (SP_TAKEOFF));
        }
        else if (mode == CLIMB)
        {
//            setSpeed = (long)getParameter (SP_CLIMB) * 10;
//            setSetSpeed((long)getParameter (SP_CLIMB) * RPMFACTOR);
            setSetSpeed(getParameter (SP_CLIMB));
        }
        else if (mode == CRUISE)
        {
//            setSpeed = (long)getParameter (SP_CRUISE) * 10;
//            setSetSpeed((long)getParameter (SP_CRUISE) * RPMFACTOR);
            setSetSpeed(getParameter (SP_CRUISE));
        }
        else if (mode == HOLD)
        {
            if (isSlaveProp())
            {
                WORD setspeed;
            	setspeed = getSlaveRPMSettings ();
                if (setspeed==0)
                {
//                    setspeed = (long)holdSpeed*RPMFACTOR;
                    setspeed = holdSpeed;
                	// default to set speed if something is wrong
//                    setSpeed =  (long)getParameter (SP_HOLD) * 10;
                }
                setSetSpeed(setspeed);
            }
            else
            {
//                setSpeed = (long)holdSpeed * 10;
//                setSetSpeed((long)holdSpeed * RPMFACTOR);
                setSetSpeed(holdSpeed);
            }
        }
        else if(mode == FEATHER || mode == BETA)		// MHH:17/07/2025. What about REVERSE?
        {
            Set_cState(C_IDLE,1310);
            Set_current_control(0,155);
            setSetSpeed(actualSpeed);
            return;
        }
        else
        {
//            setSpeed = actualSpeed;
            setSetSpeed(actualSpeed);
        }

#if REMOTE_VERSION
    }
    // if we are in ground mode, then we are directly controlling the pitch
    // motor
    if (remoteMode() == REMOTE_GROUND_MODE)
    {
        // direct control of pitch motor
        Set_cState((ControlState)remoteControlDirection(),800);

        if (cState != C_IDLE)
        {
            Set_current_control(255,90);
//            currentControl = 255;
//            currentControl = getParameter (MC_MAX_PWM);
        }
        else
        {
//            currentControl = 0;
            Set_current_control(0,100);
        }
        return;
    }
    // in flight mode, if the set speed is 0 then the control loop is
    // bypassed
#endif
    if(AutoGyro_reverse)
    {
		if((AutoGyro_state == AUTOGYRO_LATCHED) || (AutoGyro_state == AUTOGYRO_LATCHED_PIN))
    	{
    		return;
    	}
    }

#if REVERSING_VERSION
    if(Control_type == CT_REVERSE)
    {
        if (mode == FEATHER || mode == REVERSE)
        {
            keys = manualKeys() & ~MANUAL_KEY_FEATHER;
//            Debug_check_keys(keys);
            if (keys & MANUAL_KEY_COARSE)
            {
                // drive to coarse
                if (wasReversing)
                {
                    remoteStopReverse();
                    wasReversing = 0;
                }
                Set_cState(C_COARSER,900);
                Set_current_control(255,110);
//                currentControl = 255;
//                currentControl = getParameter (MC_MAX_PWM);
                RC_X_last_action = RC_X_REVERSE_COARSE;
                return;
            }
            else if (keys & MANUAL_KEY_FINE)
            {
                // drive to reverse
                if (!wasReversing)
                {
                	if(RC_X_zone == RC_S_ZONE_NORMAL)		// MHH:16/03/2018
                	{
                    	if(Reverse_RPM_too_high())
                    	{
                    		return;
                    	}
                	}

                	wasReversing = 1;
                	RC_X_last_action = RC_X_REVERSE_FINE;
                	remoteStartReverse();

//                    wasReversing = 2;		// Debug
                }
                return;
            }
            else if (wasReversing)
            {
                 // disable reverse drive
                 remoteStopReverse();
                wasReversing = 0;
            }
            Set_cState(C_IDLE,1000);
//            currentControl = 0;
            Set_current_control(0,120);
            return;
        }
        else
        {
        	if(mode == HOLD)		// MHH:09/07/2019. Could also test if Xoar mode.
        	{
        		if(currentState & S_STOP_REVERSE)
        		{
                    Set_cState(C_IDLE,1100);
                    Set_current_control(0,130);
//                    currentControl = 0;
                    return;
        		}
        	}
        }
    }
#endif

    if(AutoGyro_reverse && (AutoGyro_state == AUTOGYRO_WAIT_FINISH))	// Ignore max engine speed when getting out of latched
    {
//        if (actualSpeed < ((long)getParameter (MIN_ENGINE_SPEED)*RPMFACTOR))
        if (actualSpeed < ((long)getParameter (MIN_ENGINE_SPEED)))
        {
            Set_cState(C_IDLE,1200);
//            currentControl = 0;
            Set_current_control(0,140);

            return;
        }
    }
    else
    {
        // Check that speed is within the the controllable band
//        if ((actualSpeed < ((long)getParameter (MIN_ENGINE_SPEED)*RPMFACTOR)) ||
//            (actualSpeed > ((long)getParameter (MAX_ENGINE_SPEED)*RPMFACTOR)))
        if ((actualSpeed < ((long)getParameter (MIN_ENGINE_SPEED))) ||
            (actualSpeed > ((long)getParameter (MAX_ENGINE_SPEED))))
        {
            // If not, then set to idle and exit.

            Set_cState(C_IDLE,1300);
//            currentControl = 0;
            Set_current_control(0,150);
            return;
        }
    }
    if (setSpeed)
    {
        // only do the control loop if we have a set point...

        // check for an attempted modification to the HOLD set point
        // master prop only
        if (mode == HOLD && !isSlaveProp())
        {
            keys = manualKeys() & ~MANUAL_KEY_FEATHER;
#if REMOTE_VERSION
            // none of this if in remote mode
            if (remoteMode() > REMOTE_IDLE)
            {
                keys = 0;
                modifyHoldSpeed = 0;
            }
#endif
            if (keys)
            {
            	// set flag for next check
                if (!modifyHoldSpeed)
                {
                    modifyHoldSpeed = 1;
                }
                // set control output to force a pitch movement,
                // but only if current speed is within the range limit
//                holdSpeedTemp = actualSpeed/RPMFACTOR;
                holdSpeedTemp = actualSpeed;
//                DPRINTF("A=%d\r\n",actualSpeed);
                if (keys & MANUAL_KEY_FINE)
                {
                    if (holdSpeedTemp < getParameter (MAX_HOLD_SPEED))
                    {
                        cs = C_FINER;
                        // set mid control point
                        control = 128;
                    }
                    else
                    {
                        // too far..
                        cs = C_IDLE;
                        control = 0;
                        holdSpeedTemp = getParameter (MAX_HOLD_SPEED);
                    }
                }
                else if (keys & MANUAL_KEY_COARSE)
                {
                    if (holdSpeedTemp > getParameter (MIN_HOLD_SPEED))
                    {
                        cs = C_COARSER;
                        // set mid control point
                        control = 128;
                    }
                    else
                    {
                        // too far
                        cs = C_IDLE;
                        control = 0;
                        holdSpeedTemp = getParameter (MIN_HOLD_SPEED);
                    }
                }
            }
            // check for exit condition
            else if (modifyHoldSpeed)
            {
                // key released set current speed as set point
                // Only a local modification now

                // there are limits imposed
                // maximum hold speed is Takeoff speed
                // minimum hold speed is 80% of cruise
//                holdSpeedTemp = actualSpeed/RPMFACTOR;
                holdSpeedTemp = actualSpeed;
//                DPRINTF("H=%d\r\n",actualSpeed);

                if (holdSpeedTemp > getParameter (MAX_HOLD_SPEED))
                {
                    holdSpeedTemp = getParameter (MAX_HOLD_SPEED);
                }
                if (holdSpeedTemp < getParameter (MIN_HOLD_SPEED))
                {
                    holdSpeedTemp = getParameter (MIN_HOLD_SPEED);
                }
                holdSpeed =  holdSpeedTemp;

                // and make sure control stops now
//                setSpeed = actualSpeed;
                setSetSpeed(actualSpeed);
                modifyHoldSpeed = 0;
            }
        }
        else
        {
            // reset flag in case mode changed with key down
            modifyHoldSpeed = 0;
        }

        if (!modifyHoldSpeed)
        {
			if(Ctl_rpm_per_tick_x10)
			{

//				TraceVal=1;
				Ctl_logic();
//				TraceVal=0;
				return;
			}

        }
    }

    // Any changes......
    if ((cs != cState)|| ((WORD)control != currentControl))
    {
        Set_cState(cs,1400);
//        currentControl = (WORD)control;
        Set_current_control(control,160);

    }
}
//---------------------------------------------------------------------------------------------
// state accessor function
ControlState controlState (WORD *errorVal)
{
    *errorVal = currentControl;
    return (cState);
}
void Set_current_control(WORD cval,int ifrom)	// MHH:22/05/2024
{
	currentControl = cval;
#ifdef MH_DEBUG_SET_CC
	static WORD last_cval;
	static int last_ifrom;
	if(cval != last_cval || ifrom != last_ifrom)
	{
		int tick = LogData.log_ticks % 50;
		DPRINTF("Set_current_control:v=%d,from=%d,tick=%d\r\n",cval,ifrom,tick);
	}
	last_ifrom = ifrom;
	last_cval = cval;
#endif
}
//---------------------------------------------------------------------------------------------
void debug_speed(void)
{

}
//---------------------------------------------------------------------------------------------
//#define MH_MAXSPEED	(8000*RPMFACTOR)
#define MH_MAXSPEED	(8000)
void setSetSpeed(WORD val)
{
	if(val > MH_MAXSPEED)
	{
		debug_speed();
	}
	setSpeed = val;

}
//---------------------------------------------------------------------------------------------
WORD currentSetSpeed (void)
{
	if(setSpeed > MH_MAXSPEED)
	{
		debug_speed();
	}
    return (setSpeed);
}

WORD currentActualSpeed (void)
{
	if(actualSpeed > MH_MAXSPEED)
	{
		debug_speed();
	}
	return (actualSpeed);
}


