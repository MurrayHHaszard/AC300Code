/* ------------------------------------------------------------
Title:          Global.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   22/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Global procedure definitions

Changes:

------------------------------------------------------------ */

#ifndef _GLOBAL_H
#define _GLOBAL_H


#include "platform/ac200platform.h"		// So we can copy all .H in current directry to AC210 port directory

//#define AC210_PORT

#include "board.h"
// Corresponds to 5A. hwf.version is set to this value first time that software runs. Change as new hardware versions come out.
// See main.c hardware changes for more information.

#define far		// null def
typedef unsigned char BYTE;
typedef unsigned short  WORD;
typedef unsigned int ULONG;

extern WORD AC210_hardware_version;

#include "ac210_global.h"
//#define BYTE uint8_t
//#define WORD uint16_t
//#define ULONG uint32_t
// Version Number
extern const WORD version;
extern BYTE timerTick;		// Defined and used in main(). Helpful for knowing when a new second begins.

/*
Timer tick count (in main.c)
*/

#define ABS(a) (((a)>=(0))?(a):-(a))

#define TRUE	1
#define FALSE	0

// #defines for building versions
// un comment the appropriate ones


//#define MH_REMOTE_POS			1 		// WIP
//#define RPMFACTOR		1
#define MH_24V
#define MH_REMOTE_POS			0 		// WIP
#define DEMO_VERSION         0
#define REVERSING_VERSION    1
#define REMOTE_VERSION       1		// MHH: Now a parameter (29/10/2016) but leave as a compile switch
//#define REMOTE_VERSION       0
#define MAP_VERSION          0
//#define BETA_VERSION         1
#define BETA_VERSION         1
//#define XOAR_VERSION		 0		// MHH:Now a parameter. 29/10/2016

//#define SUBVERSION			27
//#define SUBVERSION			44		// MHH: 29/10/2016
//#define SUBVERSION			45		// MHH: 8/12/2016
//#define SUBVERSION			46		// MHH: 14/02/2017
//#define SUBVERSION			48		// MHH: 20/04/2017
//#define SUBVERSION			50		// MHH: 07/09/2017
//#define SUBVERSION			51		// MHH: 01/03/2018
//#define SUBVERSION			52		// MHH: 02/04/2018
//#define SUBVERSION			53		// MHH: 18/07/2018
//#define SUBVERSION			54		// MHH: 20/09/2018
//#define SUBVERSION			55		// MHH: 09/11/2018
//#define SUBVERSION			56		// MHH: 19/11/2018
//#define SUBVERSION			57		// MHH: 24/11/2018
//#define SUBVERSION			58		// MHH: 11/01/2019. RTC changes
//#define SUBVERSION			59		// MHH: 11/07/2019
//#define SUBVERSION			60		// MHH: 19/07/2019
//#define SUBVERSION			61		// MHH: 27/07/2019
//#define SUBVERSION			62		// MHH: 20/08/2019
//#define SUBVERSION			63		// MHH: 24/10/2019
//#define SUBVERSION			64		// MHH: 06/03/2020
//#define SUBVERSION			65		// MHH: 07/05/2020
//#define SUBVERSION			66		// MHH: 10/11/2020
//#define SUBVERSION			67		// MHH: 22/12/2020
//#define SUBVERSION			68		// MHH: 11/01/2021
//#define SUBVERSION			69		// MHH: 15/03/2021
//#define SUBVERSION			70		// MHH: 11/08/2021
//#define SUBVERSION			71		// MHH: 09/03/2022
//#define SUBVERSION			72		// MHH: 22/04/2022
// Note: Only sending 73 to Xoar, so can add other updates
//#define SUBVERSION			73		// MHH: 27/07/2022
//#define SUBVERSION			74		// MHH: 18/11/2022
//#define SUBVERSION			75		// MHH: 13/03/2023
//#define SUBVERSION			76		// MHH: 25/03/2023
//#define SUBVERSION			77		// MHH: 28/03/2023
//#define SUBVERSION			78		// MHH: 03/05/2023
//#define SUBVERSION			79		// MHH: 26/05/2023
//#define SUBVERSION			80		// MHH: 30/05/2023
//#define SUBVERSION			81		// MHH: 24/07/2023
//#define SUBVERSION			82		// MHH: 11/08/2023
//#define SUBVERSION			83		// MHH: 29/08/2023
//#define SUBVERSION			84		// MHH: 02/09/2023
//#define SUBVERSION			85		// MHH: 13/09/2023
//#define SUBVERSION			86		// MHH: 20/10/2023
//#define SUBVERSION			87		// MHH: 14/11/2023
//#define SUBVERSION			88		// MHH: 22/11/2023
//#define SUBVERSION			89		// MHH: 13/12/2023
//#define SUBVERSION			90		// MHH: 19/12/2023
//#define SUBVERSION			92		// MHH: 24/12/2023
//#define SUBVERSION			93		// MHH: 31/12/2023
//#define SUBVERSION			94		// MHH: 22/01/2024
//#define SUBVERSION			95		// MHH: 27/01/2024
//#define SUBVERSION			96		// MHH: 30/01/2024
//#define SUBVERSION			97		// MHH: 04/02/2024
//#define SUBVERSION			98		// MHH: 03/04/2024 Add Slider Control
//#define SUBVERSION			99		// MHH: 06/04/2024 Add Slider Control
//#define SUBVERSION			100		// MHH: 15/05/2024 Add Control Delta logic + change version logic to allow 3 digit subversion and 2 digit version type
//#define SUBVERSION			101		// MHH: 22/05/2024
//#define SUBVERSION			102		// MHH: 29/05/2024
//========================================================
/*
 * Bugfix: 12/11/2024. Russell showed me that when using RPM generator (2000 prop rpm, 5600 rpm at GR of 2.80) that with Setspeed of 5700 (TO) should have been driving
 *                     FINE until FINE STOP, and with Setspeed of 5000 (CRUISE) should have been driving COARSE until COARSE stop.
 *
 * Suspect that Set_dState() was not being called.
 *
 * void Set_BL_dState(MotorDriveState newstate,int ifrom)
{

#ifdef MH_XXX	// MHH:12/11/2024. Russell's bug fix.
	if(ifrom == BL_dstate_ifrom)
	{
		if(BL_dstate == newstate) return;
	}
//	if(newstate != MD_IDLE)
	{
		if(DPRINTF_debug) DPRINTF("NewState:%d:%d:%d\r\n",timerTick,ifrom,(int)newstate);
	}
//	Set_cState(newstate);
#endif
	Set_dState(newstate,100);
//	BL_no_dstate_changes = true;
	BL_dstate = newstate;
	BL_dstate_ifrom=ifrom;

}

Debug Code:	MHH:12/11/2024
Added
 	 void SIG100_debug_command(void)

 	 and associated routines to see what commands were being sent.


 */
//#define SUBVERSION			103		// MHH: 13/11/2024. Note: AC300 defined from here on.
//===========================================================================================
/* Changes to void Brushless_control(void)
 *  To allow pause while waiting for delta_rpm to settle and also to take into account delta_rpm when calculating differential part of 'D'
 *  Changes to AC200User to allow modification of Brushless->Control (511) field.
 *  The value is a factor in calculating the differential value and is represented as a percentage. Eg value of 150 means internal diff value is scaled up
 *  by factor of 150/100 = 1.5
 *  New AC300User version is 10.003
 *
 *
 *  MHH:19/11/2024. Change AC300Calibrator so that data such as user defined feather stop can be changed without calibrating.
 *  New AC300Calibrator version is 1.16
 *  //            if (Calibrate.completed)
 *          if(Hub_rec.update_count > 0)    // MHH:19/11/2024
 *
 *
 *
 */

//#define SUBVERSION			104		// MHH: 18/11/2024
//================================================================================================
/* Date: 22/11/2024
   Problem: Brushless_control() logic hunting when deadband reduced to small value (eg 10) and motor is highly geared (eg below 100 to 1)
 * Fix: Add special logic when actual_rpm is close to target so that it just runs at low speed for about of a tenth of a second (bump logic).
 * Source file:Control.c
 * Note: The bump_ticks have been incorporated into the brushless control variable in this format:
 *  bbnnn where bb = bump_ticks and nnn = delta_factor_percent.
 *  e.g. 2150 would have bump_ticks = 2 and delta_factor_percent = 150
 *  Default (if BCV=0) is bump_ticks = 4, delta_factor_percent = 100
*/
//#define SUBVERSION			105		// MHH: 23/11/2024
//================================================================================================
/* Date: 30/11/2024
 * Problem: Brushless_control() logic hunting when trying to control RANS Aircraft 916 motor
 * Fix: Completely rewrite Brushless_control() to use simplified RPM per tick logic instead of PID logic.
 * Source file: Control.c
 * Note: From run logs, it appears that BCV (Brushless Control Value) for RANS 916 is about 140 (14.0 rpm/tick)
 *       BCV for AM 10 degree per sec prop appears to be about 300 (30.0 rpm/tick)
 *
 */
//#define SUBVERSION			106		// MHH: 30/11/2024
//================================================================================================
/* Date: 02/12/202
 *
 * Improve Brushless_control logic in conjunction with testing on simulator.
 *
 */
//#define SUBVERSION			107		// MHH: 02/12/2024
//================================================================================================
/* Date: 02/12/202
 *
 * Improve Brushless_control logic in conjunction with testing on simulator.
 *
 */
//#define SUBVERSION			108		// MHH: 02/12/2024

//#define SUBVERSION			109		// MHH: 05/12/2024
//===============================================================================================
/*  Date: 07/12/2024
 *  Add Brushless_get_speed() function and call in Brushless_control() for FINE and COARSE switch options.
 */

//#define SUBVERSION			110		// MHH: 07/12/2024
//===============================================================================================
/*
 * //#ifdef MH_XXX
		if(BL_wait_count < 10)		// Wait minimum of 0.2 secs
		{
			return;
		}
 *
 */
//#define SUBVERSION			111		// MHH: 09/12/2024
//================================================================================================
/*
 * Date:12/12/2024
 * Problem: AC300 not Auto-Feathering when RPM greater than MIN_ENGINE_SPEED
 * Fix: Add these lines to ControlCycle() routine:
 *
 *      else if(mode == FEATHER)		// MHH:12/12/2024
        {
            Set_cState(C_IDLE,1310);
            Set_current_control(0,155);
            setSetSpeed(actualSpeed);
            return;
        }

 *
 */
//#define SUBVERSION			112		// MHH: 12/12/2024
//================================================================================================
/*
 * Date:13/12/2024
 * Problem: AC300 not returning from Feather if RPM >= Set RPM of non-feather mode.
 * Fix: (1) Add these lines to ControlCycle() routine:
 *
 * 	if(fState != F_IDLE)	// MHH:13/12/2024
	{
		return;
	}
	   (2) Remove this test from reverseFeatherProc (void):

#ifdef MH_UNFEATHER_RPM_TEST    	// MHH:13/12/2024

    if(Feather_mode == FM_UNFEATHERING)	// MHH:11/08/2021. When coming out of feather, if RPM > minimum, do ordinary FINE logic.
    {
    	if(cSpeed > (getParameter (MIN_ENGINE_SPEED)))
    	{
    		test = 4;
    	}
    }
#endif
	(3) Only check reverseState if not brushless (also reverseFeatherProc())
	if(Sig100_connected == false)	// MHH:13/12/2024
    {
        switch (reverseState)
        {
        case uCheckState0:

 *

 *
 */
//#define SUBVERSION			113		// MHH: 13/12/2024
//================================================================================================
/*
 * Date:14/12/2024
 * Problem: When in standard Remote Control mode AC300 not returning "RS_T" termination message from RC_F=S command and RC_C=S command
 * Fix: Add this line to void RC_S_ControlCycle() routine:
 *
 * 			if(Sig100_connected) Soft_mode = 0;	// MHH:14/12/2024
 *
 *
 */
//#define SUBVERSION			114		// MHH: 14/12/2024
//================================================================================================
/*
 * Date:23/12/2024
 * Problem: subversion 114 lost the RPM when control word parameter was set as 8 for beta.
 * Fix: Change controlCycle():
 *
 *     actualSpeed = Get_RPM();		// MHH:22/12/2024
//	if(fState != F_IDLE)	// MHH:13/12/2024
	if((fState != F_IDLE) && (fState != F_BETA_IDLE))	// MHH:22/12/2024
 *
 *
 */
//#define SUBVERSION			115		// MHH: 22/12/2024

//================================================================================================
/*
 * Date:26/12/2024
 * Problem: AC300RemoteTest and REMOTE_GROUND_MODE operation such as move to FINE stop and brushless speed was not set to max speed (0).
 * Fix: Change controlCycle():
 *
 *     actualSpeed = Get_RPM();		// MHH:22/12/2024
 *     Ctl_brushless_speed = 0;		// MHH:26/12/2024. Max speed is default.
 *
 *
 */
//#define SUBVERSION			116		// MHH: 26/12/2024
//================================================================================================
/*
 * Date:12/01/2025
 * Problem: AC300RemoteTest_10_003 and Test Calibrate and Pos option: Sometimes it returned a Status of 10 or 8+2 = REVERSE STOP + COARSE STOP when it should
 *          have returned COARSE_STOP.
 * Fix: Change RC_S_ControlCycle() so that if(Sig100_connected) it does not update sysState with State_fast_result. Also exit routines State_StartFastTest()
 *      and State_NextFastTest() if Sig100_connected.
 *
 */
//#define SUBVERSION			117		// MHH: 12/01/2025
//================================================================================================
/*
 * Date:29/01/2025
 * Problem: RPM brushless control logic not working well.
 * Fix: Rewrite Brushless_control() routine in control.c
 *
 */
//#define SUBVERSION			118		// MHH: 29/01/2025

//================================================================================================
/*
 * Date:05/02/2025
 * Problem: Under Manual control, when going coarse from the FINE stop (and vica verca from COASE stop) the brushless speed was set to 8000 RPM.
 * Fix:  Change Get_manual_command(void) so that when starting manual FINE or manual COARSE the first second is at 50% speed.
 *
 */

//================================================================================================
/*
 * Date:10/02/2025
 * Problem: Control algorithm still not working well for ICON. Not limiting top RPM to 5800 (takeoff speed = 5700)
 * Fix: Increase speed of motor from what was 10% of motor speed for every 1% prop RPM from target to 100% speed for 3.5% from target and smaller speeds
 *      as approach target RPM. In addition, add logic so that if the FINE stop is on, or there has been no FINE driving for a second, AND the RPM is increasing
 *      at at least 180 RPM / sec over the past 10 ticks, then the projected rpm is increased by the RPM increase over the last 10 ticks.
 */

//================================================================================================
//#define SUBVERSION			119		// MHH: 10/02/2025
/*
 * Date:12/02/2025
 * Simplification: Use the Control Value (200) parameter to hold the Ticks/RPM value for brushless control, and use the brushless control field
 * to specify the percent (1 implied dec place) away from the target RPM that brushless control moves at full speed. Usually set to 35 = 3.5%
 */
//#define SUBVERSION			120		// MHH: 12/02/2025
//================================================================================================
/*
 * Date 20/03/2025
 * Development: Add Version 2 features to standard remote control.
 * This includes support for auxiliary serial port at 115.2K baud, plus CAN bus support, plus speed control for brushless motors.
 * These features are documented in AC300RemoteCommandSet_V2.docx
 */
//#define SUBVERSION			121		// MHH: 20/03/2025

//================================================================================================
/*
 *  Date: 28/04/2025
 *  Problem: Setting position in reverse zone where angle was negative not working.
 *  Fix: Ensure that when extracting angle value from CAN packet that the 2 bytes are treated as int16, not uint16
 */
//=================================================================================================
/*
 * Date: 02/05/2025
 * Modification: Change throttle control logic in AC210_get_throttle_from_engine(void) comms.c to allow Rotax throttle percentage from CANbus to
 *               be in a range of 0 to 115% instead of 0 to 100% and also to remove mapping of UL percentages, range 30% to 89% so that actual percentages
 *               are used in throttle control logic. Note that this required changes to AC300User:form2.cs
 */
//#define SUBVERSION			122		// MHH: 02/05/2025
//=================================================================================================
/*
 * Date: 23/05/2025
 * Modification: Change Feather Logic so that in case of a position error it will feather anyway (both auto and manual). Note this required changes
 *               to hub software too.
 *
 * Date 23/05/2025: Change remote control status_flags so that:
 *     if(Hub_pos_error)		// MHH:26/05/2025
    {
    	 currentState |= S_ERROR_POSITION;
    }
 *  Note: S_ERROR_POSITION = 0x800 but when shifted right 4 bits becomes 0x80 or 128.
 *        S_ERROR_POSITION was formerly S_ERROR_VOLTAGE but that was not being used. Remote Control documentation needs to be updated to change VOLTAGE to POSITION
 *
 *  For CAN commands, the bit_flags 128 is set in the status flags byte,
 *
 *  CAN_snd_status_msg.Data[3] = status;
 *
 *  in void RC_S_Send_CAN_status2(WORD eval,char ctype)
 *
 *  for CAN commands 701 to 704.
 */

//#define SUBVERSION			123		// MHH: 23/05/2025
//=================================================================================================
/*
 * Date: 29/05/2025
 * Problem: When Remote Control standard mode and parameter RC Port (331) set to 2 for CAN port, and no CAN transceiver sub pcb then the MCU was frozen as there
 *          was a continuous CAN read error which generated a CAN interrupt.
 * Fix: In AC210_Aux2_CAN_Init(void) when changing pins p0.10 and p0.11 to GP read mode, activate pullup on both pins as this is normal value when CAN transceiver
 *      attached.
 *
 *  Date: 29/05/2025
 *  Problem: Xoar reported that when they continuously sent standard remote positioning commands (eg. RC_SETPOS=20.0) at 20 ms intervals that the hub did not stay
 *           at 20.0 degree but "vibrated" and they sent a video of motor showing movement. Russell reproduced problem but at an interval of 10 ms, as did I.
 *  Fix: I think that because commands were being sent at a rate of more than 1 per tick that the input ring buffer overflowed which led to the loss of
 *       parts of the command, either leading to a bad command or a bad value. To fix modified routines checkForCommand() and checkForCommand2() to process more than
 *       1 command per cycle.
 *
 *
 */

//#define SUBVERSION			124		// MHH: 29/05/2025
//=================================================================================================
/*
 * Date: 09/06/2025
 * Problem: Xoar tried to do a software update for Hub but it failed with a "Write command fail: 0" and subsequent attempts to update when after using the restart logic
 *          it failed to initialise.
 * Fixes: a) Change line in comms.c
 * 					Sig100_hub_link = true;	// MHH:09/06/2025. In case hub is in reboot state
 * 			even if SIG100 is not connected.
 * 		  b) Do not attempt initialisation in ac210_sig100.c if Sig100_hub_link is true
 * 		  Note: Other changes to AC300Hub_Programmer utility
 *
 *
 */
//#define SUBVERSION			125		// MHH: 09/06/2025

//=================================================================================================
/*
 * Date: 10/06/2025
 * Request: Xoar want additional data in the Diagnostics export to CSV function. Specifically angle, status flag, status message, error code and current operating mode.
 *
 * Changes: Most of the changes are in Diagnostics utility Form6.cs but also needed to record cmode in log file so have added a new data byte (31) which was a filler
 *          See log.c
 *
 */
//=================================================================================================
/*
 * Date: 19/06/2025
 * Request: Xoar want Remote Control data field which currently contains data for setspeed if setspeed active, otherwise blade angle to contain the target blade angle
 * if they already have specified blade angle in data fields.
 *
 * Changes: See change to RC_S_Send_status2(WORD eval,char ctype) in remote.c
 */

//#define SUBVERSION			126		// MHH:19/06/2025
/*
 * Date: 28/06/2025
 * Changed AC210_ee_load(WORD value) to allow dumping a 1 megabyte file by pressing "Save 1Mb" button instead of 5Mb. Can still save 5Mb by menu.
 *
 *  Disallow Remote Control instructions if a position error and BL_diagnostics parameter & 128 == 0 then return error 14.
 *  Note: Setting BL_diagnostics parameter bit 128 on allows remote control to ignore error.
 *
 */

//#define SUBVERSION			127		// MHH:28/06/2025
/*
 * Date: 05/07/2025. AC300 was giving incorrect run number (-1) to Hub at initialisation.
 * Fix: Added function AC300_get_run_number() to return correct number.
 */


//#define SUBVERSION			128		// MHH:05/07/2025

/*
 * Date: 12/07/2025
 * Change: Lots of changes to Brushless_control() so that it now uses blade position and rpm_per_degree to work out target_blade_angle
 */
//#define SUBVERSION			129		// MHH:12/07/2025

/*
 * Date: 17/07/2025
 * Change: Lots of changes to digital.c and feather.c to allow beta and feather to work together. Need to check that reverse and feather still working.
         Also need to check that everything works correctly for led3 option. Note that beta and feather only work if param (308) 4 LED enable is set to 1
 */

//#define SUBVERSION			130		// MHH:17/07/2025

/*
 *  Date 19/07/2025
 *
 *  Bug fixes to get dState and Feather_mode working correctly for BETA and REVERSE
 */
//#define SUBVERSION			131		// MHH:19/07/2025
/*
 *
 * Date:21/07/2025. When not fully feathered and moving out of feather AC300 not starting FEATHER_REVERSE
 *                  2 changes: In void SIG100_send_command(void) comment out following line:
 *                  //    	    		resetStateBits (310,S_STOP_COARSE);	// MHH:21/07/2025.
 *                  In leds.c, change lines:
 *      if(dState == MD_FEATHER_REVERSE)
    	{
    		sState &= ~S_STOP_COARSE;
    	}
    	to:
    	if(dState == MD_FEATHER_REVERSE || dState == MD_FEATHER)	// MHH:21/07/2025
    	{
    		sState &= ~S_STOP_COARSE;
    	}

 *
 *
 *
 */

//#define SUBVERSION			132		// MHH:21/07/2025
/*
 * Modified  Brushless_control(void) to make logic control overspeed in takeoff mode. Use estimated rpm (BL_prop_rpm2) instead of actual BL_prop_rpm.
 *
 */
//#define SUBVERSION			133		// MHH:06/08/2025#define SUBVERSION			133		// MHH:06/08/2025

/*
 *  MHH:10/08/2025
 *
 *  Restored Control_logic from pre-angle version so that customers have consistent interface.
 */

/*
 * MHH:12/08/2025 Add support for diagnostic pkt from hub, starting with header '^', 9 bytes including enc_rpm and dac0_v
 */

//#define SUBVERSION			134		// MHH:12/08/2025

/*
 *  Added prop_deadband2 and modified special case of takeoff to be more sensitive to delta rpm in brushless_control()
 */

//#define SUBVERSION			135		// MHH:13/08/2025

/*
 * MHH:14:/08/2025. BL_enc_rpm was sometimes non-zero in Diagnostics when angle was not changing. Now set to zero if a short pkt ('-'), or actual value if
 *                  a long diagnostic pkt ('*').
 */


//#define SUBVERSION			136		// MHH:14/08/2025
/*
 *    MHH:17/09/2025. Russell reported that when PROP_RPM = 0 that "beta ready" LED signal of FINE led flashing RED at 1 Hz was not happening.
 *    This is because the NO RPM signal of orange flashing at 1 Hz was overriding it.
 *    Fix: Put check in leds.c where no_speed_signal is checked to ignore it if going in to beta.
 *
 *
 *     if((invalidConfig == false) && no_speed_signal)		// No speed signal
    {
//    	if(TimeInSeconds & 1)		// MHH: 28/01/2019. Supposed to be at 1 HZ
    	if(XoarStatus != 11)		// MHH:16/09/2025. Not if going into beta

    MHH: 17/09/2025. Russell reported that when in BETA and driving FINE and an overcurrent occurs that pitch motor can keep driving and instead of the REVERSE
    	LED showing solid RED the FINE and COARSE lights showed sold RED.

    Cause: The reason that the FINE and COARSE LEDs were solid red is that the overCurrentDir was being set to zero. This could happen after the motor drive is
           set to MD_IDLE if the current takes some time to get below the current limit.
    Fix: Change overcurrent logic in sstate.c to only update overCurrentDir if overCurrentTrip = zero.
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
   *
 */
//#define SUBVERSION			137		// MHH:17/09/2025

/*
 * MHH:20/09/2025. Slight adjustment to voltage conversion in AC210_convert_voltage(WORD raw) in analog.c.
 *  Now multiply by 0.859 instead of 0.915.
 *
 *  AC200User value for hub voltage was too high - 13.9 V instead of 12.5V.
 *  Fix: Modify SIG100_get_hub_voltage(void) function so that value is divided by 10.954 before being adjusted by any hub adjust percentage.
 *
 *  Simplify and improve accuracy of AC300 voltage by saving as tenths of a volt /2 in log file, instead of tenths/5 previous to version 10.138
 *  Change brushless voltage log format to the same as controller voltage format for simplicity.
 *  Note: Need version AC300Diagnostics_10_013 or later to use this voltage format.
 */

//#define SUBVERSION			138		// MHH:20/09/2025

/*
 * MHH:26/09/2025. Russell reported that when in BETA and being driven FINE by manual keys and REVERSE stop is reached that the REVERSE LED
 *                 was not blinking GREEN at 1 Hz but instead was ORANGE.
 *
 * Fix: Commented out some code in Ac210_sig100.c that was converting a REVERSE stop into a FEATHER stop but only if not a 4LED controller.
 *
//			if(Sig100_hub.ud_stops & HUB_UD_REVERSE_STOP)	// Reverse enabled?
#ifdef MH_XXX		// MHH:26/09/2025
			if(Enc_data.user_defined_stops & HUB_UD_REVERSE_STOP)	// Reverse enabled?
			{
				if(dState == MD_BETA || dState == MD_BETA_EXIT)
 *
 *
 */
//#define SUBVERSION			139		// MHH:26/09/2025

/*
 * MHH:15/10/2025. Changes to Brushless_control() to work with new simulator logic (mhsim5.c). Mainly controlling speed as getting closer to
 *                target speed, special logic for takeoff mode and speed over limit. Modify speeds depending on gearbox ratio.
 */

//#define SUBVERSION			140		// MHH:15/10/2025

/*
 * MHH:24/11/2025. Changes to void Brushless_control(void). Change Speed_control to take into account the Enc_data.cal_motor_ratio, so a smaller faster
 *  ratio will use correspondingly slower speeds to try and ensure a constant rate of change between different hubs with different ratios.
 *  Knowing that it takes about 0.3 of a second to change speed of pitch control motor, use less changes: Full speed, medium speed and slow speed.
 */

//#define SUBVERSION			141		// MHH:24/11/2025
/*
 * MHH:16/12/2025 Change format of BL for '+' and '-' so that instead of "+p" or "-m" with optional speed appended,it is now in format
 *                "+sc" or "-sc" where s is an ascii character representing speed and c is the checksum for the preceding 2 bytes.
 *                The reason for this change is that occasionally when the speed byte was corrupted the hub assumed full speed.
 *                Note Hub software was changed from 147 to 148. So that Hub software is compatible with earlier versions, the AC300 version
 *                number is now passed on initialisation as part of the 'i' packet. The AC200 already knows the hub software version, so it choses
 *                format to send to hub based on software revision.
 *
 * MHH:17/12/2025 To try and verify when rpm is changing is caused by throttle change (eg takeoff) the log pwm byte is now used to record the RPM change in the
 *                last 10 ticks. If it is above 50 and prop_rpm > prop takeoff rpm then maximum coarse speed is requested. This required a change to AC300Diagnostics
 *                (now version 10.015) to display the value (pen = BlueViolet).
 *
 * MHH:18/12/2025 Add 2 checks when Calibrating, after receiving ATXCALIBRATE command (comms.c):
 *                1) RPM must be zero
 *                2) If not remote control board then mode must be MANUAL
 *
 */
//#define SUBVERSION			142		// MHH:18/12/2025

/*
 * MHH:23/12/2025. The AC300UAV4 (RC) board had stopped talking to the hub PCB because I had added a pin (P1.17) to control the HDC in the SIG100 pcb. This pin
 *                 had previously been used to control one of the front panel LEDs.
 *                 To fix, remove the pin definitions held in board_sysinit.c and instead define explicitly in AC210_LED_Init(void) if a RC type board.
 *                 Note: RC_board flag set if P1.9 grounded. Brushless flag set if P1.10 grounded.
 *
 * MHH:23/12/2025  Change DEF_FLASH_LOG_RATE from 0 to 1 in param.c. If zero then no logging and possibly does not set logctl record up properly.
 *
 * MHH:06/01/2026  Modify ee_get_logctl() in ac210_log.c so that when initialising the logctl data is checked against the flash data (Verify_logctl_data) and if it
 *                 fails check then an attempt is made to fix it (Find_correct_logctl_data).
 *
 *                 Modify AC210_logix_build() so that it handles very long runs by displaying checkpoints. Also, if missing a start checkpoint, allow a run to
 *                 start from first checkpoint.
 *
 *                 Note: we should test logctl verify and index rebuild against a fresh flash file, especially one that has not wrapped.
 */

//#define SUBVERSION			143		// MHH:23/12/2025

/*
 * MHH:10/01/2026	Problem: Had a problem with exchange of software revision numbers between new and old versions. This resulted in
 *                  sending old form packets when hub was expecting new form packets.
 *                  Fix: Added 'k'pkt to initialisation routine. 'k' pkt tells hub the rev number of the AC300.
 *
 *                  Problem: In case AC300 had any related problems to Hub software with the p_BufferLen variable, deleted it whenever used and now
 *                  use strlen() function if needed.
 *
 */

//#define SUBVERSION			144		// MHH:10/01/2026
/*
 * MHH:16/01/2026. Problem: Russell showed me that when using AC300RemoteTest program with a brushed reversing (not beta) hub that simple commands such
 *                 as go fine or go coarse were not working consistently. It turned out that when the mosfets were trying to (say) move fine, and the
 *                 pitch was at a fine stop, that the status tests were grounding through the mosfet, causing an apparent current which caused the status
 *                 test to terminate prematurely.
 *                 Fix: Whenever a stop is encountered set the drive pins to idle.
 *
 *                 Problem: When feathering with a very slow pitch motor it never reached the feather stop.
 *                 Fix: This was because there was a time limit of 1000 ticks (20 seconds) on the feather and unfeather remote control commands. This has been changed
 *                 to 2000 ticks (40 seconds).
 *
 *                 Problem: When trying to interrupt a feather command with an unfeather command it would stop the feather but not start reverse feather. The next
 *                 unfeather command would put it into reverse feather.
 *                 Fix: Change RC_S_unfeather() so that reverse is started if controlFeathering or the feather or coarse stops active, not just the stops.
 *
 *                 Problem: When starting an unfeather while feathering, it briefly unfeathered then stopped.
 *                 Fix: This was caused by Soft_mode being set to braking when feathering was stopped, so modified Set_drive_pins2() so that braking would
 *                 only be set if previously  a fine or coarse was active.
 *                 Note: May be a good idea to stop braking if any drive command is given that is not an idle.
 *
 */


//#define SUBVERSION			145		// MHH:16/01/2026

/*
 *  MHH:21/01/2026. Problem: Control logic not working well for brushed motors.
 *                  Fix: Restore logic from AC210_4_77A. Note also needed to bring similar version of ac210_drive.c
 */

//#define SUBVERSION			146		// MHH:21/01/2026

/*
 * MHH:30/01/2026. Brushless control was moving from XoarStatus 5 (Fine limit) to XoarStatus 8 (Moving fine at fine pitch limit) every alternate tick, causing
 *                 when trying to move FINE at FINE stop.
 *                 Fix: Use commands (eg C_FINER, C_COARSER) inside Brushless_control() instead of Drive settings (dState).
 */

/*
 * MHH:31/01/2026. Brushless_control() was being bypassed for reasons associated with a brushed control logic fix (MHH:02/11/2018), which caused the change in
 *                 rpm calculations to be bypassed, which showed up in run graph.
 *                 Fix: Move the call to Brushless_control() to before the "fix". Note: I have doubts about the MHH:02/11/2018 fix and would like to remove
 *                 "fix" and see what happens.
 *
 *
 * MHH:01/02/2026. Add logic to Brushless_control() so that if RPM rate of increase >= 50 per 100 ms, and actual prop RPM >= (set prop rpm - 200) then
 *                 pitch changes coarse at maximum speed.
 */
//#define SUBVERSION			147		// MHH:01/02/2026
/*
 * 					Problem: Run graph of UL showed switching between XoarStatus of 8 and XoarStatus of 5 if RPM increase rate > 50.
 * 					Fix:Move "return" from overspeed logic in control.c
 *
 * 					Problem: Run 32 of UL test, at time 00:47 when delta RPM >= 50 but actual RPM within deadband then overspeed logic was not being reached.
 * 					Fix: Move overspeed logic before within deadband check within Brushless_control()
 *
 * 					Problem: Run 32 of UL test, at time 01:06 the delta RPM exceeds 100/100 ms, and the start of the coarse pitch change is too late to stop
 * 					rpm overspeeding.
 * 					Fix: Modify overspeed logic to change rpm_margin from 200 to 400 if delta RPM >= 100
 *
 * 					Problem: Run 31 of UL test, at time 00:35 when going fine the overspeed logic was triggered, causing brief coarse movement.
 * 					Fix: Ignore overspeeding logic if change in angle (BL_delta_angle) over last 10 ticks was >= -0.2 degrees.
 */


//#define SUBVERSION			148		// MHH:02/02/2026
/*
 * MHH:04/02/2026. Add logic to allow AD0.5 to monitor temperature for AC300UAV5 board. This required changes to AC300User and AC300Diagnostics to show
 *                 temperature.
 *
 *
 * MHH:05/02/2026.	Change Brushless_control() so that delta RPM logic can be triggered at target prop RPM - 1000 (instead of 400) and that it will remain
 *                  active until delta RPM < 10.
 *
 */

// #define SUBVERSION			149		// MHH:04/02/2026
/*
 *  MHH:08/02/2026. Problem: V149 not working at customer, appears to think it is RC only version.
 *  				Fix: #ifdef logic in AC210_BoardInit(void) where AC210_remote_control_board flag is set if AC210_RC_PIND is zero.
 *  				Now rely on (ps.parms[RC_MISC_OPTIONS] & 1) to test if RC only board.
 */

//#define SUBVERSION			150		// MHH:08/02/2026

/*
 * MHH:09/02/2026.  Problem: All LEDS flashing RED, no SIG100 communication.
 * 					Fix: HDC  was being set low. if brushless, always open pin (1,23) in output value 1. Also, had accidentally removed
 * 					brushless test in version 10_150 from STATE_SetPins(). Put back.
 */

//#define SUBVERSION			151		// MHH:09/02/2026

/*
 * MHH:16/02/2026. Problem: Delta prop rpm not high enough on ICON test Rotax 916.
 *                 Fix: Change Brushless_control() to use engine rpm, not prop rpm.
 */
/*
 * MHH:17/02/2026. Problem: Using Prop rpm instead of engine rpm made control of geared RPM such as Rotax 916 clumsy.
 *                 Fix: Changed made logic compare engine rpm with target and use different defined margins for slow, medium and high pitch change speed.
 */

/*
 * MHH:18/02/2026. Problem: Customer with Rotax 916 getting Rotax overspeed errors when engine speed exceeded 5800 rpm.
 *                 Fix: Change Brushless Control parameter to Brushless Overspeed parameter and use that parameter to make pitch coarser if looking
 *                      like overspeed could happen.
 */

//#define SUBVERSION			152		// MHH:18/02/2026

/*
 * MHH:19/02/2026. Extend Overspeed logic so that it starts up to 200 rpm from rpm overspeed value, instead of 100 rpm
 */

/*
 * MHH:20/02/2026. Extend overspeed logic so that it can handle a setspeed less than overspeed - 100 rpm. Eg if overspeed = 5800, can handle setspeed
 *                 in any range 5700 or less.
 */

//#define SUBVERSION			153		// MHH:20/02/2026
/*
 * MHH:11/03/2026. Change Get_speed_from_ratio(int motor_speed) so that correct speed is selected depending on hub GB ratio.
 */
//#define SUBVERSION			154		// MHH:11/03/2026

/*
 * MHH:17/03/2026. Had introduced version coding error for version 'I' hardware, in AC300UAV6. Had coded 'I' as 1001, should have been 0111.
 *                 to fix: If AC210_hardware_version == 'G' and BL and RC pins are grounded then AC210_hardware_version = 'I'
 */

//#define SUBVERSION			155		// MHH:17/03/2026

/*
 * MHH:19/03/2026. LEDs not working on RC board. Add Map_led_RC logic in ac210_leds.c
 */

//#define SUBVERSION			156			// MHH:19/03/2026

/*
 * MHH:27/03/2026. Feather and Reverse not working on same AC300 controller. Same with Feather + Beta.
 *
 * 					Fix: Modify int Get_SIG100_command_from_dState(void)
 *
 *
 */
//#define SUBVERSION			157			// MHH:27/03/2026

/*
 * MHH:08/04/2026. Change LED display for when rpm > BETA_MAX_RPM or RPM > BETA_MAX_RPM_ENGAGED.
 *                 Previously FINE LED was flashing red at 2 Hz 25% duty
 *                 Now FINE LED flashes ON/OFF rapidly 3 times, once per 2 second.
 */
//#define SUBVERSION			158			// MHH:08/04/2026

/*
 * MHH:13/04/2026. Change Brushless_control() so that bl_speed is a percent, not '1' to '9' + 1. Needs at least hub software version 1.51
 *                 Also need Diagnostics 10.023 at least to see run graph.
 *
 *                 Modify control logic so that BL_speed is inversely proportional to difference of rpm to target rpm, if hub version >= 151
 */


//#define SUBVERSION			159			// MHH:13/04/2026

/*
 * MHH:16/04/2026. Add Scale_pct (513) parameter to brushless ac300user tab, instead of combining with NCV.
 */
//#define SUBVERSION			160			// MHH:16/04/2026
/*
 * MHH:17/04/2026. Add param file to each log run
 */
//#define SUBVERSION			161			// MHH:17/04/2026

/*
 * MHH:15/05/2026.Problem: type 'F' hardware not showing green flashing leds at coarse and fine stops when under manual control.
 *  			  Fix:Modify void ADC_UpdateAC200_RawValues(uint8_t chan) to subtract 100 instead of 50 for type 'F' hardware.
 */
//#define SUBVERSION			162			// MHH:15/05/2026
/*
 * MHH:19/05/2026. Add new keywords:
 *                 ATXPOSLGET=nnn		// Convert nnn which is length from CHS in tenths of a mm into an encoder value. if nnn > reverse stop, returns enc_pos  of reverse stop
 *                                      // if no '=' then returns current encoder position.
 *                 ATXPOSLSET=nnn		// Convert nnn into encoder value and position. Must be in MANUAL mode and RPM=0.
 *                 ATXANGLE				// returns "A=nn.nn" which is the current blade angle.
 */

//#define SUBVERSION			163			// MHH:19/05/2026

/*
 * MHH:20/05/2026. Allow setting position to 2 decimal places for hub calibration:
 * 			("XPOSLSET2")
 *
 */


//#define SUBVERSION			164			// MHH:20/05/2026
/*
 *  MHH:23/05/2026. Increase accuracy of hub cam radius to 2 decimal places, and also increase accuracy of TDC offset from CHS to 2 decimal places
 *                  Increase accuracy of blade_offset to 2 decimal places
 */
//#define SUBVERSION			165			// MHH:23/05/2026

/*
 * MHH:08/06/2026.  Add "XPOSENC" to position exactly at encoder value. Modify positioning logic to only accept exact position
 */
#define SUBVERSION			166			// MHH:08/06/2026

#define PCB_VERSION_MULTIPLIER	100000			// MHH:15/10/2024

// DO Not have a MAP REMOTE variant

BYTE timerTicks (void);
// watchdog timer rest in main
//void watchIt (void);
void watchIt (WORD from);		// MHH: 28/11/2016

#define WD_MAIN		1000		// identify source file
#define	WD_ANALOG	2000
#define WD_COMMS	3000
#define WD_CONTROL	4000
#define WD_DIGITAL	5000
#define WD_DRIVE	6000
#define WD_FEATHER	7000
#define WD_LEDS		8000
#define WD_PARAM	9000
#define WD_REMOTE	10000
#define WD_RPM		11000
#define WD_SSTATE	12000
#define WD_DIAGS	13000

#define WD_AC210	20000

// PCB version number, in main.c
//BYTE pcbVersion (void);
WORD pcbVersion (void);	// MHH:09/01/2019

extern WORD Control_type;		// MHH: 27/09/2017
#define CT_NON_FEATHERING	0
#define CT_FEATHERING		1
#define CT_BETA				2
#define CT_REVERSE			3

// Note: We will use the DIM pin in Auxiliary Connector to allow AutoGyro reverse.

extern BYTE AutoGyro_reverse;
extern BYTE AutoGyro_state;
#define AUTOGYRO_IDLE				0
#define AUTOGYRO_REVERSING			1
#define AUTOGYRO_REVERSING_STOPPED 	2
#define AUTOGYRO_WAIT_KEYS			3
#define AUTOGYRO_AT_REVERSE_STOP	4
#define AUTOGYRO_UNLATCHED			5
#define AUTOGYRO_LATCHED			6
#define AUTOGYRO_LATCHED_PIN		7
#define AUTOGYRO_TAKEOFF			8
#define AUTOGYRO_EXIT				9
#define AUTOGYRO_WAIT_FINISH		10


/*
Debug control
*/
// debug flag for rest of code, defined in main.c
extern WORD debug;
// Debug is a set of flags, so that we can selectively
// debug sections of code
#define DEBUG_SPEED_INPUT    0x0001
#define DEBUG_SYSTEM_STATE   0x0002
#define DEBUG_SWITCH_STATE   0x0004
#define DEBUG_PI_CONTROL     0x0008
// analog channels
#define DEBUG_MOTOR_CURRENT  0x0010
#define DEBUG_MOTOR_STATE    0x0020
#define DEBUG_DIMMER         0x0040
#define DEBUG_MANIFOLD_PRESS 0x0080
#define DEBUG_XOAR			 0x0080		// Because Manifold pressure not used
#define DEBUG_ANALOG_VOLTAGE 0x0100

#define DEBUG_MOTOR_DRIVE    0x0200
#define DEBUG_FEATHER        0x0400
#define DEBUG_REVERSE        0x0800		// Note: Not used as at 20/10/2017
#define DEBUG_REMOTE_MODE    0x1000

#define DEBUG_STATE_MATRIX   0x2000
#define DEBUG_FINE_STOP		 0x4000		// MHH:07/07/2017. Could be setting any stop. This is starting point.

// PCB versions..
#define PCB_VER_1            (BYTE) 2      // all boards up to version 2 (with cutout on pcb)
#define PCB_VER_2            (BYTE) 3      // V3 rectangular pcbs
#define PCB_VER_3            (BYTE) 4      // V4 rectangular boards
#define PCB_VER_4			 (BYTE) 5	   // V5 LPC1768 boards
// flag for monitor output mode. defined in main.c
extern BYTE monitor;

// OverCurrent Trip
extern BYTE overCurrentTrip;
extern WORD overCurrentDir;
#if BETA_VERSION
	extern BYTE betaOverCurrent;
#endif
// bad config
extern BYTE invalidConfig;

extern WORD lastSpeed;		// defined in drive.c

// debug string output, defined in comms.c
void txDebug (const char  far *mess);
void txDebug2 (const char  far *mess);	// Only used by AC2_TEST
void txDebugVal(char far *mess,WORD w);

void Remote_reply (const char *mess);

//=====================================================================
void setMainTimerLimit(WORD w);
//=====================================================================
//----------------------------------------------------------------------
void Delay_ms(WORD ms);
void M16_Reboot(void);
WORD Set_trace_val(WORD trace_val);
void Restore_trace_val(WORD trace_val);
BYTE Trace(char far *mess);
void TraceReturn(BYTE t,char far *mess);
void TraceMsg(char far *mess);
extern WORD TraceVal;
extern WORD BrakeDebug_val;
void BrakeDebug(char far *mess);
void BrakeDebugVal(char far *mess,WORD w);
void AT_Poke(void);
void AC210_eelog(void);

//----------------------------------------------------------------------
extern WORD Auto_flag;		// defined in comms.c

#define CLI_LOG	1
extern BYTE CLI_log;
extern BYTE EEPROM_log;

//#define AT_BRAKE 1
//extern WORD AT_BRAKE_val;		// Indicates how many cycles to brake, should be at least AT_SoftStop_val

#define AT_POKE	1
extern ULONG TimeInSeconds;
extern BYTE AT_POKE_val;
extern WORD AT_SoftStart_val;	// In cycles,default 5
extern WORD AT_SoftStop_val;	// Number of cycles to SoftStop
extern int Ctl_rpm_per_tick_x10;	// max rate of RPM change used by ctl routine. Usually 5/8 x max rate motor/gerbox can do.
extern int AT_Ctl_tune;		// Flag to show RPM/Tick change rate
extern int AT_Trace;
extern WORD AT_Remote;		// declared in comms.c

extern char Ctl_pbuff[256];
void Ctl_report_rate_change(void);
void Ctl_init_tune(void);
//----------------------------------------------------------------------
// Following defs for Xoar implementation.
#define MH_XOAR_DEBUG		0	// Set to zero for live version

#define REMOTE_COMMS_NONE		0
#define REMOTE_COMMS_LEGACY		1
#define REMOTE_COMMS_XOAR		2
#define REMOTE_COMMS_STANDARD	3
#define REMOTE_COMMS_XOAR2		4
#define REMOTE_COMMS_CAN		5

extern BYTE RemoteCommsType;	// defined in comms.c

#define SERIAL_MODE_ASCII	0
#define SERIAL_MODE_XOAR	1
#define SERIAL_MODE_CAN		2

#define XOAR_HEAD_0		0xEB
#define XOAR_HEAD_1		0x90

#define CAN_HEAD_0		'C'
#define CAN_HEAD_1		'X'

typedef struct SERIAL_PKT_struct
{
	BYTE Mode;
	BYTE Len;
	BYTE Mode_TimeoutVal;
	BYTE Pkt_TimeoutVal;
	BYTE Pkt_Head_0;
}SERIAL_PKT_t;
//typedef struct SERIAL_PKT_struct SERIAL_PKT_t;
extern SERIAL_PKT_t S_Pkt;		// Defined in comms.c
extern BYTE XoarStatus;			// value set in leds.c
extern BYTE XoarSendFlag;		// comms.c
extern char DebugString[];
extern void initXoar(void);			// in comms.c

//#if XOAR_VERSION > 0
#define	XOAR_DEBUG			// Don't think this is used any more.
void MH_Debug(char far *mess);
extern void XoarCheckManual(void);	// in comms.c
extern void XoarSendPacket(void);
extern void XoarSetRemoteMode(void);
extern BYTE XoarGetSwitches(void);	// in digital.c
//#endif

#endif

#define AC2_TEST
#ifdef AC2_TEST
#define AC2_TEST_ON		123			// MHH: 03/03/2018. To be sure.
extern BYTE ac2_test_flag;
extern void AC2_TEST_start(void);
extern void AC2_TEST_stage(void);
#endif


void send (const char far *buf, BYTE numChar);	// in comms.c
void Abort(ULONG err,char far *msg);
int DiagsError(ULONG err);
void DebugAbort(far char *emsg);

#define MH_EEPROM_FAST_WRITE
#ifdef MH_EEPROM_FAST_WRITE
void ParameterFastUpdate(void);
#endif

//extern int RemotePos_active;

void AC210_SIG100_get_delta_angle(void);
extern float BL_delta_angle;

extern WORD currentState;
extern unsigned long TimerCount;
extern uint32_t Get_uint32(uint8_t *data_p);
uint16_t Get_uint16(uint8_t *data_p);

#if MH_REMOTE_POS
int CheckRemotePosCommand(void);
void RemotePos_ControlCycle(void);
#define AC200_H0        '_'
#define AC200_H1        '@'
#endif
