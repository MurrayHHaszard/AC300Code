/* ------------------------------------------------------------
Title:          main.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   22/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Main program loop implementation

Changes: TAKEOFF

------------------------------------------------------------ */

#include <stdio.h>
#include <string.h>
#include <math.h>	// For square root

#include "ac210_sig100.h"
#include "global.h"
#include "analog.h"
#include "comms.h"
#include "digital.h"
#include "leds.h"
#include "param.h"
#include "rpm.h"
#include "sstate.h"
#include "drive.h"
#include "control.h"
#include "feather.h"
#include "manifold.h"
#include "device.h"
#include "bitserial.h"
#include "diags.h"
#include "xoar.h"

#if REMOTE_VERSION
#include "remote.h"
#endif


//#elif MAP_VERSION
#ifdef MH_PRE_BETA
#if MH_REMOTE_POS

const WORD version = 700+SUBVERSION;

#elif (REVERSING_VERSION)

const WORD version = 800+SUBVERSION;

#elif BETA_VERSION

const WORD version = 500+SUBVERSION;

#else
#endif
#endif
//#define VERSION	(400+SUBVERSION)
#define VERSION	(10000+SUBVERSION)		// MHH:15/05/2024
const WORD version = VERSION;
#ifdef AC210_PORT
//const unsigned char Version_id[]={254,4,253,3,VERSION/256,VERSION&255};	// May be able to scan for it
const unsigned char Version_id[]={'X','X',':','V',':',0,VERSION/256,VERSION&255,':',':'};	// May be able to scan for it
#endif

/*
======================================================================================================
	Hardware version information for AC210_PORT:
	Note: AC210_HARDWARE_VERSION defined in global.h

	Version 5A. Initial version for testing. Made by PCBGOGO. Received September 2017

	Version 5B:
		12 V sense:
			Changed R71 from 3K to 4.7K as with 3K as could only measure battery voltage up to 12.9V.
		    With 4.7K can measure up to 18 V
		Lamp Dim sense (pin 7, auxiliary inputs):
			Added R87B as voltage splitter as per AC200.
		Throttle Position sense: (pin 8, auxiliary inputs):
			Added R88B as voltage splitter as per AC200.
======================================================================================================


    Version Information
    2.10    the first released code with a version number. Coincides with
            Software Upload using non Mitsubishi uploader. Includes changes
            to operation of overcurrent trip, fix to low RPM signal, changes to
            hold operation.
    3.10    06/07/2001. Released code for Version 3 Hardware. Contains
            fixes for state display. Fixed current sensing software, Modified
            hold mode to be rest on power up only. Upper and lower limits on
            hold mode operation, "no speed" indicator limit setable.
            Feather mode "auto" exit instituted.
    3.11    16/07/2001. Disable low speed indicator when in feather mode
    3.12    19/07/2001. Reverse feather when switch out of feather mode. Motor current multiplier
            modified slightly.
    3.14    (skipped .13). Added multipliers for table mode. depends on
            whether error increasing or not measured over a 0.5 second interval
            fixed 3.12 so reverse feather doesn't smoke the FETS
    3.15    30/07/2001. Added high speed debug mode. outputs control state, drive state, set speed, actual speed
            and control output every 2 ticks, and whole debug every second.Icrease dead band max to
            150 RPM
    3.16    07/08/2001. Modified behaviour of feather reverse. Now recovers almost correctly
    3.17    13 & 14/08/2001. Further modification to exiting feather state. Remove minimum speed requirement
            when the feather or coarse stop is set, so that, regardless, controller can recover from a feather
            Also subtracted idle current from the data sent through to PC software
    3.18    Hardware mods to current sense circuit. Filter wasn't a filter really. Changed gain to allow larger
            current sense range. Fixed "overcurrent" indicator by removing motor current offset
    3.19    Modified exit feather. New parameter for drive time when in feather reverse. Drives once, tests
            coarse stop and tries again if not cleared. Change to gain on current sense. Minor changes to
            overcurrent indications.
    3.20    Replace engine speed check on feather reverse, also do a check before automatically going to
            feather reverse from a feather
    3.21    Fix for exit feather. Forgot the *10 in comparing measured RPM with minimum controllable RPM
    3.22    Added control word, table equation and divisor storage. Non Feathering version based on control
            word bit 0 set..
    3.23    Altered defaults to something more sensible, also added security code... Modified PWM brightness
            so that increased voltage on backlight means increased light.
    3.24    Scaling for pressure sensor
    3.25    Added manifold pressure control system.
    3.26    Added detection of version 3 PCB, along with changes to watchdog timer chain
    3.27    Stopped non feathering version trying to do a feather reverse
    3.28    Added current gain and offset factors in E2ROM
    3.29    Turned off drive altogether at idle
    3.30    Do not start if parameter file invalid
    3.31    V4 board software Initial  Software
    3.32    V4 First Release Software
    4.01    V4 board official first release
    4.02    Added fine&coarse or fine&feather stops open together as error (open motor)
            states
    4.03    Check for non-feather version fo above check
    4.04    Additions for Slave device operation
    4.05    Removed extraneous tx messages that may have interfered with slave device operation
    4.06    Moved remote device coms to software serial port
    4.07    Changed to synch to set speed rather than actual speed
    4.08    Only update set speed on slave if master is in a flight mode (TO/CL/CR/HO) AND setspeed
            is non-zero
    4.09    Implementation of BETA reversing mode
    4.10    Fix for wrong open state
    4.11    State display for beta mode, and change to triggering system
    4.12    State display change and change to beta exit trigger
    4.14    Fix to exit beta (use timer, and don't use Feather lower drive to exit) and state display
    4.15    Alter drive state in beta mode so as not to look at feather manual switch, also fixed
            drive indication in beta exit mode
    4.16    Rewrite Proportional control module to full PID. Added check for missed pulses in
            RPM inputs, Added start pulse for motor drive (0..10mSec ful drive controlled by timer A3)
    4.17    Check Coarse stop during Beta mode exit. Remove missing pulse detection
    4.18    Modified Beta mode, so that if speed goes over limit it drops relay out
            when in beta mode, also provides overspeed warning and will not go into beta
            mode if overspeed
    4.19    An overcurrent in BETA mode requires a reset to clear
    4.20    Changed Beta Indication to Fine LED
    4.21    General rework on status indications to help BETA mode indicators
    		Also set correct stop for BETA_EXIT, and latched Overcurrent in BETA
    4.22    Added different parameter for max RPM when beta is engaged (as opposed to
            max RPM to engage)
    4.23    Retain overcurrent indicator for Beta
    4.24    Add a hold speed parameter setting, change feather drive indicaton to 5HZ
    		flashing
	4.25	Add Xoar serial packet logic, special Xoar version is 6.25 and 6.26 (debug). MHH
	4.27	Add ATSPEED logic to generate RPM signals. 20-Oct-15
	4.28	Add ATLOG command 26-Oct-15
			Add ATREMOTE=n command. If n non zero, and AC200 is remote version (8.nn) then remote commands will
			not timeout.
	4.29	Add Brake logic. 29-Oct-2015
	4.30 - 4.40
			Add ATPWM=n		command, where 0 = 62.5 kHz and is new default, 1=31.25 kHz, 2=20.833 kHz and is old default
			[Note: ATPWM=n is now ATPWMR=n to be consistent with param logic when updating from AC200User
			Add SoftStart logic to drive.c
			Add Softbraking logic to drive.c
			Add Ctl_logic as alternative to PID logic for automatic RPM control
			Add AC200User:Misc->New Control Val.
				Value of zero = use existing PID logic
				Value of between 20 (2.0) and 200 (20.0) is parameter which controls sensitivity of Ctl_logic.
				This value should be between 50% and 65% of the maximum RPM/Tick rate of change, which can be obtained using
				ATCTUNE command (below).

	4.41	Add ATCTUNE=n where if n is non-zero AC200 will report RPM/Tick rate of change which is used
	        in conjunction with Manual FINE/COARSE and full throttle to get a starting value for
	        ATCTL value.
			Add AC200User:Misc->PWM rate parameter to allow PWM rate to be softcoded. (See ATPWM=n).26-Nov-2015
			Change param.c:setParameter:eepromwrite so that only a single parameter is updated as seemed to be causing timing error.27-Nov-2015
			sstate.c:motorCurrent test restored to < 50 because LED display was not flashing green when stop and motor running. 27-Nov-2015
	4.42    Release. 27-Nov-2015
	        Coarse LED was flashing red at 5 Hz when driving in to Beta. Fix was to check fs != BETA in LED feather logic.
	        Tues 8/12/15
 	7.43    Is remote positioning version developed for Google.
	4.44    Change Xoar remote from compile switch to parameter (REMOTE_COMMS_TYPE).	MHH:29/10/2016
	        Change REMOTE compile switch to always on, but make REMOTE control a parameter.
	        	REMOTE_COMMS_TYPE == 0 -> No remote control
	        	REMOTE_COMMS_TYPE == 1 -> Normal remote control
	        	REMOTE_COMMS_TYPE == 2 -> Xoar remote control
	4.45	Port to AC210. Add Diagnostics (DIAGS.C). Add Disable Diags option to AC200User. MHH:8/12/2016
			Speed Parameter update by only writing once at the end of parameter list.
			New PC programs: AC210Programmer, AC200Diagnostics
    4.46	Add flash and serial logging to AC210 and limited logging via serial port to AC200. MHH:16/02/2017
            Add log file ticks/sample parameter to Misc section of parameters
            Automate tune in Diagnostics.
    4.48    Sep 2017
            Add Hardware revision logic (AC210_5B)
            Modify AC210 PCB to change value of R71 from 3K to 4.7K as it was too low for voltage divider for battery voltage
            Note: Hand modified all AC210_A boards R71 from 3K to 4.7K
			Add AC2_TEST logic in AC2_TEST.C
			Integrate BETA and Reversing
			Add new Remote Control Logic.
	4.50    Tuesday 28/11/2017
			Add AutoGyro logic. See AC200AutoGyro.docx March 2018
	4.52	 02/04/2018
			Modifications for 24V:
			Change supply and dimmer calcs to allow for different voltage splitters (analog.c) 18/07/2018
			Use MH_ATSPEED ifdef to comment out all ATSPEED related routines as already have ATMMAX parameter (MC_MAX_PWM)
			Modify Soft Start logic to use target speed, not 255, as PWM limit.
			Modify Soft Start logic to use 10 cycles, not 5 to start (fifth of second vs tenth of a second)
			Modify Brake logic to use 10 cycles, not 5.
	4.53	18/07/2018

			09/08/2018:
				Modify Brake logic to use 10 cycles, not 5:
				Modified AT_SoftStop_val from 5 to 10, but this has no effect, so restore to existing value.
				Modified AT_BRAKE_val from 5 to 10, to use 10 cycles.
				Auto control was hunting on positioning hub with Maxon motor/gearbox and no load.
				Changed control.c:CTL_MIN_PWM_VAL from 180 to 50. Not sure how this will affect global motor, or loaded motor.
				control.c: Change lines with:
					currentControl = 255;	// default;
	                to:
	                currentControl = 	getParameter (MC_MAX_PWM);	// default MHH:09/08/2018
	        10/09/2018:
	            Problem: AC210 not seeing low power when using Maxon with no load.
	            Increase sensitivity by defining low power cutoff value:ADC_ZERO_CURRENT_VAL=4 in ac210_adc.c
	         13/09/2018. ac210_rpm.c: if rpm below 600, was setting to zero. Comment out that line.
			19/09/2018. Added calibration logic for positioning. ac210_sig60.c. When no RPM and in manual mode, move blade from full fine to
			    full coarse, then back again to full fine. This tells AC200 where stops are. AC200 will blink Feather light green 3 times
			    to confirm position is correct, or red 3 times to show changing record of stop positions.
			    Add Encoder Revision number to AC200RemoteTest, as well as ability to set position.
			20/09/2018. Modify drive.c:setDrive() [AC2TEST only routine] to directly control drive and speed, as AC2TEST does not use
			    soft-start/brake routines. Also modify setDriveToFeather() to directly control speed if AC2TEST in progress.

	4.54 20/09/2018
			24/09/2018. Added ATSYS=4 to display SIG60_input_errors
			02/11/2018. Modified RPM interrupt logic to use one similar to mhuart0.c version, derived from encoder logic as the moving
			    average logic was causing the logging to generate new runs by giving incorrect RPM at low levels.
				Modified Ctl_logic() in control.c so that if drive state is FINE and FINE_STOP is active or drive state is COARSE and
				COARSE_STOP is active then it bypasses normal control logic and sets PWM to max, as without bypass the controller was
				trying to move FINE or COARSE in small increments which caused the FINE/COARSE LED to blink ORANGE. With the modification
				the LED now blinks GREEN.
				Modified AC210_eelog() in log.c so that speed sampling is slowed only when in AUTO and dState == MD_IDLE and no stops on,
				not depending on if motor was running.
				Modified scaledValue() in analog.c so that if motor current less than 20 ma it is set to zero. Maxon at zero load uses
				about 50 ma.
			03/11/2018. Changed CTL_MIN_PWM_VAL from 50 to 200 in control.c so that AC200 will give longer minimum FINE/COARSE bursts.
			    With 50 a minimum burst (for fine adjustment) only lasted 1 tick. The effect was FINE/COARSE leds flashing orange on and off
			    quickly, but very little movement of blade.
                Add an extra test for "being_driven" in updateSystemState() in sstate.c to check if any current is being used or Drive_wait_flag
                is set which indicates braking as the state test was being run while braking or current present which was generating false
                FINE and COARSE stops (LED was set to GREEN).

            09/11/2018. Changed ADC sample rate back to 1000 Hz from 5000 Hz, changed CURRENT sample size to 20, changed ADC filter factor
                from 90% to 80% to try and fix current measuring problems such as high offset needed, slow response.
      4.55 09/11/2018
              Modify AC200 logic board by adding 47uf cap to 3.3V line.
              Modify current measurement logic to handle non-zero values when there is no current.
      4.56 19/11/2018
              Modify new current measurement logic to use offset
      4.57 24/11/2018
			  Modify current measurement logic to handle new current sense amplifier in ac210 5H series. Note 5H and greater will need a
			  single Current Sense resistor of 15 milli-Ohms instead of 4 * 0.39 Ohms in existing power boards.
              Add Real Time Clock logic (includes parameter in Misc tab)
              Add logic to display AC210 board letter (eg 5H), also modify AC200User and Ac200Diagnostics to display
              Modify current measurement logic to handle low values from D1 boards.
      4.58 11/01/2019
      	  	  22/01/2019. Minor changes to AC210_eelog() in log.c to reduce unnecessary log entries for minor changes in rpm.
      	  	              Also, change tolerance for auto logging from +/- 1% to +/- 0.2% when not active.

	       	  07/06/2019. Fix Xoar status = 0 (should be 4) when RPM=0 by taking out state check in leds.c
	       	              Modify Xoar cycle rate from 1Hz to 5Hz to see if that fixes a problem. main.c
	       	  28/06/2019. Add Xoar2 protocol
	       	  10/07/2019. Added REVERSE to Xoar2 protocol.
	  4.59  11/07/2019
	          17/07/2019. Modify FEATHER+REVERSE logic so that when exiting REVERSE mode that it works correctly.
	                      Also modify feather.c so that when changing from FEATHER to REVERSE it will exit feather.
            19/07/2019. Type 'D' PCB was reporting current when there wasn't any. Needed an offset of -5 or -6. Modify ac210_adc.c to set
                        current to zero if less than 8. Previous test was 2.
      4.60 19/07/2019
      	  	 27/07/2019. When Xoar coming out of Feather, sometimes had XoarStatus of 13 (correct) and sometimes 1. Add test for
      	  	             motor drive state = MD_FEATHER_REVERSE in reverseFeatherProc().
      4.61 27/07/2019
       	   	 05/08/2019. Change rawval test to < 20 (was < 10) for PCB type 'D' in ac210_adc.c.
      4.61A  05/08/2019
            19/08/2019. Autogyro reverse not working. Change all occurrences of S_STOP_FEATHER to S_STOP_REVERSE in AutoGyro_reverse_control()
                        in feather.c as stop bit changed for feather and reverse logic.
      4.62 20/08/2019
            28/08/2019. Changes to allow Auxiliary Serial and CAN to work on same pins on 10 pin connector.
            24/10/2019. Change MAX_ENGINE_SPEED hard coded value from 10,000 to 20,000
      4.63 24/10/2019
           06/03/2020 Modify braking logic in ac210_drive.c to soften braking (spread over 10 cycles instead of 5), in case braking was
                      causing premature brush wear in Maxon motors.
                      Add current limiting logic to sstate.c (MH_CURRENT_LIMIT)
                      Add ability for Xoar2 logic to reset overcurrent trip
                      Add XOAR_ERROR_CURRENT_OVERLOAD (17) to Xoar errors. This indicates overCurrentTrip flag has been set.
      4.64 06/03/2020
           09/03/2020 Modify sstate.c to remove current control 70% option as brings total current below CTL_MIN_PWM_VAL (200),
                      so control routine may not work correctly.
           06/04/2020 Restore braking logic to previous as was "hunting" when I tested without a load.
           07/05/2020 Comment out MH_CURRENT_LIMIT logic in sstate.c as could be masking over-current problem.
      4.65 07/05/2020
           29/10/2020 Record supply voltage (eg 12.5V) in log file. AC200Diagnostics modified to display.
           26/11/2020 Change all long rpm variables that had an implied decimal point (eg 5000.0 rpm) to WORD variables with no
                      implied decimal point.
           26/11/2020 Improve RPM control logic:
                      Old logic:
                      if((target_rpm - deadband) < (current_rpm + pipeline_rpm) < (target_rpm + deadband)) then
                       	   set_idle()
                       	   return
                      endif
                      This meant that if (say) current_rpm = 4949, target_rpm=5000, deadband=50, then pipeline_rpm needs just to be >=1 to set_idle
                      which resulted in very small amounts of "current time" to try and correct rpm (resulting in flickering of LED COARSE or LED FINE)
                      and also did not allow control to any closer than +/- 50 rpm for rotax 912,

                      New logic:
                      target_min_rpm = target_rpm - deadband;
                      target_max_rpm = target_rpm + deadband;
                      if(driving_fine) then		// ie increasing rpm
                      	  target_min_rpm = target_rpm - (deadband/2)
                      endif
                      if(driving_coarse) then	// ie decreasing rpm
                      	  target_max_rpm = target_rpm + (deadband/2)
                      endif
                      if(target_min_rpm < (current_rpm + pipeline_rpm) < target_max_rpm) then	// pipeline_rpm can be negative.
                       	   set_idle()
                       	   return
                      endif
                      Note: This has allowed RPM control to +/- 10 RPM for Rotax 912.
           26/11/2020 Add Master/Slave logic.
                      Master/Slave, along with serial ports are now parameters, on same tab as Control value.
                      There are 2 serial port options, both which use the CN5 (AUX) pins.
                      Option 1 is UART3 which has no RS232 transceiver, and uses CN5 pins P3 (tx),P4 (rx) and P5 (Gnd)
                      Wiring is:
                      P3 to P4
                      P4 to P3
                      P5 to P5
                      Option 2 is UART2 which has an optional RS232 transceiver in AC200 high power logic boards made from 08/2020 on,
                      and uses CN5 pins P2 (tx),P9 (rx) and P5 (Gnd)
                      Wiring is:
                      P2 to P9
                      P9 to P2
                      P5 to P5

                It is recommended that option 1 is used as this works with all AC200 V5 hardware. Option 2 may be useful if UART3 is being
                used for something else, or the AC200 controllers are more than a meter apart and rs232 transceivers are needed.

				Note: If Master AC200 is not in Manual or Feather mode then
				         it will always send via selected serial port (1 or 2) actual rpm data to the slave AC200.
				      endif
				      Slave AC200 will always receive via selected serial port (1 or 2), but will ignore master commands unless in Hold mode.
				      if Slave AC200 is in hold mode then
				        if(it is receiving valid commands from Master AC200) then
				          Feather light will be set to green
				          It will set it's target_rpm to AC200 Master actual rpm and use parameter slave_deadband for tolerance
				        else
				          Feather light will be set to red flashing 1 Hz.
				          target rpm will be set to hold_rpm with normal deadband
				        endif
				      endif
      4.66 26/11/2020
		   22/12/2020 Modify ac210_adc.c to take 5000 samples/sec instead of 1000. Modify adc filter value from 80 to 60 to make more
			           sensitive measurements.
			          Add slip-ring check logic for both state and current modes. Add slip-ring value to logging.
			          Changed soft-start so now only does one tick at start value.
			          Change control logic so that power-tick is used up over 16 ticks, not 32.
      4.67 22/12/2020
           11/01/2021 Remove slip-ring check logic for state mode.
                      Make slip-ring log value a multiple of 10, to reduce log space
                      Add Xoar Status codes 23 "Slipring error while pitch decreasing" and
                                            24 "Slipring error while pitch increasing"
                      Comment out #define MH_INT_CURRENT, so current interruption doesn't happen
      4.68 11/01/2021
			15/03/2021 Add Relay De-icing logic. Note: Changes to AC200User to support new De-icer tab and
			           AC200Xoar2Test to allow new Xoar instruction code
			           X2_INSTRUCTION_CODE_SET_DEICE and new error 18 XOAR_ERROR_DEICER_NOT_ENABLED.
       4.69 15/03/2021
			11/08/2021 When COARSE microswitch open, and wanting to move FINE, if Feathering then was using reverse-feather logic.
			           Have added a test to reverseFeatherProc() in feather.c so that exits reverse-feather logic if rpm > min_rpm.
       4.70 11/08/2021
            22/12/2021 Map XoarStatus 23 (Slipring error while pitch decreasing) to XoarStatus 1 when sending pkt to Xoar
                       Map XoarStatus 24 (Slipring error while pitch increasing) to XoarStatus 2 when sending pkt to Xoar
                       as their software has not been updated to handle XoarStatus > 22.
                       See #define MH_MAP_SLIPRING_STATUS in comms.c
		    09/03/2022 if Xoar2 change setspeed then ensure that it is within minimum and maximum hold speed (remote.c)

       4.71 09/03/2022
            22/04/2022 Bug fix: In BETA, parameter BETA_MAX_RPM_ENGAGED was being multiplied by 10 in feather.c:featherControl(). This is a result of
                       changes introduced in 4.66 on 22/12/2020 to change internal RPM from value with an implied decimal point (true RPM * 10).
            22/04/2022 Xoar2 remote reverse logic: Add checks so that AC200 checks that if BETA_CHECK_RPM is true then actual RPM is below BETA_MAX_RPM
                       before starting reverse movement (Xoar error 19 is too high) and also when reversing RPM must be below BETA_MAX_RPM_ENGAGED or
                       reversing will be stopped and Xoar error 20 returned. Changes are in remote.c:XoarReverseCode()and remote.c:remoteControlCycle()
                       Reversing can be restarted using the Xoar2 reverse command.
        4.72 22/04/2022
             27/07/2022 Xoar had a situation where "head_page" was set to zero. Added routine:ee_find_head_page_from_last_flash_sector_erased(void)
                       to ac210_log.c to use last sector erased to find where head_page should be and update.
         4.73 27/07/2022: Note: Only sending to Xoar, wait for general update.
			23/09/2022 Add ability to reset overCurrentTrip flag from standard remote control logic. New keyword "RC_RESET_OC".
					   Logic in remote.c:process_RC_Standard(). Note that It also allows setting overCurrentTrip flag on for testing.

		Version = AC200_5_473A.BIN


		4.74 18/11/2022. Support for SIG100 - first test release...

 MHH:20/05/2024. A lot of brushless work, plus recent change to control algorithm from rpm/tick to P+D, plus change to revision format.
 Note VS programs that needed changing for revision change: AC200User,AC200Diagnostics,AC200Programmer,AC200RemoteTest

		10.100 20/05/2024.
			Add logic to use PWM to slow brushed pitch change motor to 50% pwm if close to target RPM.
		10.101 22/05/2024
		    Remove 50% PWM logic as will not move motor under load.
		    Change speed control to PD type (Position, Differential)
		    Change Brake logic so that a minimum of 16 ticks braking must be completed after each speed control sequence.
		    This seems to help prevent "hunting".
		    Add NCV (New Control Value) logic in Drive_softmode_start() so that:
		    if(projected error is within 3% of setSpeed) then
		    	if(NCV_hi_digit == 1) max ticks = 4
		    	if(NCV_hi_digit == 2) max ticks = 3
		    	if(NCV_hi_digit == 1) max ticks = 2
		    endif
		10.102 29/05/2024
*/

volatile BYTE waiting;
// ticks counts up to 49, (1 seconds worth) so we can flash at 1 Hz
BYTE timerTick;
ULONG TimeInSeconds;
WORD debug;

// Flags the 0.5 second state dump
BYTE monitor;

// Flags an overcurrent state. reset by switching to manual
// and back to an auto mode
BYTE overCurrentTrip = 0;
#if BETA_VERSION
	BYTE betaOverCurrent = 0;	// Beta mode overcurrent NEVER reset
#endif
WORD overCurrentDir = 0;

void Set_overcurrent_dir(WORD v)
{
	overCurrentDir = v;
}

BYTE pcb_Version = PCB_VER_1;

BYTE invalidConfig = 0;

void UpdateStatsRecord(void);

void detectPcbVersion (void)
{
#ifdef AC210_PORT
	pcb_Version = PCB_VER_4;		// LPC1768 version
	AC210_dummy_str(Version_id);	// Need to reference Version_id otherwise not included in binary file!!!
//	printf("Version_id:%s\r\n",Version_id);	// Need to reference Version_id otherwise not included in binary file!!!
#else
	// Version 3 and beyond boards (the rectangular ones) have a loopback between
    // TxD and RxD on serial port 0
    int count;

    // set up serial port for 19200 baud
    U0BRG = 51;
    U0C0 = 0x18;
    U0MR = 0x05;
    U0C1 |= 0x05;

    U0TB = 0x53;
    // delay for transmission
    for (count = 5000; count; count--)
    {

    }
    //check receive character
    if (0x53 == (BYTE)U0RB)
    {
        pcb_Version = PCB_VER_2;
    }
    // turn off the port
    U0C1 = 0;

    // Version 4 and beyond boards (the rectangular ones) have a loopback between
    // pins 1 and 100 P96, P97
    if (pcb_Version == PCB_VER_2)
    {
        // set up 96 as out, 97 as in
        PRCR |= 0x04;
        PD9  &= ~0xC0;
        PRCR |= 0x04;
        PD9  |= 0x40;

        P9 |= 0x40;
        if ((P9 & 0x80) == 0x80)
        {
            P9 &= ~0x40;
            if ((P9 & 0x80) == 0x00)
            {
                // swap directions
                PRCR |= 0x04;
                PD9  &= ~0xC0;
                PRCR |= 0x04;
                PD9  |= 0x80;
                P9 |= 0x80;
                if ((P9 & 0x40) == 0x40)
                {
                    P9 &= ~0x80;
                    if ((P9 & 0x40) == 0x00)
                    {
                        // Version 4
                        pcb_Version = PCB_VER_3;
                    }
                }

            }
        }
        PRCR |= 0x04;
        PD9  &= ~0xC0;
    }
#endif
}

//BYTE pcbVersion (void)
WORD pcbVersion (void)	// MHH:06/01/2019
{
//    return pcb_Version;
    return pcb_Version * 100 + AC210_hardware_version;
}

WORD mainTimerCount;
WORD mainTimerLimit;
//#define EXTREME_MEASURES
#ifdef EXTREME_MEASURES
void setMainTimerLimit(WORD w)
{
	txDebugVal("setmainTimerLimit",w);
	mainTimerLimit = w;
}
#endif

// Note, the following 2 routines are NOT to be declared with the pragma interrupt
// scheme as the one above is. The ISR parts are done in ncr???.a30

#ifdef AC210_PORT
void Drive_softcheck(void);

void watchIt (WORD from)
{
#ifdef MH_XXX
	if(from != 1020)	// MHH:29/05/2025. Debug!!!
	{
		PRINTF("w:%d\r\n",from);
	}
#endif
	AC210_watchIt(from);
}
//extern WORD currentState;
void watchIt2 (WORD from)
{
#ifdef MH_XXX
	if((currentState & S_STOP_COARSE) == 0)	// MHH:22/05/2025. Debug!!!
	{
		mh_debug();
	}
#endif
	AC210_watchIt2(from);
}
#else
//-------------------------------------------------------------------------
extern void setLEDS (BYTE state);	// In leds.c
static void resetWatchdog(void)
{
	        WDTS = 0x00;
}

static void wait_ms(WORD ms)
{
	ULONG i;
	for(i=0;i<ms*100;i++) resetWatchdog();
}
//-------------------------------------------------------------------------
// derived from AC210 code in ac210.c
void FlashAllLEDS(int tenths)
{
	int f;
	int flashes=tenths;

	for(f=0;f<flashes;f++)
	{
		setLEDS(127);
	    wait_ms(50);
		setLEDS(0);
		wait_ms(50);
	}
}
//-------------------------------------------------------------------------
#ifndef AC210_PORT
static BYTE Abort_count;
void Abort(ULONG err_num,char far *msg)
{
	char buf[20];


	if(Abort_count++ == 0)		// Avoid any possible recursion
	{
		resetWatchdog();
		txDebug("Abort:");
		txDebug(msg);
		sprintf(buf,":%d\r\n",err_num);
		txDebug(buf);
		resetWatchdog();
		DiagsError(err_num+100000);
		FlashAllLEDS(50);		// same as ac210
	}

// A bit dangerous to reset M16 as if the abort is generated too early it may stop FlashLoad working
// Could always put a test in to see if was operating normally before abort....

//    PRCR |= 0x02;
//    PM0 |= 0x08;

}
//-------------------------------------------------------------------------
void M16_Reboot(void)
{
    PRCR |= 0x02;
    PM0 |= 0x08;
}
#endif
//-------------------------------------------------------------------------
static BYTE wdCount=0;		// Watchdog count Ithink
static BYTE AC200_watchdog_reset_count;
static WORD wdFrom;
void watchDog (void)
{
    // get here when watchdog underflows....
    // reset the processor after count of 20 (equals approx 5 seconds)
	if(mainTimerLimit == 0)		// So we can disable watchdog and slow maintimer
	{
	    wdCount++;
	}
    if (wdCount > 20)
    {
//		txDebug("WD:\r\n");
//		Delay_ms(100)	;
		if(AC200_watchdog_reset_count++ == 0)
		{
	        WDTS = 0x00;
//		printf("AC210_Watchdog_Reset:%05d\r\n",AC210_watchdog_from);
			DiagsError(wdFrom+200000);
			FlashAllLEDS(50);		// same as ac210
		}
//        wdCount = 0;
        PRCR |= 0x02;
        PM0 |= 0x08;
    }
    else
    {
        WDTS = 0x00;
        txDebug("wd:\r\n");
    }
}

void dummyIntOut (void)
{
//    captures all unintentional interrupts from fixed interuupts (in flash)

}

#pragma INTERRUPT dummyLocalInt
void dummyLocalInt (void)
{
//    captures all unintentional interrupts from other interuupts (in RAM)

}

//void watchIt ()
void watchIt (WORD from)	// MHH:28/11/2016
{
	wdFrom = from;

	// reset the watchdog
   	WDTS = 0x00;
   	wdCount = 0;
}

void watchIt2 (WORD from)	// MHH:16/02/2017
{
	wdFrom = from;
}

void initVectorTable (void)
{
    // initialise the interrupt vector table
    int cnt;
    for (cnt=0; cnt < 64; cnt++)
    {
        vector_table[cnt] = (unsigned long)dummyLocalInt;
    }
}
#endif

unsigned long TimerCount;

BYTE timerTicks (void)
{
    return (timerTick);
}
#ifndef AC210_PORT
#pragma INTERRUPT mainTimer
#endif
void mainTimer (void)
{
    // main timer interrupt, goes off 50 times per second
    // and is used to control main loop

	TimerCount++;
	if(mainTimerLimit == 0)	// Normal path
	{
    	waiting = 0;
		return;
	}
	mainTimerCount++;	// This gives option of slowing for debug
	if(mainTimerCount >= mainTimerLimit)
	{
		waiting = 0;
		mainTimerCount = 0;
	}
}
int LMT87_convert_millivolts_to_celcius(int millivolts);
extern WORD Auto_index;
extern WORD Auto_throttle;
extern WORD AC200_slider_percent;
extern int AC200_slider_angle;
WORD AC200_display_slider_percent=200;	// Force initial display
extern WORD AC200_slider_value;
void dumpState (int type)
{
	char sbuf[60];
    char tmp[10];
    WORD rv, xx;
    long ss;
#if MAP_VERSION
    PressureValue pv;
#endif

    if (!type)
    {
        sprintf (sbuf, "STATE=");
    }
    else
    {
        sprintf (sbuf, "STATEF=");
    }

    if (!type)
    {
    	rv = Auto_operating_mode();
//    	rv = operatingMode();
        sprintf (tmp, "%d,",rv);
        strcat (sbuf, tmp);
    }
    rv = controlState(&xx);
    sprintf(tmp,"%d,", rv);
    strcat (sbuf, tmp);
    rv = driveState();
    sprintf(tmp,"%d,", rv);
    strcat (sbuf, tmp);
    if (!type)
    {
//        rv = featherState();
        rv = Control_type;
        sprintf(tmp,"%d,", rv);
        strcat (sbuf, tmp);
        rv = scaledValue (A_MOTOR_CURRENT);
        if(Sig100_connected)
        {
        	if(rv < 100) rv = 0;
        }
        sprintf(tmp,"%d,", rv);
        strcat (sbuf, tmp);
        rv = systemState ();
        sprintf (tmp, "%d,%d,", ((rv & S_RUN_FINE) ? 1 : 0),
                 ((rv & S_STOP_FINE) ? 1 : 0));
        strcat (sbuf, tmp);
        sprintf (tmp, "%d,%d,", ((rv & S_RUN_COARSE) ? 1 : 0),
                 ((rv & S_STOP_COARSE) ? 1 : 0));
        strcat (sbuf, tmp);
        sprintf (tmp, "%d,%d,", ((rv & S_RUN_FEATHER) ? 1 : 0),
                 ((rv & S_STOP_FEATHER) ? 1 : 0));
        strcat (sbuf, tmp);
    }
//    ss = currentSetSpeed ()/RPMFACTOR;
    ss = currentSetSpeed ();
    ss *= 10;		// MHH:20/11/2020.
    sprintf (tmp, "%ld.%d,", ss/10, ss %10);
    strcat (sbuf, tmp);
//    ss = currentActualSpeed ()/RPMFACTOR;
    ss = currentActualSpeed ();
    ss *= 10;		// MHH:20/11/2020.
    sprintf (tmp, "%ld.%d,", ss/10, ss %10);
    strcat (sbuf, tmp);

    sprintf (tmp, "%d", xx);
    strcat (sbuf,tmp);

#ifdef AC210_PORT
#ifdef MH_SIG60
    if(AC210_SIG60_Connected())
    {
    	float angle = AC210_SIG60_angle();
    	sprintf(tmp,",%3.1f\r\n",angle);
        strcat (sbuf, tmp);
    }
#endif
    if(Sig100_connected)
    {
    	float angle = AC210_SIG100_angle();
//    	sprintf(tmp,",%3.1f\r\n",angle);
    	sprintf(tmp,",%3.2f\r\n",angle);		// MHH:19/05/2023
    	strcat (sbuf, tmp);
    }
#endif

#if MAP_VERSION
// Pressure Sensor
    if (manifoldPressureEnabled())
    {
        pv = manifoldPressureCheck();
        rv = scaledValue (A_MANIFOLD_PRESSURE);
        sprintf (tmp, ",%d,%d.%d", pv, rv/10, rv%10);
        strcat (sbuf, tmp);
    }
#endif

    strcat (sbuf, "\r\n");
    txDebug (sbuf);
#ifdef MH_RTC		// Note: May want to check AC200User version and/or if RTC is enabled and/or hardware there and working.
    uint32_t rtc_secs;
    rtc_secs = AC210_RTC_secs();
    if(rtc_secs != 0)
    {
        PRINTF("AC200RTC=%ld\r\n",rtc_secs);
    }
#endif
	if(getParameter(DEICER_ENABLE) == 2)
	{
		WORD adc_val = rawValue(A_DIMMER);
		int deicer_amps = (adc_val * 452 + 500)/1000;		// Amps with 1 dec place
		PRINTF("AC200DEI=%ld\r\n",deicer_amps);
	}
	if(Sig100_connected)
	{
//		int voltage = (((ADC_adjusted_voltage << 4) + 11))/12;	// To 1 dec place
		float fvoltage = SIG100_get_hub_voltage();
#ifdef MH_XXX	// MHH:25/06/2025
		float fvoltage = (float)(ADC_adjusted_voltage * 16);
		if(Enc_data.voltage_correct_pct == 0) Enc_data.voltage_correct_pct = 100;	// Set default
		fvoltage  = (fvoltage * Enc_data.voltage_correct_pct)/100;
//		fvoltage /= 10.954;		// By experiment, should have an implied decimal place
		fvoltage /= 10;		// MHH:25/06/2025 Now that we have voltage_correct_pct
#endif
//		int voltage = (int)(fvoltage + 0.5);
//		int voltage = (((ADC_adjusted_voltage << 4) + 10))/11;	// To 1 dec place, after pull-up in hub pcb disabled 03/01/2023
		int voltage = (int)(fvoltage);	// MHH:22/09/2025. To be consistent with log logic.
		voltage = ((voltage+1)>>1)<<1;	// Get rid of least significant bit as jittery in AC200User

		PRINTF("AC200HBV=%d\r\n",voltage);
		PRINTF("AC200HBL=%d\r\n",SIG100_line_percent);


		int temp_adc = ADC_adjusted_temp << 4;	// get ADC measured value to 8 significant bits

		//  We know lpc845 is 3.3V, so convert millivolts = 3300 * adc_v / 4096

		int millivolts = (3300 * temp_adc) >> 12;

		int temp_celcius = LMT87_convert_millivolts_to_celcius(millivolts);	// Tricky, uses formula
//		temp_celcius = ((temp_celcius+1)>>1)<<1;	// Get rid of least significant bit as jittery in AC200User
		PRINTF("AC200HBT=%d\r\n",temp_celcius);
		if(AC210_remote_control_board)
		{
			temp_celcius = scaledValue(A_TEMPERATURE);
			PRINTF("AC200TEM=%d\r\n",temp_celcius);
		}
	}
	if(getParameter(TC_PORT))	// Throttle control?
	{
		if(Auto_index)
		{
			PRINTF("AC200THR=%d\r\n",Auto_throttle);
			PRINTF("AC200TIX=%d\r\n",Auto_index);
		}
	}
	PRINTF("AC200RUN=%03d\r\n",Stat_Rec.run_number);	// MHH:19/12/2023
    if(Sig100_connected)
    {
    	if(getParameter(SLIDER_CONTROL_ENABLE) == 1)
    	{
    		PRINTF("AC200SCV=%d\r\n",AC200_slider_value);
    		if(AC200_slider_percent != AC200_display_slider_percent)
    		{
    			AC200_display_slider_percent = AC200_slider_percent;
    	    	PRINTF("AC200SCP=%d\r\n",AC200_slider_percent);
    	    	float slider_angle = AC200_slider_angle;
    	    	slider_angle /= 10;
    	    	PRINTF("AC200SCA=%3.1f\r\n",slider_angle);
    		}
    	}
    }

}
//-------------------------------------------------------------
const float Cf433 = 0.00433;
const float Cf1358 = 13.582;

int LMT87_convert_millivolts_to_celcius(int millivolts)
{
	int temp_celcius;

	float Vtemp = millivolts;

	// Now to use formula on page 10 of LMT87 data sheet.

	float exp1 = (2230.8 - Vtemp);
	float exp2 = (Cf1358 * Cf1358) + 4 * Cf433 * exp1;
	float exp3 = sqrtf(exp2);
	float exp4 = Cf1358 - exp3;
	float exp5 = 2 * (-Cf433);
	float exp6 = exp4/exp5 + 30;
	temp_celcius = (int)(exp6 + 0.5);

	return temp_celcius;
}
int LMT87_convert_celcius_to_millivolts(int celcius)
{
	float T = celcius;
	float exp1 = (T - 30) * 2 * (-Cf433) - Cf1358;
	exp1 = - exp1;
	float exp2 = exp1 * exp1;
	float exp3 = exp2 - Cf1358 * Cf1358;
	float exp4 = exp3 / (4 * Cf433);

	float Vtemp = 2230.8 - exp4;
	int millivolts = (int)(Vtemp + 0.5);
	return millivolts;
}
// ------------------------------------------------------------
#if CLI_LOG > 0
// Note: May be useful to send State eg stopped coarse, running coarse
// Also, we do need to know direction
BYTE CLI_log;
BYTE EEPROM_log;
extern WORD lastSpeed;
extern BYTE lastDir;
void CLI_SendLine(void)
{
    char sbuf[60];
//	WORD systemstate;
    long setspeed,actualspeed;
    MotorDriveState dstate;

    setspeed    = currentSetSpeed ();
    actualspeed = currentActualSpeed ();
// An alternative would be to examine lastDir, although this is output to drive pins.
//    systemstate = systemState ();
    dstate = driveState();
// Note: We could decode systemstate here, but for speed will do at other end
//    sprintf (sbuf, ":D:%4ld,%4ld,%4d,%4d,%3d\r\n", setspeed/10, actualspeed/10,lastSpeed,systemstate,lastDir);
//    sprintf (sbuf, ":D:%4ld,%4ld,%4d,%4d\r\n", setspeed/10, actualspeed/10,lastSpeed,systemstate);
    sprintf (sbuf, ":D:%4ld,%4ld,%4d,%4d\r\n", setspeed/10, actualspeed/10,lastSpeed,(WORD)dstate);
    txDebug (sbuf);
}
#endif

// ------------------------------------------------------------
long DelayCount;
void DelayRoutine(void)
{
	char delayBuf[32];
	if(timerTick == 0)
	{
		sprintf(delayBuf,"DelayCount:%ld\r\n",DelayCount);
		txDebug(delayBuf);
		DelayCount=0;
	}
	while(waiting)
	{
		DelayCount++;
	}
}
// ------------------------------------------------------------
void TestTimer(void)
{
#ifndef AC210_PORT	// probably not needed for port
	WORD t1,t2,t3;
	char tbuf[64];

	if(timerTick) return;	// Only once a second
	t1 = TB0;
	DelayCount=0;
	while(DelayCount++ < 1000);
	t2 = TB0;
	t3 = t1 - t2;
	sprintf(tbuf,"TestTimer: t1=%u, t2=%u, t3 = %u\r\n",t1,t2,t3);
	txDebug(tbuf);
#endif
}
// ------------------------------------------------------------
#ifndef AC210_PORT
unsigned long GetTime_ms(void)
{
	unsigned long time_ms;
	WORD t1;

	t1 = 39999 - TB0;
	t1 /= 2000;		// convert to millisecs
	time_ms = TimerCount * 20;	// 20 msecs/tick
	time_ms += t1;
	return time_ms;
}

#endif
// ------------------------------------------------------------
void Delay_ms(WORD ms)
{
#ifdef AC210_PORT
	wait_ms(ms);	// May have to take watchdog into account
#else

	unsigned long end_ms;

	end_ms = GetTime_ms() + ms;
	while(GetTime_ms() < end_ms) watchIt(WD_MAIN + 10);// make sure watch dog doesn't get us
#endif
}
//extern uint32_t RTC_secs;
//--------------------------------------------------------------------------------------------
uint32_t TimeInTicks;
static void IncrementTimer(void)
{
	if (++timerTick > 49)
	{
		timerTick = 0;
		TimeInSeconds++;
	}
	TimeInTicks++;
}
// --------------------------------------------------------------------------------------------
void checkForAllCommands(void)
{
	checkForCommand ();		// MHH:07/02/2025 ???
	checkForCommand2();		// MHH:21/11/2023. Remote control port = 1
	checkRemoteCANcommand();
}
//----------------------------------------------------------------------------------------------
short int Xoar_ticks_left;
#ifdef AC210_PORT
void AC210_main(void)
#else
void main(void)
#endif
{
    BYTE lc = LED_OFF;
    BYTE lccount= 0;
//    ULONG test;
    char tmp[20];
//    int i;

    // Set default interrupt vectors
#ifndef AC210_PORT
    initVectorTable();
    // enable interrupts
    enableInterrupt();
#endif

    // start watchdog
    watchIt(WD_MAIN+20);

    // pcb Version Detection
    detectPcbVersion();

    // initialise the serial port
    initComms();

    // Load parameters from EEPROM
    loadParameters ();	// MHH:12/07/2023. Now done in ac210.c, before serial init

    wait_ms(20);		// MHH:09/08/2023. Allow any output in serial internal buffers to complete before resetting serial.

    AC210_SerialInit();		// Set up 19.2k serial port
//    Wait_secs_no_watchdog(5);		// MHH:29/05/2025. Debug

    AC210_log_init();
    AC210_RTC_init();

    AC210_brushless_pin_init();		// No action if not brushless

//    AC210_SerialInit();		// Set up 19.2k serial port


    // start A/D
    initAnalog ();


    watchIt(WD_MAIN+30);

    // indicator initialisation..
    initLEDS ();

    // Init speed inputs
    initSpeed ();

    // System State monitoring
    initSystemState();

    // init switch inputs
    initDigital ();

    // PI
    initControl ();

    // feather
    initFeather ();

    // Init main drive control
    initDriveControl();

#if MAP_VERSION
    //Initialise Manifold Pressure tables
    loadManifoldPressureLimits();
#endif

#if REMOTE_VERSION
    initRemoteMode();
#endif

//    initAttachedDevices();

//    initBitSerial();

	initXoar();	// comms.c

	Sig100_init();

    timerTick = 0;
    waiting = 1;

    monitor = 0;

#ifndef AC210_PORT
    // set up a 20 mSec timer on TB0
    vector_table[26] = (unsigned long)mainTimer;
    TB0MR = 0x40;   // divide by 8, timer mode
    // load the count
    TB0 = 39999;    //16 MHz clock, freq /(n+1)
    // enable interrupt
    TB0IC = 0x04;
    // start the timer
    TABSR |= 0x20;
#endif

    sprintf (tmp, "Reset[%d]\r\n", invalidConfig);
    txDebug (tmp);

//	setMainTimerLimit(100);		// DEBUG!!!!
    L2PRINTF("Run:%d\r\n",Stat_Rec.run_number+1);	// It will get incremented

//    Wait_secs_no_watchdog(5);
    for (;;)
    {
#ifdef AC2_TEST
    	if(ac2_test_flag == AC2_TEST_ON)
    	{
#ifdef AC210_PORT
	    	p_Wait();
#else
	    	while (waiting);
#endif
            watchIt(WD_MAIN+35);

            IncrementTimer();

            waiting = 1;
            AC2_TEST_stage();
            continue;
    	}
#endif


//    	SendDiags();		// in comms.c. Only if flag set
        if (!invalidConfig)		// MHH:01/06/2017
        {
        	if(timerTick % 5 == 0)		// ten times a second
        	{
          		UpdateStatsRecord();		// In param.c
        	}
    		if(EEPROM_log)
    		{
    			if((timerTick+2) % EEPROM_log == 0) // The +2 should make it happen 2 ticks before diags
    			{
    		    	AC210_eelog();
    			}
    		}
        }
#ifdef AC210_PORT
    	p_Wait();		// Allows us to do some checks while waiting.
#else
    	while (waiting);
#endif


#ifdef EXTREME_MEASURES
    	if(mainTimerLimit)	// Debug I think
		{
//			TraceVal=1;
			for(i=0;i<100;i++) {
		        checkForCommand ();
				Delay_ms(10);
			}
		}
#endif

 //		TraceMsg("main:1");

        watchIt(WD_MAIN+40);

        IncrementTimer();

        waiting = 1;


#if MH_REMOTE_POS
		CheckRemotePosCommand();	// Do here so can reply in same tick.
#endif


#ifdef MH_DEBUG_POKE
		if(AT_POKE_val)
		{
	        checkForCommand ();
			AT_Poke();
	        if(AT_POKE_val == 3) {
				DelayRoutine();
			}
			if(AT_POKE_val == 4) {
				TestTimer();
			}
			if(AT_POKE_val == 5) AT_POKE_val = 0;
			continue;
		}
#endif

		bool enter_control_loop = true;

		if(ps.parms[REMOTE_COMMS_TYPE] == 5)	// Are we acting as a Serial to CAN converter?
		{
			enter_control_loop = false;
			invalidConfig = false;			// So we don't get red flashing leds
		}
		if(invalidConfig)	enter_control_loop = false;
//		if (!invalidConfig)
		if (enter_control_loop)
        {
            // detect mode changes
            updateOperatingMode ();
            // are we in feather
            if (isFeatheringOrReversingProp() || isBetaProp())
            {
                featherControl ();
            }

            if(isMasterProp())
            {
                watchIt2(WD_MAIN+42);
                AC210_master_check();
            }
            else
            {
                if (isSlaveProp())
                {
                    watchIt2(WD_MAIN+43);
                    AC210_slave_check();
                }
            }

#ifdef AC210_PORT
            watchIt2(WD_MAIN+45);
//            AC210_SIG100_check();

            if(timerTick == 0)		// Once a second
            {
                watchIt2(WD_MAIN+48);		// MHH:04/02/2021
                AC210_DeIcer_check();
            }

#endif

#if REMOTE_VERSION
#if MH_REMOTE_POS
			if(RemotePos_active)	// Possibly make this switch an eprom parameter?
			{
//				TraceVal = 1;
				RemotePos_ControlCycle();
//				TraceVal = 0;
			}
			else
			{
            	if (remoteMode())
            	{
                	remoteControlCycle();
            	}
			}
#else
            watchIt2(WD_MAIN+50);

            checkForAllCommands();

            AC210_SIG100_check();	// MHH:17/02/2025. Moved to here so have position and flags before remote control cycle
            AC210_SIG100_get_delta_angle();
            if(getParameter(REMOTE_COMMS_TYPE))	// If not = 0
            //            if (remoteMode())
            {
                remoteControlCycle();
            }
            else
            {
                watchIt2(WD_MAIN+55);
            	Auto_cycle_check();
            }
#endif
#endif
            // update the PI control
            watchIt2(WD_MAIN+60);
//            AC210_SIG100_check();	// MHH:23/05/2023. Moved to here so have position and flags before control cycle
            controlCycle ();
//            AC210_SIG100_send();	// MHH:23/05/2023.


            // transfer this to the motor drive

            watchIt2(WD_MAIN+70);
            if(TimeInSeconds != 0)	// Not for the first second as need to allow RPM to settle
            {
                updateDriveControl ();
                Drive_softcheck();
            }
            // system state check
            watchIt2(WD_MAIN+80);
            AC210_SIG100_send();	// MHH:25/02/2025.
            updateSystemState ();
        }
        else
        {
            // serial port command monitoring
            watchIt2(WD_MAIN+100);

            checkForAllCommands();

        }
//        TraceMsg("main:2");
        // reflect the system state in the LED display
        watchIt2(WD_MAIN+90);
        updateLEDS ();

//		TraceMsg("main:3");

//        checkBitSerial();
//		TraceMsg("main:4");
#ifdef MH_XXX		// MHH:21/11/2023. Looks redundant
#if CLI_LOG > 0
		if(CLI_log)
		{
			if(timerTick % CLI_log == 0) // depending on value, 5 = tenths of a sec
			{
				CLI_SendLine();
			}
		}
#endif
        watchIt2(WD_MAIN+120);
		if(AT_Ctl_tune)
		{
			if(timerTick % 10 == 0) // five times a second
			{
				Ctl_report_rate_change();
			}
		}
#endif

//#if XOAR_VERSION > 0
        watchIt2(WD_MAIN+125);

        bool xoar_send_packet = false;
        switch(RemoteCommsType)
        {
        case REMOTE_COMMS_XOAR:
/*
			if((timerTick%10)== 1) 	// MHH:07/06/2019. Try 5 times a second at Xoar request.
//			if(timerTick == 25)
			{	// every second
				xoar_send_packet = true;
			}
        	break;
*/
        case REMOTE_COMMS_XOAR2:
			if(--Xoar_ticks_left <= 0)
			{
				if(Xoar_ticks_left == 0)	// Not first time through.
				{
					xoar_send_packet = true;
				}
				WORD data_interval_tenths = getParameter (RC_DATA_INTERVAL);
				if(data_interval_tenths < 1 || data_interval_tenths > 10) data_interval_tenths = 10;	// default
				Xoar_ticks_left = data_interval_tenths * 5;
			}
        	break;
        }
        if(xoar_send_packet)
        {
			XoarSetRemoteMode();
			XoarCheckManual();
			// If monitor not = 0 then it will be connected to AC200User or similar
			if(XoarSendFlag && monitor == 0) {
				XoarSendPacket();
			}
        }

        //#endif
//		TraceMsg("main:5");
        watchIt2(WD_MAIN+130);

        if (monitor)
        {

            if (monitor == 2)
            {
                if (timerTick % 4 == 0)
                {
                    dumpState (1);
                }
                if (timerTick == 25)
                {
                    dumpState (0);
                }
            }
            else if (monitor == 1)
            {
                if ((timerTick == 0) || (timerTick == 25))
                {
                    dumpState (0);
                }
            }
        }

        watchIt2(WD_MAIN+140);
#ifdef MH_OLD_SLAVE_LOGIC
        // If a slave, identify yourself if in the hold position
        if (isSlaveProp() && (operatingMode() == HOLD))
        {
            if ((timerTick == 0) ||
                (timerTick == 16)||
                (timerTick == 33))
            {
                sendSlaveIdent();
            }
        }
        // If there is a slave attached, send the data
        if (deviceAttached (SLAVE_UNIT))
        {
            if ((timerTick == 0) ||
                (timerTick == 16)||
                (timerTick == 33))
            {
                sendSlaveData();
            }
        }
#endif
//		TraceMsg("main:6");
        watchIt2(WD_MAIN+150);


        // led toggle 1 Hz
        if (!timerTick)	// Probably just happens once a second (when timerTick == 0)
        {
#ifdef MH_PETER_SLAVE_LOGIC
            // device ticks // this will need to be added to
            // as more devices can be attached
            if (!isSlaveProp())
            {
                // do a master tick
                deviceMasterTick();
            }
            else
            {
                deviceSlaveTick();
            }
#endif
            lccount++;
            if (lccount == 5)	// Think this is the blinking LED on the PCB.
            {
                lccount = 0;
                diagnosticDisplay (lc);
                if (!lc)
                {
                    lc++;
                }
                else
                {
                    lc <<= 1;
                }
                if (lc == 0x10)
                {
                    lc = 0;
                }
            }
        }
//		TraceMsg("main:7");
        watchIt2(WD_MAIN+160);
    }
}
