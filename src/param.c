/* ------------------------------------------------------------
Title:          Param.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   23/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        System Parameter storage & Retrieval
                Contains the interface to the EEPROM
                ( this includes a software I2C port )
Changes:

------------------------------------------------------------ */
#include <string.h>
#include <stdio.h>

#include "global.h"
#include "param.h"
#include "digital.h"
#include "manifold.h"
#include "diags.h"
#include "log.h"

#include "ac210_log.h"
#include "ac210_sig100.h"

// Parameter Storage Layout Version control
#define PARAM_VERSION 16

#ifdef MH_EEPROM_FAST_WRITE
WORD EEPROM_writes;
#endif

// character sequences used to identify parameters in the comms..
const char pName[MAX_P_INDEX][5] = {
    "SPTO",  // 0.Take-off
    "SPCL",  // 1.Climb
    "SPCR",  // 2.Cruise
    "SPNS",  // 3.No Speed Indicator
//    "SINP",  // 4.speed input selector
//    "SFIG",  // 5.scaling factor mag input
//    "FCIG",  // 7.filter coefficient ign
//	"FCMG",  // 8.filter coeficient mag (unused)

	"PID2",  // 4. MHH:28/03/2023 Deadband2
//    "XXXX",  // 5. Unused
//    "RCBD",  // 5. MHH:30/09/2023. Remote Control Board.
    "RCMO",  // 5. MHH:22/02/2025. Remote Control Miscellaneous Options.
    "SFMG",  // 6.scaling factor magnetic input
    "PILO",  // 7. MHH:28/03/2023 Low rpm for deadband2
	"PIHI",  // 8.MHH:28/03/2023 high RPM
    "EMAX",  // 9.max control speed
    "EMIN",  // 10.min control speed
    "HMAX",  // 11.hold max
    "HMIN",  // 12.hold min

//    "PIGN",  // 13.PI gain
//	"PIDG",  // 14.Differential gain
	"SCEN",	 // MHH:03/04/2024. 13 Slider Control Enable
	"SCXV",	//                  14 Slider Control maX Value
    "PIDB",  // 15.dead band
//    "PIIG",  // 16.integral gain
	"CTSD",		// 16.Slave dead band. MHH:10/11/2020
//    "PIST",  // 17.PI sample tick count
	"CTMS",	// 17. MasterSlave. Master = 1, Slave = 2. MHH:10/11/2020
    "CTPT",  // 18. Master Slave port. (0 = no port, 1 = standard port, 2 = Auxiliary). MHH:23/11/2020

	"BLCT",  // 19.Brushless control
//	"DRSP",  // 19.Drive Start Pulse msec
    "MMAX",  // 20.drive max
//    "TCMX",  // 20. Throttle Control Max
    "RCPT",  // 21.Remote Control Port. MHH:21/11/2023
//    "MMIN",  // 21.drive min
    "MCLM",  // 22.motor current limit
    "FRTL",  // 23.feather reverse time limit
    "ACCW",  // 24.control word (bit mapped)

	"TCMN",  // 25.Throttle Control minimum
//	"TBEQ",  // 25.Table Equation ID
//    "TBDV",  // 26.Table Divisor	// 27
	"BLDI",	 // 26. Brushless Diagnostics. 04/07/2023
    "BLEN",  // 27.Brushless Enable	22/11/2022
	"BLHE",  // 28. Brushless Motor Heater Enable
	"BLHT",  // 29. Brushless Motor Heater start Temperature
    "BLHV",  // 30. Brushless Motor Hub software version,

    "BLHI",  // 31. Brushless Motor Hub ID,
	"BLUC",	 // 32. Brushless update count
	"BLMD",  // 33. Brushless modified date
    "BLHS",  // 34. Brushless Motor Hub Stops
    "BLFS",  // 35. Brushless Motor Fine Stop Angle (Measured in degrees)
    "BLCS",  // 36. Brushless Motor Coarse Stop Angle,
    "BLES",  // 37. Brushless Motor Feather Stop Angle,
    "BLRS",  // 38. Brushless Motor Reverse Stop Angle,

	"BLMR",  // 39. Brushless Motor Ratio,
    "BLPP",  // 40. Brushless Motor Pole Pairs,

	"SLT0",	 // 41. Single lever SETRPM
	"SLT1",	 // 42.
	"SLT2",  // 43.
	"SLT3",  // 44.
	"SLT4",  // 45.
	"SLT5",  // 46.
	"SLT6",  // 47.
	"SLT7",  // 48.
#ifdef MH_XXX
	"BLTE",  // 41. Brushless total errors

	"BLEC",  // 42. Brushless error count
	"BLEL",  // 43. Brushless last error
	"BLER",  // 44. Brushless last error run

	"BLAC",  // 45. Brushless abort count
	"BLAE",  // 46. Brushless last abort error
	"BLAR",  // 47. Brushless last abort run

//    "MMR2",  // MP_MAP_RPM_2,
//    "MMR3",  // MP_MAP_RPM_3,
//    "MMR4",  // MP_MAP_RPM_4,
//    "MMR5",  // MP_MAP_RPM_5,
//    "MMP1",  // MP_MAP_MFP_1,
//    "MMP2",  // MP_MAP_MFP_2,
//    "MMP3",  // MP_MAP_MFP_3,
//    "MMP4",  // MP_MAP_MFP_4,
//    "MMP5",  // MP_MAP_MFP_5,
    "MMDB",  // MP_MAP_DEADBAND, // 48
#endif

    "BRPM",  // BETA_MAX_RPM,	(49)
    "BCKR",  // BETA_CHK_RPM	(50)
    "BCKS",  // BETA_CHK_SWITCH (51)
    "BRMV",	 // BETA_RPM_MOVING (52)
    "SPHO",   // Set point Hold Speed (53)

    "CRPT",	// RPM_per_10_tick for new Control routines (54)
    "PWMR",	// PWM rate,0 = 62.5 kHz and is new default, 1=31.25 kHz, 2=20.833 kHz and is old default (55)
    "RCTY",	// Remote Comms Type. 0 = No Remote Comms, 1 =  Legacy Remote Comms, 2 = Xoar, 3 = Standard (56)
	"DENA",	// Diagnostics Enable (57)
	"FLOG",	// Flash Log enable for AC210 only (58)
	"APSL",	// Auxiliary Serial Port control (59)
	"APCN",	// Auxiliary CAN Port control (60)

	"TCPT",	// Throttle Control Port (61)
	"TCTO",	// Take-off percent (62)
	"TCCL",	// Climb percent (63)
	"TCTS",	// Take-off max secs (64)
	"RCDI", // Remote Control Data Interval(65)
	"RCFE",	// Remote Control Fields (66)

	"SLT8",	// Single lever SETRPM (67)
	"SLT9", // (68)
	"RTCE",	// RTC Enable (69)
	"LED4", // LED4 Enable (70)
	"DIEN",	// DEICER_ENABLE (71)
	"DION",	// DECICER_ON_SECS (72)
	"DIOF",	// DECICER_OFF_SECS (73)
	"SCXA", // Slider Control maX Angle(74)
	"SCNV", // Slider Control miN Value (75)
	"SCNA", // Slider Control miN Angle (76)
	"TCMX", // Throttle Control Max (77)
	"RCTM", // Remote Control Terminate Minimum (78)
	"RCBA", // Remote Control Baud Rate
	"RCPN", // Remote Control Prop Number
	"BLSP",	// Brushless bl_scale_pct

};


// storage layout for 512 Byte EEPROM
// this struct occupies the first 256 byte page
// its maximum size is 240 bytes.. equivalent to 117 parameters
#ifdef MH_YYY
typedef struct
{
    ULONG checkCode;
    WORD  pVersion;
    WORD  parms[MAX_P_INDEX];
} ParamStore;
#endif
// this table occupies the second 256 byte page. This is a direct table driven control
// system, with the control error value produced via a lookup into this table
#ifdef MH_TABLE_LOGIC
typedef struct
{
    BYTE  controlValue[256];
} ControlTable;
#endif

// Default values for the parameters. Used when checks fail
// Set Points
#define DEF_SP_TAKEOFF          5700    // RPM
#define DEF_SP_CLIMB            5400    // RPM
#define DEF_SP_CRUISE           5000    // RPM
#define DEF_SP_NO_RPM           100     // RPM    // "No RPM" indicator
#define DEF_SP_HOLD             DEF_SP_CRUISE

// Speed Inputs
#define DEF_SINP                MAG_INPUT
#define DEF_SF_IGN_SPEED        100     // %
#define DEF_SF_MAG_SPEED        243     // %
#define DEF_FC_IGN_SPEED        70      // %
#define DEF_FC_MAG_SPEED        80      // %

// Limits
#define DEF_MAX_ENGINE_SPEED    8000    // RPM
#define DEF_MIN_ENGINE_SPEED    2000    // RPM
#define DEF_MAX_HOLD_SPEED      DEF_SP_TAKEOFF
#define DEF_MIN_HOLD_SPEED      ((DEF_SP_CRUISE*8)/10)

// PI Controller
#define DEF_PROP_FACTOR         500     // prop gain
#define DEF_PI_DEAD_BAND        60      // RPM
//#define DEF_INT_FACTOR          50      // int gain
#define DEF_CT_SLAVE_DEADBAND	30		// MHH:10/11/2020
#define DEF_CT_MASTER_SLAVE		0
#define DEF_CT_PORT				0

#define DEF_DIFF_FACTOR         210     // diffferential gain
//#define DEF_PI_STICKS             0     // MHH:01/02/2018. Not used any more.
//#define DEF_PI_STICKS             5     // sample ticks for P and D terms
#define DEF_TB_LEAVING_GAIN     400     // Table lookup error increasing

#define DEF_MOTOR_START_PULSE     0     // Start pulse in milliseconds
// Motor PWM limits
#define DEF_MC_MAX_PWM          255     // 8 bit PWM
#define DEF_MC_MIN_PWM          180     // Loading determined.. min to move it

// Over current sense
#define DEF_MC_CURRENT_LIMIT    2500    // mA

// Feather Reverse time limit
#define DEF_FR_TIME_LIMIT       30      // seconds

// New ones COntrol word (bit mapped)
#define DEF_AC_CONTROL_WORD     0       // Standard, feather enabled

// BETA mode reversing
#define DEF_BETA_MAX_RPM        1000    // 1000 rpm max before entering BETAmode
#define DEF_BETA_CHK_RPM        0       // Do not check rpm by default
#define DEF_BETA_CHK_SWITCH     0       // Do not check switch input by default
#define DEF_BETA_MAX_RPM_ENGAGED 1000   // while running

// Theses are here for storage only, table values are generated by a PC somewhere
#define DEF_TB_EQUATION         1       //  (y=x*x*x)
#define DEF_TB_DIVISOR          1       // no division

// Manifold Pressure
#define DEF_MP_VALUE            0       // All set to Zero

//#define DEF_CTL_RPM_PER_10_TICK	100			// Use new control by default
//#define DEF_CTL_RPM_PER_10_TICK	1050			// MHH:28/05/2024
#define DEF_CTL_RPM_PER_10_TICK	100			// MHH:12/02/2025

#define DEF_PWM_RATE			1
#define DEF_REMOTE_COMMS_TYPE	0
#define DEF_REMOTE_COMMS_DATA_INTERVAL	0
#define DEF_REMOTE_COMMS_TERMINATE_MIN	0

#define DEF_DIAGS_ENABLE		1
//#define DEF_FLASH_LOG_RATE		0
#define DEF_FLASH_LOG_RATE		1		// MHH:23/12/2025

#define DEF_AUX_PORTS_SERIAL_CTL	0
#define DEF_AUX_PORTS_CAN_CTL	1

#define DEF_TC_PORT				0				// 61 Throttle control port
#define DEF_TC_TAKEOFF_PCT		95
#define DEF_TC_CLIMB_PCT		90
#define DEF_TC_TAKEOFF_MAX_SECS	300				// = 5 minutes, as specified in Rotax manual

#define DEF_TC_MAX_100_PCT		100				// MHH:30/01/2024. These allow mapping of throttle values
#define DEF_TC_MIN_0_PCT		0

#define DEF_PO_ENABLE			0
#define DEF_PO_GEAR_RATIO		231				// A Maxon ratio

#define DEF_RTC_ENABLE			0
#define DEF_LED4_ENABLE			0
#define DEF_DEICER_ENABLE		0
#define DEF_DEICER_ON_SECS		30
#define DEF_DEICER_OFF_SECS		30


#define PAGE_SIZE   16

#define CONFIG_PAGE 0
//#define TABLE_PAGE 1		// MHH:28/11/2016
#define STAT_PAGE		1

// current sense table defs
#define HW_FACTOR_PAGE 0
#define HW_FACTOR_OFFSET 240


// I2C port definitions

#ifndef AC210_PORT
#define I2CPORT     P6
#define I2CPORTDIR  PD6
#define I2CCLOCK    0x02
#define I2CDATA     0x01
#endif
// Variable storage
ParamStore ps;
//ControlTable ct;		// MHH:28/11/2016
HardwareFactors hwf;


// Local prototypes needed
void setDefaults (void);

// function prototypes for I2C bit bashing code
// real low level
void i2cStart(void);
void i2cStop(void);
void i2cReadStart (void);
// up to bytes
BYTE i2cReadByte (BYTE ack);
BYTE i2cWriteByte (BYTE b);
// up to messages

//char dbuf[40];

#ifdef AC210_PORT
static bool param_flash_change;
static bool rtc_enable_change;
#endif

//------------------------------------------------------------------------------------------------
WORD Control_type;		// MHH: 27/09/2017
BYTE AutoGyro_reverse;
BYTE AutoGyro_state;

/*
#define CT_NON_FEATHERING	0
#define CT_FEATHERING		1
#define CT_BETA				2
#define CT_REVERSE			3

Have a legacy system to deal with.

Non-feathering is identified by bit 1 in AC_CONTROL_WORD.
Beta-reverse is identified by bit 8.

The only way in existing system to identify non_beta_reversing is by compile constant REVERSING_VERSION.

Bit 1 = 0, which indicates feathering in AC_CONTROL_WORD.

So, have added an extra bit_flag to ac_control_word (0x10) to identify NON_BETA_REVERSE.

*/
void Param_set_control_type(void)
{
	WORD cw = ps.parms[AC_CONTROL_WORD];

	if(cw & CW_BETA_REVERSING)
	{
		Control_type = CT_BETA;
		return;
	}

	if(cw & CW_NON_BETA_REVERSING)
	{
		Control_type = CT_REVERSE;
		if(cw & CW_AUTOGYRO_REVERSE)	// Note: Not setting Control_type to special value at this stage
		{
			AutoGyro_reverse = true;
		}
		// This means NON_BETA_REVERSING will always be classed as feathering to be consistent with legacy system.
		ps.parms[AC_CONTROL_WORD] &= ~CW_NON_FEATHERING;	// Make sure non-feathering bit is off to be consistent with legacy system
		return;
	}
	if(cw & CW_NON_FEATHERING)
	{
		Control_type = CT_NON_FEATHERING;
		return;
	}
// If we drop through then it is set to default of feathering.

	Control_type = CT_FEATHERING;	// default

}

//------------------------------------------------------------------------------------------------
// Load all from EEPROM
void loadParameters (void)
{
    BYTE count;

#ifndef AC210_PORT
    // set up port.. Only need clock as output..
    PD6 |= I2CCLOCK;
#endif


#ifdef AC210_PORT
    if(sizeof(hwf) != 16)	// This check because default int size for LPC1768 is 4 bytes, but 2 bytes on MC80
    {
    	DebugAbort("loadParameters:sizeof(hwf) != 16");
    	return;
    }
#endif


    count = 0;

    while (count < 4)
    {
        // First, make sure hardware is in correct state
#ifndef AC210_PORT
        i2cStop ();
#endif
        // clear parameter store
        memset (&ps, 0, sizeof(ps));
        memset (&hwf, 0, sizeof(hwf));

        // read in the parameter storage from the  EEPROM
        if (readI2C (EEPROMREAD, CONFIG_PAGE, 0, (BYTE *)&ps, sizeof(ps)))
        {
            // check the check code
            if ((ps.checkCode == PARAM_CHECK_CODE) && (ps.pVersion == PARAM_VERSION))
            {
				if(ps.parms[FLASH_LOG_RATE] > 50) ps.parms[FLASH_LOG_RATE] = DEF_FLASH_LOG_RATE;
				if(ps.parms[RC_DATA_INTERVAL] > 100) ps.parms[RC_DATA_INTERVAL] = DEF_REMOTE_COMMS_DATA_INTERVAL;
				if(ps.parms[RC_TERMINATE_MIN] > 10000) ps.parms[RC_TERMINATE_MIN] = 0;	// default

				if(ps.parms[AUX_PORTS_SERIAL_CTL] == 0xFFFF)	// Software upgrade and parameters not set?
				{												// Then set to defaults. Could write to EEPROM, but will happen on parameter change
					ps.parms[AUX_PORTS_SERIAL_CTL] = DEF_AUX_PORTS_SERIAL_CTL;
					ps.parms[AUX_PORTS_CAN_CTL] = DEF_AUX_PORTS_CAN_CTL;
					ps.parms[TC_PORT] = DEF_TC_PORT;
					ps.parms[TC_TAKEOFF_PCT] = DEF_TC_TAKEOFF_PCT;
					ps.parms[TC_CLIMB_PCT] = DEF_TC_CLIMB_PCT;
					ps.parms[TC_TAKEOFF_MAX_SECS] = DEF_TC_TAKEOFF_MAX_SECS;
				}
#ifdef MH_XXX	// MHH:11/02/2025
				if(ps.parms[CTL_RPM_PER_TICK_X10] < 1000)	// MHH:20/05/2024
				{
					ps.parms[CTL_RPM_PER_TICK_X10] = DEF_CTL_RPM_PER_10_TICK;		// default
				}
#endif
				if(ps.parms[RTC_ENABLE] == 0xFFFF)	// MHH: 01/01/2019
				{
					ps.parms[RTC_ENABLE] = DEF_RTC_ENABLE;
				}
				if(ps.parms[LED4_ENABLE] == 0xFFFF)	// MHH: 01/01/2019
				{
					ps.parms[LED4_ENABLE] = DEF_LED4_ENABLE;
				}
				if(ps.parms[CT_MASTER_SLAVE] > 2)	// MHH:23/11/2020
				{
					ps.parms[CT_MASTER_SLAVE] = DEF_CT_MASTER_SLAVE;
				}
				if(ps.parms[CT_PORT] > 2)
				{
					ps.parms[CT_PORT] = DEF_CT_PORT;
				}
				if(ps.parms[DEICER_ENABLE] == 0xFFFF)	// MHH: 09/02/2021
				{
					ps.parms[DEICER_ENABLE] = DEF_DEICER_ENABLE;
					ps.parms[DEICER_ON_SECS] = DEF_DEICER_ON_SECS;
					ps.parms[DEICER_OFF_SECS] = DEF_DEICER_OFF_SECS;
				}
#ifdef MH_XXX
				if(ps.parms[BL_MOTOR_RATIO] == 1900)	// MHH:24/03/2023. Fix ini bug..
			    {
			    	ps.parms[BL_MOTOR_RATIO] = 190;
			    }
#endif
			    if(ps.parms[PI_DEAD_BAND2] == 655535) ps.parms[PI_DEAD_BAND2] = 0;	// MHH:29/07/2023
				if(ps.parms[PI_LO_RPM] <= 100 || ps.parms[PI_LO_RPM] == 65535) ps.parms[PI_LO_RPM] = 0;
			    if(ps.parms[PI_HI_RPM] <= 100 || ps.parms[PI_HI_RPM] == 65535) ps.parms[PI_HI_RPM] = 0;

			    if(ps.parms[SL_T0] == 65535)
			    {
			    	ps.parms[SL_T0] = 0;
			    	ps.parms[SL_T1] = 0;
			    	ps.parms[SL_T2] = 0;
			    	ps.parms[SL_T3] = 0;
			    	ps.parms[SL_T4] = 0;
			    	ps.parms[SL_T5] = 0;
			    	ps.parms[SL_T6] = 0;
			    	ps.parms[SL_T7] = 0;
			    	ps.parms[SL_T8] = 0;
			    	ps.parms[SL_T9] = 0;
			    }

			    if(ps.parms[RC_MISC_OPTIONS] == 65535) ps.parms[RC_MISC_OPTIONS] = 0;	// MHH:22/02/2025
			    if(ps.parms[RC_PORT] >2) ps.parms[RC_PORT] = 0;	// MHH:21/02/2025
			    if(ps.parms[RC_BAUDRATE] == 65535) ps.parms[RC_BAUDRATE] = 0;	// MHH:21/02/2025
			    if(ps.parms[RC_PROPNUM] == 65535) ps.parms[RC_PROPNUM] = 1;	// MHH:10/03/2025

			    if(ps.parms[TC_MAX_100_PCT] < 50 || ps.parms[TC_MAX_100_PCT] > 100) ps.parms[TC_MAX_100_PCT] = 100;	// default.
			    if(ps.parms[TC_MIN_0_PCT] > 50) ps.parms[TC_MIN_0_PCT] = 0;	// default.

			    ps.parms[MC_MAX_PWM] = DEF_MC_MAX_PWM;		// MHH:08/05/2024. Because was using location for throttle control, now moved.

			    if(ps.parms[BL_SCALE_PCT] < 10 || ps.parms[BL_SCALE_PCT > 500]) ps.parms[BL_SCALE_PCT] = 100;

// read in current sense values
                readI2C (EEPROMREAD, HW_FACTOR_PAGE, HW_FACTOR_OFFSET, (BYTE *)&hwf, sizeof(hwf));

			    if(hwf.RC_S_mode == 65535) hwf.RC_S_mode = 1;		// MHH:03/11/2023

                if ((hwf.currentGain == 0xFFFF) || (hwf.currentGain == 0))
                {
                    // default current sense offset and gain. gain is to 2 dp
                    hwf.currentOffset = 0;
                    hwf.currentGain =  1250; // 12.50
                	writeI2C (EEPROMWRITE, HW_FACTOR_PAGE, HW_FACTOR_OFFSET, (BYTE *)&hwf, sizeof (hwf));
                }

                Param_init_read_diags();

#ifdef MH_XXX		// MHH:11/12/2023
//                AC210_plog_init();		// MHH:08/08/2023
                AC210_log_init();
                AC210_RTC_init();
#endif
                Param_set_control_type();
                invalidConfig = 0;
                return;
            }
        }
        count++;
    }
    // Load in the defaults, but do not write to flash
if(AT_Trace)
{
	txDebug("Setting Defaults\r\n");
}

    setDefaults();
    invalidConfig = 1;

}
//----------------------------------------------------------------------------------------------------------
// MHH:24/12/2016 Moved out of loadParameters() so that we can access earlier for logging routines
void Param_init_read_diags(void)
{
     readI2C (EEPROMREAD, STAT_PAGE, 0, (BYTE *)&Stat_Rec, sizeof(Stat_Rec));	// MHH:28/11/2016
// Quick and dirty initialise

     if(Stat_Rec.check_code != 12345)
     {
     	Diags_zero_rec();
     }
     Diags_init_stats();
	DPRINTF("Init:log_last_diag_pos:%d, log_start_data_pos:%d\r\n\r\n",Stat_Rec.log_last_diags_pos,Stat_Rec.log_start_data_pos);

}
//-----------------------------------------------------------------------------------------------------------
// Accessor routines
WORD getParameter ( ParamIndex idx)
{
/*
	if(idx == AUX_PORTS_SERIAL_CTL)	// MHH:18/11/2022 DEBUG!!!!!
	{
		return 0;
	}
*/

    return (ps.parms[idx]);
}
int get_signedParameter(ParamIndex idx)
{
	int16_t v = (int16_t)ps.parms[idx];
	return v;
}
//-----------------------------------------------------------------------------------------------------------
#define BL_FEATHER_ENABLED	4
#define BL_REVERSE_ENABLED	8

BYTE isFeatheringProp (void)
{
	if(ps.parms[BL_ENABLED] == 1)
	{
		WORD stops = ps.parms[BL_HUB_STOPS];
		if((stops & BL_FEATHER_ENABLED) == 0) return false;
#ifdef MH_XXX	// MHH:20/05/2025. Allow feathering under AC300 control if position error
		if(Hub_pos_error) return false;		// MHH:25/07/2023. Disable if Hub position error
#endif
	}
	BYTE result =  ((ps.parms[AC_CONTROL_WORD] & CW_NON_FEATHERING) == 0);
	return result;
}

BYTE isFeatheringOrReversingProp (void)
{
	if(ps.parms[BL_ENABLED] == 1)
	{
		WORD stops = ps.parms[BL_HUB_STOPS];
		if((stops & (BL_FEATHER_ENABLED | BL_REVERSE_ENABLED)) == 0) return false;
#ifdef MH_XXX	// MHH:20/05/2025. Allow feathering under AC300 control if position error. Not sure how this will affect reversing.
		if(Hub_pos_error) return false;		// MHH:25/07/2023. Disable if Hub position error
#endif
	}

	WORD cw = ps.parms[AC_CONTROL_WORD];
//	BYTE result = (((cw & CW_NON_FEATHERING) == 0) || ((cw & CW_NON_BETA_REVERSING) != 0));	// MHH:13/02/2019
	BYTE result = (((cw & CW_NON_FEATHERING) == 0) || ((cw & CW_NON_BETA_REVERSING) != 0) || ((cw & CW_BETA_REVERSING) != 0));	// MHH:14/07/2025
	return result;
//	return ((ps.parms[AC_CONTROL_WORD] & CW_NON_FEATHERING) == 0);
}



/*
BYTE manifoldPressureEnabled (void)
{
    return ((ps.parms[AC_CONTROL_WORD] & MANIFOLD_PRESSURE_ENABLE) != 0);
}
*/

BYTE isFeatherAndReverseProp (void)	// MHH:12/02/2019
{
	if(ps.parms[BL_ENABLED] == 1)
	{
		WORD stops = ps.parms[BL_HUB_STOPS];
//		if((stops & (BL_FEATHER_ENABLED | BL_REVERSE_ENABLED)) != (BL_FEATHER_ENABLED | BL_REVERSE_ENABLED)) return false;
		if((stops & (BL_FEATHER_ENABLED | BL_REVERSE_ENABLED)) == 0) return false;	// MHH:29/07/2023
//		if(Hub_pos_error) return false;		// MHH:25/07/2023. Disable if Hub position error
		return true;
	}
    return false;
}

BYTE isReverseOnlyProp (void)	// MHH:12/02/2019
{
	if(ps.parms[BL_ENABLED] == 1)
	{
		WORD stops = ps.parms[BL_HUB_STOPS];
		if((stops & (BL_FEATHER_ENABLED | BL_REVERSE_ENABLED)) != BL_REVERSE_ENABLED) return false;
		return true;
//		if(Hub_pos_error) return false;		// MHH:25/07/2023. Disable if Hub position error
	}

	WORD cw = ps.parms[AC_CONTROL_WORD];
    BYTE result = (cw & CW_NON_BETA_REVERSING);
    return result;
}

BYTE isSlaveProp (void)
{
//    return ((ps.parms[AC_CONTROL_WORD] & CW_SLAVE_SYNCH) != 0);
    return (ps.parms[CT_MASTER_SLAVE] == CT_SLAVE);
}

BYTE isMasterProp (void)
{
//    return ((ps.parms[AC_CONTROL_WORD] & CW_SLAVE_SYNCH) != 0);
    return (ps.parms[CT_MASTER_SLAVE] == CT_MASTER);
}

BYTE isBetaProp (void)
{
	if(ps.parms[BL_ENABLED] == 1)
	{
		WORD stops = ps.parms[BL_HUB_STOPS];
		if((stops & BL_REVERSE_ENABLED) == 0) return false;
		if(Hub_pos_error) return false;		// MHH:25/07/2023. Disable if Hub position error
	}

	BYTE result =  ((ps.parms[AC_CONTROL_WORD] & CW_BETA_REVERSING) != 0);
	return result;
}

BYTE isReversingProp (void)
{
	if(ps.parms[BL_ENABLED] == 1)
	{
		WORD stops = ps.parms[BL_HUB_STOPS];
		if((stops & BL_REVERSE_ENABLED) == 0) return false;
		if(Hub_pos_error) return false;		// MHH:25/07/2023. Disable if Hub position error
	}
	BYTE result = ((ps.parms[AC_CONTROL_WORD] & CW_NON_BETA_REVERSING) != 0);
	return result;
}


// This is called internally only to change parameters temporarily (eg for tune needs to allow remote)
void setTempParameter ( ParamIndex idx, WORD nval)
{
	ps.parms[idx] = nval;
}

// returns non zero if nval is out of range
BYTE setParameter ( ParamIndex idx, WORD nval)
{
    BYTE rval=0;
    int16_t sval=(int16_t)nval;	// In case we need to look at signed value
    // check the ranges here

    switch (idx)
    {
#ifdef AC210_PORT
    default:
    	break;
#endif

    case SP_TAKEOFF:
    case SP_CLIMB:
    case SP_CRUISE:
    case SP_HOLD:
    case PI_LO_RPM:
    case PI_HI_RPM:
        // must be between MIN & MAX engine speed
        rval = ((nval >= ps.parms[MIN_ENGINE_SPEED]) &&
                (nval <= ps.parms[MAX_ENGINE_SPEED]));
        break;

    case SP_NO_RPM:
        // No real limit.. less than MIN_ENGINE_SPEED
        rval = (nval < ps.parms[MIN_ENGINE_SPEED]);
        break;

    case SF_MAG_SPEED:
        // % figure. Scaling factor for speed input.
        // 100 = 1.00, 33 = 0.33.. Max is 10.00
        rval = (nval <= 1000);
        break;

#ifdef MH_XXX
    case FC_MAG_SPEED:
        // % figure. Must be < 100% for new values to get through
        // filter
        rval = (nval < 100);
        break;
#endif
        // Limits
    case MAX_ENGINE_SPEED:
        // MHH:24/10/2019. Change from 10,000 RPM to 20,000
        rval = ((nval <= 20000) && (nval > ps.parms[MIN_ENGINE_SPEED]));
        break;

    case MIN_ENGINE_SPEED:
        // 0 RPM ??
        rval = ((nval > 0) && (nval < ps.parms[MAX_ENGINE_SPEED]));
        break;

    case MAX_HOLD_SPEED:
        // max is takeoff RPM
        rval = ((nval >= ps.parms[MIN_HOLD_SPEED]) && (nval <= ps.parms[SP_TAKEOFF]));
        break;

    case MIN_HOLD_SPEED:
        // min is 80% of cruise
//        rval = ((nval <= ps.parms[MAX_HOLD_SPEED]) && (nval >= ((ps.parms[SP_CRUISE]*8)/10)));
// MHH: 04/03/2020: Remove 80% of cruise test
    	rval = (nval <= ps.parms[MAX_HOLD_SPEED]);
        break;

#ifdef MH_XXX
        // PI Controller
    case PROP_FACTOR:
        // Can be up to 50 (5000)
        rval = (nval <= 5000);
        break;

    case DIFF_FACTOR:
        // Can be up to 50 (5000)
        rval = (nval <= 5000);
        break;
#endif
    case PI_DEAD_BAND:
        // RPM dead band (+/- figure, ie 15 is +-15..)
        rval = (nval <= 150);
        break;

    case PI_DEAD_BAND2:
        // RPM dead band (+/- figure, ie 15 is +-15..)
        rval = (nval <= 200);
        break;

    case CT_SLAVE_DEADBAND:	// MHH:10/11/2020
        // Slave RPM dead band (+/- figure, ie 15 is +-15..)
        rval = (nval <= 150);
        break;

    case CT_MASTER_SLAVE:	// MHH:10/11/2020
    	rval = (nval >= 0 && nval <= 2);
    	break;

    case CT_PORT:		// MHH:23/11/2020
    	rval = (nval >= 0 && nval <= 2);
    	break;
#ifdef MH_XXX
    case START_ONE_SHOT_MS:
        rval = (nval > 0 && nval < 500);
        break;
#endif
    case BL_CONTROL:
//        rval = (nval > 0 && nval < 500);
        rval = true;	// MHH:23/11/2024
        break;
#ifdef MH_XXX	// MHH:30/01/2024
        // Motor PWM limits
    case MC_MAX_PWM:
        // Motor max for 8 bit = 254 (driver chip needs an off period)
//        rval = ((nval <= 255) && (nval >= ps.parms[MC_MIN_PWM]));
        rval = (nval <= 255);
        break;
#endif
#ifdef MH_XXX
    case MC_MIN_PWM:
        // practical min for motion is about 180 with a load on motor
        // can never be 1, because drive controller needs a signal to
        // charge boost capacitors
        rval = ((nval >= 1) && (nval <= ps.parms[MC_MAX_PWM]));
        break;
#endif

        // Over current sense
    case MC_CURRENT_LIMIT:
        // in mA, 6000 = 6 Amps
        rval = (nval <= 6000);
        break;

        // feather reverse time limit
    case FR_TIME_LIMIT:
        rval = (nval <= 90);
        break;

    case AC_CONTROL_WORD:
        rval = 1;
// Note: Could check if correct but better to do in AC200User
        Param_set_control_type();
        break;

#ifdef MH_XXX
    case TB_EQUATION:
        rval = (nval < 5);
        break;

    case TB_DIVISOR:
        rval = 1;
        break;
#endif
    case BL_DIAGNOSTICS:
//    	rval = (nval == 0 || nval == 123 || nval == 456);	// Special code...
    	rval = (nval < 32);		// MHH:29/10/2023. 1=123,2=456,4=ACX210 Board is BL type,8,16 are spare.
    	break;

    case BL_ENABLED:
    	rval = (nval >= 0 && nval <= 1);
    	break;

#ifdef MH_XXX    	// MHH:27/01/2024. These fields should not be updated by AC200User as they are updated by AC200Calibrator
    case BL_MOTOR_RATIO:
    	rval = (nval >= 500 && nval <= 10000);	// Allow between 50.0 and 1000.0
    	break;

    case BL_MOTOR_POLE_PAIRS:
    	rval = (nval >= 1 && nval <= 4);
    	break;

    case BL_HUB_STOPS:
    	rval = (nval >= 0 && nval <= 14 && (nval & 1)== 0);
    	break;

    case BL_FINE_STOP:
    	// Looking for a value in degrees with an implied decimal point

    	rval = (nval >= 100 && nval <= 250);	// Allow between 10.0 and 25.0 degrees
    	break;

    case BL_COARSE_STOP:
    	// Looking for a value in degrees with an implied decimal point

//    	rval = (nval >= 250 && nval <= 450);	// Allow between 25.0 and 45.0 degrees
    	rval = (nval >= 0 && nval <= 450);	// Allow between 0 and 45.0 degrees. MHH:07/02/2023
    	break;

    case BL_FEATHER_STOP:
    	// Looking for a value in degrees with an implied decimal point

    	rval = (nval >= 750 && nval <= 900);	// Allow between 75.0 and 90.0 degrees
    	break;

    case BL_REVERSE_STOP:
    	// Looking for a value in degrees with an implied decimal point
//    	rval = (nval >= -300 && nval <= 0);	// Allow between -30.0 and 0.0 degrees
    	rval = (sval >= -300 && sval <= 0);	// Allow between -30.0 and 0.0 degrees
    	break;

    case BL_HUB_ID:	// No edits
    	rval = 1;
    	break;
#endif
    case BL_HEATER_ENABLED:
    	rval = (nval >= 0 && nval <= 1);
    	break;

    case BL_HEATER_START_TEMPERATURE:
    	rval = (sval >= -20 && sval <= 10);	// Allow between -20 and +10 degrees
    	break;

    case BL_SCALE_PCT:	// MHH:16/04/2026
    	rval = (nval >= 10 && nval <= 500);
    	break;


/*
        //manifold pressure Stuff
    case MP_MAX_RPM_1:
    case MP_MAX_RPM_2:
    case MP_MAX_RPM_3:
    case MP_MAX_RPM_4:
    case MP_MAX_RPM_5:
    case MP_MAP_RPM_1:

    case MP_MAP_RPM_2:
    case MP_MAP_RPM_3:
    case MP_MAP_RPM_4:
    case MP_MAP_RPM_5:
        rval = (nval <= ps.parms[MAX_ENGINE_SPEED]);
        break;
*/
/*
    case MP_MAX_MFP_1:
    case MP_MAX_MFP_2:
    case MP_MAX_MFP_3:
    case MP_MAX_MFP_4:
    case MP_MAX_MFP_5:
    case MP_MAP_MFP_1:
*/
//    case MP_MAP_MFP_2:
//    case MP_MAP_MFP_3:
//    case MP_MAP_MFP_4:
//    case MP_MAP_MFP_5:
//        rval = (nval <= 500);         // 50.0 inches mercury
//        break;

    case SL_T0:			// Single lever SETRPM values
    case SL_T1:
    case SL_T2:
    case SL_T3:
    case SL_T4:
    case SL_T5:
    case SL_T6:
    case SL_T7:
    case SL_T8:
    case SL_T9:
    	rval = (nval == 0 || ((nval >= ps.parms[MIN_ENGINE_SPEED]) && (nval <= ps.parms[MAX_ENGINE_SPEED])));
    	break;

//    case MP_MAX_DEADBAND:
#ifdef MH_XXX
    case MP_MAP_DEADBAND:
        rval = (nval <= 50);          // 5.0 inches mercury
        break;
#endif
    case BETA_MAX_RPM:
#ifdef MH_OLD_BETA_CHECK
    	rval = ((nval >= ps.parms[MIN_ENGINE_SPEED]) &&
                (nval <= ps.parms[MAX_ENGINE_SPEED]));
#else
        rval = ((nval >= 500) &&	// MHH:23/11/2016 Was rejecting Beta Max RPM as MIN_ENGINE_SPEED was 2000....
                (nval <= ps.parms[MAX_ENGINE_SPEED]));
#endif
        break;

    case BETA_CHK_RPM:
    case BETA_CHK_SWITCH:
        rval = ((nval == 0) || (nval == 1));
        break;

    case BETA_MAX_RPM_ENGAGED:
#ifdef MH_OLD_BETA_CHECK
    	rval = ((nval >= ps.parms[MIN_ENGINE_SPEED]) &&
                (nval <= ps.parms[MAX_ENGINE_SPEED]));
#else
        rval = ((nval >= 500) &&	// MHH:23/11/2016 Was rejecting Beta Max RPM Engaged as MIN_ENGINE_SPEED was 2000....
                (nval <= ps.parms[MAX_ENGINE_SPEED]));
#endif
        break;

    case CTL_RPM_PER_TICK_X10:
// Assume 1 decimal place
//    	rval = (nval == 0 || (nval >= CTL_RPM_MIN && nval <= CTL_RPM_MAX));
    	rval = (nval < 1000);		// NCV < 1000
//    	rval = (nval <= 65000);		// MHH:13/04/2026
    	break;

    case PWM_RATE:
    	rval = (nval <= 2);
//		sprintf(Ctl_pbuff,"Setting PWM rate to :%d\r\n",nval);
//		txDebug(Ctl_pbuff);

//		txDebug("Setting PWMRate\r\n");
    	break;

    case REMOTE_COMMS_TYPE:
    	rval = (nval <= 5);
//		txDebug("Setting RemoteCommsType\r\n");
    	break;

    case DIAGS_ENABLE:
    	rval = (nval <= 1);
#ifdef MH_XXX
    	if(nval == 0)
    		printf("Diagnostics Disabled\r\n");
    	else
    		printf("Diagnostics Enabled\r\n");
#endif
    	break;

    case FLASH_LOG_RATE:
    	rval = (nval <= 50);
    	if(LogData.enabled)	// MHH:04/02/2024. Only if LogData enabled.
    	{
        	if(rval)
        	{
        		if(ps.parms[FLASH_LOG_RATE] != nval)
        		{
        			param_flash_change = true;		// Set flag as if we do now will take time and may miss other parameters
        		}
        	}
    	}
    	break;

    case AUX_PORTS_SERIAL_CTL:
#ifdef AC210_PORT
    	rval = (nval <= 3);
#else
    	rval = (nval <= 1);
#endif
    	break;

    case AUX_PORTS_CAN_CTL:
    	rval = (nval <= 1);
    	break;

    case TC_PORT:				// Throttle Control. Will only work with AC210
    	rval = (nval <= 2);
    	break;

    case TC_TAKEOFF_PCT:
    	rval = (nval >= 80 && nval <= 100);
    	break;

    case TC_CLIMB_PCT:
    	rval = (nval >= 60 && nval <= 95);
    	break;

    case TC_TAKEOFF_MAX_SECS:
    	rval = (nval >= 100 && nval <= 500);	// Rotax = 300 (5 minutes)
    	break;

    case TC_MAX_100_PCT:
    	rval = (nval > 50 && nval <= 115);
    	break;

    case TC_MIN_0_PCT:
    	rval = (nval >= 0 && nval <= 50);
    	break;

    case RC_DATA_INTERVAL:
    	rval = (nval <= 100);		// Max of 10 secsond (arbitrary)
    	break;

    case RC_DATA_FIELDS:
    	rval = (nval <= 15);			// 4 fields defined so far
    	break;

    case RC_TERMINATE_MIN:
    	rval = (nval < 10000);		// Not sure what max should be. Zero should mean always send a termination message.
    	break;

    case RC_MISC_OPTIONS:
    	rval = (nval <= 7);			// 3 bit flags
    	break;

    case RC_PORT:
    	rval = (nval <= 2);			// Only 0 or 1 or 2.
    	break;

    case RC_BAUDRATE:
    	rval = false;
    	if(nval == 0) rval = true;		// Default = 19,200 Serial, maybe CANAero default if CAN?
    	if(nval == 192) rval = true;	// Serial baud rate / 100
    	if(nval == 1152) rval = true;	// Could add other valid rates here
    	if(nval == 125) rval = true;		// CAN baud rate divided by 1000. This is CANAero rate.
    	if(nval == 250) rval = true;		// CAN baud rate divided by 1000
    	if(nval == 500) rval = true;		// CAN baud rate divided by 1000
    	if(nval == 1000) rval = true;		// Add other rates as needed. Could put in a readonly table...
    	break;

    case RC_PROPNUM:
    	if(nval > 0 && nval <= 4) rval = true;
    	break;

    case RTC_ENABLE:
    	rval = (nval <= 1);
    	if(rval)
    	{
    		if(ps.parms[RTC_ENABLE] != nval)
    		{
    			rtc_enable_change = true;		// Set flag as if we do now will take time and may miss other parameters
    		}
    	}
    	break;

    case LED4_ENABLE:
    	rval = (nval <= 1);
    	break;

    case DEICER_ENABLE:		// MH:09/02/2021
    	rval = (nval <= 2);
    	break;

    case DEICER_ON_SECS:		// MH:09/02/2021
    	rval = (nval <= 120);
    	break;

    case DEICER_OFF_SECS:		// MH:09/02/2021
    	rval = (nval <= 120);
    	break;

    case SLIDER_CONTROL_ENABLE:	// MHH:03/04/2024
    	rval = (nval <= 1);
    	break;

    case SLIDER_CONTROL_MAX_VALUE:
    	rval = (nval < 1024);
    	break;
    case SLIDER_CONTROL_MAX_ANGLE:
    	rval = (nval <= 40);		// Could check that hi > lo
    	break;
    case SLIDER_CONTROL_MIN_VALUE:
    	rval = (nval < 1024);
    	break;
    case SLIDER_CONTROL_MIN_ANGLE:
    	rval = (nval <= 360);		// Could check that hi > lo
//    	rval = (sval <= -40);		// Could check that hi > lo
    	break;


    }
    if(rval == 0) return 1;		// MHH: 19/11/2016

    // passed the bounds checking
    ps.parms[idx] = nval;

// If we drop through, then a valid parameter which needs writing to eeprom..
#ifdef MH_EEPROM_FAST_WRITE
	EEPROM_writes++;
#else
#define MH_NEW_EEWRITE
// Note: Was getting what I think are timing issues when writing to eeprom with longer array.
// The last value (PWMR) was being ignored. Writing just what is necessary fixes that problem
#ifdef MH_NEW_EEWRITE
//        ps.parms[idx] = nval;
		if(idx == SP_TAKEOFF)	// One that we know will always happen
		{
	        writeI2C (EEPROMWRITE, CONFIG_PAGE, 0, (BYTE *)&ps, sizeof (ps));	// This write defaults
		}
		else
		{
        // passed the bounds checking
        // write it to EEPROM
        	writeI2C (EEPROMWRITE, CONFIG_PAGE, 6+idx*2, (BYTE *)&ps.parms[idx], 2);
		}
#else
        // passed the bounds checking
//        ps.parms[idx] = nval;
        // write it to EEPROM
        writeI2C (EEPROMWRITE, CONFIG_PAGE, 0, (BYTE *)&ps, sizeof (ps));
#endif
#endif
        return 0;
}

BYTE writeParameter ( ParamIndex idx, WORD nval)	// MHH:15/03/2021
{
	if(idx > MAX_P_INDEX)
	{
		return 1;
	}
	ps.parms[idx] = nval;
	writeI2C (EEPROMWRITE, CONFIG_PAGE, 6+idx*2, (BYTE *)&ps.parms[idx], 2);
	return 0;
}



#ifdef MH_TABLE_LOGIC
void setTableValue (BYTE idx, BYTE val)
{
    ct.controlValue[idx] = val;
    // Update EE copy with single BYTE write
    writeI2C ( EEPROMWRITE, TABLE_PAGE, idx,  (BYTE *)&ct.controlValue[idx], 1);
}
#endif

// current sense gain and offset
int  getCurrentOffset (void)
{
    return (hwf.currentOffset);
}

WORD getCurrentGain (void)
{
    return (hwf.currentGain);

}
//--------------------------------------------------------------------------------------
void RC_S_write_param(void)
{
//    txDebug("RC_S_write_param\r\n");
	writeI2C (EEPROMWRITE, HW_FACTOR_PAGE, HW_FACTOR_OFFSET+4, (BYTE *)&hwf.RC_S_mode,4);
}
//--------------------------------------------------------------------------------------
#ifdef AC2_TEST		// AC2_TEST
void AC2_TEST_setCurrentOffset (int cof)
{
    hwf.currentOffset += cof;
    writeI2C (EEPROMWRITE, HW_FACTOR_PAGE, HW_FACTOR_OFFSET, (BYTE *)&hwf, sizeof (hwf));
}

void AC2_TEST_setCurrentGain (int cg)
{
    if (cg > 0)
    {
        if (hwf.currentGain < 2000)
        {
            hwf.currentGain += 2;
            writeI2C (EEPROMWRITE, HW_FACTOR_PAGE, HW_FACTOR_OFFSET, (BYTE *)&hwf, sizeof (hwf));
        }
    }
    else
    {
        if (hwf.currentGain > 900)
        {
            hwf.currentGain -= 2;
            writeI2C (EEPROMWRITE, HW_FACTOR_PAGE, HW_FACTOR_OFFSET, (BYTE *)&hwf, sizeof (hwf));
        }
    }
}
#endif
//--------------------------------------------------------------------------------------
void setCurrentOffset (int cof)
{
    hwf.currentOffset = cof;
    writeI2C (EEPROMWRITE, HW_FACTOR_PAGE, HW_FACTOR_OFFSET, (BYTE *)&hwf, sizeof (hwf));
}

void setCurrentGain (WORD cg)
{
    hwf.currentGain = cg;
    writeI2C (EEPROMWRITE, HW_FACTOR_PAGE, HW_FACTOR_OFFSET, (BYTE *)&hwf, sizeof (hwf));
}

// Set default parameters in store, and write back to EEPROM
void setDefaults (void)
{
    ps.checkCode = PARAM_CHECK_CODE;
    ps.pVersion = PARAM_VERSION;
    // set points
    ps.parms[SP_TAKEOFF] = DEF_SP_TAKEOFF;
    ps.parms[SP_CLIMB] = DEF_SP_CLIMB;
    ps.parms[SP_CRUISE] = DEF_SP_CRUISE;
    ps.parms[SP_NO_RPM] = DEF_SP_NO_RPM;
    ps.parms[SP_HOLD] = DEF_SP_HOLD;

    // Speed Inputs
//    ps.parms[SINP] = DEF_SINP;
//    ps.parms[SF_IGN_SPEED] = DEF_SF_IGN_SPEED;
    ps.parms[SF_MAG_SPEED] = DEF_SF_MAG_SPEED;
//    ps.parms[FC_IGN_SPEED] = DEF_FC_IGN_SPEED;
//    ps.parms[FC_MAG_SPEED] = DEF_FC_MAG_SPEED;

    // Limits
    ps.parms[MAX_ENGINE_SPEED] = DEF_MAX_ENGINE_SPEED;
    ps.parms[MIN_ENGINE_SPEED] = DEF_MIN_ENGINE_SPEED;
    ps.parms[MAX_HOLD_SPEED] = DEF_MAX_HOLD_SPEED;
    ps.parms[MIN_HOLD_SPEED] = DEF_MIN_HOLD_SPEED;

    // PI Controller
#ifdef MH_XXX
    ps.parms[PROP_FACTOR] = DEF_PROP_FACTOR;
    ps.parms[DIFF_FACTOR] = DEF_DIFF_FACTOR;
#endif
    ps.parms[PI_DEAD_BAND] = DEF_PI_DEAD_BAND;
    ps.parms[PI_DEAD_BAND2] = 0;	// MHH:29/07/2023
    /*
    ps.parms[INT_FACTOR] = DEF_INT_FACTOR;
    ps.parms[PI_SAMPLE_TICKS] = DEF_PI_STICKS;
    */
    ps.parms[CT_SLAVE_DEADBAND] = DEF_CT_SLAVE_DEADBAND;	// MHH:10/11/2020
    ps.parms[CT_MASTER_SLAVE] = DEF_CT_MASTER_SLAVE;
    ps.parms[CT_PORT] = DEF_CT_PORT;

//    ps.parms[TB_LEAVING_GAIN] = DEF_TB_LEAVING_GAIN;

//    ps.parms[BL_CONTROL] = 160;		// MHH:10/04/2023
    ps.parms[BL_CONTROL] = 0;		// MHH:18/02/2026.

    // Motor PWM limits
    ps.parms[MC_MAX_PWM] = DEF_MC_MAX_PWM;

    // Over current sense
    ps.parms[MC_CURRENT_LIMIT] = DEF_MC_CURRENT_LIMIT;

    // feather reverse time limit
    ps.parms[FR_TIME_LIMIT] = DEF_FR_TIME_LIMIT;

    ps.parms[AC_CONTROL_WORD] = DEF_AC_CONTROL_WORD;
//    ps.parms[TB_EQUATION] = DEF_TB_EQUATION;
//    ps.parms[TB_DIVISOR] = DEF_TB_DIVISOR;

/*
    ps.parms[MP_MAX_RPM_1] = DEF_MP_VALUE;
    ps.parms[MP_MAX_RPM_2] = DEF_MP_VALUE;
    ps.parms[MP_MAX_RPM_3] = DEF_MP_VALUE;
    ps.parms[MP_MAX_RPM_4] = DEF_MP_VALUE;
    ps.parms[MP_MAX_RPM_5] = DEF_MP_VALUE;

  */
    ps.parms[BL_ENABLED] = 0;
    ps.parms[BL_MOTOR_RATIO] = 190;	// Common ratio of 190.0
    ps.parms[BL_MOTOR_POLE_PAIRS] = 1;
    ps.parms[BL_HUB_STOPS] = 0;
    ps.parms[BL_FINE_STOP] = 180;	// Default of 18.0 degrees

    ps.parms[BL_COARSE_STOP] = 350;	// Default of 35.0 degrees
    ps.parms[BL_FEATHER_STOP] = 810;	// Default of 81.0 degrees
    ps.parms[BL_REVERSE_STOP] = -200;	// Default of -20.0 degrees
    ps.parms[BL_HUB_ID] = 0;
    ps.parms[BL_HUB_SOFTWARE_VERSION] = 0;
    ps.parms[BL_HEATER_ENABLED] = 0;
    ps.parms[BL_HEATER_START_TEMPERATURE] = 0;
    ps.parms[BL_DIAGNOSTICS] = 0;
    ps.parms[BL_SCALE_PCT] = 100;

    ps.parms[SL_T0] = 0;
    ps.parms[SL_T1] = 0;
    ps.parms[SL_T2] = 0;
    ps.parms[SL_T3] = 0;
    ps.parms[SL_T4] = 0;
    ps.parms[SL_T5] = 0;
    ps.parms[SL_T6] = 0;
    ps.parms[SL_T7] = 0;
    ps.parms[SL_T8] = 0;
    ps.parms[SL_T9] = 0;


 /*

    //    ps.parms[MP_MAP_RPM_1] = DEF_MP_VALUE;
    ps.parms[MP_MAP_RPM_2] = DEF_MP_VALUE;
    ps.parms[MP_MAP_RPM_3] = DEF_MP_VALUE;
    ps.parms[MP_MAP_RPM_4] = DEF_MP_VALUE;
    ps.parms[MP_MAP_RPM_5] = DEF_MP_VALUE;


    ps.parms[MP_MAX_MFP_1] = DEF_MP_VALUE;
    ps.parms[MP_MAX_MFP_2] = DEF_MP_VALUE;
    ps.parms[MP_MAX_MFP_3] = DEF_MP_VALUE;
    ps.parms[MP_MAX_MFP_4] = DEF_MP_VALUE;
    ps.parms[MP_MAX_MFP_5] = DEF_MP_VALUE;
    ps.parms[MP_MAP_MFP_1] = DEF_MP_VALUE;
*/
    //ps.parms[MP_MAP_MFP_2] = DEF_MP_VALUE;
//    ps.parms[MP_MAP_MFP_3] = DEF_MP_VALUE;
//    ps.parms[MP_MAP_MFP_4] = DEF_MP_VALUE;
//    ps.parms[MP_MAP_MFP_5] = DEF_MP_VALUE;

//    ps.parms[MP_MAX_DEADBAND] = DEF_MP_VALUE;
//    ps.parms[MP_MAP_DEADBAND] = DEF_MP_VALUE;

    ps.parms[BETA_MAX_RPM] = DEF_BETA_MAX_RPM;
    ps.parms[BETA_CHK_RPM] = DEF_BETA_CHK_RPM;
    ps.parms[BETA_CHK_SWITCH] = DEF_BETA_CHK_SWITCH;
    ps.parms[BETA_MAX_RPM_ENGAGED] = DEF_BETA_MAX_RPM_ENGAGED;

    ps.parms[CTL_RPM_PER_TICK_X10] = DEF_CTL_RPM_PER_10_TICK;
    ps.parms[PWM_RATE] = DEF_PWM_RATE;
	ps.parms[REMOTE_COMMS_TYPE] = DEF_REMOTE_COMMS_TYPE;
	ps.parms[DIAGS_ENABLE] = DEF_DIAGS_ENABLE;
	ps.parms[FLASH_LOG_RATE] = DEF_FLASH_LOG_RATE;


	ps.parms[AUX_PORTS_SERIAL_CTL] = DEF_AUX_PORTS_SERIAL_CTL;
	ps.parms[AUX_PORTS_CAN_CTL] = DEF_AUX_PORTS_CAN_CTL;
	ps.parms[TC_PORT] = DEF_TC_PORT;
	ps.parms[TC_TAKEOFF_PCT] = DEF_TC_TAKEOFF_PCT;
	ps.parms[TC_CLIMB_PCT] = DEF_TC_CLIMB_PCT;
	ps.parms[TC_TAKEOFF_MAX_SECS] = DEF_TC_TAKEOFF_MAX_SECS;
	ps.parms[TC_MAX_100_PCT] = DEF_TC_MAX_100_PCT;
	ps.parms[TC_MIN_0_PCT] = DEF_TC_MIN_0_PCT;

	ps.parms[RC_DATA_INTERVAL] = 0;
	ps.parms[RC_DATA_FIELDS] = 0;
	ps.parms[RC_TERMINATE_MIN] = 0;
	ps.parms[RC_MISC_OPTIONS] = 0;
	ps.parms[RC_PORT] = 0;
	ps.parms[RC_BAUDRATE] = 0;
	ps.parms[RC_PROPNUM] = 1;

	ps.parms[RTC_ENABLE] = 0;
	ps.parms[LED4_ENABLE] = 0;

	// and write this back to the EEPROM
//    writeI2C (EEPROMWRITE, CONFIG_PAGE, 0, (BYTE *)&ps, sizeof (ps));
    writeI2C (EEPROMWRITE, CONFIG_PAGE, 0, (BYTE *)&ps, sizeof (ps));	// MHH:22/12/2025

#ifdef MH_TABLE_LOGIC
    // Now the control value lookup table
    for (cnt =0; cnt < 256; cnt++)
    {
        ct.controlValue[cnt] = cnt;
    }
#endif

    // and write it to I2C
    //writeI2C (EEPROMWRITE, TABLE_PAGE, 0, (BYTE *)&ct, sizeof(ct));

    hwf.currentOffset = 0;
    hwf.currentGain =  1250; // 12.50

    hwf.RC_S_mode = 1;		// MHH:03/11/2023

}
//--------------------------------------------------------------------------------------------------------------------
BYTE Param_updating_stats;
void WriteStatsRec(char far*desc,WORD wlen)
{
#ifdef AC210_PORT
//		printf("Updating stats page from %s\r\n",desc);
#else
#ifdef MH_DEBUG_WRITE_STATS
		txDebug("Updating stats page from: ");
		txDebug(desc);
		txDebug("\r\n");
#endif
#endif
		if(Param_updating_stats) return;

#ifdef MH_DPRINT
		DPRINTF("WriteStatsRec:from:%s,wlen = %d\r\n",desc,wlen);
		DPRINTF("log_last_diag_pos:%d, log_start_data_pos:%d\r\n\r\n",Stat_Rec.log_last_diags_pos,Stat_Rec.log_start_data_pos);
#endif
		Param_updating_stats=TRUE;	// Set flag in case of watchdog timeout or abort
		writeI2C (EEPROMWRITE, STAT_PAGE, 0, (BYTE *)&Stat_Rec, wlen);	// MHH:10/12/2016
		Param_updating_stats=FALSE;
}
//--------------------------------------------------------------------------------------------------------------------
#ifdef MH_EEPROM_FAST_WRITE
// The idea is to write parameter table only once, as AC200User changes back to monitor mode
void ParameterFastUpdate(void)
{
  	if(EEPROM_writes > 0)
   	{
        writeI2C (EEPROMWRITE, CONFIG_PAGE, 0, (BYTE *)&ps, sizeof (ps));
        EEPROM_writes = 0;
#ifdef AC210_PORT
//	printf("EEPROM updated\r\n");
		if(param_flash_change)
		{
			param_flash_change = false;
			AC210_log_init();
			WORD nval = ps.parms[FLASH_LOG_RATE];
//			AC210_eelog_change_rate(nval);
			Log_change_rate(nval);
		}
		if(rtc_enable_change)
		{
			rtc_enable_change = false;
			AC210_RTC_init();
		}

#endif
  	}
}
#endif

#ifdef AC210_PORT
// Note: On existing system, page size is 256 bytes, so need to multiply page number by 256 to get byte offset

BYTE readI2C ( BYTE slave_addr,
               BYTE page,
               BYTE addr,
               BYTE *buffer,
               WORD len)
{
	uint32_t pos;
	pos = page *256 + addr;
	p_eeprom_i2c_pos(pos);
	if(p_eeprom_i2c_read(buffer,len) == 0) return 1;	// success
	return 0;		// fail
}
BYTE writeI2C ( BYTE slave_addr,
                BYTE page,
                BYTE addr,
                BYTE *buffer,
                WORD len)
{
	uint32_t pos;
	pos = page *256 + addr;
	p_eeprom_i2c_pos(pos);
	if(p_eeprom_i2c_write(buffer,len) == 0) return 1;	// success
	return 0;		// fail
}
#endif
