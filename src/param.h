/* ------------------------------------------------------------
Title:          param.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   22/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Local header for parameter storage & retrieval

Changes:

------------------------------------------------------------ */
#ifndef _PARAM_H
#define _PARAM_H

// Speed Input Sub definitions
#define MAG_INPUT 0
#define IGN_INPUT 1

#define CTL_RPM_MIN		20		// For new ctl logic
//define CTL_RPM_MAX		350
#define CTL_RPM_MAX		4100		// MHH:29/05/2024

// control word bit definitions
#define CW_NON_FEATHERING           (WORD)0x0001
//#define MANIFOLD_PRESSURE_ENABLE (WORD)0x0002
//#define CW_REVERSE_RELAY 			(WORD)0x0002	// MHH:12/02/2019
#define CW_SLAVE_SYNCH              (WORD)0x0004
#define CW_BETA_REVERSING           (WORD)0x0008
#define CW_NON_BETA_REVERSING		(WORD)0x0010	// MHH:27/09/2017
#define CW_AUTOGYRO_REVERSE		 	(WORD)0x0020	// MHH:17/03/2018

#define CT_MASTER		1
#define CT_SLAVE		2

// I2C interface EEPROM definitions
#define EEPROMWRITE 0xA0        // Ored with Page Number, bit position 1
#define EEPROMREAD  0xA1        // Ored with Page Number   "     "     "

BYTE writeI2C ( BYTE slave_addr,
                BYTE page,
                BYTE addr,
                BYTE *buffer,
                WORD len);
BYTE readI2C ( BYTE slave_addr,
               BYTE page,
               BYTE addr,
               BYTE *buffer,
               WORD len);


BYTE I2C_read_page(BYTE *buffer,BYTE page,WORD len);

void AC2_TEST_setCurrentOffset (int cof);
void AC2_TEST_setCurrentGain (int cg);

// this struct sits in the last 16 bytes of the first page
typedef struct
{
    short int  currentOffset;	// 0. MHH:10/04/2017
    WORD currentGain;			// 2.
    WORD RC_S_mode;				// 4.
    WORD RC_S_setspeed;			// 6.
    WORD spare[4];
} HardwareFactors;

extern HardwareFactors hwf;

void RC_S_write_param(void);
void POS_write_param(void);


// Parameter definitions
// for the sake of ease of use, ALL system parameters are WORDS
// the enum below is used as an index into the array of WORDS stored in
// the EEPROM.

typedef enum
{
    // set points
    SP_TAKEOFF = 0,			// 0
    SP_CLIMB,				// 1
    SP_CRUISE,				// 2
    SP_NO_RPM,				// 3

    // Speed Inputs
//    SINP,                   // 4
//    SF_IGN_SPEED,			// 5
//    FC_IGN_SPEED,			// 7
//    FC_MAG_SPEED,			// 8 (this isn't used)

	PI_DEAD_BAND2,			// 4 MHH:28/03/2023
//	RC_BOARD,				// 5 MHH:30/09/2023
	RC_MISC_OPTIONS,		// 5 MHH:22/02/2025
    SF_MAG_SPEED,			// 6 Scaling factor
	PI_LO_RPM,				// 7. These must be after scaling factor to work with AC200User
	PI_HI_RPM,				// 8

    // Limits
    MAX_ENGINE_SPEED,       // 9
    MIN_ENGINE_SPEED,		// 10

    MAX_HOLD_SPEED,			// 11
    MIN_HOLD_SPEED,			// 12

    // PI Controller
    SLIDER_CONTROL_ENABLE,	// 13 MHH:03/04/2024
	SLIDER_CONTROL_MAX_VALUE, // 14
//	PROP_FACTOR,			// 13
//    DIFF_FACTOR,           	// 14
    PI_DEAD_BAND,			// 15
//    INT_FACTOR,
	CT_SLAVE_DEADBAND,		// 16
//    PI_SAMPLE_TICKS,
	CT_MASTER_SLAVE,		// 17
//    TB_LEAVING_GAIN,
	CT_PORT,				// 18

    BL_CONTROL,      		// 19


	// Motor PWM limits
/*
 *  MHH:08/05/2024. Was a mistake to overwrite MAX_PWM as the parameters are not backwardly compatible. In other words, if you try running an older version
 *  of AC200 software then it gets wrong value for MAX_PWM. To fix, use a new location for TC_MAX_100_PCT.
 */

	MC_MAX_PWM,				// 20
//	TC_MAX_100_PCT,				// 20 MHH:30/01/2024
    RC_PORT,				// 21 MHH:21/11/2023
    // Over current sense
    MC_CURRENT_LIMIT,		// 22

    // Feather reverse time
    FR_TIME_LIMIT,			// 23

    // new features
    AC_CONTROL_WORD,        // 24
//    TB_EQUATION,
    TC_MIN_0_PCT,			// 25 MHH:30/01/2024
//    TB_DIVISOR,
	BL_DIAGNOSTICS,			// MHH:04/07/2023

    // Was Manifold Pressure Table Points
    BL_ENABLED,						// 27.  1 = Enabled, all other values disabled
    BL_HEATER_ENABLED,				// 28.
    BL_HEATER_START_TEMPERATURE,	// 29.
    BL_HUB_SOFTWARE_VERSION,		// 30. Note: User cannot change

	BL_HUB_ID,
	BL_UPDATE_COUNT,
    BL_MODIFIED_DATE,
	BL_HUB_STOPS,		// Bit flags for hardware micro-switch stops implemented. FINE stop is mandatory.
    BL_FINE_STOP,		// This one must be measured, all other stops a relative in Hub.
    BL_COARSE_STOP,
    BL_FEATHER_STOP,
    BL_REVERSE_STOP,

	BL_MOTOR_RATIO,		// Store internally with one implied decimal position
    BL_MOTOR_POLE_PAIRS,     // Needed to convert Hub position to angle

	SL_T0,				// Single lever throttle setrpm
	SL_T1,
	SL_T2,
	SL_T3,
	SL_T4,
	SL_T5,
	SL_T6,
	SL_T7,
#ifdef MH_XXX
	BL_TOTAL_ERRORS,           // These BL fields not used - can be reallocated.
    BL_ERR_COUNT,
	BL_ERR_LAST,
    BL_ERR_RUN,
    BL_ABORT_COUNT,
	BL_ABORT_LAST,
	BL_ABORT_RUN,
//    MP_MAP_MFP_2,           // 44
//    MP_MAP_MFP_3,			// 45
//    MP_MAP_MFP_4,
//    MP_MAP_MFP_5,
    MP_MAP_DEADBAND,        // 48
#endif
    BETA_MAX_RPM,           // 49
    BETA_CHK_RPM,			// 50
    BETA_CHK_SWITCH,		// 51
    BETA_MAX_RPM_ENGAGED,	// 52
    SP_HOLD,				// 53

    CTL_RPM_PER_TICK_X10,	// 54
    PWM_RATE,				// 55
    REMOTE_COMMS_TYPE,		// 56
	DIAGS_ENABLE,			// 57
	FLASH_LOG_RATE,			// 58

	AUX_PORTS_SERIAL_CTL,		//59
							// 1 = AC210 Aux serial port at 115.2K, UL or AM rout format
	AUX_PORTS_CAN_CTL,		//60
							// 1 = AC210 Rotax CAN format

	TC_PORT,				// 61 Throttle control port. Zero = Disabled, 1 = Aux Serial Port,2 = Aux CAN port
	TC_TAKEOFF_PCT,			// 62
	TC_CLIMB_PCT,			// 63
	TC_TAKEOFF_MAX_SECS,	// 64

	RC_DATA_INTERVAL,		// 65
	RC_DATA_FIELDS,			// 66

	SL_T8,					// 67 Single lever setrpm
	SL_T9,					// 68

//	FILL_2,					// 67 Position enable  	MHH:Redundant 29/03/2023
//	FILL_3,					// 68 Gear ratio		MHH:Redundant 29/03/2023
	RTC_ENABLE,				// 69 RTC Enable
	LED4_ENABLE,			// 70 LED4 Enable
	DEICER_ENABLE,			// 71 MHH:09/02/2021
	DEICER_ON_SECS,			// 72
	DEICER_OFF_SECS,		// 73
	SLIDER_CONTROL_MAX_ANGLE, // 74
	SLIDER_CONTROL_MIN_VALUE,	// 75
	SLIDER_CONTROL_MIN_ANGLE,	// 76
	TC_MAX_100_PCT,				// 77 MHH:08/05/2024. Moved from position 20.

	RC_TERMINATE_MIN,		// 78. MHH:08/02/2025.
	RC_BAUDRATE,			// 79. MHH:21/02/2025 Applies to Auxiliary Serial and CAN
	RC_PROPNUM,				// 80. MHH:10/03/2025. Applies only to CAN
	BL_SCALE_PCT,			// 81. MHH:16/04/2026
    // Array Index
    MAX_P_INDEX				// 81
} ParamIndex;

#define PARAM_CHECK_CODE		135798642L
typedef struct
{
    ULONG checkCode;
    WORD  pVersion;
    WORD  parms[MAX_P_INDEX];
} ParamStore;
// strings used to identify parameters in the comms..

extern ParamStore ps;
extern const char pName[MAX_P_INDEX][5];

// Load the set of parameters from EEPROM
void loadParameters (void);


// Accessor routines

void Diags_zero_rec(void );
void SendDiags(void);
void UpdateStatsDate(ULONG yymmdd);
//void WriteStatsRec(char *desc);

int get_signedParameter(ParamIndex idx);
WORD getParameter ( ParamIndex idx);
BYTE setParameter ( ParamIndex idx, WORD nval);
void setTempParameter(ParamIndex idx,WORD val);	// MHH: 05/09/2018. Used by remote control to temporarily set parameters.&ps
BYTE writeParameter ( ParamIndex idx, WORD nval);	// MHH:15/03/2021. Write a single parameter

BYTE getTableValue (BYTE idx);
void setTableValue( BYTE idx, BYTE val);

// current sense gain and offset
int  getCurrentOffset (void);
WORD getCurrentGain (void);
void setCurrentOffset (int cof);
void setCurrentGain (WORD cg);

BYTE isFeatheringProp (void);
BYTE isFeatheringOrReversingProp (void);

//BYTE manifoldPressureEnabled (void);
BYTE isReverseRelay(void);
BYTE isReverseOnlyProp (void);
BYTE isFeatherAndReverseProp (void);
BYTE isSlaveProp (void);
BYTE isBetaProp (void);
BYTE isMasterProp (void);

BYTE GetPWM_CycleRate(void);
void SetPWM_CycleRate(BYTE value);
#ifdef MH_ATSPEED
BYTE GetPWM_MaxSpeed(void);
void SetPWM_MaxSpeed(BYTE value);
#endif

#endif

