/*
 * ac210_sig60.h
 *
 *  Created on: 17/09/2018
 *      Author: Murray
 */

#ifndef AC210_SIG100_H_
#define AC210_SIG100_H_

#include "board.h"

extern int AC200Enc_version;
//extern bool SIG60_connected;

extern uint8_t BL_enc_rpm;
extern uint8_t BL_dac0_v;

extern uint16_t ADC_BL_corrected_voltage;
extern uint8_t ADC_adjusted_voltage;
extern uint8_t SIG100_line_percent;
extern uint8_t ADC_adjusted_temp;
extern bool Sig100_hub_link;
extern bool Hublink_command_sent;
extern uint8_t Ctl_brushless_speed;

extern bool Hub_pos_error;
extern bool Hub_not_ready_error;
extern uint8_t BL_flags;

float SIG100_get_hub_voltage(void);	// MHH:25/06/2025
int AC300_get_run_number(void);		// MHH:05/07/2025


//----------------------------------------------------------------------------
// Minimum needed to calculate angle
struct HUB_CALC
{
	float cam_radius;
	float hall_incr_mm;
	float blade_offset;
	float tdc_pos_offset;
	float tdc_mm_offset;
	int16_t fine_stop_angle;		//  This is set by Calibrator based on micro-switch stop, others are user defined
	int16_t coarse_stop_angle;		//
	int16_t feather_stop_angle;		//
	int16_t reverse_stop_angle;		//
};
extern struct HUB_CALC Hub_calc;

typedef enum	// Note: This must be the same as Hub definition.
{
    CAL_IDLE=0,
	CAL_START,
	CAL_MAIN,
	CAL_FINISH,
	CAL_ERROR,
} CAL_state;
#ifdef MH_XXX
{
    CAL_IDLE=0,
	CAL_START,
//	CAL_FINE_STOP,
	CAL_COARSE_HARDSTOP,
//	CAL_COARSE_HARDSTOP_FINE,
//	CAL_COARSE_HARDSTOP_SLOW,
	CAL_FINE_HARDSTOP,
//	CAL_FINE_HARDSTOP_COARSE,
//	CAL_FINE_HARDSTOP_SLOW,
	CAL_RETURN_FINE_STOP,
	CAL_FINISH,
} CAL_state;
#endif

#ifdef MH_XXX
typedef enum
{
    BL_IDLE = 0,
    BL_MAJOR_FINE,
	BL_MAJOR_COARSE,
    BL_BRAKE_FINE,
	BL_BRAKE_COARSE,
	BL_MINOR_FINE,
	BL_MINOR_COARSE
} BL_CTL_STATE;
#endif

#ifdef MH_XXX

typedef enum
{
    CAL_IDLE=0,
	CAL_START,
	CAL_FINE_STOP,
	CAL_COARSE_HARDSTOP,
	CAL_COARSE_HARDSTOP_FINE,
	CAL_COARSE_HARDSTOP_SLOW,
	CAL_FINE_HARDSTOP,
	CAL_FINE_HARDSTOP_COARSE,
	CAL_FINE_HARDSTOP_SLOW,
	CAL_RETURN_FINE_STOP,
	CAL_FINISH,
} CAL_state;
#endif
extern CAL_state Hub_calibrate_state;
extern int Hub_calibrate_checkval;	// Safety
extern uint8_t Hub_calibrate_pole_pairs;
extern uint8_t Hub_calibrate_type;	// MHH:25/08/2023

int SIG100_getc_timeout(uint32_t millisecs);
bool SIG100_UARTReadable(void);
int SIG100_set_position_len(uint16_t len_in_mm_1dec);
int SIG100_get_position_len(uint16_t len_in_mm_1dec);
int SIG100_set_position_encoder(int ienc);

void Sig100_init(void);

int LMT87_convert_celcius_to_millivolts(int celcius);

#define SIG100_TIMEOUT_MILLISECS	5
#define SSP_MAGIC_1			11111
#define SSP_MAGIC_2			22222
#define SSP_MAGIC_3         33333


#define P_BL_DIAG_ON			1
#define P_BL_ACCEL_BOARD		2
#define P_BL_LOGIC_BRUSHLESS	4

#define P_BL_RC_ALLOW			128
// 8 and 16 spare
//-------------------------------------------------------------------------------------------------------------------
typedef struct
{

	uint8_t type;			// 0. eg 'E' for error
	uint8_t command;		// 1.
	uint16_t code;			// 2.
	uint16_t ac200_run;		// 4.
	uint16_t secs;			// 6.
	int16_t eval;			// 8. eval, used for extra info, eg in a position correction how large the correction
} Enc_err_td;				// 10.

typedef struct
{
	uint16_t magic;				// 0.
	uint16_t creation_time;		// 2. May use for creation time
	uint16_t hub_id;			// 4.
	uint16_t update_count;		// 6. Incremented by AC200HubCalibrator only
	uint16_t modified_date;		// 8.
	uint16_t creation_date;		// 10.

	uint16_t cal_tolerance;		// 12. In Ecount units, equivalent to +/- 0.05 mm, provided by Calibrator. Could be unit8_t
	uint16_t cal_motor_ratio;	// 14. Test hub uses 294.6
	uint16_t cal_cam_length;	// 16. In tenths of a mm, usually 26.0
	int16_t cal_blade_offset;	// 18. In degrees, usually 30.0
	uint16_t cal_leadscrew_mm;	// 20. In mm, 3 implied decimal places. Default is 3.175 (25.4/8)

	uint8_t cal_pole_pairs;		// 22.
	uint8_t user_defined_stops;	// 23. Bit flags of user defined stops (feather and reverse)
	uint16_t cal_coarse_stop;	// 24. Was user defined, but now is provided by calibrator.
	uint16_t ud_feather_stop;	// 26. User defined coarse stops
	uint16_t ud_reverse_stop;	// 28.

	uint16_t cal_fine_stop;		// 30.
//	uint16_t cal_coarse_hard_stop;	// 32.
	uint16_t cal_tdc_pos_offset;		// 32. MHH:04/07/2023. Same value, different name.
	uint16_t cal_fine_hard_stop;		// 34.
	uint16_t ref_max;				// 36.Could be 1 byte. These fields build picture of reference stops
	uint16_t ref_start[3];			// 38.
	uint16_t ref_end[3];			// 44.
	uint8_t voltage_min;			// 50. MHH:29/06/2025
	uint8_t cal_current_max;		// 51. MHH:03/09/2025
//	uint16_t ref_main;				// 50.
	uint16_t cal_fine_stop2;		// 52. There can be play depending on if cam is pushing or pulling. Since the fine stop is used to
									//	   check the accuracy of the position, it is valid inside the range fine_stop to fine_stop2, +/- tolerance
	uint8_t hw_flags;				// 54 hardware flags eg op_amp
	uint8_t hw_version;				// 55. 18/07/2023

	uint8_t sw_flags;				// 56. MHH:23/06/2025 eg 24V
	uint8_t voltage_correct_pct;	// 57. MHH:25/06/2025

	uint16_t cal_pos;				// 58. Returned from calibrator, could be moved out
	uint32_t crc32;					// 60.
									// 64.Should fit in 1 flash page
/*
*	Update following data to memory if an error, but limit rate it can be updated to (say) once every 10 seconds
*/
	uint16_t magic2;				// 64.0.When just loading from this area
	uint8_t run_flags;				// 66.2 Eg 1 = not allowed to FEATHER
	uint8_t update_count2;			// 67.3 So we can update AC200 with copy of data following magic3 on init if different to AC200
	uint16_t ac200_run;				// 68.4. Use for error reporting
	uint16_t hub_run;				// 70.6
//	uint16_t rpm_starts;			// 72.8
	uint16_t ac300_software_version;	// 72.8 MHH:16/12/2025
	uint16_t hub_software_version;	// 74.10
	uint16_t calibrate_pos;			// 76.12
	uint16_t fill1;					// 78.14

	uint16_t error_count;			// 80.16
	uint8_t error_type;				// 82.18	eg 'E'
	uint8_t error_command;			// 83.19
	uint16_t error_number;			// 84.20
	uint16_t error_ac200_run;		// 86.22
	uint16_t error_data;			// 88.24 Anything else that helps define an error
	uint8_t error_badpos1_count;	// 90.26 Increment if position is invalid on loading
	uint8_t error_badpos2_count;	// 91.27
	uint16_t error_fill[2];			// 92.28	Fill areas for future use

	uint16_t error_end_data;		// 96.32	Total of 32 bytes in data 2 area

	uint16_t pos1;					// 98.34 These 3 updated together, so we know if we have an rpm restart (if motor was going when hub mcu stopped)
	uint16_t pos1_check;			// 100.36 pos complement, to check pos correct
	uint16_t rpm;					// 102.38
	uint16_t pos2;					// 104.40
	uint16_t pos2_check;			// 106.42
	uint16_t rpm2;					// 108.44
	uint16_t fill2[2];				// 110.46
	uint16_t magic3;				// 114.50
	uint16_t end_data2;				// 116.52
	// Note: Copy until up to rpm from hub, as maintaining data below in struct is handled by ac200
	// We may not need any of the data below in the hub. Have decided that it is simpler and more robust to send any error data from the hub to ac200
	// rather than trying to store, as the storing may be causing the error.
	uint16_t total_errors;			// 118.
	uint16_t error_count2;			// 120
	uint16_t abort_count;			// 122
	uint16_t watchdog_count;		// 124
	uint16_t fill3;					// 126
	Enc_err_td error;				// 128.
	Enc_err_td abort;				// 138.
	Enc_err_td watchdog;			// 148.
	Enc_err_td errtbl[4];			// 158	// Note: This takes over 128 bytes, but room for 256 in page zero of diags
	uint16_t end;					// 198.

}Enc_data_td;
extern Enc_data_td Enc_data;
extern int Encoder_Pos;

#ifdef MH_XXX
typedef struct
{
	uint16_t magic;				// 0.
	uint16_t creation_time;		// 2.
//	uint32_t magic;				// 0.
	uint16_t hub_id;			// 4.
	uint16_t update_count;		// 6. Incremented by AC200HubCalibrator only
	uint16_t modified_date;		// 8.
	uint16_t creation_date;		// 10.

	uint16_t cal_tolerance;		// 12. In Ecount units, equivalent to +/- 0.05 mm, provided by Calibrator. Could be unit8_t
	uint16_t cal_motor_ratio;	// 14. Test hub uses 294.6
	uint16_t cal_cam_length;	// 16. In tenths of a mm, usually 26.0
	int16_t cal_blade_offset;	// 18. In degrees, usually 30.0
	uint16_t cal_leadscrew_mm;	// 20. In mm, 3 implied decimal places. Default is 3.175 (25.4/8)

	uint8_t cal_pole_pairs;		// 22. Could be uint8_t
	uint8_t user_defined_stops;	// 23. Bit flags of user defined stops (coarse,feather and reverse)

	uint16_t ud_coarse_stop;		// 24. User defined coarse stop
	uint16_t ud_feather_stop;	// 26.
	uint16_t ud_reverse_stop;	// 28.

	uint16_t cal_fine_stop;		// 30.
//	uint16_t cal_coarse_hard_stop;	// 32. MHH:04/07/2023. Now defined as zero.
	uint16_t cal_tdc_pos_offset;		// 32. MHH:04/07/2023. Same value, different name.
//	uint16_t cal_tdc_offset_mm;			// 32. Could use as tdc offset to coarse hard stop
	uint16_t cal_fine_hard_stop;		// 34.
	uint16_t ref_max;				// 36.Could be 1 byte. These fields build picture of reference stops
	uint16_t ref_start[3];			// 38.
	uint16_t ref_end[3];				// 44.
	uint16_t ref_main;				// 50.
	uint16_t fill[3];				// 52.

	uint16_t cal_pos;				// 58. Returned from calibrator, could be moved out
	uint32_t crc32;					// 60.

	uint16_t magic2;				// 64.0.When just loading from this area
	uint8_t run_flags;				// 66.2 Eg 1 = not allowed to FEATHER
	uint8_t update_count2;			// 67.3 So we can update AC200 with copy of data following magic3 on init if different to AC200
	uint16_t ac200_run;				// 68.4. Use for error reporting
	uint16_t hub_run;				// 70.6
	uint16_t rpm_starts;			// 72.8
	uint16_t hub_software_version;	// 74.10
	uint16_t calibrate_pos;			// 76.12
	uint16_t fill1;					// 78.14

	uint16_t error_count;			// 80.16
	uint8_t error_type;				// 82.18	eg 'E'
	uint8_t error_command;			// 83.19
	uint16_t error_number;			// 84.20
	uint16_t error_ac200_run;		// 86.22
	uint16_t error_data;			// 88.24 Anything else that helps define an error
	uint8_t error_badpos1_count;	// 90.26 Increment if position is invalid on loading
	uint8_t error_badpos2_count;	// 91.27
	uint16_t error_fill[2];			// 92.28	Fill areas for future use

	uint16_t error_end_data;		// 96.32	Total of 32 bytes in data 2 area

	uint16_t pos1;					// 98.34 These 3 updated together, so we know if we have an rpm restart (if motor was going when hub mcu stopped)
	uint16_t pos1_check;			// 100.36 pos complement, to check pos correct
	uint16_t rpm;					// 102.38
	uint16_t pos2;					// 104.40
	uint16_t pos2_check;			// 106.42
	uint16_t rpm2;					// 108.44
	uint16_t fill2[2];				// 110.46
	uint16_t magic3;				// 114.50
	uint16_t end_data2;				// 116.52

	// Note: Copy until up to rpm from hub, as maintaining data below in struct is handled by ac200
	// We may not need any of the data below in the hub. Have decided that it is simpler and more robust to send any error data from the hub to ac200
	// rather than trying to store, as the storing may be causing the error.

	uint16_t total_errors;			// 118.
	uint16_t error_count2;			// 120
	uint16_t abort_count;			// 122
	uint16_t watchdog_count;		// 124
	uint16_t fill3[2];
	Enc_err_td error;				// 128.
	Enc_err_td abort;				// 138.
	Enc_err_td watchdog;			// 148.
	Enc_err_td errtbl[4];			// 158	// Note: This takes over 128 bytes, but room for 256 in page zero of diags
	uint16_t end;					// 198.

}Enc_data_td;
#endif

#endif /* AC210_SIG100_H_ */
