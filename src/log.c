/*
 * log.c
 *
 *  Created on: 14/02/2017
 *      Author: Murray
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "global.h"
#include "analog.h"
#include "control.h"
#include "diags.h"
#include "digital.h"
#include "drive.h"
#include "log.h"
#include "param.h"
#include "sstate.h"
#include "uuencode.h"

#ifdef AC210_PORT
#include "ac210_log.h"
#include "ac210_sig100.h"	// MHH:05/12/2022. For Sig100 data.
#endif

M8_T M8_data;
M8_T M8_dataB;		// Compare with new to find differences.

LogData_T LogData;
Log_checkpoint_T Log_checkpoint;

//-------------------------------------------------------------------------
// Only called if EEPROM_log set to zero
static bool Logging;
void Log_finish(void)
{
	if(LogData.serial_flag)
	{
		Log_putc(255);	// eof
	}
	if(Logging)
	{
		Logging = false;
		Log_putc_write_buffer();	// Consider check pointing
	}
	LogData.serial_flag = 0;
}
//-------------------------------------------------------------------------
void Log_change_rate(WORD value)	// from comms.c
{
	if(value == 0)
	{
		Log_finish();
	}
	if(value != EEPROM_log)
	{
		EEPROM_log = value;
		LogData.first_checkpoint = false;	// So we get a checkpoint on start
	}
}
//-------------------------------------------------------------------------
static BYTE Log_serial_val;		// Used to know if used by AC200 CLI or Ac200Diagnostics
#define LOG_SERIAL_DIAG		1
#define LOG_SERIAL_CLI		2

void Log_serial_command(WORD value)
{
	if(value == 0)
	{
		Log_finish();
//		txDebug("LOG_HEADER\r\n");
		WriteStatsRec("Log_serial_command",DIAGS_LONG_LEN);
		Diags_send_eeprom(DIAGS_U_PREFIX,0);
		Diags_send_rec_all();
//	    txDebug("LOG_FINISH\r\n");
		Log_serial_val = 0;
		LogData.serial_flag = 0;
		return;
	}
	EEPROM_log = 1;				// Set log sample rate to max of 50 per sec
	LogData.first_checkpoint = false;	// So we get a checkpoint on start
	LogData.enabled = true;
	LogData.log_ticks = 0;
	LogData.serial_write_ix  = 0;		// In case was logging to flash
	LogData.serial_flag = 1;
	Log_serial_val = value;
#ifdef MH_XXX
	if(value == LOG_SERIAL_CLI)				// Called from AC200CLI?
	{
// Note: Could send identifying header here, so Diagnostics knows if AC200 or AC210 when converting log file
	    txDebug("LOG_START\r\n");
		Diags_send_eeprom(DIAGS_U_PREFIX);
	}
#endif
}
//-------------------------------------------------------------------------
// Usually called once a minute
void Log_put_data(uint8_t *p_data,int plen)	// Same as previous, but no header or len or checksum to be compatible with existing
{
	for(int i=0;i<plen;i++)
	{
		uint8_t b = *p_data++;
		Log_putc(b);	// Output data
	}
}
void Log_put_nulls(int len)
{
	for(int i=0;i<len;i++)
	{
		Log_putc(0);	// Output data
	}
}
void Log_Checkpoint(uint8_t checkpoint_code)
{
//	int i;
//	uint8_t c;
//	uint8_t *bp;

	if(LogData.enabled == false) return;		// MHH:20/12/2023

	// First zero both saved entries to force all non-zero fields to be written next log
	LogData.checkpoint_secs = 0;
	if(checkpoint_code == EELOG_C_CHECKPOINT_TIMEOUT)
	{
		if(LogData.serial_write_ix == 0) return;	// Only checkpoint timeout if data in buffer
	}
	Log_checkpoint.extended = 0;
	if(checkpoint_code == EELOG_C_CHECKPOINT_START)
	{
		LogData.first_checkpoint = true;
//		Log_checkpoint.extended = 'X';		// MHH:20/04/2026
	}
	Log_putc(EELOG_C_ESCAPE1);
	Log_putc(EELOG_C_ESCAPE2);
	Log_putc(checkpoint_code);

	if(checkpoint_code == EELOG_C_CHECKPOINT_DIAGS)
	{									// Here if new run
		LogData.log_ticks = 0;
	}
	else
	{
		Log_checkpoint.run=Stat_Rec.run_number;
		Log_checkpoint.ticks_per_log = EEPROM_log;

		Log_checkpoint.m8_data = M8_data;
		M8_dataB = M8_data;						// make sure up to date
		Log_put_data((uint8_t *)&Log_checkpoint,LOG_CHECKPOINT_SIZE);	// Output checkpoints
	}
#ifdef MH_XXX
	if(Log_checkpoint.extended == 'X')
	{
		int ps_len = sizeof(ps);
		int hwf_len = sizeof(hwf);
		Log_put_data((uint8_t *)&ps,ps_len);				// Output parameters
		int nulls_len = 256 - hwf_len - ps_len;
		Log_put_nulls(nulls_len);
		Log_put_data((uint8_t *)&hwf,hwf_len);				// Output hardware factors
	}
#endif
	Log_putc(EELOG_C_ESCAPE3);	// End of checkpoint
	Log_putc_write_buffer();
}
//-------------------------------------------------------------------------
// Called from main.c
extern uint8_t Sig100_bad_receive_code;
extern uint16_t Ctl_deadband;
extern WORD lastSpeed;
//static ULONG last_time_in_secs;
extern MotorDriveState dState;
extern WORD currentState;
extern WORD Auto_throttle;

extern uint8_t BL_command;
extern uint8_t BL_speed;

extern BYTE RC_S_mode_byte;	// MHH:10/06/2025

//#define NOT_ACTIVE_SECS		(50*2)			// 2 secs

void AC210_eelog(void)
{
//	static uint16_t count_not_running;
//	static long actualspeed;
//	static uint16_t display_rpm;
	uint16_t actualspeed;
	uint16_t setspeed;
//	static uint16_t last_manual_speed;
//	static int manual_unchanged_count;
	uint8_t op_mode;

	WORD pwm_val;

#ifdef MH_CHECK_TIME
	static uint32_t last_t2;
	uint32_t t2;
	uint32_t t_elapsed;
#endif

	if(LogData.enabled == false) return;	// So we don't overwrite if no magic

	if(TimeInSeconds == 0)
	{
		LogData.log_ticks = (50 / EEPROM_log);	// Set initial tick to 1 second. 50 = AC10 cycles/sec
		return;			// Takes a while to settle. Thinks it is in manual mode initially
	}

#ifdef AC210_PORT
	if(LogData.log_ticks * EEPROM_log > (TimeInSeconds+1) * 50)
	{
		DPRINTF("AC210_eelog:log_ticks:%d,TimeInSeconds:%d\r\n",LogData.log_ticks,TimeInSeconds);
		DebugAbort("Bad log_tick value");
		return;
	}
#endif


#ifdef MH_XXX
	if(TimeInSeconds != last_time_in_secs)
	{
		last_time_in_secs = TimeInSeconds;
		if(++LogData.checkpoint_secs >= EELOG_CHECKPOINT_SECS)
		{
#ifdef MH_CHECK_TIME
			t2 = us_ticker_read();
			t_elapsed = t2 - last_t2;
			last_t2 = t2;
			printf("Log_checkpoint: Elapsed:%d secs",(t_elapsed/1000000));
#endif
			Log_Checkpoint(EELOG_C_CHECKPOINT_TIMEOUT);
		}
	}
#endif
	if((LogData.log_ticks > 1) && ((LogData.log_ticks % EELOG_CHECKPOINT_TICKS) == 0))	// MHH:22/04/2026
	{
		Log_Checkpoint(EELOG_C_CHECKPOINT_TIMEOUT);
	}
	if(LogData.putc_flush)
	{
		Log_putc_write_buffer();
	}

	Logging = true;
	LogData.ticks_per_log = EEPROM_log;
	LogData.log_ticks++;

	M8_data.ticks = LogData.log_ticks;
	actualspeed = (uint16_t)(currentActualSpeed());
	setspeed    = (uint16_t)(currentSetSpeed ());	// MHH: 19/11/2020. Was failing if > 65535 as casting not bracketed correctly
	op_mode     = (uint8_t)Auto_operating_mode();

    if(op_mode == MANUAL)
    {
    	setspeed = actualspeed;
    	M8_data.deadband = 0;
    }
    else
    {
    	M8_data.deadband = Ctl_deadband;
    }
    int temp_code = 255;		// MHH:04/02/2026
    if(AC210_remote_control_board)
    {
    	int ac_temperature = scaledValue(A_TEMPERATURE);
    	temp_code = ac_temperature + 50;	// Fit into 1 byte
    	if(temp_code < 0) temp_code = 0;
    	if(temp_code > 254) temp_code = 254;
    }
    M8_data.ac_temperature = temp_code;
    M8_data.actualspeed   = actualspeed;
    M8_data.setspeed      = setspeed;
    M8_data.current       = (uint16_t)(scaledValue (A_MOTOR_CURRENT))/10;

    if(Sig100_connected)
    {
    	if(M8_data.current < 12) M8_data.current = 0;	// stored in hundreths of an amp.
//        M8_data.pwm_val = 0;
    	if(operatingMode () > TAKEOFF || operatingMode() < HOLD) Engine_rpm_change_in_five_ticks = 0;
        M8_data.pwm_val = (uint8_t)(Engine_rpm_change_in_five_ticks);	// MHH:30/01/2026
    }
    else
    {
        M8_data.pwm_val       = (uint8_t)lastSpeed;		// Not sure what controlState is returning in pwm_val
    }

    M8_data.sys_state     = (uint16_t) systemState ();
    M8_data.xoar_status   = XoarStatus;

	M8_data.control_state = controlState(&pwm_val);

    M8_data.manual_keys   = (uint8_t)manualKeys();

    M8_data.op_mode       = op_mode;
    M8_data.motor_drive_state  = (uint8_t) driveState();

    M8_data.throttle = Auto_throttle;		// MHH:05/05/2023
/*
    uint8_t new_voltage = (uint8_t)(scaledValue (A_SUPPLY)+1)/2;	// Accuracy of +- 0.2V
    uint8_t diff        = abs(new_voltage - M8_data.voltage);
    if(diff > 1)M8_data.voltage = new_voltage;
*/
//    M8_data.voltage       = (uint8_t)(scaledValue (A_SUPPLY)+3)/5;	// Accuracy of +- 0.5V
    M8_data.voltage       = (uint8_t)((scaledValue (A_SUPPLY)+1)/2);	// MHH:20/09/2025. From version 10.138  Accuracy of +- 0.2V
#ifdef MH_SLIPRING_DATA
    M8_data.slipring      = ADC_get_slipring_log_data();	// MHH:1/4/23. Remove slipring data - not used after finding brush problem.
#endif
    float angle = AC210_SIG100_angle();		// MHH:05/12/2022.Brushless fields added.
	angle *= 10;	// Get ready to convert to integer with 1 implied decimal place round
//    M8_data.bl_angle      = (int16_t) angle;
    M8_data.bl_angle      = (int16_t) iround(angle);	// MHH:27/11/2025
    int v = (int)SIG100_get_hub_voltage();	// MHH:20/09/2025
    M8_data.bl_voltage = (v+1)/2;
//    M8_data.bl_voltage = (ADC_BL_corrected_voltage+1)/2;	// MHH:20/09/2025. Same format as voltage
//    M8_data.bl_voltage    = ADC_adjusted_voltage;	// Hopefully smoothed by averaging in lpc845
//    M8_data.bl_voltage    = ((ADC_adjusted_voltage + 2)>>2)<<2;	// Lose 2 significant bits, so takes less data with variations
//    M8_data.bl_voltage    += M8_data.bl_voltage;	// * 2
    M8_data.bl_temperature = ADC_adjusted_temp;
    M8_data.bl_line_err_code = Sig100_bad_receive_code;

    M8_data.bl_command = BL_command;
    M8_data.bl_speed = BL_speed;
//    M8_data.bl_speed = Ctl_brushless_speed;	// MHH:06/02/2025. May fix bug?
    M8_data.bl_flags = BL_flags;
    M8_data.rc_s_mode = RC_S_mode_byte;		// MHH:10/06/2025

    // M8_data.bl_error = not coded yet.....
    M8_data.bl_enc_rpm = BL_enc_rpm;
    M8_data.bl_dac0_v  = BL_dac0_v;
    if(Log_pack_data())
    {	// This way should always write whole drecs
    	if(LogData.serial_write_ix >= EE_SER_WRITE_TRIGGER)
    	{
    		Log_putc_write_buffer();
    	}
#ifdef MH_RB_LOGIC
    	ee_check_data();
#endif
    }

/*
#define MANUAL_KEY_FEATHER 0x01
#define MANUAL_KEY_COARSE  0x02
#define MANUAL_KEY_FINE    0x04

// accessor function
BYTE manualKeys (void);
*/
    /* from sstate.h
#define S_IDLE          (WORD)0x0000

#define S_RUN_FINE      (WORD)0x0001
#define S_RUN_COARSE    (WORD)0x0002
#define S_RUN_FEATHER   (WORD)0x0004
#define S_RUN_REVERSE   (WORD)0x0004

#define S_STOP_FINE     (WORD)0x0010
#define S_STOP_COARSE   (WORD)0x0020
#define S_STOP_FEATHER  (WORD)0x0040
#define S_STOP_REVERSE  (WORD)0x0040

#define S_ERROR_OPEN    (WORD)0x0100
#define S_ERROR_CURRENT (WORD)0x0200
#define S_ERROR_SWITCH  (WORD)0x0400
#define S_ERROR_VOLTAGE (WORD)0x0800
*/

}
//-------------------------------------------------------------------------
void Log_putc(uint8_t c)
{
	if(LogData.serial_write_ix >= EE_SER_WRITE_MAX)
	{
		Log_putc_write_buffer();
	}
	ee_serial_write_buff[LogData.serial_write_ix++] = c;
//	printf("<%d=%02x>",LogData.putc_pos2,c);
#ifdef AC210_PORT
	if(LogData.serial_flag == 0)	Log_update_flash_pos();
#endif
}
//---------------------------------------------------------------------------------------
int Log_pack_data(void)
{
	uint8_t *p_new;
	uint8_t *p_old;
	uint8_t offset;
	uint8_t byte;
	int ix_first = 0;
	int ix_last = 0;
	int i;
	int ix;

// First step is to see if there are any changes. We exclude ticks from compare.
// Because we need to know where last changed byte is, we will start from last and move towards first.

	p_new = (uint8_t *) &M8_data;
	p_old = (uint8_t *) &M8_dataB;
	for(i=M8_SIZE-1;i>3;i--)
	{
		if(p_new[i] != p_old[i])
		{
			ix_last = i;
			break;
		}
	}
//#ifdef MH_XXX		// MHH:13/12/2023
	if(LogData.first_checkpoint == false)	// Force a checkpoint first time
	{
		Log_Checkpoint(EELOG_C_CHECKPOINT_START);
		return 0;
	}
//#endif

	if(ix_last == 0) return 0;		// No changes.

// OK, now to send the offset of each changed byte then the byte.
// Only send ticks if the difference between last sent is more than 1

	if(M8_data.ticks == M8_dataB.ticks+1) 	// We want to force ticks to be written at least once per 10 drecs to help with checking data
	{
		if(LogData.no_tick_count++ < 10)
		{
			ix_first = 4;
		}
	}
	if(ix_first < 4) LogData.no_tick_count=0;

	for(ix=ix_first;ix<=ix_last;ix++)
	{
		if(p_new[ix] != p_old[ix])
		{
			offset = (uint8_t)(ix + 1);		// add 1 so we never have a zero. May use zero byte for special purpose
			if(ix == ix_last) offset |= M8_HIBIT;	// so we know it is last of this tick
			byte   = p_new[ix];
			Log_putc(offset);
			Log_putc(byte);
		}
	}

	M8_dataB = M8_data;				// update dataB
	return 1;
}
//----------------------------------------------------------------------------
//#define EE_SER_BUFF_SIZE	100
uint8_t ee_serial_write_buff[EE_SER_BUFF_SIZE];
//uint8_t LogData.serial_write_ix=0;
//uint8_t LogData.serial_flag;

void Log_serial_output(void)
{
	uint8_t *psrc;
	int bytes_left;
	int tlen;

	psrc = (uint8_t *)ee_serial_write_buff;;
	bytes_left = LogData.serial_write_ix;
	while(bytes_left > 0)
	{
		tlen = MIN(bytes_left,45);
//		send(":D:",3);
		send(":U:",3);		// 28/02/2017
		UUEncodeLine(psrc,tlen);
		txDebug((char *)UU_uBuf);
		send("\r\n",2);
		bytes_left -= tlen;
		psrc += tlen;
	}
//	txDebug(":A:EOF\r\n");
	LogData.serial_write_ix = 0;
}
//----------------------------------------------------------------------------
void Log_putc_write_buffer(void)
{
	LogData.putc_flush = false;
	if(LogData.serial_write_ix == 0) return;	// Not sure about this...but allows us to call even if not sure

	if(LogData.enabled == false)	// MHH:22/12/2023
	{
		LogData.serial_write_ix = 0;
		return;
	}


	if(LogData.serial_flag)
	{
		Log_serial_output();
		return;
	}
#ifdef AC210_PORT
//	Log_putc_write_buffer_flash();
	Log_putc_write_buffer_flash_2();
#endif
}
