/*
 * diags.c
 *
 *  Created on: 29/11/2016
 *      Author: Murray
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "global.h"
#include "diags.h"
#include "drive.h"
#include "param.h"
#include "control.h"
#include "uuencode.h"
#include "log.h"
#ifdef AC210_PORT
#include "ac210_logix.h"
#endif

// These 2 needed for AT_SYS function. Perhaps should move debug code to separate file?

#include "sstate.h"
#include "digital.h"
#include "analog.h"	// for A_MOTOR_CURRENT

/* Ideas: Extra check word at end of record so that stats do not get zeroed unless both check words are wrong.
 *
 * Ability to save stats and parameter record to a text file (that can be emailed to us)
 *
 * Ability to update information such as license number, maybe propeller type by Diagnostics program, but only
 * if (say) a special ini file present, with (say) a keyword and value. This could be kept in a special directory to avoid
 * accidentally sending it anywhere
 *
 * Record total motor usage, next to total time. Currently we do not record usage when no RPM.
 *
 * Extend error record so that it has some parts of stats record that are useful, and do away with error stats record.
 *
 * Set timer counter so that it is reset after each write.
 *
 */

//-------------------------------------------------------------------------

#define DIAGS_PERIOD		60		// seconds
#define SRC_DIAGS			WD_DIAGS

//int DiagLen;
//char DiagLine[80];
//BYTE SendDiags_err_tbl;
//BYTE SendDiags_flag;
//WORD SendDiags_ix;
WORD Diags_write_secs;
BYTE Diags_rpm_started;
BYTE Diags_motor_running;
BYTE Diags_param_update;
WORD Diags_version;

td_Stats Stat_Rec;
//td_Tot_TimeStats Diags_tot;
//td_Local_TimeStats Diags_run;
td_Error Diags_error;

//------------------------------------------------------------------------------
bool DebugAbort_active;
void DebugAbort(far char *emsg)		// So we can put a breakpoint without going through diagnostics
{
//	PRINTF_FLUSH;
	if(DebugAbort_active) return;		// In case recursion
	DebugAbort_active = true;
	L2PRINTF("DebugAbort:%s\r\n",emsg);	// MHH:09/08/2023
	DebugAbort_active = false;
#ifdef MH_XXX
	txDebug("DebugAbort:");
	txDebug(emsg);
	txDebug("\r\n");
#endif
#ifdef AC210_PORT
//	printf("DebugAbort:%s\r\n",emsg);
#endif
}
//---------------------------------------------------------------------------------------------
void Diags_send_eeprom(WORD u_prefix,int pos)
{
	int bytes_to_send;
	int bytes_left;
	WORD ulen;
	far uint8_t *psrc;

#ifdef AC210_PORT
	bytes_to_send = 1024;
//	printf("Diags_send_rec:sending current rec\r\n");

	if(pos == -1)		// From ee_prom?
	{
		p_eeprom_i2c_pos(pos);
		if(p_eeprom_i2c_read(UU_fBuff, bytes_to_send))
		{
		    txDebug(":A:ERR\r\n");
			DebugAbort("i2c_read");
		}
	}
	else
	{
		ee_getc_save_position();
		ee_getc_position(pos);
		for(int i=0;i<1024;i++) UU_fBuff[i] = ee_getc();
		ee_getc_restore_position();
	}

	td_Stats  *p_dr = (td_Stats *)(UU_fBuff+256);
	p_dr->log_filesize = Stat_Rec.log_filesize;	// Use this for progress bar.
//	p_dr->log_filesize = d_r->log_filesize;	// Use this for progress bar.

#else
	bytes_to_send = 512;

	I2C_read_page(UU_fBuff,0,256);			// Read 256 bytes into UU_fbuff
	I2C_read_page(UU_fBuff+256,1,256);	// Read next 256 bytes into UU_buff
#endif

	bytes_left = bytes_to_send;
	psrc = (far uint8_t *)UU_fBuff;
	while(bytes_left > 0)
	{
		int tlen = MIN(bytes_left,45);
		if(u_prefix == DIAGS_U_PREFIX)
		{
			send(":U:",3);
		}
		ulen = UUEncodeLine(psrc,tlen);
		UU_uBuf[ulen++] = 13;
		UU_uBuf[ulen++] = 10;
		send((char *)UU_uBuf,ulen);
//		txDebug((char *)UU_uBuf);
//		send("\r\n",2);
		bytes_left -= tlen;
		psrc += tlen;
	}
	txDebug(":A:EOF\r\n");

}
//---------------------------------------------------------------------------------------------
// Send Diags rec

#ifdef AC210_PORT
int Diags_last_drec_pos;
td_Stats Diags_last_d_r_sent;
#endif
//void Diags_send_rec(td_Stats *d_r,int pos)

void Diags_uuencode_send_buff(uint8_t *pbuff,int buff_len,int u_prefix)
{
	int bytes_left = buff_len;
	uint8_t *psrc = pbuff;
	int ulen;
	while(bytes_left > 0)
	{
		int tlen = MIN(bytes_left,45);
		if(u_prefix == DIAGS_U_PREFIX)
		{
			send(":U:",3);
		}
		ulen = UUEncodeLine(psrc,tlen);
		UU_uBuf[ulen++] = 13;
		UU_uBuf[ulen++] = 10;
		send((char *)UU_uBuf,ulen);
//		txDebug((char *)UU_uBuf);
//		send("\r\n",2);
		bytes_left -= tlen;
		psrc += tlen;
	}
	txDebug(":A:EOF\r\n");

}
void Diags_send_rec_all(void)
{
	Diags_last_drec_pos = -1;
	Diags_last_d_r_sent = Stat_Rec;		// Useful to get log info
	WriteStatsRec("Before Diags send",DIAGS_SHORT_LEN);
	int bytes_to_send = 1024;

	p_eeprom_i2c_pos(0);
	if(p_eeprom_i2c_read(UU_fBuff, bytes_to_send))
	{
	    txDebug(":A:ERR\r\n");
		DebugAbort("i2c_read");
	}
	Diags_uuencode_send_buff(UU_fBuff,bytes_to_send,DIAGS_NO_PREFIX);
}
#ifdef MH_XXX
void Diags_send_rec(td_Stats *d_r,int pos,int send_len)
{
	uint8_t *psrc;

	Diags_last_drec_pos = pos;
	Diags_last_d_r_sent = *d_r;		// Useful to get log info

/*
 * Change logic to send either first 512 bytes of eeprom (Ac200) or 1024 bytes (AC210)
 */

//#ifdef MH_XXX	// MHH:18/04/2026
	if(pos != -1)		// Not current DiagRec
	{
		if(send_len == DIAGS_LONG_LEN)
		{
			psrc = (uint8_t *)d_r;
			int bytes_left = sizeof(td_Stats);
	//		printf("Diags_send_rec:sending history rec\r\n");
			txDebug(":A:Diags_rec\r\n");

			while(bytes_left > 0)
			{
				int tlen = MIN(bytes_left,45);
				UUEncodeLine(psrc,tlen);
				txDebug((char *)UU_uBuf);
				send("\r\n",2);
				bytes_left -= tlen;
				psrc += tlen;
			}
			txDebug(":A:EOF\r\n");
			return;
		}
		mh_debug();
	}

//#endif
	WriteStatsRec("Before Diags send",DIAGS_SHORT_LEN);
	Diags_send_eeprom(DIAGS_NO_PREFIX,pos);
}
#endif
//------------------------------------------------------------------------------------------------
#define CAT_WATCHDOG	0
#define CAT_ABORT		1

int DiagsError(ULONG err)
{
	BYTE from_watchdog;
	ULONG _error;
	td_Error *pEtbl;

#ifdef MH_CHECK_ID		// Now checking in param.c:WriteStatsRec()

	int id = ((err / 1000)) * 1000;		// mask out sub_id
	if(id == SRC_DIAGS) return 1;			// Do nothing if called from this module.
#endif

	// Note: Still have to consider how to handle a watchdog timeout in diagnostics. Probably best to
	// not record either.

	from_watchdog = (err >= 200000);

	if(from_watchdog)
	{
		Stat_Rec.watchdog_restarts++;
		Stat_Rec.run_errs.watchdog_restarts++;
	}
	else
	{
		Stat_Rec.abort_restarts++;
		Stat_Rec.run_errs.abort_restarts++;
	}

	Diags_error.run_number = Stat_Rec.run_number;
	Diags_error.run_secs = Stat_Rec.run_secs;
	Diags_error.rpm_secs = Stat_Rec.run.rpmSecs.tot;
	Diags_error.err_num  = err;
	Diags_error.count = 1;

	// See if the same error as  recorded. If it is then just increment count.

	pEtbl = Stat_Rec.err_tbl;
	_error = pEtbl->err_num;
	if(err == _error)
	{
		Diags_error.count = pEtbl->count+1;
	}
	else
	{
		Stat_Rec.err_tbl[2] = Stat_Rec.err_tbl[1];	// Shuffle down
		Stat_Rec.err_tbl[1] = Stat_Rec.err_tbl[0];
	}
	Stat_Rec.err_tbl[0] = Diags_error;	// Move in latest error

	// Possibly could set watchdog on here....
	WriteStatsRec("DiagsError",DIAGS_LONG_LEN);

	return 0;
}
//------------------------------------------------------------------------------------------------
void Diags_update_rec(void);
void Sig100_zero_enc_data(void);
void Diags_format(void)
{
	Stat_Rec.rtc_start_secs = AC210_RTC_secs();
	Stat_Rec.log_flags = DIAGS_LOG_FLAG_EXTENDED;		// store params too.
	Stat_Rec.ac_version = version;					// MHH:17/04/2026
	Stat_Rec.hw_version = AC210_hardware_version;	// MHH:17/04/2026
	Stat_Rec.run_secs = 0;
	LogData.first_checkpoint = false;

}
//------------------------------------------------------------------------------------------------
void Diags_zero_rec(void )
{
	memset((BYTE *)&Stat_Rec,0,sizeof(Stat_Rec));
//	memset((BYTE *)&Diags_tot,0,sizeof(Diags_tot));
//	memset((BYTE *)&Diags_run,0,sizeof(Diags_run));

	Stat_Rec.check_code = DIAGS_REC_CHECK_CODE;
	Stat_Rec.run_number = 1;	// initial run number
//	Stat_Rec.pcb_version = pcbVersion ();
	Diags_format();
	Stat_Rec.log_last_diags_pos = EELOG_END_CHAIN;
   	WriteStatsRec("Diags_zero_rec",DIAGS_LONG_LEN);
   	Sig100_zero_enc_data();
	Diags_write_secs = 0;
//	Log_write_stat_index(&Stat_Rec,-1);		// MHH:04/02/2024
}
//------------------------------------------------------------------------------------------------
#define DIAGS_SEND_DATA_1	1
#define DIAGS_SEND_DATA_2	2

#define DIAGS_SELFTEST_11	65511
#define DIAGS_SELFTEST_12	65512

#define DIAGS_ZERO_REC		65534

void Diags_AT(WORD value)
{
	switch(value)
	{
	case 0:		// Send current stat rec if zero
		Diags_update_rec();
		Diags_send_rec_all();
		return;

	case DIAGS_ZERO_REC:
		Diags_zero_rec();
		return;

#ifdef AC210_PORT
	default:
		AC210_eelog_send_diag_rec(value);
		return;

	case DIAGS_SELFTEST_11:
		AC210_log_init();
		return;
#endif
	}
}
//------------------------------------------------------------------------------------------------
// Called when initialising...
void Diags_check_rec_size(void)
{
	int s1,s2,s3,s4;
	char stmp[40];
	int s=sizeof(Stat_Rec);
	if(s != DIAGS_LONG_LEN)
	{
		sprintf(stmp,"Diags_check_rec_size:s=%d\r\n",s);
		txDebug(stmp);
		s1 = sizeof(td_long_tstats);
		sprintf(stmp,"sizeof(td_long_tstats): %d\r\n",s1);
		txDebug(stmp);
		s2 = sizeof(td_short_tstats);
		sprintf(stmp,"sizeof(td_short_tstats): %d\r\n",s2);
		txDebug(stmp);
		s3 = sizeof(td_Tot_TimeStats);
		sprintf(stmp,"sizeof(td_Tot_TimeStats): %d\r\n",s3);
		txDebug(stmp);
		s1 = sizeof(td_short_TimeStats);
		sprintf(stmp,"sizeof(td_short_TimeStats): %d\r\n",s1);
		txDebug(stmp);
		s4 = sizeof(td_Error);
		sprintf(stmp,"sizeof(td_Error): %d\r\n",s4);
		txDebug(stmp);

		Abort(SRC_DIAGS+10,"Diags_check_rec_size:size not 256");
	}
}
//------------------------------------------------------------------------------------------------
void Diags_init_stats(void)
{
	Diags_check_rec_size();
}
//------------------------------------------------------------------------------------------------------------------------------
void UpdateStatsDate(ULONG yymmdd)
{
	if(Stat_Rec.start_date == 0)
	{
		Stat_Rec.start_date = yymmdd;
	}
	if(Diags_param_update)	// Set to true when copying params
	{
		Diags_param_update = false;
		Stat_Rec.param_update_date = yymmdd;
		Stat_Rec.log_flags |= DIAGS_LOG_FLAG_PARAMS;
//		Stat_Rec.param_update_run = Stat_Rec.run_number;
	}
	Stat_Rec.pc_user_date = yymmdd;
	Stat_Rec.log_flags |= DIAGS_LOG_FLAG_PC_DATE;
	WriteStatsRec("UpdateStatsDate",DIAGS_SHORT_LEN);
	Diags_write_secs = 0;
}
//------------------------------------------------------------------------------------------------------------------------------
void Diags_update_rec(void)	// Usually will be called once a minute, more if sent ATDIAGS=n
{
	Log_putc_write_buffer();			// MHH:24/12/2023. To be sure, won't take time if already flushed.
	Stat_Rec.log_filesize = AC210_current_log_filesize(3);
//	Stat_Rec.ac_version = (WORD)pcbVersion() * 1000 + version;
//	Stat_Rec.ac_version = version;	MHH:09/01/2019
}
//------------------------------------------------------------------------------------------------------------------------------
void Diags_update_secs(WORD op_mode)
{
//	Diags_run.rpmSecs.tot++;

	if(Stat_Rec.tot.rpmSecs.tot < Stat_Rec.run.rpmSecs.tot)	// This to try and trap bug
	{
		printf("tot.rpmSecs.tot:%d, run.rpmSecs.tot:%d\r\n",Stat_Rec.tot.rpmSecs.tot,Stat_Rec.run.rpmSecs.tot);
		DebugAbort("Diags_update_secs: tot < run");
		return;
	}

	Stat_Rec.tot.rpmSecs.tot++;
	Stat_Rec.run.rpmSecs.tot++;

	switch(op_mode)
	{
	case MANUAL:
//		Diags_run.rpmSecs.manual++;
		Stat_Rec.tot.rpmSecs.manual++;
		Stat_Rec.run.rpmSecs.manual++;
		break;

	case TAKEOFF:
//		Diags_run.rpmSecs.takeoff++;
		Stat_Rec.tot.rpmSecs.takeoff++;
		Stat_Rec.run.rpmSecs.takeoff++;
		break;

	case CLIMB:
//		Diags_run.rpmSecs.climb++;
		Stat_Rec.tot.rpmSecs.climb++;
		Stat_Rec.run.rpmSecs.climb++;
		break;

	case CRUISE:
//		Diags_run.rpmSecs.cruise++;
		Stat_Rec.tot.rpmSecs.cruise++;
		Stat_Rec.run.rpmSecs.cruise++;
		break;

	case HOLD:
//		Diags_run.rpmSecs.hold++;
		Stat_Rec.tot.rpmSecs.hold++;
		Stat_Rec.run.rpmSecs.hold++;
		break;

	default:
//		Diags_run.rpmSecs.other++;
		Stat_Rec.tot.rpmSecs.other++;
		Stat_Rec.run.rpmSecs.other++;
		break;
	}
}
//------------------------------------------------------------------------------------------------------------------------------
static BYTE motor_Tenths_tot;
static BYTE motor_Tenths_manual;
static BYTE motor_Tenths_takeoff;
static BYTE motor_Tenths_climb;
static BYTE motor_Tenths_cruise;
static BYTE motor_Tenths_hold;
static BYTE motor_Tenths_other;
// Note: This could be done with a table, but keep simple for now
void Diags_update_tenths(WORD op_mode)
{
	//	Diags_run.motorTenths.tot++;
	if(++motor_Tenths_tot >= 10)
	{
		motor_Tenths_tot=0;
		Stat_Rec.tot.motorSecs.tot++;
		Stat_Rec.run.motorSecs.tot++;
	}
	switch(op_mode)
	{
	case MANUAL:
//		Diags_run.motorTenths.manual++;
		if(++motor_Tenths_manual >= 10)
		{
			motor_Tenths_manual=0;
			Stat_Rec.tot.motorSecs.manual++;
			Stat_Rec.run.motorSecs.manual++;
		}
		break;

	case TAKEOFF:
//		Diags_run.motorTenths.takeoff++;
		if(++motor_Tenths_takeoff >= 10)
		{
			motor_Tenths_takeoff=0;
			Stat_Rec.tot.motorSecs.takeoff++;
			Stat_Rec.run.motorSecs.takeoff++;
		}
		break;

	case CLIMB:
//		Diags_run.motorTenths.climb++;
		if(++motor_Tenths_climb >= 10)
		{
			motor_Tenths_climb=0;
			Stat_Rec.tot.motorSecs.climb++;
			Stat_Rec.run.motorSecs.climb++;
		}
		break;

	case CRUISE:
//		Diags_run.motorTenths.cruise++;
		if(++motor_Tenths_cruise >= 10)
		{
			motor_Tenths_cruise=0;
			Stat_Rec.tot.motorSecs.cruise++;
			Stat_Rec.run.motorSecs.cruise++;
		}
		break;

	case HOLD:
//		Diags_run.motorTenths.hold++;
		if(++motor_Tenths_hold >= 10)
		{
			motor_Tenths_hold=0;
			Stat_Rec.tot.motorSecs.hold++;
			Stat_Rec.run.motorSecs.hold++;
		}
		break;

	default:
//		Diags_run.motorTenths.other++;
		if(++motor_Tenths_other >= 10)
		{
			motor_Tenths_other=0;
			Stat_Rec.tot.motorSecs.other++;
			Stat_Rec.run.motorSecs.other++;
		}
		break;

	}
}
//------------------------------------------------------------------------------------------------------------------------------
static bool Diags_opencircuit_started;
static bool Diags_overcurrent_started;
void Diags_update_circuit_vals(void)
{
// Only update opencircuit values if rpm detected
	if(Diags_rpm_started)
	{
		if(Diags_error.sstate & S_ERROR_OPEN)
		{
//			Diags_run.opencircuit_tenths++;
			Stat_Rec.run_errs.opencircuit_tenths++;
			if(Stat_Rec.run_errs.opencircuit_tenths % 10 == 0)
			{
				Stat_Rec.tot.circuit.opencircuit_secs++;
			}
			if(Diags_opencircuit_started == FALSE)
			{
				Diags_opencircuit_started = TRUE;
//				Diags_run.opencircuit_count++;
				Stat_Rec.run_errs.opencircuit_count++;
				Stat_Rec.tot.circuit.opencircuit_count++;
			}
		}
		else
		{
			Diags_opencircuit_started = FALSE;
		}
	}
// Always update overcurrent values

	if(Diags_error.sstate & S_ERROR_CURRENT)
	{
//			Diags_run.overcurrent_tenths++;
		Stat_Rec.run_errs.overcurrent_tenths++;
		if(Stat_Rec.run_errs.overcurrent_tenths % 10 == 0)
		{
			Stat_Rec.tot.circuit.overcurrent_secs++;
		}
		if(Diags_overcurrent_started == FALSE)
		{
			Diags_overcurrent_started = TRUE;
//				Diags_run.overcurrent_count++;
			Stat_Rec.run_errs.overcurrent_count++;
			Stat_Rec.tot.circuit.overcurrent_count++;
		}
	}
	else
	{
		Diags_overcurrent_started = FALSE;
	}
}
//------------------------------------------------------------------------------------------------------------------------------
void Diags_zero_run_totals(void)
{
	memset((BYTE *)&Stat_Rec.run,0,sizeof(td_short_TimeStats));	// zero this run's totals
	memset((BYTE *)&Stat_Rec.run_errs,0,sizeof(td_run_errs));
//	Stat_Rec.log_ticks = 0;
}
//------------------------------------------------------------------------------------------------------------------------------
//extern uint8_t I2C_hex_buff[];
void Diags_new_run(void)
{
#ifdef MH_YYY
#ifdef AC210_PORT
	if(LogData.serial_flag == 0)	// Don't write if serial logging
	{
		AC210_eelog_write_diags();
	}
#endif
#endif

//	int ps_len = MAX_P_INDEX * 2 + 6;
//	int hwf_len = 16;

	Log_Checkpoint(EELOG_C_CHECKPOINT_DIAGS);		// Note: Checkpoint flushes flash buffer
	int diags_pos = AC210_log.putc_pos;				// Position Diags_rec will be written to. MHH:17/04/2026.

	int current_log_filesize = AC210_current_log_filesize(2);
	if(current_log_filesize < 0) current_log_filesize = 0;		// An error?
	Stat_Rec.log_filesize = current_log_filesize + DIAGS_LONG_LEN;
	Log_put_data((uint8_t *)&Stat_Rec,DIAGS_LONG_LEN);				// Output diags_rec


	int ps_len = sizeof(ps);							// Output parameters for extended diagnostics
	int hwf_len = sizeof(hwf);
	Log_put_data((uint8_t *)&ps,ps_len);				// Output parameters
	int nulls_len = 256 - hwf_len - ps_len;
	Log_put_nulls(nulls_len);
	Log_put_data((uint8_t *)&hwf,hwf_len);				// Output hardware factors

	Log_putc_write_buffer();	// flush

	Log_write_stat_index(&Stat_Rec,diags_pos);		// To eeprom

	Stat_Rec.run_number++;

	if(LogData.enabled)
	{
		Stat_Rec.log_last_diags_pos = diags_pos;	// Update Stat_Rec with new log positions
		Stat_Rec.log_start_data_pos = AC210_log.putc_pos;		// Position data will start at
	}

	Stat_Rec.log_filesize = 0;			// MHH:24/12/2024
	Diags_format();
	Diags_zero_run_totals();
	//       	Diags_init_stats();		// MHH:10/12/2016

	DPRINTF("\r\nDiags_new_run:%d\r\n",Stat_Rec.run_number);

	WriteStatsRec("Diags_new_run",DIAGS_LONG_LEN);
	Log_write_stat_index(&Stat_Rec,-1);		// Write a dummy index entry. Hopefully this means we don't have to erase entire index on reset
}
//------------------------------------------------------------------------------------------------------------------------------
// called ten times a second from main
#define DIAGS_RPM_STARTED		1
#define DIAGS_RPM_STOPPED		2
#define DIAGS_TIMEOUT			3

static BYTE run_motor_tenths2;

//#define RPM_500		(500*RPMFACTOR)		// MHH:01/11/2018 As *10
#define RPM_500		(500)		// MHH:01/11/2018 As *10
#define RPM_STOPPED		0


static bool Diags_new_run_started;
int AC300_get_run_number(void)		// MHH:05/07/2025
{
	int run_number = Stat_Rec.run_number;
	if(Diags_new_run_started == false)
	{
		run_number++;
	}
	return run_number;
}
void UpdateStatsRecord(void)
{
	BYTE flag_write=0;
	BYTE cstate;
//	BYTE Diags_motor_running;
	WORD xx;
	WORD op_mode;
	WORD sstate;
	long setspeed,actualspeed;
    MotorDriveState dstate;
    static ULONG time_in_secs;
// Note: If we start and there is an error in current, perhaps copy to err_tbl?

    if(getParameter(DIAGS_ENABLE) == 0)		// Disabled?
    {
    	return;
    }
    if(ac2_test_flag == AC2_TEST_ON) return;

    if(TimeInSeconds == 0)	// Just started?
    {
    	return;			// MHH:28/12/2023. So that we get new run and start_checkpoint created together
    }
    AC210_logctl_check_run();

    setspeed    = currentSetSpeed ();
    actualspeed = currentActualSpeed ();

    sstate = systemState ();	// May need this to decide if motor going

    if(Diags_new_run_started == false)
    {
    	Diags_new_run_started = true;
    	Diags_new_run();
    }

#ifdef MH_XXX
    if(TimeInSeconds == 0)	// Just started?
    {
        if(actualspeed >= RPM_500)		// Probably a restart of MCU, don't change run number
//        if(actualspeed >= 500)		// Probably a restart of MCU, don't change run number
        {
        	Diags_rpm_started++;	// Pretend already started
        }
    }
#endif
    if(Diags_rpm_started == 0)
    {
        if(actualspeed >= RPM_500)
        {
        	Diags_rpm_started++;
        }
    }
    else
    {
    	if(actualspeed == RPM_STOPPED)		// MHH:01/11/2018 Has engine been turned off?
		{
    		Diags_rpm_started = 0;
        	flag_write = DIAGS_RPM_STOPPED;
		}
	}

    op_mode = Auto_operating_mode();
  	dstate = driveState();
  	Diags_error.mode = op_mode;
//  	Diags_error.act_rpm = actualspeed/RPMFACTOR;
//   	Diags_error.set_rpm = setspeed/RPMFACTOR;
  	Diags_error.act_rpm = actualspeed;
   	Diags_error.set_rpm = setspeed;
   	Diags_error.dstate = (BYTE)dstate;
    cstate = controlState(&xx);
    Diags_error.cstate = cstate;
    Diags_error.sstate = sstate;
    Diags_error.xoar_status = XoarStatus;
    Diags_error.mcurrent = ((scaledValue (A_MOTOR_CURRENT)+50)/100);	// In tenths of an amp
    Diags_update_circuit_vals();

	if(time_in_secs != TimeInSeconds)
	{
		time_in_secs = TimeInSeconds;
//		Diags_run.secs++;
		Stat_Rec.tot.secs++;
		Stat_Rec.run_secs++;
		if(Diags_rpm_started)
		{
			Diags_update_secs(op_mode);
		}
		if(++Diags_write_secs >= DIAGS_PERIOD) flag_write = DIAGS_TIMEOUT;
	}
// Now update motor stats if it is going


	Diags_motor_running = FALSE;
	if(sstate & (S_RUN_FINE | S_RUN_COARSE | S_RUN_FEATHER))
	{
		if((sstate & (S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER)) == 0)
		{
			Diags_motor_running = TRUE;
		}
	}
	if(Diags_motor_running)
	{
//		Diags_run.motor_tenths2++;
//		DPRINTF("R:%d,%d,%d\r\n",TimeInSeconds,run_motor_tenths2,Stat_Rec.tot.motor_secs2);
		if(++run_motor_tenths2 >= 10)
		{
			run_motor_tenths2 = 0;
			Stat_Rec.tot.motor_secs2++;
		}
	}
//	if(Diags_rpm_started)		// MHH:13/12/2023. Record motor running even when engine RPM = 0.
	{
//		if(dstate != MD_IDLE)
		if(Diags_motor_running)
		{
			Diags_update_tenths(op_mode);
		}
	}

	if(flag_write)
	{
		Diags_write_secs = 0;
		Diags_update_rec();

		switch(flag_write)
		{
#ifdef MH_XXX
		case DIAGS_RPM_STARTED:
			WriteStatsRec("stat_rpm_started",DIAGS_LONG_LEN);
			break;
#endif

		case DIAGS_RPM_STOPPED:
			WriteStatsRec("stat_rpm_stopped",DIAGS_SHORT_LEN);
			break;

		case DIAGS_TIMEOUT:
			WriteStatsRec("60 secs elapsed",DIAGS_SHORT_LEN);
			break;
		}
	}
}

