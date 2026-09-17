/*
 * diags.h
 *
 *  Created on: 29/11/2016
 *      Author: Murray
 */

#ifndef DIAGS_H_
#define DIAGS_H_
#include "global.h"

typedef struct
{
	WORD tot;
	WORD manual;
	WORD takeoff;
	WORD climb;
	WORD cruise;
	WORD hold;
	WORD other;
	WORD fill;		// align on 32 bit boundary
}td_short_tstats;

typedef struct
{
	ULONG tot;
	ULONG manual;
	ULONG takeoff;
	ULONG climb;
	ULONG cruise;
	ULONG hold;
	ULONG other;
}td_long_tstats;

typedef struct
{
	ULONG secs;
	ULONG motor_tenths2;		// Record even if no RPM
	BYTE overcurrent_started;	// Use as flags to indicate if continuous
	BYTE opencircuit_started;
	WORD overcurrent_count;
	WORD opencircuit_count;
	ULONG overcurrent_tenths;
	ULONG opencircuit_tenths;
	td_long_tstats rpmSecs;
	td_long_tstats motorTenths;
}td_Local_TimeStats;

typedef struct
{
	WORD overcurrent_count;
	WORD overcurrent_secs;
	WORD opencircuit_count;
	WORD opencircuit_secs;
} td_circuit;

typedef struct
{
	ULONG secs;
	ULONG motor_secs2;	// All motor secs, even when no RPM
	td_long_tstats rpmSecs;
	td_long_tstats motorSecs;
	td_circuit circuit;
	WORD fill[4];			// 80 bytes
}td_Tot_TimeStats;

typedef struct
{
	td_short_tstats rpmSecs;
	td_short_tstats motorSecs;

}td_short_TimeStats;

typedef struct
{
	ULONG err_num;		//0
	WORD run_number;	//4
	WORD run_secs;		//6
	WORD rpm_secs;		//8
	WORD set_rpm;		//10
	WORD act_rpm;		//12
	WORD sstate;		//14
	BYTE mode;			//16
	BYTE cstate;		//17
	BYTE coutput;		//18
	BYTE mcurrent;		//19 in tenths of an amp.
	BYTE dstate;		//20
	BYTE xoar_status;	//21
	BYTE count;			//22
	BYTE fill;			//23
						// 24 bytes
} td_Error;

// Note: This record should be initialised by AC210Programmer, or for AC210
// by AC200 User if it notices that date field is zero.


typedef struct
{
	BYTE watchdog_restarts;
	BYTE abort_restarts;
	BYTE overcurrent_count;
	BYTE opencircuit_count;
	WORD overcurrent_tenths;
	WORD opencircuit_tenths;
}td_run_errs;

typedef struct
{
	WORD check_code;			// (0)
	WORD log_flags;				// (2)
	WORD fill;					// (4) Not used
//	WORD pcb_version;			// (4) MHH:09/01/2019
	WORD run_number;			// (6)
	WORD watchdog_restarts;		// (8)
	WORD abort_restarts;		// (10)
	ULONG start_date;			// (12) date stats started YYYYMMDD. Could be short.
	ULONG run_secs;				// (16)
	long log_last_diags_pos;	// (20) -1 if first in chain.
	long log_start_data_pos;	// (24) -1 if first in chain
	td_Tot_TimeStats tot;		// (28) 80 bytes
	td_short_TimeStats run;		// (108)32 bytes
	long log_filesize;			// (140)
//	BYTE head_wrap_cnt;			// (144)
	BYTE fill0;					// (144)
	BYTE pc_prog;				// (145) 1 = User, 2 = Diagnostics, 3 = Programmer. Set by ATCODE=n
	WORD ac_version;			// (146) MHH:17/04/2026. Use again. ac_version already handled in Datahead_rec, so could reuse this field
	td_run_errs run_errs;		// (148)  8 bytes
//	ULONG diags_save_date;		// (156) MHH:31/12/2025. Not used. Was meant to be set by Diagnostics when saving, not used by AC210/AC200
	WORD fill1;					// (156)
	WORD hw_version;			// (158) MHH:17/04/2026.
	ULONG param_update_date;	// (160) MHH:17/04/2026. Note both these 32 bit dates could be converted 16 bit date values.
	ULONG pc_user_date;			// (164)
	ULONG rtc_start_secs;		// (168) MHH:26/12/2018.
//	WORD fill2[2];				// (168) 4 bytes

	td_Error err_tbl[3];		// (172) 72 bytes
	WORD fill2[4];				// (244) 8 bytes	// MHH:19/04/2026
	ULONG check2;				// (252) 4 bytes. May use as additional verification.
								// (256)
} td_Stats;
#define DIAGS_SHORT_LEN			172
//#define DIAGS_LONG_LEN			248
#define DIAGS_LONG_LEN			256			// MHH:19/04/2026
#define DIAGS_REC_CHECK_CODE	12345

#define DIAGS_LOG_FLAG_EXTENDED			1
#define DIAGS_LOG_FLAG_PC_DATE			2
#define DIAGS_LOG_FLAG_PARAMS			4

// Consider adding calibration flag.

extern td_Stats Stat_Rec;
extern BYTE Diags_param_update;
#ifdef AC210_PORT
extern int Diags_last_drec_pos;
extern WORD Diags_version;
extern bool Diags_RS232_page_logic;
extern td_Stats Diags_last_d_r_sent;
//#define FAR
#endif

extern char DiagLine[80];
extern BYTE Diags_motor_running;

void WriteStatsRec(char far *desc,WORD len);
//void WriteStatsRec(char desc,WORD len);
void Diags_init_stats(void);
void Diags_new_run(void);
void Diags_send_rec_all(void);
void Diags_send_rec(td_Stats far *d_r,int pos,int send_len);
void Diags_update_rec(void);
void Diags_uuencode_send_buff(uint8_t *pbuff,int buff_len,int u_prefix);

void Diags_send_UU_fbuff_page(int page);

#define DIAGS_NO_PREFIX			0
#define DIAGS_U_PREFIX			1
void Diags_send_eeprom(WORD u_prefix,int pos);

//void Diags_send_rec(far td_Stats *d_r,int pos);

//void Diags_ini_send(void);
void Diags_AT(WORD value);
void Param_init_read_diags(void);

#endif /* DIAGS_H_ */
