/*
 * ac210_logix.c
 *
 *  Created on: 13/04/2017
 *      Author: Murray
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ac210_global.h"
#include "ac210_log.h"
#include "ac210_logix.h"
#include "ac210_ssp.h"

#include "global.h"
#include "analog.h"
#include "control.h"
#include "diags.h"
#include "digital.h"
#include "drive.h"
#include "param.h"
#include "sstate.h"
#include "uuencode.h"

//--------------------------------------------------------------------------------------------------------------
typedef struct
{
	int run;
	int data_pos;
	int diag_pos;
	int eof_pos;
	td_Stats d_r;
} SCAN_TD;
static SCAN_TD Scan;


//-----------------------------------------------------------------------------------------
void Disable_log_data(int ifrom)
{
	L2PRINTF("Disable_log_data:from:%d\r\n",ifrom);
	LogData.enabled = false;	// Turn logging off
}
//-----------------------------------------------------------------------------------------
LOG_INDEX_T Log_index;
int Index_page = -1;
//#define INDEX_PAGE_SIZE		128
uint8_t Index_page_buff[INDEX_PAGE_SIZE];
//-----------------------------------------------------------------------------------------
LOG_INDEX_T *pIndex;
//--------------------------------------------------------------------------------------------------------------
static int Index_get_page(int index_page)
{
	int index_pos = EE_LOG_INDEX_OFFSET + index_page * INDEX_PAGE_SIZE;
	p_eeprom_i2c_pos(index_pos);
	if(p_eeprom_i2c_read(Index_page_buff,INDEX_PAGE_SIZE))
	{
		DebugAbort("Index_get_page:p_eeprom_i2c_read fail");
		return -1;
	}
	Index_page = index_page;
	return 0;
}
//--------------------------------------------------------------------------------------------------------------
int Index_get_index(int run)
{
	int index = (run - LOG_INDEX_ADJUST) % LOG_INDEX_MAX;
	int index_page  = index / INDEX_PAGE_MAX;
	int index_ix    = index % INDEX_PAGE_MAX;
	if(index_page != Index_page)
	{
		if(Index_get_page(index_page))
		{
			return -1;
		}
	}
	pIndex = (LOG_INDEX_T *)Index_page_buff;
	pIndex += index_ix;
	return 0;
}
//-----------------------------------------------------------------------------------------
static int Log_get_index_pos(int run)
{
	int run_ix = (run - LOG_INDEX_ADJUST) % LOG_INDEX_MAX;
	int ix_offset = run_ix * LOG_INDEX_ELE_SIZE;
	int pos = EE_LOG_INDEX_OFFSET + ix_offset;
	return pos;
}
//-----------------------------------------------------------------------------------------
void Log_write_stat_index(td_Stats *p_DR,int log_diag_pos)
{
	LOG_INDEX_T log_index;

	int run = p_DR->run_number;
	int pos = Log_get_index_pos(run);
	log_index.run   = run;
	log_index.fill	= 0;	// keep tidy
	log_index.fill2 = 0;
	log_index.log_start_pos = p_DR->log_start_data_pos;
;
	log_index.log_diag_pos  = log_diag_pos;
//	log_index.head_wrap_cnt = p_DR->head_wrap_cnt;	// hopefully this will be same as first checkpoint
//	log_index.head_wrap_cnt = LogCtl.head_wrap_cnt;
	p_eeprom_i2c_pos(pos);
	uint8_t *bp = (uint8_t *)&log_index;
	p_eeprom_i2c_write(bp,LOG_INDEX_ELE_SIZE);
	Index_page = -1;		// Force reload of index page
}
//-----------------------------------------------------------------------------------------
int Log_read_stat_index(int run)
{
	int pos = Log_get_index_pos(run);
	p_eeprom_i2c_pos(pos);
	uint8_t *bp = (uint8_t *)&Log_index;
	if(p_eeprom_i2c_read(bp,LOG_INDEX_ELE_SIZE))
	{
		DebugAbort("Log_read_stat_index:p_eeprom_i2c_read fail");
		return -1;
	}
	return 0;
}
//----------------------------------------------------------------------------------------------------
int AC210_logix_erase(WORD value)
{
	int run = value;
	if(run >= Stat_Rec.run_number)
	{
		PRINTF("Run too high\r\n");
		return 1;
	}
	if(run < 1)
	{
		PRINTF("Run too low\r\n");
		return 1;
	}
	if(Log_read_stat_index(run))
	{
		PRINTF("Error reading index\r\n");
		return 1;
	}
	if(Log_index.run == LOG_INDEX_ERASED_RUN)
	{
		PRINTF("run already erased\r\n");
		return 1;
	}
	if(Log_index.run != run)
	{
		PRINTF("run != index run (%d)\r\n",Log_index.run);
		return 1;
	}
	memset((char *)&Log_index,255,LOG_INDEX_ELE_SIZE);
	int pos = Log_get_index_pos(run);
	p_eeprom_i2c_pos(pos);
	uint8_t *bp = (uint8_t *)&Log_index;
	p_eeprom_i2c_write(bp,LOG_INDEX_ELE_SIZE);
	Index_page = -1;		// Force reload of index page
	PRINTF("Run:%d erased\r\n",run);
	return 0;
}
//----------------------------------------------------------------------------------------------------
/*
 * MHH:03/08/2023. Instead of current log run index which uses 12 bytes per entry, could just us a single 4 byte pointer to the diagnostic rec.
 * We could have an entry in the LogCtl structure indicating the new log index structure.
 * We can verify the run by reading the ssp log entry pointed to by the pointer.
 * A zero offset pointer is valid, so would need a high value to indicate an unused or erased entry.
 *
 *  It could be simply rebuilt by logic similar to that in Build_run_ptr_table() in AC200Diagnostics:Form1.cs
 *
 *
 *  The start position of the data would be implied by the position of the diagnostic rec for the previous run. If not correct, we could scan for it using
 *  similar logic to rebuild of index.
 *
 *  Currently the logic supports 1000 entries (12,000 bytes) and high run numbers overwrite lower. E.g 1010 run would overwrite run 10 index struct.
 * 	It is helpful to have the run number in the index, so see below for solution.
 *
 * 	Because the pointer can fit in 3 bytes, we can use the spare byte to hold the high order of the run number. If we allow (say) 1024 * 8 bytes for the table
 * 	this would allow up to max of 2048 entries (4 bytes per entry). 1024 * 16 would give us 4096 maximum entries, if this fits in the eeprom.
 *
 * 	The eeprom is 1 Mbit or 128k bytes, so plenty of room.
 *
 * 	 We could easily allocate 32k bytes, giving a max of 8k run ptrs. This would make it highly unlikely that a new run could be overwritten by an old one
 * 	 when rebuilding the ptr table, but to be on the safe side we could use the spare byte (or nibble if necessary) to hold the high bits of the run number.
 *
 * 	 As the run number is 2 bytes, and the position implies the first 13 bits, then in a 16 bit number there is only 3 extra bits which could be stored
 * 	 in the high nibble.
 *
 * 	 We could use the high bit to indicate deleted or unused.
 *
 *	 The only situation that might be tricky is run 1. We could treat it as a special case, and always assume data starts at offset zero. That might
 *	 be simplest.
 *
 *	 We could convert the current log index format to the new format by just scanning and building the table, so existing systems could be easily
 *	 updated.
 *
 *	 We could even have both tables in eeprom memory initially, and have the page offset in the LogCtrl structure.
 *
 *	 Would be useful to have a routine which displayed the index in simple format. E.g:
 *	 Run number, diags_ptr, possible like:
 *	 		0		1		2		3		4		5		6		7		8		9
 *  0000	ptr0	ptr1	ptr2	ptr3	ptr4	ptr5	ptr6	ptr7	ptr8	ptr9
 *  0010
 *  0020
 *
 *	 and one page at a time, with page as an argument. Run zero would be treated as deleted, but run 8192 (wrapped) would not.
 *
 *
 * typedef struct
{
	uint16_t run;
	uint8_t head_wrap_cnt;
	uint8_t fill;		// Note: This could be used for flags. Eg Data corrupt, Diags Corrupt, maybe Data overwritten?
	int log_start_pos;
	int log_diag_pos;
} LOG_INDEX_T;
 */
void AC210_display_logctl(void)
{
	PRINTF("magic.......%d\r\n",LogCtl.magic);
	PRINTF("head_page...%d\r\n",LogCtl.head_page);
	PRINTF("tail_page...%d\r\n",LogCtl.tail_page);
//	PRINTF("head_wrap_cnt....%d\r\n",LogCtl.head_wrap_cnt);
//	PRINTF("tail_wrap_cnt....%d\r\n",LogCtl.tail_wrap_cnt);
	PRINTF("max_pages...%d\r\n",LogCtl.max_pages);
	PRINTF("last_flash_sector_erased..%d\r\n",LogCtl.last_flash_sector_erased);
}
//--------------------------------------------------------------------------------------------------------------
//static bool Log_index_rebuild;

// Return codes

#define SCAN_ERROR				-1

#define SCAN_EOF				1
#define SCAN_CHECKPOINT_START	2
#define SCAN_CHECKPOINT_TIMEOUT	3
#define SCAN_DIAG_REC			4


uint32_t Log_peek4(void)
{
	uint8_t buff4[4];
	ee_getc_save_position();
	for(int i=0;i<4;i++) buff4[i] = ee_getc();
	ee_getc_restore_position();
	return Get_uint32(buff4);
}
#ifdef MH_XXX
uint32_t Log_peek4_pos(int pos)
{
	uint8_t buff4[4];
	ee_getc_save_position();
	ee_getc_position(pos);
	for(int i=0;i<4;i++) buff4[i] = ee_getc();
	ee_getc_restore_position();
	return Get_uint32(buff4);
}
#endif

static int Log_scan_next_run(int run)
{
//	td_Stats d_r; TimeInSeconds
	ULONG timeinseconds = RTC_secs;

	uint8_t b,c;

	if(gVerbose >= 10) PRINTF("Log_scan_next_run:Looking for next run\r\n");
	int eof_cnt = 0;
	int eof_pos = 0;
	Scan.run = -1;
	Scan.data_pos = -1;
	Scan.diag_pos = 0;
	Scan.eof_pos = 0;

	for(int i=0;;i++)
	{
		int escape_pos = AC210_log.getc_pos2;
		b = ee_getc();
		if(b == FLASH_EOF)
		{
			if(eof_cnt++ == 0)
			{
				eof_pos = AC210_log.getc_pos2;
			}
			if(eof_cnt > 5)
			{
				PRINTF("Flash EOF\r\n");
				Scan.eof_pos = eof_pos;
				return SCAN_EOF;
			}
			continue;
		}
		eof_cnt = 0;
		if(b == EELOG_C_ESCAPE1)
		{
			b = ee_getc();
			if(b == EELOG_C_ESCAPE2)
			{
				uint8_t type = ee_getc();
				switch(type)
				{
				case EELOG_C_CHECKPOINT_START:
					if(gVerbose >= 10) PRINTF("Found Checkpoint_start\r\n");
					if(ee_get_checkpoint_data() == 0)
					{
						if(gVerbose >= 10) PRINTF("Checkpoint Ok, run number:%d\r\n",Log_checkpoint.run);
						if(Log_checkpoint.run < run) continue;	// Only if there are more than 1000 runs
						if(Log_checkpoint.run > run)
							{
								Scan.run = Log_checkpoint.run;
								Scan.data_pos = escape_pos;
							}
					}
					continue;

				case EELOG_C_CHECKPOINT_TIMEOUT:
					if(gVerbose >= 10) PRINTF("Found Checkpoint_timeout\r\n");
					if(ee_get_checkpoint_data() == 0)
					{
						if(gVerbose >= 10) PRINTF("Checkpoint Ok, run number:%d\r\n",Log_checkpoint.run);
						if(RTC_secs >= timeinseconds + 3)
						{
							timeinseconds = RTC_secs;
							PRINTF("Checkpoint run:%d,page:%d\r\n",Log_checkpoint.run,escape_pos/1024);
						}
						if(Log_checkpoint.run < run) continue;	// Only if there are more than 1000 runs
						if(Log_checkpoint.run > Scan.run)
							{
								Scan.run = Log_checkpoint.run;
								Scan.data_pos = escape_pos;
							}

						// Not sure what to do if we have next run. Could make that the start of run.
					}
					if(gVerbose >= 10) PRINTF("Scanning for next checkpoint\r\n");
					continue;		// MHH:03/01/2026

				case EELOG_C_CHECKPOINT_DIAGS:
					if(gVerbose >= 10) PRINTF("Found Diags Record Checkpoint:\r\n");
					c = ee_getc();
					if(c == EELOG_C_ESCAPE3)
					{
						int diags_pos = AC210_log.getc_pos2;
#ifdef MH_XXX
						if(Log_peek4() == PARAM_CHECK_CODE)	// MHH:18/04/2026. Param rec?
						{
							AC210_log.getc_pos2 += 256;	// Yes
						}
#endif
						if(Log_read_diag_rec(&Scan.d_r) == 0)
						{

//							if(Scan.run != Scan.d_r.run_number) continue;
//							if(Scan.data_pos != Scan.d_r.log_start_data_pos) continue;	// Eg if first checkpoint overwritten
							// Because we have started scan after the tail_pos, then this run file should be OK
							Scan.diag_pos = diags_pos;
							if(gVerbose >= 10) PRINTF("Diags Record OK, run:%d, diag_pos:%d\r\n",Scan.run,Scan.diag_pos);
							return SCAN_DIAG_REC;
						}
					}
					else
					{
						if(gVerbose >= 10) PRINTF("Expecting ESCAPE3\r\n");
					}
					continue;
				}
			}
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
int Log_drec_cnt;
//static int Log_find_start_data_pos(void);

//--------------------------------------------------------------------------------------------------------------
// The plan is to write an index entry for every diagnostic record that we can, without checking to see if internal data is OK.
// We should be able to adapt diagnostics program to filter out bad data.
//-----------------------------------------------------------------------------------------
static int Logix_erase_index(void)
{
	Index_page = -1;
	memset(Index_page_buff,255,INDEX_PAGE_SIZE);
	p_eeprom_i2c_pos(EE_LOG_INDEX_OFFSET);		// 1024

/*
#define EE_LOG_INDEX_OFFSET	1024
#define LOG_INDEX_MAX		1000		// Arbitrary
#define LOG_INDEX_ELE_SIZE	(sizeof(LOG_INDEX_T))
#define LOG_INDEX_MAX_POS	(EE_LOG_INDEX_OFFSET + LOG_INDEX_MAX*LOG_INDEX_ELE_SIZE)
#define INDEX_PAGE_MAX		(INDEX_PAGE_SIZE/LOG_INDEX_ELE_SIZE)
*/

	for(int pos = EE_LOG_INDEX_OFFSET;pos < LOG_INDEX_MAX_POS;pos+=INDEX_PAGE_SIZE)
	{
		if(p_eeprom_i2c_write(Index_page_buff, INDEX_PAGE_SIZE)) {
			Disable_log_data(100);
//			LogData.enabled = false;	// Turn logging off
//			DebugAbort("EE_erase_index_and_logctl:i2c_write fail");
			return -1;
		}
	}
	return 0;
}
//--------------------------------------------------------------------------------------------------------------
// Use similar logic to scanning index for AC200Diagnostics history option
/*
 * Possibilities are:
 * X = erased page
 * d = data
 * H = Head page
 * T = Tail Page
 * F = Run file
 * Fs = File start
 * Fd= File diagsrec
 *
 *             1         2         3         4
 *   01234567890123456789012345678901234567890123456789
 *
 *   T         0                 H
 *  |dddddddddddddddddddddddddddddXXXXXXXXXXXXXXXXXXXXX|		// A:case where there has been no wrap
 *        FFFF													// T=0,Fs>T,Fd>Fs,Fd<H
 *
 *         H   0 T
 *  |dddddddXXXXXdddddddddddddddddddddddddddddddddddddd|		// B:case H<T, but first erased page not page zero.
 *   FFF                                              F			// File is wrapped
 *   															// T=20,Fs>T,Fd<Fs,Fd<H. H2 = H+50, Fs2=Fs+50
 *   															// Fs > T, Fd2 > Fs, Fd2<H2
 *
 *         H   0 T
 *  |dddddddXXXXXdddddddddddddddddddddddddddddddddddddd|		// B2:case H>T, where there has been wrap, but first erased page not page zero.
 *   FFFF                                               		// File is not wrapped
 *                                                              // H2=H+50,Fs2=Fs+50,Fd2=Fd+50
 *
 *         H   0 T
 *  |dddddddXXXXXdddddddddddddddddddddddddddddddddddddd|		// B3:case where there has been wrap, but first erased page not page zero.
 *                                                FFFFF 		// File is not wrapped
 *
 *
 *       T     0                                     H
 *  |XXXXdddddddddddddddddddddddddddddddddddddddddddddX|		// C:case where erased pages wrap
 *                                                FFFF
 *
 *      T      0                                      H
 *  |XXXddddddddddddddddddddddddddddddddddddddddddddddd|		// D:case where first page erased but no erase wrap
 *                                                 FFFF
 *
 *
 *   T         0                                    H
 *  |ddddddddddddddddddddddddddddddddddddddddddddddddXX|		//E: case where last page erased, but header yet to wrap
 *   FFFF
 *
 *typedef struct
{
	uint16_t run;
//	uint8_t head_wrap_cnt;	// MHH:11/12/2023
	uint8_t fill;
	uint8_t fill2;		// Note: This could be used for flags. Eg Data corrupt, Diags Corrupt, maybe Data overwritten?
	int log_start_pos;
	int log_diag_pos;
} LOG_INDEX_T;
 *
 *
 */



void AC210_index_verify(void)
{
	td_Stats d_r;
//	int log_file_size;
	int run;

	AC210_watchdog_active = false;				// This could take a while

	int tail_page = LogCtl.tail_page;		// MHH:11/12/2023. Assume that this has been fixed before we get here.
	int head_page = LogCtl.head_page;

	int pages_used = head_page - tail_page;
	bool head_wrap = false;
	if(pages_used < 0)
	{
		pages_used += FLASH_PART_SIZE;
		head_wrap = true;
	}
	int tail_pos = tail_page * FLASH_PAGE_SIZE;

	PRINTF("Index_verify:\r\n");
	PRINTF("Pages used  :%d\r\n",pages_used);	// Could call AC210_display_logctl() for more info

	int first_run = Stat_Rec.run_number - LOG_INDEX_MAX;
	first_run = MAX(1,first_run);
//	int dpos = AC210_log.putc_pos2;	// probably
	int dpos;

	Index_page = -1;	// In case of restore or reset


//	uint32_t tail_pos = LogCtl.last_flash_sector_erased * FLASH_SECTOR_SIZE;	// Check if data has been erased
//	uint32_t tail_address = (LogCtl.tail_wrap_cnt << 24) + tail_pos;
	int ecnt=0;
	int tot_run_errors = 0;
	int lcnt=0;
	int log_start_pos;

	bool file_wrap = false;

	Log_putc_write_buffer();	// flush

	for(run = Stat_Rec.run_number;run >= first_run;run--)
	{
		if(ecnt) tot_run_errors++;
		ecnt = 0;
		PRINTF("Run:%6d:",run);
		if(run == Stat_Rec.run_number)		// First time?
		{
#ifdef MH_PUTC_POS2
			dpos = AC210_log.putc_pos2;	// probably
#else
			dpos = AC210_log.putc_pos;	// probably
#endif
			d_r = Stat_Rec;		// Get current Stat rec
			log_start_pos = d_r.log_start_data_pos;
		}
		else
		{
			Index_get_index(run);
			if(pIndex->run == LOG_INDEX_ERASED_RUN)	// 65535
			{
				PRINTF("**Data has been erased for this run (%d)\r\n",run);
				continue;
			}
			if(pIndex->run != run)
			{
				ecnt++;
				PRINTF("**Index run (%d) not equal run (%d)\r\n",pIndex->run,run);
				continue;

			}
			int data_pos = pIndex->log_start_pos;	// MHH:11/12/2023
			log_start_pos = data_pos;
			int diag_pos = pIndex->log_diag_pos;
			int data_pos2 = data_pos;				// Use the pos2 values to check for wrap
			int diag_pos2 = diag_pos;
			if(head_wrap)
			{
				data_pos2 += FLASH_PART_SIZE;
				diag_pos2 += FLASH_PART_SIZE;
			}
			if(file_wrap)
			{
				if(data_pos > diag_pos)			// Should not wrap twice!
				{
					ecnt++;
					PRINTF("!!! Double wrap?? Should never get here, index may be corrupt\r\n");
					break;
				}
				else
				{
					data_pos2 -= FLASH_PART_SIZE;
					diag_pos2 -= FLASH_PART_SIZE;
				}
			}
			else
			{
				if(data_pos > diag_pos)
				{
					file_wrap = true;
					data_pos2 -= FLASH_PART_SIZE;
				}
			}
			if(data_pos2 < tail_pos)
			{
				PRINTF("**Data_start < tail, this run file overwritten.");
				if(tail_page != 0)
				{
					PRINTF("Normal exit for wrapped log.\r\n");
				}
				break;			// Suspect overwritten
			}


			dpos = diag_pos;
			if(eelog_read_diag_rec(&d_r,dpos))
			{
				ecnt++;
				PRINTF("**Invalid Diagnostic Record for run\r\n")
				continue;
			}
			if(d_r.run_number != run)
			{
				ecnt++;
				PRINTF("**Log run (%d) not equal index run (%d)\r\n",d_r.run_number,run);
				continue;
			}
		}
		if(log_start_pos == -1)
		{
			PRINTF("log_start_data_pos = -1\r\n");
			continue;

		}
		else
		{
			if(log_start_pos < 0 || log_start_pos > FLASH_PART_SIZE)
			{
				ecnt++;
				PRINTF("**Invalid log_start_pos for run (%d)\r\n",log_start_pos);
				continue;
			}
		}
		if(eelog_read_checkpoint_rec(EELOG_C_CHECKPOINT_START,log_start_pos))
		{
			ecnt++;
			PRINTF("**Invalid Checkpoint start record for run\r\n")
			continue;
		}
		if(Log_checkpoint.run != run)
		{
			ecnt++;
			PRINTF("**Checkpoint run (%d) not equal index run (%d)\r\n",Log_checkpoint.run,run);
			continue;
		}
		PRINTF("OK\r\n");
		if(lcnt++ >= 40)
		{
			lcnt = 0;
			if(PC_ReturnToContinue() == 'X')
			{
				return;
			}
		}
	}
	if(ecnt) tot_run_errors++;
	PRINTF("Finished Index verify, total index errors:%d\r\n",tot_run_errors);
}
//--------------------------------------------------------------------------------------------------------------
#ifdef MH_XXX
static int Get_checkpoint_start(int data_pos)
{
	// Check to see if we have a valid CHECKPOINT_START

	ee_getc_position(data_pos);
	uint8_t c = ee_getc();
	if(c != EELOG_C_ESCAPE1)
	{
		PRINTF("Expecting ESCAPE1\r\n");
		return -1;
	}
	c = ee_getc();
	if(c != EELOG_C_ESCAPE2)
	{
		PRINTF("Expecting ESCAPE2\r\n");
		return -1;
	}
	c = ee_getc();
	if(c != EELOG_C_CHECKPOINT_START)
	{
		PRINTF("Expecting CHECKPOINT_START\r\n");
		return -1;
	}
	ee_checkpoint_get_data();
	c = ee_getc();
	if(c != EELOG_C_ESCAPE3)
	{
		PRINTF("Expecting ESCAPE3\r\n");
		return -1;
	}
	return 0;
}
#endif
//--------------------------------------------------------------------------------------------------------------
#ifdef MH_YYY
static int Find_start_data_pos(int oldest_run)
{
	PRINTF("Find_start_data_pos:\r\n");

	// We need to scan from tail_pos to find the first diagnostic record, then use that information to see if
	// the data starts after the tail

#ifdef MH_XXX
	uint32_t tail_pos = 0;
	if(LogCtl.tail_wrap_cnt > 0)
	{
		tail_pos = LogCtl.last_flash_sector_erased * FLASH_SECTOR_SIZE;
	}
#endif
	uint32_t tail_pos = LogCtl.tail_page * FLASH_PAGE_SIZE;

	PRINTF("Starting scan at position:\%d\r\n",tail_pos);

	ee_getc_position(tail_pos);

	for(int i=0;;i++)
	{
		int scan_code = Log_scan_next_run(oldest_run);
		switch(scan_code)
		{
		case SCAN_EOF:
			PRINTF("SCAN_EOF\r\n");
			return -1;

		case SCAN_DIAG_REC:
			PRINTF("SCAN_DIAG_REC: Run:%d, data_pos:%d\r\n",Scan.run,Scan.data_pos);
			return Scan.data_pos;

		default:
			PRINTF("Unknown scan_code");
			DebugAbort("Unknown scan_code");
			return -1;
		}
	}
	return -1;	// Should never get here
}
#endif
//--------------------------------------------------------------------------------------------------------------
//uint8_t UU_fBuff[UU_FBUFF_SIZE];
#ifdef MH_YYY
int Logctl_read_page(int page)
{
	int pos = EELOG_START_POS + page * EELOG_PAGESIZE;

	int rv = AC210_ssp_flash_read(pos,UU_fBuff,EELOG_PAGESIZE);
	if(rv)
	{
		L2PRINTF("Logctl_read_page::Bad read:%d\r\n",rv);
	}
	return rv;
}
#endif
void Logctl_display(td_LOGCTL *pLog_ctl)
{
	if(gVerbose == 0) return;

	PRINTF("LogCtl:\r\n");
	PRINTF("magic...............%d\r\n",pLog_ctl->magic);
	PRINTF("head_page...........%d\r\n",pLog_ctl->head_page);
	PRINTF("tail_page...........%d\r\n",pLog_ctl->tail_page);
	PRINTF("last_sector_erased..%d\r\n",pLog_ctl->last_flash_sector_erased);
	PRINTF("run_number..........%d\r\n",pLog_ctl->run_number);
	PRINTF("max_pages...........%d\r\n",pLog_ctl->max_pages);
	int pages_used = pLog_ctl->head_page - pLog_ctl->tail_page + 1;
	if(pages_used < 0) pages_used += FLASH_PART_PAGES;	// MHH:31/12/2025
	PRINTF("Pages used..........%d\r\n",pages_used);	// Could call AC210_display_logctl() for more info

	PRINTF("\r\n");
	PRINTF_FLUSH;
}

void Logctl_display2(void)
{
	DPRINTF("LogCtl:\r\n");
	DPRINTF("magic...............%d\r\n",LogCtl.magic);
	DPRINTF("head_page...........%d\r\n",LogCtl.head_page);
	DPRINTF("tail_page...........%d\r\n",LogCtl.tail_page);
	DPRINTF("last_sector_erased..%d\r\n",LogCtl.last_flash_sector_erased);
//	DPRINTF("head_wrap_count.....%d\r\n",LogCtl.head_wrap_cnt);
//	DPRINTF("tail_wrap_count.....%d\r\n",LogCtl.tail_wrap_cnt);
	DPRINTF("run_number..........%d\r\n",LogCtl.run_number);
	DPRINTF("max_pages...........%d\r\n",LogCtl.max_pages);
	DPRINTF("\r\n");
	DPRINTF_FLUSH;
}



int ee_write_logctl(void);
#ifdef MH_YYY
bool Is_page_erased(void)
{
	uint8_t b;
	for(int bx=0;bx<EELOG_PAGESIZE;bx++)	// memcmp faster...
	{
		b = UU_fBuff[bx];
		if(b != 255)
		{
			return false;
		}
	}
	return true;
}
#endif
/*
 * Possibilities are:
 * X = erased page
 * d = data
 * H = Head page
 * T = Tail Page
 *
 *   T                           H
 *  |dddddddddddddddddddddddddddddXXXXXXXXXXXXXXXXXXXXXX|		// A:case where there has been no wrap
 *
 *         H     T
 *  |dddddddXXXXXddddddddddddddddddddddddddddddddddddddd|		// B:case where there has been wrap, but first erased page not page zero.
 *
 *       T                                            H
 *  |XXXXddddddddddddddddddddddddddddddddddddddddddddddX|		// C:case where erased pages wrap
 *
 *      T                                              H
 *  |XXXdddddddddddddddddddddddddddddddddddddddddddddddd|		// D:case where first page erased but no erase wrap
 *
 *   T                                               H
 *  |dddddddddddddddddddddddddddddddddddddddddddddddddXX|		//E: case where last page erased, but header yet to wrap
 *
 *
 *
 */
#ifdef MH_YYY
bool Is_this_page_erased(int page_pos)
{
	int read_len = 16;
	int read_pos = page_pos + FLASH_PAGE_SIZE - read_len;		// Read last 16 bytes of page first
	AC210_flash_read_part(read_pos,UU_fBuff,read_len);
	if(ee_check_all_255(UU_fBuff,read_len) == false)	// last 16 bytes of page erased?
	{
		return false;		// No
	}
	read_pos = page_pos;	// Yes, check entire page erased
	read_len = FLASH_PAGE_SIZE;
	AC210_flash_read_part(read_pos,UU_fBuff,read_len);
	if(ee_check_all_255(UU_fBuff,read_len) == false)	// entire page erased?
	{
		return false;		// No
	}
	return true;		// Entire page erased

}
#endif

void 	AC210_logctl_repair2(void );
void AC210_logctl_repair(void)
{
	AC210_watchdog_active = false;				// This could take a while
	AC210_logctl_repair2();
	AC210_watchdog_active = true;
}


void AC210_logix_build(void)
{
//	ParamStore *param;
	AC210_watchdog_active = false;				// This could take a while
	char *pdesc = "Index_build";

	// Step 1 is to erase the index file.
	PRINTF("%s:\r\n",pdesc);
	PRINTF("Erasing existing index...\r\n");

	if(Logix_erase_index())
	{
		PRINTF(":A:ERR:I2C write fail\r\n");
		return;
	}
	PRINTF("Erased\r\n")

	// Ok, now have to scan for the first valid Diagnostics record, similar to Data_verify()

	int pages_used = LogCtl.head_page - LogCtl.tail_page;
//	if(pages_used < 0) pages_used += EELOG_SPACE;
	if(pages_used < 0) pages_used += EELOG_MAX_PAGES;	// MHH:07/12/2023

	if(gVerbose > 0)
	{
		PRINTF("%s:\r\n",pdesc);
		PRINTF("Pages used  :%d\r\n",pages_used);	// Could call AC210_display_logctl() for more info
	}

	Index_page = -1;	// In case of restore or reset

	int oldest_run = Stat_Rec.run_number - LOG_INDEX_MAX;
	oldest_run = MAX(1,oldest_run);

	// We are really after the position of the first diagnostics record
#ifdef MH_XXX
	uint32_t tail_pos = 0;
	if(LogCtl.tail_wrap_cnt > 0)
	{
		tail_pos = LogCtl.last_flash_sector_erased * FLASH_SECTOR_SIZE;
	}
#endif
	uint32_t tail_pos = LogCtl.tail_page * FLASH_PAGE_SIZE;

	if(gVerbose > 0) PRINTF("Starting scan at position:\%d\r\n",tail_pos);

	ee_getc_position(tail_pos);

	int scan_code = -1;
//	int last_run = 0;
	while(scan_code != SCAN_EOF)
	{
		scan_code = Log_scan_next_run(oldest_run);
		if(gVerbose > 0) PRINTF_FLUSH;
		switch(scan_code)
		{
		case SCAN_EOF:
			break;

		case SCAN_DIAG_REC:
/*

 We need to consider possibility that there may be some missing run files.
 One possibility is to just leave a gap of high vals in the index position
 then any routine which scans the index will need to be aware that high vals does
 have to mean the end of the index. Perhaps we could write a special index
 as the last one, say an index of all zeroes.

 The other way of doing it is to always use the current stat_rec number as the
 highest possible run number. So, a routine such as ATXISHOW would not terminate on
 high vals, but on the run number. Probably simplest solution.

*/
			if(gVerbose > 0) PRINTF("SCAN_DIAG_REC: Run:%d, data_pos:%d\r\n",Scan.run,Scan.data_pos);
			if(Scan.run < 0 || Scan.data_pos < 0)	// MHH:01/09/2026
			{
				if(gVerbose > 0) PRINTF("No data checkpoint, ignoring\r\n")
				break;
			}
			PRINTF("Adding run: %d\r\n",Scan.run);
			Scan.d_r.log_start_data_pos = Scan.data_pos;		// MHH:05/01/2026
			Log_write_stat_index(&Scan.d_r,Scan.diag_pos);		// Update index
#ifdef MH_XXX			// MHH:02/09/2026. Debug code, not needed for rebuild.
			if(Scan.d_r.log_flags & DIAGS_LOG_FLAG_EXTENDED)
			{
				AC210_flash_read_part(Scan.diag_pos+256,UU_fBuff,256);
				param = (ParamStore *) &UU_fBuff;
				long check_code = param->checkCode;
				if(check_code != PARAM_CHECK_CODE)
				{
					PRINTF("Extended Diagnostics record but cannot find Param record\r\n");
				}

			}
#endif
			if(Scan.run >= Stat_Rec.run_number -1)
			{
				scan_code = SCAN_EOF;		// This should cause exit while
//				PRINTF(":A:EOF\r\n");
//				return;
			}
			break;

		default:
			PRINTF(":A:ERR:Unknown scan_code\r\n");
			return;
		}
		if(gVerbose > 0) PRINTF_FLUSH;
	}
	if(scan_code == SCAN_EOF)
	{
		if(Stat_Rec.log_last_diags_pos != Scan.diag_pos)	// Should not matter with index, but...
		{
			Stat_Rec.log_last_diags_pos = Scan.diag_pos;
			PRINTF("Updating current diag_rec pointer (%d) to run %d\r\n",Scan.diag_pos,Scan.run);
			WriteStatsRec("AC210_logix_build",DIAGS_LONG_LEN);
		}

		PRINTF(":A:EOF\r\n");
		return;
	}
	PRINTF(":A:ERR:Error exit\r\n");
}

//--------------------------------------------------------------------------------------------------------------
// Instead of scanning the index file, then checking it against the data file, we will scan the data file and check it
// against the index file

bool Test_page_erased(uint8_t *bp)
{
	for(int i=0;i<FLASH_PAGE_SIZE;i++)
	{
		if(*bp++ != 255) return false;
	}
	return true;
}

int Find_actual_page(int ipage)
{
	int max_pages = LogCtl.max_pages;
	if(max_pages == 0) max_pages = FLASH_PART_PAGES;

	int page = ipage;
	if(page < 0) page = max_pages + page;
	page = page % max_pages;
	return page;
}
int Get_log_page(int ipage)
{
	int page = Find_actual_page(ipage);
	int page_pos = page * FLASH_PAGE_SIZE;
	return AC210_flash_read_part(page_pos,UU_fBuff,FLASH_PAGE_SIZE);
}

bool Check_log_page_erased(int ipage)
{
	Get_log_page(ipage);
	return Test_page_erased(UU_fBuff);
}
td_LOGCTL TestLogCtl;
int Find_correct_logctl_data(void)
{
	PRINTF("Find_correct_logctl_data:searching for correct head and tail page...\r\n");

	int page;
	int first_erased_page = -1;
	int max_pages = LogCtl.max_pages;
	if(max_pages == 0) max_pages = FLASH_PART_PAGES;

	int first_page = ((AC210_log.eof_page / 64) * 64) + 63;
	for(page = first_page;page < max_pages;page += 64)	// Read last page of each sector
	{
		if(Check_log_page_erased(page))
		{
			first_erased_page = page;
			break;
		}
	}
	if(first_erased_page == -1)		// Maybe try something else?
	{
		PRINTF("Cannot find first erased page\r\n");
		return -2;
	}
	PRINTF("First erased page:%d\r\n",first_erased_page);
	int tail_page = -1;
	if(Check_log_page_erased(max_pages - 1))	// Special case for tail page if we have not been around the clock
	{
		if(Check_log_page_erased(0) == false)
		{
			tail_page = 0;
		}
	}

/*
 * Now we know first and last erased sectors, scan backwards up to 2 sectors for head_page and forwards up to 2 sectors for tail page
 */

	int head_page = -1;
	for(page = first_erased_page-1;page >= first_erased_page - 128;page--)
	{
		if(Check_log_page_erased(page) == false)
		{
			head_page = page;
			break;
		}
	}
	if(head_page == -1)
	{
		PRINTF("Could not find head_page\r\n");
		return 1;
	}

	int last_erased_page = first_erased_page;
	int tail_page2 = -1;
	for(page = first_erased_page + 1;page <= first_erased_page + 128;page++)
	{
		if(Check_log_page_erased(page) == false)
		{
			tail_page2 = page;
			break;
		}
		last_erased_page = page;
	}
	if(tail_page == -1)
	{
		tail_page = tail_page2;
	}

	if(tail_page == -1)
	{
		PRINTF("Could not find tail_page\r\n");
		return -2;
	}
	if((tail_page % 64) != 0)		// As we delete 64 pages at a time.
	{
		PRINTF("tail_page = %d, expecting it to be a multiple of 64\r\n",tail_page);
		return -2;
	}
	head_page = Find_actual_page(head_page);
	tail_page = Find_actual_page(tail_page);
	TestLogCtl.magic = LOGCTL_MAGIC;
	TestLogCtl.head_page = head_page;
	TestLogCtl.tail_page = tail_page;
	TestLogCtl.max_pages = max_pages;
	TestLogCtl.last_flash_sector_erased = ((last_erased_page-63)/FLASH_LOG_PAGES_PER_SECTOR) + FLASH_SECTOR_OFFSET;
	if(gVerbose)
	{
		PRINTF("TestLogCtl:\r\n");
		Logctl_display(&TestLogCtl);
	}
	return 0;
}
int AC300_map_logdata_file(int ival)
{
	int lpage=0;
	int lsector = 0;
	AC210_watchdog_active = false;				// This could take a while
	PRINTF("\r\nAC300_map_logdata_file\r\n");
	char c=0;
//	for(lsector=0;lsector <=10;lsector++)
	for(lsector=0;lsector <=254;lsector++)
	{
		PRINTF("\r\nSector:%3d,Page:%5d: ",lsector,lsector*64);
		for(int sec_page=0;sec_page<=63;sec_page++)
		{
			if(sec_page != 0 && sec_page %10 == 0) PC_putc(32);
			lpage = lsector*64 + sec_page;
			c = 'D';			// default to data
			if(Check_log_page_erased(lpage))
			{
				c = '.';		// Page is erased
			}
			PC_putc(c);
		}
	}
	PRINTF("\r\n\r\nFinished\r\n");
	AC210_watchdog_active = true;
	return 0;
}
int Verify_logctl_data(bool fix_logctl)
{


	if(gVerbose > 0)
	{
		Logctl_display(&LogCtl);
		PRINTF("AC210_log.eof_page:%d\r\n",AC210_log.eof_page);
	}

	TestLogCtl = LogCtl;
	int page_err = 0;

	if(LogCtl.magic != LOGCTL_MAGIC)
	{
		page_err++;
		PRINTF("LogCtl.magic = %d\r\n",LogCtl.magic);
	}
/*
 *  First verify head and tail page.
 *  If page is a head page, then the next page should always be erased (all 255).
 *  If page is a tail page then previous page should be erased (all 255).
 *  Maximum sectors = 255 for logical file.
 *  Pages per sector = 64.
 *  Maximum pages for logical file = 64 * 255 = 16320 = FLASH_PART_PAGES
 *  First page = 0, last page = 16319
 *
 *  If page < 0 then page = logical page + FLASH_PART_PAGES
 *  page = page % FLASH_PART_PAGES
 */

	if(Check_log_page_erased(LogCtl.tail_page))
	{
		PRINTF("tail_page erased\r\n");
		page_err++;
	}

	if(Check_log_page_erased(LogCtl.tail_page - 1) == false)
	{
		PRINTF("tail_page -1 not erased\r\n");
		page_err++;
	}
	if(Check_log_page_erased(LogCtl.head_page))
	{
		PRINTF("head_page erased\r\n");
		page_err++;
	}

	if(Check_log_page_erased(LogCtl.head_page+1) == false)
	{
		PRINTF("head_page+1 not erased\r\n");
		page_err++;
	}
	if((LogCtl.tail_page % 64) != 0)
	{
		PRINTF("Expecting tail page:%d as first page of sector\n",LogCtl.tail_page);
		page_err++;
	}

	int max_pages = LogCtl.max_pages;
	if(max_pages == 0) max_pages = FLASH_PART_PAGES;

	TestLogCtl.max_pages = max_pages;

	if(page_err)
	{
		L2PRINTF("LogCtl page error:\r\n");
		int rv = Find_correct_logctl_data();
		if(rv != 0) return rv;
		if(fix_logctl)
		{
			LogCtl = TestLogCtl;
			L2PRINTF("Fixing LogCtl\r\n)");
			return ee_update_logctl();
		}
	}
	return 0;
}
void AC210_logctl_repair2(void)
{
	PRINTF("AC210_logctl_repair:\r\n");

#ifdef MH_YYY
	int head_page = -1;
	int tail_page = -1;
	int last_page_erased=-1;
	int first_page_erased = -1;

	int page_pos;
	int first_data_page = -1;
	int last_data_page = -1;
	for(int log_sector = 0;log_sector<255;log_sector++)
	{
		page_pos = (log_sector + 1) * FLASH_SECTOR_SIZE - FLASH_PAGE_SIZE;
		if(Is_this_page_erased(page_pos))	// Last page of sector erased?
		{
			last_page_erased = page_pos / FLASH_PAGE_SIZE;
			page_pos = log_sector * FLASH_SECTOR_SIZE;		// Check first page of sector
			if(Is_this_page_erased(page_pos))
			{
				if(first_page_erased < 0)
				{
					first_page_erased = page_pos / FLASH_PAGE_SIZE;
				}
			}
			else
			{
				int sector_pos = page_pos;
				for(int sector_page=1;sector_page<64;sector_page++)	// We know first page data, last page erased.Find last data page
				{
					int sector_page_pos = sector_pos + sector_page * FLASH_PAGE_SIZE;
					if(Is_this_page_erased(sector_page_pos))
					{
						if(first_page_erased < 0) first_page_erased = sector_page_pos / FLASH_PAGE_SIZE;
						break;
					}
					else
					{
						int data_page = sector_page_pos / FLASH_PAGE_SIZE;
						if(data_page > last_data_page) last_data_page = data_page;
					}
				}
			}
		}
		else
		{
			last_data_page = page_pos / FLASH_PAGE_SIZE;
			if(first_data_page < 0) first_data_page = last_data_page - 63;	// Because if last page data, then first must be to with sector erase
		}
	}

	PRINTF("End of log data loop\r\n");
//	PRINTF_FLUSH;

	if(last_page_erased == -1)	// Note: Could decide to erase a sector and set things accordingly.
	{
		PRINTF("?No erased pages found\r\n");
		return;
	}

	if(first_data_page > 0 && last_data_page < (FLASH_PART_PAGES - 1))	// Test for wrap of erased pages
	{
		PRINTF("Erased pages wrap, getting head and tail from data page values\r\n");
		head_page = first_data_page;
		tail_page = last_data_page;
	}
	else
	{
		head_page = first_page_erased - 1;
		tail_page = last_page_erased + 1;
	}

	head_page %= FLASH_PART_PAGES;			// In case of wrap
	tail_page %= FLASH_PART_PAGES;

	PRINTF("head_page = %d, tail_page = %d\r\n",head_page,tail_page);

	uint32_t last_sector_erased = (head_page / 64) + 1;		// 64 pages in sector, plus the unused first sector. Possible that it is one sector short but
															// that will not matter as next sector will be erased and updated if necessary



	/*
	 * #define LOGCTL_REC_SIZE		(sizeof(td_LOGCTL))
typedef struct
{
	int magic;
	int head_page;
	int tail_page;					// tail_page and last_flash_sector_erased probably should be updated together.
	int last_flash_sector_erased;
	uint8_t head_wrap_cnt;	// Incremented every time head wraps
	uint8_t tail_wrap_cnt;	// Incremented every time tail wraps
	WORD run_number;		// Same value as run_number in current stat_rec.
	int max_pages;
} td_LOGCTL;
extern td_LOGCTL LogCtl;
	 *
	 */
#endif

	Find_correct_logctl_data();
	PRINTF("Existing LogCtl settings:\r\n");
	Logctl_display(&LogCtl);
#ifdef MH_YYY
	td_LOGCTL log_ctl;

	log_ctl.magic = LOGCTL_MAGIC;
	log_ctl.head_page = head_page;
	log_ctl.tail_page = tail_page;
	log_ctl.last_flash_sector_erased = last_sector_erased;
	log_ctl.run_number = Stat_Rec.run_number;
	log_ctl.max_pages = EELOG_MAX_PAGES;
#endif
	PRINTF("Proposed LogCtl settings:\r\n");
	Logctl_display(&TestLogCtl);
	PRINTF("Update LogCtl (Y/N)? <N>");
	for(;;)
	{
		int c = PC_getc();
		if(c != -1)
		{
			if(c == 'y' || c == 'Y')
			{
				PRINTF("Updating..\r\n");
				LogCtl = TestLogCtl;
				ee_write_logctl();
			}
			break;
		}
	}
	PRINTF("Finished\r\n");

}
void AC210_data_verify(void)
{
	td_Stats d_r;
//	Log_checkpoint_T run_checkpoint;
	LOG_INDEX_T run_log_index;	// So we don't conflict with Log_index
//	int run;

//	char *pdesc = "Data_verify";
	PRINTF("Data_verify\r\n");

	AC210_watchdog_active = false;				// This could take a while

	if(Verify_logctl_data(false))
	{
		return;
	}
	LogCtl = TestLogCtl;

	Index_page = -1;	// In case of restore or reset

	int oldest_run = Stat_Rec.run_number - LOG_INDEX_MAX;
	oldest_run = MAX(1,oldest_run);

#ifdef MH_XXX

	int start_data_pos = 0;
	if(LogCtl.tail_wrap_cnt > 0 || oldest_run != 1)
	{
		start_data_pos = Find_start_data_pos(oldest_run);	// Note: another way is to filter out older runs in main loop
	}
#endif
	int start_data_pos = 0;
	if(LogCtl.tail_page > LogCtl.head_page || oldest_run != 1)
	{
//		start_data_pos = Find_start_data_pos(oldest_run);	// Note: another way is to filter out older runs in main loop
		start_data_pos = LogCtl.tail_page * FLASH_PAGE_SIZE;
	}

	// Note: We could return on an error, but then would have no chance of rebuilding.

	if(start_data_pos <0) start_data_pos = 0;

	ee_getc_position(start_data_pos);

	PRINTF("start_data_pos:%d\r\n",AC210_log.getc_pos2);

	// Use Log_index to rebuild index, but will be using for lookup in verify

	run_log_index.log_start_pos = start_data_pos;
	int run_number = -1;

	int last_diags_pos = -1;
	ee_getc_cnt = 0;
	Log_drec_cnt = 0;
	int diag_rec_cnt = 0;
	int ecnt=0;
//	gVerbose = 20;
	int eof_cnt = 0;
	for(;;)
	{
		ecnt = 0;
		AC210_log.getc_pos2_save = AC210_log.getc_pos2;
		PRINTF_FLUSH;		// Wait for buffer to be output

		uint8_t b = ee_getc();
#ifdef MH_YYY
		if(b == 0)
		{
			ecnt++;
			PRINTF("b=0\r\n");
			continue;
		}
#endif
		if(b == FLASH_EOF)
		{
			if(eof_cnt++ < 32) continue;
			break;			// Could double check by seeing if next few in sequence are same
		}
		else
		{
			eof_cnt = 0;
		}
		if(b == EELOG_C_ESCAPE1)
		{
			uint8_t c = ee_getc();
			if(c != EELOG_C_ESCAPE2)	// MHH:03/01/2026. 254 is a valid value, no need to flag as an error.
			{
//				ecnt++;
//				PRINTF("Expecting ESCAPE2\r\n");
				continue;
			}
			c = ee_getc();
//			int log_start_pos=0;
			switch (c)
			{
			case EELOG_C_CHECKPOINT_START:
//				log_start_pos = AC210_log.getc_pos2 - 3;	// MHH:12/12/2023
				if(ee_get_checkpoint_data())
				{
					ecnt++;
					break;
				}
				// get run number

				if(gVerbose > 5)
				{
					PRINTF("CHECKPOINT_START\r\n");
					PRINTF("run:          %d\r\n",Log_checkpoint.run);
					PRINTF("data_pos:     %d\r\n",AC210_log.getc_pos2_save);
				}

				// Can be multiple start checkpoints. How? We want first.

				if(Log_checkpoint.run > run_number)	// Log data has corrupt area where run_num goes from 49 back to 48 then up again
				{
//					run_checkpoint = Log_checkpoint;	// save to check index
					run_number = Log_checkpoint.run;
					Log_drec_cnt = 0;
//					run_log_index.log_start_pos = log_start_pos;		// MHH:12/12/2023
					run_log_index.log_start_pos = AC210_log.getc_pos2_save;
//					run_log_index.head_wrap_cnt = Log_checkpoint.head_wrap_cnt;
					PRINTF("CPS:RUN=%d\r\n",run_number);
					PRINTF_FLUSH;
				}
				if(gVerbose > 5)	PRINTF("Log_drec_cnt  : %d\r\n",Log_drec_cnt);

				continue;

			case EELOG_C_CHECKPOINT_TIMEOUT:
				if(ee_get_checkpoint_data())
				{
					ecnt++;
					break;
				}
				if(Log_checkpoint.run > run_number)
				{
					ecnt++;
					run_number = Log_checkpoint.run;		// In case we missed checkpoint start.
					Log_drec_cnt = 0;
					run_log_index.log_start_pos = AC210_log.getc_pos2_save;
					PRINTF("CPT:RUN=%d,secs=%d\r\n",run_number,Log_checkpoint.m8_data.ticks/50);
					PRINTF_FLUSH;
					ecnt++;

					// I think this was caused when I had 2 run_num=48 and I used eerepair to fix. Ignore for now
//					if(gVerbose > 5)
#ifdef MH_YYY
					{
						PRINTF("CHECKPOINT_TIMEOUT\r\n");
						PRINTF("run:          %d\r\n",Log_checkpoint.run);
						PRINTF("run_number    %d\r\n",run_number);
						PRINTF("data_pos:     %d\r\n",AC210_log.getc_pos2_save);
						PRINTF("*** run should = run_number\r\n");
					}
#endif
//					break;

//					DebugAbort("AC210_log_build_index:checkpoint  run != run_number");
//					return;
				}
				continue;		// Note: not a BREAK

			case EELOG_C_CHECKPOINT_DIAGS:
				// Consider reading index if verify, and comparing with actual data

				diag_rec_cnt++;
				if(gVerbose > 5)
				{
					PRINTF("\r\nFound Diags Record Checkpoint:\r\n");
					PRINTF("Log_drec_cnt:   %d\r\n",Log_drec_cnt);
					PRINTF("data_pos:       %d\r\n",AC210_log.getc_pos2_save);
				}
				c = ee_getc();
				if(c != EELOG_C_ESCAPE3)
				{
					ecnt++;
					PRINTF("Expecting ESCAPE3");
					break;
				}
				// Diags Rec should follow
#ifdef MH_XXX
				if(Log_peek4() == PARAM_CHECK_CODE)	// MHH:18/04/2026. Param rec?
				{
					AC210_log.getc_pos2 += 256;	// Yes
				}
#endif
				uint32_t diags_pos = AC210_log.getc_pos2;
				run_log_index.log_diag_pos = diags_pos;
				if(Log_read_diag_rec(&d_r))
				{
					ecnt++;
					PRINTF("Invalid Diag_rec\r\n")
					break;
				}
			// Note: we could adapt this routine to fix positions if not correct

//				if(gVerbose > 5)
				{
					PRINTF("run_number    :%10d\r\n",d_r.run_number);
					PRINTF("Log_drec_cnt  :%10d\r\n",Log_drec_cnt);
					PRINTF("start_data_pos:%10d\r\n",d_r.log_start_data_pos);
					PRINTF("diags_pos     :%10d\r\n",diags_pos);
					PRINTF("last_diags_pos:%10d\r\n",d_r.log_last_diags_pos);
					PRINTF_FLUSH;
				}
//				int ecnt=0;
				if(d_r.run_number != run_number)
				{
					ecnt++;
					PRINTF("d_r.run_number != run_number\r\n");
				}
#ifdef MH_YYY
				if(run_log_index.log_start_pos != d_r.log_start_data_pos)
				{
					ecnt++;
					PRINTF("run_log_index.log_start_pos != d_r.log_start_data_pos\r\n");
				}
				if(start_data_pos == 0  || diag_rec_cnt > 1)		// first diag_rec ptr to prev will not be valid if wrapped
				{
					if(last_diags_pos != d_r.log_last_diags_pos)
					{
						ecnt++;
						PRINTF("last_diags_pos != d_r.log_last_diags_pos\r\n");
					}
				}
				// Note: If there is a difference, probably should use values in this scan??
#endif

				PRINTF_FLUSH;

				last_diags_pos = diags_pos;
				if(ecnt)
				{
					continue;	// data seems in sync, just values don't match
				}

				// This run looks fine, now to check it against the index

				Index_get_index(run_number);
				if(pIndex->run != run_number)
				{
					ecnt++;
					PRINTF("**Index run (%d) not equal run (%d)\r\n",pIndex->run,run_number);
					continue;
				}
				if(pIndex->log_start_pos != run_log_index.log_start_pos)
				{
					ecnt++;
					PRINTF("**Index start_pos  (%d) not equal run start_pos (%d)\r\n",pIndex->log_start_pos,run_log_index.log_start_pos);
					continue;
				}
#ifdef MH_XXX		// MHH:11/12/2023
				if(pIndex->head_wrap_cnt != run_log_index.head_wrap_cnt)
				{
					ecnt++;
					PRINTF("**Index wrap_cnt(%d) not equal run wrap_cnt (%d)\r\n",pIndex->head_wrap_cnt,run_log_index.head_wrap_cnt);
					continue;
				}
#endif
				if(pIndex->log_diag_pos != run_log_index.log_diag_pos)
				{
					ecnt++;
					PRINTF("**Index diag_pos  (%d) not equal run diag_pos (%d)\r\n",pIndex->log_diag_pos,run_log_index.log_diag_pos);
				}
				continue;		// Note: not a BREAK


			default:
				PRINTF("Expecting Checkpoint");
				ecnt++;
				break;
			}
		}
	}
#ifdef MH_YYY
// Looks like data if we get to here. Keep getting data until we get an offset with hi-bit set

		if(ecnt)
		{
			if(Log_scan_next_run(run_number) == SCAN_EOF)
			{
				break;
			}
			ecnt = 0;
		}
		else
		{

			if(Log_get_next_data(b)== 0)
			{
				Log_drec_cnt++;
			}
			else
			{
				if(Log_scan_next_run(run_number) == SCAN_EOF)
				{
					break;
				}
				// Could switch depending on return value if we are rebuilding index.
				// Possibly making zero or -1 the index ptrs if either data or drec invalid
			}
		}
	}
#endif
//	int ecnt = 0;
	if(run_number != Stat_Rec.run_number)
	{
		ecnt++;
		PRINTF("run_number != Stat_Rec.run_number\r\n");
	}
	if(last_diags_pos != Stat_Rec.log_last_diags_pos)
	{
		ecnt++;
		PRINTF("last_diags_pos != Stat_Rec.log_last_diags_pos\r\n");
		// Probably should update Stat_Rec here
	}
	if(run_log_index.log_start_pos != Stat_Rec.log_start_data_pos)
	{
		ecnt++;
		PRINTF("run_log_index.log_start_pos != Stat_Rec.log_start_data_pos\r\n");
	}
	if(ecnt)
	{
		PRINTF(":A:Stat_Rec and index needs rebuild\r\n");
	}
	else
	{
		PRINTF("Stat_Rec OK\r\n");
	}
//	printf("\r\nFinished\r\n");
	PRINTF(":A:EOF\r\n");
}
//--------------------------------------------------------------------------------------------------------------
// Note: we could have 3 options:
// 1) Scan data and verify against index.
// 2) Scan index, and verify against data
// 3) Combination of both


#ifdef MH_XXX
#define INDEX_VERIFY		1
#define DATA_VERIFY			2

void AC210_logix_verify(WORD value)
{
	if(value & INDEX_VERIFY)
	{
		Index_verify();
	}
	if(value & DATA_VERIFY)
	{
		Data_verify();
	}
}
#endif
//--------------------------------------------------------------------------------------------------------------
void AC210_logix_show(void)
{
	int drec_pos;
	int start_data_pos;
//	int head_wrap_cnt;
	int run;
	AC210_watchdog_active = false;				// This could take a while

	PRINTF("\r\nATX_show_index:\r\n");
//	PC_flush_output();
	//               1         2         3         4         5
	//      123456789012345678901234567890123456789012345678901234567890
	PRINTF("       Rix       Run    Dstart     DRpos  Filesize\r\n")
	Index_page = -1;	// In case of restore or reset

#ifdef MH_XXX
	uint32_t tail_pos = LogCtl.last_flash_sector_erased * FLASH_SECTOR_SIZE;	// Check if data has been erased
	uint32_t tail_address = (LogCtl.tail_wrap_cnt << 24) + tail_pos;
#endif
//	uint32_t tail_pos = LogCtl.tail_page * FLASH_PAGE_SIZE;	// Check if data has been erased
//	uint32_t tail_address = (LogCtl.tail_wrap_cnt << 24) + tail_pos;


//	bool tail_displayed = false;

	int rix_max = Stat_Rec.run_number;
	if(Stat_Rec.run_number > LOG_INDEX_MAX) rix_max = LOG_INDEX_MAX;

	// So, if LOG_INDEX_MAX = 100, then rix would go from 0 to 99
	for(int rix=0;rix<rix_max;rix++)
	{
		Index_get_index(rix+1);
		drec_pos = pIndex->log_diag_pos;
// Using index data_pos as against drec data_pos allows repair of data as can modify index but not drec.
		start_data_pos = pIndex->log_start_pos;
#ifdef MH_XXX
//		head_wrap_cnt  = pIndex->head_wrap_cnt;
		uint32_t start_data_address = (head_wrap_cnt << 24) + start_data_pos;
		if(tail_displayed == false)
		{
			if(start_data_address >= tail_address)
			{
				PRINTF(">>tail_address:%d:%d\r\n",LogCtl.tail_wrap_cnt,tail_pos);
				tail_displayed = true;
			}
		}
#endif
		run = pIndex->run;

		int filesize = 0;
		if(drec_pos != -1)		// Dummy entry for currect stat_rec
		{
			filesize = drec_pos - start_data_pos;
			if(filesize < 0) filesize += EELOG_SPACE;
		}

		PRINTF("  %8d  %8d  %8d  %8d  %8d\r\n",rix,run,start_data_pos,drec_pos,filesize);
//		PRINTF("rix:%5d, drec_pos:%8d, start_data_pos:%d, head_wrap_cnt:%d, run:%d\r\n",rix,drec_pos,start_data_pos,head_wrap_cnt,run);
//		if(drec_pos == -1) break;
		if(run != LOG_INDEX_ERASED_RUN)
		{
			if(rix != (run-1) % LOG_INDEX_MAX)
			{
				DebugAbort("rix != (run-1) % LOG_INDEX_MAX");
				return;
			}
		}
#ifdef MH_XXX	// MHH:02/09/2026
		if((rix+1)%40 == 0)
		{
			if(PC_ReturnToContinue() == 'X')
			{
				break;
			}
		}
#endif
#ifdef MH_READ_DIAG
		if(eelog_read_diag_rec(&d_r,drec_pos))
		{
		    txDebug(":A:ERR:Invalid Check code(2)\r\n");
			return;		// error
		}
#endif
	}
	PRINTF("Finished\r\n");
	AC210_watchdog_active = true;
}
//--------------------------------------------------------------------------------------------------------------
#ifdef MH_XXX
int Log_find_start_data_pos(void)
{
// Consider a LogCtl value that we update for oldest valid run.

// Note: We are currently scanning all used entries.
// The purpose of the oldest logic is in case of run number wrap (eg run > 1000)

	if(LogCtl.head_wrap_cnt == 0)
	{
		return 0;		// No wrap yet
	}
	PRINTF("Log_find_start_data_pos:\r\n");

	int drec_pos;
	uint8_t head_wrap_cnt;
	int start_data_pos;
	int run;
	int tail_pos = LogCtl.last_flash_sector_erased * FLASH_SECTOR_SIZE;

	int oldest_valid_run = 65535;
	int oldest_valid_start_pos = 0;

	Index_page = -1;	// In case of restore or reset
	for(int rix=1;rix<=LOG_INDEX_MAX;rix++)
	{
		Index_get_index(rix);

		drec_pos = pIndex->log_diag_pos;
// Using index data_pos as against drec data_pos allows repair of data as can modify index but not drec.
		start_data_pos = pIndex->log_start_pos;
		head_wrap_cnt       = pIndex->head_wrap_cnt;
		run            = pIndex->run;
		if(gVerbose > 10)
		{
			PRINTF("  %8d  %8d  %8d  %8d  %8d\r\n",rix,run,start_data_pos,head_wrap_cnt,drec_pos);
		}
		if(run == 65535) break;
		if(rix != run % LOG_INDEX_MAX)
		{
			DebugAbort("rix != run % 1000");
			return -1;
		}
		if(head_wrap_cnt == LogCtl.head_wrap_cnt || start_data_pos >= tail_pos)
		{
			if(run < oldest_valid_run)
			{
				oldest_valid_run = run;
				oldest_valid_start_pos = pIndex->log_start_pos;
			}
		}
	}
	if(oldest_valid_run == 65535)
	{
		DebugAbort("At end of index table, could not find valid start_data_pos\r\n");
		return -1;
	}
	PRINTF("oldest_valid_run:%d, oldest_valid_start_pos:%d\r\n",oldest_valid_run,oldest_valid_start_pos);
// Note: Could double check by reading to see if we have a start checkpoint
	return oldest_valid_start_pos;
}
#endif
