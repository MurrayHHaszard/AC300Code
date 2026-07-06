/*
 * ac210log.c
 *
 *  Created on: 12/12/2016
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

//#define EELOG_FLAG_WRAPPED		1

AC210_log_T AC210_log;

static int Log_index_filesize;

AC210_range_T AC210_range;
//-------------------------------------------------------------------------
int ee_read_log(uint8_t *buff,int len,int pos)
{
	return AC210_flash_read_part(pos,buff,len);
}
//-------------------------------------------------------------------------
/*
#define LOGCTL_MAGIC			12345
#define EE_LOGCTL_POS				512
#define LOGCTL_HEAD_OFFSET		4
#define LOGCTL_HEAD_SIZE		4
#define LOGCTL_REC_SIZE		(sizeof(td_LOGCTL))
typedef struct
{
	int magic;
	int head_page;
	int tail_page;
	int last_flash_sector_erased;
	uint8_t head_wrap_cnt;	// Incremented every time head wraps
	uint8_t fill[3];
} td_LOGCTL;
*/
td_LOGCTL LogCtl;
//----------------------------------------------------------------------------------------------------
int ee_check_logctl(char *from)
{
	if(LogCtl.max_pages != EELOG_MAX_PAGES)
	{
		L2PRINTF("%s:Invalid value:LogCtl.max_pages=%d\r\n",from,LogCtl.max_pages);
		return -1;
	}

	if(LogCtl.last_flash_sector_erased < 1 || LogCtl.last_flash_sector_erased >= FLASH_MAX_SECTORS)
	{
		L2PRINTF("%s:Invalid value:LogCtl.last_flash_sector_erased=%d\r\n",from,LogCtl.last_flash_sector_erased);
		return -1;
	}

	uint32_t head_page = LogCtl.head_page;
	uint32_t tail_page = LogCtl.tail_page;

	if(head_page > FLASH_PART_PAGES)
	{
		L2PRINTF("%s:Invalid value:head_page=%d  > MAX_PAGES\r\n",from,head_page);
		return -1;
	}
	if(tail_page > FLASH_PART_PAGES)
	{
		L2PRINTF("%s:Invalid value:tail_page=%d  > MAX_PAGES\r\n",from,tail_page);
		return -1;
	}
#ifdef MH_456	// MHH:10/12/2023. Now ignoring head and tail wrap count.
	int page_diff = head_page - tail_page;
//	if(page_diff < 2)	// MHH:08/12/2023. page_diff may be zero
	if(page_diff < 0)	// MHH:08/12/2023. page_diff may be zero
	{
		L2PRINTF("%s:Invalid value:page_diff < 2, page_diff=%d\r\n",from,page_diff);
		return -1;

	}
	if(page_diff >= EELOG_MAX_PAGES)	// MHH:08/12/2023
	{
		L2PRINTF("%s:Invalid value:page_diff > MAX_PAGES, page_diff=%d\r\n",from,page_diff);
		return -1;

	}
#endif
	return 0;

}
int ee_write_logctl()
{
	if(LogCtl.magic != LOGCTL_MAGIC)
	{
		L2PRINTF("ee_write_logctl:Bad magic:%d\r\n",LogCtl.magic);
		return -1;
	}
#ifndef MH_XXX

	if(ee_check_logctl("ee_write_logctl"))
	{
		return -1;
	}

#else
	LogCtl.magic = 0;
	LogCtl.head_page = 0;	// MHH:30/12/2025. Test only!!!
	LogCtl.tail_page = 0;
	LogCtl.max_pages = 0;
#endif

	uint8_t* p_buf = (uint8_t *)&LogCtl;

	p_eeprom_i2c_pos(EE_LOGCTL_POS);
	if(p_eeprom_i2c_write(p_buf, LOGCTL_REC_SIZE)) {
		Disable_log_data(200);
//		LogData.enabled = false;	// Turn logging off
		DebugAbort("ee_write_logctl:eeprom_i2c_write");
		return -1;
	}
	return 0;
}
//----------------------------------------------------------------------------------------------------
/*
 *  MHH:11/12/2023. On initialisation, if LogCtl is not valid then try a quick repair, based on information in the Diags rec.
 */

// Only here if a head_page value of zero. Value of zero OK if last_flash_sector erased == 1.
#ifdef MH_XXX
int ee_find_head_page_from_last_flash_sector_erased2(int last_flash_sector_erased)
{
/*
 *  If last_flash_sector_erased = 1 then we can either scan from the start of the sector, incrementing by page_bytes (1024) and then
 *  the first page we find that is erased (all 255) is the page after the head_page. The alternative is to read from the end of the
 *  sector backwards, and the first non erased page is the head_page.
 *
 *  So, if last_sector_erased
 *
 *
 */


	int last_page_erased = last_flash_sector_erased * FLASH_LOG_PAGES_PER_SECTOR - 1;	// Should be 64 pages per sector
//	int last_page_erased = (last_flash_sector_erased - FLASH_SECTOR_OFFSET) * FLASH_LOG_PAGES_PER_SECTOR;	// MHH:11/12/2023

	int pos = last_page_erased * EELOG_PAGESIZE;
	int check_page = last_page_erased;
	uint8_t *p_dst = ee_serial_read_buff;
	int rlen = 8;		// Read 8 bytes
	for(int i=0;i<128;i++)
	{
		AC210_ssp_flash_read(pos+EELOG_START_POS,p_dst,rlen);
		if(ee_check_all_255(rlen) == false)	// Found a data page?
		{
			if(check_page != LogCtl.head_page)
			{
				LogCtl.head_page = check_page;
				ee_update_head_page();
			}
			return 0;
		}
		check_page--;
		pos -= EELOG_PAGESIZE;
		if(pos < 0)
		{
			return -1;
		}

	}
	return -1;
}
#endif
#ifdef MH_XXX		// MHH:31/12/2025. Unused
int ee_logctl_get_eof_pos(void)
{
	int read_page = LogCtl.head_page;
//	DPRINTF("LogCtl.head_page = %d\r\n",LogCtl.head_page);
	int read_pos = read_page * FLASH_PAGE_SIZE;		// Align on page boundary.
//	read_pos = erased_sector_end_pos - FLASH_PAGE_SIZE;	// We know the last page in the sector is erased...
	int ix = 0;
	int start_erased_bytes = -1;
	int data_page_count = 0;
	bool found_erased_page = false;
	int data_page_pos = 0;
	int erased_page_pos = 0;
	for(int i=0;i<20;i++)
	{
		read_pos %= FLASH_PART_SIZE;
		erased_page_pos = read_pos;

		DPRINTF("ee_logctl_get_eof_pos:scan for erased page,read_pos:%d\r\n",read_pos);

		AC210_flash_read_part(read_pos,UU_fBuff,FLASH_PAGE_SIZE);
		if(ee_check_all_255(UU_fBuff,FLASH_PAGE_SIZE))	// Found an erased page?
		{
			found_erased_page = true;
			break;		// Yes, now to look for eof position.
		}
		data_page_count++;
		data_page_pos = read_pos;
		read_pos += FLASH_PAGE_SIZE;
	}
	if(found_erased_page == false)
	{
		L2PRINTF("ee_logctl_get_eof_pos:Could not find an erased_page, LogCtl.head_page = %d\r\n",LogCtl.head_page);
		Disable_log_data(640);
		return -1;			// Consider setting flag to stop log file
	}
	if(data_page_count == 0)
	{
		L2PRINTF("ee_logctl_get_eof_pos:Could not find a data page,LogCtl.head_page = %d\r\n",LogCtl.head_page);
		Disable_log_data(650);
		return -1;
	}
	if(data_page_count > 2)
	{
		L2PRINTF("ee_logctl_get_eof_pos:%%data_page_count > 2 = %d, LogCtl.head_page = %d\r\n",data_page_count,LogCtl.head_page);
	}
	read_pos = data_page_pos;		// Re load previous data page
	AC210_flash_read_part(read_pos,UU_fBuff,FLASH_PAGE_SIZE);
	int last_ix = -1;		// We have a data page, now see if it is partly or completely erased

	for(ix=FLASH_PAGE_SIZE - 1;ix>=0;ix--)
	{
		uint8_t b = UU_fBuff[ix];
		if(b != 255) break;
		last_ix = ix;
	}
	if(last_ix == -1)	// Is data page partly erased?
	{
		start_erased_bytes = erased_page_pos;	// No, then start_erased_bytes is start erased page,
	}
	else
	{
		start_erased_bytes = data_page_pos + last_ix;
	}
	// Note: We could align on (say) a 16 byte (or page) boundary,but this would not be compatible with existing logic
	return start_erased_bytes;
}
#endif
#ifdef MH_YYY
int ee_get_tail_page(void)
{
	/*
	 * MHH:23/12/2023. If the first 32 bytes of the last page of the partition is erased, and the first page is not then tail_page is zero.
	 * Otherwise, search from head page.
	 *
	 */

	int tail_page = -1;
	int read_len = 32;
	int read_pos = FLASH_PART_SIZE - FLASH_PAGE_SIZE;
	AC210_flash_read_part(read_pos,UU_fBuff,read_len);

	if(ee_check_all_255(UU_fBuff,read_len))	// Is the last page in partition erased?
	{	// Yes, then test first 32 bytes of file
		read_pos = 0;
		int read_len = 32;
		AC210_flash_read_part(read_pos,UU_fBuff,read_len);
		if(ee_check_all_255(UU_fBuff,read_len) == false)
		{
			tail_page = 0;	// This should be case unless we wrap
			return tail_page;
		}
	}

	/*
	 *  OK, if we have wrapped then need to scan from head page, passed any erased pages up to the tail page.
	 *
	 *  We may be able to check against existing LogCtl tail value.
	 *
	 */


	/*
	 * 	We know that the tail page MUST be the the first page of a sector, because we erase sector by sector. So it will move from zero to 64 to 128 etc.
	 *
	 * 	We also know that if the partition is not corrupt then the data file must have wrapped, as we have checked the tail_page = 0 option.
	 *
	 * 	We know that the tail page can not be in the same sector as the head_page (since we have wrapped).
	 *
	 * 	So, the tail_page can only be in 1 of 2 places:
	 * 	1) First page of the next sector from the head_page.
	 * 	2) First page in the next sector + 1 from the head page.
	 *
	 * 	Note: We also know that the last_sector erased has to be the previous sector from where the tail page found. (unless not wrapped).
	 *
	 *
	 *
	 */


	int head_file_sector = AC210_log.eof_page / FLASH_LOG_PAGES_PER_SECTOR;
	read_pos = (head_file_sector + 1) * FLASH_SECTOR_SIZE;
	read_len = 32;
//	read_pos = ((head_file_sector + 1) * FLASH_SECTOR_SIZE) - read_len;
	int tail_file_sector = -1;
	// Scan for first non-erased sector
	for(int i=1;i<=2;i++)
	{
		read_pos %= FLASH_PART_SIZE;
		AC210_flash_read_part(read_pos,UU_fBuff,read_len);	// Read first 32 bytes of sector
		if(ee_check_all_255(UU_fBuff,read_len) == false)	// Are they erased?
		{
			tail_file_sector = head_file_sector + i;
			break;
		}
		read_pos += FLASH_SECTOR_SIZE;
	}
	if(tail_file_sector == -1)
	{
		L2PRINTF("ee_get_tail_page:Cannot find tail_file_sector\r\n");
		Disable_log_data(670);
		return -1;
	}
	tail_page = tail_file_sector * FLASH_LOG_PAGES_PER_SECTOR;
	tail_page %= FLASH_PART_PAGES;
	return tail_page;
}
#endif
int ee_check_logctl_ini()
{
#ifdef MH_YYY
	bool l2print_details = false;

	if(LogCtl.last_flash_sector_erased < 1)
	{
		L2PRINTF("ee_check_logctl_ini:last_flash_sector_erased < 1 :%d\r\n",LogCtl.last_flash_sector_erased);
		l2print_details = true;
		// Hopefully can recover value
//		Disable_log_data(600);
//		return -1;
	}
#endif
/*
 *  MHH:24/12/2024. It is possible that the file address = start_data_pos + filesize is less than the actual file size, if the Stat_Rec was not updated,
 *  since it is only updated every minute. However, it should not be greater than the actual pointer, nor greater than the head_page.
 */

	int statrec_eof_pos = Stat_Rec.log_start_data_pos + Stat_Rec.log_filesize;
	statrec_eof_pos %= FLASH_PART_SIZE;
	int statrec_eof_page = statrec_eof_pos / FLASH_PAGE_SIZE;

	int page_diff = LogCtl.head_page - statrec_eof_page;
	if(page_diff < 0)
	{
//		page_diff += FLASH_PART_SIZE;
		page_diff += FLASH_PART_PAGES;		// MHH:30/12/2025
	}

	/*
	 * MHH:24/12/2023. At present we are checking to see if the LogCtl.head_page and the stat_rec head page are similar values (LogCtl.head_page allowed to
	 * be bigger). If the LogCtl.head page was (accidentally) set to zero then and the stat_rec head page looked valid, then perhaps could use stat_rec head page instead
	 * of LogCtl.head_page? Currently flagging error and disabling logging...
	 */
	if(page_diff > 100)		// Arbitrary, but surely no more than 100 pages per minute...
	{
		L2PRINTF("ee_check_logctl_ini:statrec_eof_page = %d, LogCtl.head_page = %d\r\n",statrec_eof_page,LogCtl.head_page);
		Disable_log_data(610);
		return -1;
	}

#ifdef MH_YYY
	if(LogCtl.head_page == 0)		// In case set to zero accidentally
	{
		if(statrec_eof_page < (FLASH_PART_PAGES - 20))		// And we have not wrapped
		{
			L2PRINTF("ee_check_logctl_ini:LogCtl.head_page = 0, setting to:%d\r\n",statrec_eof_page);
			LogCtl.head_page = statrec_eof_page;
			l2print_details = true;
		}
	}
#endif
	DPRINTF("ee_check_logctl_ini:statrec_eof_page = %d, LogCtl.head_page = %d\r\n",statrec_eof_page,LogCtl.head_page);

	int start_pos = LogCtl.head_page * FLASH_PAGE_SIZE;
	if(start_pos > 5) start_pos-=5;		// We want at least 5 data bytes
//	if(start_pos > 10000) start_pos-=10000;		// We want at least 5 data bytes

	ee_getc_position(start_pos);

	uint8_t b;
	int data_count = 0;
	int cnt_255=0;
	int start_erased_pos=-1;
	bool found_start_erased = false;
	for(int i=0;i<20000;i++)
	{
		b = ee_getc();
		if(b != 255)
		{
			data_count++;
			cnt_255 = 0;
		}
		else
		{
			if(cnt_255++ == 0)
			{
				start_erased_pos = AC210_log.getc_pos2 - 1;
			}
			if(cnt_255 > FLASH_PAGE_SIZE)
			{
				if(data_count != 0)	found_start_erased = true;
//				found_start_erased = true;	// MHH:17/07/2025. Could not find any data bytes, but
				break;
			}
		}
	}
	if(found_start_erased == false)
	{
		L2PRINTF("ee_check_logctl_ini:Could not find start erased, LogCtl.head_page = %d\r\n",LogCtl.head_page);
		Disable_log_data(610);
		return -1;

	}
	int log_ctl_eof_pos = start_erased_pos;

#ifdef MH_XXX
	int log_ctl_eof_pos = ee_logctl_get_eof_pos();
	if(log_ctl_eof_pos < 0)
	{
		return -1;
	}
#endif
	AC210_log.putc_pos = log_ctl_eof_pos;
	AC210_log.putc_pos2 = AC210_log.putc_pos;
	AC210_log.eof_page  = AC210_log.putc_pos / FLASH_PAGE_SIZE;

	int log_filesize = AC210_current_log_filesize(1);
	if(log_filesize < 0)
	{
		return -1;
	}
	statrec_eof_pos = Stat_Rec.log_start_data_pos + log_filesize;
	statrec_eof_pos %= FLASH_PART_SIZE;

	if(AC210_log.putc_pos !=  statrec_eof_pos)
	{
		L2PRINTF("ee_check_logctl_ini:AC210_log.putc_pos (%d) !=  statrec_eof_pos (%d)\r\n",AC210_log.putc_pos,statrec_eof_pos);
		Disable_log_data(665);
		return -1;
	}
	L2PRINTF("ee_check_logctl_ini:AC210_log.putc_pos = %d\r\n",AC210_log.putc_pos);

#ifdef MH_YYY
	int tail_page = ee_get_tail_page();
	if(tail_page < 0)
	{
		return - 1;
	}
#endif
/*
 * OK, we know head_page and tail_page. Knowing the head_page means that we should be able to work out last_sector_erased, as it must be either
 * the head_page sector or the next one. If the next one starts with a data page, then it must be the head_page sector;
 */
#ifdef MH_YYY
	int head_page_sector = AC210_log.eof_page / FLASH_LOG_PAGES_PER_SECTOR;
	int last_part_sector_erased = head_page_sector;
	int read_pos = (head_page_sector + 1) * FLASH_SECTOR_SIZE;

	read_pos %= FLASH_PART_SIZE;
	int read_len = 32;
	AC210_flash_read_part(read_pos,UU_fBuff,read_len);	// Read first 32 bytes of sector
	if(ee_check_all_255(UU_fBuff,read_len))	// Are they erased?
	{
		last_part_sector_erased = head_page_sector + 1;
	}
	last_part_sector_erased %= FLASH_PART_SECTORS;
	int last_raw_sector_erased = last_part_sector_erased + FLASH_SECTOR_OFFSET;

	if(l2print_details)
	{
		L2PRINTF("ee_check_logctl_ini:\r\n")
		L2PRINTF("                 LogCtl  New vals\r\n");
						  //       dddddd    dddddd
		L2PRINTF("head_page...........%6d    %6d\r\n",LogCtl.head_page,AC210_log.eof_page);
		L2PRINTF("tail_page...........%6d    %6d\r\n",LogCtl.tail_page,tail_page);
		L2PRINTF("last_sector_erased..%6d    %6d\r\n",LogCtl.last_flash_sector_erased,last_raw_sector_erased);
	}
	else
	{
		DPRINTF("ee_check_logctl_ini:\r\n")
		DPRINTF("                 LogCtl  New vals\r\n");
						  //       dddddd    dddddd
		DPRINTF("head_page...........%6d    %6d\r\n",LogCtl.head_page,AC210_log.eof_page);
		DPRINTF("tail_page...........%6d    %6d\r\n",LogCtl.tail_page,tail_page);
		DPRINTF("last_sector_erased..%6d    %6d\r\n",LogCtl.last_flash_sector_erased,last_raw_sector_erased);
	}

//	LogCtl.head_page = head_page;


	LogCtl.head_page = AC210_log.eof_page;
	LogCtl.tail_page = tail_page;
	LogCtl.last_flash_sector_erased = last_raw_sector_erased;
	LogCtl.max_pages = EELOG_MAX_PAGES;		// MHH:30/12/2025


#endif
	LogCtl.run_number = Stat_Rec.run_number;

	if(ee_write_logctl() != 0)
	{
		Disable_log_data(680);
		return -1;
	}

/*
 * typedef struct
{
	WORD check_code;			// (0)
	WORD log_flags;				// (2) Not used at this stage
	WORD serial;				// (4) Not used
//	WORD pcb_version;			// (4) MHH:09/01/2019
	WORD run_number;			// (6)
	WORD watchdog_restarts;		// (8)
	WORD abort_restarts;		// (10)
	ULONG start_date;			// (12) date stats started YYYYMMDD
	ULONG run_secs;				// (16)
	long log_last_diags_pos;	// (20) -1 if first in chain.
	long log_start_data_pos;	// (24) -1 if first in chain
	td_Tot_TimeStats tot;		// (28) 80 bytes
	td_short_TimeStats run;		// (108)32 bytes
	long log_filesize;			// (140)
	BYTE head_wrap_cnt;			// (144)
	BYTE pc_prog;				// (145) 1 = User, 2 = Diagnostics, 3 = Programmer. Set by ATCODE=n
//	WORD ac_version;			// (146) ac_version already handled in Datahead_rec, so could reuse this field
	WORD fill_1;
	td_run_errs run_errs;		// (148)  8 bytes
	ULONG diags_save_date;		// (156) Set by Diagnostics when saving, not used by AC210/AC200
	ULONG param_update_date;	// (160)
	ULONG pc_user_date;			// (164)
	ULONG rtc_start_secs;		// (168) MHH:26/12/2018.
//	WORD fill2[2];				// (168) 4 bytes

	td_Error err_tbl[3];		// (172) 72 bytes
	ULONG check2;				// (244) 4 bytes. May use as additional verification.
								// (248)
} td_Stats;
 */
// First thing to do is work out where the last erase sector should be....


	return 0;
}
//----------------------------------------------------------------------------------------------------
int ee_read_logctl()
{
	uint8_t* p_buf = (uint8_t *)&LogCtl;
	p_eeprom_i2c_pos(EE_LOGCTL_POS);
	if(p_eeprom_i2c_read(p_buf, LOGCTL_REC_SIZE)) {
		Disable_log_data(300);
//		LogData.enabled = false;	// Turn logging off
		DebugAbort("ee_read_logctl:eeprom_i2c_read");
		return -1;
	}
	return 0;
}
//----------------------------------------------------------------------------------------------------
#ifdef MH_EXTENDED
void ee_check_logctl_extended(void)	// MHH:08/12/2023.
{
	if(LogCtl.extended == 'Y') return;

	// Need to fix a few things....

	LogCtl.extended = 'Y';

	LogCtl.erase_wrap_cnt = 0;
	if(LogCtl.tail_wrap_cnt != 0) LogCtl.erase_wrap_cnt = 1;

	LogCtl.head_wrap_cnt = 0;
	LogCtl.tail_wrap_cnt = 0;

	if(LogCtl.head_page < LogCtl.tail_page)
	{
		LogCtl.head_wrap_cnt = 1;
	}
	ee_write_logctl();	// Update.
}
#endif
int Verify_logctl_data(bool fix_logctl);
int ee_get_logctl(void)
{
	if(ee_read_logctl() < 0)
	{
		return -1;
	}
	int statrec_eof_pos = Stat_Rec.log_start_data_pos + Stat_Rec.log_filesize;
	statrec_eof_pos %= FLASH_PART_SIZE;
	int statrec_eof_page = statrec_eof_pos / FLASH_PAGE_SIZE;
	AC210_log.eof_page  = statrec_eof_page;	// MHH:03/01/2026. We will check this later

	if(LogCtl.magic != LOGCTL_MAGIC)
	{
		L2PRINTF("ee_get_logctl:No magic\r\n");
		if(Stat_Rec.run_number == 1 && Stat_Rec.start_date == 0)	// First time?
		{
			eeprom_log_reset(997);
			return 0;
		}
		else
		{
			if(Verify_logctl_data(true) != 0)
			{
				Disable_log_data(700);			// MHH:20/12/2023
				return -1;
			}
		}
	}
	else
	{
		if(Verify_logctl_data(true) != 0)
		{
			Disable_log_data(710);			// MHH:20/12/2023
			return -1;
		}
	}
	return ee_check_logctl_ini();	// MHH:11/12/2023
}
//-------------------------------------------------------------------------
uint8_t ee_serial_read_buff[EE_SER_BUFF_SIZE];
uint8_t ee_serial_read_ix=EE_SER_BUFF_SIZE;
//-------------------------------------------------------------------------------------------
void ee_getc_read_buffer(void)
{
	int pos = AC210_log.getc_pos;		// end of last buffer read
	uint8_t *p_dst = ee_serial_read_buff;
	int bytes_left = EE_SER_BUFF_SIZE;
	for(;;)
	{
		if(pos >= EELOG_SPACE) pos = 0;		// wrap around
		int space = EELOG_SPACE - pos;
		int rlen = MIN(bytes_left,space);
		AC210_flash_read_part(pos,p_dst,rlen);
		pos += rlen;
		bytes_left -= rlen;
		if(bytes_left == 0) break;
		p_dst += rlen;
	}

	ee_serial_read_ix=0;
	AC210_log.getc_pos = pos;
}
//-------------------------------------------------------------------------------------------
int ee_getc_cnt;
uint8_t ee_getc(void)
{
	if(ee_serial_read_ix >= EE_SER_BUFF_SIZE)
	{
		ee_getc_read_buffer();
	}
	uint8_t rv = ee_serial_read_buff[ee_serial_read_ix++];
	if(++AC210_log.getc_pos2 >= EELOG_SPACE)
	{
		AC210_log.getc_pos2 -= EELOG_SPACE;
	}
	ee_getc_cnt++;
	return rv;
}
//--------------------------------------------------------------------------------------------------------------
void ee_getc_position(int pos)
{
	ee_serial_read_ix = EE_SER_BUFF_SIZE;	// Force buffer read
	AC210_log.getc_pos = pos;
	AC210_log.getc_pos2 = pos;			// I think this is correct at time of position??

}
//--------------------------------------------------------------------------------------------------------------
static int ee_getc_save_pos;
void ee_getc_save_position(void)
{
	ee_getc_save_pos = AC210_log.getc_pos2;
}
//--------------------------------------------------------------------------------------------------------------
void ee_getc_restore_position()
{
	ee_getc_position(ee_getc_save_pos);
}

//--------------------------------------------------------------------------------------------------------------
void ee_ignore_data(int len)
{
	for(int i=0;i<len;i++) ee_getc();
}
void ee_checkpoint_get_data(void)
{
	uint8_t *bp = (uint8_t *)&Log_checkpoint;
	for(int i=0;i<LOG_CHECKPOINT_SIZE;i++) *bp++ = ee_getc();
#ifdef MH_XXX
	if(Log_checkpoint.extended == 'X')
	{
		ee_ignore_data(256);
	}
#endif
}
//----------------------------------------------------------------------------------------------------
uint8_t Log_dup_getc(void);
static int Log_check_lticks;	// Use this to check ticks are in sequence
static int Log_count_no_lticks;	// This is used for verify
static M8_T M8Data_in;

int ee_get_checkpoint_data(void)
{
	ee_checkpoint_get_data();

//	gVerbose = 5;
	if(gVerbose >= 5)
	{
		PRINTF("CP:Run:%d,Secs=%d\r\n",Log_checkpoint.run,Log_checkpoint.m8_data.ticks/50);
	}
	uint8_t c = ee_getc();
	if(c != EELOG_C_ESCAPE3)
	{
		if(gVerbose > 5) PRINTF("ee_get_checkpoint_data:Expecting ESCAPE3");
//		DebugAbort("ee_get_checkpoint_data:Expecting ESCAPE3");
		return -1;
	}
	M8Data_in = Log_checkpoint.m8_data;
	Log_check_lticks = 0;
	Log_count_no_lticks = 0;
	return 0;
}
//----------------------------------------------------------------------------------------------------
int AC210_reset_logctl(void)
{
	memset((char *)&LogCtl,0,LOGCTL_REC_SIZE);
	LogCtl.magic = LOGCTL_MAGIC;
	LogCtl.max_pages = EELOG_MAX_PAGES;
	LogCtl.head_page = 0;	// These handled by memset above
	LogCtl.tail_page = 0;
	LogCtl.last_flash_sector_erased = 1;		// Was zero, but need to be consistent with head_wrap and tail_wrap
	return ee_write_logctl();
}
//----------------------------------------------------------------------------------------------------
int ee_update_head_page(void)
{
	uint8_t* p_buf = (uint8_t *)&LogCtl.head_page;
	p_eeprom_i2c_pos(EE_LOGCTL_POS + LOGCTL_HEAD_OFFSET);
	if(p_eeprom_i2c_write(p_buf, LOGCTL_HEAD_SIZE))
	{
		Disable_log_data(600);
//		LogData.enabled = false;	// Turn logging off
		DebugAbort("ee_update_head_page:eeprom_i2c_write");
		return -1;
	}
	return 0;
}
//----------------------------------------------------------------------------------------------------
int ee_update_logctl(void)
{
	LogCtl.magic = LOGCTL_MAGIC;
	LogCtl.run_number = Stat_Rec.run_number;
	return ee_write_logctl();
}
//----------------------------------------------------------------------------------------------------
void AC210_logctl_check_run()
{
    if(LogCtl.run_number != Stat_Rec.run_number)
    {
    	ee_update_logctl();
    }
}
//----------------------------------------------------------------------------------------------------
#ifdef MH_123
int ee_write_log(uint8_t *buff,int len,int pos)
{
	if(len > EE_SER_BUFF_SIZE)
	{
		DebugAbort("ee_write_log:len > EE_SER_BUFF_SIZE");
		return -1;
	}
	if(len <= 0)
	{
		DebugAbort("ee_write_log:len <= 0");
		return -1;
	}
	if(pos >= EELOG_SPACE)
	{
		DebugAbort("ee_write_log:pos >= EELOG_SPACE");
	}
	if(pos + len > EELOG_SPACE)
	{
		DebugAbort("ee_write_log:pos+len > EELOG_SPACE");
	}

	uint32_t log_page = pos /EELOG_PAGESIZE;		// 1024
	if(log_page != LogCtl.head_page)
	{
		DebugAbort("ee_write_log:log_page != head_page");
	}
	uint32_t flash_pos = pos + EELOG_START_POS;
	if(AC210_ssp_flash_write(flash_pos,buff,len) == -1)
	{
		DebugAbort("ee_write_log:AC210_ssp_flash_write");
	}
	uint32_t new_pos = pos + len;
	new_pos %= EELOG_SPACE;
	uint32_t new_log_page = new_pos / EELOG_PAGESIZE;
	if(new_log_page != LogCtl.head_page)
	{
		LogCtl.head_page = new_log_page;
		if(ee_update_head_page())
		{
			return -1;
		}
		int page_within_sector = LogCtl.head_page % FLASH_LOG_PAGES_PER_SECTOR;	// 64 log pages per flash sector
	//	page_within_sector = 63;	// !!! Test only
		if(page_within_sector == (FLASH_LOG_PAGES_PER_SECTOR -1))						// if page == 63
		{
			uint32_t current_sector = LogCtl.head_page /FLASH_LOG_PAGES_PER_SECTOR + FLASH_SECTOR_OFFSET;
			uint32_t next_sector = current_sector + 1;
			if(next_sector >= FLASH_MAX_SECTORS) next_sector = FLASH_SECTOR_OFFSET;	// Ignore sector zero, has very slow erase time
			if(next_sector != LogCtl.last_flash_sector_erased)
			{
				AC210_log.flash_erase_sector = next_sector;		// Will do this when time
			}
		}
	}
	return 0;
}
#endif
//-------------------------------------------------------------------------------------------
void Log_update_flash_pos(void)
{
	AC210_log.putc_pos2++;
	AC210_log.putc_pos2 %= EELOG_SPACE;
// Note: The erase sector routine updates logctl if successful, a head wrap and erase should not happen at same time
	if(AC210_log.flash_erase_sector != 0)
	{
		AC210_log_check_erase_sector();
	}
}
//-------------------------------------------------------------------------------------------
/*
 * void Flog_write_ring_data(int min_count)
{
	if(Flog_mram_head.last_byte_erased < (Flog_mram_head.data_ptr + 32768))	// Half a sector
	{
		Flog_mram_head.last_byte_erased += FLOG_SECTOR_SIZE;
		Hub_flog_write_mram_head();		// Update header
		uint32_t sector = (Flog_mram_head.last_byte_erased >> 16);	// Divide by sector size
		SSP_Erase_Sector(sector);
		return;			// May take a while, do something else
	}


	if(Flog_ring_data_count() <= min_count) return;		// Zero to flush

	int data_v;
	uint32_t data_ix = 0;
	for(;data_ix<FLOG_DATA_SIZE;)
	{
		data_v = Flog_get_ring_data();
		if(data_v < 0) break;
		Flog_data_out[data_ix++] = (uint8_t)data_v;
	}
	uint32_t data_len = data_ix;
	uint32_t pos = Flog_mram_head.data_ptr;
	Flog_mram_head.data_ptr += data_len;
	pos &= 0xffffff;		// Must fit in 16 Megabyte range
	Hub_ssp_raw_flash_write(pos,Flog_data_out,data_len);
}
 *
 */
// This allows access to sector zero
//This is trickier than the read as the write will page wrap
#ifdef MH_FLASH_WRITE2
#define SSP_FLASH_PAGE_SIZE	256
uint32_t AC210_raw_flash_write_count;
uint32_t AC210_raw_flash_write_bytes;
void SSP_WriteBuffer(uint32_t byte_address, uint8_t *pBuffer, uint32_t len);
int SSP_ReadBuffer(uint32_t byte_address, uint8_t *pBuffer, uint32_t len);

#ifdef MH_READ_AFTER_WRITE
uint8_t Write_check_buffer[SSP_FLASH_PAGE_SIZE];
#endif
void AC210_ssp_raw_flash_write_2(uint32_t byte_address,uint8_t *buff, uint32_t len)
{
	AC210_raw_flash_write_count++;
	AC210_raw_flash_write_bytes += len;

	uint32_t pos = byte_address;
	uint8_t *bp = buff;
	uint32_t bytes_left = len;

	while(bytes_left > 0)
	{
		uint32_t offset = pos % SSP_FLASH_PAGE_SIZE;
		uint32_t bytes_avail = SSP_FLASH_PAGE_SIZE - offset;
		uint32_t wlen = MIN(bytes_left,bytes_avail);
		pos &= 0xffffff;		// MHH:08/12/2023. Must fit in 16 Megabyte range
		SSP_WriteBuffer(pos,bp,wlen);
#ifdef MH_READ_AFTER_WRITE
		SSP_ReadBuffer(pos,Write_check_buffer,wlen);	// MHH:09/12/2023. Suspicious multiple writes without erase setting bytes to 255
		if(memcmp(bp,Write_check_buffer,wlen))
		{
			DPRINTF("AC210_ssp_raw_flash_write_2:read after write not same!!!\r\n");
			DPRINTF("byte_address:%d,len:%d\r\n",byte_address,len);
		}
#endif
		bytes_left -= wlen;
		pos += wlen;
		bp += wlen;
	}
}
#endif
void Log_putc_write_buffer_flash_2(void)
{
	uint32_t data_len = LogData.serial_write_ix;

	if(data_len > EE_SER_BUFF_SIZE)
	{
		DebugAbort("Log_putc_write_buffer_flash:bytes_left");
		return;
	}

	uint32_t log_pos = AC210_log.putc_pos;

	if(log_pos > FLASH_PART_SIZE - 50)
	{
		mh_debug();
	}

	log_pos %= FLASH_PART_SIZE;		// Must fit in file

#ifndef MH_XXX
	AC210_flash_write_part(log_pos,ee_serial_write_buff,data_len);
#else
	uint32_t raw_pos = log_pos + FLASH_PART_OFFSET;
//	AC210_ssp_raw_flash_write_2(raw_pos,ee_serial_write_buff,data_len);
	AC210_ssp_raw_flash_write(raw_pos,ee_serial_write_buff,data_len);
#endif


	AC210_log.putc_pos += data_len;
//	AC210_log.putc_pos += LogData.serial_write_ix;
	AC210_log.putc_pos %= FLASH_PART_SIZE;
	LogData.serial_write_ix = 0;
	AC210_log.eof_page = AC210_log.putc_pos / FLASH_PAGE_SIZE;

//	DPRINTF("Log_putc_buffer_flash_2:log_pos = %d, data_len = %d, last_pos_written = %d\r\n",log_pos,data_len,AC210_log.putc_pos-1);


	if(LogCtl.head_page != AC210_log.eof_page)
	{
		LogCtl.head_page = AC210_log.eof_page;
		ee_update_head_page();
#ifdef MH_DPRINTF
		DPRINTF("Log_putc_write_buffer_flash_2:LogCtl.head_page = %d\r\n",LogCtl.head_page);
#endif
		uint32_t page_within_sector = LogCtl.head_page % FLASH_LOG_PAGES_PER_SECTOR;
		if(page_within_sector >= 60)		// Leave at least 1 erased page
		{
			uint32_t next_log_sector = 1 + (LogCtl.head_page / FLASH_LOG_PAGES_PER_SECTOR);
			next_log_sector %= FLASH_PART_SECTORS;
			uint32_t next_raw_sector = next_log_sector + FLASH_SECTOR_OFFSET;		// + 1
			if(next_raw_sector != LogCtl.last_flash_sector_erased)
			{
#ifdef MH_DPRINTF
				DPRINTF("Log_putc_buffer_flash_2:flash_erase_sector:%d\r\n",next_raw_sector);
#endif
				AC210_log.flash_erase_sector = next_raw_sector;		// This will be erased from a wait routine
			}
		}
	}
}

//-------------------------------------------------------------------------------------------
#ifdef MH_123
void Log_putc_write_buffer_flash(void)
{

	uint32_t pos  = AC210_log.putc_pos;

	//	uint32_t t1 = us_ticker_read();
//	int bytes_left = LogData.serial_write_ix+EELOG_EOF_BYTES;	// So we add 8 nulls at end
	int bytes_left = LogData.serial_write_ix;
	if(bytes_left > EE_SER_BUFF_SIZE)
	{
		DebugAbort("Log_putc_write_buffer_flash:bytes_left");
		return;
	}

	// Work out page that eof marker starts on, if changes will update current page after doing this write

	uint32_t eof_pos = AC210_log.putc_pos + LogData.serial_write_ix;
	if(eof_pos >= EELOG_SPACE) eof_pos -= EELOG_SPACE;
	AC210_log.eof_page = eof_pos / EELOG_PAGESIZE;

	uint8_t *p_src = ee_serial_write_buff;
	for(;;)
	{
		if(pos >= EELOG_SPACE)		// Was EEPROM_SIZE
		{
			if(pos > EELOG_SPACE)
			{
				DebugAbort("Log_putc_write_buffer_flash: pos > EELOG_SPACE");
				return;
			}
			pos = 0;		// wrap around
//			Stat_Rec.log_flags |= EELOG_FLAG_WRAPPED;	// Superceded by head_wrap_cnt
			printf("Log_putc_write_buffer_flash:wrapping\r\n");
		}
		int space = EELOG_SPACE - pos;	// Was EEPROM_SIZE
		int wlen = MIN(bytes_left,space);

		if(ee_write_log(p_src,wlen,pos) < 0)
		{
			return;
		}
		pos += wlen;
		bytes_left -= wlen;
		if(bytes_left == 0) break;
		if(bytes_left > EE_SER_BUFF_SIZE)
		{
			DebugAbort("Log_putc_write_buffer_flash:bytes_left(2)");
			return;
		}
		p_src += wlen;
	}
//	uint32_t t2 = us_ticker_read();
	AC210_log.putc_pos += LogData.serial_write_ix;
	if(AC210_log.putc_pos >= EELOG_SPACE) AC210_log.putc_pos -= EELOG_SPACE;

	LogData.serial_write_ix = 0;
//	printf("\r\nFlush:AC210_log.putc_pos:%d, ms:%d\r\n",AC210_log.putc_pos,(t2-t1+500)/1000);
}
#endif
//-------------------------------------------------------------------------
#ifdef MH_XXX
int eelog_check_diags_rec(void)
{
	td_Stats d_r;
	printf("eelog_check_diags_rec:\r\n");
	uint32_t diags_pos = AC210_log.getc_pos2;

	if(Log_read_diag_rec(&d_r))
	{
		return -2;
	}

	// Note: we could adapt this routine to fix positions if not correct

	printf("run_number    :%10d\r\n",d_r.run_number);
	printf("diags_pos     :%10d\r\n",diags_pos);
	printf("last_diags_pos:%10d\r\n",d_r.log_last_diags_pos);
	printf("start_data_pos:%10d\r\n",d_r.log_start_data_pos);
	return  d_r.log_last_diags_pos;
}
#endif
//-------------------------------------------------------------------------
uint32_t display_2d1_count;
#ifdef MH_DISPLAY_M8DATA
static bool Display2D1(M8_T *p_d)
{
	if(display_2d1_count  == 0)
	{
		printf("   Count     Pos   Ltick     PWM  ActRPM  SetRPM Current\r\n");
	}

	AC210_uart3_wait();		// Wait for buffer to be output

	if(++display_2d1_count % 40 == 0)
	{
		if(ReturnToContinue() == 'X') return true;
	}
	//         1         2         3         4
	//1234567890123456789012345678901234567890
	//nnnnnnnnNNNNNNNNnnnnnnnnNNNNNNNNnnnnnnnnNNNNNNNN
	//   Count     Pos   Ltick     PWM  ActRPM Current
	printf("%8d",display_2d1_count);
	printf("%8d",AC210_log.getc_pos2_save);
	printf("%8d",p_d->ticks);
//	printf("%s.control_state = %d\r\n",p_d1->control_state);
	printf("%8d",p_d->pwm_val);
    printf("%8d",p_d->actualspeed);
    printf("%8d",p_d->setspeed);
    printf("%8d",p_d->current * 10);
//    printf("%s.motor_drive_state = %02x H\r\n",p_d1->motor_drive_state);
//    printf("%s.run_state         = %02x H\r\n",p_d1->run_state);
//    printf("%s.map2              = %02x H\r\n",p_d1->map2);
    printf("\r\n");
    AC210_uart3_wait();		// Wait for buffer to be output
/*
    if(display_2d1_count >= 806)
    {
    	int x=1;
    }
*/
    return false;
}
//-------------------------------------------------------------------------
static void DisplayM8_D(M8_T *p_d)
{
	printf("ticks         = %d\r\n",p_d->ticks);
	printf("pwm_val       = %d\r\n",p_d->pwm_val);
    printf("actualspeed   = %d\r\n",p_d->actualspeed);
    printf("setspeed      = %d\r\n",p_d->setspeed);
    printf("current       = %d\r\n",p_d->current);
	printf("control_state = %d\r\n",p_d->control_state);
    uint16_t sys_state = p_d->sys_state;
    uint8_t sys_run   = (uint8_t) sys_state & 0xf;
    uint8_t sys_stops = (uint8_t)((sys_state >> 4)&0xf);
	uint8_t sys_errs  = (uint8_t)((sys_state >> 8)&0xf);
	printf("sys_run       = %d\r\n",sys_run);
    printf("sys_stops     = %d\r\n",sys_stops);
    printf("sys_errs      = %d\r\n",sys_errs);
    printf("xoar_status   = %d\r\n",p_d->xoar_status);
    printf("op_mode       = %d\r\n",p_d->op_mode);
    printf("manual_keys   = %d\r\n",p_d->manual_keys);
    printf("motor_drive_state = %02x H\r\n",p_d->motor_drive_state);
    printf("\r\n");
}
#endif
//-------------------------------------------------------------------------
void DebugBreak(void)
{
	AC210_uart3_wait();
	printf("\r\nDebugBreak\r\n");
}
//-------------------------------------------------------------------------
#ifdef MH_BINARY_PRINT
void BinaryPrint(uint32_t u)
{
	uint32_t bit_mask = (1<<31);
	char buff[36];
	uint8_t ix=0;
	bool found_bit_on=false;
	for(int i=0;i<32;i++)
	{
		uint8_t bit = 0;
		if((bit_mask & u) != 0)
		{
			bit = 1;
			found_bit_on = true;
		}
		if(i == 24)found_bit_on = true;		// Force full print of last byte
		if(found_bit_on)
		{
			if((i % 4) == 0) buff[ix++] = ' ';	// Put spaces between groups
			if((i % 8) == 0) buff[ix++] = ' ';
			buff[ix++] = (char)(bit + '0');
		}
		bit_mask >>= 1;
	}
	buff[ix] = 0;
	printf("[%s]",buff);
}
#endif
//-------------------------------------------------------------------------
// mhh:25/07/2022

bool ee_check_all_255(uint8_t *buff,int len)
{
	uint8_t *bp = buff;
	for(int j=0;j<len;j++)
	{
		if(*bp++ != FLASH_EOF) return false;
	}
	return true;
}

// Only here if a head_page value of zero. Value of zero OK if last_flash_sector erased == 1.
int ee_find_head_page_from_last_flash_sector_erased(void)
{
	int last_flash_sector_erased = LogCtl.last_flash_sector_erased;
	int last_page_erased = last_flash_sector_erased * FLASH_LOG_PAGES_PER_SECTOR - 1;	// Should be 64 pages per sector

	int pos = last_page_erased * EELOG_PAGESIZE;
	int check_page = last_page_erased;
	uint8_t *p_dst = ee_serial_read_buff;
	int rlen = 8;		// Read 8 bytes
	for(int i=0;i<128;i++)
	{
		AC210_flash_read_part(pos,p_dst,rlen);
		if(ee_check_all_255(ee_serial_read_buff,8) == false)	// Found a data page?
		{
			if(check_page != LogCtl.head_page)
			{
				LogCtl.head_page = check_page;
				ee_update_head_page();
			}
			return 0;
		}
		check_page--;
		pos -= EELOG_PAGESIZE;
		if(pos < 0)
		{
			return -1;
		}

	}
	return -1;
}

#ifdef MH_YYY
int ee_log_scan_for_write_pos(void)
{
	// We know page it is on, look for 8 eofs...


	// MHH:22/07/2022. Xoar has case where head_page = 0.
	// Should be able to find real head page by scanning backwards page by page from last sector erased.

	if(LogCtl.head_page == 0)
	{
		if(ee_find_head_page_from_last_flash_sector_erased() < 0) return -1;
	}

	AC210_log.getc_pos  = LogCtl.head_page * EELOG_PAGESIZE;
	AC210_log.getc_pos2 = AC210_log.getc_pos;

	uint32_t start_pos = AC210_log.getc_pos;

	ee_serial_read_ix = EE_SER_BUFF_SIZE;		// force read

	int eof_ix = 0;
	int i_start_eof = 0;

	for(int i=0;i<(EELOG_PAGESIZE*4)+FLASH_EOF_COUNT;i++)
	{
		uint8_t c = ee_getc();

		if(c != FLASH_EOF)		// because Flash memory is initialised to 255
		{
			eof_ix = 0;
		}
		else
		{
			if(eof_ix == 0) i_start_eof = i;
			if(++eof_ix >= FLASH_EOF_COUNT) break;
		}
	}
	if(eof_ix != FLASH_EOF_COUNT)
	{
//		DebugAbort("ee_log_scan_for_write_pos: eof not found");
		return -1;
	}
	AC210_log.putc_pos = start_pos + i_start_eof;
	AC210_log.putc_pos2 = AC210_log.putc_pos;
	AC210_log.eof_page  = AC210_log.putc_pos / EELOG_PAGESIZE;
	if(LogCtl.head_page != AC210_log.eof_page)
	{
		// Should not get here, but just in case...
		LogCtl.head_page = AC210_log.eof_page;
		ee_update_head_page();
	}
	L2PRINTF("AC210_log.putc_pos:%d\r\n",AC210_log.putc_pos);
	return 0;
}
#endif
//-------------------------------------------------------------------------
void AC210_log_init(void)
{
	if(LogData.enabled) return;		// already done
	if(getParameter(FLASH_LOG_RATE) == 0)
	{
		return;		// This way we can disable flash logging logic
	}

	int s = sizeof(M8_T);
	if(s != 32)
	{
		LPRINTF("Log_init: sizeof(M8_T) = %d\r\n",s);
		DebugAbort("AC210_log_init:1");
	}
//	AC210_log.log_ticks=0;			// Should we init at the start of a run? Probably
#ifdef MH_XXX
	AC210_log.start_data_pos = Stat_Rec.log_start_data_pos;
#endif
//	AC210_log.head_wrap_cnt  = Stat_Rec.head_wrap_cnt;	// Maybe should get from LogCtl?? But we want it to be in sync with start_data...
	LogData.log_ticks = 0;	// Cope with restart in diagnostics
	if(ee_get_logctl() < 0)
	{
		return;
	}
	LogData.enabled = true;
#ifdef MH_YYY	// MHH:22/12/2023. putc position now handled in ee_get_logctl()
	if(ee_log_scan_for_write_pos() == 0)
	{
		LogData.enabled = true;
	}
	else
	{
		DebugAbort("AC210_log_init:cannot find eof for eelogfile");
		return;
	}
#endif
	if(LogData.enabled)
	{
		Log_change_rate(getParameter(FLASH_LOG_RATE));
	}
}
//--------------------------------------------------------------------------------------------------------------
static void Checkpoint_put_data(uint8_t type)
{
	Log_putc(EELOG_C_ESCAPE1);
	Log_putc(EELOG_C_ESCAPE2);
	Log_putc(type);

	uint8_t *bp = (uint8_t *)&Log_checkpoint;
	for(int i=0;i<LOG_CHECKPOINT_SIZE;i++)
	{
		Log_putc(*bp++);
	}
	Log_putc(EELOG_C_ESCAPE3);
}
//-----------------------------------------------------------------------------------------
#ifdef MH_XXX
void AC210_eelog_write_diags(void)
{
//	printf("\r\nAC210_eelog_write_diags:\r\n");

	Log_Checkpoint(EELOG_C_CHECKPOINT_DIAGS);		// Note: Checkpoint flushes flash buffer

	int last_diags_pos = AC210_log.putc_pos;		// Position Diags_rec will be written to

	int current_log_filesize = AC210_current_log_filesize(2);
	if(current_filesize < 0) current_filesize = 0;		// An error?
	Stat_Rec.log_filesize = current_log_filesize + DIAGS_LONG_LEN;

	uint8_t *pb = (uint8_t *) &Stat_Rec;
	for(int i=0;i<DIAGS_LONG_LEN;i++) Log_putc(*pb++);
	Log_putc_write_buffer();	// flush

	Log_write_stat_index(&Stat_Rec,last_diags_pos);		// To eeprom

	Stat_Rec.log_last_diags_pos = last_diags_pos;	// Update Stat_Rec with new log positions

	Stat_Rec.log_start_data_pos = AC210_log.putc_pos;		// Position data will start at
//	Stat_Rec.head_wrap_cnt   = LogCtl.head_wrap_cnt;		// MHH:11/12/2023
#ifdef MH_YYY		// MHH:13/12/2023. This is already done in Diags_new_run()
	LogData.first_checkpoint = false;		// Force a flash checkpoint
	WriteStatsRec("eelog_write_diags",DIAGS_LONG_LEN);
#endif
}
#endif
//-----------------------------------------------------------------------------------------
static int EE_erase_index_and_logctl(void);

void eeprom_log_reset(WORD val)
{
	AC210_watchdog_active = false;				// This could take a while
	L2PRINTF("eeprom_log_reset:\n\r");

	L2PRINTF("Erasing Flash sector 1\r\n");
	AC210_ssp_flash_erase(1);
	L2PRINTF("Erasing Flash sector 255\r\n");
	AC210_ssp_flash_erase(255);		// MHH:04/01/2026. Allows logctl recovery as if last page all 255 and first page not 255 then assume no wrap.
	EE_erase_index_and_logctl();
	if(AC210_reset_logctl())
	{
		Abort(AC210_SRC_LOG+10,"AC210_reset_logctl fail");
	}
	AC210_log.putc_pos = 0;
	AC210_log.putc_pos2 = 0;
	AC210_log.getc_pos = 0;
	AC210_log.getc_pos2 = 0;

	LogData.serial_write_ix = 0;		// !!!
	LogData.first_checkpoint = false;	// So we get a checkpoint on start
	LogData.enabled = true;				// MHH:20/12/2023. In case disabled when initialising.
	Log_change_rate(getParameter(FLASH_LOG_RATE));	// MHH:22/12/2023.
#ifdef MH_CHECKPOINT_ZERO
	LogData.zero_compare_data = true;	// to be safe
	LogData.zero_check_data = true;
#endif
	Stat_Rec.log_last_diags_pos = EELOG_END_CHAIN;
	Stat_Rec.log_start_data_pos = 0;
   	WriteStatsRec("eeprom_log_reset",DIAGS_LONG_LEN);

   	LogData.log_ticks = 0;
//	AC210_log_init();		// Double check - This can cause recursive call!!
	AC210_watchdog_active = true;
}
//--------------------------------------------------------------------------------------------------------
void ee_output_buffer(void)
{
	char pbuff[80];
	uint8_t *fsrc=UU_fBuff;                       // src = start of file buffer
    UU_Sum20 = 0;
    int bleft = 1024;
    int lcnt = 0;

#ifdef MH_RCV_DUMP_REPLY		// leave for now
    int save_bleft = bleft;             // In case we have to RESEND
    uint8_t *save_fsrc = fsrc;
#endif

    while (bleft > 0) {
        int copylen = 45;
        if(bleft < 45) copylen = bleft;
        UUEncodeLine(fsrc,copylen);
        UUChecksumLine(copylen);
        fsrc += copylen;
        bleft-= copylen;

#ifdef MH_WAIT_MS
        wait_ms(1);      // What about 1? YES
#endif
       //        wait_ms(0);      // What about ZERO???? NO!!

        sprintf(pbuff,":E:%s\r\n",UU_uBuf);
        txDebug(pbuff);

#ifdef MH_WAIT_MS
        wait_ms(10);        // Fails without a pause here!!!
#endif

// Now check if we have send 20 lines or this is the last line.

        lcnt++;
        if(lcnt >= 20 || bleft == 0) {
        	char str[20];
        	sprintf(str,":E:%d\r\n",UU_Sum20);
            txDebug(str);		// Send checksum
//            SendSerialNL(CommandBuffer);   // Send checksum
            UU_Sum20 = 0;
            lcnt = 0;
#ifdef MH_RCV_DUMP_REPLY		// leave for now
            ReceiveSerialNL();
            if(strcmp(Rbuffer,"RESEND") == 0) {
                if(ErrorCount++ > 20) {
                    pc.printf("Too many errors,exiting\n\r");
                    exit(1);
                }
                pc.printf("Checksum Error, RESENDING last block\n\r");
                bleft = save_bleft;
                fsrc  = save_fsrc;
            } else {
                if(strcmp(Rbuffer,"OK")) {
                    pc.printf("Checksum Error\n\r");
                    exit(1);
                }
            }
#endif
        }
    }
}
//--------------------------------------------------------------------------------------------------------
//#define AC210_EELOG_HEADER_PAGES			4

void ee_dump_all_pages(void)
{
// May be helpful to dump current Stat_Rec as well, as this gives pointers to last used.
// If we dump entire first page of eeprom first then we also have parameter file

// Note: Will need to consider tail_page!!
	uint32_t file_pages = LogCtl.head_page;
	if(LogCtl.head_page < LogCtl.tail_page)	// MHH:31/07/2023. wrap around?
	{
		file_pages = LogCtl.max_pages;
	}

	AC210_watchdog_active = false;				// This could take a while

#ifdef MH_VERBOSE
	printf("AC210_ee_dump_all_pages: Dumping %d pages\r\n",file_pages);
	printf("Param and Diag rec page\r\n");
#endif

	uint32_t pos = 0;
	p_eeprom_i2c_pos(pos);
	if(p_eeprom_i2c_read(UU_fBuff, EELOG_PAGESIZE))
	{
	    txDebug(":A:ERR\r\n");
		DebugAbort("i2c_read");
	}
	ee_output_buffer();

	pos = 0;
	for(int page=0;page<=file_pages;page++)
	{
#ifdef MH_VERBOSE
		printf("FilePage:%d\r\n",page);
#endif
		if(ee_read_log(UU_fBuff,EELOG_PAGESIZE,pos) < 0)
		{
		    txDebug(":A:ERR\r\n");
		    printf("Error reading log\r\n");
		    return;
		}
		ee_output_buffer();
		pos += EELOG_PAGESIZE;
	}
	txDebug(":A:PLOG\r\n");
	AC210_plog_display(1);
    txDebug(":A:EOF\r\n");
#ifdef MH_VERBOSE
	printf("Finished dump\r\n");
#endif

}
//--------------------------------------------------------------------------------------------------------
#ifdef MH_XXX
typedef struct
{
	uint16_t magic;
	uint16_t page_map_max;
	struct
	{
		uint16_t from_page;
		uint16_t to_page;
		int16_t page_adjust;		// May be + or -.
	}pagemap[2];
} td_DUMP5Mb;
#endif
td_DUMP5Mb Dump5Mb;


#define DUMP_PAGES		5*1024		// Note: we can make this (say) 10 pages for testing.
//#define DUMP_PAGES		100		// Test with 100 pages
extern void Put_int2(uint8_t *dst,int16_t v);		// In ac210_sig100.c
int Pages_output;
void ee_dump_page_range(int first_page,int last_page)
{
//	DPRINTF("ee_dump_page_range:first_page=%d,last_page=%d\r\n",first_page,last_page);
	int pos = first_page * EELOG_PAGESIZE;
	for(int page=first_page;page<=last_page;page++)
	{
#ifdef MH_VERBOSE
		printf("FilePage:%d\r\n",page);
#endif
		if(ee_read_log(UU_fBuff,EELOG_PAGESIZE,pos) < 0)
		{
			txDebug(":A:ERR\r\n");
			printf("Error reading log\r\n");
			return;
		}
//#define MH_DEBUG_TEST_5MB
#ifdef MH_DEBUG_TEST_5MB
		Put_int2(UU_fBuff,(int16_t) page);
		Put_int2(UU_fBuff+2,(int16_t) page+page_adjust);
#endif
		ee_output_buffer();
		pos += EELOG_PAGESIZE;
		Pages_output++;
	}
//	DPRINTF("Pages_output:%d\r\n",Pages_output);
}
void ee_dump_last_XMb_pages(int megabytes)	// MHH:31/07/2023
{

	int start_page;

	uint32_t pos = 0;	// First load eeprom first page
	p_eeprom_i2c_pos(pos);
	if(p_eeprom_i2c_read(UU_fBuff, EELOG_PAGESIZE))
	{
		txDebug(":A:ERR\r\n");
		DebugAbort("i2c_read");
	}

//	uint16_t dump_pages = (uint16_t)DUMP_PAGES;		// WE may want to change size of dump from diagnostics.
	uint16_t dump_pages = (uint16_t)(megabytes * 1024);		// WE may want to change size of dump from diagnostics.
	Dump5Mb.magic = 12345;
	Dump5Mb.page_map_max = 1;			// default
	if(LogCtl.head_page >= LogCtl.tail_page || LogCtl.head_page > dump_pages)	// Wrapped?
	{
		start_page = LogCtl.head_page - dump_pages;	// No
		if(start_page < 0) start_page = 0;
		Dump5Mb.pagemap[0].page_adjust = -start_page;
		Dump5Mb.pagemap[0].from_page = start_page;
		Dump5Mb.pagemap[0].to_page = LogCtl.head_page;
	}
	else
	{	// wrapped. Going to send out in time order, so oldest pages first...

		Dump5Mb.page_map_max = 2;
		int first_dump_pages = dump_pages -LogCtl.head_page;
		start_page = LogCtl.max_pages - first_dump_pages;				// First the oldest pages...
		if(start_page < LogCtl.tail_page)	// MHH:30/12/2023
		{
			start_page = LogCtl.tail_page;
			first_dump_pages = LogCtl.max_pages - start_page;
		}

		Dump5Mb.pagemap[0].page_adjust = -start_page;					// offset in pages from zero (if any)
		Dump5Mb.pagemap[0].from_page = start_page;
		Dump5Mb.pagemap[0].to_page = LogCtl.max_pages-1;					// page 16,319

		Dump5Mb.pagemap[1].page_adjust = first_dump_pages;					// Now the wrapped portion, up to the head page.
		Dump5Mb.pagemap[1].from_page = 0;
		Dump5Mb.pagemap[1].to_page = LogCtl.head_page;
	}

	uint8_t *psrc = (uint8_t *) &Dump5Mb;		// This structure will be used to map pages in diagnostics program.
//	uint8_t *pdst=UU_fBuff + EE_LOGCTL_POS + 32;
	uint8_t *pdst=UU_fBuff + EE_DUMP5MB_OFFSET;	// Next to plog head
	memcpy(pdst,psrc,sizeof(Dump5Mb));
	ee_output_buffer();

	AC210_watchdog_active = false;				// This could take a while

	int from_page,to_page;
	Pages_output = 0;
	for(int i=0;i<Dump5Mb.page_map_max;i++)
	{
		from_page = Dump5Mb.pagemap[i].from_page;
		to_page = Dump5Mb.pagemap[i].to_page;
//		page_adjust = Dump5Mb.pagemap[i].page_adjust;
		ee_dump_page_range(from_page,to_page);	// Could have just passed the index...
	}
	txDebug(":A:PLOG\r\n");
	AC210_plog_display(1);
	txDebug(":A:EOF\r\n");
#ifdef MH_VERBOSE
	DPRINTF("Finished dump\r\n");
#endif
	AC210_watchdog_active = true;
}
#ifdef MH_XXX
void ee_dump_last_5Mb_pages(void)	// MHH:31/07/2023
{

	int start_page;

	uint32_t pos = 0;	// First load eeprom first page
	p_eeprom_i2c_pos(pos);
	if(p_eeprom_i2c_read(UU_fBuff, EELOG_PAGESIZE))
	{
		txDebug(":A:ERR\r\n");
		DebugAbort("i2c_read");
	}

	uint16_t dump_pages = (uint16_t)DUMP_PAGES;		// WE may want to change size of dump from diagnostics.
	Dump5Mb.magic = 12345;
	Dump5Mb.page_map_max = 1;			// default
	if(LogCtl.head_page >= LogCtl.tail_page || LogCtl.head_page > dump_pages)	// Wrapped?
	{
		start_page = LogCtl.head_page - dump_pages;	// No
		if(start_page < 0) start_page = 0;
		Dump5Mb.pagemap[0].page_adjust = -start_page;
		Dump5Mb.pagemap[0].from_page = start_page;
		Dump5Mb.pagemap[0].to_page = LogCtl.head_page;
	}
	else
	{	// wrapped. Going to send out in time order, so oldest pages first...

		Dump5Mb.page_map_max = 2;
		int first_dump_pages = dump_pages -LogCtl.head_page;
		start_page = LogCtl.max_pages - first_dump_pages;				// First the oldest pages...
		if(start_page < LogCtl.tail_page)	// MHH:30/12/2023
		{
			start_page = LogCtl.tail_page;
			first_dump_pages = LogCtl.max_pages - start_page;
		}

		Dump5Mb.pagemap[0].page_adjust = -start_page;					// offset in pages from zero (if any)
		Dump5Mb.pagemap[0].from_page = start_page;
		Dump5Mb.pagemap[0].to_page = LogCtl.max_pages-1;					// page 16,319

		Dump5Mb.pagemap[1].page_adjust = first_dump_pages;					// Now the wrapped portion, up to the head page.
		Dump5Mb.pagemap[1].from_page = 0;
		Dump5Mb.pagemap[1].to_page = LogCtl.head_page;
	}

	uint8_t *psrc = (uint8_t *) &Dump5Mb;		// This structure will be used to map pages in diagnostics program.
//	uint8_t *pdst=UU_fBuff + EE_LOGCTL_POS + 32;
	uint8_t *pdst=UU_fBuff + EE_DUMP5MB_OFFSET;	// Next to plog head
	memcpy(pdst,psrc,sizeof(Dump5Mb));
	ee_output_buffer();

	AC210_watchdog_active = false;				// This could take a while

	int from_page,to_page;
	Pages_output = 0;
	for(int i=0;i<Dump5Mb.page_map_max;i++)
	{
		from_page = Dump5Mb.pagemap[i].from_page;
		to_page = Dump5Mb.pagemap[i].to_page;
//		page_adjust = Dump5Mb.pagemap[i].page_adjust;
		ee_dump_page_range(from_page,to_page);	// Could have just passed the index...
	}
	txDebug(":A:PLOG\r\n");
	AC210_plog_display(1);
	txDebug(":A:EOF\r\n");
#ifdef MH_VERBOSE
	DPRINTF("Finished dump\r\n");
#endif
	AC210_watchdog_active = true; // 768
}
#endif
//---------------------------------------------------------------------------------------------
static int UU_fbuff_ix;
static void UU_putc(uint8_t c)
{
	UU_fBuff[UU_fbuff_ix++] = c;
	if(UU_fbuff_ix >= EELOG_PAGESIZE)
	{
		ee_output_buffer();
		UU_fbuff_ix = 0;
	}
}
//-----------------------------------------------------------------------------------------
void AC210_ee_load_range(void)
{
	if(Stat_Rec.log_start_data_pos < 0)
	{
#ifdef MH_VERBOSE
		printf("AC210_ee_load: start_data_pos < 0\r\n");
#endif
		txDebug(":A:ERR:start_data_pos < 0\r\n");
		return;
	}

	AC210_watchdog_active = false;				// This could take a while

	// Note: Maybe make this into a subroutine

#ifdef MH_VERBOSE
	printf("Param and Diag rec page\r\n");
#endif

//	uint32_t pos = 0;
	p_eeprom_i2c_pos(EELOG_PARAM_POS);
	if(p_eeprom_i2c_read(UU_fBuff, EELOG_PAGESIZE))
	{
	    txDebug(":A:ERR\r\n");
		DebugAbort("i2c_read");
	}
	ee_output_buffer();

//	ee_serial_read_ix = EE_SER_BUFF_SIZE;				// force read

	int first_run = AC210_range.first_run;
	int last_run  =  AC210_range.last_run;

	int run_offset = 0;		// Use this to adjust start_data and last_drec ptr
	int last_drec_pos = -1;	// End of chain
	UU_fbuff_ix=0;

	int start_data_pos;
	int drec_pos;
	td_Stats d_r;
	Index_page = -1;	// In case of restore or reset

	Log_putc_write_buffer();	// MHH:10/12/2023. So we can use putc_pos, not putc_pos2

	for(int run=first_run;run<=last_run;run++)
//	for(int ix = Dpos_ix-1;ix>=0;ix--)
	{
		if(run == Stat_Rec.run_number)
		{
			d_r = Stat_Rec;
			drec_pos = AC210_log.putc_pos2;	// probably
			start_data_pos = d_r.log_start_data_pos;
		}
		else
		{
			Index_get_index(run);
			if(pIndex->run == LOG_INDEX_ERASED_RUN)
			{
				continue;		// Ignore
			}
//			Index_get_next();
			drec_pos = pIndex->log_diag_pos;
#ifdef MH_XXX
			if(Log_peek4_pos(drec_pos) == PARAM_CHECK_CODE)	// MHH:18/04/2026. Param rec?
			{
				drec_pos += 256;	// Yes
			}
#endif
// Using index data_pos as against drec data_pos allows repair of data as can modify index but not drec.
			start_data_pos = pIndex->log_start_pos;
			if(eelog_read_diag_rec(&d_r,drec_pos))
			{
//			    txDebug(":A:ERR:Invalid Check code(2)\r\n");
//				return;		// error
				continue;		// Just ignore this run, something wrong
			}
		}

		ee_getc_position(start_data_pos);
		int log_max_size = drec_pos - start_data_pos;
		if(log_max_size < 0) log_max_size += EELOG_SPACE;

		int eof_ix = 0;

#ifdef MH_VERBOSE
		printf("AC210_ee_load: log_max_size = %d\r\n",log_max_size);
#endif
		int bytes_out = 0;
		for(int i=0;i<log_max_size;i++)
		{
			uint8_t c = ee_getc();
			if(c != FLASH_EOF)		// because Flash memory is initialised to 255
			{
				eof_ix = 0;
			}
			else
			{
				if(++eof_ix >= FLASH_EOF_COUNT) break;
			}
			bytes_out++;
			UU_putc(c);
		}

	// Could possibly output an eof here??

		bytes_out++;
		UU_putc(FLASH_EOF);		// 04/04/17: Try, getting an odd bug when trying to display graph. Possibly caused by lack of eof

	// OK, we have written all the data. Now to adjust the diagnostics rec and write it.

		d_r.log_start_data_pos = run_offset;
		d_r.log_last_diags_pos = last_drec_pos;
		run_offset += bytes_out;
		last_drec_pos = run_offset;
		uint8_t *bp= (uint8_t *) &d_r;
		for(int j=0;j<DIAGS_LONG_LEN;j++) UU_putc(*bp++);	// output diags_rec
		run_offset += DIAGS_LONG_LEN;
	}

	while(UU_fbuff_ix > 0)
	{
		UU_putc(0);		// null fill to eof of page and flush
	}
	// OK, still have to tell AC200Diagnostics where the last diags rec is.
	PRINTF(":A:DPOS=%d\r\n",last_drec_pos);

//	txDebug(":A:EOF\r\n");

}
//---------------------------------------------------------------------------------------------
#define EELOAD_DUMP_ALL		65535
#define EELOAD_DUMP_5MB		65534		// MHH:31/07/2023
#define EELOAD_DUMP_1MB		65533		// MHH:27/06/2025
// Dump current could be 65534??

void AC210_ee_load(WORD value)	// called from comms.c
{
	Log_putc_write_buffer();	// MHH:10/12/2023. So we can use putc_pos, not putc_pos2

	if(value == EELOAD_DUMP_ALL)
	{
		ee_dump_all_pages();
		return;
	}
#ifdef MH_XXX
	if(value == EELOAD_DUMP_5MB)	// MHH:31/07/2023
	{
		ee_dump_last_5Mb_pages();
		return;
	}

	if(value == EELOAD_DUMP_1MB)	// MHH:27/06/2025
	{
		ee_dump_last_1Mb_pages();
		return;
	}
#endif
	if(value == EELOAD_DUMP_5MB)	// MHH:31/07/2023
	{
		ee_dump_last_XMb_pages(5);
		return;
	}

	if(value == EELOAD_DUMP_1MB)	// MHH:27/06/2025
	{
		ee_dump_last_XMb_pages(1);
		return;
	}


	if(Stat_Rec.log_start_data_pos < 0)
	{
#ifdef MH_VERBOSE
		printf("AC210_ee_load: start_data_pos < 0\r\n");
#endif
		txDebug(":A:ERR:start_data_pos < 0\r\n");
		return;
	}
	AC210_watchdog_active = false;				// This could take a while

//	ee_serial_read_ix = EE_SER_BUFF_SIZE;				// force read
//	AC210_log.getc_pos = Diags_last_d_r_sent.log_start_data_pos;	// For this run

	ee_getc_position(Diags_last_d_r_sent.log_start_data_pos);

	int last_pos;
	if(Diags_last_drec_pos == -1)		// If current rec
	{
//		last_pos = (Diags_last_d_r_sent.log_eof_page + 2) * EELOG_PAGESIZE;
		last_pos = AC210_log.putc_pos2 + EELOG_PAGESIZE;	// MHH:03/02/2017
		Log_putc_write_buffer();						// Flush buffer
#ifdef MH_VERBOSE
		int len = AC210_log.putc_pos2 - AC210_log.getc_pos;
		PRINTF("getc_pos:%d,putc_pos2:%d,len:%d\r\n",AC210_log.getc_pos,AC210_log.putc_pos2,len);
#endif
	}
	else
	{
		last_pos = Diags_last_drec_pos;
	}
	int log_max_size = last_pos - AC210_log.getc_pos;
	if(log_max_size < 0) log_max_size += EELOG_SPACE;	// Think this will work, need to test wrap.

	int eof_ix = 0;

#ifdef MH_VERBOSE
	PRINTF("AC210_ee_load: log_max_size = %d\r\n",log_max_size);
#endif
	int uu_ix = 0;
	int bytes_out = 0;
	for(int i=0;i<log_max_size;i++)
	{
		uint8_t c = ee_getc();
		if(c != FLASH_EOF)		// because Flash memory is initialised to 255
		{
			eof_ix = 0;
		}
		else
		{
			if(++eof_ix >= FLASH_EOF_COUNT) break;
		}
		UU_fBuff[uu_ix++] = c;
		bytes_out++;
		if(uu_ix >= EELOG_PAGESIZE)
		{
			ee_output_buffer();
			uu_ix = 0;
		}
	}
	if(uu_ix > 0)
	{
		for(int i=uu_ix;i<EELOG_PAGESIZE;i++) UU_fBuff[i] = FLASH_EOF;	// eof fill to end of page
		ee_output_buffer();
	}
#ifdef MH_VERBOSE
	printf("\r\nFinished:bytes_out=%d\r\n",bytes_out);
#endif
	if(eof_ix != FLASH_EOF_COUNT && (Diags_last_drec_pos == -1))
	{
	    txDebug(":A:ERR:No EOF\r\n");
//		DebugAbort("AC210_ee_load:eof not found");
	}
	else
	{
	    txDebug(":A:EOF\r\n");
	}
}

//---------------------------------------------------------------------------------------------
int Log_read_diag_rec(td_Stats *d_r)
{
	uint8_t *bp= (uint8_t *) d_r;
	for(int i=0;i<DIAGS_LONG_LEN;i++) *bp++ = ee_getc();	// get diags_rec
	// Now check
	if(d_r->check_code != DIAGS_REC_CHECK_CODE)
	{
		DebugAbort("Log_read_diag_rec: invalid check_code");
		return-1;
	}
	return 0;
}
//---------------------------------------------------------------------------------------------
static int Log_load_diags_rec(int run,td_Stats *p_DR)
{
	int log_filesize;
	int diags_pos = -1;
	if(run == Stat_Rec.run_number)	// Current Rec?
	{
		*p_DR = Stat_Rec;
		log_filesize = AC210_log.putc_pos - Stat_Rec.log_start_data_pos;
		if(log_filesize < 0) log_filesize += EELOG_SPACE;
		p_DR->log_filesize = log_filesize;
		Log_index.log_diag_pos = -1;
		Log_index.log_start_pos = Stat_Rec.log_start_data_pos;
	}
	else
	{
		if(Log_read_stat_index(run))
		{
		    txDebug(":A:ERR DiagIndex read fail\r\n");
			return -1;
		}
		diags_pos = Log_index.log_diag_pos;
		if(diags_pos < 0)
		{
		    txDebug(":A:ERR DiagPos < 0\r\n");
			return -1;		// error
		}
#ifdef MH_XXX
		if(Log_peek4_pos(diags_pos) == PARAM_CHECK_CODE)	// MHH:18/04/2026. Param rec?
		{
			diags_pos += 256;	// Yes
		}
#endif
		if(eelog_read_diag_rec(p_DR,diags_pos))
		{
		    txDebug(":A:ERR DiagRec invalid\r\n");
			return -1;		// error
		}
		log_filesize = Log_index.log_diag_pos - Log_index.log_start_pos;
	}
	if(log_filesize < 0) log_filesize += EELOG_SPACE;
	p_DR->log_filesize = log_filesize;
	Log_index_filesize = log_filesize;
	return 0;
}
//#endif
//---------------------------------------------------------------------------------------------
static void Log_add_to_stat_rec(td_Stats *p_DR)
{
// Need to add contents of duplicate d_r to Stat_Rec, both total and run.

	td_Stats prev_DR;

	int dup_run = p_DR->run_number;
	int prev_run = dup_run - 1;
	if(prev_run == 0)	// Only if we try and dup run 1
	{
		memset((char *)&prev_DR,0,sizeof(prev_DR));
	}
	else
	{
		if(Log_load_diags_rec(prev_run,&prev_DR))
		{
			return;
		}
	}
	Stat_Rec.tot.secs        += p_DR->tot.secs - prev_DR.tot.secs;
	Stat_Rec.run_secs        += p_DR->run_secs - prev_DR.run_secs;
	Stat_Rec.tot.motor_secs2 += p_DR->tot.motor_secs2 - prev_DR.tot.motor_secs2;

	ULONG *pdst_tot_rpmSecs = &Stat_Rec.tot.rpmSecs.tot;
	ULONG *pdst_tot_mtrSecs = &Stat_Rec.tot.motorSecs.tot;

	WORD *pdst_run_rpmSecs  = &Stat_Rec.run.rpmSecs.tot;
	WORD *pdst_run_mtrSecs  = &Stat_Rec.run.motorSecs.tot;

	WORD *psrc_run_rpmSecs  = &p_DR->run.rpmSecs.tot;
	WORD *psrc_run_mtrSecs  = &p_DR->run.motorSecs.tot;

	for(int i=0;i<7;i++)
	{
		pdst_tot_rpmSecs[i] += psrc_run_rpmSecs[i];
		pdst_run_rpmSecs[i] += psrc_run_rpmSecs[i];

		pdst_tot_mtrSecs[i] += psrc_run_mtrSecs[i];
		pdst_run_mtrSecs[i] += psrc_run_mtrSecs[i];
	}

// Leave modifying opencircuit and overcurrent for now
	WriteStatsRec("Log_add_to_stat_rec",DIAGS_SHORT_LEN);

	PRINTF("Current Stat_Rec updated\r\n");
}
//---------------------------------------------------------------------------------------------
bool Log_duplicate;
uint8_t Log_dup_getc(void)
{
	uint8_t b = ee_getc();
	if(Log_duplicate)
	{
		Log_putc(b);
	}
	return b;
}
//---------------------------------------------------------------------------------------------------------------------
static void Log_error(char *msg)
{
	PRINTF("%s,Log_drec_cnt:%d,ee_getc_cnt:%d\r\n",msg,Log_drec_cnt,ee_getc_cnt);
	PRINTF_FLUSH;
	//	AC210_uart0_wait();		// Wait for buffer to be output
	if(gVerbose > 5)
	{
		DebugAbort(msg);
	}
}
//---------------------------------------------------------------------------------------------------------------------
static int Display_lcnt;
static bool Display_M8Data_in(void)
{
/*

#define S_IDLE          (WORD)0x0000

#define S_RUN_FINE      (WORD)0x0001
#define S_RUN_COARSE    (WORD)0x0002
#define S_RUN_FEATHER   (WORD)0x0004
#define S_RUN_REVERSE   (WORD)0x0008

#define S_STOP_FINE     (WORD)0x0010
#define S_STOP_COARSE   (WORD)0x0020
#define S_STOP_FEATHER  (WORD)0x0040
#define S_STOP_REVERSE  (WORD)0x0040

#define S_ERROR_OPEN    (WORD)0x0100
#define S_ERROR_CURRENT (WORD)0x0200
#define S_ERROR_SWITCH  (WORD)0x0400
#define S_ERROR_VOLTAGE (WORD)0x0800
*/

#ifdef MH_DISPLAY_M8_DATA	// MHH:03/07/2019
	PRINTF("%8d%8d%8d%8d  0x%04x ",Log_drec_cnt,M8Data_in.ticks,M8Data_in.setspeed,M8Data_in.actualspeed,M8Data_in.sys_state);

	// Note: Consider displaying this data if gVerbose > 30

	int sys_state = M8Data_in.sys_state;
	if(sys_state & S_RUN_FINE) PC_puts("RunFine ");
	if(sys_state & S_RUN_COARSE) PC_puts("RunCoarse ");
	if(sys_state & S_RUN_FEATHER) PC_puts("RunFeather ");

	if(sys_state & S_STOP_FINE) PC_puts("StopFine ");
	if(sys_state & S_STOP_COARSE) PC_puts("StopCoarse ");
	if(sys_state & S_STOP_FEATHER) PC_puts("StopFeather ");

	if(sys_state & S_ERROR_OPEN) PC_puts("ErrorOpen ");
	if(sys_state & S_ERROR_CURRENT) PC_puts("ErrorCurrent ");

	PC_puts("\r\n");
//	PRINTF_FLUSH;

	if(Display_lcnt++ > 40)
	{
		Display_lcnt=0;
		if(PC_ReturnToContinue() == 'X') return true;
	}
#endif
	return false;
}
//---------------------------------------------------------------------------------------------------------------------
int Log_get_next_data(uint8_t ib)
{
//	static uint8_t Log_count_no_lticks;
	int cnt=0;
	uint8_t b = ib;
	uint8_t first_b = b;
	uint8_t *pdata = (uint8_t *)&M8Data_in;
	uint8_t offset;
	uint8_t last_offset=0;

	if(gVerbose > 20)
	{
		PC_putc('{');
	}
	for(;;)
	{
		if(++cnt > M8_SIZE)
		{
			Log_error("Log_get_next_data: cnt>32");
			return -1;
		}
		offset = (b & 0x7f);
		if(offset <= last_offset)	// Should never have a duplicate
		{
			Log_error("Log_get_next_data: offset <= last_offset");
			return -1;
		}
		last_offset = offset;

		if(offset > M8_SIZE)
		{
			Log_error("Log_get_next_data: offset too high");
			return -1;
		}
		uint8_t c       = Log_dup_getc();
		uint8_t c_old   = pdata[offset-1];	// This check to make sure working OK.
		if(c == c_old)
		{
			Log_error("Log_get_next_data: c==c_old");
			return -1;
		}
		pdata[offset-1] = c;		// Because offset has 1 added
		if(gVerbose > 20)
		{
			if(cnt > 1) PC_putc(',');
			PRINTF("%d:%d",offset-1,c);
		}
		if(b & M8_HIBIT) break;		// This is last offset,byte pair for tick
		b = Log_dup_getc();			// Get next offset,byte pair
	}
	if(gVerbose > 20)
	{
		Display_lcnt++;
		PC_puts("}\r\n");
	}

// Now check to see if tick data was sent through
// Note: We will be checking that there is at least tick written per 10 drecs
//	int x=1;			// This is a reminder to implement 1 tick per 10 drec check

	offset = (first_b & 0x7f);
	if(offset > 4)
	{
		M8Data_in.ticks++;			// No, so increment ticks
		if(Log_count_no_lticks++ > 10)
		{
			Log_error("Log_get_next_data: Log_count_no_lticks > 10");
			return -1;
		}
	}
	else
	{
		Log_count_no_lticks = 0;
		if(M8Data_in.ticks <= Log_check_lticks)
		{
			Log_error("Log_get_next_data: ticks <= Log_check_lticks");
			return -1;
		}
	}
	Log_check_lticks = M8Data_in.ticks;
	if(gVerbose > 10)
	{
		if(Display_M8Data_in()) return -1;
	}
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------
void AC210_ee_duplicate(WORD value)
{
	Log_duplicate = true;
	AC210_watchdog_active = false;				// MHH:04/08/2023
	AC210_ee_verify(value);
	AC210_watchdog_active = true;				// MHH:04/08/2023
	Log_duplicate = false;
}
//---------------------------------------------------------------------------------------------------------------------
void AC210_ee_verify(WORD value)
{
	td_Stats d_r;
	td_Stats d_r2;
	char str[50];
	char *func = "verify";
//	if(Log_duplicate) func = "duplicate";
	if(Log_duplicate)	// MHH:04/08/2023
	{
		func = "duplicate";
		Diags_new_run();
	}


	int run = value;
	if(Log_load_diags_rec(run,&d_r))
	{
		return;
	}
	Log_putc_write_buffer();		// flush, otherwise cannot see latest

#ifdef MH_CHECKPOINT_ZERO
	M8_zero_data();
#endif
//	ee_serial_read_ix = EE_SER_BUFF_SIZE;	// force read
//	AC210_log.getc_pos = Log_index.log_start_pos;


	ee_getc_position(Log_index.log_start_pos);

//	AC210_log.getc_pos2 = AC210_log.getc_pos;

	bool eof=false;
	ee_getc_cnt = 0;
//	Log_index_filesize -=4;

	uint8_t b1,b2,b3;
	Log_drec_cnt = 0;
	while(eof == false)
	{
#ifdef MH_XXX
		if(Log_duplicate)	// This logic because we do not want to start copying Diagnostic escape sequence
		{
			if(ee_getc_cnt >= Log_index_filesize)
			{
				PRINTF("EOF\r\n");
				break;
			}
		}
#endif
		AC210_log.getc_pos2_save = AC210_log.getc_pos2;
		PRINTF_FLUSH;		// Wait for buffer to be output

		b1 = ee_getc();
//		uint8_t b = Log_dup_getc();
		if(b1 == 0)
		{
			sprintf(str,"AC210_ee_%s:b=0",func);
			DebugAbort(str);
			return;
		}

		if(b1 == FLASH_EOF)
		{
			break;			// Could double check by seeing if next few in sequence are same
		}
		if(b1 == EELOG_C_ESCAPE1)
		{
			b2 = ee_getc();
//			uint8_t c = Log_dup_getc();
			if(b2 != EELOG_C_ESCAPE2)
			{
				sprintf(str,"AC210_ee_%s:Expecting ESCAPE2",func);
				DebugAbort(str);
			}
			b3 = ee_getc();
//			c = Log_dup_getc();
			char *desc;
			switch (b3)
			{
			case EELOG_C_CHECKPOINT_START:
			case EELOG_C_CHECKPOINT_TIMEOUT:
				desc = "START";
				if(b3 == EELOG_C_CHECKPOINT_TIMEOUT)	desc="TIMEOUT";
				if(ee_get_checkpoint_data())
				{
					return;				// Here if error. Could ignore and scan for next?
				}
				// get run number

				PRINTF("CHECKPOINT_%s\r\n",desc);
				if(Log_checkpoint.run != run)	// Log data has corrupt area where run_num goes from 49 back to 48 then up again
				{
					PRINTF("Checkpoint run:%d\r\n",Log_checkpoint.run);
				}
				if(gVerbose > 10)
				{
					if(Display_M8Data_in()) return;
				}

				if(Log_duplicate)
				{
					Log_checkpoint.run = Stat_Rec.run_number;	// because we have to add duplicate to newest log.
//					Log_checkpoint.head_wrap_cnt = LogCtl.head_wrap_cnt;	// MHH:11/12/2023
#ifdef MH_YYY
					if(Stat_Rec.head_wrap_cnt == 0)
					{
						Stat_Rec.head_wrap_cnt = LogCtl.head_wrap_cnt;	// This will be used for index entry
					}
#endif
					Checkpoint_put_data(b3);
				}
				continue;

			case EELOG_C_CHECKPOINT_DIAGS:
				PRINTF("CHECKPOINT_DIAGS\r\n");
				uint8_t c = ee_getc();
				if(c != EELOG_C_ESCAPE3)
				{
					sprintf(str,"AC210_ee_%s:Expecting ESCAPE3",func);
					DebugAbort(str);
					return;
				}
				// Diags Rec should follow
				uint32_t diags_pos = AC210_log.getc_pos2;
#ifdef MH_XXX

				if(Log_peek4() == PARAM_CHECK_CODE)	// MHH:18/04/2026. Param rec?
				{
					AC210_log.getc_pos2 += 256;	// Yes
				}
#endif

				if(Log_read_diag_rec(&d_r2))
				{
					return;
				}
				if(diags_pos != Log_index.log_diag_pos)
				{
					PRINTF("diags_pos:%d != Log_index.log_diags_pos:%d\r\n",diags_pos,Log_index.log_diag_pos);
				}
				else
				{
					PRINTF("diag_pos OK\r\n");
				}
				eof = true;		// We really want to exit main loop
				continue;

			default:
				sprintf(str,"AC210_ee_%s:Expecting Checkpoint",func);
				DebugAbort(str);
				return;
			}
		}

// Looks like data if we get to here. Keep getting data until we get an offset with hi-bit set
		if(Log_duplicate)
		{
			Log_putc(b1);
		}
		Log_drec_cnt++;
		if(Log_get_next_data(b1))
		{
			return;
		}
	}
	if(Log_duplicate)
	{
		Log_putc_write_buffer();		// flush
		Log_add_to_stat_rec(&d_r);
	}
	PRINTF("%s finished\r\n",func);
}
//---------------------------------------------------------------------------------------------------------------------
int eelog_read_diag_rec(td_Stats *p_dr,long pos)
{
	// Read on a byte by byte basis in case we wrap around at end of memory
//	printf("Reading Diags rec from position:%ld\r\n",pos);

//	ee_serial_read_ix = EE_SER_BUFF_SIZE;				// force read
//	AC210_log.getc_pos = pos;

	ee_getc_position(pos);
	if(Log_read_diag_rec(p_dr))
	{
		return 1;
	}
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------
//WORD NCV_word;
int eelog_read_param_rec(uint8_t *p_buff,int diags_pos,int run)
{
	// Read on a byte by byte basis in case we wrap around at end of memory
//	printf("Reading Diags rec from position:%ld\r\n",pos);

//	ee_serial_read_ix = EE_SER_BUFF_SIZE;				// force read
//	AC210_log.getc_pos = pos;

	ee_getc_position(diags_pos+256);		//Because param_rec written directly after diags_rec.


#ifdef MH_XXX

	uint8_t *pdst = p_buff;
//	int len = LOG_CHECKPOINT_SIZE + 4;


	int len = LOG_CHECKPOINT_SIZE + 3;


	for(int i=0;i<len;i++) *pdst++ = ee_getc();

	if(p_buff[0] != EELOG_C_ESCAPE1) return 0;
	if(p_buff[1] != EELOG_C_ESCAPE2) return 0;
	if(p_buff[2] != EELOG_C_CHECKPOINT_START) return 0;
	if(Get_uint16(p_buff+3) != run) return 0;
	if(p_buff[6] != 'X') return 0;		// Extended?

	// OK, looks valid, lets read it.

#endif


	uint8_t *pdst = p_buff;
	for(int i=0;i<256;i++) *pdst++ = ee_getc();

	ParamStore *pps = (ParamStore *)p_buff;;

	if(pps->checkCode != PARAM_CHECK_CODE) return 0;

//	NCV_word = pps->parms[CTL_RPM_PER_TICK_X10];

	//	if(Get_uint32(p_buff) != PARAM_CHECK_CODE) return 0;
//	uint8_t c = ee_getc();
//	if(c != EELOG_C_ESCAPE3) return 0;

	return 256;
}

//---------------------------------------------------------------------------------------------
int eelog_read_checkpoint_rec(uint8_t type,long pos)
{
//	ee_serial_read_ix = EE_SER_BUFF_SIZE;				// force read
//	AC210_log.getc_pos = pos;

	ee_getc_position(pos);
	uint8_t c = ee_getc();
	if(c != EELOG_C_ESCAPE1)
	{
		return -1;
	}
	c = ee_getc();
	if(c != EELOG_C_ESCAPE2)
	{
		return -1;
	}
	c = ee_getc();
//	if(c != type)
	if(c != EELOG_C_CHECKPOINT_START && c != EELOG_C_CHECKPOINT_TIMEOUT)	// MHH:05/01/2026
	{
		return -1;
	}
	ee_checkpoint_get_data();
	c = ee_getc();
	if(c != EELOG_C_ESCAPE3)
	{
		return -1;
	}
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------
void AC210_eelog_history(WORD value)
{
	td_Stats d_r;
	int log_file_size;
	int run;

	AC210_watchdog_active = false;				// This could take a while

	int pages_used = LogCtl.head_page - LogCtl.tail_page;
	if(pages_used < 0) pages_used += EELOG_MAX_PAGES;
	d_r = Stat_Rec;	// MHH:11/11/2022, compiler reported d_r not initialised.
	PRINTF(":D:DiagsU={%d,%d}\r\n",pages_used,d_r.log_flags);

	int first_run = Stat_Rec.run_number - LOG_INDEX_MAX;
	first_run = MAX(1,first_run);
//	int dpos = AC210_log.putc_pos2;	// probably
	int dpos;

	Index_page = -1;	// In case of restore or reset


	Log_putc_write_buffer();	// MHH:10/12/2023. So we can use putc_pos, not putc_pos2
	int recs_sent = 0;
	for(run = Stat_Rec.run_number;run >= first_run;run--)
	{
		if(recs_sent >= 100) break;		// MHH:22/04/2026

		if(run == Stat_Rec.run_number)		// First time?
		{
			dpos = AC210_log.putc_pos;	// probably
			d_r = Stat_Rec;		// Get current Stat rec
		}
		else
		{
			Index_get_index(run);
			if(pIndex->run == LOG_INDEX_ERASED_RUN)
			{
				continue;		// ignore any erased runs.
			}

			dpos = pIndex->log_diag_pos;
			if(eelog_read_diag_rec(&d_r,dpos))
			{
				txDebug(":A:ERR:Invalid Check code\r\n");	// No, then error
				continue;		// error
			}
			uint32_t log_start_pos = pIndex->log_start_pos;			// MHH:05/01/2026
			if(eelog_read_checkpoint_rec(EELOG_C_CHECKPOINT_START,log_start_pos) != 0)	// Error reading start data?
			{
				if(d_r.run_number == 1 && log_start_pos == 0)	// MHH:04/02/2024
				{
					break;
				}
				txDebug(":A:ERR:Invalid Check code\r\n");	// No, then error
				continue;
			}
		}

		log_file_size = dpos - d_r.log_start_data_pos;
		if(log_file_size < 0) log_file_size += EELOG_SPACE;
		recs_sent++;
		if(Diags_version >= 108)		// Should check that we have RTC and it is working too.
		{
			ULONG date_v = 0;
			if(d_r.log_flags & DIAGS_LOG_FLAG_EXTENDED)
			{
				if(d_r.log_flags & DIAGS_LOG_FLAG_PC_DATE) date_v = d_r.pc_user_date;
				if(d_r.log_flags & DIAGS_LOG_FLAG_PARAMS) date_v = 	d_r.param_update_date * 10;
			}
#ifdef MH_XXX			// MHH:01/05/2026. Just leave dates for extended, otherwise gets complicated.
			else
			{
				if(d_r.pc_user_date)
				{
					date_v = d_r.pc_user_date;
					if(d_r.param_update_date == d_r.pc_user_date) date_v = d_r.param_update_date * 10;		// to distinguish
				}
			}
#endif
//			PRINTF(":D:DiagsH2={%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}\r\n",d_r.run_number,d_r.rtc_start_secs,d_r.run_secs,
				PRINTF(":D:DiagsH2={%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}\r\n",d_r.run_number,date_v,d_r.run_secs,
					d_r.run.rpmSecs.tot,d_r.run.motorSecs.tot,
					log_file_size,
					d_r.run_errs.overcurrent_count,d_r.run_errs.opencircuit_count,
					d_r.run_errs.watchdog_restarts,d_r.run_errs.abort_restarts);
		}
		else
		{
			PRINTF(":D:DiagsH={%d,%d,%d,%d,%d,%d,%d,%d}\r\n",d_r.run_number,d_r.run.rpmSecs.tot,d_r.run.motorSecs.tot,
					log_file_size,
					d_r.run_errs.overcurrent_count,d_r.run_errs.opencircuit_count,
					d_r.run_errs.watchdog_restarts,d_r.run_errs.abort_restarts);
		}

//	    txDebug(DiagLine);

	}
    txDebug(":A:EOF\r\n");
}
//------------------------------------------------------------------------------------------------------------------
int AC210_current_log_filesize(int ifrom)
{
//	Log_putc_write_buffer();			// MHH:24/12/2023. To be sure, won't take time if already flushed.
	if(ifrom != 1)
	{
		if(LogData.enabled == false) return 0;
	}

	int log_filesize = AC210_log.putc_pos - Stat_Rec.log_start_data_pos;
	if(LogCtl.tail_page == 0)		// No wrap yet
	{
		if(AC210_log.putc_pos < Stat_Rec.log_start_data_pos)
		{
			L2PRINTF("AC210_current_log_filesize(%d):putc_pos (%d) < log_start_data_pos (%d)\r\n",ifrom,AC210_log.putc_pos,Stat_Rec.log_start_data_pos);
			Disable_log_data(123);
			return -1;
		}
	}
	if(log_filesize < 0)	// Wrap?
	{
		log_filesize += FLASH_PART_SIZE;
	}
#ifdef MH_YYY
	if(log_filesize > 5000000)
	{
		L2PRINTF("AC210_current_log_filesize:putc_pos (%d) < log_start_data_pos (%d)\r\n",AC210_log.putc_pos,Stat_Rec.log_start_data_pos);
		Disable_log_data(123);
		return 0;
	}
#endif
	return log_filesize;
}
//------------------------------------------------------------------------------------------------------------------
void AC210_eelog_send_diag_rec(WORD value)
{
	td_Stats d_r;
	int run = value;
	int log_filesize;
//	int diags_pos = -1;
//	int diags_pos2 = -1;
//	int send_len = 1024;

	if(run == Stat_Rec.run_number)	// Current Rec?
	{
		d_r = Stat_Rec;
		log_filesize = AC210_log.putc_pos - Stat_Rec.log_start_data_pos;
		if(log_filesize < 0) log_filesize += EELOG_SPACE;
		d_r.log_filesize = log_filesize;
		Diags_update_rec();
		Diags_send_rec_all();
		return;
	}
	if(Log_read_stat_index(run))
	{
	    txDebug(":A:ERR DiagIndex read fail\r\n");
		return;
	}
	int diags_pos = Log_index.log_diag_pos;
//	int data_pos = Log_index.log_start_pos;

	if(diags_pos < 0)
	{
	    txDebug(":A:ERR DiagPos < 0\r\n");
		return;		// error
	}
//	send_len = DIAGS_LONG_LEN;
	if(eelog_read_diag_rec(&d_r,diags_pos))
	{
	    txDebug(":A:ERR DiagRec invalid\r\n");
		return;		// error
	}
//		ee_getc_position(save_pos);
	log_filesize = Log_index.log_diag_pos - Log_index.log_start_pos;
	if(log_filesize < 0) log_filesize += EELOG_SPACE;
	d_r.log_filesize = log_filesize;
	memcpy(UU_fBuff,(uint8_t *)&d_r,DIAGS_LONG_LEN);

	Diags_last_drec_pos = diags_pos;		// Note may be simpler to just remember run, and load data through index.
	Diags_last_d_r_sent = d_r;		// Useful to get log info

//	Diags_send_rec(&d_r,diags_pos,send_len);

	if((d_r.log_flags & DIAGS_LOG_FLAG_EXTENDED) == 0)
	{
		Diags_uuencode_send_buff(UU_fBuff,DIAGS_LONG_LEN,DIAGS_NO_PREFIX);
		return;
	}
	if(eelog_read_param_rec(UU_fBuff+256,Log_index.log_diag_pos,run) != 256)
	{
		Diags_uuencode_send_buff(UU_fBuff,DIAGS_LONG_LEN,DIAGS_NO_PREFIX);
		txDebug(":A:ERR Param invalid\r\n");
		return;		// error
	}

	int next_diag_pos = -1;
	for(int i=0;i<10;i++)
	{
		run--;
		if(run == 0) break;
		if(Log_read_stat_index(run)) continue;	// Ignore invalid entries

		if(eelog_read_param_rec(UU_fBuff+512,Log_index.log_diag_pos,run) == 256)
		{
			next_diag_pos = Log_index.log_diag_pos;
			break;
		}
	}
	if(next_diag_pos < 0)
	{
		Diags_uuencode_send_buff(UU_fBuff,512,DIAGS_NO_PREFIX);
		txDebug(":A:ERR Param2 invalid\r\n");
		return;		// error
	}
	Diags_uuencode_send_buff(UU_fBuff,768,DIAGS_NO_PREFIX);	// Format Diags rec,old param rec, new param rec
}
//------------------------------------------------------------------------------------------------------------------
void AC210_log_check_erase_sector(void)
{
	if(AC210_log.flash_erase_sector != 0)
	{
		if(AC210_ssp_flash_erase_if_ready(AC210_log.flash_erase_sector))
		{
			LogCtl.last_flash_sector_erased = AC210_log.flash_erase_sector;
/*
 *
 * 		MHH:10/12/2023
 * 		If LogCtl.tail_page is inside the sector we are erasing then increment tail page by pages in sector (64).
 *
 * 		This way the tail page is one ahead of erased pages, and it handles the initial situation where tail page is zero
 * 		and erased sectors are ahead of it (and LogCtl.head_page)
 *
 * 			Eg, erasing sector 1, first_page = 0
 * 			                   2             = 64
 * 			                   255           = 16,320
 */

			int first_page_in_erased_sector = (LogCtl.last_flash_sector_erased - 1) * FLASH_LOG_PAGES_PER_SECTOR;
			if(LogCtl.tail_page == first_page_in_erased_sector)
			{
				LogCtl.tail_page += FLASH_LOG_PAGES_PER_SECTOR;
				LogCtl.tail_page %= FLASH_PART_PAGES;		// In case wrap
			}
			ee_update_logctl();
			AC210_log.flash_erase_sector = 0;
			L2PRINTF("AC210_log_check_erase_sector:Erased sector:%d, LogCtl.tail_page = %d\r\n",LogCtl.last_flash_sector_erased,LogCtl.tail_page);
			// Could set a flag to update Stat_Rec here. Will be updated within 60 secs anyway.
		}
	}
}
//-----------------------------------------------------------------------------------------
static int EE_erase_index_and_logctl(void)
{
	LPRINTF("EE_erase_index_and_logctl:\r\n");

	// The plan is to set all bytes from 512 to the end of the index to 255.
	// Hopefully this will show which bytes we have written to.

	memset(Index_page_buff,255,INDEX_PAGE_SIZE);
	p_eeprom_i2c_pos(EE_LOGCTL_POS);		// 512

/*
#define LOG_INDEX_OFFSET	1024
#define LOG_INDEX_MAX		1000		// Arbitrary
#define LOG_INDEX_ELE_SIZE	(sizeof(LOG_INDEX_T))
#define LOG_INDEX_MAX_POS	(LOG_INDEX_OFFSET + LOG_INDEX_MAX*LOG_INDEX_ELE_SIZE)
#define INDEX_PAGE_MAX		(INDEX_PAGE_SIZE/LOG_INDEX_ELE_SIZE)
*/

#ifdef MH_ERASE_ENTIRE_INDEX
	for(int pos = EE_LOGCTL_POS;pos < LOG_INDEX_MAX_POS;pos+=INDEX_PAGE_SIZE)
	{
		if(p_eeprom_i2c_write(Index_page_buff, INDEX_PAGE_SIZE)) {
			LogData.enabled = false;	// Turn logging off
			DebugAbort("EE_erase_index_and_logctl:i2c_write fail");
			return -1;
		}
	}
#else
	// Just erase first 512 bytes, should not be necessary to erase everything if we add index entry when new run

//	if(p_eeprom_i2c_write(Index_page_buff, INDEX_PAGE_SIZE)) {
	if(p_eeprom_i2c_write(Index_page_buff, 32))	// MHH:09/08/2023. Don't erase PLOG!!!
	{
		Disable_log_data(400);
//		LogData.enabled = false;	// Turn logging off
		DebugAbort("EE_erase_index_and_logctl:i2c_write fail");
		return -1;
	}
#endif
	return 0;
}
//--------------------------------------------------------------------------------------------------------------
// Save first 2 pages of EEPROM and first 50 pages of Flash in Flash sector zero
void AC210_save_flash(void)
{
	AC210_watchdog_active = false;				// This could take a while
	PRINTF("AC210_save_flash:\r\n");
	PRINTF("Erasing sector zero..\r\n");
	AC210_ssp_flash_erase(0);
	PRINTF("erased\r\n");

	PRINTF("Copying first 2 pages of EEPROM to Flash sector zero..\r\n");
//	uint32_t file_pages = LogCtl.head_page;

	p_eeprom_i2c_pos(EELOG_PARAM_POS);
	int flash_dst_pos = 0;

	for(int epage=0;epage<2;epage++)
	{
		if(p_eeprom_i2c_read(UU_fBuff, EELOG_PAGESIZE))
		{
		    txDebug(":A:ERR\r\n");
			DebugAbort("i2c_read");
			return;
		}
		AC210_ssp_raw_flash_write(flash_dst_pos,UU_fBuff,EELOG_PAGESIZE);
		flash_dst_pos += EELOG_PAGESIZE;
	}
	PRINTF("done\r\n");
	PRINTF("LogCtl.head_page: %d\r\n",LogCtl.head_page);

	int flash_src_pos = FLASH_PART_OFFSET;
	PRINTF("Now copying 50 pages from Flash, position:%d to position:%d\r\n",flash_src_pos,flash_dst_pos);
	for(int fpage=0;fpage<50;fpage++)
	{
		AC210_ssp_flash_read(flash_src_pos,UU_fBuff,EELOG_PAGESIZE);
		flash_src_pos += EELOG_PAGESIZE;
		AC210_ssp_raw_flash_write(flash_dst_pos,UU_fBuff,EELOG_PAGESIZE);
		flash_dst_pos += EELOG_PAGESIZE;
	}
	PRINTF("Finished\r\n");
}
//--------------------------------------------------------------------------------------------------------------
// Reverse of AC210_save_flash()

void AC210_restore_flash(void)
{
	AC210_watchdog_active = false;				// This could take a while
	PRINTF("AC210_restore_flash:\r\n");

	PRINTF("Erasing sector one..\r\n");
	AC210_ssp_flash_erase(1);
	PRINTF("erased\r\n");

	PRINTF("Copying first 2 pages from Flash sector zero to EEPROM ..\r\n");
//	uint32_t file_pages = LogCtl.head_page;

	p_eeprom_i2c_pos(EELOG_PARAM_POS);
	int flash_src_pos = 0;

	for(int epage=0;epage<2;epage++)
	{
		AC210_ssp_flash_read(flash_src_pos,UU_fBuff,EELOG_PAGESIZE);
		flash_src_pos += EELOG_PAGESIZE;
		if(p_eeprom_i2c_write(UU_fBuff, EELOG_PAGESIZE))
		{
		    txDebug(":A:ERR\r\n");
			DebugAbort("i2c_write");
			return;
		}
	}
	PRINTF("done\r\n");

	int flash_dst_pos = FLASH_PART_OFFSET;
	PRINTF("Now copying 50 pages from Flash sector zero, position:%d to position:%d\r\n",flash_src_pos,flash_dst_pos);
	for(int fpage=0;fpage<50;fpage++)
	{
		AC210_ssp_flash_read(flash_src_pos,UU_fBuff,EELOG_PAGESIZE);
		flash_src_pos += EELOG_PAGESIZE;
		AC210_ssp_raw_flash_write(flash_dst_pos,UU_fBuff,EELOG_PAGESIZE);
		flash_dst_pos += EELOG_PAGESIZE;
	}

	wait_ms(100);
	PRINTF("Rebooting...\r\n");
	PRINTF_FLUSH;
	wait_ms(100);
	AC210_reboot();
	//	NVIC_SystemReset();			// Should not get passed here

//	PRINTF("Finished\r\n");
}
//==================================================================================================================
//#define MH_FLASH_WRAP_TEST
#ifdef MH_FLASH_WRAP_TEST
static int save_n;
static uint8_t TestWriteBuffer[12],TestReadBuffer[50];
void AC210_Flash_Wrap_Test(WORD val)
{
	AC210_watchdog_active = false;				// This could take a while

	printf("SSP_Flash_Wrap_Test:\n\r");

	uint32_t n;

	if(val == 0)
	{
		eeprom_log_reset(997);


		uint32_t t1 = us_ticker_read();

		for(n=1;;n++)

		{
			sprintf((char *)TestWriteBuffer,"%11d",n);
			int len=0;
			for(int i=0;;i++)
			{
				uint8_t c = TestWriteBuffer[i];
				if(c == 0) break;
				Log_putc(c);
				len++;
			}
	#ifdef MH_DISPLAY_WRAP
			if(LogCtl.head_page == 0 && n > 100000)
			{
				printf("n = %d at wrap\r\n",n);
			    AC210_uart3_wait();		// Wait for buffer to be output
			}
	#endif
			if(len != 11)
			{
				DebugAbort("len != 11");
				return;
			}
			if(save_n == 0 && LogCtl.head_page == 128)
			{
				save_n = n;
				printf("save_n = %d\r\n",save_n);
			}
			if(n % 10000 == 0)
			{
				printf("n = %d, head_page = %d, tail_page = %d, last_sector_erased = %d\r\n"
						,n,LogCtl.head_page,LogCtl.tail_page,LogCtl.last_flash_sector_erased);
				   AC210_uart3_wait();		// Wait for buffer to be output
				   if(n == 1680000) break;
//				   if(LogCtl.tail_page > 0 && LogCtl.head_page > 50) break;
			}
		}
		Log_putc_write_buffer();		// Flush buffer

		uint32_t t2 = us_ticker_read();
		uint32_t elapsed_us = t2 - t1;
		printf("\r\nFinished writing, n = %d, elapsed = %d (ms)\r\n",n,elapsed_us/1000);
	//	ReturnToContinue();

	}
	else
	{
		n = 1680000;
		ee_read_logctl();
	}
	printf("Now checking\r\n");

	AC210_log.getc_pos  = LogCtl.tail_page * EELOG_PAGESIZE;
	AC210_log.getc_pos2 = AC210_log.getc_pos;
	ee_serial_read_ix = EE_SER_BUFF_SIZE;		// force read
	for(int i=0;i<10;i++)
	{
		TestReadBuffer[i] = ee_getc();
	}
	printf("%s\r\n",TestReadBuffer);
	ReturnToContinue();

	int n2 = atoi((char *)TestReadBuffer);

	int last_n2 = n2;
	printf("n2 = %d\r\n",n2);
	ReturnToContinue();
	for(int i=0;;i++)
	{

#ifdef MH_XXX
		if(last_n2 >= 1679987)
		{
			printf("n2 = %d\r\n",n2);
			ReturnToContinue();
		}
#endif
		for(int j=0;j<11;j++)
		{
			TestReadBuffer[j] = ee_getc();
		}
		n2 = atoi((char *)TestReadBuffer);
		if(n2 != last_n2 + 1)
		{
			printf("Error:n2 = %d,last_n2 = %d\r\n",n2,last_n2);
			ReturnToContinue();
		}
		last_n2 = n2;
		if(n2 % 10000 == 0)
		{
			printf("n2:%d, AC210_log.getc_pos:%d, AC210_log.getc_pos2:%d\r\n",n2,AC210_log.getc_pos, AC210_log.getc_pos2);
		    AC210_uart3_wait();		// Wait for buffer to be output
		}
		if(n2 == n) break;
	}
	printf("Found n!!\r\n");
	ReturnToContinue();

}
#else
void AC210_Flash_Wrap_Test(WORD val)
{
	PRINTF("AC210_Flash_Wrap_Test:Dummy routine\r\n");
}
#endif
//----------------------------------------------------------------------------------------------------
td_PLOG_HEAD Plog_head;
#define PLOG_HEAD_MAGIC		123456789
const uint32_t UINT32_HIVAL=0xffffffff;
int ee_plog_read_head(void)
{
	uint8_t* p_buf = (uint8_t *)&Plog_head;
	p_eeprom_i2c_pos(EE_PLOG_HEAD_OFFSET);
	if(p_eeprom_i2c_read(p_buf,EE_PLOG_HEAD_LEN))
	{
		DebugAbort("ee_plog_read_head:eeprom_i2c_read");
		return -1;
	}
	return 0;
}

int ee_plog_write_head(void)
{
	if(Plog_head.magic != PLOG_HEAD_MAGIC)	// Corrupt?
	{
		Plog_head.enabled = false;
		return -1;
	}

	uint8_t* p_buf = (uint8_t *)&Plog_head;
	p_eeprom_i2c_pos(EE_PLOG_HEAD_OFFSET);
	if(p_eeprom_i2c_write(p_buf, EE_PLOG_HEAD_LEN))
	{
		Plog_head.enabled = false;
		DebugAbort("ee_plog_write_head:eeprom_i2c_write");
		return -1;
	}
	return 0;
}

int ee_plog_write_data(uint32_t data_pos,uint8_t *pdata,uint32_t len)
{
	uint32_t pos = EE_PLOG_DATA_OFFSET + data_pos;
	p_eeprom_i2c_pos(pos);
	if(p_eeprom_i2c_write(pdata, len))
	{
		Disable_log_data(500);
//		LogData.enabled = false;	// Turn logging off
		DebugAbort("ee_plog_write_data:eeprom_i2c_write");
		return -1;
	}
	return 0;
}
int ee_plog_new_head(void)
{
	Plog_head.magic = PLOG_HEAD_MAGIC;
	Plog_head.data_ptr = 0;
	Plog_head.data_check = -1;
	return ee_plog_write_head();
}
void AC210_plog_puts2(const char *s)
{
//	if(Plog_head.enabled == false) return;

	int s_len = strlen(s);

	uint32_t data_check_val = (Plog_head.data_ptr ^ Plog_head.data_check);
	if((data_check_val) != UINT32_HIVAL) return;	// Corrupt!!

	uint32_t data_offset = Plog_head.data_ptr % EE_PLOG_SIZE;	// ptr does not wrap, so find offset
	uint32_t data_len = s_len;
	uint8_t *byte_address = (uint8_t *)s;

	int end_p = data_offset + s_len - 1;
	if(end_p >= EE_PLOG_SIZE)		// handle possible wrap
	{
		end_p = EE_PLOG_SIZE - 1;
		data_len = EE_PLOG_SIZE - data_offset;
	}
	if(ee_plog_write_data(data_offset,byte_address,data_len))	// Update log
	{
		Plog_head.enabled = false;
		return;		// error
	}

	if(data_len < s_len)		// wrap?
	{
		data_offset = 0;
		byte_address += data_len;
		data_len = s_len - data_len;		// Write remaining bytes
		if(ee_plog_write_data(data_offset,byte_address,data_len))	// Update log
		{
			Plog_head.enabled = false;
			return;		// error
		}
	}
	Plog_head.data_ptr += s_len;
	Plog_head.data_check = Plog_head.data_ptr ^ UINT32_HIVAL;	// Idea is pos + check = -1
	ee_plog_write_head();
}
static uint8_t Plog_count;
void AC210_plog_puts(const char *s)
{

	if(Plog_head.enabled == false) return;

	char plog_line[160];

	uint32_t tsecs = TimeInSeconds;
//	uint32_t tsecs = Systick_secs;

	int ss = tsecs % 60;
	int tmins = tsecs / 60;
	int mm = tmins % 60;
	int hh = tmins / 60;

	char *p_line = plog_line;

	if(Plog_count == 0)
	{
		Plog_count++;
		*p_line++ = '\r';
		*p_line++ = '\n';
	}
	sprintf(p_line,"%02d:%02d:%02d %s",hh,mm,ss,s);
	AC210_plog_puts2(plog_line);
}

void AC210_plog_init(void)
{
	Plog_head.enabled = false;	// Turn print logging off.

	if(ee_plog_read_head() != 0) return;

	uint32_t data_check_val = (Plog_head.data_ptr ^ Plog_head.data_check);

	bool data_check = (data_check_val == UINT32_HIVAL);

	if(Plog_head.magic == PLOG_HEAD_MAGIC)
	{
		if(data_check)
		{
			Plog_head.enabled = true;
		}
	}
	else
	{
		if(data_check == false)
		{
			if(ee_plog_new_head() != 0) return;
			Plog_head.enabled = true;
		}
	}
}
#define PLOG_PAGESIZE	128
uint8_t Plog_buffer[PLOG_PAGESIZE];
int Plog_getc_pos;
int Plog_page = -1;	// Force read on page 0

int Plog_getpage(int page)
{
	Plog_page = page;
	uint32_t page_pos = EE_PLOG_DATA_OFFSET + (Plog_page * PLOG_PAGESIZE);

	uint8_t* p_buf = (uint8_t *)&Plog_buffer;
	p_eeprom_i2c_pos(page_pos);
	if(p_eeprom_i2c_read(p_buf,PLOG_PAGESIZE))
	{
		DebugAbort("Plog_get_page:eeprom_i2c_read");
		return -1;
	}
	return 0;
}

int Plog_getc(void)
{
	if(Plog_getc_pos >= EE_PLOG_SIZE) Plog_getc_pos = 0;	// Handle wrap
	if(Plog_getc_pos < 0) Plog_getc_pos = EE_PLOG_SIZE  + Plog_getc_pos;	// How can it be negative?

	int page = Plog_getc_pos / PLOG_PAGESIZE;
	int offset = Plog_getc_pos % PLOG_PAGESIZE;
	int rv;
	if(page != Plog_page)
	{
		rv = Plog_getpage(page);
		if(rv != 0) return rv;
	}
	rv = Plog_buffer[offset];
	Plog_getc_pos++;
	return rv;
}

void AC210_plog_display(int val)
{
	int c;
	if(val == 123)
	{
		PC_puts("Resetting plog file\r\n");
		ee_plog_new_head();
		return;
	}
	AC210_watchdog_active = false;				// MHH:12/08/2023.This could take a while
	if(val == 1)
	{
		PC_puts("{start}\r\n");
	}
	int plog_size;
	if(Plog_head.data_ptr < EE_PLOG_SIZE)
	{
		Plog_getc_pos = 0;
		plog_size = Plog_head.data_ptr;
	}
	else
	{
		Plog_getc_pos = Plog_head.data_ptr % EE_PLOG_SIZE;
		plog_size = EE_PLOG_SIZE;
	}

//	uint32_t imax = Plog_head.data_ptr % EE_PLOG_SIZE;	// This doesn't take into account wrap, just for test.
	for(int i=0;i<plog_size;i++)
	{
		c = Plog_getc();
//		if(c <= 0 || c >= 128) break;
		if(c < 0) break;
		PC_putc(c);
	}
	if(val == 1)
	{
		PC_puts("{end}\r\n");
	}
	AC210_watchdog_active = true;
}
