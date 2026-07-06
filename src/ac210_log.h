/*
 * ac210log.h
 *
 *  Created on: 13/12/2016
 *      Author: Murray
 */

#ifndef AC210LOG_H_
#define AC210LOG_H_

#include "diags.h"
#include "log.h"

#define EE_PLOG_DATA_OFFSET		32768		//(32 * 1024)
#define EE_PLOG_HEAD_OFFSET		(512 + 32)		// Next to LogCtl
#define EE_PLOG_HEAD_LEN	12			// We do not want to overwrite enabled
//#define EE_PLOG_SIZE			32768		// 32 pages
//#define EE_PLOG_SIZE			1024		// 1 page for testing
#define EE_PLOG_SIZE			(1024*16)	// 16 pages for now, we have up to 128K total space

typedef struct
{
	uint32_t magic;
	uint32_t data_ptr;
	uint32_t data_check;	// Idea is data_check + data_ptr = 0
	bool enabled;
} td_PLOG_HEAD;
extern td_PLOG_HEAD Plog_head;
void AC210_plog_init(void);


#define EE_DUMP5MB_OFFSET	(512 + 64)		// Next to PLOG_HEAD but only output to Diagnostics, not saved. Also, probably redundant now that tables are
											// built in Diagnostics.

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
extern td_DUMP5Mb Dump5Mb;

#define FLASH_SIZE 			(16 * 1024 * 1024)		// 16 megabytes
#define FLASH_PAGE_SIZE		1024
#define FLASH_SECTOR_SIZE	65536			// bytes
#define FLASH_SECTORS		256

#define FLASH_PART_SECTORS		255		// Normal
#define FLASH_PART_OFFSET		FLASH_SECTOR_SIZE						// 1 sector offset
//#define FLASH_PART_SIZE			(FLASH_SIZE - FLASH_PART_OFFSET)		// MHH:08/12/2023
#define FLASH_PART_SIZE			(FLASH_SECTOR_SIZE * FLASH_PART_SECTORS)
//#define FLASH_SPACE_MASK		0xffffff
//#define FLASH_PART_SECTORS		(FLASH_MAX_SECTORS-FLASH_SECTOR_OFFSET)
//#define FLASH_PART_SECTORS		3		// Test!!!
#define FLASH_PART_PAGES		(FLASH_PART_SIZE/FLASH_PAGE_SIZE)

#define EEPROM_RESET_SIZE		200
#define EELOG_MAGIC				247627837

#define EELOG_PARAM_POS			0
#define EELOG_PAGESIZE		FLASH_PAGE_SIZE		// logical page, when changes we update page field
#define EELOG_SPACE			FLASH_PART_SIZE
#define EELOG_MAX_PAGES		FLASH_PART_PAGES
#define EELOG_START_POS		FLASH_PART_OFFSET		// Start in sector 1
#define EEPROM_SIZE			FLASH_SIZE

#define FLASH_SECTOR_OFFSET	1				// Ignore sector zero as slow erase
//#define FLASH_MAX_SECTORS	256
#define FLASH_MAX_SECTORS	(FLASH_SIZE/FLASH_SECTOR_SIZE)	// Assume FLASH_SIZE is a multiple of sector_size
//#define FLASH_START			(FLASH_SECTOR_SIZE * FLASH_SECTOR_OFFSET)
#define FLASH_EOF			255
#define FLASH_EOF_COUNT		20
#define FLASH_LOG_PAGES_PER_SECTOR	(FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE)


//#define FLASH_MAX_LOG_PAGES		(FLASH_SIZE / EELOG_PAGESIZE)
//#define FLASH_PAGE_OFFSET	(FLASH_START/EELOG_PAGESIZE)
//#endif

//-------------------------------------------------------------------------
uint32_t AC210_get_eeprom_pos(void);
uint8_t ee_getc(void);
void ee_getc_position(int pos);
void ee_getc_save_position(void);
void ee_getc_restore_position();
void ee_checkpoint_get_data(void);

int ee_get_checkpoint_data(void);
int eelog_read_diag_rec(td_Stats *p_dr,long pos);
int eelog_read_checkpoint_rec(uint8_t type,long pos);

bool ee_check_all_255(uint8_t *buff,int len);

int Log_read_diag_rec(td_Stats *d_r);
int Log_get_next_data(uint8_t ib);

//-------------------------------------------------------------------------
#define LOGCTL_MAGIC			123454321
#define EE_LOGCTL_POS				512
#define LOGCTL_HEAD_OFFSET		4
#define LOGCTL_HEAD_SIZE		4
#define LOGCTL_REC_SIZE		(sizeof(td_LOGCTL))
typedef struct
{
	int magic;						// 0.
	int head_page;					// 4.
	int tail_page;					// 8. tail_page and last_flash_sector_erased probably should be updated together.
	int last_flash_sector_erased;	// 12.
	uint8_t fill[2];				// 16 MHH:11/12/2023. Think head_wrap_cnt and tail_wrap_count redundant.
	WORD run_number;				// 18. Same value as run_number in current stat_rec.
	int max_pages;					// 20.
} td_LOGCTL;						// 24
extern td_LOGCTL LogCtl;

//----------------------------------------------------------------------------------
typedef struct
{
	uint32_t getc_pos;		// last pos read from memory, can be ahead of pos2
	uint32_t getc_pos2;
	uint32_t getc_pos2_save;	// Used when displaying
	uint32_t putc_pos;		// last pos written to memory, can be behind pos2
	uint32_t putc_pos2;
//	uint32_t last_diags_pos;
//	uint32_t start_data_pos;
//	uint8_t head_wrap_cnt;
//	uint8_t fill[3];
	uint32_t eof_page;
	uint32_t flash_erase_sector;
	uint8_t *pD1;
	uint8_t *pD2;
}AC210_log_T;

extern uint8_t ee_serial_read_buff[EE_SER_BUFF_SIZE];
extern uint8_t ee_serial_read_ix;
extern int ee_getc_cnt;
extern AC210_log_T AC210_log;
extern uint8_t I2C_hex_buff[256];

#endif /* AC210LOG_H_ */
