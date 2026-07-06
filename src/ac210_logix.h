/*
 * ac210_logix.h
 *
 *  Created on: 13/04/2017
 *      Author: Murray
 */

#ifndef AC210_LOGIX_H_
#define AC210_LOGIX_H_

#include "ac210_log.h"
#include "diags.h"

#define LOG_INDEX_ERASED_RUN	65535
#define LOG_INDEX_ADJUST	1		// Will be changing to 1
#define EE_LOG_INDEX_OFFSET	1024
//#define LOG_INDEX_MAX		100		// Test!!!
#define LOG_INDEX_MAX		1000		// Arbitrary
#define LOG_INDEX_ELE_SIZE	(sizeof(LOG_INDEX_T))
#define LOG_INDEX_MAX_POS	(EE_LOG_INDEX_OFFSET + LOG_INDEX_MAX*LOG_INDEX_ELE_SIZE)
//#define INDEX_PAGE_MAX		(INDEX_PAGE_SIZE/LOG_INDEX_ELE_SIZE)
#define INDEX_PAGE_MAX		10
#define INDEX_PAGE_SIZE		(INDEX_PAGE_MAX*LOG_INDEX_ELE_SIZE)	// This way an integer number of elements fit into buffer

void Log_write_stat_index(td_Stats *p_DR,int log_diag_pos);
int Log_read_stat_index(int run);

int Index_get_index(int run);

void Disable_log_data(int ifrom);
void Logctl_display(td_LOGCTL *pLog_ctl);
void Logctl_display2(void);

void AC210_index_verify(void);
void AC210_data_verify(void);

typedef struct
{
	uint16_t run;
//	uint8_t head_wrap_cnt;	// MHH:11/12/2023
	uint8_t fill;
	uint8_t fill2;		// Note: This could be used for flags. Eg Data corrupt, Diags Corrupt, maybe Data overwritten?
	int log_start_pos;
	int log_diag_pos;
} LOG_INDEX_T;

extern int Index_page;
extern int Log_drec_cnt;
extern LOG_INDEX_T *pIndex;
extern LOG_INDEX_T Log_index;
extern uint8_t Index_page_buff[INDEX_PAGE_SIZE];

#endif /* AC210_LOGIX_H_ */
