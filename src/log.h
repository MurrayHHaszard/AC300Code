/*
 * log.h
 *
 *  Created on: 14/02/2017
 *      Author: Murray
 */

#ifndef LOG_H_
#define LOG_H_
#include "global.h"

#define EELOG_C_ESCAPE1		254		// special characters.
#define EELOG_C_ESCAPE2		'{'
#define EELOG_C_ESCAPE3		'}'
#define EELOG_C_CHECKPOINT_START	1
#define EELOG_C_CHECKPOINT_TIMEOUT	2
#define EELOG_C_CHECKPOINT_DIAGS	3
//#define EELOG_C_CHECKPOINT_DIAGS_AND_PARAM	4		// MHH:17/04/2026
//#define EELOG_C_CHECKPOINT_EOF		4

//#define HI_VALS4		0xffffffff
#define EELOG_END_CHAIN		-1

#define EE_SER_BUFF_SIZE		128			// Must divide into page size with no remainder
#define EE_SER_WRITE_MAX		(EE_SER_BUFF_SIZE)
#define EE_SER_WRITE_TRIGGER	64			// Use this to ensure whole drecs are written

#define EELOG_CHECKPOINT_SECS	60			// one minute
#define EELOG_CHECKPOINT_TICKS	500			// 10 seconds

typedef struct {
uint32_t ticks;					// (0)
uint16_t actualspeed;			// (4)
uint16_t setspeed;				// (6)
uint16_t current;				// (8)
uint16_t sys_state;				// (10) includes sys_run, sys_stops, sys_errs
uint8_t op_mode;				// (12) Manual, takeoff, climb etc
uint8_t manual_keys;			// (13)
uint8_t xoar_status;			// (14)
uint8_t control_state:2;		// (15:0-1) Idle,F,C,Brake. May not be needed
uint8_t motor_drive_state:4;	// (15:2-5)
uint8_t fill:2;					// (15:6-7)
uint8_t pwm_val;				// (16)
uint8_t throttle;				// (17) 0 to 100 %
uint8_t voltage;				// (18)	MHH:29/10/2020.
//uint8_t slipring;				// (19) MHH:20/12/2020
uint8_t bl_dac0_v;				// (19) MHH:12/08/2025
int16_t bl_angle;				// (20) MHH:05/12/2022
uint8_t bl_voltage;				// (22)
uint8_t bl_temperature;			// (23)
uint8_t bl_line_err_code;		// (24)
//uint8_t bl_error;				// (25) Not used yet...
uint8_t bl_enc_rpm;				// (25) MHH:12/08/2025
//uint16_t deadband;				// (26)
uint8_t deadband;				// (26) MHH:04/02/2026
uint8_t ac_temperature;			// (27) MHH:04/02/2026
uint8_t bl_command;				// (28)	Note: bl_command and speed could be combined into 1 byte, if speed is a value between 0 and 9.
uint8_t bl_speed;				// (29)	 Also, could use pwm_val instead, as these fields are mutually exclusive. Maybe get working first
uint8_t bl_flags;				// (30)  MHH:27/07/2023. Eg NOT_READY_BIT
uint8_t rc_s_mode;				// (31)  MHH:10/06/2025
                                // (32)
} M8_T;

#define M8_SIZE		(sizeof(M8_T))	// should be 32.
#define M8_HIBIT	128


//----------------------------------------------------------------------------------
typedef struct
{
	bool putc_flush;
	bool enabled;
	bool first_checkpoint;
#ifdef MH_CHECKPOINT_ZERO
	bool zero_compare_data;
	bool zero_check_data;
#endif
	uint8_t no_tick_count;
	uint8_t serial_write_ix;
	uint8_t serial_flag;
	uint8_t map_byte;
//	uint8_t dflags;
	uint8_t checkpoint_secs;
	uint8_t ticks_per_log;		// number of ticks between logs. Zero means no logging. 50 means once a second.
	uint32_t cnt2;
	uint32_t log_ticks;		// May need to tie in with diagnostics?
}LogData_T;


typedef struct
{
	uint16_t run;
	uint8_t ticks_per_log;
	uint8_t extended;		// MHH:20/04/2026
	M8_T m8_data;
} Log_checkpoint_T;

#define LOG_CHECKPOINT_SIZE		(sizeof(Log_checkpoint_T))	// Should be 4 + 32 = 36

typedef struct
{
	int first_run;
	int last_run;
}AC210_range_T;

extern Log_checkpoint_T Log_checkpoint;
extern LogData_T LogData;
extern AC210_range_T AC210_range;
extern int16_t Engine_rpm_change_in_five_ticks;		// MHH:30/01/2026

//extern uint8_t ee_serial_write_ix;
//extern uint8_t ee_serial_flag;
extern uint8_t ee_serial_write_buff[EE_SER_BUFF_SIZE];


void Log_putc_write_buffer(void);
void Log_putc(uint8_t c);
void Log_put_nulls(int len);
int Log_pack_data(void);
void Log_put_data(uint8_t *p_data,int plen);
void Log_Checkpoint(uint8_t checkpoint_code);
void Log_serial_command(WORD ival);
void Log_change_rate(WORD value);

void Log_putc_write_buffer_flash(void);
void Log_update_flash_pos(void);
int ee_update_logctl(void);

void Log_putc_write_buffer_flash_2(void);

uint32_t Log_peek4(void);
uint32_t Log_peek4_pos(int pos);

#endif /* LOG_H_ */
