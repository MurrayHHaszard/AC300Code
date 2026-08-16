/*
 * ac210_sig60.c
 *
 *  Created on: 15/12/2017
 *      Author: Murray
 */

#include "ac210_sig100.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>		// for abs()

#include "board.h"

// TODO: insert other include files here
#include "ac210_global.h"
#include "ac210_pins.h"
#include "analog.h"
#include "control.h"
#include "digital.h"
#include "leds.h"
#include "param.h"
#include "sstate.h"
#include "feather.h"
#include "drive.h"
#include "remote.h"
#include "diags.h"

#define PC_PUTS(a)			PC_puts(a)
#define PC_PUTC(a)			PC_putc(a)
//#define PC_PRINTF(...) {p_BufferLen = sprintf(p_Buffer,__VA_ARGS__);Board_UART0PutSTR(p_Buffer);}
#define PC_PRINTF(...) {sprintf(p_Buffer,__VA_ARGS__);Board_UART0PutSTR(p_Buffer);}	// MHH:10/01/2026

Enc_data_td Enc_data;

//===========================================================================================================
// From mbed geometry.cpp '5'

#define PI 3.14159265


//WORD Enc_motor_gear_ratio=231;		// Will need to get this from param

//=============================================================================================================
/* The 'top angle' of 30 degrees is at the top of the pivot, so it is convenient to make all
    calculations relative to that point.

    The formula is: Angle = 30 + (arc-sine(distance from pivot-point/CAM_RADIUS))

   Probably simplest to define 30 degrees as position zero, everything else relative.XSIG100

   Note that distance can be plus or minus relative to position zero, so the angle would be plus or minus from 30 degrees.
*/
//-------------------------------------------------------------------------
float tan_d(float angle_in_degrees)
{
    float angle_in_radians = angle_in_degrees * PI / 180.0;
    return tan(angle_in_radians);
}
float SIG100_get_hub_voltage(void)	// MHH:25/06/2025
{
	float fvoltage = (float)(ADC_adjusted_voltage * 16);
//	fvoltage /= 10.954;		// By experiment, should have an implied decimal place
	if(fvoltage < 1800.0F)
	{
		fvoltage *= 0.9F;		// MHH:16/10/2025. By experiment, should have an implied decimal place
	}
	if(Enc_data.voltage_correct_pct == 0) Enc_data.voltage_correct_pct = 100;	// Set default
	fvoltage  = (fvoltage * Enc_data.voltage_correct_pct)/100;
//		fvoltage /= 10.954;		// By experiment, should have an implied decimal place
//	ADC_BL_corrected_voltage = (uint16_t)fvoltage;
	fvoltage /= 10.0F;		// MHH:16/10/2025.
	return fvoltage;
}
uint16_t Encoder_target_pos;
bool Manual_position_encoder=false;
extern int Prop_rpm;
uint8_t Manual_position_count;
int SIG100_get_position_len(uint16_t len_in_mm_2dec)
{
	float pos_mm = ((float)len_in_mm_2dec)/ 100;
	int encoder_pos = (int)((pos_mm/Hub_calc.hall_incr_mm)+0.5F);
//	if(encoder_pos > Enc_data.cal_fine_hard_stop) encoder_pos = Enc_data.cal_fine_hard_stop;
	if(encoder_pos > Enc_data.ud_reverse_stop) encoder_pos = Enc_data.ud_reverse_stop;
	return encoder_pos;
}
int SIG100_set_position_encoder(int ienc)
{
	Encoder_target_pos = ienc;
//	if(Encoder_target_pos > Enc_data.ud_reverse_stop) Encoder_target_pos = Enc_data.ud_reverse_stop;
//	if(Encoder_target_pos < Enc_data.ud_feather_stop) Encoder_target_pos = Enc_data.ud_feather_stop;;

	Manual_position_encoder = true;
	Manual_position_count = 0;
	return Encoder_target_pos;

}
int SIG100_set_position_len(uint16_t len_in_mm_2dec)
{
	Manual_position_encoder = false;
	if(operatingMode() != MANUAL) return 0;	// May have to allow other possibility for RC.
	if(Prop_rpm > 0) return 0;

	float pos_mm = ((float)len_in_mm_2dec)/ 100;
	int epos = (int)((pos_mm/Hub_calc.hall_incr_mm)+0.5F);
	return SIG100_set_position_encoder(epos);
}
//------------------------------------------------------------------------------
#ifdef MH_XXX	// MHH:28/04/2025
int iround(float f)
{
	int rv;
	if(f >= 0)
	{
		rv = (int)(f + 0.5);
	}
	else
	{
		rv = (int)(f - 0.5);
	}
	return rv;
}
#endif
//================================================================================
struct HUB_CALC Hub_calc;
int Encoder_Pos;
CAL_state Hub_calibrate_state;
int Hub_calibrate_checkval;	// Safety
uint8_t Hub_calibrate_pole_pairs;	// Because we need to know when calibrating for speed control
uint8_t Hub_calibrate_type;			// MHH:25/08/2023. May be useful for "manual" calibration.
int Hub_calibrate_last_ms_stop_pos;
uint8_t Cal_hardstop_count;
bool Cal_hardstop_sent;
uint16_t Cal_fine_pos;
uint16_t Cal_fine_pos2;
uint16_t Cal_coarse_pos;
uint16_t Cal_coarse_hard_pos;
uint16_t Cal_fine_hard_pos;
//int16_t Cal_fine_ms_offset;
//int16_t Cal_coarse_ms_offset;
//int16_t Cal_last_pos;
//uint8_t Cal_current_count;
uint8_t Cal_flags;
uint8_t Cal_flags_saved;
bool Manual_calibration_complete;
uint8_t LEDS_manual_calibration_status;
uint8_t Sig100_init_status;
extern BYTE codeEntered;	// defined in comms.c
uint8_t Manual_mode_error;	// MHH:13/09/2023

#ifdef MH_XXX		// MHH:02/02/2026
int Motor_position(void)
{
	return Encoder_Pos / Enc_data.cal_pole_pairs;
}
#endif


//#define CAL_CURRENT_EXTRA		50	// milliamps
//extern uint16_t ADC_cal_raw_current;
int Hub_encoder_pos;
void Hub_return_pos(void)
{
	if(codeEntered)
	{
		if(Hub_encoder_pos != Encoder_Pos)
		{
			Hub_encoder_pos = Encoder_Pos;
			PRINTF("P=%d\r\n",Encoder_Pos);
			DPRINTF("P=%d\r\n",Encoder_Pos);
			PRINTF_FLUSH;
		}
	}
}

void Set_encoder_pos(int ipos,int ifrom)
{
	if(ipos > 20000)
	{
		DPRINTF("Set_encoder_pos:ipos=%d,ifrom=%d\r\n",ipos,ifrom);
	}
	Encoder_Pos = ipos;
}
bool Hub_return_position;
//int Hub_calibrate_last_ms_stop_pos;
//int Hub_calibrate_save_enc_pos;
typedef enum
{
	MAN_IDLE=0,
	MAN_START,
	MAN_COARSE_TO_HARD_STOP,
	MAN_FINE_TO_HARD_STOP,
	MAN_COARSE_TO_FINE_MS,
	MAN_FINE_TO_FINE_MS,
	MAN_FINISH,
	MAN_ERROR,	// MHH:13/09/2023
} manual_Mode;
manual_Mode Manual_mode;
//#define MAX_CURRENT	1000
void Hub_calibrate_main(void)
{
//	static int Save_enc_pos;
//	static int Same_pos_count;
	switch(Manual_mode)
	{
	default:
		return;

	case MAN_COARSE_TO_HARD_STOP:
		Hub_calibrate_state = CAL_MAIN;
		break;
	case MAN_FINE_TO_HARD_STOP:
		if(Cal_hardstop_sent == false)
		{
			Cal_hardstop_sent = true;
			Cal_coarse_hard_pos = 0;	// by definition
			Hub_calibrate_last_ms_stop_pos = 0;
//			PRINTF("CAL:C=%d\r\n",Cal_coarse_hard_pos);
			PRINTF("CAL:C=%d,%d\r\n",Cal_flags,Cal_coarse_hard_pos);
			return;
		}
		if(Cal_flags != Cal_flags_saved)
		{
			if(Hub_calibrate_last_ms_stop_pos != 0)
			{
//				DPRINTF("Encoder_Pos=%d,ms_stop_pos=%d\r\n",Encoder_Pos,Hub_calibrate_last_ms_stop_pos);
				Set_encoder_pos(Hub_calibrate_last_ms_stop_pos,100);
//				Hub_calibrate_last_ms_stop_pos = 0;
			}

			if((Cal_flags_saved & 2) && ((Cal_flags & 2) == 0))
			{
				Cal_coarse_pos = Encoder_Pos;
//				Cal_coarse_ms_offset = Cal_coarse_hard_pos - Encoder_Pos;	// Do we need to know this?

			}
			Cal_flags_saved = Cal_flags;

			//			PRINTF("Cal_flags:%d,pos:%d\r\n",Cal_flags,Encoder_Pos);
			PRINTF("CAL:S=%d,%d\r\n",Cal_flags,Encoder_Pos);
			if(Cal_flags & 1)
			{
				if(Cal_fine_pos == 0)
				{
					Cal_fine_pos = Encoder_Pos;
//					Cal_fine_ms_offset = Cal_coarse_hard_pos - Encoder_Pos;	// Do we need to know this?
				}
				//				PRINTF("Fine MS offset:%d\r\n",Cal_coarse_hard_pos - Encoder_Pos);
			}
			return;
		}
		break;
	case MAN_COARSE_TO_FINE_MS:
		if(Cal_fine_hard_pos == 0)
		{
			if(Hub_calibrate_last_ms_stop_pos != 0)
			{
				Set_encoder_pos(Hub_calibrate_last_ms_stop_pos,200);
			}
			Cal_fine_hard_pos = Encoder_Pos;
			PRINTF("CAL:F=%d\r\n",Cal_fine_hard_pos);
			Cal_flags_saved = 0;
			return;
		}
		break;

	case MAN_FINE_TO_FINE_MS:
	case MAN_FINISH:	// Here after Hub has finished calibrate
		if(Cal_fine_pos2 == 0)
		{
			if(Cal_flags != Cal_flags_saved)
			{
				if(Hub_calibrate_last_ms_stop_pos != 0)
				{
					Set_encoder_pos(Hub_calibrate_last_ms_stop_pos,300);
				}
				if(((Cal_flags_saved & 1) == 0) && (Cal_flags & 1))
				{
					Cal_fine_pos2 = Encoder_Pos;
					PRINTF("CAL:f=%d\r\n",Cal_fine_pos2);	// Note: Will need to change Calibrator to handle this
					Hub_calibrate_state = CAL_FINISH;
					return;
				}
				Cal_flags_saved = Cal_flags;
			}
		}
		break;

	case MAN_ERROR:
//		PRINTF("CAL:E=%d\r\n",Manual_mode_error);
		Hub_calibrate_state = CAL_ERROR;
		break;

	}
	Hub_return_pos();
}
void Hub_calibrate_check(void)
{
//	WORD current=0;
//	int hub_size,pos_from_chs,pos_offset_from_fine_ms;

	if(Hub_calibrate_state == CAL_IDLE)
	{
		return;
	}

	if(Hub_calibrate_checkval != 12345)
	{
		Abort( AC210_SRC_SIG100+10,"Hub_calibrate:checkval");
	}

	switch(Hub_calibrate_state)
	{
	default:
		return;

	case CAL_START:
		if(Manual_mode != MAN_IDLE)
		{
			DPRINTF("Calibrate:Moving to COARSE hard stop,pos:%d\r\n",Encoder_Pos);
	//		Cal_fine_ms_stop_found = false;
			Hub_calibrate_state = CAL_MAIN;
	//		Cal_current_count = 0;
			Cal_flags_saved = 0;
			Hub_calibrate_last_ms_stop_pos=0;
			Cal_coarse_pos = 0;
			Cal_fine_pos = 0;
			Cal_fine_pos2 = 0;
			Cal_hardstop_sent = false;
			Cal_fine_hard_pos = 0;
			Hub_return_pos();
		}
		return;


	case CAL_MAIN:
// Note: We will need a way to decide when to exit. Probably when return status from Hub is 'Finish'
		Hub_calibrate_main();
		return;

	case CAL_FINISH:
		if(Manual_mode == MAN_IDLE)
		{
			Hub_calibrate_state = CAL_IDLE;
			Hub_calibrate_checkval = 0;
	//				PRINTF("Calibrate:Finished calibration,pos:%d\r\n",Encoder_Pos);
			PRINTF("CAL:X=%d\r\n",Encoder_Pos);

	//		pos_from_chs = Cal_coarse_hard_pos - Encoder_Pos;
	//		pos_offset_from_fine_ms = Encoder_Pos - Cal_fine_pos;
			DPRINTF("CAL:X=%d\r\n",Encoder_Pos);
			DPRINTF("Coarse_hard_pos = %d\r\n",Cal_coarse_hard_pos);
			DPRINTF("Cal_coarse_pos  = %d\r\n",Cal_coarse_pos);
			DPRINTF("Cal_fine_pos    = %d\r\n",Cal_fine_pos);
			DPRINTF("Cal_fine_pos2   = %d\r\n",Cal_fine_pos2);
			DPRINTF("Fine_hard_pos   = %d\r\n",Cal_fine_hard_pos);
			if(Manual_calibration_complete)
			{
				L2PRINTF("Manual calibration complete\r\n");
				// Could reboot here, or try resetting initial SIG100 status so that AC200 gets new Enc_data.
				// Try status first. Note: Need a LED display to confirm success (or failure).
				Sig100_init_status = 0;
				LEDS_manual_calibration_status = 2;
			}
			else
			{
				if(LEDS_manual_calibration_status == 1)
				{
					L2PRINTF("Manual calibration FAIL\r\n");
					LEDS_manual_calibration_status = 3;
				}
			}
		}
		Hub_return_pos();
//		Hub_return_position = true;
		return;

	case CAL_ERROR:	// MHH:13/09/2023
//		DPRINTF("CE:MM=%d\r\n",(int)Manual_mode);
		if(Manual_mode == MAN_IDLE)
		{
			PRINTF("CAL:E=%d\r\n",Manual_mode_error);
			Hub_calibrate_state = CAL_IDLE;
			Hub_calibrate_checkval = 0;
			if(LEDS_manual_calibration_status == 1)
			{
				L2PRINTF("Manual calibration FAIL\r\n");
				LEDS_manual_calibration_status = 3;
			}
		}
//		Hub_return_pos();
		//		Hub_return_position = true;
		return;

	}
}


//=============================================================================================
void HDC_write(bool bval)
{
//	Chip_GPIO_WritePortBit(LPC_GPIO, PIN_DEF2(SIG60_HDC_PIND), bval);
//	Chip_GPIO_WritePortBit(LPC_GPIO, SIG60_HDC_PORT, SIG60_HDC_PIN, bval);
}
//-------------------------------------------------------------------------------------------------
uint8_t SIG100_display;

//-------------------------------------------------------------------------------
void Aux_allow_interrupts(void);
void Aux_Set_no_interrupts(void);
//=======================================================================================================================
//#define HUB_UD_COARSE_STOP	2
#define HUB_UD_FEATHER_STOP 4
#define HUB_UD_REVERSE_STOP 8

//----------------------------------------------------------------------------
bool Sig100_connected;
void AC200_SIG100_putchar(char ch);
bool Sig100_display;
void Sig100_UARTPutChar(char ch)
{
	AC200_SIG100_putchar(ch);
//	if(Sig100_display) PC_putc(ch);
}
//----------------------------------------------------------------------------
int Sig100_err;
void SIG100_error(int ival)
{
	Sig100_err = ival;
//	PC_PRINTF("SIG100_error:%d\r\n",ival);

}

//----------------------------------------------------------------------------
#define SIG100_PACKET_MAX	32
#define SIG100_LARGE_PKT_LEN	8
#define SIG100_DIAG_PKT_LEN		9
#define SIG100_SMALL_PKT_LEN	4
int SIG100_packet_ix;
int SIG100_packet_len;
uint8_t SIG100_input_packet[SIG100_PACKET_MAX+2];

#define SIG100_UART_READABLE	Aux_readable
#define SIG100_UART_GETC		Aux_getc
#define SIG100_UART_GETC_TIMEOUT		Aux_getc_timeout

#define SIG100_UART LPC_UART3


/*
 *  MHH:28/07/2023. Have changed init status logic to allow addition of a 'K' packet in the future which will contain the 32 bytes following the
 *                  first 64 bytes of Enc_data.
 */



#define SIG100_INI_I_LOADED		1
#define SIG100_INI_J_LOADED		2
#define SIG100_INI_K_LOADED		4

#define SIG100_INI_ALL_LOADED	7
//--------------------------------------------------------------------------------------------
TIMER_t Motor_timer;
void Sig100_init(void)		// called from main()
{
	  Timer_start(&Motor_timer);
}
//--------------------------------------------------------------------------------------------
uint8_t Sig100_bad_receive_code;
void Sig100_bad_receive(int ival)
{
	if(Sig100_init_status != SIG100_INI_ALL_LOADED) return;

	DPRINTF("D:%d\r\n",ival);
	Sig100_bad_receive_code = (uint8_t)ival;
	return;

//	char c = '0' + ival;
//	PC_PUTC(c);
}
//--------------------------------------------------------------------------------------------
bool SIG100_checksum_input_pkt(const int plen)
{
	uint8_t checksum = 0;
	for(int i=0;i<plen;i++)
	{
		checksum += SIG100_input_packet[i];
	}
	if(checksum == 0) return true;
	return false;
}
//--------------------------------------------------------------------------------------------
uint8_t Sig100_send_pkt[20];
uint8_t Sig100_send_pkt_len;
void Sig100_checksum_output_pkt(const int plen)
{
	Sig100_send_pkt_len = plen;
	uint8_t checksum = 0;
	int last_ix = plen -1;
	for(int i=0;i<last_ix;i++)
	{
		checksum += Sig100_send_pkt[i];
	}
	uint8_t checksum_byte = -checksum;	// Idea is that total of all bytes = 0
	Sig100_send_pkt[last_ix] = checksum_byte;
}
//--------------------------------------------------------------------------------------------
void Sig100_send_output_pkt(const int plen)
{
	for(int i=0;i<plen;i++)
	{
		char ch = (char)Sig100_send_pkt[i];
		Sig100_UARTPutChar(ch);
	}
}
//--------------------------------------------------------------------------------------------
#ifdef MH_SHOW
char Show_buffer[32];
void Show_command(void)
{
//	return;
	if(Sig100_send_pkt_len == 0) return;
	char *bp = Show_buffer;
	for(int i=0;i<Sig100_send_pkt_len;i++)
	{
		uint8_t c = Sig100_send_pkt[i];
		if(c > 32 && c < 127)
		{
			*bp++ = c;
		}
		else
		{
			int slen = sprintf(bp,"<%d>",c);
			bp+= slen;
		}
		if(bp - Show_buffer >= 30) break;
	}
	*bp++ = 0;
	DPRINTF("CL:%d,%s\r\n",Sig100_send_pkt_len,Show_buffer);
}
#endif
void Sig100_checksum_and_send_pkt(const int plen)
{
	Sig100_checksum_output_pkt(plen);
	Sig100_send_output_pkt(plen);
//	Show_command();
}

#define SIG100_I_PKT_LEN		14
#define SIG100_J_PKT_LEN		8
//--------------------------------------------------------------------------------------------
WORD Get_word(uint8_t *src)
{
	WORD w = src[0] | (src[1] << 8);
	return w;
}
//--------------------------------------------------------------------------------------------
/*
 *  *  Packet 'i' return
 *
 *  	0			'>' Header
 *  	1			'i'
 *  	2-3			Hub ID
 *  	4-5			Update count
 *      6			checksum
 *
 */

uint8_t Hub_data[220];	// MHH:22/07/2023. Increased from 200.
uint8_t Hub_header[4];

int Aux_getc_timeout(uint32_t millisecs);
uint32_t Wiki_CRC32(const uint8_t data[],size_t data_length);
void Sig100_copy_Hub_data_to_Enc_data(int data_len);

uint32_t Get_uint32(uint8_t *data_p)
{
	// Assume litle endian
	uint32_t result = data_p[0] | (data_p[1] << 8) | (data_p[2] << 16) | (data_p[3] << 24);
	return result;
}
uint16_t Get_uint16(uint8_t *data_p)
{
	uint16_t result = data_p[0] | (data_p[1] << 8);
	return result;
}

int16_t Get_int16(uint8_t *data_p)
{
	int16_t result = data_p[0] | (data_p[1] << 8);
	return result;
}

#define EE_HUBDATA_POS	768
void Sig100_zero_enc_data(void)	// Called from diags when resetting diags and log file
								// MHH:23/07/2023 or ATXZERO_ENCDATA=0 command
{
//	if(ps.parms[BL_ENABLED] != 1) return;

	uint8_t *enc_data_start_p = (uint8_t *)&Enc_data;
	int len = sizeof(Enc_data);
	memset(enc_data_start_p,0,len);
	Enc_data.magic = SSP_MAGIC_1;
	Enc_data.magic2 = SSP_MAGIC_2;
	Enc_data.magic3 = SSP_MAGIC_3;
//	int data_offset = src_p - enc_data_start_p;
//	int pos = 768;
//	p_eeprom_i2c_pos(pos);
	p_eeprom_i2c_pos(EE_HUBDATA_POS);	// MHH:28/11/2025


	p_eeprom_i2c_write(enc_data_start_p,len);

	Sig100_init_status = 0;		// Will need to reload data from hub.
}


/*
 * typedef struct
{
	uint16_t code;			// 0.
	uint16_t command;		// 2. Could be byte
	uint16_t ac200_run;		// 4.
	uint16_t count;			// 6. of this code, if multiple occurrences of error.
	uint16_t from;			// 8. where it was called from
	uint16_t total;			// 10.
} Enc_err_td;				// 12.
 *
 */

bool Sig100_load_Enc_data_from_page_zero(void)
{
//	int pos = 768;
	int len = sizeof(Enc_data);
	if(len > sizeof(Hub_data))
	{
		Abort( AC210_SRC_SIG100+20,"Sig100_load:len>sizeof(Hub_data)");
	}
	p_eeprom_i2c_pos(EE_HUBDATA_POS);
	uint8_t *bp = (uint8_t *)Hub_data;
	if(p_eeprom_i2c_read(bp,len))
	{
		return false;
	}
	uint32_t data_crc32 = Get_uint32(Hub_data + 60);

	uint32_t crc32 = Wiki_CRC32(Hub_data,60);
	if(data_crc32 != crc32 )
	{
		return false;
	}

	// Data looks OK, now to copy to Enc_data struct.

	Sig100_copy_Hub_data_to_Enc_data(len);

	return true;
}


void Copy_Enc_data_to_page_zero(uint8_t *src_p,int len)
{
	uint8_t *enc_data_start_p = (uint8_t *)&Enc_data;
	int data_offset = src_p - enc_data_start_p;
	int pos = EE_HUBDATA_POS + data_offset;
	if(len + data_offset > 256)
	{
		Abort( AC210_SRC_SIG100+30,"Copy_Enc_data:Copy too long");
	}
	p_eeprom_i2c_pos(pos);
	p_eeprom_i2c_write(src_p,len);
}


Enc_err_td Hub_err;
bool Sig100_process_error_pkt()
{
//	DPRINTF("Sig100_error\r\n");
	if(SIG100_checksum_input_pkt(9) == false)
	{
		return false;
	}
	if(SIG100_input_packet[1] != 'e') return false;

/*
{
	uint16_t code;			// 0.
	uint8_t command;		// 2.
	uint8_t fill;
	uint16_t ac200_run;		// 4.
	uint16_t count;			// 6. of this code, if multiple occurrences of error.
	uint16_t total;			// 8..
} Enc_err_td;				// 10

*/
	/*  *  Packet '#' return
		*
		*  	0			'#' Header
		*  	1			'e'
		*  	2			Error type 'E','A',"W'
			3			Command
		*  	4-5			Error code
		*   6   		checksum
		*/



// OK, looks like we have a valid error pkt. Now to store the details in page zero, and update any counts.
//   We will have an updated error record in Enc_data structure.
//   Should be able to copy input_pkt directly into work error structure

	uint8_t error_type = SIG100_input_packet[2];
	Hub_err.type = error_type;
	Hub_err.command = SIG100_input_packet[3];
	Hub_err.code = Get_uint16(SIG100_input_packet+4);
	Hub_err.eval = Get_int16(SIG100_input_packet+6);
	Hub_err.ac200_run = Stat_Rec.run_number;
	Hub_err.secs = (uint16_t)TimeInSeconds;

	Enc_err_td *enc_err_p;
	switch(error_type)
	{
	case 'E':
		enc_err_p = (Enc_err_td *)&Enc_data.error;
		Enc_data.error_count2++;
		break;

	case 'A':
		enc_err_p = (Enc_err_td *)&Enc_data.abort;
		Enc_data.abort_count++;
		break;

	case 'W':
		enc_err_p = (Enc_err_td *)&Enc_data.watchdog;
		Enc_data.watchdog_count++;
		break;

	default:
		return false;

	}
	Enc_data.errtbl[3] = Enc_data.errtbl[2];	// Shuffle entries down, and add one we are about to overwrite to top
	Enc_data.errtbl[2] = Enc_data.errtbl[1];
	Enc_data.errtbl[1] = Enc_data.errtbl[0];
	Enc_data.errtbl[0] = *enc_err_p;

	*enc_err_p = Hub_err;

	Enc_data.total_errors++;
	uint8_t *enc_data_p = (uint8_t *)&Enc_data.total_errors;
	uint8_t *end_p = (uint8_t *)&Enc_data.end;
	int len = end_p - enc_data_p;
	Copy_Enc_data_to_page_zero(enc_data_p,len);
//	DPRINTF("Sig100_process_error_pkt:total_errors:%d,len=%d",Enc_data.total_errors,len);
	return true;

}
//--------------------------------------------------------------------------------------------
void Sig100_update_param_range(ParamIndex first_parameter,ParamIndex last_parameter)
{
    uint8_t *param_start_p = (uint8_t *)&ps;
    uint8_t *param_hub_p = (uint8_t *)(&ps.parms[first_parameter]);
    uint8_t *param_hub_end_p = (uint8_t *)(&ps.parms[last_parameter]);
    uint32_t pos = param_hub_p - param_start_p;
    int len = param_hub_end_p - param_hub_p + 2;

    p_eeprom_i2c_pos(pos);
	p_eeprom_i2c_write(param_hub_p,len);
}
//--------------------------------------------------------------------------------------------
void Sig100_update_brushless_error_params(void)
{
}
//------------------------------------------------------------------------------------------------------
void Sig100_update_readonly_params(void)
{
   ps.parms[BL_HUB_ID] = Enc_data.hub_id;
   ps.parms[BL_UPDATE_COUNT] = Enc_data.update_count;
   ps.parms[BL_MODIFIED_DATE] = Enc_data.modified_date;
   ps.parms[BL_HUB_STOPS] = Enc_data.user_defined_stops;
   ps.parms[BL_FINE_STOP] = Hub_calc.fine_stop_angle;
   ps.parms[BL_COARSE_STOP] = Hub_calc.coarse_stop_angle;
   ps.parms[BL_FEATHER_STOP] = Hub_calc.feather_stop_angle;
   ps.parms[BL_REVERSE_STOP] = Hub_calc.reverse_stop_angle;
   float f = Enc_data.cal_motor_ratio;
   f /= 100;
   int ratio = iround(f);
   ps.parms[BL_MOTOR_RATIO] = ratio;	// ps.parms just for show, no implied decimal place
   ps.parms[BL_MOTOR_POLE_PAIRS] = Enc_data.cal_pole_pairs;

   Sig100_update_param_range(BL_HUB_ID,BL_MOTOR_POLE_PAIRS);

}

/*
	 *  *  Packet 'k' return
	 *
	 *  	0			'>' Header
	 *  	1			'K'
	 *  	2-3			encoder pos
	 *  	4-5			reserved future use
	 *      6			checksum
*/


#define SIG100_k_PKT_LEN		7
bool Sig100_verify_k_pkt(void)		// MHH:08/01/2026
{
	int c;
	for(;SIG100_packet_ix<SIG100_k_PKT_LEN;)
	{
		c = SIG100_UART_GETC();
		if(c < 0) return false;
		SIG100_input_packet[SIG100_packet_ix++] = (uint8_t)c;
	}

	if(SIG100_checksum_input_pkt(SIG100_k_PKT_LEN) == false)
	{
		return false;
	}
	Sig100_init_status |= SIG100_INI_K_LOADED;
	Set_encoder_pos(Get_word(SIG100_input_packet+2),400);	// May not need this yet
	return true;
}
bool Sig100_verify_j_pkt(void)
{
	// OK, going to break the rules here and wait for a response.
	// Expect header of ">j"
	// Then "{ndddd...cccc}
	// where n = number of bytes expected, including crc32
	// d = data bytes
	// cccc = 4 byte crc32

	// We should set a timeout, or read with a timeout. Not sure about going into sig100 read mode without finishing sending.
	// Could check to see if anything left to send, and/or status uart

	int count=0;
	int c;
	for(;SIG100_packet_ix<4;)
	{
//		c = Aux_getc_timeout(100);		// Allow a tenth of a second
		c = SIG100_UART_GETC();
		if(c < 0) return false;
		SIG100_input_packet[SIG100_packet_ix++] = (uint8_t)c;
	}
	if(SIG100_input_packet[2] != '{') return false;
	int data_count = SIG100_input_packet[3];


	// The new plan is that we do not overwrite any error data, so just copy 80 bytes
	if(data_count != 84) return false;

	count = 0;
	for(int i=0;i<data_count;i++)
	{
		c = Aux_getc_timeout(100);		// Allow a tenth of a second
		if(c < 0) break;
		Hub_data[count++] = (uint8_t)c;
	}
	c = Aux_getc_timeout(100);
	if(c != '}') return false;

	// Data seems in correct format. Now to check crc32 value
	int data_size = data_count - 4;
	uint32_t data_crc32 = Get_uint32(Hub_data + data_size);

	uint32_t crc32 = Wiki_CRC32(Hub_data,data_size);
	if(data_crc32 != crc32 ) return false;

	// Data looks OK, now to copy to Enc_data struct.

	Sig100_copy_Hub_data_to_Enc_data(data_size);

	//Now need to write this data to page zero of diagnostics page (first 256 bytes = params + stats).
	// Write at position (256 * 3) = 768 as is unused.

//	int pos = 768;
	p_eeprom_i2c_pos(EE_HUBDATA_POS);
	uint8_t *bp = (uint8_t *)Hub_data;
	p_eeprom_i2c_write(bp,data_size);		// Should be 128 bytes

	Sig100_update_readonly_params();
//	Sig100_init_status = 3;		// All done!!
	Sig100_init_status |= SIG100_INI_J_LOADED;
	return true;
}
//--------------------------------------------------------------------------------------------
// MHH:27/07/2023. Modify _I_pkt logic to add Enc_data.update_count2.
// In future, we can compare against Enc_data and update 32 byte field using _k_pkt logic.
#define SIG100_i2_PKT_LEN		12
bool Sig100_verify_i_pkt(void)
{
	int c;
	for(;SIG100_packet_ix<SIG100_i2_PKT_LEN;)
	{
//		c = Aux_getc_timeout(100);		// Allow a tenth of a second
		c = SIG100_UART_GETC();
		if(c < 0) return false;
		SIG100_input_packet[SIG100_packet_ix++] = (uint8_t)c;
	}

	if(SIG100_checksum_input_pkt(SIG100_i2_PKT_LEN) == false)
	{
		return false;
	}
	Sig100_init_status |= SIG100_INI_I_LOADED;

	/*
		 *  *  Packet 'i' return
		 *
		 *  	0			'>' Header
		 *  	1			'I'
		 *  	2-3			creation date
		 *  	4-5			Hub ID
		 *  	6-7			Update count
		 *  	8-9			Hub software version
		 *  	10			Update count 2 (MHH:08/01/2026. Seems unused).
		 *      11			checksum
	*/
	WORD creation_time = Get_word(SIG100_input_packet+2);
	WORD hub_id = Get_word(SIG100_input_packet+4);
	WORD update_count = Get_word(SIG100_input_packet+6);
//	WORD total_errors = Get_word(SIG100_input_packet+6);
	WORD hub_software_version = Get_word(SIG100_input_packet+8);
	if(hub_software_version < 149)
	{
		Sig100_init_status |= SIG100_INI_K_LOADED;	// MHH:08/01/2026
	}

//	WORD param_hub_id = getParameter (BL_HUB_ID);
//	WORD param_update_count = getParameter (BL_UPDATE_COUNT);
//	WORD param_total_errors = getParameter (BL_TOTAL_ERRORS);
	WORD param_hub_software_version = getParameter(BL_HUB_SOFTWARE_VERSION);
//	uint8_t update_count2 = SIG100_input_packet[10];
	if(param_hub_software_version != hub_software_version)
	{
		writeParameter (BL_HUB_SOFTWARE_VERSION, hub_software_version);
		Enc_data.hub_software_version = hub_software_version;

#ifdef MH_XXX			// MHH:22/07/2023
		Enc_data.rpm_starts = 0;	// Following are not needed at this end
		Enc_data.pos = 0;
		Enc_data.rpm = 0;
		uint8_t * enc_data_p = (uint8_t *)&Enc_data.rpm_starts;
		Copy_Enc_data_to_page_zero(enc_data_p,8);
#endif
		uint8_t * enc_data_p = (uint8_t *)&Enc_data.hub_software_version;	// MHH:22/07/2023
		Copy_Enc_data_to_page_zero(enc_data_p,2);
	}

	// Now check if page zero data is same
	if(Sig100_load_Enc_data_from_page_zero() == false)
	{
//		Sig100_init_status = 2;
		return true;

	}

	if(param_hub_software_version == 107)	// MHH:24/07/2023. Force ini because of change of format
	{
		if(Enc_data.magic != SSP_MAGIC_1 && Enc_data.magic2 != SSP_MAGIC_2)
		{
			Sig100_zero_enc_data();
//			Sig100_init_status = 2;
			return true;
		}
	}

	if(creation_time == Enc_data.creation_time && update_count == Enc_data.update_count && hub_id == Enc_data.hub_id)
	{
//		Sig100_init_status = 3;	// Hopefully normal path
		Sig100_init_status |= 	SIG100_INI_J_LOADED;
//		if(Enc_data.update_count2 == update_count2) Sig100_init_status |= 	SIG100_INI_K_LOADED;
		return true;
	}
//#define MH_FORCE_INI_J
#ifdef MH_FORCE_INI_J
	Sig100_init_status = 2; // MHH:DEBUG!!!
	return true;
#endif
//	Sig100_init_status = 2;	// Looks like we need a full Enc_data update from Hub...
	return true;

#ifdef MH_XXX


	int c;
	for(;SIG100_packet_ix<9;)
	{
//		c = Aux_getc_timeout(100);		// Allow a tenth of a second
		c = SIG100_UART_GETC();
		if(c < 0) return false;
		SIG100_input_packet[SIG100_packet_ix++] = (uint8_t)c;
	}
//	if(count != 11) return false;

//	if(SIG100_input_packet[0] != '>') return false;
//	if(SIG100_input_packet[1] != 'i') return false;

	if(SIG100_checksum_input_pkt(9) == false)
	{
		return false;
	}
	/*
		 *  *  Packet 'i' return
		 *
		 *  	0			'>' Header
		 *  	1			'i'
		 *  	2-3			Hub ID
		 *  	4-5			Update count
		 *  	6-7			Hub software version
		 *      8			checksum
	*/
	WORD hub_id = Get_word(SIG100_input_packet+2);
	WORD update_count = Get_word(SIG100_input_packet+4);
//	WORD total_errors = Get_word(SIG100_input_packet+6);
	WORD hub_software_version = Get_word(SIG100_input_packet+6);

	WORD param_hub_id = getParameter (BL_HUB_ID);
	WORD param_update_count = getParameter (BL_UPDATE_COUNT);
//	WORD param_total_errors = getParameter (BL_TOTAL_ERRORS);
	WORD param_hub_software_version = getParameter(BL_HUB_SOFTWARE_VERSION);
	if(param_hub_software_version != hub_software_version)
	{
		writeParameter (BL_HUB_SOFTWARE_VERSION, hub_software_version);
		Enc_data.hub_software_version = hub_software_version;
#ifdef MH_XXX	// MHH:22/07/2023
		Enc_data.rpm_starts = 0;	// Following are no needed at this end
		Enc_data.pos = 0;
		Enc_data.rpm = 0;
#endif
		uint8_t * enc_data_p = (uint8_t *)&Enc_data.rpm_starts;
		Copy_Enc_data_to_page_zero(enc_data_p,8);
	}

//#define MH_FORCE_INI_J
#ifdef MH_FORCE_INI_J
	Sig100_init_status = 2; // MHH:DEBUG!!!
	return true;
#endif
	if(hub_id == param_hub_id && update_count == param_update_count)
	{
		if(Sig100_load_Enc_data_from_page_zero())
		{
			Sig100_init_status = 3;	// Hopefully normal path
			return true;

		}
	}
	Sig100_init_status = 2;	// Looks like we need a full Enc_data update from Hub...
	return true;
#endif
}

//--------------------------------------------------------------------------------------------
bool Sig100_verify_I_pkt(void)		// This version uses creation_time as an extra identifier
{
	int c;
	for(;SIG100_packet_ix<11;)
	{
//		c = Aux_getc_timeout(100);		// Allow a tenth of a second
		c = SIG100_UART_GETC();
		if(c < 0) return false;
		SIG100_input_packet[SIG100_packet_ix++] = (uint8_t)c;
	}

	if(SIG100_checksum_input_pkt(11) == false)
	{
		return false;
	}
	Sig100_init_status |= SIG100_INI_I_LOADED;

	/*
		 *  *  Packet 'i' return
		 *
		 *  	0			'>' Header
		 *  	1			'I'
		 *  	2-3			creation date
		 *  	4-5			Hub ID
		 *  	6-7			Update count
		 *  	8-9			Hub software version
		 *      10			checksum
	*/
	WORD creation_time = Get_word(SIG100_input_packet+2);
	WORD hub_id = Get_word(SIG100_input_packet+4);
	WORD update_count = Get_word(SIG100_input_packet+6);
//	WORD total_errors = Get_word(SIG100_input_packet+6);
	WORD hub_software_version = Get_word(SIG100_input_packet+8);
	if(hub_software_version < 149)
	{
		Sig100_init_status |= SIG100_INI_K_LOADED;	// As if hub is using ini_I_pkt it does not have K logic.
	}

//	WORD param_hub_id = getParameter (BL_HUB_ID);
//	WORD param_update_count = getParameter (BL_UPDATE_COUNT);
//	WORD param_total_errors = getParameter (BL_TOTAL_ERRORS);
	WORD param_hub_software_version = getParameter(BL_HUB_SOFTWARE_VERSION);
	if(param_hub_software_version != hub_software_version)
	{
		writeParameter (BL_HUB_SOFTWARE_VERSION, hub_software_version);
		Enc_data.hub_software_version = hub_software_version;

#ifdef MH_XXX			// MHH:22/07/2023
		Enc_data.rpm_starts = 0;	// Following are not needed at this end
		Enc_data.pos = 0;
		Enc_data.rpm = 0;
		uint8_t * enc_data_p = (uint8_t *)&Enc_data.rpm_starts;
		Copy_Enc_data_to_page_zero(enc_data_p,8);
#endif
		uint8_t * enc_data_p = (uint8_t *)&Enc_data.hub_software_version;	// MHH:22/07/2023
		Copy_Enc_data_to_page_zero(enc_data_p,2);
	}

	// Now check if page zero data is same
	if(Sig100_load_Enc_data_from_page_zero() == false)
	{
//		Sig100_init_status = 2;
		return true;

	}

	if(param_hub_software_version == 107)	// MHH:24/07/2023. Force ini because of change of format
	{
		if(Enc_data.magic != SSP_MAGIC_1 && Enc_data.magic2 != SSP_MAGIC_2)
		{
			Sig100_zero_enc_data();
//			Sig100_init_status = 2;
			return true;
		}
	}

	if(creation_time == Enc_data.creation_time && update_count == Enc_data.update_count && hub_id == Enc_data.hub_id)
	{
		Sig100_init_status |= 	SIG100_INI_J_LOADED;
//		Sig100_init_status = 3;	// Hopefully normal path
		return true;
	}
//#define MH_FORCE_INI_J
#ifdef MH_FORCE_INI_J
	Sig100_init_status = 2; // MHH:DEBUG!!!
	return true;
#endif
//	Sig100_init_status = 2;	// Looks like we need a full Enc_data update from Hub...
	return true;
}
//--------------------------------------------------------------------------------------------
/*
 *  *  Packet 'J' return
 *
 *  	0			'>' Header
 *  	1			'J'
 *  	2-3			actual coarse stop, return only
 *  	4-5			Hub ID
 *  	6			spare
 *      7			checksum
 *
 */
//#ifdef MH_HHHH
//--------------------------------------------------------------------------------------------
int SIG100_good_receives_per_sec;
int SIG100_bad_receives_per_sec;
uint8_t SIG100_line_percent;
extern WORD EEPROM_writes;	// Ignore SIG100 while updating parameters
uint8_t ADC_adjusted_voltage;
//uint16_t ADC_BL_corrected_voltage;
uint8_t ADC_adjusted_temp;
bool SIG100_beta_mode_on(void);
void SIG100_rcv_pkt(void)
{
	int ci;
	for(;;)
	{
		if(SIG100_UART_READABLE() == false)
		{
			break;
		}
		ci = SIG100_UART_GETC();
		if(SIG100_packet_ix < SIG100_PACKET_MAX)
		{
			SIG100_input_packet[SIG100_packet_ix++] = ci;
		}
	}
}

//---------------------------------------------------------------------------------------------------------
//#define POS_CHANGE_BIT		128
//#define CALIBRATE_POS_BIT		128
#define NO_FEATHER_BIT			128		// Only set if position looks invalid at fine stop
#define NOT_READY_BIT			64		// Opposite value of READY_PIN
#define MAN_CALIBRATE_COMPLETE_BIT	32	// MHH:28/08/2023. Manual calibration complete
#define MOTOR_ENABLED_BIT		16		// From AC200hubBL.c

#ifdef MH_XXX
#define DIR_STOPPED 	0
#define DIR_COARSE    	1
#define DIR_FINE   		2
#define DIR_UNKNOWN 	3

#define DIR_MASK		3

#endif
uint8_t BL_command;
bool Sig100_check_return_flags=true;
bool Sig100_motor_timer;
int Sig100_epos;
uint8_t BL_enc_rpm;
uint8_t BL_dac0_v;

void SIG100_check_flags(void)
{
	if(Sig100_check_return_flags == false) return;
	// Note: Could put Hub software Revision check here. First revision to support this is 106

}
bool Hub_pos_error;
bool Hub_not_ready_error;
//uint8_t Hub_not_ready_count;
uint8_t BL_flags;
bool BL_feather_when_error=false;
void SIG100_handle_no_feather_flag(uint8_t flags)
{
	BL_flags = flags;	// Save for diagnostic log.
	if(flags & NO_FEATHER_BIT)
	{
		Hub_pos_error = true;
		LED_overlay_flags |= LED_OVERLAY_FLAG_BL_POS_ERROR;
	}
	else
	{
		Hub_pos_error = false;
		LED_overlay_flags &= ~LED_OVERLAY_FLAG_BL_POS_ERROR;
	}
	if(flags & NOT_READY_BIT)
	{
		Hub_not_ready_error = true;
	}
	else
	{
		Hub_not_ready_error = false;
	}
}
void SIG100_handle_stop_flags(void)
{
	uint8_t stop_flags = SIG100_input_packet[1];	// Common position for short and long pkt.
	SIG100_handle_no_feather_flag(stop_flags);
	ADC_adjusted_voltage = SIG100_input_packet[2];
	stop_flags <<= 4;	// To be consistent with existing logic

	if(stop_flags & S_STOP_COARSE)	// MHH:22/05/2025. Debug!!!
	{
		mh_debug();
	}
	if(Hub_pos_error)
	{
		if(BL_feather_when_error)
		{
			stop_flags |= S_STOP_FEATHER;		// MHH:22/05/2025. Fake feather stop if hub position error
		}
		else
		{
			stop_flags &= ~S_STOP_FEATHER;
		}
	}

	if(stop_flags & S_STOP_REVERSE)
	{
		stop_flags &= ~S_STOP_FINE;		// Mask out fine stop if reverse on
	}

	if(stop_flags & S_STOP_FEATHER)
	{
		stop_flags &= ~S_STOP_COARSE;		// Mask out coarse stop if feather on
	}
	else
	{
		if(operatingMode() == MANUAL)
		{
//			if(Sig100_hub.ud_stops & HUB_UD_FEATHER_STOP && dState != MD_BETA)	// Feather enabled?
			if(Enc_data.user_defined_stops & HUB_UD_FEATHER_STOP && dState != MD_BETA)	// Feather enabled?
			{
				BYTE mankeys = manualKeys();
				if((mankeys & MANUAL_KEY_FEATHER) && (mankeys & MANUAL_KEY_COARSE))
				{
					stop_flags &= ~S_STOP_COARSE;		// Mask out coarse stop if feathering
				}
			}
//			if(Sig100_hub.ud_stops & HUB_UD_REVERSE_STOP)	// Reverse enabled?
#ifdef MH_XXX		// MHH:26/09/2025
			if(Enc_data.user_defined_stops & HUB_UD_REVERSE_STOP)	// Reverse enabled?
			{
				if(dState == MD_BETA || dState == MD_BETA_EXIT)
				{
//					if(SIG100_beta_mode_on())
					if(getParameter(LED4_ENABLE) != 1)		// MHH:15/07/2025. No extra mode?
					{
//						stop_flags &= ~S_STOP_FINE;
						if(stop_flags & S_STOP_REVERSE)	// For Beta reverse, if REVERSE_STOP on then REVERSE_STOP off, FEATHER_STOP on
						{
							stop_flags &= ~(S_STOP_REVERSE);
							stop_flags |= S_STOP_FEATHER;
						}
					}
				}
			}
#endif
		}
	}
	resetStateBits (300,S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER | S_STOP_REVERSE);	// Turn stop bit flags off.
	setStateBits(300,(WORD)stop_flags);

}
//-------------------------------------------------------------------------------------------
//#ifdef MH_SIG100_CHECK_PACKET
#ifdef MH_GET_ENC_SPEED
int Encoder_prev_pos;
int Enc_incr;
int Enc_rpm;
TIMER_t Enc_speed_timer;
void SIG100_get_enc_speed(void)
{
    Enc_incr = Encoder_Pos - Encoder_prev_pos;
    Encoder_prev_pos = Encoder_Pos;
	int msecs = Timer_read_ms(&Enc_speed_timer);
    Timer_reset(&Enc_speed_timer);
    if(msecs == 0) return;	// just in case
    Enc_rpm = 60000 * Enc_incr / msecs;
    Enc_rpm /= Enc_data.cal_pole_pairs;
    DPRINTF("Enc_rpm:%d\r\n",Enc_rpm);
}
#endif
#ifdef MH_XXX
uint8_t Sig100_hub_count;
uint8_t BL_count;
void Sig100_check_count(void)
{
	Sig100_hub_count = (uint8_t)SIG100_UART_GETC();
	uint8_t diff = BL_count - Sig100_hub_count;

	if(diff != 1)
	{
		DPRINTF("Sig100_check_count:BL_count=%d,hub_count==%d\r\n",BL_count,Sig100_hub_count);
	}
}
#endif

bool SIG100_check_packet(void)
{
	SIG100_packet_ix = 0;	// Assume that entire packet is waiting.

	int ci = 0;

	for(;;)
	{
		ci = SIG100_UART_GETC();
		if(ci < 0)
		{
			Sig100_bad_receive(1);
			return false;		// Nothing here...
		}

		bool valid_header = false;
		switch(ci)
		{
		case '_':
		case '&':
		case '#':
		case '>':
		case '@':
		case '*':
		case '^':
			valid_header = true;
			break;
		}
		if(valid_header) break;
	}

	// The only way to exit this loop is EOF or a valid header
	SIG100_input_packet[SIG100_packet_ix++] = (uint8_t) ci;
//	uint8_t flags;
	switch(ci)
	{
	default:		// Should never get here
		return false;

	case '_':	// Normal 4 byte response packet
		SIG100_rcv_pkt();
		BL_enc_rpm = 0;				// MHH:14/08/2025
		BL_dac0_v = 0;				// MHH:04/09/2025
		if(SIG100_packet_ix < SIG100_SMALL_PKT_LEN) // Allow it to be greater than 3, in case spurious
		{
			Sig100_bad_receive(2);
			return false;
		}
		if(SIG100_checksum_input_pkt(SIG100_SMALL_PKT_LEN) == false)
		{
			Sig100_bad_receive(3);
			return false;
		}
		SIG100_handle_stop_flags();
		SIG100_check_flags();
		return true;

	case '*':  	// MHH:01/09/2023. Response if Hub diagnostics on. Was type '@'
				//  Response to 'A' command.  Changed because I want to get temperature too. Note 2 spare bytes.
		SIG100_rcv_pkt();
		if(SIG100_packet_ix != 9) // Expect 9 bytes
		{
			Sig100_bad_receive(31);
			return false;
		}
		if(SIG100_checksum_input_pkt(9) == false)
		{
			Sig100_bad_receive(32);
			return false;
		}
		ADC_adjusted_temp = SIG100_input_packet[3];
		BL_enc_rpm = SIG100_input_packet[4];
		BL_dac0_v   = SIG100_input_packet[5];
		SIG100_handle_stop_flags();
		Encoder_Pos = Get_uint16(SIG100_input_packet+6);
		if(Manual_position_encoder)
		{
//			if((Encoder_Pos >= Encoder_target_pos - 1) && (Encoder_Pos <= Encoder_target_pos + 1))
			if(Encoder_Pos == Encoder_target_pos)	// MHH:07/06/2026
			{
				if(++Manual_position_count >= 5)
				{
					Manual_position_encoder = false;
				}
			}
		}
		return true;


	case '@':		// Special packet if calibrating
		SIG100_rcv_pkt();
		if(SIG100_packet_ix != 9) // Expect 9 bytes
		{
			Sig100_bad_receive(21);
			return false;
		}
		if(SIG100_checksum_input_pkt(9) == false)
		{
			Sig100_bad_receive(22);
			return false;
		}
		SIG100_handle_stop_flags();	// MHH:05/08/2023. Was commented out. May need an argument to routine to distinguish cases.
		uint8_t stop_flags = SIG100_input_packet[1];	// Common position for short and long pkt.
		Cal_flags = (stop_flags & 63);	// Mask out high 4 bits
#ifdef MH_XXX	// MHH:05/08/2023
		SIG100_handle_no_feather_flag(stop_flags);
		stop_flags <<= 4;	// To be consistent with existing logic
		resetStateBits (310,S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER | S_STOP_REVERSE);	// Turn stop bit flags off.
		setStateBits(310,(WORD)stop_flags);
		ADC_adjusted_voltage = SIG100_input_packet[2];
#endif
		Manual_mode_error = SIG100_input_packet[2];		// MHH:13/09/2023
		Manual_mode = (manual_Mode)SIG100_input_packet[3];
/*
 * 			Field		Offset	length	Description
 * 			id			0		1		Identifier = '@'
 * 			flags		1		1		flags. eg FINE_STOP, NOT_READY
 * 			voltage		2		1
 * 			type		3		1
 * 										0 = Enc_Position
 * 										1 = Coarse_hard_stop
 * 										2 = Coarse MS stop
 * 										3 = Fine MS stop (1)
 * 										4 = Fine hard stop
 * 										5 = Fine MS stop (2)
 * 										6 = finished calibrate
 * 										Note: If needed, could just use 4 bits, and other 4 bits for something else
 *			stop_pos	4		2		2 byte unsigned integer
 *			pos			6		2		2 byte unsigned integer
 *			checksum 	8		1
 *
 */
//		Encoder_Pos = (int16_t)(SIG100_input_packet[5] | (SIG100_input_packet[6] << 8));
// Note: Sometimes Encoder position will actually be position of a stop
		Manual_calibration_complete = ((stop_flags & MAN_CALIBRATE_COMPLETE_BIT) != 0);
		Hub_calibrate_last_ms_stop_pos = Get_uint16(SIG100_input_packet+4);
		Set_encoder_pos(Get_uint16(SIG100_input_packet+6),500);
		return true;


	case '&':	// Longer 8 byte response packet with position
		SIG100_rcv_pkt();
		if(SIG100_packet_ix < SIG100_LARGE_PKT_LEN) // Allow it to be greater than 8, in case spurious bytes
		{
			Sig100_bad_receive(4);
			return false;
		}
		if(SIG100_checksum_input_pkt(SIG100_LARGE_PKT_LEN) == false)
		{
			Sig100_bad_receive(5);
			return false;
		}
		SIG100_handle_stop_flags();
		ADC_adjusted_temp = SIG100_input_packet[3];
		BL_enc_rpm = SIG100_input_packet[4];		// MHH:14/08/2025
		BL_dac0_v = 0;				// MHH:04/09/2025
		Set_encoder_pos(Get_uint16(SIG100_input_packet + 5),600);
		SIG100_check_flags();
		return true;

	case '^':	// Diagnostics 9 byte response packet with Enc_rpm and DAC0_v
		SIG100_rcv_pkt();
		if(SIG100_packet_ix < SIG100_DIAG_PKT_LEN) // Allow it to be greater than 9, in case spurious bytes
		{
			Sig100_bad_receive(4);
			return false;
		}
		if(SIG100_checksum_input_pkt(SIG100_DIAG_PKT_LEN) == false)
		{
			Sig100_bad_receive(5);
			return false;
		}
		SIG100_handle_stop_flags();
		ADC_adjusted_temp = SIG100_input_packet[3];
		BL_enc_rpm = SIG100_input_packet[4];
		Set_encoder_pos(Get_uint16(SIG100_input_packet + 5),700);
//		Encoder_Pos = (int16_t)(SIG100_input_packet[5] | (SIG100_input_packet[6] << 8));
		BL_dac0_v   = SIG100_input_packet[7];
		SIG100_check_flags();
		return true;

	case '#':	// Error
		SIG100_rcv_pkt();
		Sig100_process_error_pkt();
		return true;

	case '>':	// Initialisation packet response;
		ci = SIG100_UART_GETC();
		if(ci < 0) return false;
		SIG100_input_packet[SIG100_packet_ix++] = (uint8_t) ci;
		switch(ci)
		{
		case 'i':
			return Sig100_verify_i_pkt();
		case 'j':
			return Sig100_verify_j_pkt();
		case 'I':	// This version includes creation time to identify
			return Sig100_verify_I_pkt();
		case 'k':
			return Sig100_verify_k_pkt();	// MHH:08/01/2026. Confirm I pkt received but only if sent.
		}
		return false;
	}
	return false;		// Should never get here

}
//----------------------------------------------------------------------------
void Put_int2(uint8_t *dst,int16_t v)
{
	*dst++ = v & 0xff;
	*dst   = (v >> 8);
}
//----------------------------------------------------------------------------
/*
 *  Format:
 *  	Offset		Desc
 *  	0			'!'		Packet header, data to follow
 *  	1			'I'		Type of data is Initial data
 *  	2			User define stop flags
 *  	3			Motor pole pairs (only used for soft FINE stop)
 *  	4-5			Hub user defined coarse stop, relative to fine stop in encoder increments
 *  	6-7			Hub user defined feather stop
 *  	8-9			Hub user defined reverse stop
 *  	10-11		Motor gear ratio (only used for soft FINE stop)
 *		12			spare
 *  	13			checksum
 *
 *cal_tdc_pos_offset
 *
*/

float Hub_blade_angle(float fpos)
{

	// fpos is the distance in mm (+ or -) from TDC position

	float result,radians,degrees;

	float sine_val = (fpos/Hub_calc.cam_radius);

	if(sine_val < -1.0) sine_val = -1;	// This should never happen
	if(sine_val > 1.0) sine_val = 1;

	radians = asinf(sine_val);
	degrees = radians * 180.0 / PI;
	result = Hub_calc.blade_offset + degrees;
	return result;
}
//------------------------------------------------------------------------------------------------------------
float Hub_blade_position(float fdegrees)     // Reverse of Hub_blade_angle()
{
	// fPos is the distance in mm (+ or -) from the ANGLE_TOP position

	float radians,degrees;
	float fpos,pos_over_radius;

	degrees = fdegrees - Hub_calc.blade_offset;     // relative to ANGLE_TOP
	radians = degrees * PI / 180.0;
	//    pos_over_radius = sin(radians);
	pos_over_radius = sinf(radians);
	fpos = pos_over_radius * Hub_calc.cam_radius;
	return fpos;
}
//------------------------------------------------------------------------------------------------------------
int Sig100_pos_from_angle(float angle)
{
	float pos_mm  = Hub_blade_position(angle);

	float encoder_pos = pos_mm / Hub_calc.hall_incr_mm;
	int enc_pos = iround(encoder_pos);
	enc_pos = Enc_data.cal_tdc_pos_offset - enc_pos;		// MHH:22/07/2023
	return enc_pos;
}
//------------------------------------------------------------------------------------------------------------
float Sig100_GetBladeAngle(int encoder_pos)
{
    float epos = (float)encoder_pos;
	float pos_mm = epos * Hub_calc.hall_incr_mm;

	pos_mm = Hub_calc.tdc_mm_offset - pos_mm;	// MHH:04/07/2023

	float blade_angle = Hub_blade_angle(pos_mm);
    return blade_angle;
}
int16_t Sig100_int16_get_blade_angle(int  pos)
{
	float angle = Sig100_GetBladeAngle(pos);
	// Now convert to int16 with one implied decimal point
	angle *= 10;
	int iangle = iround(angle);
	return (int16_t)iangle;
}

void Sig100_ini_hub_calc(void)
{
	float motor_ratio = Enc_data.cal_motor_ratio;
	float pole_pairs = Enc_data.cal_pole_pairs;
	float lead_screw_mm = Enc_data.cal_leadscrew_mm;

	motor_ratio /= 100;			// 2 implied dec places
	lead_screw_mm /= 1000;	// 3 implied decimal places

	Hub_calc.hall_incr_mm = lead_screw_mm / (motor_ratio * pole_pairs);

	float blade_offset = Enc_data.cal_blade_offset;
	if(blade_offset > 1000)	// MHH:24/05/2026
	{
		blade_offset /= 100;
	}
	else
	{
		blade_offset /= 10;			// 1 implied dec place
	}

	Hub_calc.blade_offset = blade_offset;
	float cam_length = Enc_data.cal_cam_length;
	if(cam_length > 1000)	// MHH:23/05/2026. All 2 dec places of accuracy.
	{
		cam_length /= 100;
	}
	else
	{
		cam_length /= 10;
	}
//	cam_length /= 10;			// 1 implied dec place
	Hub_calc.cam_radius = cam_length;
	Hub_calc.tdc_pos_offset = Enc_data.cal_tdc_pos_offset;	// MHH:04/07/2023
	Hub_calc.tdc_mm_offset  = Hub_calc.tdc_pos_offset * Hub_calc.hall_incr_mm;
	Hub_calc.fine_stop_angle = Sig100_int16_get_blade_angle(Enc_data.cal_fine_stop);
	Hub_calc.coarse_stop_angle = Sig100_int16_get_blade_angle(Enc_data.cal_coarse_stop);
	Hub_calc.feather_stop_angle = Sig100_int16_get_blade_angle(Enc_data.ud_feather_stop);
	Hub_calc.reverse_stop_angle = Sig100_int16_get_blade_angle(Enc_data.ud_reverse_stop);
}
//---------------------------------------------------------------------------------------------------
#define SIG100_j_PKT_LEN 4
// This should be the only routine that initialises Enc_data
void Sig100_copy_Hub_data_to_Enc_data(int data_len)
{
	uint8_t *dst_p = (uint8_t *)&Enc_data;		// Do we need & ?
	for(int i=0;i<data_len;i++)
	{
		*dst_p++ = Hub_data[i];
	}
	Sig100_ini_hub_calc();
}
//----------------------------------------------------------------------------
#define HUB_UD_COARSE_STOP		2
#define HUB_UD_FEATHER_STOP		4
#define HUB_UD_REVERSE_STOP		8
int Get_SIG100_command_from_drive_pins(void);
int Get_SIG100_command_from_dState(void);
#ifdef MH_XXX
void Start_RC_calibration(void)		// MHH:07/01/2025
{
	if(currentActualSpeed() > 0) return;

	WORD ac200_switches = p_GetAC200_Switches(false);
	if((ac200_switches & 5) != 5) return;	// Must be MANUAL+HOLD

	if(Hub_calibrate_state >= CAL_START) return;	// Already started

	Hub_calibrate_state = CAL_START;
	Hub_calibrate_type = 1;			// This will tell hub not a PC requested calibrate
	Hub_calibrate_checkval = 12345;	// For safety
	LEDS_manual_calibration_status = 1;	// Feather LED should blink orange twice every 2 secs until finished.
	L2PRINTF("Remote Control calibration started\r\n");

	// Note: May need a way to know that calibration has finished. Perhaps a RC query, or perhaps (say) if calibrate type = RC, then when finished send

}
#endif
void Check_manual_calibration(void)
{
//	BYTE mankeys = manualKeys();


	if(TimeInSeconds > 10) return;
	if(currentActualSpeed() > 0) return;

	WORD ac200_switches = p_GetAC200_Switches(false);
	if((ac200_switches & 5) != 5) return;	// Must be MANUAL+HOLD

	BYTE mankeys = ac200_switches >> 6;
	if(Hub_calibrate_state >= CAL_START) return;	// Already started
//	if(mankeys != 0) DPRINTF("Mankeys:%d\r\n",mankeys);
	if((mankeys & MANUAL_KEY_FEATHER) && (mankeys & MANUAL_KEY_FINE))
	{
		Hub_calibrate_state = CAL_START;
		Hub_calibrate_type = 1;			// This will tell hub not a PC requested calibrate
		Hub_calibrate_checkval = 12345;	// For safety
		LEDS_manual_calibration_status = 1;	// Feather LED should blink orange twice every 2 secs until finished.
		L2PRINTF("Manual calibration started\r\n");
	}
}
bool AC200_slider_active = false;
bool AC200_slider_beta_100_percent = false;
WORD AC200_slider_beta_percent=200;
WORD AC200_slider_percent;
int AC200_slider_angle;
extern float RC_S_target_angle;
uint8_t Hub_manual_fine_speed_count=0;		// MHH:05/02/2025
uint8_t Hub_manual_coarse_speed_count=0;
char Get_manual_command(void)
{
	char c = '.';
	BYTE mankeys = manualKeys();
	WORD current;
	WORD state;

	if(mankeys & MANUAL_KEY_FEATHER)
	{
		if(mankeys & MANUAL_KEY_COARSE)
		{
			mh_debug();
		}
	}
	switch(Control_type)
	{
	case CT_BETA:
		if(dState == MD_BETA)
		{
			if(SIG100_beta_mode_on())
			{
	//			resetStateBits (12345,S_STOP_FINE);	// This is done in sstate.c:progressTest for brushed.

				if(AC200_slider_active == false)
				{
					if(AC200_slider_percent == 100)
					{
						AC200_slider_beta_100_percent = true;
					}
					if(AC200_slider_beta_100_percent)
					{
						if(AC200_slider_percent <= 95)
						{
							AC200_slider_active = true;
						}
					}
				}
				if(AC200_slider_active)
				{
					float angle = AC210_SIG100_angle();
					float target_angle = (float)AC200_slider_angle;
					target_angle /= 10;
					float diff = angle - target_angle;
					if(diff > 0.05 || diff < -0.05)
					{
						RC_S_target_angle = target_angle;
						c = 'p';
						return c;
					}
				}

				if(mankeys & MANUAL_KEY_COARSE)
				{
					c = '+';
					AC200_slider_active = false;
					AC200_slider_beta_100_percent = false;
				}
				else
				{
					if(mankeys & MANUAL_KEY_FINE)
					{
						c = 'R';	// Reverse
						resetStateBits(2100,S_RUN_FINE);	// This is done in updateSystemState() in brushed
//						setStateBits(12345,S_RUN_FEATHER);
						setStateBits(2200,S_RUN_REVERSE);	// MHH:15/07/2025
						AC200_slider_active = false;
						AC200_slider_beta_100_percent = false;
					}
				}
			}
			return c;
		}
		else
		{
			AC200_slider_active = false;
			AC200_slider_beta_100_percent = false;

		}
		if(dState == MD_BETA_EXIT)	// Strangely, updateOperatingMode() keeps opMode in MANUAL until beta exit finished.
		{
			c = '+';
			return c;
		}
		break;

	case CT_REVERSE:
		if(mankeys & MANUAL_KEY_FEATHER)
		{
			if(mankeys & MANUAL_KEY_FINE)
			{
				c = 'R';
				return c;
			}
		}
		break;

	case CT_FEATHERING:
		if(mankeys & MANUAL_KEY_FEATHER)
		{
			if(mankeys & MANUAL_KEY_COARSE)
			{
				if(Hub_pos_error)
				{
					if(BL_feather_when_error)
					{
						c = '.';
						return c;
					}
					state = BL_flags << 4;
					if (state & S_STOP_COARSE)
					{
						current = scaledValue (A_MOTOR_CURRENT);
#ifdef MH_XXX
						if(current != Feather_current)
						{
							Feather_current = current;
							if(current > 0)		// Debug only
							{
								DPRINTF("FC=%d\r\n",current);
							}
						}
#endif

						if(current > 2500)	// Same as Hub uses for calibrating
						{

							BL_feather_when_error = true;
							c = '.';
							return c;
						}
					}
				}
				c = 'F';
				return c;
			}
		}
		break;
	}

//#define MH_TEST_MODE
#ifdef MH_TEST_MODE
#define MANUAL_COUNT_LIMIT		150

	if(mankeys & MANUAL_KEY_COARSE)
	{
		c = '+';		// Default
		Hub_manual_fine_speed_count=0;	// MHH:05/02/2025
		if(Hub_manual_coarse_speed_count < MANUAL_COUNT_LIMIT)	// One second
		{
			Ctl_brushless_speed = '1';		// start at 50% speed
//			Ctl_brushless_speed = '5';		// start at 50% speed
			Hub_manual_coarse_speed_count++;
		}
		else // Test!!!
		{
			Ctl_brushless_speed = '5';		// start at 50% speed
		}
	}
	else
	{
		if(mankeys & MANUAL_KEY_FINE)
		{
			c = '-';
			BL_feather_when_error = false;
			Hub_manual_coarse_speed_count=0;	// MHH:05/02/2025
			if(Hub_manual_fine_speed_count < MANUAL_COUNT_LIMIT)	// One second
			{
				Ctl_brushless_speed = '2';		// start at 50% speed
//				Ctl_brushless_speed = '5';		// start at 50% speed
				Hub_manual_fine_speed_count++;
			}
			else	// Test!!!!
			{
				Ctl_brushless_speed = '5';		// start at 50% speed
			}

		}
	}
#else
#define MANUAL_COUNT_LIMIT		50
//#define MANUAL_COUNT_LIMIT		100
#define MANUAL_LOW_SPEED		'5'
	uint8_t manual_low_speed = 128 + 50;
	if(Enc_data.hub_software_version < 151) manual_low_speed = '5';
	if(mankeys & MANUAL_KEY_COARSE)
	{
		c = '+';		// Default
		Hub_manual_fine_speed_count=0;	// MHH:05/02/2025
		if(Hub_manual_coarse_speed_count < MANUAL_COUNT_LIMIT)	// One second
		{
//			Ctl_brushless_speed = '5';		// start at 50% speed
			Ctl_brushless_speed = manual_low_speed;		// start at 50% speed
			Hub_manual_coarse_speed_count++;
		}

	}
	else
	{
		if(mankeys & MANUAL_KEY_FINE)
		{
			c = '-';
			BL_feather_when_error = false;
			Hub_manual_coarse_speed_count=0;	// MHH:05/02/2025
			if(Hub_manual_fine_speed_count < MANUAL_COUNT_LIMIT)	// One second
			{
//				Ctl_brushless_speed = '5';		// start at 50% speed
				Ctl_brushless_speed = manual_low_speed;		// start at 50% speed
				Hub_manual_fine_speed_count++;
			}
		}
	}
#endif
	if(Manual_position_encoder)
	{
		if(c != '.')
		{
			Manual_position_encoder = false;
		}
		else
		{
			c = 'p';		// position.
		}
	}
	return c;
}
//----------------------------------------------------------------------------------------------------
char RC_S_position_request(void);
#ifdef MH_XXX	// MHH:05/04/2024
void SIG100_send_position_request(char c)
{
/*
 *  First job is to convert to hub encoder position
 *
 *  Format:
 *  Byte	Contents
 *  0		'P'
 *  1		low order hub_pos
 *  2		high order hub_pos
 *  3		xor byte 1
 *  4		xor byte 2
*/

	int ac200_pos = Sig100_pos_from_angle(RC_S_target_angle);
//	int16_t hub_pos = ac200_pos - Sig100_hub.fine_stop_offset;	// Relative to fine stop
	int16_t hub_pos = ac200_pos;
	Sig100_send_pkt[0] = c;	// Either 'P' for normal position, or 'p' for reverse position
	Put_int2(Sig100_send_pkt+1,hub_pos);
	Sig100_send_pkt[3] = ~Sig100_send_pkt[1];
	Sig100_send_pkt[4] = ~Sig100_send_pkt[2];
	Sig100_send_output_pkt(5);

	// We could just send this once, unless the angle changes. If we did, then we would need to save the position at the hub, plus verify that the
	// pkt was received correctly. Keep it simple for now....
}
#endif
//----------------------------------------------------------------------------------------------------
void AC200_SIG100_putstr(char *src);
#define COMMAND_PREFIX	'|'
#define SIG100_i_PKT_LEN 5
#define SIG100_k_PKT_LEN 7
//uint8_t BL_command;
uint8_t BL_speed;
uint8_t Hub_diagnostics_count;

#ifdef MH_XXX
void Sig100_send_count(void)
{
	AC200_SIG100_putchar((char)BL_count++);
}
#endif
//bool Sig100_send_current;	// Could make this a parameter value, changing by keyword only (not AC200User)...
ULONG Time_in_secs_save;
#define COMMAND_LEN		20
char Command[COMMAND_LEN];
char Command_prev[COMMAND_LEN];
uint8_t Command_len;
int Command_dups;
uint32_t Command_time_in_ticks;
//TimeInTicks;
void SIG100_start_command(void)
{
	AC200_SIG100_putchar(COMMAND_PREFIX);
	Command_len = 0;
	uint32_t ticks = TimeInTicks - Command_time_in_ticks;
	Command_time_in_ticks = TimeInTicks;
	if(ticks > 2)
	{
//		DPRINTF("[Start_command:ticks = %d]\r\n",ticks);
	}
}
void SIG100_command_putchar(char c)
{
	AC200_SIG100_putchar(c);
	Command[Command_len++] = c;
}
void SIG100_command_putchar2(char c)	// Don't send
{
//	AC200_SIG100_putchar(c);
	Command[Command_len++] = c;
}
void SIG100_command_putstr(char *src)
{
	char c;
	for(;;)
	{
		c = *src++;
		if(c == 0) return;
		AC200_SIG100_putchar(c);
		if(Command_len < COMMAND_LEN)
		{
			Command[Command_len++] = c;
		}
	}
}
void SIG100_command_putstr2(char *src)	// Dont send
{
	char c;
	for(;;)
	{
		c = *src++;
		if(c == 0) return;
//		AC200_SIG100_putchar(c);
		if(Command_len < COMMAND_LEN)
		{
			Command[Command_len++] = c;
		}
	}
}

void SIG100_debug_command(void)
{
//	if(strcmp(Command,Command_prev) || ((Command_dups > 10) && Command[0] != '.'))	// New command?
	bool print_command = true;
	if(Command[0] == '.')
	{
		if(Command_prev[0] == '.')
		{
			print_command = false;
		}
	}
	if(print_command)
	{
		PRINTF("C:%s\r\n",Command)
	}
	strcpy(Command_prev,Command);
#ifdef MH_XXX
	if(strcmp(Command,Command_prev))
	{
		if(Command_dups)
		{
			DPRINTF("[Dups:%d]\r\n",Command_dups);
			Command_dups = 0;
		}
		DPRINTF("[Command:%s]\r\n",Command);
		strcpy(Command_prev,Command);
	}
	else
	{
		Command_dups++;
	}
#endif
}

void SIG100_end_command(void)
{
	Command[Command_len++] = 0;	// Append null

#ifdef MH_DEBUG_COMMAND	// MHH:13/11/2024
	SIG100_debug_command();
#endif
}
void SIG100_send_dot_command(void)
{
#ifdef MH_YYY	// MHH:15/10/2025
	char ticks = timerTicks();
	AC200_SIG100_putchar('#');
	AC200_SIG100_putchar(ticks);
	SIG100_end_command();
	Command_len = 0;
	Command[Command_len++] ='#';
	SIG100_end_command();
#else
	AC200_SIG100_putchar('.');
	Command_len = 0;
	Command[Command_len++] ='.';
	SIG100_end_command();
#endif

}
void SIG100_send_movement_pkt(char c)
{
	Sig100_send_pkt[0] = c;	// movement type
	uint8_t speed = Ctl_brushless_speed;
	if(Enc_data.hub_software_version < 151)
	{
		if(speed == 0) speed = ('9' + 1);
	}
	Sig100_send_pkt[1] = speed;
	Sig100_checksum_and_send_pkt(3);

}
void SIG100_send_command_pkt(char c)
{
//	uint8_t bl_last_command = BL_command;
	BL_command = (uint8_t)c;		// For logging...
	BL_speed = Ctl_brushless_speed;


	if(ps.parms[BL_DIAGNOSTICS] & P_BL_ACCEL_BOARD)	// MHH:18/10/2023. Add for vibration logic
	{
		if(TimeInSeconds != Time_in_secs_save)
		{
			Time_in_secs_save = TimeInSeconds;
			c = 'V';			// Send a vibration command instead of what we are doing
		}
	}

/*
 *
 *  The idea is to allow some AC200 hub functionality if the hub diagnostics option is set. After an AC200 command, 10 '.' pkts are sent, then
 *  it will send 'A' (which contain current) type packets instead of '.'Set_dState
 *
 *
 */

//	if((ps.parms[BL_DIAGNOSTICS] & P_BL_DIAG_ON) || (ps.parms[BL_DIAGNOSTICS] & P_BL_ACCEL_BOARD))	// MHH:18/10/2023. Add for vibration logic
	if(ps.parms[BL_DIAGNOSTICS] & (P_BL_DIAG_ON | P_BL_ACCEL_BOARD))	// MHH:18/10/2023. Add for vibration logic
	{
		if(c == '.')		// Most common
		{
			c = 'A';
		}
#ifdef MH_XXX		// MHH:18/10/2023. Have changed hub so that an 'A' disables motor
			if(Hub_diagnostics_count == 0)
			{
				c = 'A';
			}
			else
			{
				Hub_diagnostics_count--;
				AC200_SIG100_putchar(c);
				return;
			}
		}
		else
		{
			Hub_diagnostics_count = 10;		// Arbitrary, enough to ensure at least 1 '.' received
		}
#endif
	}
	else
	{
		if(c == '.')		// Most common
		{

			SIG100_send_dot_command();
//			AC200_SIG100_putchar(c);
			return;
		}
	}
	SIG100_start_command();
//	AC200_SIG100_putchar(COMMAND_PREFIX);
	int16_t hub_pos;
	WORD current;
	switch(c)
	{
	default:
		PRINTF("SIG100_send_command_pkt:Unknown command:%c",c);
		return;

	case '+':
		if(Enc_data.hub_software_version <= 147)	// MHH:16/12/2025
		{
			SIG100_command_putstr("+p");
			if(Ctl_brushless_speed)
			{
				SIG100_command_putchar(Ctl_brushless_speed);
			}
			SIG100_end_command();
		}
		else
		{
			SIG100_send_movement_pkt(c);
		}
		return;

	case '-':
		if(Enc_data.hub_software_version <= 147)	// MHH:16/12/2025
		{
			SIG100_command_putstr("-m");
			if(Ctl_brushless_speed)
			{
				SIG100_command_putchar(Ctl_brushless_speed);
			}
			SIG100_end_command();
		}
		else
		{
			SIG100_send_movement_pkt(c);
		}
		return;


	case 'A':		// MHH:30/06/2023. amps, used for testing.
		SIG100_command_putstr2("A");
		Sig100_send_pkt[0] = 'A';
		Sig100_send_pkt[1] = 0;						// Could have 4 bits for sub command (eg calibrate) and 4 bits to show last type received
		current = scaledValue (A_MOTOR_CURRENT);	// There may be a way of getting more immediate value...
		Put_int2(Sig100_send_pkt+2,current);
		Sig100_checksum_and_send_pkt(5);
		SIG100_end_command();
		return;

	case 'C':
		SIG100_command_putstr("C!mode=1");
//		AC200_SIG100_putstr("C!mode=1");
		SIG100_end_command();
		return;

	case 'F':		// Feather
		SIG100_command_putstr("Fe");
		SIG100_end_command();
//		AC200_SIG100_putstr("Fe");
		return;

	case 'H':		// Heater
		SIG100_command_putstr("Ht");
		SIG100_end_command();
//		AC200_SIG100_putstr("Ht");
		return;

	case 'i':		// Initialisation
		Sig100_send_pkt[0] = 'i';	// Tell hub some new arguments coming
		Sig100_send_pkt[1] = 'I';	// Initial data
		Put_int2(Sig100_send_pkt+2,(WORD)AC300_get_run_number());	// MHH:05/07/2025
#ifdef MH_YYY
		Put_int2(Sig100_send_pkt+4,version);						// MHH:16/12/2025
		Sig100_checksum_and_send_pkt(SIG100_i_PKT_LEN+2);			// MHH:16/12/2025
#else
		Sig100_checksum_and_send_pkt(SIG100_i_PKT_LEN);			// MHH:08/01/2026
#endif
		SIG100_command_putstr2("i");
		SIG100_end_command();
		return;

	case 'j':
		Sig100_send_pkt[0] = 'j';	// Tell hub we want Enc_data  (128 bytes)
		Sig100_send_pkt[1] = 'J';	// Initial data
		Sig100_checksum_and_send_pkt(3);
		SIG100_command_putstr2("j");
		SIG100_end_command();
		return;


	case 'k':
		Sig100_send_pkt[0] = 'k';	// Only send this if hub version > 149
		Sig100_send_pkt[1] = 'K';	// Initial data
		Put_int2(Sig100_send_pkt+2,version);					// MHH:08/01/2026
		Sig100_send_pkt[4] = 0;		// Reserved for future use
		Sig100_send_pkt[5] = 0;		// Reserved for future use
		Sig100_checksum_and_send_pkt(SIG100_k_PKT_LEN);			// MHH:08/01/2026
		SIG100_command_putstr2("k");
		SIG100_end_command();
		return;


	case 'R':		// Reverse?
		SIG100_command_putstr("Rv");
		SIG100_end_command();
//		AC200_SIG100_putstr("Rv");
		return;

	case 'p':		// Position command
	case 'P':

		if(Manual_position_encoder)
		{
			hub_pos = Encoder_target_pos;
		}
		else
		{
			hub_pos = (uint16_t)Sig100_pos_from_angle(RC_S_target_angle);
		}
		DPRINTF("T=%d\r\n",hub_pos);
		Sig100_send_pkt[0] = c;	// Either 'P' for normal position, or 'p' for reverse position
		Put_int2(Sig100_send_pkt+1,hub_pos);
		Sig100_send_pkt[3] = ~Sig100_send_pkt[1];
		Sig100_send_pkt[4] = ~Sig100_send_pkt[2];
		Sig100_send_output_pkt(5);
		SIG100_command_putchar2(c);
		SIG100_end_command();
		return;


/*
 * typedef struct
{
	uint16_t ac200_secs;	// 0
	uint16_t prop_rpm;		// 2
} AC200_v_data_td;			// 4.
 */
#ifdef MH_XXX		// MHH:18/05/2026
	case 'V':		// Vibration command
		Sig100_send_pkt[0] = 'V';
		Sig100_send_pkt[1] = 'i';
		uint16_t seconds = (uint16_t)TimeInSeconds;
		Put_int2(Sig100_send_pkt+2,seconds);
		uint16_t prop_rpm = (uint16_t)Prop_rpm;
		Put_int2(Sig100_send_pkt+4,prop_rpm);
		Sig100_checksum_and_send_pkt(7);
		SIG100_command_putchar2('V');
		SIG100_end_command();
		return;
#endif

	case 'X':	// Calibration command
		Sig100_send_pkt[0] = 'X';
		Sig100_send_pkt[1] = 'c';
		Sig100_send_pkt[2] = (uint8_t)Hub_calibrate_state;
		if(Hub_calibrate_state == CAL_START)
		{
			Sig100_send_pkt[3] = Hub_calibrate_pole_pairs;
			Sig100_send_pkt[4] = Hub_calibrate_type;	// MHH:25/08/2023
		}
		else
		{
			current = scaledValue (A_MOTOR_CURRENT);	// There may be a way of getting more immediate value...
			Put_int2(Sig100_send_pkt+3,current);
		}
		Sig100_checksum_and_send_pkt(6);
		SIG100_command_putchar2('X');
		SIG100_end_command();
		return;
	}
}

//----------------------------------------------------------------------------
void SIG100_send_init(void)
{
#ifdef MH_XXX
	switch(Sig100_init_status)
	{
	case 0:
		SIG100_send_command_pkt('i');
		return;
	case 2:
		SIG100_send_command_pkt('j');
		return;

	default:
		return;		// Should never get here
	}
#endif
	if(Sig100_hub_link) return;		// MHH:09/06/2025

	if((Sig100_init_status & SIG100_INI_I_LOADED) == false)
	{
		SIG100_send_command_pkt('i');
		return;
	}
	if((Sig100_init_status & SIG100_INI_J_LOADED) == false)
	{
		SIG100_send_command_pkt('j');
		return;
	}
	if((Sig100_init_status & SIG100_INI_K_LOADED) == false)
	{
		SIG100_send_command_pkt('k');
		return;
	}



// Add K code here...

}
//------------------------------------------------------------------------------------------------------------------
uint8_t Hub_heater_threshhold_adc8;
bool Hub_manual_mode = false;
char Get_SIG100_speed_from_remote(void);
void SIG100_send_command(void)
{
	char c;
//	Sig100_init_status = 2;	// Just until we have coded AC200hubBL...


	if(Sig100_hub_link) return;		// Not sure if it will get here

//	if(Sig100_init_status != 3)	// Have we initialised?
	if(Sig100_init_status != SIG100_INI_ALL_LOADED)	// Have we initialised?
	{
#ifdef MH_GET_ENC_SPEED
		if(Sig100_init_status == 0)
		{
			Timer_start(&Enc_speed_timer);
		}
#endif
		SIG100_send_init();
		return;
	}

    OpMode op_mode = operatingMode();
    if(op_mode == MANUAL)
    {
    	Check_manual_calibration();
    }

	if(Hub_calibrate_checkval == 12345)
	{
		Hub_calibrate_check();
		if(Hub_calibrate_state == CAL_IDLE) return;

		SIG100_send_command_pkt('X');
		return;
	}
	if(Hub_return_position)
	{
		Hub_return_pos();
	}

//    OpMode op_mode = operatingMode();
//    if(op_mode == HOLD)
//#define MANUAL_SPEED_FAST		0
	uint8_t manual_speed_fast = 128 + 100;
	if(Enc_data.hub_software_version < 151) manual_speed_fast = 0;
//#define MANUAL_SPEED_FAST		'3'
    if(op_mode == MANUAL)
    {
//    	Ctl_brushless_speed = 0;		// MHH:05/02/2025. Default. May be changed in Get_manual_command();'H'
    	Ctl_brushless_speed = manual_speed_fast;		// MHH:05/02/2025. Default. May be changed in Get_manual_command();
    	if(Hub_manual_mode == false)	// MHH:05/02/2025
    	{
    		Hub_manual_mode = true;
    		Hub_manual_fine_speed_count=0;
    		Hub_manual_coarse_speed_count=0;
    	}
    	c = Get_manual_command();
    	if(c != '+' && c != '-')
    	{
    		Hub_manual_fine_speed_count=0;
    		Hub_manual_coarse_speed_count=0;
    	}
//    	Ctl_brushless_speed = '2';	// MHH:12/11/2024
    }
    else
    {
    	Hub_manual_mode = false;;
    	if (overCurrentTrip)	// MHH:24/07/2023
    	{
    		c = '.';
    	}
    	else
    	{
    		//		c = (char) Get_SIG100_command_from_drive_pins();
    		if(op_mode == HOLD)
    		{
    			char c = RC_S_position_request();
    			if(c)
    			{
    				SIG100_send_command_pkt(c);
    				return;
    			}
//    			Only if in Ground Mode????
//

    			char remote_speed = Get_SIG100_speed_from_remote();
    			if(remote_speed != 255)
    			{
        			if(remote_speed > 0)
        			{
// Leave as is as hub should take either format
        				remote_speed += '0';
        				Ctl_brushless_speed = remote_speed;
        			}
        			else
        			{
        				Ctl_brushless_speed = 0;		// For debug
        			}
    			}
    		}
    		c = (char) Get_SIG100_command_from_dState();
    		if(c == 'F')	// Feather?
    		{
//    	    	if(Hub_pos_error == 0)
    	    	{
//    	    		resetStateBits (310,S_STOP_COARSE);	// MHH:21/07/2025. We need this bit to decide when to exit reverse. So that LED display matches brushed feather
    	    	}
    		}
    	}
    }
	if(c == '.')
	{
		if(getParameter (BL_HEATER_ENABLED) == 1)
		{
			if(Hub_heater_threshhold_adc8 == 0)	// First time only
			{
				int celcius_threshhold = get_signedParameter(BL_HEATER_START_TEMPERATURE);
				int mv = LMT87_convert_celcius_to_millivolts(celcius_threshhold);
				// Still have to get from millivolts to adcv/16

				Hub_heater_threshhold_adc8 = (mv <<8)/3300;	// Covert from mv to adc value
			}
			if(ADC_adjusted_temp != 0)
			{
				if(ADC_adjusted_temp > Hub_heater_threshhold_adc8)	// works in reverse
				{
					c = 'H';	// We want some heat. Note: Could make (say) "Ht" to be sure
					// Also, if we are using 24V, may want to alternate, or PWM in hub?
				}
			}
		}
	}
	SIG100_send_command_pkt(c);
	// Note, we can add an argument to modify speed if necessary.
}
//----------------------------------------------------------------------------
void SIG100_check_zone(void)
{
/*
 * This is just so that when reversing the correct LED display is made. In brushed, the zone  is checked in progressTest()
 */

	if(Control_type != CT_REVERSE) return;

	// May apply to CT_BETA too?

//	if(Encoder_Pos < -(Sig100_hub.pole_pairs * 50))	// How far from fine stop (zero) are we.
//	if(Encoder_Pos < (Enc_data.cal_fine_stop -(Enc_data.cal_tolerance * 10)))	// How far from fine stop (zero) are we.
	if(Encoder_Pos > (Enc_data.cal_fine_stop +(Enc_data.cal_tolerance * 100)))	// MHH:28/07/2023.
	{
		Set_Zone(RC_S_ZONE_REVERSE);
	}
	else
	{
		Set_Zone(RC_S_ZONE_NORMAL);
	}
}
//----------------------------------------------------------------------------
#define SIG100_TICK_TIMEOUT	100		 // Check if still connected
int SIG100_ini_count;
bool SIG100_comms_error;
extern WORD currentState;
bool Hublink_command_sent;

void Sig100_clear_hub_serial(void)
{
	int ci;
	for(;;)
	{
		ci = SIG100_UART_GETC();
		if(ci < 0) return;
	}
}
bool AC200_send_sig100;
BYTE AC200_slider_percent_count;
WORD AC200_slider_value;
WORD AC200_slider_percent10;
//WORD AC200_slider_pos;

#define BL_ANGLE_TBL_MAX	10
uint8_t BL_angle_ix = 0;
float BL_angle_t[BL_ANGLE_TBL_MAX];
float BL_delta_angle;
void AC210_SIG100_get_delta_angle(void)
{
	static float last_delta_angle;
	if(Sig100_connected == false) return;

	float angle = AC210_SIG100_angle();		// Quick and dirty way of finding if delta_rpm being caused by change in angle.
	float oldest_angle = BL_angle_t[BL_angle_ix];
	BL_angle_t[BL_angle_ix++] = angle;
	BL_angle_ix %= BL_ANGLE_TBL_MAX;

	BL_delta_angle = angle - oldest_angle;

	if(BL_delta_angle != last_delta_angle)
	{
//		DPRINTF("DeltaA:%3.2f\r\n",BL_delta_angle);
		last_delta_angle = BL_delta_angle;
	}
}

void AC210_SIG100_check(void)
{
	static uint8_t timeout_count;
	AC200_send_sig100 = false;		// MHH:23/05/2023
	if(Sig100_hub_link)
	{
		if(Hublink_command_sent == false)
		{
			SIG100_send_command_pkt('C');
			Hublink_command_sent = true;	// Puts Hub into line based command mode
		}
		return;
	}

	if(EEPROM_writes > 0) return;	// MHH:18/10/2022

	if(SIG100_comms_error)	return;	// MHH:18/11/2022. Quick fix, otherwise cannot connect AC200User.


#ifdef MH_XXX	// This parameter controls UART2, not UART3.
	if(getParameter(AUX_PORTS_SERIAL_CTL) != AUX_PORT_SIG60)	// To be sure
	{
		return;
	}
#endif
	if(getParameter (BL_ENABLED) != 1) return;


	if(Sig100_connected)
	{
		resetStateBits(2400,S_ERROR_OPEN);
	}
	else
	{
		currentState |= S_ERROR_OPEN;
	}

	if(SIG100_check_packet())
	{
		SIG100_good_receives_per_sec++;
		Sig100_bad_receive_code = 0;
		Sig100_connected = true;
		timeout_count = 0;
//		SIG100_process_packet();
//		SIG100_send_command();
		AC200_send_sig100 = true;
	}
	else
	{
		SIG100_bad_receives_per_sec++;
//		SIG100_send_command();
		AC200_send_sig100 = true;
//		Sig100_UARTPutChar('.');
		if(timeout_count++ > SIG100_TICK_TIMEOUT)
		{
			timeout_count = 0;
			Sig100_connected = false;
			// Serious error if we get here....
			SIG100_error(101);
		}
	}
	if(timerTick == 0)
	{
		if(SIG100_display == 1)	// This is turned on from comms.c
		{
			PRINTF("SIG100:Good:%d,Bad:%d\r\n",SIG100_good_receives_per_sec,SIG100_bad_receives_per_sec);
		}
		SIG100_line_percent = SIG100_good_receives_per_sec *2;
		SIG100_good_receives_per_sec = 0;
		SIG100_bad_receives_per_sec = 0;
	}
	if(getParameter(SLIDER_CONTROL_ENABLE) == 1)
	{
		WORD v2 = scaledValue (A_SLIDER_CONTROL);
		AC200_slider_value = v2;	// Value between 0 and 1023
		WORD slider_value = v2;

		WORD slider_control_max_value = getParameter(SLIDER_CONTROL_MAX_VALUE);
		WORD slider_control_min_value = getParameter(SLIDER_CONTROL_MIN_VALUE);
		int slider_control_max_angle = getParameter(SLIDER_CONTROL_MAX_ANGLE);
		int slider_control_min_angle = getParameter(SLIDER_CONTROL_MIN_ANGLE);
		if(slider_control_max_angle > 180) slider_control_max_angle -= 360;
		if(slider_control_min_angle > 180) slider_control_min_angle -= 360;

		int slider_maxval = MAX(slider_control_max_value,slider_control_min_value);
		int slider_minval = MIN(slider_control_max_value,slider_control_min_value);

		if(slider_value > slider_maxval) slider_value = slider_maxval;
		if(slider_value < slider_minval) slider_value = slider_minval;
		int slider_value_range = slider_control_max_value - slider_control_min_value;	// May be negative
		int slider_angle_range = slider_control_max_angle - slider_control_min_angle;
		int new_slider_percent;
		if(slider_value_range > 0)
		{
			new_slider_percent = (slider_value - slider_control_min_value) * 1000 / slider_value_range;
		}
		else
		{
			new_slider_percent = (slider_control_min_value - slider_value) * 1000 / (-slider_value_range);
		}

		bool update_slider_percent = false;
		if(new_slider_percent >= 1000) update_slider_percent = true;
		if(new_slider_percent == 0) update_slider_percent = true;
		if(update_slider_percent == false)
		{
			if(new_slider_percent < AC200_slider_percent10 - 5 || new_slider_percent > AC200_slider_percent10 + 5) // Try and stop jitter
			{
				if(AC200_slider_percent_count++ > 5)
				{
					update_slider_percent = true;
				}
			}
			else
			{
				AC200_slider_percent_count = 0;
			}
		}
		if(update_slider_percent)
		{
			AC200_slider_percent10 = new_slider_percent;
			AC200_slider_percent = new_slider_percent / 10;
			int new_slider_angle = slider_control_min_angle * 10 + (AC200_slider_percent * slider_angle_range)/10;	// In tenths of a degree
			AC200_slider_angle = new_slider_angle;
		}

		//    	PRINTF("AC200SCP=%d\r\n",v2)
	}
}
//------------------------------------------------------------------------------------------
void AC210_SIG100_send(void)
{
	if(AC200_send_sig100)
	{
		AC200_send_sig100 = false;
		SIG100_send_command();
	}
}
//------------------------------------------------------------------------------------------
float AC210_SIG100_angle(void)	// Called from remote.c
{
	float angle = Sig100_GetBladeAngle(Encoder_Pos);
	return angle;
}
