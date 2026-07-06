/*
 * xoar.h
 *
 *  Created on: 27/06/2019
 *      Author: Murray
 */

#ifndef XOAR_H_
#define XOAR_H_

#define X2_SEND_PKT_LEN		12
#define X2_INSTRUCTION_CODE_AUTO_SPEED		'A'
#define X2_INSTRUCTION_CODE_MAN_COARSE		'B'
#define X2_INSTRUCTION_CODE_MAN_FINE		'C'
#define X2_INSTRUCTION_CODE_FEATHER			'D'
#define X2_INSTRUCTION_CODE_SET_ANGLE		'E'
#define X2_INSTRUCTION_CODE_REVERSE			'F'
#define X2_INSTRUCTION_CODE_RESET_OVERCURRENT	'G'
#define X2_INSTRUCTION_CODE_SET_DEICE		'H'

#define XOAR_ERROR_BAD_CHECKSUM			1
#define XOAR_ERROR_CODES_NOT_SAME		2
#define XOAR_ERROR_PARAMETERS_NOT_SAME 	3
#define XOAR_ERROR_INVALID_CODE			4
#define XOAR_ERROR_INVALID_PARAMETER	5
#define XOAR_ERROR_NOT_FEATHERING_TYPE	6
#define XOAR_ERROR_ALREADY_FEATHERING	7
#define XOAR_ERROR_NOT_HOLD_MODE		8
#define XOAR_ERROR_NOT_FEATHER_STATE	9
#define XOAR_ERROR_IN_FEATHER_STATE		10
#define XOAR_ERROR_CODE_NOT_IMPLEMENTED	11
#define XOAR_ERROR_NOT_REVERSING_TYPE	12
#define XOAR_ERROR_ALREADY_REVERSING	13
#define XOAR_ERROR_NOT_REVERSE_STATE	14
#define XOAR_ERROR_IN_REVERSE_STATE		15
#define XOAR_ERROR_NO_CURRENT_OVERLOAD	16
#define XOAR_ERROR_CURRENT_OVERLOAD		17
#define XOAR_ERROR_DEICER_NOT_ENABLED	18
#define XOAR_ERROR_RPM_GT_BETA_MAX_ENGAGE 19
#define XOAR_ERROR_RPM_GT_BETA_MAX_RUN 	20


//#if XOAR_VERSION > 0
#if MH_XOAR_DEBUG > 0
#define XOAR_PACKET_TIMEOUT	100		// 2 secs
#else
#define XOAR_PACKET_TIMEOUT	5
#endif

struct XOAR_RCV_PKT_struct
{
    BYTE header[2];
    WORD rpm_command;
    WORD rpm_current;
    BYTE status_gear;
    BYTE status_controller;
    BYTE reserved;
    BYTE checksum;
};
struct XOAR_SND_PKT_struct
{
    BYTE header[2];
    WORD rpm_command[3];
    BYTE reserved;
    BYTE checksum;
};

#if defined(__GNUC__)   // LPCXpresso Tools
//  #define PRE_PACK
  #define POST_PACK     __attribute__((packed))
#endif
struct XOAR2_RCV_PKT_struct
{
    BYTE header[2];			//0-1
    WORD rpm_command;		//2-3
    WORD rpm_current;		//4-5
    BYTE status_gear;		//6
    BYTE status_controller;	//7
    WORD voltage;			//8-9
    BYTE current;			//10
    WORD angle;				//11-12 Need to check if word alignment problem
    BYTE last_error;		// 13
    BYTE spare;				//14
    BYTE checksum;			//15
}POST_PACK;
struct XOAR2_SND_PKT_struct
{
    BYTE header[2];					//0-1
    BYTE instruction_code[3];		//2-4
    WORD instruction_parameter[3];	//5-10
    BYTE checksum;					//11
}POST_PACK;

typedef struct XOAR_SND_PKT_struct XOAR_SND_PKT_t;
typedef struct XOAR_RCV_PKT_struct XOAR_RCV_PKT_t;

typedef struct XOAR2_SND_PKT_struct XOAR2_SND_PKT_t;
typedef struct XOAR2_RCV_PKT_struct XOAR2_RCV_PKT_t;

extern BYTE Xoar2_error;
extern short int Xoar_ticks_left;

void Xoar2_set_error(BYTE ecode);
void XoarProcessCommand2(BYTE instruction_code,WORD instruction_parameter);
void XoarProcessCommand3(BYTE instruction_code,WORD instruction_parameter);


#endif /* XOAR_H_ */
