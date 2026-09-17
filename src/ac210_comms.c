/*
 * @brief UART interrupt example with ring buffers
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

// port_uart.c - Derived from mhcan3.c:uart_rb.c

#include "chip.h"
#include "board.h"
#include <string.h>
#include <stdlib.h>

#include "ac210_global.h"
#include "comms.h"
#include "control.h"
#include "digital.h"
#include "param.h"


//#define MH_SHOW_AUX2
#define MH_TRY_CHIP_RB
#ifdef MH_TRY_CHIP_RB
int PC_hublink_send_count;
int PC_hublink_send_count2;
int Sig100_hublink_send_count;

/* Transmit and receive ring buffers */
//STATIC RINGBUFF_T txring, rxring;

/* Transmit and receive ring buffer sizes */
//#define U0_SRB_SIZE	2048
#define UART_SRB_SIZE 512	/* Send */
#define UART_RRB_SIZE 256	/* Receive */

/* Transmit and receive buffers */
static uint8_t U0_rxbuff[UART_RRB_SIZE], U0_txbuff[UART_SRB_SIZE];
//static uint8_t U0_rxbuff[UART_RRB_SIZE], U0_txbuff[U0_SRB_SIZE];
static uint8_t U2_rxbuff[UART_RRB_SIZE], U2_txbuff[UART_SRB_SIZE];
static uint8_t U3_rxbuff[UART_RRB_SIZE], U3_txbuff[UART_SRB_SIZE];

/* Transmit and receive ring buffers */
RINGBUFF_T U0_txring, U0_rxring;
RINGBUFF_T U2_txring, U2_rxring;
RINGBUFF_T U3_txring, U3_rxring;
void Send_serial(LPC_USART_T *pUART, RINGBUFF_T *pRB, const void *data, int bytes);

#else
#endif

//-----------------------------------------------------------------------------
void AC210_uart0_wait(void)
{
#ifdef MH_TRY_CHIP_RB
	while(RingBuffer_IsEmpty(&U0_txring) == false);
#else
#endif
}
//-----------------------------------------------------------------------------
void AC210_uart3_wait(void)
{
#ifdef MH_TRY_CHIP_RB
	while(RingBuffer_IsEmpty(&U3_txring) == false);
#else
#endif
}
//---------------------------------------------------------------------------------
// Was defined in board.c
//bool Aux_flag=true;

static bool AUX_serial_used(void)
{
	if(getParameter(AUX_PORTS_SERIAL_CTL) != 0) return true;	// Could be more than 1 if we add other serial options.
	return false;
}
//----------------------------------------------------------------------------------
static bool com3_disabled;
void AC210_Disable_COM3(void)
{
	com3_disabled = true;
	NVIC_DisableIRQ(UART3_IRQn);
	Chip_UART_DeInit(LPC_UART3);
}
//----------------------------------------------------------------------------------
bool Aux_serial_enabled=false;	// Change this if we want diagnostics from aux serial port
void AC200_SIG100_putchar(char ch);
uint32_t Board_UARTPutChar_count;
void Board_UARTPutChar(char ch)	// Think this is only called by printf
{
	while((Chip_UART_ReadLineStatus(LPC_UART0) & UART_LSR_THRE) == 0);	// Wait for ready

//	while ((Chip_UART_ReadLineStatus(LPC_UART0) & UART_LSR_THRE) == 0) {}

	Chip_UART_SendByte(LPC_UART0, ch);
	Board_UARTPutChar_count++;

#ifdef MH_XXX	// MHH:09/08/2023
	if(Aux_serial_enabled == false) return;	// MH:03/03/2018. When using AC2_TEST and rpm linked to aux_port was causing rpm to generate random
	                                        // signals when outputting serial diagnostics, which caused run number to get incremented a lot.


	if(AUX_serial_used()) return;
// Note: May need a flag to show if LPC_UART3 not being used, otherwise ringbuffer will fill and watchdog will be triggered.
	if(com3_disabled)
	{
		return;
	}
#endif
#ifdef MH_TRY_CHIP_RB
//	AC200_SIG100_putchar(ch);	// MHH:09/08/2023. Should not be writing to SIG100 serial!!!
#else
	UartPutcRingBuffer(&Uart3OutputRB,LPC_UART3,(uint8_t)ch);
#endif
}
//----------------------------------------------------------------------------------
extern bool Hublink_command_sent;
extern bool Sig100_hub_link;
void Aux2_puts(char *string);
void mh_break(void)
{

}
void AC200_SIG100_putchar(char ch)
{
#ifdef MH_TRY_CHIP_RB
//	char buf[2];
//	buf[0] = ch;
//	Chip_UART_SendRB(LPC_UART3, &U3_txring, &ch, 1);
	Send_serial(LPC_UART3,&U3_txring,&ch,1);
//    #define DEBUG_SIG100_OUTPUT	// MHH:09/06/2025
    #ifdef DEBUG_SIG100_OUTPUT
		char c6[6];
		if(ch < 32 || ch > 120)
		{
			sprintf(c6,"<%2x>",ch);
			if(ch == 0x7c)
			{
				mh_break();
			}
		}
		else
		{
			sprintf(c6,"%c",ch);
		}
		Aux2_puts(c6);
    #endif
#else
	UartPutcRingBuffer(&Uart3OutputRB,LPC_UART3,(uint8_t)ch);
#endif
}
//----------------------------------------------------------------------------------
#ifdef MH_XXX	// MHH:09/06/2025
void AC200_SIG100_putstr(char *src)
{
	char c;
	for(;;)
	{
		c = *src++;
		if(c == 0) return;
		AC200_SIG100_putchar(c);
	}
}
#endif
//----------------------------------------------------------------------------------
/* Gets a character from the UART, returns EOF if no character is ready */
// Also was defined in board.c
int Board_UARTGetChar(void)
{
	if(AUX_serial_used()) return -1;
#ifdef MH_TRY_CHIP_RB
	char data[2];
	int result = RingBuffer_Pop(&U3_rxring,data);
	if(result == 1)
	{
		return (int)data[0];
	}
	return -1;		// Empty or error
#else
	return UartGetRingByte(&Uart3InputRB);
#endif
}
//-------------------------------------------------------------------------------
bool Aux_readable(void)
{
#ifdef MH_TRY_CHIP_RB
	int count = RingBuffer_GetCount(&U3_rxring);
	if(count > 0) return true;
	return false;
#else
	return UartReadable(&Uart3InputRB);
#endif
//	return UartReadable(&Uart3InputRB);
}
//-------------------------------------------------------------------------------
void Aux2_uart2_wait(void)	// MHH:11/12/2023
{
	while(RingBuffer_IsEmpty(&U2_txring) == false);
}
//-------------------------------------------------------------------------------
bool Aux2_readable(void)
{
#ifdef MH_TRY_CHIP_RB
	int count = RingBuffer_GetCount(&U2_rxring);
	if(count > 0) return true;
	return false;
#else
	return UartReadable(&Uart2InputRB);
#endif
//	return AC210_Aux2_Readable();
}
//-------------------------------------------------------------------------------
int Aux2_getc(void)
{
#ifdef MH_TRY_CHIP_RB
	char data[2];
	int result = RingBuffer_Pop(&U2_rxring,data);
	if(result == 1)
	{
		return (int)data[0];
	}
	return -1;		// Empty or error
#else
	return UartGetRingByte(&Uart2InputRB);
#endif
//	return AC210_Aux2_UARTGetChar();
}
//-------------------------------------------------------------------------------
// Was same as Board_UARTPutChar, but we want to be able to disable Board_UARTPutChar()
void Aux2_putc(char ch)
{
#ifdef MH_TRY_CHIP_RB
//	char buf[2];
//	buf[0] = ch;
//	Chip_UART_SendRB(LPC_UART2, &U2_txring, &ch, 1);
	Send_serial(LPC_UART2,&U2_txring,&ch,1);
#else

	UartPutcRingBuffer(&Uart2OutputRB,LPC_UART2,(uint8_t)ch);
#endif
}
//-------------------------------------------------------------------------------
//bool Aux2_init_flag;		// MHH:12/07/2023
void Aux2_init(void);
void Aux2_puts(char *string)
{
//	if(ps.parms[AUX_PORTS_SERIAL_CTL] == 0) return;
	if(ps.parms[AUX_PORTS_SERIAL_CTL] != 3) return;	// MHH:19/05/2023
	if(AC210_remote_control_port == 1) return;		// MHH:21/11/2023
	if(AC210_remote_control_port == 2) return;		// MHH:28/02/2025 CAN RC?
#ifdef MH_XXX	// MHH:12/07/2023
	if(Aux2_init_flag == false)
	{
		Aux2_init();		// Perhaps should only init if AUX_PORTS_SERIAL_CTL == 3?
	}
#endif
	char *bp = string;
	while(*bp) Aux2_putc(*bp++);

#ifdef MH_SHOW_AUX2
    OpMode mode = operatingMode();
    if(mode != MANUAL)		// So we can use manual for AC200User
    {
    	txDebug("AUX2:>");
    	txDebug(string);
    }
#endif
}
void Aux2_puts2(const char *string)
{
	const char *bp = string;
	while(*bp) Aux2_putc(*bp++);
}
//-------------------------------------------------------------------------------
void InitUARTn(int n,int baud_rate);
#ifdef MH_XXX
void Aux2_init(void)
{
	Aux2_init_flag = true;
	AC210_Aux2_Serial_Init();
	int baud_rate = 115200;
#ifdef MH_XXX
	int v = getParameter(AUX_PORTS_SERIAL_CTL);
	if(v == 3)	// MHH:16/05/2023:This was to allow 19.2K serial pos command. Think redundant.
	{
		baud_rate = 19200;
	}
#endif
	InitUARTn(2,baud_rate);
//	AC210_Aux2_UARTPutSTR("Aux2_init:\r\n");
	Aux2_puts("Aux2_init:\r\n");
}
#endif
//-------------------------------------------------------------------------------
int Aux_getc(void)
{
#ifdef MH_TRY_CHIP_RB
	char data[2];
	int result = RingBuffer_Pop(&U3_rxring,data);
	if(result == 1)
	{
		return (int)data[0];
	}
	return -1;
#else
	return UartGetRingByte(&Uart3InputRB);
#endif
}
//-------------------------------------------------------------------------------
// Used by SIG60
int Aux_getc_timeout(uint32_t millisecs)
{

	uint32_t finish_us = us_ticker_read() + millisecs * 1000;
	for(;;)
	{
		if(Aux_readable())	// byte available?
		{
			return Aux_getc();
		}
		if(us_ticker_read() >= finish_us)
		{
			return -1;
		}
	}
}
//-------------------------------------------------------------------------------
// Was same as Board_UARTPutChar, but we want to be able to disable Board_UARTPutChar()
void Aux_putc(char ch)
{
#ifdef MH_TRY_CHIP_RB
	AC200_SIG100_putchar(ch);
#else
		UartPutcRingBuffer(&Uart3OutputRB,LPC_UART3,(uint8_t)ch);
#endif
}
//-------------------------------------------------------------------------------
void Aux_puts(char *string)
{
	char *bp = string;
	while(*bp) Aux_putc(*bp++);
}
//-------------------------------------------------------------------------------
#define AUX_serial_MAX     128
char AUX_serial_line[AUX_serial_MAX+2];
int AUX_serial_ix;
bool AUX_serial_line_available(void)
{
    int c;

    while(true)
    {
//        if(pc.readable() == false) return false;
        if(Aux_readable() == false) return false;

//        c = pc.getc();
        c = Aux_getc();
        AUX_serial_line[AUX_serial_ix++] = c;
        if(AUX_serial_ix >= AUX_serial_MAX)
        {
            AUX_serial_ix = 0;
            continue;
        }
        if(c == 10 || c == 13)
        {
            if(AUX_serial_ix == 1)      // Ignore empty lines
            {
                AUX_serial_ix = 0;
            }
            else
            {
                AUX_serial_line[AUX_serial_ix++] = 0;
                return true;
            }
        }
    }
}
//-------------------------------------------------------------------------------
static void AUX_serial_reset(void)
{
    AUX_serial_ix = 0;
    AUX_serial_line[0] = 0;
}
//-------------------------------------------------------------------------------
#define AUX2_serial_MAX     128
char AUX2_serial_line[AUX2_serial_MAX+2];
int AUX2_serial_ix;
WORD AUX2_serial_throttle;
WORD AUX2_serial_throttle_count;
WORD AUX2_serial_rate;
bool AUX2_serial_line_available(void)
{
    int c;

    while(true)
    {
//        if(pc.readable() == false) return false;
        if(Aux2_readable() == false) return false;

//        c = pc.getc();
        c = Aux2_getc();
        AUX2_serial_line[AUX2_serial_ix++] = c;
        if(AUX2_serial_ix >= AUX2_serial_MAX)
        {
            AUX2_serial_ix = 0;
            continue;
        }
        if(c == 10 || c == 13)
        {
            if(AUX2_serial_ix == 1)      // Ignore empty lines
            {
                AUX2_serial_ix = 0;
            }
            else
            {
                AUX2_serial_line[AUX2_serial_ix++] = 0;
                return true;
            }
        }
    }
}
bool AUX2_UL_serial_line_available(void)
{
    int c;

    while(true)
    {
        if(Aux2_readable() == false) return false;

        c = Aux2_getc();
        if(c == 13)		// UL line delimiter
        {
//        	AUX2_serial_ix = 0;
        	return true;
        }

        if(AUX2_serial_ix < 30)	// Only interested in first 20 bytes for UL
        {
            AUX2_serial_line[AUX2_serial_ix++] = c;
        }
    }
}
//-------------------------------------------------------------------------------
static void AUX2_serial_reset(void)
{
    AUX2_serial_ix = 0;
    AUX2_serial_line[0] = 0;
}
//-------------------------------------------------------------------------------
static char Decode_line[40];
#define DECODE_OK       0
#define DECODE_ERROR    1

extern WORD Auto_throttle;		// defined in comms.c
#ifdef MH_XXX
static int Decode_serial_AM(void)
{
    int throttle;
    // ,airspeed,altitude;
    if(AUX2_serial_line[4] != '[') return DECODE_ERROR;
    if(AUX2_serial_line[16] != ']') return DECODE_ERROR;

    memcpy(Decode_line,AUX2_serial_line,16);

    // Expect in format:
    // S:X=[nnn,nnn,nnn]
    // 01234567890123456
    //           1

    Decode_line[8] = 0;     // null terminate numeric values

//    Decode_line[12] = 0;
//    Decode_line[16] = 0;

    throttle = atoi(Decode_line+5);
//    airspeed = atoi(Decode_line+9);
//    altitude = atoi(Decode_line+13);

//    pc.printf("X:%d,%d,%d\r\n",throttle,airspeed,altitude);

//    Auto_throttle = throttle;

    AUX2_serial_throttle = throttle;
    AUX2_serial_throttle_count++;
//    Command_speed_knots = airspeed;
//    Command_altitude_feet = altitude*1000;
    return DECODE_OK;
}
#endif
//-------------------------------------------------------------------------------
// Returns -1 if error
int My_atoi4(const char *src_p)
{
	uint8_t *bp = (uint8_t *)src_p;
	int rv = 0;
	int digit;
	for(int i=0;i<4;i++)	// expect 4 characters
	{
		digit = *bp++;
		if(digit < '0' || digit > '9') return -1;	// error
		rv = rv * 10 + (digit - '0');
	}
	return rv;
}
static int Decode_serial_UL(void)
{
    int throttle;
    // ,airspeed,altitude;

#ifdef MH_SHOW_UL_LINE
    AUX2_serial_line[20]= 0;
    PC_puts(AUX2_serial_line);
    PC_puts("\r\n");
#endif
    if(AUX2_serial_line[14] != ',') return DECODE_ERROR;
    if(AUX2_serial_line[19] != ',') return DECODE_ERROR;


    throttle = My_atoi4(AUX2_serial_line + 15);
    if(throttle < 0) return DECODE_ERROR;
    AUX2_serial_throttle = throttle;
    AUX2_serial_throttle_count++;
    return DECODE_OK;
}
//-------------------------------------------------------------------------------
WORD AUX_serial_setspeed;
WORD AUX_serial_actual;
OpMode AUX_serial_mode;
//WORD AUX2_slave_count;
WORD AUX_master_count;

static const uint8_t decode_pos[]={4,   9, 14, 16, 21, 26,0};
static const uint8_t decode_chr[]={',',',',';',',',',',']'};

static int Decode_serial_slave(void)
{
	for(int i=0;;i++)
	{
		uint8_t pos = decode_pos[i];
		if(pos == 0) break;

		uint8_t chr = decode_chr[i];

		uint8_t b = Decode_line[pos];

		if(b != chr)
			return DECODE_ERROR;
	}

//    memcpy(Decode_line,Slave_buff,28);

    // Expect in format:
    // !M[m,ssss,aaaa;m,ssss,aaaa]
    // 012345678901234567890123456789012
    //           1         2         3

	for(int i=0;;i++)
	{
		uint8_t pos = decode_pos[i];
		if(pos == 0) break;

		Decode_line[pos] = 0;		// null terminate numeric values
	}

    WORD m1  = atoi(Decode_line+3);
	WORD ss1 = atoi(Decode_line+5);
    WORD as1 = atoi(Decode_line+10);

    WORD m2  = atoi(Decode_line+15);
    WORD ss2 = atoi(Decode_line+17);
    WORD as2 = atoi(Decode_line+22);

    if(m1 != m2)
    {
    	return DECODE_ERROR;
    }

    if(ss1 != ss2 || as1 != as2)
    {
    	return DECODE_ERROR;
    }

    AUX_serial_mode = (OpMode)m1;
    AUX_serial_setspeed = ss1;
    AUX_serial_actual = as1;
    AUX_master_count = 0;

  // Need a way to make sure that we are getting at least 3 messages a second.

    return DECODE_OK;
}
//-------------------------------------------------------------------------------
#ifdef MH_XXX
static void AUX2_process_line(void)
{
//	printf("Aux:%s\r\n",AUX2_serial_line);		// For now

#if MH_SHOW_AUX
	Aux_puts("AUX:");
	Aux_puts(AUX2_serial_line);
	Aux_puts("\r\n");
#endif
//	PRINTF("AUX:%s\r\n",AUX2_serial_line);		// Debug

// Note: Could decode depending on getParameter(AUX2_PORTS_SERIAL_CTL)
// ie 1 = AM format
//    2 = UL format

	Decode_serial_UL();

#ifdef MH_XXX
	if(memcmp(AUX2_serial_line,"S:X",3) == 0)
	{
		Decode_serial_AM();
	}
#endif
}
#endif
//-------------------------------------------------------------------------------
void AC210_aux2_check_line(void)		// Called from checkForCommand() in comms.c
{
#ifdef MH_XXX    // MHH:12/07/2023
	if(Aux2_init_flag == false)
    {
    	Aux2_init();
    }
#endif
	while(AUX2_UL_serial_line_available())		// Could limit loops here...
	{
//		AUX2_process_line();
		Decode_serial_UL();
		memset(AUX2_serial_line,0,20);
		AUX2_serial_ix = 0;
	}
}
//-------------------------------------------------------------------------------
#ifdef MH_MASTER_CHECK
void Master_check(void)
{
    int message_count = 0;
    int error_count = 0;
    while(AUX2_serial_line_available())		// Could limit loops here...
	{
		if(memcmp(AUX2_serial_line,"!S:",3) == 0)
		{
			message_count++;
			uint8_t c = AUX2_serial_line[3];
			if(c != 'G')
			{
				error_count++;
			}
		}
		AUX2_serial_reset();
	}
    if(message_count != 0)
    {
    	if(error_count < message_count)	// Did we get a valid message?
    	{
            AUX2_slave_count = 0;		// zero slave count timer
    		//      		Aux2_puts("!S:G\r\n");	// Good
    	}
    }
}
#endif
//----------------------------------------------------------------------------------------------
#define CTL_NO_PORT			0
#define CTL_PORT_AUX1		1
#define CTL_PORT_AUX2		2

void AC210_master_check(void)
{
//    if(AUX2_slave_count < 1000) AUX2_slave_count++;

    WORD port = getParameter(CT_PORT);
    switch(port)
    {
    default:
    	return;
    case CTL_PORT_AUX1:
    	break;
    case CTL_PORT_AUX2:
#ifdef MH_XXX    	// MHH:12/07/2023
    	if(Aux2_init_flag == false)
    	{
    	   	Aux2_init();
    	}
#endif
    	break;
    }

#ifdef MH_MASTER_CHECK
    if((timerTick+1) % 10)		// five times a second
    {
    	Master_check();
    	return;
    }
#endif
    OpMode mode = operatingMode();
/*
	if(mode < HOLD || mode > TAKEOFF)
	{
		return;
	}
*/
    // Always send, less chance of an error.

//    long setspeed    = currentSetSpeed ()/RPMFACTOR;
//    long actualspeed = currentActualSpeed ()/RPMFACTOR;

    WORD setspeed    = (WORD)currentSetSpeed ();
    WORD actualspeed = (WORD)currentActualSpeed ();

    int m = (int)mode;

	sprintf(Decode_line,"!M[%d,%4d,%4d;%d,%4d,%4d]\r\n",m,setspeed,actualspeed,m,setspeed,actualspeed);
    if(port == CTL_PORT_AUX2)
    {
    	Aux2_puts(Decode_line);
    }
    else
    {
    	Aux_puts(Decode_line);
    }
}
#ifdef MH_MASTER_SLAVE_STANDARD_PORT
//-------------------------------------------------------------------------------
static void Slave_copy_command(char *ibuff)
{
	for(AUX2_serial_ix=0;AUX2_serial_ix<AUX2_serial_MAX;AUX2_serial_ix++)
	{
		AUX2_serial_line[AUX2_serial_ix] = ibuff[AUX2_serial_ix];
		if(ibuff[AUX2_serial_ix] == 0)
		{
			break;
		}
	}
}
void AC210_check_slave_command(char *ibuff)
{
	if (isSlaveProp())
	{
	    WORD port = getParameter(CT_PORT);
	    if(port == CTL_PORT_AUX1)
	    {

	    	Slave_copy_command(ibuff);
	    }
	}
}
//--------------------------------------------------------------------------------------------
// Copy from command buffer
//char AUX2_serial_line[AUX2_serial_MAX+2];
//int AUX2_serial_ix;
void AC210_copy_command_to_aux(char * ibuff)
{
	for(AUX2_serial_ix=0;AUX2_serial_ix<AUX2_serial_MAX;AUX2_serial_ix++)
	{
		AUX2_serial_line[AUX2_serial_ix] = ibuff[AUX2_serial_ix];
		if(ibuff[AUX2_serial_ix] == 0)
		{
			break;
		}
	}
}
#endif
//-------------------------------------------------------------------------------------------------
void AC210_slave_check(void)
{
    // Always receive, but only use values if in HOLD mode

    WORD port = getParameter(CT_PORT);
    switch(port)
    {
    default:
    	return;
    case CTL_PORT_AUX1:
    	break;
    case CTL_PORT_AUX2:
#ifdef MH_XXX    	// MHH:12/07/2023
    	if(Aux2_init_flag == false)
    	{
    	   	Aux2_init();
    	}
#endif
    	break;
    }

    if(AUX_master_count < 1000) AUX_master_count++;

    int message_count = 0;
    int error_count = 0;
    while(true)
    {
    	if(port == CTL_PORT_AUX2)
    	{
    		if(AUX2_serial_line_available() == false)
    		{
    			break;
    		}
    		memcpy(Decode_line,AUX2_serial_line,28);
    		AUX2_serial_reset();
    	}
    	else
    	{
    		if(AUX_serial_line_available() == false)
    		{
    			break;
    		}
    		memcpy(Decode_line,AUX_serial_line,28);
    		AUX_serial_reset();
    	}
		if(memcmp(Decode_line,"!M[",3) == 0)
		{
			message_count++;
			int rv = Decode_serial_slave();
			if(rv == DECODE_ERROR)
			{
				error_count++;
			}
		}
		AUX2_serial_reset();
    }
#ifdef MH_AUX2_SERIAL_ONLY
    while(AUX2_serial_line_available())		// Could limit loops here...
	{
		if(memcmp(AUX2_serial_line,"!M[",3) == 0)
		{
			message_count++;
			int rv = Decode_serial_slave();
			if(rv == DECODE_ERROR)
			{
				error_count++;
			}
		}
		AUX2_serial_reset();
	}
#endif
#if MH_SEND_SLAVE_STATUS
    if(mode == HOLD)	// Only send return status if in HOLD mode
    {
        if(message_count != 0)
        {
        	if(error_count < message_count)	// Did we get a valid message?
        	{
          		Aux2_puts("!S:G\r\n");	// Good
        	}
        	else
        	{
          		Aux2_puts("!S:B\r\n");	// Bad
        	}
        }
    }
#endif
}
//--------------------------------------------------------------------------------------------
#define MASTER_SLAVE_TIMEOUT_TICKS		50
/*
bool Is_slave_sending(void)
{
	return (AUX2_slave_count < MASTER_SLAVE_TIMEOUT_TICKS);
}
*/
//---------------------------------------------------------
BYTE isSlaveCommsActive (void)		// Was in ac210_notcoded.c
{
	if(AUX_serial_mode < HOLD || AUX_serial_mode > TAKEOFF)	return 0;
	return (AUX_master_count < MASTER_SLAVE_TIMEOUT_TICKS);
}
//---------------------------------------------------------
static WORD Slave_deadband;
WORD getSlaveDeadband(void)
{
	return Slave_deadband;
}

//BYTE getSlaveRPMSettings (long *setRPM)	// Also in ac210_notcoded.c
WORD getSlaveRPMSettings (void)	// Also in ac210_notcoded.c
{
	WORD setrpm = 0;
	if(AUX_serial_mode >= HOLD && AUX_serial_mode <= TAKEOFF)
	{
		if(isSlaveCommsActive())
		{
//			setrpm = AUX_serial_actual;

// Must avoid situation where Master actual RPM is not close to Master Set RPM (eg engine failure).
// We do not want Slave to try and follow Master to zero RPM.

			WORD deadband = getParameter (PI_DEAD_BAND);
			WORD slave_deadband = getParameter(CT_SLAVE_DEADBAND);
			WORD takeoff_speed = getParameter(SP_TAKEOFF);
			WORD min_hold_speed = getParameter(MIN_HOLD_SPEED);

			WORD adiff = abs(AUX_serial_setspeed - AUX_serial_actual);
			if(adiff > (deadband+20))	// Has setspeed been changed?
			{
				setrpm = AUX_serial_setspeed;
				Slave_deadband = deadband;
			}
			else
			{
				setrpm = AUX_serial_actual;	// This should be normal path
				Slave_deadband = slave_deadband;
			}
			if(setrpm < min_hold_speed) setrpm = min_hold_speed;
			if(setrpm > (takeoff_speed + deadband)) setrpm = takeoff_speed;
		}
	}
	return setrpm;
//	*setRPM = (long)setrpm*10;

//	return 0;
}
//-------------------------------------------------------------------------------
WORD AC210_aux2_get_throttle(void)
{
//	static ULONG AUX2_time_in_secs;

// Keep rate info in case there is a problem with CAN

	if(timerTick == 0)
//	if(AUX2_time_in_secs != TimeInSeconds)
	{
//		AUX2_time_in_secs = TimeInSeconds;
		AUX2_serial_rate  = AUX2_serial_throttle_count;
		AUX2_serial_throttle_count = 0;
	}

	if(AUX2_serial_rate <= 2)
	{
		return AUTO_THROTTLE_TIMEOUT;
	}
	//	Display_can_throttle();
	return AUX2_serial_throttle;
}

//---------------------------------------------------------------------------------
void PC_flush_output(void)
{
	AC210_uart0_wait();
}
//---------------------------------------------------------------------------------
extern void ResetReceiver(void);		// defined in comms.c
void PC_reset_input(void)
{
    ResetReceiver();
}
//-------------------------------------------------------------------------------
//----------------------------------------------------------------------------
int PC_RB_bytes_avail(void)
{
	return (U0_rxring.head - U0_rxring.tail);
}
int PC_getc_timeout(uint32_t millisecs)
{

	uint32_t finish_us = us_ticker_read() + millisecs * 1000;
	for(;;)
	{
		if(PC_RB_bytes_avail() > 0)	// byte available?
		{
			return PC_getc();
		}
		if(us_ticker_read() >= finish_us)
		{
			return -1;
		}
	}
}
int PC_getc(void)
{
#ifdef MH_TRY_CHIP_RB
	char data[2];
	int result = RingBuffer_Pop(&U0_rxring,data);
	if(result == 1)
	{
		return (int)data[0];
	}
	return -1;
#else
	return UartGetRingByte(&Uart0InputRB);
#endif
}

// Note: We cannot have a PC_getc because LPC_UART0 is a special case and all input characters are sent via
// rxProcessChar(c) (comms.c) to rxBuf.
#ifdef MH_XXX
#define PC_BUFF_MAX 		64
uint8_t PC_buff[PC_BUFF_MAX];
uint8_t PC_buff_len=0;
int PC_get_line(int timeout)
{
	AC210_uart0_wait();		// Wait for buffer to be output
	PC_buff_len=0;
	int b;
	int eol=0;
	for(int i=0;i<timeout;i++)
	{
		b = PC_getc();
		if(b == -1)
		{
			wait_ms(100);
		}
		else
		{
			if(b > 32)
			{
				if(PC_buff_len >= PC_BUFF_MAX)
				{
					return -1;
				}
				PC_buff[PC_buff_len++] = (uint8_t)b;
			}
			else
			{
				if((eol & 1) == 0)
				{
					if(b == 10) eol |= 1;
				}
				if((eol & 2) == 0)
				{
					if(b == 13) eol |= 2;
				}
				if((eol & 3) == 3)
				{
					PC_buff[PC_buff_len++] = 0;
					return PC_buff_len;
				}
			}
		}
	}
	PC_buff[PC_buff_len++] = 0;
	return -1;
}
#endif
//------------------------------------------------------------------------------------------------
uint32_t UART0_LSR;
uint32_t UART0_IIR;
void UART0_IRQHandler(void)
{
#ifdef MH_TRY_CHIP_RB
	UART0_IIR = LPC_UART0->IIR;
	Chip_UART_IRQRBHandler(LPC_UART0, &U0_rxring, &U0_txring);
#else
#endif
}
//------------------------------------------------------------------------------------------------
void UART2_IRQHandler(void)
{
#ifdef MH_TRY_CHIP_RB
	Chip_UART_IRQRBHandler(LPC_UART2, &U2_rxring, &U2_txring);
#else
#endif
}
//------------------------------------------------------------------------------------------------
void UART3_IRQHandler(void)
{
#ifdef MH_TRY_CHIP_RB
	Chip_UART_IRQRBHandler(LPC_UART3, &U3_rxring, &U3_txring);
#else
#endif
}
//------------------------------------------------------------------------------------------------
// Note: For AC210 using UART0 for AC200 19.2k comms, pins P0.2 (TXD0) and P0.3 (RXD0)
// Currently printf also uses same uart, so maybe use auxiliary uart for debug?
// Auxiliary serial port is uart3, P4.28 (TXD3) and P4.29 (RXD3)
// Have modified board.h so that DEBUG_UART is defined as LPC_UART3, not LPC_UART0

void InitUARTn(int n,int baud_rate)
{
	LPC_USART_T * lpc_uart;
	LPC175X_6X_IRQn_Type uart_irqn;
//	uint32_t baud_rate;

//	baud_rate = 115200;	// default
//	baud_rate = 19200;	// default MHH:24/11/2020

	switch(n)
	{
	default:
		Abort(AC210_SRC_COMMS+20,"InitUART: Unexpected UART number.");
		break;		// Not really needed

// Using this one for AC200 19.2k connection with PC

	case 0:		// Uart0
		lpc_uart  = LPC_UART0;
		uart_irqn = UART0_IRQn;
//		baud_rate = 19200;
		break;

// Note: Could probably be done better in a table.
#ifdef MH_DEFINE_OTHER_UARTS
	case 1:		// Uart1
		lpc_uart  = LPC_UART1;
		uart_irqn = UART1_IRQn;
		break;
#endif

	case 2:		// Uart2
		lpc_uart  = LPC_UART2;
		uart_irqn = UART2_IRQn;
//		baud_rate = 19200;		// MHH:10/11/2020. For Master/Slave, we need transceiver + slow baud rate
		break;

	case 3:		// Uart3		// Note: Uart3 used to connect to SIG100
		lpc_uart  = LPC_UART3;
		uart_irqn = UART3_IRQn;
		break;
	}

	Board_UART_Init(lpc_uart);
	//	Board_LED_Set(0, false);

		/* Setup UART for 115.2K8N1 */
	Chip_UART_Init(lpc_uart);
	Chip_UART_SetBaud(lpc_uart, baud_rate);

	if(n == 2 && baud_rate == 115200 && getParameter(TC_PORT) == TC_PORT_SERIAL)	// MHH:22/01/2023. UL has even parity enabled!!
	{
		Chip_UART_ConfigData(lpc_uart,UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_EN |UART_LCR_PARITY_EVEN);
	}
	else
	{
		Chip_UART_ConfigData(lpc_uart, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	}

//	Chip_UART_SetupFIFOS(lpc_uart, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV3));

	Chip_UART_SetupFIFOS(lpc_uart, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |	UART_FCR_TX_RS | UART_FCR_TRG_LEV3));
//	Chip_UART_SetupFIFOS(lpc_uart, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |	UART_FCR_TX_RS | UART_FCR_TRG_LEV0));


//	Chip_UART_SetupFIFOS(lpc_uart, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(lpc_uart);

//	Chip_UART_IntEnable(lpc_uart, (UART_IER_RBRINT));
	Chip_UART_IntEnable(lpc_uart, (UART_IER_RBRINT | UART_IER_RLSINT));
//	Chip_UART_IntEnable(lpc_uart, (UART_IER_RBRINT | UART_IER_RLSINT | UART_IER_THREINT));

		/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(uart_irqn, 1);
//	NVIC_ClearPendingIRQ(uart_irqn);	// MHH:04/07/2018, commented out 05/12/2022??


	Chip_UART_ReadByte(lpc_uart);	// Sometimes enabling UART3 resulted in an interrupt loop because of a CTI error (character timeout).

#ifdef MH_XXX
	if(n == 3)
	{
		Chip_UART_ReadByte(LPC_UART3);	// Sometimes enabling UART3 resulted in an interrupt loop because of a CTI error (character timeout).
										// This clears timeout error
	}
#endif
	NVIC_EnableIRQ(uart_irqn);
}
//------------------------------------------------------------------------------------------------
int UART0_enabled;
void p_rxDisableInt(void)
{
//	NVIC_DisableIRQ(UART0_IRQn);	// MHH:23/03/2023. Don't think needed
//	UART0_enabled=0;
}
//------------------------------------------------------------------------------------------------
void p_rxEnableInt(void)
{
//	NVIC_EnableIRQ(UART0_IRQn);		// MHH:23/03/2023. Don't think needed
//	UART0_enabled=1;
}
//------------------------------------------------------------------------------------------------
int Debug_val = 0;
void AC210_PC_ChangeBaud(int baud_rate)
{
	AC210_uart0_wait();	// This waits until output buffer is empty.
    wait_ms(40);
    Chip_UART_SetBaud(LPC_UART0, baud_rate);
//    RingBuffer_Flush(&U0_rxring);	// MHH:11/08/2023
    wait_ms(40);
    if(Debug_val)
    {
//    	printf("PC_baud_change:%d\r\n",baud_rate);
    }
}
//------------------------------------------------------------------------------------------------
#ifdef MH_XXX
void Aux_Set_no_interrupts(void)
{
	NVIC_DisableIRQ(UART3_IRQn);
	Chip_UART_IntDisable(LPC_UART3, (UART_IER_RBRINT | UART_IER_RLSINT | UART_IER_THREINT));	// Disable all UART interrupts
    LPC_UART3->FCR |= (UART_FCR_RX_RS | UART_FCR_TX_RS);	// Reset FIFO RX and TX
}
//------------------------------------------------------------------------------------------------

void Aux_allow_interrupts(void)
{
#ifdef MH_TRY_CHIP_RB
	RingBuffer_Flush(&U3_txring);
	RingBuffer_Flush(&U3_rxring);
#else
#endif
	if(getParameter(AUX_PORTS_SERIAL_CTL) == 0)	// MHH:03/07/2018. no Input interrupts if debug
    {
		Chip_UART_IntEnable(LPC_UART3, UART_IER_THREINT);
    }
	else
	{
		Chip_UART_IntEnable(LPC_UART3, (UART_IER_RBRINT | UART_IER_RLSINT | UART_IER_THREINT));
	}
	NVIC_ClearPendingIRQ(UART3_IRQn);
 	NVIC_EnableIRQ(UART3_IRQn);
}
//------------------------------------------------------------------------------------------------
#define MH_CHANGE_UART3_BAUD
#ifdef MH_CHANGE_UART3_BAUD
#define AUX_WAIT_MS		20
void Aux_ChangeBaud(int baud_rate)
{
    Aux_Set_no_interrupts();
    AC210_uart3_wait();	// Flush output buffer (if any)
    wait_ms(AUX_WAIT_MS);
//    Aux_Set_no_interrupts();

    Chip_UART_SetBaud(LPC_UART3, baud_rate);
    for(int i=0;i<AUX_WAIT_MS;i++)
    {
    	wait_ms(1);
        LPC_UART3->FCR |= (UART_FCR_RX_RS | UART_FCR_TX_RS);	// Reset FIFO RX and TX (to be sure!)
    }
//    wait_ms(40);
 	Aux_allow_interrupts();
}
#endif
#endif
//=========================================================================
uint8_t AC210_remote_control_port=0;
int AC210_SerialInit(void)
{

#ifdef MH_TRY_CHIP_RB
	RingBuffer_Init(&U0_rxring, U0_rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&U0_txring, U0_txbuff, 1, UART_SRB_SIZE);
//	RingBuffer_Init(&U0_txring, U0_txbuff, 1, U0_SRB_SIZE);

	RingBuffer_Init(&U2_rxring, U2_rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&U2_txring, U2_txbuff, 1, UART_SRB_SIZE);

	RingBuffer_Init(&U3_rxring, U3_rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&U3_txring, U3_txbuff, 1, UART_SRB_SIZE);

#else
#endif

	InitUARTn(0,19200);
	if((ps.parms[REMOTE_COMMS_TYPE] == REMOTE_COMMS_STANDARD) || AC210_remote_control_board)
	{
		AC210_remote_control_port = ps.parms[RC_PORT];
	}
//	AC210_Aux2_Serial_Init();	// Default is to set up as UART2
	int baudrate;
	switch(AC210_remote_control_port)
	{
	default:
		InitUARTn(2,115200);	// MHH:21/02/2025. Useful for Diagnostics
		Aux2_puts("Aux2 init\r\n");	// param not loaded yet.
		break;

	case 1:						// Serial port used for RC commands
		baudrate = ps.parms[RC_BAUDRATE];
		if(baudrate == 0)  baudrate = 192;
		baudrate *= 100;
		InitUARTn(2,baudrate);	// MHH:21/02/2025.
		break;

	case 2:						// CAN port used for RC commands
		baudrate = ps.parms[RC_BAUDRATE];
		if(baudrate == 0)  baudrate = 125;
		baudrate *= 1000;
		AC210_CAN_Init(baudrate,CAN_MODE_RC);		// May need to differentiate from CANAero. Eg not setup FLUT
		break;

	}
#ifdef MH_XXX
	if(AC210_remote_control_port == 1)
	{
		int baudrate = ps.parms[RC_BOARDRATE];
		if(baudrate == 0)  baudrate = 19200;
//		InitUARTn(2,19200);	// MHH:21/11/2023.
		InitUARTn(2,baudrate);	// MHH:21/02/2025.
	}
	else
	{
		InitUARTn(2,115200);	// MHH:12/07/2023.
	}
	Aux2_puts("Aux2 init\r\n");	// param not loaded yet.
#endif

	InitUARTn(3,19200);
	return 0;
}


void Display_ringbuffer(int uart_number,char *desc,RINGBUFF_T *RingBuff)
{
	PRINTF("Uart:%d, %s: size:%d, head:%d, tail:%d\r\n",uart_number,desc,RingBuff->count,RingBuff->head,RingBuff->tail);
}
bool Uart0_rx_tx_link;
void Display_uart0_rx_ringbuffer(void)
{
	PRINTF("U0_rx_ring: size:%d,head:%d,tail:%d\r\n",U0_rxring.count,U0_rxring.head,U0_rxring.tail);
	if(U0_rxring.head == 0)
	{
		PRINTF("\r\nNo data\r\n");
		return;
	}

	int start = U0_rxring.head;
	int finish = start + UART_RRB_SIZE;
	for(int p = start;p<finish;p++)
	{
		int ix = p & (UART_RRB_SIZE-1);
		uint8_t ch = U0_rxbuff[ix];
		if(ch >= 32 && ch <= 'z')
		{
			PC_putc((char)ch);
		}
		else
		{
			PRINTF("<%d>",ch);
		}
	}
	PRINTF("\r\n[End]\r\n");
}
void Display_uart2_rx_ringbuffer(void)
{
	PRINTF("U2_rx_ring: size:%d,head:%d,tail:%d\r\n",U2_rxring.count,U2_rxring.head,U2_rxring.tail);
	if(U2_rxring.head == 0)
	{
		PRINTF("\r\nNo data\r\n");
		return;
	}

	int start = U2_rxring.head;
	int finish = start + UART_RRB_SIZE;
	for(int p = start;p<finish;p++)
	{
		int ix = p & (UART_RRB_SIZE-1);
		uint8_t ch = U2_rxbuff[ix];
		if(ch >= 32 && ch <= 'z')
		{
			PC_putc((char)ch);
		}
		else
		{
			PRINTF("<%d>",ch);
		}
	}
	PRINTF("\r\n[End]\r\n");
}
void AC210_display_serial_ringbuffer_data(int val)	// MHH:12/08/2023
{

	if(val >= 1000)	// MHH:15/08/2023
	{
		if(val == 1115)
		{
			PRINTF("Changing baud rate to 115K\r\n");
		    wait_ms(200);
		    AC210_PC_ChangeBaud(115200);
		    return;
		}
		if(val == 1192)
		{
			PRINTF("Changing baud rate to 19.2K\r\n");
		    wait_ms(200);
		    AC210_PC_ChangeBaud(19200);
		    return;

		}
		if(val == 1001)	// Need to reset AC200 to get back from this
		{
			PRINTF("Linking UART0 RX to UART0 TX\r\n")
		    wait_ms(200);
			Uart0_rx_tx_link = true;
		}
		return;
	}

	if(val >= 100)
	{
		if(val == 100)
		{
			Display_uart0_rx_ringbuffer();
		}
		if(val == 200)
		{
			Display_uart2_rx_ringbuffer();
		}
	}
	if(val == -1 || val == 1)
	{
		Display_ringbuffer(0,"RX",&U0_rxring);
		Display_ringbuffer(0,"TX",&U0_txring);
		PRINTF("\r\n");
	}
	if(val == -1 || val == 2)
	{
		Display_ringbuffer(2,"RX",&U2_rxring);
		Display_ringbuffer(2,"TX",&U2_txring);
		PRINTF("\r\n");
	}
	if(val == -1 || val == 3)
	{
		Display_ringbuffer(3,"RX",&U3_rxring);
		Display_ringbuffer(3,"TX",&U3_txring);
		PRINTF("\r\n");
	}
}
//------------------------------------------------------------------------------------------------
void AC210_SerialDeInit(void)
{
//	printf("SerialDeInit:Closing UART0\n\r");
	NVIC_DisableIRQ(UART0_IRQn);
	Chip_UART_DeInit(LPC_UART0);
}
//======================================================================
// Routines called by AC200 logic.
//----------------------------------------------------------------------
// serial port buffers. Tx buffer is circular
// rx buffer is message based, and pointers are reset to 0
// after receipt of a <CR><LF> combination
#define TX_BUF_SIZE 200
#define RX_BUF_SIZE 50

//int Debug_val = 0;
char Debug_last_line[256];
char Debug_line[256];
int Debug_dup_count=0;
//const char *Debug_transmit="T: ";
char *Debug_transmit="T: ";
//const char *Debug_receive="R: ";
char *Debug_receive="R: ";
extern uint8_t UU_uBuf[64];

void Debug_SendStringLen(const char *buf,uint32_t numChar)
{
	if(Debug_val == 0) return;
	if(ac2_test_flag == AC2_TEST_ON) return;

	if(numChar > 20) numChar = 20;

/*
	if(numChar > 30)		// Quick and dirty way to ignore uuencoded data
	{
		return;
	}
*/
	if(buf == (char *)UU_uBuf)
	{
		Board_UARTPutSTR("T: UU_uBuf\r\n");
		return;
	}

	if(Debug_val > 1)		// Hex display?
	{
		printf("[");
		for(int i=0;i<numChar;i++)
		{
			printf("%02x ",buf[i]);
		}
		printf("]\r\n");
	}
	strcpy(Debug_line,Debug_transmit);
	int slen = MIN(250,numChar);
	memcpy(Debug_line+3,buf,slen);
	slen += 3;
	Debug_line[slen++]=0;	// Trailing null

	if(memcmp(Debug_line,Debug_last_line,slen) == 0)
	{
		if(Debug_dup_count == 3)		//  duplicates?
		{
			Board_UARTPutSTR(Debug_transmit);
			Board_UARTPutSTR("*** Duplicates ***\r\n");	// Transmit
		}
		if(Debug_dup_count++ >=3 ) return;
	}
	else
	{
		Debug_dup_count = 0;
		memcpy(Debug_last_line,Debug_line,slen);
	}
	if(Debug_dup_count > 0)
	{
		if(memcmp(Debug_line+3,"STATE",5) != 0) Debug_dup_count = 0;	// Only count STATE or STATEF as duplicates
	}
	Board_UARTPutSTR(Debug_line);
}
void Debug_ShowCommand(char *buf)
{
	if(Debug_val == 0) return;

	Board_UARTPutSTR(Debug_receive);	// Receive
	Board_UARTPutSTR(buf);	// Receive
}
//----------------------------------------------------------------------------
void PC_puts(const char *buf)
{
	p_send(buf,strlen(buf));
}
//---------------------------------------------------------------------------------
void PC_putc(char ch)
{
#ifdef MH_TRY_CHIP_RB
	if(Sig100_hub_link)
	{
		PC_hublink_send_count++;
		return;
	}
//	char buf[2];
//	buf[0] = ch;
//	Chip_UART_SendRB(LPC_UART0, &U0_txring, &ch, 1);
	p_send(&ch,1);
#else
	UartPutcRingBuffer(&Uart0OutputRB,LPC_UART0,(uint8_t)ch);
#endif
}
//-----------------------------------------------------------------------------
void Send_serial(LPC_USART_T *pUART, RINGBUFF_T *pRB, const void *data, int bytes)
{
	if(pRB->data == (void *)0)	// MHH: 09/08/2023. Trying to print before ring buffer initialised?
	{
		return;	// Probably should abort
	}

	int bytes_to_go = bytes;
	int bytes_sent = 0;
//	uint8_t *p_src = buf;
	while(true)	// Make sure all bytes are sent, even if we have to wait
	{
		uint32_t ret = Chip_UART_SendRB(pUART, pRB, data + bytes_sent, bytes_to_go);
		bytes_to_go -= ret;
		if(bytes_to_go == 0) return;
		bytes_sent += ret;
		wait_ms(1);
	}
}
//----------------------------------------------------------------------------
void p_send(const char *buf,uint32_t numChar)
{
	//	char tempbuff[100];		// Quick and dirty..
//#define MH_DISPLAY
#ifdef MH_DISPLAY
	static char display_buf[100];
	int i,len;
	bool display_line;

	if(buf[0] == XOAR_HEAD_0) {
		memcpy(display_buf,buf,10);
		printf(">>[");
		for(i=0;i<10;i++) printf("%02x ",display_buf[i]);
		printf("]\n\r");
	} else {
		display_line = true;
		if(memcmp(buf,"STATE",5) == 0) {
			if(memcmp(buf,display_buf,numChar) == 0) {
				display_line = false;
			}
		}
		if(display_line) {
			len = MIN(99,numChar);
			memcpy(display_buf,buf,len);
			display_buf[len] = 0;		// So we use printf
			printf(">> %s",display_buf);
		}
	}
#endif

#ifdef MH_TRY_CHIP_RB
	if(Sig100_hub_link)
	{
		PC_hublink_send_count2++;
		return;
	}
#ifdef MH_AAA
	uint32_t bytes_to_go = numChar;
	uint32_t bytes_sent = 0;
//	uint8_t *p_src = buf;
	while(true)	// Make sure all bytes are sent, even if we have to wait
	{
		uint32_t ret = Chip_UART_SendRB(LPC_UART0, &U0_txring, buf + bytes_sent, bytes_to_go);
		bytes_to_go -= ret;
		if(bytes_to_go == 0) return;
		bytes_sent += ret;
		wait_ms(1);
	}
//    Chip_UART_SendRB(LPC_UART0, &U0_txring, buf, numChar);
#else
	if(U0_txring.data == (void *)0)	// ring buffer initialised?
	{
		Board_UARTPutSTR((char *)buf);	// Trying to avoid compiler warning for "const"
		return;
	}
	Send_serial(LPC_UART0,&U0_txring,buf,numChar);
#endif
#else
#endif
}

