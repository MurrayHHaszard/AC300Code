/*
 * ac210_board.c
 *
 *  Created on: 22/10/2016
 *      Author: Murray
 */

#include "chip.h"
#include "board.h"
#include <string.h>
#include <stdlib.h>

#include "ac210_adc.h"
#include "ac210_global.h"
#include "ac210_sig100.h"

#include "param.h"

/*****************************************************************************
 * Private functions
 ****************************************************************************/

typedef struct {
	uint32_t pingrp:3;		/* Pin group */
	uint32_t pinnum:5;		/* Pin number */
	uint32_t modefunc:24;	/* Function and mode. */
} PINMUX_GRP_J2_T;

#define J2_NC		0
#define J2_INPUT	1
#define J2_OUTPUT	2
#define J2_PWM		3
#define	J2_ADC		4
#ifdef MH_OLD_PORT
STATIC const PINMUX_GRP_J2_T J2_map2[] =
{
		{0,0,J2_NC},		// dummy first entry
		{AC210_J2_DIG_PORT,  AC210_J2_1_STATE_U1_PIN,  J2_OUTPUT},	/* J2_1 State U1 */
		{AC210_J2_DIG_PORT,  AC210_J2_2_STATE_U2_PIN,  J2_OUTPUT},	/* J2_2 State U2 */
		{AC210_J2_DIG_PORT,  AC210_J2_3_STATE_U3_PIN,  J2_OUTPUT},	/* J2_3 State U3 */
		{AC210_J2_DIG_PORT,  AC210_J2_4_STATE_L1_PIN,  J2_OUTPUT},	/* J2_4 State L1 */
		{AC210_J2_DIG_PORT,  AC210_J2_5_STATE_L2_PIN,  J2_OUTPUT},	/* J2_5 State L2 */
		{AC210_J2_DIG_PORT,  AC210_J2_6_STATE_L3_PIN,  J2_OUTPUT},	/* J2_6 State L3 */
		{AC210_ADC_PORT,  AC210_J2_7_MOTOR_STATE_SENSE_PIN,  J2_ADC},		/* J2_7 Motor State Sense AD0.0 */
		{AC210_J2_DIG_PORT,  AC210_J2_8_DRIVE_U1_PIN,  J2_OUTPUT},	/* J2_8 Drive U1 */
		{0,0,J2_NC},			/* J2_9 Not Connected */
		{AC210_J2_DIG_PORT,  AC210_J2_10_DRIVE_U2_PIN,  J2_OUTPUT},	/* J2_10 Drive U2 */
		{AC210_J2_DIG_PORT,  AC210_J2_11_DRIVE_U3_PIN,  J2_OUTPUT},	/* J2_11 Drive U3 */
		{AC210_PWM_PORT,  AC210_J2_12_DRIVE_L1_PIN,   J2_PWM},		/* J2_12 Drive L1 PWM1.3 */
		{AC210_PWM_PORT,  AC210_J2_13_DRIVE_L2_PIN,   J2_PWM},		/* J2_13 Drive L2 PWM1.2 */
		{AC210_PWM_PORT,  AC210_J2_14_DRIVE_L3_PIN,   J2_PWM},		/* J2_14 Drive L3 PWM1.1 */
		{AC210_J2_DIG_PORT,  AC210_J2_15_COARSE_SENSE_PIN,  J2_INPUT},	/* J2_15 Coarse sense */
		{AC210_J2_DIG_PORT,  AC210_J2_16_FINE_SENSE_PIN,  J2_INPUT},	/* J2_16 Fine sense */
};
#else
STATIC const PINMUX_GRP_J2_T J2_map2[] =
{
		{0,0,J2_NC},		// dummy first entry
		{PIN_DEF2(AC210_J2_1_STATE_U1_PIND),  J2_OUTPUT},	/* J2_1 State U1 */
		{PIN_DEF2(AC210_J2_2_STATE_U2_PIND),  J2_OUTPUT},	/* J2_2 State U2 */
		{PIN_DEF2(AC210_J2_3_STATE_U3_PIND),  J2_OUTPUT},	/* J2_3 State U3 */
		{PIN_DEF2(AC210_J2_4_STATE_L1_PIND),  J2_OUTPUT},	/* J2_4 State L1 */
		{PIN_DEF2(AC210_J2_5_STATE_L2_PIND),  J2_OUTPUT},	/* J2_5 State L2 */
		{PIN_DEF2(AC210_J2_6_STATE_L3_PIND),  J2_OUTPUT},	/* J2_6 State L3 */
		{PIN_DEF2(AC210_J2_7_MOTOR_STATE_SENSE_PIND),  J2_ADC},		/* J2_7 Motor State Sense AD0.0 */
		{PIN_DEF2( AC210_J2_8_DRIVE_U1_PIND),  J2_OUTPUT},	/* J2_8 Drive U1 */
		{0,0,J2_NC},			/* J2_9 Not Connected */
		{PIN_DEF2(AC210_J2_10_DRIVE_U2_PIND),  J2_OUTPUT},	/* J2_10 Drive U2 */
		{PIN_DEF2(AC210_J2_11_DRIVE_U3_PIND),  J2_OUTPUT},	/* J2_11 Drive U3 */
		{PIN_DEF2(AC210_J2_12_DRIVE_L1_PIND),   J2_PWM},		/* J2_12 Drive L1 PWM1.3 */
		{PIN_DEF2(AC210_J2_13_DRIVE_L2_PIND),   J2_PWM},		/* J2_13 Drive L2 PWM1.2 */
		{PIN_DEF2(AC210_J2_14_DRIVE_L3_PIND),   J2_PWM},		/* J2_14 Drive L3 PWM1.1 */
		{PIN_DEF2(AC210_J2_15_COARSE_SENSE_PIND),  J2_INPUT},	/* J2_15 Coarse sense */
		{PIN_DEF2(AC210_J2_16_FINE_SENSE_PIND),  J2_INPUT},	/* J2_16 Fine sense */
};

#endif

static void Board_J2_Init(void)
{
	/* Pin PIO0_22 is configured as GPIO pin during SystemInit */
	/* Set the PIO_22 as output */
//	Chip_GPIO_WriteDirBit(LPC_GPIO, LED0_GPIO_PORT_NUM, LED0_GPIO_BIT_NUM, true);
	uint8_t pin;
	uint8_t port;
	uint8_t func;

	PINMUX_GRP_J2_T pdata;
	for(int j=1;j<=16;j++)
	{
		pdata = J2_map2[j];
		pin = pdata.pinnum;
		port = pdata.pingrp;
		func = pdata.modefunc;

		switch(func)
		{
		case J2_INPUT:
			Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin, false);
			break;

		case J2_OUTPUT:
			Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin, true);
			break;

// Handle other types later...

		default:
			break;
		}
	}
}
/*****************************************************************************
 * Public functions
 ****************************************************************************/
#define AC210_AUX_UART		LPC_UART3		// Used for Auxiliary port.

void AC210_Aux_Serial_Init(void)
{
	Board_UART_Init(AC210_AUX_UART);

	Chip_UART_Init(AC210_AUX_UART);
	Chip_UART_SetBaud(AC210_AUX_UART, 115200);
	Chip_UART_ConfigData(AC210_AUX_UART, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);

	/* Enable UART Transmit */
	Chip_UART_TXEnable(AC210_AUX_UART);

}
//-----------------------------------------------------------------------------
#define AC210_AUX2_UART		LPC_UART2		// Used for Auxiliary port.
#ifdef MH_XXX
void AC210_Aux2_Serial_Init(void)
{
	// First make sure CAN pins are not active.

	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_2_DIG3_CAN_PIND),IOCON_MODE_INACT | IOCON_FUNC0); /* P0.1, 47, CAN-TD1, also used to output a MagInt signal for test */
	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),IOCON_MODE_INACT | IOCON_FUNC0); /* P0.0, 46, CAN-RD1 */

	// Now initialise serial pins

	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_2_DIG3_TX2_PIND),IOCON_MODE_INACT | IOCON_FUNC1);	/* P0.10, 48, TXD2 */
	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_9_DIG4_RX2_PIND),IOCON_MODE_INACT | IOCON_FUNC1);	/* P0.11, 49, RXD2 */

#ifdef AC210_AUX2_NO_RINGBUFFER

	Board_UART_Init(AC210_AUX2_UART);

	Chip_UART_Init(AC210_AUX2_UART);
	Chip_UART_SetBaud(AC210_AUX2_UART, 115200);
	Chip_UART_ConfigData(AC210_AUX2_UART, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);

	/* Enable UART Transmit. Note: We may not want to be able to transmit for throttle control. */
	Chip_UART_TXEnable(AC210_AUX2_UART);
#endif
}
#endif
//-----------------------------------------------------------------------------
void AC210_Aux2_CAN_Init(void)
{
	// First make sure serial pins are not active.

	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_2_DIG3_TX2_PIND),IOCON_MODE_PULLUP | IOCON_FUNC0);	/* P0.10, 48, TXD2 */
	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_9_DIG4_RX2_PIND),IOCON_MODE_PULLUP | IOCON_FUNC0);	/* P0.11, 49, RXD2 */

//	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_2_DIG3_TX2_PIND),IOCON_MODE_INACT | IOCON_FUNC0);	/* P0.10, 48, TXD2 */
//	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_9_DIG4_RX2_PIND),IOCON_MODE_INACT | IOCON_FUNC0);	/* P0.11, 49, RXD2 */

	// Now initialise CAN pins

	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_2_DIG3_CAN_PIND),IOCON_MODE_INACT | IOCON_FUNC2); /* #80, P0.5 (TD2)*/
	Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),IOCON_MODE_INACT | IOCON_FUNC2); /* #81, P0.4 (RD2)*/

}
//-----------------------------------------------------------------------------
bool AC210_Aux2_Readable(void)
{
	return ((Chip_UART_ReadLineStatus(AC210_AUX2_UART) & UART_LSR_RDR) != 0);
}
//-----------------------------------------------
/* Gets a character from the UART, returns EOF if no character is ready */
int AC210_Aux2_UARTGetChar(void)
{
	if (Chip_UART_ReadLineStatus(AC210_AUX2_UART) & UART_LSR_RDR) {
		return (int) Chip_UART_ReadByte(AC210_AUX2_UART);
	}
	return EOF;
}
//-----------------------------------------------
/* Sends a character on the UART */
void AC210_Aux2_UARTPutChar(char ch)
{
	while ((Chip_UART_ReadLineStatus(AC210_AUX2_UART) & UART_LSR_THRE) == 0) {}
	Chip_UART_SendByte(AC210_AUX2_UART, (uint8_t) ch);
}
//-----------------------------------------------
/* Outputs a string on the debug UART */
void AC210_Aux2_UARTPutSTR(char *str)
{
	while (*str != '\0') {
		AC210_Aux2_UARTPutChar(*str++);
	}
}
//-----------------------------------------------------------------------------
bool Board_Test_Fine(void)
{
	return Chip_GPIO_ReadPortBit(LPC_GPIO, PIN_DEF2(AC210_J2_16_FINE_SENSE_PIND));
//	return Chip_GPIO_ReadPortBit(LPC_GPIO, AC210_J2_DIG_PORT, AC210_J2_16_FINE_SENSE_PIN);
}
bool Board_Test_Coarse(void)
{
	return Chip_GPIO_ReadPortBit(LPC_GPIO, PIN_DEF2(AC210_J2_15_COARSE_SENSE_PIND));
//	return Chip_GPIO_ReadPortBit(LPC_GPIO, AC210_J2_DIG_PORT, AC210_J2_15_COARSE_SENSE_PIN);
}
//-----------------------------------------------------------------------------
void AC210_Test_Fine_Coarse(void)
{
	if(Board_Test_Fine())
		{PRINTF("Fine = true\r\n");}
	else
		{PRINTF("Fine = false\r\n");}

	if(Board_Test_Coarse())
		{PRINTF("Coarse = true\r\n");}
	else
		{PRINTF("Coarse = false\r\n");}
}
//-----------------------------------------------
/* Sends a character on the UART */
void AC210_Aux_UARTPutChar(char ch)
{
	while ((Chip_UART_ReadLineStatus(AC210_AUX_UART) & UART_LSR_THRE) == 0) {}
	Chip_UART_SendByte(AC210_AUX_UART, (uint8_t) ch);
}
//-----------------------------------------------
/* Gets a character from the UART, returns EOF if no character is ready */
int AC210_Aux_UARTGetChar(void)
{
	if (Chip_UART_ReadLineStatus(AC210_AUX_UART) & UART_LSR_RDR) {
		return (int) Chip_UART_ReadByte(AC210_AUX_UART);
	}
	return EOF;
}
//-----------------------------------------------
/* Outputs a string on the debug UART */
void AC210_Aux_UARTPutSTR(char *str)
{
	while (*str != '\0') {
		AC210_Aux_UARTPutChar(*str++);
	}
}
//--------------------------------------------------------------------------------------------------
STATIC const uint8_t Board_HW_version_pind_t[] =
{
		AC210_HW_VERSION_0_PIND,
		AC210_HW_VERSION_1_PIND,
		AC210_HW_VERSION_2_PIND,
		AC210_HW_VERSION_3_PIND,
};

WORD AC210_hardware_version;
/*
 *  Pin:8410
 * 	0	0000	P
 * 	1	0001	O
 * 	2	0010	N
 * 	3	0011	M
 * 	4	0100	L
 * 	5	0101	K
 * 	6	0110	J
 * 	7	0111	I
 * 	8	1000	H
 * 	9	1001	G **
 * 	10	1010	F **
 * 	11	1011	E **
 * 	12	1100	D **
 * 	13	1101	C **
 * 	14	1110	B **
 * 	15	1111	A **
 *
 */



static void Board_Get_Hardware_Version(void)
{
	uint8_t version_bits = 0;
	uint8_t bit = 1;
	for(int i=0;i<4;i++)
	{
		uint8_t pind = Board_HW_version_pind_t[i];
		uint8_t port = PINPORT(pind);
		uint8_t pin = PINMASK(pind);
		if(Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin))
		{
			version_bits |= bit;
		}
		bit += bit;		// shift bit 1 place
	}

/*
Version 5A should have version_bits = 15 (0xf) = 0b1111
Version 5B should have version_bits = 14 (0xe) = 0b1110
Version 5C should have version_bits = 13 (0xd) = 0b1101
etc
*/
	uint8_t subversion = 15 - version_bits;		// invert bits to get number from 0 to 15
	AC210_hardware_version = 'A' + subversion;
}
//--------------------------------------------------------------------------------------------------
void AC210_brushless_pin_init(void)
{
	if(getParameter (BL_ENABLED) != 1) return;

	if(ps.parms[RC_MISC_OPTIONS] & 1)
	{
		AC210_remote_control_board = true;		// MHH:03/11/2023. RC_BOARD should be redundant now, was only added when no pin to identify RC board
		Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_ADC5_TEMPERATURE_PIND),IOCON_MODE_INACT | IOCON_FUNC3); /* P1.31, 20, AD0.5, also used to output a MagInt signal for test */
	}

//	if(ps.parms[BL_DIAGNOSTICS] & P_BL_LOGIC_BRUSHLESS) AC210_brushless_board = true;
	AC210_brushless_board = true;	// MHH:09/02/2026

	if(AC210_brushless_board)	// MHH:03/11/2023. Note: Currently RC board does not use Pin 1,23, but leave in in case this changes. Eg to change SIG100 baudrate
	{
		Chip_GPIO_WriteDirBit(LPC_GPIO,	PIN_DEF2(AC210_J2_6_STATE_L3_PIND),true);	// Set Pin 1.23 to output
		Chip_GPIO_WritePortBit(LPC_GPIO, PIN_DEF2(AC210_J2_6_STATE_L3_PIND),true);	// Turn on. May be able to get away with read pull-up in future??
	}
}

bool AC210_brushless_board;
bool AC210_remote_control_board;
void AC210_BoardInit(void)
{
	Board_Get_Hardware_Version();

	if(AC210_hardware_version == 'G')	// MHH:17/03/2026. To fix version coding error in AC300UAV6
	{
		if(Chip_GPIO_ReadPortBit(LPC_GPIO, PIN_DEF2(AC210_RC_PIND)) == false)	// MHH:17/03/2026
		{
			if(Chip_GPIO_ReadPortBit(LPC_GPIO, PIN_DEF2(AC210_BL_PIND)) == false)
			{
				AC210_hardware_version = 'I';
				AC210_remote_control_board = true;
				AC210_brushless_board = true;
			}
		}
	}

#ifdef MH_XXX	// MHH:08/02/2026
	if(Chip_GPIO_ReadPortBit(LPC_GPIO, PIN_DEF2(AC210_RC_PIND)) == false)	// MHH:03/11/2023
	{
		AC210_remote_control_board = true;		// MHH:23/12/2025.
		Chip_IOCON_PinMuxSet(LPC_IOCON,PIN_DEF2(AC210_ADC5_TEMPERATURE_PIND),IOCON_MODE_INACT | IOCON_FUNC3); /* P1.31, 20, AD0.5, also used to output a MagInt signal for test */
	}
	if(Chip_GPIO_ReadPortBit(LPC_GPIO, PIN_DEF2(AC210_BL_PIND)) == false)	// MHH:03/11/2023
	{
		AC210_brushless_board = true;
	}
#endif
	Chip_IOCON_DisableOD(LPC_IOCON, PIN_DEF2(AC210_RTC_SDA1_PIND));		// MHH:02/01/2019 Disable Open Drain, as we will be testing for DS3231 attached
	Chip_IOCON_DisableOD(LPC_IOCON, PIN_DEF2(AC210_RTC_SCL1_PIND));

	/* Initialize LEDs */

	AC210_LED_Init();
#ifdef MH_XXX	// MHH:19/03/2026
	if(AC210_remote_control_board == false)		// MHH:23/12/2025
	{
		AC210_LED_Init();
//		AC210_Switch_Init();		// MHH:05/01/2026
	}
#endif

#ifdef AC210_INTEGRATED
	AC210_Switch_Test_Integrated();		// MHH:30/01/2019
#endif


	Board_J2_Init();

	//	Chip_GPIO_WriteDirBit(LPC_GPIO,AC210_AUX_PORT_0, AC210_AUX_2_DIG3_PIN,  false);			// Set read mode for SIG60_HDC_PIN
//	Chip_GPIO_WriteDirBit(LPC_GPIO,	AC210_J2_DIG_PORT,  AC210_J2_ENABLE_PIN,true);	// Set J2 enable to output
	Chip_GPIO_WriteDirBit(LPC_GPIO,	PIN_DEF2(AC210_J2_ENABLE_PIND),true);	// Set J2 enable to output
//	Chip_GPIO_WritePortBit(LPC_GPIO, AC210_J2_DIG_PORT,  AC210_J2_ENABLE_PIN,true);	// Turn on (disable)
	Chip_GPIO_WritePortBit(LPC_GPIO, PIN_DEF2(AC210_J2_ENABLE_PIND),true);	// Turn on (disable)

//#define MH_TEST_INTERRUPT	// MHH:12/07/2023. This may be stopping aux2 serial from receiving data?
#ifdef MH_TEST_INTERRUPT		// MHH:15/12/2020
	Chip_GPIO_WriteDirBit(LPC_GPIO,	PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),true);	// Set J2 enable to output
	Chip_GPIO_WritePortBit(LPC_GPIO, PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),false);	// Turn off

#endif

}
//--------------------------------------------------------------------------------------------------
//#define DEICER_ON_SECS		10
//#define DEICER_OFF_SECS		10
static uint8_t deicer_secs;
static bool deicer_on;
#define DEICER_ENABLE_UNUSED	0
#define DEICER_ENABLE_OFF		1
#define DEICER_ENABLE_ON		2

void AC210_DeIcer_check(void)	// MHH:04/02/2021
{
	WORD deicer_enable = getParameter(DEICER_ENABLE);
	if(deicer_enable == DEICER_ENABLE_OFF)
	{
		if(deicer_on)
		{
			Chip_GPIO_WritePortBit(LPC_GPIO, PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),false);	// Turn off
			deicer_on = false;
			deicer_secs = 0;
		}
		return;
	}
	if(deicer_enable != DEICER_ENABLE_ON)
	{
		return;
	}

	if(deicer_on)
	{
		if(++deicer_secs >= getParameter(DEICER_ON_SECS))
		{
			Chip_GPIO_WritePortBit(LPC_GPIO, PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),false);	// Turn off
			deicer_on = false;
			deicer_secs = 0;
		}
	}
	else
	{
		if(++deicer_secs >= getParameter(DEICER_OFF_SECS))
		{
			Chip_GPIO_WritePortBit(LPC_GPIO, PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),true);	// Turn on
			deicer_on = true;
			deicer_secs = 0;
		}

	}
}
//--------------------------------------------------------------------------------------------------
void AC210_J2_Enable(void)
{
//	Chip_GPIO_WritePortBit(LPC_GPIO, AC210_J2_DIG_PORT,  AC210_J2_ENABLE_PIN, false);	// Turn off (enable)
	Chip_GPIO_WritePortBit(LPC_GPIO, PIN_DEF2(AC210_J2_ENABLE_PIND), false);	// Turn off (enable)
}
//----------------------------------------------------------------------------------------------------
#ifdef MH_J2_DIAGTEST
// Actually can not read on all pins as some (15 and 16) have diodes
// To simplify we will just read with the state test pins (pins 1 to 6).
static bool J2_diags_test_pin(int ix,bool expected)
{
	uint8_t pin;
	uint8_t port;
	uint8_t func;
	PINMUX_GRP_J2_T pdata;
//	for(int j=1;j<=6;j++)

// By testing all pins, we should also pick up any bridging.

	for(int j=1;j<=14;j++)
	{
		if(j == ix) continue;	// Cannot read/write at same time
		if(j == 7) continue;	// pin 7 is motor state sense, not a bridge suspect here, maybe in ADC pin group

		pdata = J2_map2[j];
		pin = pdata.pinnum;
		port = pdata.pingrp;
		func = pdata.modefunc;
		if(func == J2_NC) continue;		// not connected

		bool bit_on=Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin);
		if(bit_on != expected)
		{
			return false;
		}
	}
	return true;
}
//----------------------------------------------------------------------------------------------------
static uint8_t J2_diags_set_direction(uint8_t i,bool output)
{
	PINMUX_GRP_J2_T pdata;
	pdata = J2_map2[i];
	uint8_t pin = pdata.pinnum;
	uint8_t port = pdata.pingrp;
	uint8_t func = pdata.modefunc;
	if(func == J2_NC) return 1;		// not connected

	Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin, output);			// Set direction
	return 0;
}
//----------------------------------------------------------------------------------------------------
static uint8_t J2_diags_set_mode(uint8_t i,uint32_t mode)
{
	PINMUX_GRP_J2_T pdata;
	pdata = J2_map2[i];
	uint8_t pin = pdata.pinnum;
	uint8_t port = pdata.pingrp;
	uint8_t func = pdata.modefunc;
	if(func == J2_NC) return 1;		// not connected

	Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, mode);	// Set mode
	return 0;
}
//----------------------------------------------------------------------------------------------------
static uint8_t J2_diags_set_bit(uint8_t i,bool on)
{
	PINMUX_GRP_J2_T pdata;
	pdata = J2_map2[i];
	uint8_t pin = pdata.pinnum;
	uint8_t port = pdata.pingrp;
	uint8_t func = pdata.modefunc;
	if(func == J2_NC) return 1;		// not connected

	Chip_GPIO_WritePortBit(LPC_GPIO, port, pin, on);			// Turn on/off
	return 0;
}
//----------------------------------------------------------------------------------------------------
static void J2_init_all_pins(void)
{
	printf("Setting all J2 pins to FUNC0 and input, no pull-ups or pull-downs..");
	for(int i=1;i<=16;i++)
	{
		if(J2_diags_set_mode(i,IOCON_FUNC0)) continue;	// not connected
		J2_diags_set_direction(i,false);					// setting to input
	}
	printf("done\r\n");
}
#endif
//----------------------------------------------------------------------------------------------------
extern bool AC210_watchdog_active;		// defined in ac210.c
//----------------------------------------------------------------------------------------------------
#define P_BUFF_MAX		256
//int p_BufferLen;
char p_Buffer[P_BUFF_MAX];
void Board_UART0PutSTR(const char *buff)
{
#ifdef MH_XXX		// MHH:10/01/2026
	if(p_BufferLen <= 0) return;	// Ignore errors
	if(p_BufferLen >= P_BUFF_MAX-1)
	{
		DebugAbort("Board_UART0PutSTR:len > 256");
	}
	p_send(buff,p_BufferLen);
#endif
	int slen = strlen(buff);
	p_send(buff,slen);

}
//----------------------------------------------------------------------------------------------------
/* Idea: Make all test output to COM1 (main serial port)
    Can we make a macro that is similar to printf but outputs to COM1?
    Think we had one somewhere
	Idea: Can we save and restore all pin functions? Probably, but is it worth effort?
      	  Also, we may need to reinitialse functions - eg ADC, PWM

*/

typedef struct {
	uint16_t pin:8;			/* Pin. 1 to 100 */
	uint16_t pingrp:3;		/* Pin group */
	uint16_t pinnum:5;		/* Pin number */
} PINMUX_GRP_TEST_T;

STATIC const PINMUX_GRP_TEST_T All_pinmap[] =
{

		// Side 1

		{6,0,26},		/* AD0.3 */
		{7,0,25},		/* AD0.2 */
		{8,0,24},		/* AD0.1 */
		{9,0,23},		/* AD0.0 */

		// Note: Pin 21 = AD0.4, but is next to XTAL pin and NC

		// Side 2

		{32,1,18},		/* J2.1 */
		{33,1,19},		/* J2.2 */
		{34,1,20},		/* J2.3 */
		{35,1,21},		/* J2.4 */
		{36,1,22},		/* J2.5 */
		{37,1,23},		/* J2.6 */

		// Note: J2.7 is connected to AD0.0

		{38,1,24},		/* J2.8 */

		// J2.9 = NC

		{39,1,25},		/* J2.10 */
		{40,1,26},		/* J2.11 */

		// 41 = GND, 42 = VDD

		// J2.12 = PWM1.3
		// J2.13 = PWM1.2
		// J2.14 = PWM1.1

		{43,1,27},		/* J2.15 */
		{44,1,28},		/* J2.16 */

		// Side 3

		// 51 = RLED2 but is isolated
		// 53 = P2.10 which is special boot pin. Should be pulled up. Leave for this test

		{56,0,22},		/* RPM interrupt pin */

		// switches

		{57,0,21},		/* J3.6 */
		{58,0,20},		/* J3.4 */
		{59,0,19},		/* J3.2 */
		{60,0,18},		/* J3.1 */

		// Note: 61 through 63 pinnum is not sequential

		{61,0,17},		/* J3.3 */
		{62,0,15},		/* J3.5 */
		{63,0,16},		/* J3.7 */

		// J3.8 = GND

		// LEDS

		{64,2,9},		/* J4.1 */
		{65,2,8},		/* J4.3 */
		{66,2,7},		/* J4.5 */
		{67,2,6},		/* J4.2 */
		{68,2,5},		/* J4.4 */
		{69,2,4},		/* J4.6 */


		// PWM 1 to 3

		{73,2,2},		/* PWM1.3 */
		{74,2,1},		/* PWM1.2 */
		{75,2,0},		/* PWM1.1 */


		// Side 4

		// 76 to 79 = Flash SSP logic

		// 80 and 81 are used for COM3, which we should see if working.
		// 98 and 99 are used from COM0, which we will see if not working

		{0,0,0},		/* End of table */

};
//----------------------------------------------------------------------------------------------------
static void bridge_error(int i,int e)
{
	PINMUX_GRP_TEST_T pdata = All_pinmap[i];
	PRINTF("Pin %d [p%d.%d] Error %d\r\n",pdata.pin,pdata.pingrp, pdata.pinnum,e);
}
//----------------------------------------------------------------------------------------------------
void AC210_selftest_bridging(void)
{
// First J2

	AC210_watchdog_active = false;		// Turn off watchdog

	PRINTF("AC210_selftest_bridging:\r\n");
	PRINTF("AC210 board must be powered separately, and NOT connected to power board\r\n");

//	uint8_t pin;
//	uint8_t pinnum;
//	uint8_t port;
	PINMUX_GRP_TEST_T pdata;
	PINMUX_GRP_TEST_T next_pdata;

// First step is to put all pins into input mode, with pull down
	//IOCON_MODE_PULLDOWN

	int cnt = 0;
	for(int i=0;;i++)
	{
		pdata = All_pinmap[i];
		if(pdata.pin == 0) break;

		cnt++;

		Chip_IOCON_PinMuxSet(LPC_IOCON, pdata.pingrp, pdata.pinnum, IOCON_FUNC0 | IOCON_MODE_PULLDOWN);	// Set mode

		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  false);		// Set read mode
	}
	PRINTF("Initialised %d pins\r\n",cnt);

/* Now, if we loop through again, and for each pin, if there is a consecutive pin (ie they are physically next to each other)
 * we set the current pin to write mode, then write 1 to pin, then test next pin to see if it turns on.
*/

	int ecnt = 0;
	int ccnt = 0;

	for(int i=0;;i++)
	{
		pdata = All_pinmap[i];
		if(pdata.pin == 0) break;

		next_pdata = All_pinmap[i+1];

		if(next_pdata.pin != pdata.pin +1) continue;

		ccnt++;

		// if we drop through then we have 2 consecutive pins.

		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  true);			// Set write mode

		for(int j=0;j<10;j++)
		{
			Chip_GPIO_WritePortBit(LPC_GPIO, pdata.pingrp, pdata.pinnum, false);			// Turn off
			wait_ms(1);
			if(Chip_GPIO_ReadPortBit(LPC_GPIO,next_pdata.pingrp, next_pdata.pinnum))
			{
				bridge_error(i,1);
				ecnt++;
				break;
			}
			Chip_GPIO_WritePortBit(LPC_GPIO, pdata.pingrp, pdata.pinnum, true);			// Turn on
			wait_ms(1);
			if(Chip_GPIO_ReadPortBit(LPC_GPIO,next_pdata.pingrp, next_pdata.pinnum))
			{
				bridge_error(i,2);
				ecnt++;
				break;
			}
			Chip_GPIO_WritePortBit(LPC_GPIO, pdata.pingrp, pdata.pinnum, false);			// Turn off again
			wait_ms(1);
			if(Chip_GPIO_ReadPortBit(LPC_GPIO,next_pdata.pingrp, next_pdata.pinnum))
			{
				bridge_error(i,3);			// Should never get here
				ecnt++;
				break;
			}
		}
		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  false);			// restore read mode
	}

	PRINTF("Finished test: %d pins tested, %d errors found\r\n",ccnt,ecnt);
	PRINTF("Rebooting in 1 second\r\n");
	wait_ms(1000);
//	NVIC_SystemReset();
	AC210_reboot();
}
//----------------------------------------------------------------------------------------------------
#ifdef MH_J2_DIAGTEST
void AC210_selftest_J2(void)
{
	uint8_t good_pin_t[16];
	uint8_t good_pin_ix=0;

	AC210_watchdog_active = false;		// Turn off watchdog
	PRINTF("AC210_selftest_J2\r\n");

	/*
	 * The idea is to check that all the pins on J2 (motor state etc) are changed to input only (could be HiZ too), no pull-ups
	 * or pull-downs, then one by one we turn the pins on and see if they can be read by the input pins.
	 * This relies on all the pins on J2 being joined to the same net by hardware.
	 * Another possible test is to see if there is any bridging. This will rely on not having anything connecting pins.
	 * We may be able to save the state/mode of each pin then restore after finishing each test, or we could just re-run the
	 * board initialisation routine, or could reboot.
	 */
	uint8_t pin;
	uint8_t port;
	uint8_t func;
	PINMUX_GRP_J2_T pdata;

	J2_init_all_pins();

/*
 *  Now step through each pin, except for pin 15 and 16, put in output mode and set value on and see if it can be read by all other pins
 */
	int etot=0;
	for(int i=1;i<=14;i++)
	{
		pdata = J2_map2[i];
		pin = pdata.pinnum;
		port = pdata.pingrp;
		func = pdata.modefunc;
		if(func == J2_NC) continue;		// not connected

		printf("Test J2 pin: %d, port=%d, pin=%d\r\n",i,port,pin);

		Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin, true);			// Set pin to output

		bool pin_good = true;

		for(int j=0;j<100;j++)		// test pin 100 times
		{
			Chip_GPIO_WritePortBit(LPC_GPIO, port, pin, false);			// Turn off
			wait_ms(1);		// because of caps
			if(i >= 15)
			{
				PRINTF("i>=15\r\n");
			}

			if(J2_diags_test_pin(i,false)==false)
			{
				etot++;
				pin_good = false;
				PRINTF("  Pin bad (expecting off), j=%d\r\n",j);
				break;
			}
			Chip_GPIO_WritePortBit(LPC_GPIO, port, pin, true);			// Turn on
			wait_ms(1);		// because of caps
			if(J2_diags_test_pin(i,true)==false)
			{
				etot++;
				pin_good = false;
				PRINTF("  Pin bad (expecting on), j=%d\r\n",j);
				break;
			}
		}
		if(pin_good)
		{
			good_pin_t[good_pin_ix++] = i;
		}

		Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin, false);			// Set pin back to input

		// We also need to be careful that hardware is attached. Use etot and exit if too many errors;

		if(i == 3)
		{
			if(etot >= 2)
			{
				PRINTF("Too many errors, hardware may not be attached, exiting\r\n");
				break;
			}
		}
	}
	// Now test pin 15 and 16. We do this by putting them one at a time into normal operating mode
	// which is input and pullup, then externally (any pin, 1-14, set to 1, then read pin 15 or 16, then set to zero
	// and repeat. May need to delay as has a cap and resistor.


	uint8_t output_pin = good_pin_t[0];
	PRINTF("Testing pins 15 and 16, output_pin=%d\r\n",output_pin);
	J2_diags_set_direction(output_pin,true);					// setting to output



	J2_diags_set_bit(output_pin,true);		// Setting output_pin=0
	wait_ms(10);										// wait

	for(int i=15;i<=16;i++)
	{
		pdata = J2_map2[i];
		pin = pdata.pinnum;
		port = pdata.pingrp;
		func = pdata.modefunc;

		PRINTF("Test J2 pin: %d, port=%d, pin=%d\r\n",i,port,pin);

		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_PULLUP | IOCON_FUNC0);	// FUNC0 + PULLUP, should already be read mode

		for(int j=0;j<100;j++)		// test pin 100 times
		{
			J2_diags_set_bit(output_pin,true);		// Setting output_pin=1
			wait_ms(10);										// wait
			if(Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin) == false)	// Should be true
			{
				etot++;
				PRINTF("  Pin bad (expecting on), j=%d\r\n",j);
				break;
			}
			J2_diags_set_bit(output_pin,false);		// Setting output_pin=0
			wait_ms(1);										// wait
			if(Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin) == true)	// Should be false
			{
				etot++;
				PRINTF("  Pin bad (expecting off), j=%d\r\n",j);
				break;
			}
		}
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_FUNC0);	// turn pullup off
	}

// OK, finished this test, now to try and restore pins to normal.....
	PRINTF("Finished test, not trying to restore pins, reboot if needed\r\n");

}
#endif
//----------------------------------------------------------------------------------------------------
STATIC const PINMUX_GRP_TEST_T J3_pinmap[] =
{
		// switches

		{60,0,18},		/* J3.1 */
		{59,0,19},		/* J3.2 */
		{61,0,17},		/* J3.3 */
		{58,0,20},		/* J3.4 */
		{62,0,15},		/* J3.5 */
		{57,0,21},		/* J3.6 */
		{63,0,16},		/* J3.7 */

		// J3.8 = GND

		{0,0,0},		/* End of table */

};
//----------------------------------------------------------------------------------------------------
void AC210_selftest_J3(void)
{
	PINMUX_GRP_TEST_T pdata;
	PINMUX_GRP_TEST_T pdata2;

	AC210_watchdog_active = false;		// Turn off watchdog
	PRINTF("AC210_selftest_J3\r\n");

	int cnt = 0;
	for(int i=0;;i++)
	{
		pdata = J3_pinmap[i];
		if(pdata.pin == 0) break;

		cnt++;

//		Chip_IOCON_PinMuxSet(LPC_IOCON, pdata.pingrp, pdata.pinnum, IOCON_FUNC0);	// Set mode, no pull up/down

		// Using pull-up as this is way switch pins normall operate

		Chip_IOCON_PinMuxSet(LPC_IOCON, pdata.pingrp, pdata.pinnum, IOCON_FUNC0 | IOCON_MODE_PULLUP);	// Set mode

		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  false);		// Set read mode
	}
	PRINTF("Initialised %d pins\r\n",cnt);

// Cycle through each pin in J3, turning each pin off then on and make sure that all others can read it.

	int ecnt = 0;
	for(int i=0;;i++)
	{
		pdata = J3_pinmap[i];
		if(pdata.pin == 0) break;

		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  true);			// Set write mode

		Chip_GPIO_WritePortBit(LPC_GPIO, pdata.pingrp, pdata.pinnum, false);			// Turn off
		for(int j=0;;j++)
		{
			if(j == i) continue;

			pdata2 = J3_pinmap[j];
			if(pdata2.pin == 0) break;

			if(Chip_GPIO_ReadPortBit(LPC_GPIO,pdata2.pingrp, pdata2.pinnum))
			{
				PRINTF("J3.%d OFF, J3.%d ON\r\n",i+1,j+1);
				ecnt++;
			}
		}
		Chip_GPIO_WritePortBit(LPC_GPIO, pdata.pingrp, pdata.pinnum, true);			// Turn on
		for(int j=0;;j++)
		{
			if(j == i) continue;

			pdata2 = J3_pinmap[j];
			if(pdata2.pin == 0) break;

			// This should never happen as we are pulling test pins up.

			if(Chip_GPIO_ReadPortBit(LPC_GPIO,pdata2.pingrp, pdata2.pinnum) == false)
			{
				PRINTF("J3.%d ON, J3.%d OFF\r\n",i+1,j+1);
				ecnt++;
			}
		}
		Chip_GPIO_WritePortBit(LPC_GPIO, pdata.pingrp, pdata.pinnum, false);			// Turn off again
		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  false);			// Set read mode
	}
	PRINTF("Finished test: %d pins tested, %d errors found\r\n",cnt,ecnt);

#ifdef MH_REBOOT
	PRINTF("Rebooting in 1 second\r\n");
	wait_ms(1000);
	NVIC_SystemReset();
#endif

}
//----------------------------------------------------------------------------------------------------
STATIC const PINMUX_GRP_TEST_T J4_pinmap[] =
{
		// LEDS

		{64,2,9},		/* J4.1 */
		{67,2,6},		/* J4.2 */
		{65,2,8},		/* J4.3 */
		{68,2,5},		/* J4.4 */
		{66,2,7},		/* J4.5 */
		{69,2,4},		/* J4.6 */

		{0,0,0},		/* End of table */

};
//----------------------------------------------------------------------------------------------------
// Very similar test to J3, but use pull down instead of pull up as these pins are output pins.
void AC210_selftest_J4(void)
{
	PINMUX_GRP_TEST_T pdata;
	PINMUX_GRP_TEST_T pdata2;

	AC210_watchdog_active = false;		// Turn off watchdog
	PRINTF("AC210_selftest_J4\r\n");

	int cnt = 0;
	for(int i=0;;i++)
	{
		pdata = J4_pinmap[i];
		if(pdata.pin == 0) break;

		cnt++;

//		Chip_IOCON_PinMuxSet(LPC_IOCON, pdata.pingrp, pdata.pinnum, IOCON_FUNC0);	// Set mode, no pull up/down

		// Using pull-up as this is way switch pins normall operate

		Chip_IOCON_PinMuxSet(LPC_IOCON, pdata.pingrp, pdata.pinnum, IOCON_FUNC0 | IOCON_MODE_PULLDOWN);	// Set mode

		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  false);		// Set read mode
	}
	PRINTF("Initialised %d pins\r\n",cnt);

// Cycle through each pin in J3, turning each pin off then on and make sure that all others can read it.

	int ecnt = 0;
	for(int i=0;;i++)
	{
		pdata = J4_pinmap[i];
		if(pdata.pin == 0) break;

		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  true);			// Set write mode

		Chip_GPIO_WritePortBit(LPC_GPIO, pdata.pingrp, pdata.pinnum, true);			// Turn on
		for(int j=0;;j++)
		{
			if(j == i) continue;

			pdata2 = J3_pinmap[j];
			if(pdata2.pin == 0) break;

			// This should never happen as we are pulling test pins up.

			if(Chip_GPIO_ReadPortBit(LPC_GPIO,pdata2.pingrp, pdata2.pinnum) == false)
			{
				PRINTF("J4.%d ON, J4.%d OFF\r\n",i+1,j+1);
				ecnt++;
			}
		}

		Chip_GPIO_WritePortBit(LPC_GPIO, pdata.pingrp, pdata.pinnum, false);			// Turn off
		for(int j=0;;j++)
		{
			if(j == i) continue;

			pdata2 = J4_pinmap[j];
			if(pdata2.pin == 0) break;

			if(Chip_GPIO_ReadPortBit(LPC_GPIO,pdata2.pingrp, pdata2.pinnum))
			{
				PRINTF("J4.%d OFF, J4.%d ON\r\n",i+1,j+1);
				ecnt++;
			}
		}
		Chip_GPIO_WriteDirBit(LPC_GPIO, pdata.pingrp, pdata.pinnum,  false);			// Set read mode
	}
	PRINTF("Finished test: %d pins tested, %d errors found\r\n",cnt,ecnt);
	AC210_watchdog_active = true;		// Turn on  watchdog

}
//========================================================================================================
void AC210_SET_Pin_Mask(uint8_t port,uint32_t pin_mask)
{
	LPC_GPIO_T *pLPC_port;
	pLPC_port = LPC_GPIO + port;
	pLPC_port->SET |= pin_mask;
}
//----------------------------------------------------------------------------------------------------
void AC210_CLR_Pin_Mask(uint8_t port,uint32_t pin_mask)
{
	LPC_GPIO_T *pLPC_port;
	pLPC_port = LPC_GPIO + port;
	pLPC_port->CLR |= pin_mask;
}
//----------------------------------------------------------------------------------------------------
// This routine adapted to use micro seconds, not millisecs
/* Set timer interval value */
static uint32_t RIT_usecs;
static uint32_t RIT_cmp_value;
void Chip_RIT_SetTimerInterval_us(LPC_RITIMER_T *pRITimer, uint32_t time_us)
{
	RIT_usecs = time_us;

	/* Determine approximate compare value based on clock rate and passed interval */

	uint32_t clock_rate = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_RIT);
	uint32_t us_per_clock = clock_rate / 1000000;

	// Note: Because my LPC1768 has clock_rate of exactly 24,000,000 the us_per_clock is exactly 24, so can use this logic.
	// If clock_rate was not a multiple of 1,000,000 would have to do something like:
	// us_per_clock_x_1000 = clock_rate/1000;
	// cmp_value = (us_per_clock_x_1000 * time_us + 500)/1000

	uint32_t cmp_value = us_per_clock * time_us;

	/* Set timer compare value */
	Chip_RIT_SetCOMPVAL(pRITimer, cmp_value);

	RIT_cmp_value = cmp_value;

	/* Set timer enable clear bit to clear timer to 0 whenever
	   counter value equals the contents of RICOMPVAL */
	Chip_RIT_EnableCTRL(pRITimer, RIT_CTRL_ENCLR);
	pRITimer->COUNTER = 0;	// MHH: 28/08/2017
}

// Following routines from ritimer.c example program

static volatile bool On;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	RIT interrupt handler
 * @return	Nothing
 */

#ifdef MH_RIT_DEBUG
static uint32_t RIT_ix;
static uint32_t RIT_last_us;
static uint32_t RIT_us_tbl[64];
#endif
static const bool AC210_AUX_RPM_dig1_dig2 = true;		// MHH 7/02/2018. Don't use DIG3 anymore, to be compatible with AC200

void RIT_IRQHandler(void)
{
	/* Clear interrupt */
	Chip_RIT_ClearInt(LPC_RITIMER);

	if(RIT_usecs > 364500)		// equivalent to 200 rpm on Rotax
	{
		return;					// then do not toggle, act as if zero
	}


#ifdef MH_RIT_DEBUG
	uint32_t time_us = us_ticker_read();
	uint32_t elapsed_us = time_us - RIT_last_us;
	RIT_last_us = time_us;
	RIT_us_tbl[RIT_ix++] = elapsed_us;
	RIT_ix &= 63;
#endif
	/* Toggle Pin  */
	On = (bool) !On;
	if(AC210_AUX_RPM_dig1_dig2)
	{

#define MH_PINMASK
#ifdef MH_PINMASK
		uint32_t pin_mask = (1 << PINMASK(AC210_AUX_3_DIG1_TX3_PIND)) | (1 << PINMASK(AC210_AUX_4_DIG2_RX3_PIND));
		if(On)
			AC210_SET_Pin_Mask(PINPORT(AC210_AUX_3_DIG1_TX3_PIND),pin_mask);
		else
			AC210_CLR_Pin_Mask(PINPORT(AC210_AUX_3_DIG1_TX3_PIND),pin_mask);
#else

		Chip_GPIO_WritePortBit(LPC_GPIO,AC210_AUX_DIG_COM3_PORT,AC210_AUX_3_DIG1_TX3_PIN, On);			// Toggle
		Chip_GPIO_WritePortBit(LPC_GPIO,AC210_AUX_DIG_COM3_PORT,AC210_AUX_4_DIG2_RX3_PIN, On);			// Toggle
#endif
	}
	else
	{
		Chip_GPIO_WritePortBit(LPC_GPIO,PIN_DEF2(AC210_AUX_2_DIG3_CAN_PIND), On);			// Toggle
	}
}
//----------------------------------------------------------------------------------------------------
static void RIT_init_pin(uint8_t port,uint8_t pin)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_INACT | IOCON_FUNC0);	// Set mode
	Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin,  true);			// Set write mode
	Chip_GPIO_WritePortBit(LPC_GPIO, port, pin, true);			// Turn on
}
//----------------------------------------------------------------------------------------------------
void RIT_init(uint32_t usecs)
{
	On = true;
//	Board_LED_Set(0, On);
	// Note: We are using Dig3 instead of Dig1 and Dig 2 as the AC200 does. This is because Dig1 and Dig2 are used by my debug
	// reporting routines (printf). Can always change to Dig1 and Dig 2 if becomes a problem.

	if(AC210_AUX_RPM_dig1_dig2)
	{
		AC210_Disable_COM3();		// ac210_comms.c
		RIT_init_pin(PIN_DEF2(AC210_AUX_3_DIG1_TX3_PIND));	// #82, P4.28
		RIT_init_pin(PIN_DEF2(AC210_AUX_4_DIG2_RX3_PIND));	// #85, P4.29
	}
	else
	{
		RIT_init_pin(PIN_DEF2(AC210_AUX_2_DIG3_CAN_PIND));	// #80, P0.5
	}
//	Chip_IOCON_PinMuxSet(LPC_IOCON, AC210_AUX_PORT_0, AC210_AUX_2_DIG3_PIN, IOCON_MODE_INACT | IOCON_FUNC0);	// Set mode
//	Chip_GPIO_WriteDirBit(LPC_GPIO,AC210_AUX_PORT_0, AC210_AUX_2_DIG3_PIN,  true);			// Set write mode
//	Chip_GPIO_WritePortBit(LPC_GPIO,AC210_AUX_PORT_0, AC210_AUX_2_DIG3_PIN, On);			// Turn on

	/* Initialize RITimer */
	Chip_RIT_Init(LPC_RITIMER);

	/* Configure RIT for a 1s interrupt tick rate */
	Chip_RIT_SetTimerInterval_us(LPC_RITIMER, usecs);	// if 5 ms = 200 Hz = 100 Revs per sec = 6000 rpm
//	Chip_RIT_SetTimerInterval(LPC_RITIMER, usecs);	// if 5 ms = 200 Hz = 100 Revs per sec = 6000 rpm

	NVIC_EnableIRQ(RITIMER_IRQn);
}
//----------------------------------------------------------------------------------------------------
// Note: This uses AC210_AUX_2_DIG3_PIN (P0.5) which is also the CAN TX pin
void AC210_RIT_Set(uint32_t usecs)
{
	if(RIT_usecs == 0)
	{
		RIT_init(usecs);
		return;
	}
	if(usecs == RIT_usecs) return;

	Chip_RIT_SetTimerInterval_us(LPC_RITIMER, usecs);	// if 5 ms = 200 Hz = 100 Revs per sec = 6000 rpm
}
//----------------------------------------------------------------------------------------------------
WORD AC210_board_test(WORD val)
{
	WORD rv=0;		// return val
	switch(val)
	{
	default:		// error
		rv = 1;
		break;
	case 1:
		AC210_selftest_bridging();
		break;
	case 2:
//		AC210_selftest_J2();
		break;
	case 3:
		AC210_selftest_J3();
		break;
	case 4:
		AC210_selftest_J4();
		break;

	case 5:	// MHH:29/10/2023
		AC210_Test_Fine_Coarse();
		break;

	}
	return rv;
}

