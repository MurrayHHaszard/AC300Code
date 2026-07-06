/*
 * @brief NXP LPC1769 LPCXpresso Sysinit file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
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

#include "ac210_global.h"
#include "ac210_pins.h"
#include "board.h"

/* The System initialization code is called prior to the application and
   initializes the board for run-time operation. Board initialization
   includes clock setup and default pin muxing configuration. */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Pin muxing configuration */
STATIC const PINMUX_GRP_T pinmuxing[] = {
//	{0,  0,   IOCON_MODE_INACT | IOCON_FUNC2},	/* TXD3 */
//	{0,  1,   IOCON_MODE_INACT | IOCON_FUNC2},	/* RXD3 */


// Hardware version pins. 4 pins reserved: P1.0, P1.1, P1.4 and P1.8 (chosen because they are adjacent)
// All pins are input and pulled up (version 5A)
// Version 5B has P1.0 grounded.
//         5C has P1.1 grounded
//         5D has P1.0 and P1.1 grounded
//         5D has P1.4 grounded
// etc. Room for 16 versions using this system.

	{PIN_DEF2(AC210_HW_VERSION_0_PIND),   IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_HW_VERSION_1_PIND),   IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_HW_VERSION_2_PIND),   IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_HW_VERSION_3_PIND),   IOCON_MODE_PULLUP | IOCON_FUNC0},

	// MHH:03/11/2023. Following 2 added to identify RC board and BL board. If true then they are grounded.

	{PIN_DEF2(AC210_RC_PIND),   IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_BL_PIND),   IOCON_MODE_PULLUP | IOCON_FUNC0},

#ifdef MH_RTC
//	{0,  0,   IOCON_MODE_INACT | IOCON_FUNC3},	/* SDA1 (J2-9) */
//	{0,  1,   IOCON_MODE_INACT | IOCON_FUNC3},	/* SCL1 (J2-10) */
	{PIN_DEF2(AC210_RTC_SDA1_PIND),   IOCON_MODE_PULLDOWN | IOCON_FUNC0},	/* Now I2C setup done in board.c:Board_I2C_Init() */
	{PIN_DEF2(AC210_RTC_SCL1_PIND),   IOCON_MODE_PULLDOWN | IOCON_FUNC0},	/*  */
	{PIN_DEF2(AC210_RTC_RESET_PIND),  IOCON_FUNC0},	/* Cannot pulldown */
#endif

// Main Serial Port

	{PIN_DEF2(AC210_SERIAL_TX_PIND),   IOCON_MODE_INACT | IOCON_FUNC1},
	{PIN_DEF2(AC210_SERIAL_RX_PIND),   IOCON_MODE_INACT | IOCON_FUNC1},

// Auxiliary Port.
// Note: From 5C we can use AUX Dig3 and Dig4 as either an extra  serial port or a CAN port depending on parameters.
// To do this p0.10 (TXD2) and p0.11 (RXD2) are connected to pins p0.5 (CAN-TD2) and p0.4 (CAN-RD2) respectively.
// If we are not using any set of pins then they should be set to high-Z, though may be enough just to have as read mode.

// MHH:15/12/2020. Was defaulting to CAN mode. Change to GPIO default, so can use CN5 pins to test controlling mh_interrupt_1 pcb
//	{PIN_DEF2(AC210_AUX_2_DIG3_CAN_PIND),IOCON_MODE_INACT | IOCON_FUNC2},	/* P0.5, 80, CAN-TD2, also used to output a MagInt signal for test */
//	{PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),IOCON_MODE_INACT | IOCON_FUNC2},	/* P0.4, 81, CAN-RD2 */
	{PIN_DEF2(AC210_AUX_2_DIG3_CAN_PIND),IOCON_MODE_INACT | IOCON_FUNC0},	/* P0.5, 80, CAN-TD2, also used to output a MagInt signal for test */
	{PIN_DEF2(AC210_AUX_9_DIG4_CAN_PIND),IOCON_MODE_INACT | IOCON_FUNC0},	/* P0.4, 81, CAN-RD2 */

//	{PIN_DEF2(AC210_AUX_2_DIG3_TX2_PIND),IOCON_MODE_INACT | IOCON_FUNC0},	/* P0.10, 48, TXD2 */
	{PIN_DEF2(AC210_AUX_2_DIG3_TX2_PIND),IOCON_MODE_INACT | IOCON_FUNC1},	/* MHH:11/07/2023. P0.10, 48, TXD2 */
//	{PIN_DEF2(AC210_AUX_2_DIG3_TX2_PIND),IOCON_MODE_PULLUP | IOCON_FUNC0},	/* P0.10, 48, TXD2 */
//	{PIN_DEF2(AC210_AUX_9_DIG4_RX2_PIND),IOCON_MODE_INACT | IOCON_FUNC0},	/* P0.11, 49, RXD2 */
	{PIN_DEF2(AC210_AUX_9_DIG4_RX2_PIND),IOCON_MODE_INACT | IOCON_FUNC1},	/* MHH:11/07/2023. P0.11, 49, RXD2 */

	{PIN_DEF2(AC210_AUX_3_DIG1_TX3_PIND),   IOCON_MODE_INACT | IOCON_FUNC3},
	{PIN_DEF2(AC210_AUX_4_DIG2_RX3_PIND),   IOCON_MODE_INACT | IOCON_FUNC3},
	{PIN_DEF2(AC210_AUX_7_ADC_DIMMER_PIND),  IOCON_MODE_INACT | IOCON_FUNC1},	/* AD0.3 */
//	{PIN_DEF2(AC210_AUX_8_ADC_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},	/* AD0.4, but will use for SIG60_HTC if needed, so not in ADC mode */
	{PIN_DEF2(AC210_AUX_8_ADC_PIND),  IOCON_MODE_INACT | IOCON_FUNC3},	/* MHH:27/01/2024: Try using as AD0.4, for ADC slider control. */

// EEPROM

	{PIN_DEF2(AC210_I2C0_DATA_PIND),  IOCON_MODE_INACT | IOCON_FUNC1},
	{PIN_DEF2(AC210_I2C0_CLOCK_PIND), IOCON_MODE_INACT | IOCON_FUNC1},

// SSP for Flash memory

	{PIN_DEF2(AC210_SSP1_SSEL_PIND), IOCON_MODE_INACT | IOCON_FUNC0},	// Change to GPIO func for SSEL
	{PIN_DEF2(AC210_SSP1_SCK_PIND), IOCON_MODE_INACT | IOCON_FUNC2},
	{PIN_DEF2(AC210_SSP1_MISO_PIND), IOCON_MODE_INACT | IOCON_FUNC2},
	{PIN_DEF2(AC210_SSP1_MOSI_PIND), IOCON_MODE_INACT | IOCON_FUNC2},

// Leds

#ifdef MH_XXX			// MHH:23/12/2025, Now done in AC210_LED_Init(void)
	{PIN_DEF2(AC210_LED_0_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},

#ifdef AC210_4LEDS
	{PIN_DEF2(AC210_LED_1_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},	// These 2 added for 4 LED display.
	{PIN_DEF2(AC210_LED_2_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
#endif

	{PIN_DEF2(AC210_LED_3_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_LED_4_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_LED_5_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_LED_6_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_LED_7_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_LED_8_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_LED_9_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
#endif
// Switches

	{PIN_DEF2(AC210_SWITCH_1_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_SWITCH_2_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_SWITCH_3_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_SWITCH_4_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_SWITCH_5_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_SWITCH_6_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},
	{PIN_DEF2(AC210_SWITCH_7_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},

// J1

	{PIN_DEF2(AC210_J1_MOTOR_CURRENT_SENSE_PIND),  IOCON_MODE_INACT | IOCON_FUNC1},	/* J1 Motor Current Sense AD0.1 */
	{PIN_DEF2(AC210_J1_MOTOR_SUPPLY_SENSE_PIND),  IOCON_MODE_INACT | IOCON_FUNC1},	/* J1 Motor Supply Sense AD0.2 (Battery Voltage) */
	{PIN_DEF2(AC210_J1_MAG_INT_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},

// J2
	{PIN_DEF2(AC210_J2_ENABLE_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},	/* Pull up until in write mode */

	{PIN_DEF2(AC210_J2_1_STATE_U1_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_J2_2_STATE_U2_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_J2_3_STATE_U3_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_J2_4_STATE_L1_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_J2_5_STATE_L2_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_J2_6_STATE_L3_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},

	{PIN_DEF2(AC210_J2_7_MOTOR_STATE_SENSE_PIND), IOCON_MODE_INACT | IOCON_FUNC1},	/* J2_7 Motor State Sense AD0.0 */

	{PIN_DEF2(AC210_J2_8_DRIVE_U1_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	// J2_9 Not Connected
	{PIN_DEF2(AC210_J2_10_DRIVE_U2_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},
	{PIN_DEF2(AC210_J2_11_DRIVE_U3_PIND),  IOCON_MODE_INACT | IOCON_FUNC0},

	{PIN_DEF2(AC210_J2_12_DRIVE_L1_PIND),  IOCON_MODE_INACT | IOCON_FUNC1},	/* PWM1.3 */
	{PIN_DEF2(AC210_J2_13_DRIVE_L2_PIND),  IOCON_MODE_INACT | IOCON_FUNC1},	/* PWM1.2 */
	{PIN_DEF2(AC210_J2_14_DRIVE_L3_PIND),  IOCON_MODE_INACT | IOCON_FUNC1},	/* PWM1.1 */

	{PIN_DEF2(AC210_J2_15_COARSE_SENSE_PIND),  IOCON_MODE_PULLUP | IOCON_FUNC0},	/* J2_15 Coarse Sense */
	{PIN_DEF2(AC210_J2_16_FINE_SENSE_PIND),    IOCON_MODE_PULLUP | IOCON_FUNC0},	/* J2_16 Fine Sense */
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Sets up system pin muxing */
void Board_SetupMuxing(void)
{
	Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing, sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));
}

/* Setup system clocking */
void Board_SetupClocking(void)
{
	Chip_SetupXtalClocking();

	/* Setup FLASH access to 4 clocks (100MHz clock) */
	Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU);
}

/* Set up and initialize hardware prior to call to main */
void Board_SystemInit(void)
{
	Board_SetupMuxing();
	Board_SetupClocking();
}
