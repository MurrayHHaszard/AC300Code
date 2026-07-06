/*
 * ac210_state.c
 *
 *  Created on: 16/11/2016
 *      Author: Murray
 */

#include "global.h"
#include "param.h"
#include "ac210_global.h"

//------------------------------------------------------------------------------------
#define STATE_U1_BIT	(1 << PINMASK(AC210_J2_1_STATE_U1_PIND))
#define STATE_U2_BIT	(1 << PINMASK(AC210_J2_2_STATE_U2_PIND))
#define STATE_U3_BIT	(1 << PINMASK(AC210_J2_3_STATE_U3_PIND))

#define STATE_L1_BIT	(1 << PINMASK(AC210_J2_4_STATE_L1_PIND))
#define STATE_L2_BIT	(1 << PINMASK(AC210_J2_5_STATE_L2_PIND))
#define STATE_L3_BIT	(1 << PINMASK(AC210_J2_6_STATE_L3_PIND))

#define STATE_BIT_MASK	(STATE_U1_BIT | STATE_U2_BIT | STATE_U3_BIT | STATE_L1_BIT | STATE_L2_BIT | STATE_L3_BIT)

#define STATE_FINE_HIGH		STATE_U1_BIT
#define STATE_COARSE_HIGH	STATE_U2_BIT
#define STATE_FEATHER_HIGH	STATE_U3_BIT

#define STATE_FINE_LOW		STATE_L1_BIT
#define STATE_COARSE_LOW	STATE_L2_BIT
#define STATE_FEATHER_LOW	STATE_L3_BIT

#define STATE_STOP_DIR		0
#define	STATE_FINE_DIR			(STATE_FINE_HIGH | STATE_COARSE_LOW)
#define	STATE_COARSE_DIR		(STATE_COARSE_HIGH | STATE_FINE_LOW)
#define	STATE_INTO_FEATHER_DIR	(STATE_FEATHER_HIGH | STATE_FINE_LOW)
#define	STATE_OUTOF_FEATHER_DIR	(STATE_FINE_HIGH | STATE_FEATHER_LOW)

#define STATE_PORT_NUM	PINPORT(AC210_J2_1_STATE_U1_PIND)			// Should be 1

LPC_GPIO_T *pLPC_STATE_PORT;
//----------------------------------------------------------------------
void AC210_STATE_Init(void)
{
// Set pin direction to output for STATE pins
	pLPC_STATE_PORT = LPC_GPIO + STATE_PORT_NUM;
	pLPC_STATE_PORT->DIR |= STATE_BIT_MASK;		// Set all drive pins to output
	pLPC_STATE_PORT->CLR |= STATE_BIT_MASK;		// Everything off
//	PWM_Init();
}

//----------------------------------------------------------------------
void STATE_SetPins(uint32_t state_pin_bits)
{
//	if(AC210_brushless_board) return;
//	if(getParameter (BL_ENABLED) != 1) return;		// MHH:08/02/2026
	if(getParameter (BL_ENABLED) == 1) return;		// MHH:09/02/2026
//	LPC_GPIO[STATE_PORT_NUM].CLR |= 1UL << pin;
// Clear existing settings
	pLPC_STATE_PORT->CLR |= STATE_BIT_MASK;
	pLPC_STATE_PORT->SET |= state_pin_bits;
}
const uint32_t MapStatePins[] =
{
		0,
		STATE_U1_BIT | STATE_L1_BIT,
		STATE_U1_BIT | STATE_L2_BIT,
		STATE_U1_BIT | STATE_L3_BIT,

		STATE_U2_BIT | STATE_L1_BIT,
		STATE_U2_BIT | STATE_L2_BIT,
		STATE_U2_BIT | STATE_L3_BIT,

		STATE_U3_BIT | STATE_L1_BIT,
		STATE_U3_BIT | STATE_L2_BIT,
		STATE_U3_BIT | STATE_L3_BIT,
		0
};
//----------------------------------------------------------------------
void p_ZeroStatePins(void)
{
	STATE_SetPins(0);
}
//----------------------------------------------------------------------
void p_SetStatePins(int test_state)
{
	uint32_t state_pins;
/*
	if(p_ControlPort)
	{
		mh_debug();
	}
*/
	state_pins = MapStatePins[test_state];
	STATE_SetPins(state_pins);
}

