/*
 * ac210_digital.c
 *
 *  Created on: 21/11/2016
 *      Author: Murray
 */

#include "ac210_global.h"
#ifdef MH_YYY		// MHH:05/01/2026
STATIC const uint8_t map_switches[] =
{
	0,
	AC210_SWITCH_1_PIND,
	AC210_SWITCH_2_PIND,
	AC210_SWITCH_3_PIND,
	AC210_SWITCH_4_PIND,
	AC210_SWITCH_5_PIND,
	AC210_SWITCH_6_PIND,
	AC210_SWITCH_7_PIND,
// map switches to pins
};

//----------------------------------------------------------------------------------------------------------------------------
/*
MHH:30/01/2019 Test for integrated board. Currently pins 0.20 (SW4) and 0.19 (SW2) are grounded, all rest should be pulled up.
May want to change pins later. Hold pin is 0.17 (AC210_SWITCH_3_PIND)
*/

#ifdef AC210_INTEGRATED
bool AC210_integrated;
void AC210_Switch_Test_Integrated(void)
{
	uint8_t bit = 2;		// as we are starting with j=1, not zero
	uint8_t switch_bits = 0;

	for(int j=1;j<=6;j++)
	{
		uint8_t pindef = map_switches[j];
		uint8_t port   = PINPORT(pindef);
		uint8_t pin    = PINMASK(pindef);

		if(Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin) == false)		// invert
		{
			switch_bits |= bit;
		}
		bit += bit;		// shift bit 1 place
	}

	uint8_t integrated_bits = (1<<2) + (1<<4);		// SW2 + SW4
	if(switch_bits == integrated_bits)
	{
		AC210_integrated = true;
	}
}
#endif
//---------------------------------------------------------------------------------------
/* Initializes board switches */
void AC210_Switch_Init(void)
{
//	for(int j=1;j<=6;j++)
	for(int j=1;j<=7;j++)		// MHH:30/01/2019
	{
		uint8_t pindef = map_switches[j];
		uint8_t port   = PINPORT(pindef);
		uint8_t pin    = PINMASK(pindef);

		Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin, false);
	}
}
//---------------------------------------------------------------------------
void Board_ShowSwitches(void)
{
	bool state;
	PRINTF("\r\nBoard_Switches: ");
	for(int j=1;j<=7;j++)
	{
		uint8_t pindef = map_switches[j];
		uint8_t port   = PINPORT(pindef);
		uint8_t pin    = PINMASK(pindef);
		state = Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin);
		PRINTF("%4d",state);
	}
	PRINTF("\r\n");
}
#endif
//---------------------------------------------------------------------------
#ifdef MH_OLD_PORT
typedef struct {
	uint8_t pingrp:3;		/* Pin group */
	uint8_t pinnum:5;		/* Pin number */
} PINMUX_GRP_SWITCHES_T;
STATIC const PINMUX_GRP_SWITCHES_T Switches_map2[] =
{
		{AC210_SWITCH_PORT,  AC210_SWITCH_7_PIN},		/* Manual */
		{AC210_SWITCH_PORT,  AC210_SWITCH_2_PIN},		/* Rotary Feather */
		{AC210_SWITCH_PORT,  AC210_SWITCH_3_PIN},		/* Rotary Hold */
		{AC210_SWITCH_PORT,  AC210_SWITCH_4_PIN},		/* Rotary Cruise */
		{AC210_SWITCH_PORT,  AC210_SWITCH_5_PIN},		/* Rotary Climb */
		{AC210_SWITCH_PORT,  AC210_SWITCH_6_PIN},		/* Rotary Take-off */
		{AC210_SWITCH_PORT,  AC210_SWITCH_1_PIN},		/* Feather Switch */
		{AC210_J2_DIG_PORT,  AC210_J2_15_COARSE_SENSE_PIN},	/* J2_15 Coarse sense */
		{AC210_J2_DIG_PORT,  AC210_J2_16_FINE_SENSE_PIN},	/* J2_16 Fine sense */
};
#else
STATIC const uint8_t Switches_map2[] =
{
		AC210_SWITCH_7_PIND,		/* 0.Manual */
		AC210_SWITCH_2_PIND,		/* 1.Rotary Feather */
		AC210_SWITCH_3_PIND,		/* 2.Rotary Hold */
		AC210_SWITCH_4_PIND,		/* 3.Rotary Cruise */
		AC210_SWITCH_5_PIND,		/* 4.Rotary Climb */
		AC210_SWITCH_6_PIND,		/* 5.Rotary Take-off */
		AC210_SWITCH_1_PIND,		/* 6.Feather Switch */
		AC210_J2_15_COARSE_SENSE_PIND,	/* 7. J2_15 Coarse sense */
		AC210_J2_16_FINE_SENSE_PIND,	/* 8. J2_16 Fine sense */
};
#endif
#define AC210_SWITCH_MAP2_SIZE		9
//static WORD saveSwitches;
#ifdef MH_LOCAL_DEBOUNCE
static uint16_t last_ac200_switches;
static uint16_t AC200_switches;
#endif
//uint16_t MH_switches;
//----------------------------------------------------------------------------
//#define MH_DEBUG_SWITCHES
#ifdef MH_DEBUG_SWITCHES
uint8_t MH_debug_switches=0;	// MHH:1/10/2022. See if FINE and COARSE manual working for brushless
uint16_t saveSwitches;
void mhbreak(void)
{

}
#endif
WORD GetAC200_Switches(void)
{
	uint8_t pin;
	uint8_t port;
	uint16_t ac200_switches;

	bool state;
	uint8_t pindef;
//	PINMUX_GRP_SWITCHES_T pdata;

#ifdef AC210_INTEGRATED
	if(AC210_integrated)
	{
		ac200_switches = (1 << 2);		// Hold switch
		return ac200_switches;
	}
#endif

	ac200_switches = 0;
	for(int i=0;i<AC210_SWITCH_MAP2_SIZE;i++)
	{
//		pdata = Switches_map2[i];
		pindef = Switches_map2[i];
		port   = PINPORT(pindef);
		pin    = PINMASK(pindef);
//		pin = pdata.pinnum;
//		port = pdata.pingrp;
		state = Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin);
		if(state == false)	// Invert bit value
		{
			ac200_switches |= (1 << i);
		}
	}
//#define MH_TEST_SLIPRING
#ifdef MH_TEST_SLIPRING
#define FINE_BIT		(1<<8)
#define COARSE_BIT		(1<<7)
#define FINE_COARSE_BITS (FINE_BIT | COARSE_BIT)
		if(ac200_switches & FINE_COARSE_BITS)
		{
			if((ac200_switches &FINE_COARSE_BITS) == FINE_COARSE_BITS)	// MHH:19/12/2020, testing slipring test logic
			{
				ac200_switches &= (~FINE_COARSE_BITS);		// if both FINE and COARSE sense switches on, then mask out
			}
#ifdef MH_SLIPRING_STATE_LOGIC
			if(ac200_switches & COARSE_BIT)
			{
				if(ADC_testing_slipring_idle() == false)
				{
					ac200_switches &= (~COARSE_BIT);	// May be better doing with debounce count. Think that is how status check is ignored
				}
			}
#endif
		}

#endif

#ifdef MH_LOCAL_DEBOUNCE	// This debounce replaced by AC200_debounce_input() in digital.c. 31/01/2018
	if(ac200_switches == last_ac200_switches)		// Only update switch value if it repeats over 2 cycles. 25/01/2018
	{
		AC200_switches = ac200_switches;
	}
	last_ac200_switches = ac200_switches;
#endif

// Note: Could put this under the control of a global debug variable.
//#define MH_DEBUG_SWITCHES
#ifdef MH_DEBUG_SWITCHES
	static uint16_t saveSwitches;
//	if(MH_debug_switches)
	{
		if(ac200_switches != saveSwitches)
		{
			saveSwitches = ac200_switches;
			PRINTF("New switches: %04x::",ac200_switches);
			for(int i=0;i<9;i++)
			{
				if((1<<i) & ac200_switches)
				{
					PRINTF("[%d:1]",i);
				}
			}
			PRINTF("\r\n");
		}
	}

#endif
//#define MH_DEBUG_SWITCHES2
#ifdef MH_DEBUG_SWITCHES2
#define SWITCH_MANUAL			(1<<0)
#define SWITCH_FEATHER			(1<<6)
#define SWITCH_COARSE_SENSE		(1<<7)
#define SWITCH_FINE_SENSE		(1<<8)

	if(ac200_switches & SWITCH_COARSE_SENSE)
	{
		mhbreak();	// Dummy for breakpoint;
	}
#endif




	return ac200_switches;
}
//---------------------------------------------------------------------------
#define MANUAL_SWITCH_BIT	1
#define MANUAL_KEY_MASK 0x01C0		// 0x01C0 is 0b0001 1100 0000

WORD p_GetAC200_Switches(bool maskmanual)
{
	uint16_t ac200_switches;
	ac200_switches = GetAC200_Switches();
	if(maskmanual)	// This option gets rid of all rotary switch bits
	{
		if(ac200_switches & MANUAL_SWITCH_BIT)
		{
	// See comments above
			ac200_switches &= (MANUAL_KEY_MASK | MANUAL_SWITCH_BIT);		// lifted from digital.c
		}
	}
	return ac200_switches;
}

