/*
 * ac210_leds.c
 *
 *  Created on: 21/11/2016
 *      Author: Murray
 */

#include "ac210_global.h"
#include "leds.h"

//---------------------------------------------------------------------------
uint8_t Map_leds[] =
{
	AC210_LED_0_PIND,		// 0 = diagnostic led
	AC210_LED_3_PIND,		// 1
	AC210_LED_4_PIND,		// 2
	AC210_LED_5_PIND,		// 3
	AC210_LED_6_PIND,		// 4
	AC210_LED_7_PIND,		// 5
	AC210_LED_8_PIND,		// 6
	AC210_LED_1_PIND,		// 7 This one for Feather_and_Reverse_Feather Green
	AC210_LED_2_PIND,		// 8 This one for Feather_and_Reverse_Feather Red
	AC210_LED_9_PIND,		// 9 This one for reverse_relay
	255,
		// map leds to pins
};
#define MAP_BLANK			0
uint8_t Map_leds_RC[] =	// MHH:19/03/2026
{
	AC210_LED_0_PIND,		// 0 = diagnostic led
	MAP_BLANK,				// 1
	MAP_BLANK,				// 2
	AC210_LED_5_PIND,		// 3, Coarse Green
	AC210_LED_6_PIND,		// 4, Coarse Red
	AC210_LED_7_PIND,		// 5, Fine Green
	AC210_LED_8_PIND,		// 6, Fine Red
	MAP_BLANK,				// 7 This one for Feather_and_Reverse_Feather Green
	MAP_BLANK,				// 8 This one for Feather_and_Reverse_Feather Red
	MAP_BLANK,				// 9 This one for reverse_relay
	255,
		// map leds to pins
};

uint8_t *Map_leds_p;

/* Initializes board LED(s) */

bool AC210_4leds;

void AC210_LED_Init(void)
{
/*
 *  MHH:06/04/2019. First check to see if a 4 LED system by testing to see if LED_1 and LED_2 are pulled up.
 *  If they are then it is a 3 LED system, if not then 4 LED.
 *
 *	Unfortunately, doesn't work!!
 *
 */

/*
	STATIC const uint8_t fr_leds[2] =
	{
		AC210_LED_1_PIND,		// This one for Feather_and_Reverse_Feather Green
		AC210_LED_2_PIND,		// This one for Feather_and_Reverse_Feather Red
	};
	int pullup_count=0;
	for(int j=0;j<2;j++)
	{
		uint8_t pindef = fr_leds[j];
		uint8_t port   = PINPORT(pindef);
		uint8_t pin    = PINMASK(pindef);
		if(Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin))
		{
			pullup_count++;
		}
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_MODE_INACT | IOCON_FUNC0);	// Set mode from pullup
	}
	if(pullup_count == 0)
	{
		AC210_4leds = true;
	}
*/
	if(AC210_remote_control_board)
	{
		Map_leds_p = Map_leds_RC;
	}
	else
	{
		Map_leds_p = Map_leds;
	}

	for(int j=0;;j++)
	{
		uint8_t pindef = Map_leds_p[j];
		if(pindef == MAP_BLANK) continue;
		if(pindef == 255)
		{
			break;
		}
/*
		if((AC210_4leds == false) && (pindef == AC210_LED_1_PIND))
		{
			break;
		}
*/
		uint8_t port   = PINPORT(pindef);
		uint8_t pin    = PINMASK(pindef);
		Chip_IOCON_PinMuxSet(LPC_IOCON,port,pin,IOCON_MODE_INACT| IOCON_FUNC0);		// MHH:23/12/2025
		Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin, true);
	}
}
//----------------------------------------------------------------------------------------
/* Sets the state of a board LED to on or off */
#ifdef MH_XXX	// MHH:08/04/2026
bool AC210_relay_on;
void AC210_RELAY_Set(bool On)
{
	uint8_t pindef = Map_leds_p[LED_RELAY];
	if(pindef == MAP_BLANK) return;
	uint8_t port   = PINPORT(pindef);
	uint8_t pin    = PINMASK(pindef);
	Chip_GPIO_WritePortBit(LPC_GPIO, port, pin, On);
	AC210_relay_on = On;
	if(On)
	{
		LED_overlay_flags |= LED_OVERLAY_FLAG_REVERSE_RELAY_ON;
	}
	else
	{
		LED_overlay_flags &= ~LED_OVERLAY_FLAG_REVERSE_RELAY_ON;
	}
}
#endif
//----------------------------------------------------------------------------------------
void Board_LED_Set(uint8_t LEDNumber, bool On)
{
	if(LEDNumber >= LED_RELAY)	// Turn On/off separately to avoid confusion.
	{
		return;
	}
	uint8_t pindef = Map_leds_p[LEDNumber];
	if(pindef == MAP_BLANK) return;
	uint8_t port   = PINPORT(pindef);
	uint8_t pin    = PINMASK(pindef);
	Chip_GPIO_WritePortBit(LPC_GPIO, port, pin, On);
}

/* Returns the current state of a board LED */
bool Board_LED_Test(uint8_t LEDNumber)
{
	uint8_t pindef = Map_leds_p[LEDNumber];
	if(pindef == MAP_BLANK) return false;

	uint8_t port   = PINPORT(pindef);
	uint8_t pin    = PINMASK(pindef);
	return Chip_GPIO_ReadPortBit(LPC_GPIO, port, pin);
}

void Board_LED_Toggle(uint8_t LEDNumber)
{
	Board_LED_Set(LEDNumber, !Board_LED_Test(LEDNumber));
}
//----------------------------------------------------------------------
// Expect least significant bit as diagnostoc LED bit,
// the rest in the correct order (Feather Green, Feather Red, etc) so
// we just test each bit and set the mapped LED if on, otherwise turn off.
void p_setLEDS(WORD state)
{
	bool on;
	uint8_t led;

	WORD bit=1;

	for(led=0;led<LED_RELAY;led++) {
		on = ((state & bit) != 0);
		bit <<= 1;
		Board_LED_Set(led, on);
	}
}

