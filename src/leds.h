/* ------------------------------------------------------------
Title:          leds.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Local header for LED control

Changes:

------------------------------------------------------------ */
#ifndef _LEDS_H
#define _LEDS_H

/*
State Display functions
*/

// LED control bytes
#define LED_OFF        0x00
#define LED_ON         0x01
#define LED_FLASH_1HZ  0x02
#define LED_FLASH_2HZ  0x04
#define LED_FLASH_5HZ  0x08
#define LED_FLASH_1HZ_10 0x10
#define LED_FLASH_2HZ_25 0x20
#define LED_FLASH_5HZ_25 0x40
#define LED_FLASH_P2_HZ 0x80		// 0.2 Hz

typedef enum
{
    LED_DIAG = 0,
    LED_FEATHER3_G,
    LED_FEATHER3_R,
    LED_COARSE_G,
    LED_COARSE_R,
    LED_FINE_G,
    LED_FINE_R,
#ifdef AC210_4LEDS
	LED_FEATHER4_G,
	LED_FEATHER4_R,
#endif
//	LED_BACKLIGHT,
	LED_RELAY,
    LED_ARRAY_INDEX
} LedIndex;

extern BYTE ledState[LED_ARRAY_INDEX];

extern uint8_t LED_FEATHER_R_ix;
extern uint8_t LED_FEATHER_G_ix;

extern BYTE LED_overlay_flags;		// MHH: 28/02/2018
#define LED_OVERLAY_FLAG_REMOTE_ACTIVE		1
#define LED_OVERLAY_FLAG_REVERSE_ZONE		2
#define LED_OVERLAY_FLAG_AUTOGYRO_ACTIVE	4
#define LED_OVERLAY_FLAG_AUTOGYRO_LATCHED	8
#ifdef MH_XXX
#define LED_OVERLAY_FLAG_REVERSE_RELAY_ON	16
#define LED_OVERLAY_FLAG_REVERSE_BUTTON_ON	32
#else
#define LED_OVERLAY_FLAG_BETA_MAX_RPM	16
#endif
#define LED_OVERLAY_FLAG_BL_POS_ERROR		64	// MHH:20/07/2023

extern BYTE LED_error;		// MHH: 16/03/2018
#define LED_ERROR_REVERSE	1
#define LED_ERROR_STOP_POS_OK	2
#define LED_ERROR_STOP_POS_SET	3

// Initialisation function
void initLEDS (void);

// update state display
void updateLEDS (void);

// Diagnostic LED control
void diagnosticDisplay ( BYTE state );

// Backlight Control
void setBackLight (BYTE on);

void setLEDS (WORD state);


#endif
