/* ------------------------------------------------------------
Title:          analog.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   29/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Psuedo private A/D converter stuff

Changes:

------------------------------------------------------------ */
#ifndef _ANALOG_H
#define _ANALOG_H

// Motor State Return Value
#define STATE_OPEN  0
#define STATE_OK    1
#define STATE_ERROR 2

typedef enum
{
    A_DIMMER = 0,
    A_SUPPLY,
    A_MOTOR_CURRENT,
    A_MOTOR_STATE,
//    A_MANIFOLD_PRESSURE
    A_SLIDER_CONTROL,		// MHH:02/04/2024
	A_TEMPERATURE,		// MHH:04/02/2026
	A_MAX,
} AnalogChannel;

// Initialisation
void initAnalog (void);

// Accessor routines
int scaledValue (AnalogChannel channel);

WORD rawValue (AnalogChannel channel);
extern WORD rawValues[A_MAX];
BYTE betaSwitchOn (void);

int LMT87_convert_millivolts_to_celcius(int millivolts);

#endif

