/* ------------------------------------------------------------
Title:          Feather.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Feathering control

Changes:

------------------------------------------------------------ */

#ifndef _FEATHER_H
#define _FEATHER_H

/*
Propellor Feathering control
*/

typedef enum 
{
    F_IDLE = 0,
    F_ACTIVE,
    F_COMPLETE,
    F_REVERSING,
    F_BETA_IDLE,
    F_BETA,
    F_BETA_EXIT
} FeatherState;

extern FeatherState fState;

WORD Get_param_Feather_mode(void);
void Write_param_Feather_mode(WORD fmode);

// initialisation
void initFeather (void);

// repeated call
void featherControl (void);

// state accessor
FeatherState featherState (void);

// Overcurrent shutdown
void shutdownFeather (void);

// yes start a reverse
void doReverseFeather (void);

// in unfeather cycle in the 1 second pause
BYTE waitingForStateCheck(void);


// get into beta mode
void enterBetaMode (void);

// get out of beta mode
void exitBetaMode(void);

BYTE Reverse_RPM_too_high(void);

BYTE In_Reverse_Zone(void);

void AC210_check_start_reverse(void);
void AC210_check_reverse_relay(void);
void AC210_reverse_relay_off(void);
//extern bool Feather_and_Reverse_Prop;
extern BYTE Reverse_position;
extern bool Reverse_relay_on;

#endif

