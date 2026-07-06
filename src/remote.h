/* ------------------------------------------------------------
Title:          remote.h

Copyright:      AirMaster Ltd, July 2002

Date created:   01/07/2002
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for Remote control block

Changes:

------------------------------------------------------------ */

#ifndef _REMOTE_H
#define _REMOTE_H

#if REMOTE_VERSION

void checkRemoteCANcommand(void);

void process_RC_Standard (char *cmd);

#define RC_S_ZONE_NORMAL	1
#define RC_S_ZONE_REVERSE	2

extern WORD RC_S_active;
extern BYTE RC_X_zone;

// MHH: 16/03/2018. Use last action and result of completed state test to see if in reverse or normal zone.

#define RC_X_FINE		1
#define RC_X_COARSE		2
#define RC_X_REVERSE_FINE	3
#define RC_X_REVERSE_COARSE	4
#define RC_X_FEATHER		5
#define RC_X_UNFEATHER		6
#define RC_X_WAIT_FINISH	7

extern BYTE RC_X_last_action;

void Set_Zone(BYTE zone);		// So we can catch changes.
void SIG100_check_zone(void);



BYTE RC_S_mode_is_setspeed(void);	// Called from LEDS.C to see if to set Feather LED to orange
BYTE RC_S_mode_is_active(void);	// Called from LEDS.C to see if to set shift LED display


void mh_debug(void);

typedef enum
{
    REMOTE_LOCKOUT = 0,
    REMOTE_IDLE,
    REMOTE_GROUND_MODE,
    REMOTE_FLIGHT_MODE,
    REMOTE_REVERSE_REVERSE,
	REMOTE_FEATHER_REVERSE,
    REMOTE_LOST_COMMS
} RemoteState;

extern WORD remoteSetSpeed;

typedef enum		// MHH:14/11/2023
{
    FM_IDLE = 0,
	FM_FEATHERING,
	FM_REVERSING,
	FM_UNFEATHERING,
	FM_UNREVERSING,
	FM_BETA,
	FM_REVERSE_BETA,
} FeatherMode;

extern FeatherMode Feather_mode;

void Remote_init_tune(void);
void Remote_end_tune(void);

//initialise remote operation
void initRemoteMode (void);

// are we in remote mode
RemoteState remoteMode (void);

// the control cycle call
void remoteControlCycle (void);

// lockout remote mode
void setRemoteLockout (void);

// clear remote lockout
void clearRemoteLockout (void);

// a remote command has arrived
void processRemoteCommand (char *cmd);
void process_RC_Standard (char *cmd);

WORD remoteControlDirection(void);

BYTE remoteReverseActive(void);

#endif


#endif
