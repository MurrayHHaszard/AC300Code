/* ------------------------------------------------------------
Title:          comms.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for Comms handler code

Changes:

------------------------------------------------------------ */
#ifndef _COMMS_H
#define _COMMS_H
#include "global.h"

// initialisation
void initComms (void);
void Set_Prop_RPM(WORD mag_rpm);

// check for complete message in
// called every main loop
void checkForCommand (void);
void checkForCommand2 (void);

void AC210_master_check(void);
void AC210_slave_check(void);
bool Is_master_sending(void);
bool Is_slave_sending(void);

void AC210_display_serial_ringbuffer_data(int val);	// MHH:12/08/2023

#define ATX_RETURN	255		// BYTE type does not allow -1
#define ATX_OK		0
#define ATX_ERROR	1

#define AUTO_THROTTLE_UNUSED	65535
#define AUTO_THROTTLE_TIMEOUT	65534

#define TC_PORT_NONE		0
#define TC_PORT_CAN			1
#define TC_PORT_SERIAL		2

extern uint8_t AC210_remote_control_port;	// MHH:21/11/2023
#endif
