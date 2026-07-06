/* ------------------------------------------------------------
Title:          device.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for attached devices
Changes:

------------------------------------------------------------ */
#ifndef _DEVICE_H
#define _DEVICE_H

/*
The beginnings of attached devices such as slave units, debugging terminals
display units etc
*/
#ifdef MH_PETER_DEVICE_LOGIC		// MHH:23/11/2020

#define DEVICE_INPUT_HEADER  0x5644           // "DV"
#define DEVICE_OUTPUT_HEADER 0x4444           // "DD"

#define SLAVE_UNIT    0x01
#define DISPLAY_UNIT  0x02


void initAttachedDevices(void);

BYTE deviceAttached (BYTE mask);

// comms processing routines
void processDeviceID (char *msg);
void processDeviceData (char *msg);

// To process timeouts
void deviceMasterTick (void);
void deviceSlaveTick (void);
#endif

// For slave unit operation

// Master device calls
//BYTE getSlaveRPMSettings (long *setRPM);
WORD getSlaveRPMSettings (void);	// MHH:23/11/2020
WORD getSlaveDeadband(void);		// MHH:30/11/2020
//void sendSlaveData (void);

// Slave device calls
//void sendSlaveIdent (void);
BYTE isSlaveCommsActive (void);

#endif
