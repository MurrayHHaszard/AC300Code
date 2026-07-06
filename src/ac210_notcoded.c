/*
 * port_notcoded.c
 *
 *  Created on: 15/05/2015
 *      Author: Murray
 */
//#include "port_mhac200.h"
#include "ac210_global.h"

#include "global.h"
#include "analog.h"
#include "comms.h"
#include "digital.h"
#include "leds.h"
#include "param.h"
#include "rpm.h"
#include "sstate.h"
#include "drive.h"
#include "control.h"
#include "feather.h"
#include "manifold.h"
#include "device.h"
#include "bitserial.h"

void initVectorTable (void){};
void enableInterrupt(void){}
void disableInterrupt(void){}


#ifdef MH_PETER_DEVICE_LOGIC		// MHH:23/11/2020
void initBitSerial (void){};
void checkBitSerial(void){};

void initAttachedDevices(void){};
BYTE deviceAttached (BYTE mask){return 0;}
void sendSlaveData (void){};
void deviceMasterTick (void){};
void deviceSlaveTick (void){};
void sendSlaveIdent (void){};
//---------------------------------------------------------
void processDeviceID (char *msg)
{
	Abort(AC210_SRC_NOTCODED+10,"processDeviceID:Not coded yet");
}
//---------------------------------------------------------
void processDeviceData (char *msg)
{
	Abort(AC210_SRC_NOTCODED+20,"processDeviceData:Not coded yet");
}
#endif
//================================================================================================================
void p_Set_AT_Speed(void)
{
	Abort(AC210_SRC_NOTCODED+30,"p_Set_AT_Speed:Not coded yet");
}
