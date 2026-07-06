/* ------------------------------------------------------------
Title:          device.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   28/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Monitors attached devices
Changes:

------------------------------------------------------------ */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "global.h"
#include "param.h"
#include "device.h"
#include "comms.h"
#include "control.h"
#include "bitserial.h"
#include "digital.h"

#ifndef AC210_PORT

/*  
    Devices which are attached to this unit will repeatedly transmit 
    an identifier string. This must be sent at least 3 times per second
    and if not received for one second the device will be assumed to 
    be disconnected.
    What the devices expect back is dependent on the device, for
    instance a slave unit will expect to receive RPM set point and actual
    figures which are used for control.
    
*/

#define DEVICE_IDENTIFIER_SLAVE    0x31
#define DEVICE_IDENTIFIER_DISPLAY  0x32


volatile BYTE deviceConnected;
volatile BYTE retrigger;
volatile BYTE slaveCommsActive = 0;

// slave prop data
long slaveRPMSet, slaveRPMActual;


void clearSlavePropData (void)
{
    slaveRPMSet = 0;
    slaveRPMActual = 0;
    slaveCommsActive = 0;
}

void initAttachedDevices(void)
{
    deviceConnected = 0;
    clearSlavePropData();
}

BYTE deviceAttached (BYTE mask)
{
    return ((deviceConnected & mask) != 0);
}

void processDeviceID (char *msg)
{
    // the incoming string will be DVx, where x is the device identifier as an ascii integer
    // ie DV1 indicates a slave unit
    // we know that the string starts with DV already
    char *wrk = msg;
    wrk++;
    wrk++;
    switch (*wrk)
    {
    case DEVICE_IDENTIFIER_SLAVE:
        deviceConnected = SLAVE_UNIT;
        retrigger = SLAVE_UNIT;
        break;

    case DEVICE_IDENTIFIER_DISPLAY:
        deviceConnected = DISPLAY_UNIT;
        retrigger = DISPLAY_UNIT;
        break;

    default:
        deviceConnected = 0;
        retrigger = 0;
        break;
    }
}

void processSlaveData (char *msg)
{
    // msg points to the identifier.
    // Format of data is M,SSSSs,AAAAa,SSSSs,AAAAa where M = mode, SSSSs is set speed, AAAAa is actual
    // speed (1 implied dp). Data sent twice for error check
    long ss1,ss2;
    long as1,as2;
    OpMode  cm;
    char far *comma;
    char far *wrk = (char far *)&msg[1];

   // txDebug (msg);

    ss1 = ss2 = as1 = as2 = 0;
    cm = MANUAL;
    comma = strchr(wrk, ',');
    if (comma)
    {
        *comma = 0;
        cm = (OpMode)atoi(wrk);
        wrk = ++comma;
        comma = strchr (wrk, ',');
        if (comma)
        {
            *comma = 0;
            ss1 = atol (wrk);
            wrk = ++comma;
            comma = strchr (wrk,',');
            if (comma)
            {
                *comma = 0;
                as1 = atol (wrk);
                wrk = ++comma;
                comma = strchr (wrk, ',');
                if (comma)
                {
                    *comma = 0;
                    ss2 = atol (wrk);
                    wrk = ++comma;
                    as2 = atol(wrk);
                }
            }
        }
    }
    // To be updated, the figures must be the same, the
    // mode must be takeoff, climb, cruise or hold, and
    // the setspeed must be non-zero
    if ((ss1 == ss2) && (as1 == as2) && (ss1 != 0))
    {
        if ((cm==TAKEOFF) ||
            (cm==CLIMB) ||
            (cm==CRUISE) ||
            (cm==HOLD))
        {
            slaveRPMSet = ss1;
            slaveRPMActual = as1;
        }
    }
}

void processDisplayData (char *msg)
{
    // TBD
}

void processDeviceData (char *msg)
{
    // This is run on the attached device, not the master
    // Incoming string is in format DDx........, where x is as above
    char *wrk = msg;
    wrk++;
    wrk++;
    switch (*wrk)
    {
    case DEVICE_IDENTIFIER_SLAVE:
        retrigger = 1;
        slaveCommsActive = 1;
        processSlaveData (wrk);
        break;

    case DEVICE_IDENTIFIER_DISPLAY:
        retrigger = 1;
        processDisplayData (wrk);
        break;

    default:
        break;
    }
}

// called once per second on the master unit to reset the attached device if required
void deviceMasterTick (void)
{
    if (!retrigger)
    {
        deviceConnected = 0;
    }
    retrigger = 0;
}

// called once per second on the slave device to reset data if required
void deviceSlaveTick (void)
{
    if (!retrigger)
    {
        if (isSlaveProp())
        {
            // For now just retain last received data
            //clearSlavePropData();
            // but retain a flag
            slaveCommsActive = 0;
        }
    }
    retrigger = 0;
}

// For slave unit, control loop calls this in HOLD mode
BYTE getSlaveRPMSettings (long *setRPM)
{
    // The slave set speed is the master actual speed
    *setRPM = slaveRPMSet;
    return (slaveCommsActive != 0);
}

BYTE isSlaveCommsActive (void)
{
//    char buffer[10];

//    sprintf (buffer, "CA=%d\r\n", slaveCommsActive);
//    txDebug (buffer);
    return (slaveCommsActive != 0);
}

// Master with slave attached calls this 3 x per second
void sendSlaveData (void)
{
    // Called by the Master 3 times per second, sends the required data
    // out to the slave
    char buffer[30];
    OpMode cm; 
    long ss, as;

    cm = operatingMode();
    ss = currentSetSpeed();
    as = currentActualSpeed();

    sprintf (buffer, "DD%c%d,%ld,%ld,%ld,%ld\r\n",
                     DEVICE_IDENTIFIER_SLAVE,
                     cm,ss,as,ss,as);
    writeTxMessage (buffer, strlen(buffer));
}

// Slave device calls this 3 x per second
void sendSlaveIdent (void)
{
    char buffer[10];
    sprintf (buffer, "DV%c\r\n", DEVICE_IDENTIFIER_SLAVE);
    writeTxMessage (buffer, strlen(buffer));
}
#endif
