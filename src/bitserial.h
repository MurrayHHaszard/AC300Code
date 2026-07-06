/* ------------------------------------------------------------
Title:          device.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for software serial port
Changes:

------------------------------------------------------------ */
#ifndef _BITSERIAL_H
#define _BITSERIAL_H

void initBitSerial (void);

BYTE txBusy (void);

BYTE rxMsgAvailable (void);

void readRxMessage (char *buf, BYTE *count);

void writeTxMessage (char *buf, BYTE count);

void checkBitSerial(void);

#endif
