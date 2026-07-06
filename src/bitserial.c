/* ------------------------------------------------------------
Title:          device.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   28/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Implement 2400 baud serial link
                on spare pins
Changes:

------------------------------------------------------------ */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "global.h"
#include "param.h"
#include "bitserial.h"
#include "comms.h"
#include "device.h"

#ifndef AC210_PORT
/* 
   uses port pin 8.3 and timer TB1 for send, pin 8.4 and timer TB2 for receive
   The receive line is set up for high to low edge detect in idle to detect start bit transition. The
   software then sets up a timer for a half bit time to check validity of start.
   The timer period is then set to one bit time and the data sampled on interrupts until
   the stop bits (2) have arrived.
   Transmit procedure operates under timer control, timer interval is set to whole bit time
   and data is processed out the pin.
*/

// Comms is defined as running at 2400 Baud 8 data no parity 2 stop
// Actual baud rate is 2400.24 Baud because of 16MHz clock, so error is .01%

#define HALF_BIT_TIME 3333
#define FULL_BIT_TIME 6666

#define TIMER_MODE 0x00    // timer mode, f1 select
#define TIMER_INT_ENABLE  0x04    // level 4 int

#define TX_TIMER_START  0x40
#define RX_TIMER_START  0x80

#define TXTIMER_CON TB1MR
#define TXTIMER_INT TB1IC
#define TXTIMER_CNT TB1
#define TXTIMER_INT_INDEX 27

#define RXTIMER_CON TB2MR
#define RXTIMER_INT TB2IC
#define RXTIMER_CNT TB2
#define RXTIMER_INT_INDEX 28

// INT2 is used as start edge detect for receive
#define RXSTART_INT INT2IC
#define RXSTART_INT_ENABLE 0x04    // falling edge level 4 priority
#define RXSTART_INT_INDEX 31

#define RX_BIT_MASK 0x10
#define TX_BIT_MASK 0x08

#define BUFFER_SIZE 50

// Transmitter states
typedef enum 
{
    bsTXstart = 0,
    bsTXbits,
    bsTXstop1,
    bsTXstop2,
    bsTXfin
} TxState;

typedef enum
{
    bsRXstart = 0,
    bsRXbits,
    bsRXstop1,
    bsRXstop2
} RxState;

// function prototypes
#pragma INTERRUPT rxInterrupt
void rxInterrupt (void);

#pragma INTERRUPT txInterrupt
void txInterrupt (void);

#pragma INTERRUPT rxStart
void rxStart (void);

TxState txState = bsTXstart;
RxState rxState = bsRXstart;

BYTE txBitCount, rxBitCount;
char txChar, rxChar;

char rxBuffer[BUFFER_SIZE];
char txBuffer[BUFFER_SIZE];
BYTE rxBufferCount, txBufferCount, rxMsgReceived, txBufferWriteIndex;


void setRxTimer (WORD time, BYTE start)
{
    // set up receive timer
    
    // stop it
    TABSR &= ~RX_TIMER_START;

    RXTIMER_CON = TIMER_MODE;
    RXTIMER_CNT = time;
    RXTIMER_INT = TIMER_INT_ENABLE;
    if (start)
    {
        TABSR |= RX_TIMER_START;
    }
}

void setTxTimer (WORD time, BYTE start)
{
    // set up transmit timer
    
    // stop it
    TABSR &= ~TX_TIMER_START;

    TXTIMER_CON = TIMER_MODE;
    TXTIMER_CNT = time;
    TXTIMER_INT = TIMER_INT_ENABLE;
    if (start)
    {
        TABSR |= TX_TIMER_START;
    }
}

void enableRxStartDetect (void)
{
    RXSTART_INT = RXSTART_INT_ENABLE;
}

void disableRxStartDetect (void)
{
    RXSTART_INT = 0;
}

void initBitSerial (void)
{
    ULONG far *vect_ptr;

    // Set up receive line (P8.4)
    rxState = bsRXstart;
    rxBufferCount = 0;
    rxBitCount = 0;
    rxMsgReceived = 0;
    // already an input
    // Set up ISR for timer
    vect_ptr = vector_table + RXTIMER_INT_INDEX;
    *vect_ptr = (ULONG)rxInterrupt;
    // Rx timer dont start
    setRxTimer (HALF_BIT_TIME, 0);

    // Set up ISR  for start detect
    vect_ptr = vector_table + RXSTART_INT_INDEX;
    *vect_ptr = (ULONG)rxStart;
    enableRxStartDetect();

    // set up transmitter (P8.3)
    txState = bsTXstart;
    txBufferCount = 0;
    txBufferWriteIndex = 0;
    txBitCount = 0;
    // make sure pin is high
    P8 |= TX_BIT_MASK;
    // and set to output
    PD8 |= TX_BIT_MASK;
    // Set up ISR for timer
    vect_ptr = vector_table + TXTIMER_INT_INDEX;
    *vect_ptr = (ULONG)txInterrupt;
    // Tx timer dont start
    setTxTimer (FULL_BIT_TIME, 0);
}


void rxStart (void)
{
    // High to low transition on start line
    rxState = bsRXstart;
    // disable start detect
    disableRxStartDetect();
    // enable timer for half bit time to check start
    setRxTimer (HALF_BIT_TIME, 1);
}


void rxInterrupt (void)
{
    // a bit time interrupt has occurred
    BYTE bit;

    bit = P8 & RX_BIT_MASK;


    switch (rxState)
    {
    case bsRXstart:
        if (bit)
        {
            // this is an error, because after the first half bit time
            // the line should still be low
            setRxTimer(HALF_BIT_TIME, 0);
            enableRxStartDetect();
        }
        else
        {
            // sample next line a full bit time later
            setRxTimer(FULL_BIT_TIME, 1);
            rxState = bsRXbits;
            rxBitCount = 0;
            rxChar = 0;
        }
        break;

    case bsRXbits:
        rxChar >>= 1;
        if (bit)
        {
            rxChar |= 0x80;
        }
        else
        {
            rxChar &= 0x7F;
        }
        if (++rxBitCount >= 8)
        {
            rxState = bsRXstop1;
        }
        break;

    case bsRXstop1:
        // must be high
        if (!bit)
        {
            rxState = bsRXstart;
            setRxTimer(HALF_BIT_TIME, 0);
            enableRxStartDetect();
        }
        else
        {
            rxState = bsRXstop2;
        }
        break;

    case bsRXstop2:
        rxState = bsRXstart;
        setRxTimer(HALF_BIT_TIME, 0);
        enableRxStartDetect();
        // must be high
        if (bit)
        {
            // complete character.
            rxBuffer[rxBufferCount] = rxChar;
            if (rxBufferCount < BUFFER_SIZE-1)
            {
                rxBufferCount++;
                
            }
            if (rxChar == 0x0A)
            {
                rxMsgReceived = 1;
                // disable unitl processes
                disableRxStartDetect();
            }
        }
        break;
    }
}

void txInterrupt (void)
{
    // Goes off at bit time intervals
    switch (txState)
    {
    case bsTXstart:
        P8 &= ~TX_BIT_MASK;
        txBitCount = 0;
        txState = bsTXbits;
        break;

    case bsTXbits:
        if (txChar & 0x01)
        {
            P8 |= TX_BIT_MASK;
        }
        else
        {
            P8 &= ~TX_BIT_MASK;
        }
        txChar /= 2;
        if (++txBitCount >= 8)
        {
            txState = bsTXstop1;
        }
        break;

    case bsTXstop1:
        P8 |= TX_BIT_MASK;
        txState = bsTXstop2;
        break;

    case bsTXstop2:
        P8 |= TX_BIT_MASK;
        txState = bsTXfin;
        break;
    
    case bsTXfin:
        txState = bsTXstart;
        // see if there are more chars to tx
        if (++txBufferWriteIndex < txBufferCount)
        {
            txChar = txBuffer[txBufferWriteIndex];
            // and leave ints going
        }
        else
        {
            // shut down ints
            setTxTimer  (FULL_BIT_TIME, 0);
            // and clear tx buffer count to indicate not busy
            txBufferCount = 0;
        }
    }
}

BYTE txBusy (void)
{
    return (txBufferCount != 0);
}

BYTE rxMsgAvailable (void)
{
    return (rxMsgReceived == 1);
}

void readRxMessage (char *buf, BYTE *count)
{
    memcpy (buf, rxBuffer, rxBufferCount);
    *count = rxBufferCount;
    rxBufferCount = 0;
    rxMsgReceived = 0;
}

void writeTxMessage (char *buf, BYTE count)
{
    //txDebug ("Write ");
    //txDebug (buf);

    memcpy (txBuffer, buf, count);
    txBufferCount = count;
    txBufferWriteIndex = 0;
    txChar = txBuffer[0];
    // enable the ints
    txState = bsTXstart;
    setTxTimer (FULL_BIT_TIME, 1);
}

void checkBitSerial(void)
{
    BYTE cnt;

    // check and act on incoming strings
    if (rxMsgReceived)
    {
        // Null terminate, and convert to uppercase
        rxBuffer[rxBufferCount]=0;
        for (cnt=0; cnt< rxBufferCount; cnt++)
        {
            rxBuffer[cnt] = toupper(rxBuffer[cnt]);
        }
        //txDebug ("Read ");
        //txDebug (rxBuffer);

        // Check for an attached device. The first of these will be a slave unit
        if ((rxBufferCount > 3) && (*((WORD *)rxBuffer) == DEVICE_INPUT_HEADER))
        {
            // This is a remote device identifier
            // as it starts with DV
            if (!isSlaveProp())
            {
                // cannot be processed by a slave
                processDeviceID( rxBuffer);
            }
        }

        // on a slave device, Check for master data
        else if ((rxBufferCount > 3) && (*((WORD *)rxBuffer) == DEVICE_OUTPUT_HEADER))
        {
            // This is Data from a Master Prop
            // as it starts with DD
            if (isSlaveProp())
            {
                processDeviceData( rxBuffer);
            }
        }
        // reset counter and flag
        rxMsgReceived = 0;
        rxBufferCount = 0;
        // re-enable receiver
        enableRxStartDetect();
    }
}

#endif
