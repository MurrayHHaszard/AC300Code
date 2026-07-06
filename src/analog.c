/* ------------------------------------------------------------
Title:          analog.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   29/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        A/D conversion and accessor routines

Changes:

------------------------------------------------------------ */

#include <string.h>
#include <stdio.h>

#include "global.h"
#include "analog.h"
#include "param.h"

/*
    analog conversions are performed continuously, after a complete
    scan of the selected channels, an ISR is used to process the
    results into the set of variables used by the accessor
    routines. The raw values are filtered to reduce abnormalities
*/

const char channelName[5][4] = {
    "Dim",
    "12V",
    "M_C",
    "M_S",
    "Map"
};

#define AD_INT_NO 14
#define AD_INT_PRIORITY 1

// filter is in % (80 = 0.8)
#define FILTER_VALUE 80L
#define MCMP_FILTER_VALUE 90L


// data storage
WORD rawValues[A_MAX];
WORD tempMotorCurrent;
BYTE mcCount;
//WORD tempManifoldPressure;
//BYTE mpCount;
extern int ADC_raw_current;

#ifndef AC210_PORT
// Local routines
#pragma INTERRUPT adInt
void adInt (void);
#endif


void initAnalog (void)
{
#ifndef AC210_PORT
	ULONG *vect_ptr;
#endif
	BYTE cnt;

    for (cnt=0; cnt < 5; cnt++)
    {
        rawValues[cnt] = 0;
    }
    mcCount = 0;
    tempMotorCurrent = 0;

//    mpCount = 0;
//    tempManifoldPressure = 0;


#ifndef AC210_PORT

    // initialise and start A/D conversions off

    ADCON2 = 0x01; // Use sample & hold
    // we operate in single sweep mode
    // covering channel 0 thru 5 (5 result ignored)
    ADCON0 = 0x90; // f/2 single sweep mode
    ADCON1 = 0x2A; // VREF , f/2, AN0-AN5

    // chain & enable interrupts

    vect_ptr = vector_table + AD_INT_NO;
    *vect_ptr = (ULONG)adInt;
    ADIC = AD_INT_PRIORITY;

    // now start conversions
    ADCON0 |= 0x40;
#endif
}

// scales  the values to something useful..
// 1 count here represents 4.9 milliVolts assuming a 5 Volt rail
// this depends on experimentation with the actual board

// Voltage is returned to 1dp (ie 123 == 12.3Volts)

// Current is returned as milliAmps

// Motor State sense is returned as one of a specific set
// of values.. 0 (STATE_OPEN) is a "zero" reading idicating open circuit
//             1 (STATE_OK)   is an expected return ie circuit OK
//             2 (STATE_ERROR) means that manual drive was enabled during
//                             the reading, which resulted in an overrange

WORD Analog_raw;
#ifdef AC210_PORT
//#define AC210_24V_FACTOR		(1.0)

WORD AC210_convert_voltage(WORD raw)
{
	float temp;
	float fraw = raw;
	WORD val;
	if(AC210_hardware_version >= 'F')	// 24V capable
	{
		temp = fraw;
		if(temp > 2400.0F)	// MHH:16/10/2025
    	{
    		temp *= 0.8535F;
    	}
    	else
    	{
    		temp *= 0.859F;
    	}
	}
	else
	{
//    	temp = fraw * 0.455;		// R71 = 4.7k: (5.7 * 3.2 / 4096) [theory]
    	temp = fraw * 0.5F;		// R71 = 4.7k: (5.7 * 3.2 / 4096) [theory]
	}
	temp += 0.5F;
	val = (WORD)temp;
    val = (val+5)/10;
	return val;
}
#endif

int scaledValue (AnalogChannel channel)
{
    // Return the scaled values with debug..
    char buf[40];
    int val,raw;
    double temp;
    int  cv;
    int v2;
    int temp_adc,millivolts,temp_celcius;
    // get the filtered raw value
    val = rawValues[channel];
    raw = val;
	Analog_raw = val;
    // scale it
    switch (channel)
    {
    case A_DIMMER:
#ifdef AC210_PORT		// MHH:06/03/2018. Same as for Supply with Dimmer as voltage split is same
    	val = AC210_convert_voltage(raw);
#else
        temp = (double)val * 0.0137 * 100;
        val = (WORD)temp;
        // retain 1dp
        val = (val+5)/10;
#endif
        // debug
        if (debug & DEBUG_DIMMER)
        {
            sprintf (buf, "%s -> %d.%d (%u)", &channelName[channel][0],
                                              val/10, val%10, raw);
            txDebug(buf);
        }
        break;

    case A_SUPPLY:
        // a voltage input scaled through 2K7 / 1K divider
        // voltage is approx 0.270 * actual, so scaling factor is
        // (count * .0049) / 0.270 [count * 0.0181]
#ifdef AC210_PORT
    	val = AC210_convert_voltage(raw);
// Note: Changed R71 from 3K to 4.7K from 5A to 5B, but hand solder new R71 = 4.7K to all 5A
//    	temp = (double)val * 0.47;		// [measuring]
//    	if(hwf.version == AC210_PCB_VERSION_5A)
#else
    	temp = (double)val * 0.0181 * 100;
    	val = (WORD)temp;
        // retain 1dp
        val = (val+5)/10;
#endif
        // debug
        if (debug & DEBUG_ANALOG_VOLTAGE)
        {
            sprintf (buf, "%s -> %d.%d (%u)", &channelName[channel][0],
                                              val/10, val%10, raw);
            txDebug(buf);
        }
        break;

    case A_MOTOR_CURRENT:
        // Voltage developed across 4 0.39 Ohm resistors in parallel
        // Through filters and amplifiers. Gain is intialised to 12.50  (1250)
        // Current is in milliAmps,
        // Count is stored to 1 dp for current, as values are small and filtering
        // does not work well otherwise.

#ifdef AC210_PORT
        cv = (int)val;		// MHH:24/11/2018. Offset adjustment now done in ac210_adc.c
#else
        cv = (int)val + (getCurrentOffset() * 10);
#endif
        if (cv < 0)
        {
            cv = 0;
        }
        temp = ((double)cv * (double)getCurrentGain()) / 100.0;
        val = (WORD)temp;
        // retain 0dp
        val = (val+5)/10;
//        if(val < 20) val = 0;	// MHH:02/11/2017. Less than 20 ma considered zero as Maxon draws 50 ma with no load

//		if(val < 15) val = 0;		// MHH:07/11/2017. Getting a value of 1 for current in AC200Tester sometimes when it should be zero.
//		if(val < 20) val = 0;		// MHH:09/11/2017.???

        // debug
        if (debug & DEBUG_MOTOR_CURRENT)
        {
//        	if(timerTick%25 == 0)
        	{
//            	PRINTF("ADC_raw_current=%d\r\n",ADC_raw_current);	// MHH:06/11/2018
            	sprintf (buf, "%s -> %u (%u)\r\n", &channelName[channel][0],
                         val, raw );
                txDebug(buf);
        	}
        }
        break;

    case A_MOTOR_STATE:
        // Voltage developed across 100 ohm resistor.
        // results in either 0, 1.3, or 2.6 volts at
        // Analog input for version 1 pcb, and 1.9 to 2.8 volts for an OK read
        // for version 2 PCB's.

        val /= 100;
        if (pcbVersion() == PCB_VER_1)
        {
            if ((val > 0) && (val < 5))
            {
                val = STATE_OK;
            }
            else if (val >= 5)
            {
                val = STATE_ERROR;
            }
        }
        else
        {   // Version 2 & 3 type (rectangular) PCB's
            if (val > 1)
            {
                val = STATE_OK;
            }
            else
            {
                val = STATE_OPEN;
            }
        }
        // debug
        if (debug & DEBUG_MOTOR_STATE)
        {
            sprintf (buf, "%s -> %u (%u)\r\n", &channelName[channel][0],
                     val, raw );
            txDebug(buf);
        }
        break;
#ifdef MH_MANIFOLD_PRESSURE		// MHH:02/04/2024
        case A_MANIFOLD_PRESSURE:
        #ifdef AC210_PORT
            	Abort(SRC_ANALOG+10,"scaledValue:Reference to A_MANIFOLD_PRESSURE.");
        #else
                // inches of mercury (to 1 dp) = count * 1.2096
                temp = (double)val * 12.096;
                val = (WORD)temp;
                // retain 1dp
                val = (val+5)/10;
                // debug
                if (debug & DEBUG_MANIFOLD_PRESS)
                {
                    sprintf (buf, "%s -> %u (%u)\r\n", &channelName[channel][0],
                             val, raw );
                    txDebug(buf);
                }
        #endif
#else
        case A_SLIDER_CONTROL:
#ifdef MH_XXX
        	v2 = val;
        	v2 *= 1000;		// Convert to percentage
        	v2 = (v2 + 2048) / 4096;	// Could shift
//        	if(v2 > 993) v2 = 1000;
        	val = (WORD) v2;
#else
/* MHH:05/04/2024. Assume val in range 0 to 4095.
   We want a range of between 0 and 1000.
   First divide by 4 to get in range 0 to 1023
   Then, adjust to range of 0 to 1000 by saying zero if 0 to 12, and 1000 if 1001 to 1023
*/
        	v2 = (val >> 2);		// Divide by 4
#ifdef MH_XXX
        	v2 -= 12;
        	if(v2 < 0) v2 = 0;
        	if(v2 > 1000) v2 = 1000;
#endif
        	val = (WORD)v2;
#endif
#endif
        break;


        case A_TEMPERATURE:
        	temp_adc = val;
        		//  We know lpc845 is 3.3V, so convert millivolts = 3300 * adc_v / 4096

        	millivolts = (3300 * temp_adc) >> 12;

        	temp_celcius = LMT87_convert_millivolts_to_celcius(millivolts);	// Tricky, uses formula
        	val = temp_celcius;
        	break;

        default:		// For compiler
        	break;

    }

    // and return it
    return (val);
}
#define MH_RAWVAL_FUNC	// MHH:09/02/2021
#ifdef MH_RAWVAL_FUNC	// This doesn't seem to be used.
// simply return the filtered 10 bit value from the appropriate variable
WORD rawValue (AnalogChannel channel)
{
    return (rawValues[channel]);
}
#endif

BYTE betaSwitchOn (void)
{
    // The BETA switch is on the dimmer line,
    // switch is on when input is 0
    return (rawValues[A_DIMMER] < 200);
}

#ifndef AC210_PORT

// ISR.
// Copies out raw values through a filter..

void adInt (void)
{
    BYTE cnt;
    long temp;
    WORD val;

    // re-enable interrupts for this one, as it
    // is not time critical
    enableInterrupt();



    for (cnt = 0; cnt < 5; cnt++)
    {
        val = *(((WORD*)&AD0) + cnt);
        val &= 0x03FF;

        if (cnt == A_MOTOR_CURRENT)
        {
            mcCount++;

            if (mcCount <= 60)
            {
                // add into accumulators
                tempMotorCurrent += val;
            }
            if (mcCount == 60)
            {
                tempMotorCurrent /= 6; // count * 10
                // now update the main raw value
                temp = ((long)rawValues[cnt] * MCMP_FILTER_VALUE) +
                       ((100L - MCMP_FILTER_VALUE)*(long)tempMotorCurrent);
                rawValues[cnt] = (WORD)(temp/100);
                tempMotorCurrent = 0;
            }
            if (mcCount >= 60)
            {
                mcCount = 0;
            }

        }
        else if (cnt == A_MANIFOLD_PRESSURE)
        {
            mpCount++;

            if (mpCount <= 60)
            {
                // add into accumulators
                tempManifoldPressure += val;
            }
            if (mpCount == 60)
            {
                // now update the main raw value
                temp = ((long)rawValues[cnt] * MCMP_FILTER_VALUE) +
                       ((100L - MCMP_FILTER_VALUE)*(long)(tempManifoldPressure/60));
                rawValues[cnt] = (WORD)(temp/100);
                tempManifoldPressure = 0;
            }
            if (mpCount >= 60)
            {
                mpCount = 0;
            }

        }
        else
        {
            temp = ((long)rawValues[cnt] * FILTER_VALUE) + ((100L - FILTER_VALUE)*(long)val);
            rawValues[cnt] = (WORD)(temp/100);
        }
    }

    // start off A/D again
    ADCON0 |= 0x40;

}
#endif
