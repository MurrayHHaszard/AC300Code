/* ------------------------------------------------------------
Title:          leds.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   28/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        LED initialisation and control

Changes:

------------------------------------------------------------ */

#include <string.h>
#include <stdio.h>

#include "global.h"
#include "leds.h"
#include "comms.h"
#include "digital.h"
#include "analog.h"
#include "control.h"
#include "sstate.h"
#include "rpm.h"
#include "drive.h"
#include "param.h"
#include "feather.h"
#include "manifold.h"
#if REMOTE_VERSION
#include "remote.h"
#endif
#include "device.h"

extern bool Sig100_connected;
extern uint8_t LEDS_manual_calibration_status;

/*
    Port assignment is as follows
    P5.2  On board diagnostic LED
    P1.0  FINE mode Red LED
    P1.1  FINE mode Green LED
    P1.2  COARSE mode Red LED
    P1.3  COARSE mode Green LED
    P1.4  FEATHER mode Red LED
    P1.5  FEATHER mode Green LED
    P1.6  Backlight LED control

    PWM on TA0 output is used to dim the LED's (except diagnostic LED)
    Brightness control input on AD0 provides dimmer control..
*/

#ifndef AC210_PORT
#define LEDPORT     P1
#define LEDPORTDIR  PD1
#define LEDPORTMASK 0x7F
#define DIAGLEDPORT P5
#define DIAGLEDDIR  PD5
#define DIAGLEDMASK 0x04
#endif

// Voltage levels to trigger auto backlight control.
// values indicate VOLTAGE at the input (40 = 4.0)
// I assume that the voltage is that used to drive the actual backlight
// so that as it increases, we require more brightness
// Also, when the backlights are turned off, we want full brightness
// So, backlight will come on when voltage is > .3 (3). Below this
// brightness is MAX.
#define MAX_BRIGHTNESS_BELOW 3
#define BACKLIGHT_ON_LEVEL 15
#define BACKLIGHT_OFF_LEVEL 10

/*
// led state storage index
typedef enum
{
    LED_DIAG = 0,
    LED_FEATHER3_G,
    LED_FEATHER3_R,
    LED_COARSE_G,
    LED_COARSE_R,
    LED_FINE_G,
    LED_FINE_R,
#ifdef AC210_4LEDS
	LED_FEATHER4_G,
	LED_FEATHER4_R,
#endif
//	LED_BACKLIGHT,
	LED_RELAY,
    LED_ARRAY_INDEX
} LedIndex;
*/
BYTE ledState[LED_ARRAY_INDEX];
BYTE XoarStatus;
BYTE LED_overlay_flags;		// MHH:28/02/2018
BYTE LED_error;				// MHH:16/03/2018
BYTE LED_error_cnt;

#define FEATHER3_GREEN_BIT	2
#define FEATHER3_RED_BIT	4

#define REVERSE_GREEN_BIT	2
#define REVERSE_RED_BIT		4

#define COARSE_GREEN_BIT	8
#define COARSE_RED_BIT		16
#define FINE_GREEN_BIT		32
#define FINE_RED_BIT		64
#define FEATHER4_GREEN_BIT	128
#define FEATHER4_RED_BIT	256

WORD FEATHER_green_bit=FEATHER3_GREEN_BIT;
WORD FEATHER_red_bit=FEATHER3_RED_BIT;

// local procedure prototypes
// actually set pin state
void setLEDS (WORD state);

// Initialisation function
void initLEDS (void)
{
    BYTE cnt;
    // Port and Variable initialisation
    // set all to off
    for (cnt=0; cnt<LED_ARRAY_INDEX; cnt++)
    {
        ledState[cnt] = LED_OFF;
    }

    // set output states for all LEDS to OFF BEFORE we
    // set direction pins.....
    setLEDS (0);
#ifdef MH_XXX    // MHH:08/04/2026
	AC210_RELAY_Set(false);
#endif
#ifndef AC210_PORT
    // set up port pins as outputs
    LEDPORTDIR |= LEDPORTMASK;
    DIAGLEDDIR |= DIAGLEDMASK;

    PD7 |= 0x01;

    // Set up PWM on TA0
    TA0MR = 0x07;   // 16 bit PWM f1 input, use count start flag
    // initial value is 50% brightness
    TA0 = 0x8000;
    // start up the PWM
    TABSR |= 0x01;
#endif
    // Input brightness control A/D code is in atod.c
}
uint8_t LED_FEATHER_R_ix = LED_FEATHER3_R;
uint8_t LED_FEATHER_G_ix = LED_FEATHER3_G;

// update state display
void updateLEDS (void)
{
    BYTE featheringVersion;
    BYTE cnt, ticks;
    ULONG cSpeed;
    WORD mask;
    WORD sState;
    OpMode mode;
#ifndef AC210_PORT
    WORD cVal;
    ControlState cs;
#endif
    MotorDriveState ds;
    FeatherState fs;
    BYTE manKeys;
#if MAP_VERSION
    PressureValue mp;
#endif
//    BYTE control = 0; // all off
    WORD control = 0; // all off
    BYTE no_speed_signal = false;

    AC210_4leds = (getParameter (LED4_ENABLE) == 1);

    if(AC210_4leds)
    {
#ifdef MH_XXX	// MHH:08/04/2026
    	if(AC210_relay_on)
    	{
        	LED_FEATHER_R_ix = LED_FEATHER3_R;		// Really REVERSE LED
        	LED_FEATHER_G_ix = LED_FEATHER3_G;
    	}
    	else
    	{
        	LED_FEATHER_R_ix = LED_FEATHER4_R;
        	LED_FEATHER_G_ix = LED_FEATHER4_G;
    	}
#else
    	LED_FEATHER_R_ix = LED_FEATHER4_R;
    	LED_FEATHER_G_ix = LED_FEATHER4_G;

#endif
    	FEATHER_green_bit = FEATHER4_GREEN_BIT;
    	FEATHER_red_bit = FEATHER4_RED_BIT;
    }

    featheringVersion = isFeatheringOrReversingProp() || isBetaProp();

    // set brightness
    // a high dimmer input means (I think) that high brightness is
    // required. This corresponds to a smaller PWM output, as the
    // PWM drives an active low chip select.


#ifdef MH_BACKLIGHT		// Think this is redundant
#if BETA_VERSION
    cVal = 1;
#else
    cVal = scaledValue (A_DIMMER);
#endif

    if (cVal > 127)
    {
        cVal = 127;
    }

    if (cVal < MAX_BRIGHTNESS_BELOW)
    {
        ledState[LED_BACKLIGHT] = LED_OFF;
        cVal = 1;
    }
    else if (cVal > BACKLIGHT_ON_LEVEL)
    {
        ledState[LED_BACKLIGHT] = LED_ON;
        cVal = 127 - cVal;
    }
    else if (cVal < BACKLIGHT_OFF_LEVEL)
    {
        ledState[LED_BACKLIGHT] = LED_OFF;
        cVal = 127 - cVal;
    }
    else
    {
        cVal = 127 - cVal;
    }

    // Output brightness. Lower number = Higher Brightness
#ifndef AC210_PORT
    TA0 = (WORD)(cVal * 512);
#endif
#endif


    // update state of most of LEDS (except diagnostic & backlight)
    // based on calls to various parts of the system
    sState = systemState();

    if(Sig100_connected)
    {
    	if(dState == MD_BETA)
    	{
    		sState &= ~S_STOP_FINE;
    	}
    	if(dState == MD_FEATHER_REVERSE || dState == MD_FEATHER)	// MHH:21/07/2025
    	{
    		sState &= ~S_STOP_COARSE;
    	}
    }

    mode = operatingMode();
    manKeys = manualKeys();
//    cs = controlState (&cVal);
    ds = driveState();
    fs = featherState();
    // get actual speed from control loop
    cSpeed = currentActualSpeed();

    // clear old state
    for (cnt = 1; cnt < LED_ARRAY_INDEX; cnt++)
    {
        ledState[cnt] = LED_OFF;
    }
    XoarStatus = 0;		// Initialise

    // Error states override earlier ones
    if (sState & S_ERROR_CURRENT)
    {
        switch (overCurrentDir)
        {
        case 0:		// Happens when testing BL, and BL not connected but mhload1 is.
        			// Note that overcurrent takes priority over open circuit for LEDS.
            ledState[LED_FINE_R] = LED_ON;
            ledState[LED_COARSE_R] = LED_ON;
            break;

        case S_RUN_FINE:
            ledState[LED_FINE_R] = LED_ON;
            XoarStatus = 14;	// Over-current while pitch decreasing
            break;

        case S_RUN_COARSE:
            ledState[LED_COARSE_R] = LED_ON;
            XoarStatus = 15;	// Over-current while pitch increasing
            break;

        case S_RUN_FEATHER:
            if (featheringVersion) {
                ledState[LED_FEATHER_R_ix] = LED_ON;
                XoarStatus = 16;	// Over-current while pitch increasing in feather
			}
            break;

#ifdef AC210_4LEDS
        case S_RUN_REVERSE:
        	// May need a check that it can reverse?
            ledState[LED_FEATHER3_R] = LED_ON;
            XoarStatus = 16;	// ??? May need a new Xoar error. Over-current while pitch increasing in feather
            break;
#endif
        }
    }
//    else if (sState & S_ERROR_OPEN && (Sig100_connected == false))	// MHH:22/10/22 Temporary!!!!!
    else if (sState & S_ERROR_OPEN)	// MHH:25/07/23. Should not be able to set open circuit unless no BL connection
    {
    	XoarStatus = 17;	// Open circuit failure
        ledState[LED_FINE_R] = LED_FLASH_1HZ;
        ledState[LED_COARSE_R] = LED_FLASH_1HZ;
        if (featheringVersion)
            ledState[LED_FEATHER_R_ix] = LED_FLASH_1HZ;

        if(AC210_4leds)
        {
            ledState[LED_FEATHER3_R] = LED_FLASH_1HZ;
        }
    }

    else
    {
        // for each LED, go through and set state


        // FINE LED used for no speed input Indication
        if ((mode != MANUAL) &&
            (mode != FEATHER) &&
            (fs != F_BETA) &&
            (fs != F_BETA_EXIT) &&
            (ds != MD_FEATHER_REVERSE) &&
			(ds != MD_REVERSE_REVERSE) &&
//            (!waitingForStateCheck()) &&	// MHH:07/06/2019. As per Xoar request.
            (cSpeed < getParameter(SP_NO_RPM)))
        {
#define MH_NOSPEED_SIGNAL
#ifdef MH_NOSPEED_SIGNAL
            XoarStatus = 4;		// No speed signal
            no_speed_signal = true;	// as XoarStatus will be overridden by a stop
        }
        if (sState & (S_STOP_FINE| S_RUN_FINE))
#else
            // "No speed input" (< 100 RPM)). Flash Fine led Orange at 1 Hz
        	ledState[LED_FINE_R] = LED_FLASH_1HZ;
            ledState[LED_FINE_G] = LED_FLASH_1HZ;

            XoarStatus = 4;		// No speed signal
        }
        else if (sState & (S_STOP_FINE| S_RUN_FINE))
#endif
        {
            // something to show here..
            if (sState & S_STOP_FINE)
            {
                // at a stop, so we show green LED
                // if we are still driving then it is flashing
                if (sState & S_RUN_FINE)
                {
                    // flashing
                    ledState[LED_FINE_G] = LED_FLASH_2HZ;
                    XoarStatus = 8;		// Driving at fine pitch limit
                }
                else
                {
                    ledState[LED_FINE_G] = LED_ON;
                    XoarStatus = 5;		// Fine pitch limit
                }
            }
            else
            {
                // running state only..yellow on
                ledState[LED_FINE_R] = LED_ON;
                ledState[LED_FINE_G] = LED_ON;
                XoarStatus = 1;		// Pitch decreasing
            	if (ds == MD_FEATHER_REVERSE)
            	{
                    ledState[LED_COARSE_R] = LED_FLASH_5HZ;
					XoarStatus = 13;	// Not sure if this is right
            	}
            	else
            	{
            		if(sState & S_ERROR_SLIPRING)
            		{
            			XoarStatus = 23;		// MHH:11/01/2021. Slipring error while pitch decreasing
            		}
            	}
            }
        }
#ifdef MH_SLAVE_LED_HERE
    	static BYTE cntTimer;
        // COARSE
    	if(mode != HOLD)
    	{
    		cntTimer = 10;
    	}

        if ((mode == HOLD) && (isSlaveProp()))
        {
        	if(isSlaveCommsActive())
        	{
        		static BYTE cnt;		// Start flashing feather green 5 times a sec, slowing to once every 3 secs.
        		if(cntTimer < 150)cntTimer++;
        		if(cnt++ >= cntTimer) cnt=0;
        		if(cnt == 0)
        		{
            		ledState[LED_FEATHER_G_ix] = LED_ON;	// Feather LED green means OK
        		}
//        		LED_overlay_flags |= LED_OVERLAY_FLAG_REMOTE_ACTIVE;
//        		ledState[LED_FEATHER_G_ix] = LED_ON;	// Feather LED green means OK
        	}
        	else
        	{
        		cntTimer = 10;
        	// Coarse used for no comms in slave device
            // rpm data
//            ledState[LED_COARSE_R] = LED_FLASH_1HZ;
//            ledState[LED_COARSE_G] = LED_FLASH_1HZ;
        		ledState[LED_FEATHER_R_ix] = LED_FLASH_5HZ;	// Feather LED flashing red means not OK.
        	}
        }
#endif
        if (sState & (S_STOP_COARSE| S_RUN_COARSE))
        {
            // something to show here..
            if (sState & S_STOP_COARSE)
            {
                // at a stop, so we show green LED
                // if we are still driving then it is flashing
                if (sState & S_RUN_COARSE)
                {
                    // flashing
                    ledState[LED_COARSE_G] = LED_FLASH_2HZ;
                    XoarStatus = 9;		// Driving at coarse pitch limit
                }
                else
                {
                    ledState[LED_COARSE_G] = LED_ON;
                    XoarStatus = 6;		// Coarse pitch limit
                }
            }
            else
            {
                // running state only..yellow on
                ledState[LED_COARSE_R] = LED_ON;
                ledState[LED_COARSE_G] = LED_ON;
                XoarStatus = 2;		// Pitch increasing
        		if(sState & S_ERROR_SLIPRING)
        		{
        			XoarStatus = 24;		// MHH:11/01/2021. Slipring error while pitch increasing
        		}
            }
        }

#ifdef AC210_4LEDS
        // REVERSE
        if (sState & (S_STOP_REVERSE | S_RUN_REVERSE))
        {
            // something to show here..
            if (sState & S_STOP_REVERSE)
            {
                // at a stop, so we show green LED
                // if we are still driving then it is flashing
                if (sState & S_RUN_REVERSE)
                {
                    // flashing
                    ledState[LED_FEATHER3_G] = LED_FLASH_2HZ;
                    XoarStatus = 21;		// ?? Maybe add new value? Driving at reverse pitch limit
                }
                else
                {
                    ledState[LED_FEATHER3_G] = LED_ON;
                    XoarStatus = 22;		// Reverse pitch limit
                }
            }
            else
            {
                // running state only..yellow on
                ledState[LED_FEATHER3_R] = LED_ON;
                ledState[LED_FEATHER3_G] = LED_ON;
                if(fs != F_BETA)
                {
                	if(Control_type == CT_REVERSE)	// MHH:09/02/2018 Add test for reverse
                	{
                    	ledState[LED_FINE_R] = LED_FLASH_5HZ;
                    	ledState[LED_FINE_G] = LED_OFF;
                        XoarStatus = 19;		// Pitch decreasing in reverse
                	}
                	else
                	{
                    	ledState[LED_COARSE_R] = LED_FLASH_5HZ;	// MHH: 9/12/15 Check NOT beta
                        XoarStatus = 3;		// Pitch increasing in feather
                	}
                }
            }
        }
#endif
        // FEATHER
        if (sState & (S_STOP_FEATHER | S_RUN_FEATHER))
        {
            // something to show here..
            if (sState & S_STOP_FEATHER)
            {
                // at a stop, so we show green LED
                // if we are still driving then it is flashing
                if (sState & S_RUN_FEATHER)
                {
                    // flashing
                    ledState[LED_FEATHER_G_ix] = LED_FLASH_2HZ;
                    XoarStatus = 10;		// Driving at feather pitch limit
                }
                else
                {
                    ledState[LED_FEATHER_G_ix] = LED_ON;
                    XoarStatus = 7;		// Feather pitch limit
                }
            }
            else
            {
                // running state only..yellow on
                ledState[LED_FEATHER_R_ix] = LED_ON;
                ledState[LED_FEATHER_G_ix] = LED_ON;
                if(fs != F_BETA)
                {
//                	if(Control_type == CT_REVERSE)	// MHH:09/02/2018 Add test for reverse
                	if(Feather_mode == FM_REVERSING)
                	{
                    	ledState[LED_FINE_R] = LED_FLASH_5HZ;
                    	ledState[LED_FINE_G] = LED_OFF;
                        XoarStatus = 19;		// Pitch decreasing in reverse
                	}
                	else
                	{
                    	ledState[LED_COARSE_R] = LED_FLASH_5HZ;	// MHH: 9/12/15 Check NOT beta
                        XoarStatus = 3;		// Pitch increasing in feather
                	}
                }
            }
        }

//#if XOAR_VERSION
// Note: Have used Manifold Pressure LED positions as:
//	a) manifold pressure not implemented yet (17/08/2015)
//	b) Xoar version does not (currently) allow feathering or beta, so we can use those LEDs.

//        else if (S_Pkt.Mode != SERIAL_MODE_ASCII || RC_S_mode_is_setspeed())
        else if (S_Pkt.Mode != SERIAL_MODE_ASCII)
        {
            ledState[LED_FEATHER_R_ix] = LED_ON;
            ledState[LED_FEATHER_G_ix] = LED_ON;
        }
        else if (Auto_flag)
        {
        	switch(Auto_flag)
        	{
        	case 1:				// Take-off = Red
            	ledState[LED_FEATHER_R_ix] = LED_ON;
            	break;
        	case 2:				// Climb = orange
            	ledState[LED_FEATHER_R_ix] = LED_ON;
                ledState[LED_FEATHER_G_ix] = LED_ON;
            	break;
        	case 3:				// Cruise = green
                ledState[LED_FEATHER_G_ix] = LED_ON;
                break;
        	case 4:				// Hold = green with blink
        		ledState[LED_FEATHER_G_ix] = LED_FLASH_P2_HZ;
        		break;
        	case 6:				// Manual = Red with blink
        		ledState[LED_FEATHER_R_ix] = LED_FLASH_P2_HZ;
        		break;
        	case AUTO_THROTTLE_TIMEOUT:
                // "No throttle input", only in HOLD mode and parameter selected for throttle control
                ledState[LED_FEATHER_R_ix] = LED_FLASH_1HZ;
                ledState[LED_FEATHER_G_ix] = LED_FLASH_1HZ;
                break;
        	}
        }
//#endif

#if MAP_VERSION
        // Manifold Pressure Indicators
        else if (manifoldPressureEnabled())
        {
            mp = manifoldPressureCheck();
            switch (mp)
            {
            case PRESSURE_LIMIT_EXCEEDED:
                ledState[LED_FEATHER_R_ix] = LED_ON;
                break;

            case PRESSURE_IN_WARN:
                ledState[LED_FEATHER_R_ix] = LED_ON;
                ledState[LED_FEATHER_G_ix] = LED_ON;
                break;

            case PRESSURE_IN_MAP:
                if (mode == MAP)
                {
                    ledState[LED_FEATHER_G_ix] = LED_ON;
                }
                break;
            }
        }
#endif
    }


#if REMOTE_VERSION
#ifndef XOAR_VERSION
    // we need to differentiate the LEDS from the normal..
    // do this by shifting control left 1 bit, ie on becomes flash 1Hz
    // flash 2 Hz becomes flash5 Hz etc

#ifdef MH_REMOTE_LED_SHIFT		// MHH:09/02/2018. Ifdef this logic out as is confusing

    if (S_Pkt.Mode == SERIAL_MODE_ASCII && RC_S_mode_is_active() == false)	// If not Xoar and not Remote Standard active
    {
        if (remoteMode() > REMOTE_IDLE && AT_Remote == 0)
        {
            for (cnt = 1; cnt < 7; cnt++)
            {
                if (ledState[cnt] != LED_OFF)
                {
                    ledState[cnt] <<= 1;
                }
            }
        }
    }
#endif

#endif
#endif

    // Special case is invalid config
    if (invalidConfig)
    {
        ledState[LED_COARSE_R] = LED_FLASH_5HZ;
        ledState[LED_COARSE_G] = LED_OFF;
        ledState[LED_FINE_R] = LED_FLASH_5HZ;
        ledState[LED_FINE_G] = LED_OFF;
        ledState[LED_FEATHER_R_ix] = LED_FLASH_5HZ;
        ledState[LED_FEATHER_G_ix] = LED_OFF;
        if(AC210_4leds)
        {
            ledState[LED_FEATHER3_R] = LED_FLASH_5HZ;
            ledState[LED_FEATHER3_G] = LED_OFF;
        }
        XoarStatus = 18;	// Controller software fault
    }
    else	 	// MHH: 09/03/2018.
    {
        if(LED_overlay_flags & LED_OVERLAY_FLAG_REVERSE_ZONE)
    	{
        	ledState[LED_FINE_R] = LED_FLASH_5HZ;
        	ledState[LED_FINE_G] = LED_OFF;
        	if(XoarStatus == 0 || XoarStatus == 5)
        	{
                XoarStatus = 20;		// In Reverse Zone
        	}
    	}


        static BYTE cntTimer;
        // COARSE
        if(mode != HOLD)
        {
        	cntTimer = 10;
        }

        if ((mode == HOLD) && (isSlaveProp()))
        {
        	if(isSlaveCommsActive())
        	{
        		static BYTE cnt;		// Start flashing feather green 5 times a sec, slowing to once every 3 secs.
        		if(cnt++ >= cntTimer)
        		{
        			cnt=0;
            		if(cntTimer < 150)cntTimer+= 10;
        		}

        		if(cnt == 0)
        		{
            		ledState[LED_FEATHER_G_ix] = LED_ON;	// Feather LED green means OK
        		}
        //        		LED_overlay_flags |= LED_OVERLAY_FLAG_REMOTE_ACTIVE;
        //        		ledState[LED_FEATHER_G_ix] = LED_ON;	// Feather LED green means OK
        	}
        	else
        	{
        		cntTimer = 10;
        	// Coarse used for no comms in slave device
            // rpm data
        //            ledState[LED_COARSE_R] = LED_FLASH_1HZ;
        //            ledState[LED_COARSE_G] = LED_FLASH_1HZ;
        		ledState[LED_FEATHER_R_ix] = LED_FLASH_5HZ;	// Feather LED flashing red means not OK.
        	}
        }

    }

#if BETA_VERSION
    if(Control_type == CT_BETA)
    {
        // Attempt to provide some state feedback for beta mode
        // The feedback is circular red flashing leds (ie F-C-B repeating)
        // If half way, we want to do this at 2 hz with each LED on for 3 ticks
        // if all the way, 5 hz with each LED on for 6 ticks
        // These indications are only done if the LED in question has no other
        // state to indicate.

        if ((sState & S_ERROR_CURRENT) == 0)
        {
        	// Do not override overcurrent errors
            if ((fs == F_BETA) && (ds == MD_BETA_HOLD))
            {
            	// there will not be an overcurrent to Fine in BETA mode
            	// and all drive stuff is based on Feather stop, not fine
            	LED_overlay_flags |= LED_OVERLAY_FLAG_BETA_MAX_RPM;
#ifdef MH_XXX          // MHH:08/04/2026
            	if (!invalidConfig)
            	{
            		ledState[LED_FINE_G] = LED_OFF;
                	ledState[LED_FINE_R] = LED_FLASH_2HZ_25;
            	}
#endif
            }
            else if ((fs == F_BETA) || (fs == F_BETA_EXIT))
            {
                if (!invalidConfig)
                {
                	ledState[LED_FINE_G] = LED_OFF;
                    ledState[LED_FINE_R] = LED_FLASH_5HZ;
                    XoarStatus = 12;	// Fine pitch limit override (Beta option)
                }
            }
            else if (0xFF == betaModePending() && (manKeys & MANUAL_KEY_FEATHER))
            {
            	// Overspeed, cannot get into Beta
            	LED_overlay_flags |= LED_OVERLAY_FLAG_BETA_MAX_RPM;
                if (!invalidConfig)
                {
#ifdef MH_XXX		// MHH:08/04/2026
                	ledState[LED_FINE_G] = LED_OFF;
                    ledState[LED_FINE_R] = LED_FLASH_2HZ_25;
#endif
                    XoarStatus = 11;	// Beta mode engage (step 1) - perhaps should distinguish between ready or not
                }
            }

            else if (0x01 == betaModePending())
            {
                if (!invalidConfig)
                {
                	if((LED_overlay_flags & LED_OVERLAY_FLAG_BETA_MAX_RPM) == 0)
                	{
                    	ledState[LED_FINE_G] = LED_OFF;
                        ledState[LED_FINE_R] = LED_FLASH_1HZ_10;
                	}
                    XoarStatus = 11;	// Beta mode engage (step 1)
                }
            }
        }
    }
#endif

    // calculate the actual state at this point in time
    // set the update mask for the first LED
    mask = 0x01; // Diagnostic LED
    // get timer tick count
    // do the diagnostic & 3 tricolour LEDS

    for (cnt=0; cnt < LED_ARRAY_INDEX; cnt++)		// For each LED. 0 = Diagnostic
    {

        ticks = timerTicks();
        switch (ledState[cnt])
        {
        case LED_OFF:
            break;

        case LED_ON:
            control |= mask;
            break;

        case LED_FLASH_1HZ:
            // ON at 0, OFF at 25
            if (ticks < 25)
            {
                control |= mask;
            }
            break;

        case LED_FLASH_2HZ:
            // ON at 0,26, OFF at 13,38
            ticks /= 13;
            if ((ticks %2)== 0)
            {
                control |= mask;
            }
            break;

        case LED_FLASH_1HZ_10:
        	// On at 0, OFF at 5
            if (ticks < 6)
            {
                control |= mask;
            }
            break;

        case LED_FLASH_2HZ_25:
            // ON at 0,26, OFF at 6, 32
            if ((ticks % 26) < 7)
            {
                control |= mask;
            }
            break;


        case LED_FLASH_5HZ:
            // ON at 0,10,20,30,40, OFF at 5,15,25,35,45
            ticks /= 5;
            if ((ticks % 2)== 0)
            {
                // turn on
                control |= mask;
            }
            break;

        case LED_FLASH_5HZ_25:
            // ON at 0,10,20,30,40, OFF at 2,12,22,32,42
            if ((ticks % 10) < 3)
            {
                control |= mask;
            }
            break;

        case LED_FLASH_P2_HZ:		// 0.2 Hz
            // ON at 0-39, off 40-49
        	if((TimeInSeconds %5) || (ticks < 40))		// On, except for every 5th second when ticks between 40 and 49
        	{
                control |= mask;
        	}
            break;

        }
        //update mask for next LED
        mask <<= 1;
    }
    // add in the state of the backlight
#ifdef AC210_LED_BACKLIGHT
    if (ledState[LED_BACKLIGHT] == LED_ON)
    {
        control |= 0x80;
    }
#endif

#ifdef AC210_PORT
//#define MH_DISPLAY_XOAR
#ifdef MH_DISPLAY_XOAR
    static BYTE old_XoarStatus;
    if(XoarStatus != old_XoarStatus)
    {
    	old_XoarStatus = XoarStatus;
    	PRINTF("XoarStatus:%d\r\n",XoarStatus);
    }
#endif
#endif

// Special case, manual calibration. Need to restart to clear

    if(LEDS_manual_calibration_status != 0)
    {
    	if(TimeInSeconds & 1)
    	{
    		ticks = timerTicks();
    		if(ticks <= 5 || (ticks > 15 && ticks <= 21))
    		{
    			switch(LEDS_manual_calibration_status)
    			{
    			case 1:	// Orange while calibrating
    				control |= FEATHER_red_bit;		// Turn FEATHER RED on
        			control |= FEATHER_green_bit;	// Turn FEATHER GREEN on.
    				break;
    			case 2:	// Green for successful finish
    				control &= ~FEATHER_red_bit;	// Turn FEATHER RED off
        			control |= FEATHER_green_bit;	// Turn FEATHER GREEN on.
    				break;
    			case 3:	// Red for FAIL at finish
    				control &= ~FEATHER_green_bit;	// Turn FEATHER RED off
    				control |= FEATHER_red_bit;		// Turn FEATHER RED on
    				break;
    			}
    		}
    	}
    }

    // if Remote connection then Blink RED once every 2 seconds.

    if(LED_overlay_flags & (LED_OVERLAY_FLAG_REMOTE_ACTIVE | LED_OVERLAY_FLAG_AUTOGYRO_ACTIVE))
    {
    	if(TimeInSeconds & 1)
    	{
            ticks = timerTicks();
        	if(ticks <= 5)
        	{
#ifdef MH_XXX	// MHH:30/12/2023
        		control &= ~FEATHER_green_bit;		// Turn FEATHER GREEN off
        		control |= FEATHER_red_bit;			// Turn FEATHER RED on.
#endif
        		control &= ~FEATHER_red_bit;		// Turn FEATHER RED off
        		control |= FEATHER_green_bit;			// Turn FEATHER GREEN on.
        	}
    	}
    }
    if(LED_overlay_flags & LED_OVERLAY_FLAG_BETA_MAX_RPM)	// MHH:08/04/2026. Same as above for now.
    {
    	if(TimeInSeconds & 1)	// Aiming for 3 red pulses on FINE LED once every 2 secs.
    	{
    		ticks = timerTicks();
			control &= ~FINE_GREEN_BIT;		// Turn FINE GREEN off
			if(ticks < 25)
			{
				BYTE b = ticks / 5;
				if((b & 1) == 0) 	// b even?
				{
	    			control |= FINE_RED_BIT;		// Turn FINE RED on.
				}
			}
    	}
    	LED_overlay_flags &= ~LED_OVERLAY_FLAG_BETA_MAX_RPM;
    }

    if(LED_overlay_flags & LED_OVERLAY_FLAG_BL_POS_ERROR)	// MHH:21/07/2023. Same as above for now.
    {
    	if(TimeInSeconds & 1)	// Aiming for 2 red pulses on feather LED once every 2 secs.
    	{
    		ticks = timerTicks();
    		if(ticks <= 5)
    		{
    			control &= ~FEATHER_green_bit;		// Turn FEATHER GREEN off
    			control |= FEATHER_red_bit;			// Turn FEATHER RED on.
    		}
    		else
    		{
    			if(ticks > 15 && ticks <= 21)
    			{
    				control &= ~FEATHER_green_bit;		// Turn FEATHER GREEN off
    				control |= FEATHER_red_bit;			// Turn FEATHER RED on.
    			}
    		}
    	}
    }
#ifdef MH_XXX
    if (overCurrentTrip)	// MHH:24/07/2023. 3 red pulses on feather LED once every 2 secs.
    {
    	if(TimeInSeconds & 1)
    	{
    		ticks = timerTicks();
    		if(ticks <= 5)
    		{
    			control &= ~FEATHER_green_bit;		// Turn FEATHER GREEN off
    			control |= FEATHER_red_bit;			// Turn FEATHER RED on.
    		}
    		else
    		{
    			if(ticks > 15 && ticks <= 21)
    			{
    				control &= ~FEATHER_green_bit;		// Turn FEATHER GREEN off
    				control |= FEATHER_red_bit;			// Turn FEATHER RED on.
    			}
    			else
    			{
    				if(ticks > 25 && ticks <= 31)
    				{
        				control &= ~FEATHER_green_bit;		// Turn FEATHER GREEN off
        				control |= FEATHER_red_bit;			// Turn FEATHER RED on.
    				}
    			}
    		}
    	}
    }
#endif
    if(LED_overlay_flags & LED_OVERLAY_FLAG_AUTOGYRO_LATCHED)
    {
    	if(TimeInSeconds & 1)
    	{
            ticks = timerTicks();
        	if((ticks > 15) && (ticks <= 21))
        	{
        		control &= ~FEATHER_green_bit;		// Turn FEATHER GREEN off
        		control |= FEATHER_red_bit;			// Turn FEATHER RED on.
        	}
    	}
    }

#ifdef MH_XXX 	// MHH:08/04/2026

    if(LED_overlay_flags & LED_OVERLAY_FLAG_REVERSE_RELAY_ON)
    {
    	if(TimeInSeconds & 1)
    	{
            ticks = timerTicks();
        	if(ticks <= 5)
        	{
        		control &= ~REVERSE_GREEN_BIT;		// Turn REVERSE GREEN off
        		control |= REVERSE_RED_BIT;			// Turn REVERSE RED on.
        	}
    	}
    }

    if(LED_overlay_flags & LED_OVERLAY_FLAG_REVERSE_BUTTON_ON)
    {
		control &= ~REVERSE_GREEN_BIT;		// Turn REVERSE GREEN off
		control |= REVERSE_RED_BIT;			// Turn REVERSE RED on.
		LED_overlay_flags &= ~LED_OVERLAY_FLAG_REVERSE_BUTTON_ON;	// turn flag off
    }
#endif
    // Note: This could be a subroutine with different error values and actions.
	switch(LED_error)
	{
	case LED_ERROR_REVERSE:
    	if(LED_error_cnt == 0)		// First time?
    	{
        	LED_error_cnt = 50;			// 50 cycles
    	}
    	LED_error_cnt--;
    	if(LED_error_cnt <= 0)
    	{
    		LED_error = 0;
        	break;
    	}
    	if((LED_error_cnt & 3) < 2)		// 2 cycles on, 2 off
    	{
    		control &= ~REVERSE_GREEN_BIT;		// Turn FEATHER GREEN off
    		control |= REVERSE_RED_BIT;			// Turn FEATHER RED on.
    	}
    	else
    	{
    		control &= ~REVERSE_GREEN_BIT;		// Turn FEATHER GREEN off
    		control &= ~REVERSE_RED_BIT;		// Turn FEATHER RED off.
    	}
    	break;

	case LED_ERROR_STOP_POS_OK:
	case LED_ERROR_STOP_POS_SET:
    	if(LED_error_cnt == 0)		// First time?
    	{
        	LED_error_cnt = 30;			// 30 cycles
    	}
    	LED_error_cnt--;
    	if(LED_error_cnt <= 0)
    	{
    		LED_error = 0;
        	break;
    	}
    	if((LED_error_cnt & 7) < 4)		// 4 cycles on, 4 off
    	{
    		if(LED_error == LED_ERROR_STOP_POS_OK)
    		{
        		control |= REVERSE_GREEN_BIT;			// Turn FEATHER GREEN on.
    		}
    		else
    		{
        		control |= REVERSE_RED_BIT;			// Turn FEATHER RED on.
    		}
    	}
    	else
    	{
    		control &= ~REVERSE_GREEN_BIT;		// Turn FEATHER GREEN off
    		control &= ~REVERSE_RED_BIT;		// Turn FEATHER RED off.
    	}
    	break;
	}

#ifdef MH_NOSPEED_SIGNAL
    if((invalidConfig == false) && no_speed_signal)		// No speed signal
    {
//    	if(TimeInSeconds & 1)		// MHH: 28/01/2019. Supposed to be at 1 HZ
    	if(XoarStatus != 11)		// MHH:16/09/2025. Not if going into beta
//    	{
        	if(ticks <= 5)
        	{
        		control |= FINE_GREEN_BIT;		// Turn FINE GREEN on
        		control |= FINE_RED_BIT;		// Turn FINE RED on.
        	}
//    	}
    }
#endif

    // and set the output pins

    setLEDS (control);
}
// Diagnostic LED control

#define MH_DIAG_LED		// Think this is the blinking light on PCB.
#ifdef MH_DIAG_LED
void diagnosticDisplay ( BYTE state )
{
    ledState[LED_DIAG] = state;
}
#endif
// Backlight Control
#ifdef AC210_BACKLIGHT

void setBackLight (BYTE on)
{
    if (on)
    {
        ledState[LED_BACKLIGHT] = LED_ON;
    }
    else
    {
        ledState[LED_BACKLIGHT] = LED_OFF;
    }
}
#endif

// actually set pin state
void setLEDS (WORD state)
{
#ifdef AC210_PORT
	p_setLEDS(state);
#else
	// diagnostic LED
    if (state & 0x01)
    {
        DIAGLEDPORT |= DIAGLEDMASK;
    }
    else
    {
        DIAGLEDPORT &= ~DIAGLEDMASK;
    }
    // rest of LEDS
    // drop off diag led bit
    state >>= 1;
    state &= LEDPORTMASK;
    // set all off
    LEDPORT &= ~LEDPORTMASK;
    // and set to true state
    LEDPORT |= state;
#endif
}

