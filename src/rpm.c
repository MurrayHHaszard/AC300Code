/* ------------------------------------------------------------
Title:          rpm.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   28/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        RPM inputs and calulations
Changes:

------------------------------------------------------------ */
#include <string.h>
#include <stdio.h>

#include "global.h"
#include "param.h"

/*  RPM measurement is performed by utilising counters TB3 & TB4 in period
    measurement mode. The addition of an overflow counter provides accuracy
    to less than 1RPM over the range from 1RPM up to 10,000 RPM. The clock
    input is set at 2,000,000 Hz (ie f8). Code is interrupt driven, with results
    filtered and updated after each period interrupt.

*/

// The overflow value beyond which we call it 0 RPM
// while this system will measure down to < 1 pulse per minute accurately..
// speed of response to anything at this speed range is VERY slow
// at 1 pulse per minute, it takes 60 seconds to get an update.
// For this reason, the minimum resolvable speed is set at 60 pulses per minute
// which means it will take us a second to detect that we have stopped
// Please be aware that the scaling factor does affect the real world minimum
// speed that this represents..
// ie at 1 pulse per rev, this equates to 60 RPM, however
// at    4 pulse per rev, it is 15 RPM

// Storage Data
WORD magRPM;       // No dec places.

// initialise speed monitoring code
void initSpeed (void)
{
    magRPM = 0;

}

// accessor functions
WORD magSpeed (void)
{
    char buf[30];
    static WORD LastRPM;

    magRPM = p_GetAC200_magRPM(getParameter(SF_MAG_SPEED));

    if (debug & DEBUG_SPEED_INPUT)
    {
        bool rpm_changed = false;
        if(magRPM != LastRPM)
        {
        	rpm_changed = true;
        	LastRPM = magRPM;
        }
    	if(rpm_changed || (ac2_test_flag == AC2_TEST_ON))
    	{
//        	long mag_rpm10 = (magRPM*10)/RPMFACTOR;	// MHH:20/11/2020
//    		sprintf (buf, "Mag RPM -> %lu.%d", magRPM/10, magRPM%10);
    		sprintf (buf, "Mag RPM -> %u.0", magRPM);
            txDebug(buf);
            if(rpm_changed && ac2_test_flag != AC2_TEST_ON) txDebug("\r\n");
    	}
    }
    return magRPM;
}
//-----------------------------------------------------------------------------------------------------
extern int Engine_rpm_1dec;
WORD Get_RPM(void)
{
	WORD rpm  = (Engine_rpm_1dec + 5) / 10;		// MHH:10/11/2024. Note: need to check is set to zero if prop not moving.

//	WORD rpm;
//    rpm = magSpeed ();
	return rpm;
}

