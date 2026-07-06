/* Title:          manifold.c

Copyright:      Aero Trading Ltd, December 2000

Date created:   23/11/2001
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Manifold Pressure Check Code

Changes:

------------------------------------------------------------ */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "global.h"
#include "manifold.h"
#include "analog.h"
#include "param.h"
#include "rpm.h"
#include "digital.h"

#if MAP_VERSION

typedef struct
{
    WORD rpm;
    WORD mp;
} LimitPoint;

typedef struct
{
    WORD maxDeadBand;
    WORD mapDeadBand;
    LimitPoint maxLinePoints[5];     // {RPM,PRESSURE}
    LimitPoint mapLinePoints[5];
} PressureLimit;

// Figures in table must be to 1dp, ie 2500 rpm is 25000

PressureLimit pressureLimits;

PressureValue checkPressure (ULONG rpm, WORD pressure);

void loadManifoldPressureLimits (void)
{
    ParamIndex i,j;
    BYTE ctr;

    // first clear them
    memset (&pressureLimits, 0, sizeof (pressureLimits));
    // now load them if we are enabled for MAP
    if (manifoldPressureEnabled())
    {
        watchIt();
        for (ctr = 0; ctr < 5; ctr++)
        {
            pressureLimits.maxLinePoints[ctr].rpm = getParameter (MP_MAX_RPM_1 + (ParamIndex)ctr) * 10;
            pressureLimits.maxLinePoints[ctr].mp = getParameter (MP_MAX_MFP_1 + (ParamIndex)ctr);
            pressureLimits.mapLinePoints[ctr].rpm = getParameter (MP_MAP_RPM_1 + (ParamIndex)ctr) * 10;
            pressureLimits.mapLinePoints[ctr].mp = getParameter (MP_MAP_MFP_1 + (ParamIndex)ctr);
        }
        pressureLimits.maxDeadBand = getParameter (MP_MAX_DEADBAND);
        pressureLimits.mapDeadBand = getParameter (MP_MAP_DEADBAND);
    }
}

PressureValue manifoldPressureCheck (void)
{
    // Check for pressure, return 0 if not enabled.. this will cause NO lights to be on
    ULONG currentRPM;
    WORD  currentPressure;
    PressureValue retval = PRESSURE_BELOW_MAP;

    if (manifoldPressureEnabled())
    {
        if (getParameter(SINP) == MAG_INPUT)
        {
            magSpeed (&currentRPM);
        }
        else
        {
            ignSpeed (&currentRPM);
        }

        if (currentRPM > (getParameter(MIN_ENGINE_SPEED) * 10))
        {
            // only if above min controlling range

            // current manifold pressure
            currentPressure = scaledValue (A_MANIFOLD_PRESSURE);

            // rpm & pressure both to 1 dp
            retval = checkPressure ( currentRPM, currentPressure);
        }
    }
    return (retval);
}

PressureValue checkPressure (ULONG rpm, WORD pressure)
{
    // graphs are straight line segments between up to 5 points
    LimitPoint  *gp;
    WORD        rpmTemp;
    ULONG       pressTemp;
    BYTE        count;
    WORD        maxPressure;
    PressureValue retval = PRESSURE_BELOW_MAP;


    // Check the MAX line
    gp = &pressureLimits.maxLinePoints[0];

    // find out what segment we are in.. if any
    count = 0;
    // NB count < 5 must be first... in line below
    while ((count < 5) && ((*(gp+count)).rpm > 0) && (rpm > (*(gp + count)).rpm))
    {
        count++;
    }
    // count should be 0,1,2,3,4,5

    if ((count == 5) || ((*(gp+count)).rpm == 0))
    {
        //  applicable limit was earlier one if count > 0
        if (count)
        {
            // max pressure was last figure
            maxPressure = (*(gp + count - 1)).mp;
        }
        else
        {
            // a graph full of zeros....... use default
            maxPressure = rpm/100;
        }
    }
    else if (!count)
    {
        // less than the minimum.. & min is non zero
        maxPressure = gp->mp;
    }
    else
    {
        // between 2 points
        rpmTemp = (*(gp + count)).rpm - (*(gp + count - 1)).rpm;
        pressTemp = (*(gp + count)).mp - (*(gp + count - 1)).mp;
        // to get some accuracy, because we are dividing small
        // pressure difference by a large rpm difference
        pressTemp *= 1000; 
        pressTemp /= rpmTemp;
        // presstemp holds pressure diff per rpm * 100
        // normalise everything
        rpmTemp = rpm - (*(gp + count - 1)).rpm; // rpm > lower point (1dp)
        pressTemp *= rpmTemp;                  // pressure diff to 4 dp
        pressTemp /= 1000;                     //pressure diff to 1 dp 
        maxPressure = (*(gp + count - 1)).mp + (pressTemp);
    }

    // do the comparison
    if (pressure > maxPressure)
    {
        retval =  PRESSURE_LIMIT_EXCEEDED;
    }
    else if (pressure > maxPressure - pressureLimits.maxDeadBand)
    {
        retval =  PRESSURE_IN_WARN;
    }

    // now check for within MAP range if applicable
    if (retval == PRESSURE_BELOW_MAP)
    {
        gp = &pressureLimits.mapLinePoints[0];

        // count is already set from above
        if ((count == 5) || ((*(gp+count)).rpm == 0))
        {
            //  applicable limit was earlier one if count > 0
            if (count)
            {
                // max pressure was last figure
                maxPressure = (*(gp + count - 1)).mp;
            }
            else
            {
                // a graph full of zeros....... use default
                maxPressure = rpm/100;
            }
        }
        else if (!count)
        {
            // less than the minimum.. & min is non zero
            maxPressure = gp->mp;
        }
        else
        {
            // between 2 points
            rpmTemp = (*(gp + count)).rpm - (*(gp + count - 1)).rpm;
            pressTemp = (*(gp + count)).mp - (*(gp + count - 1)).mp;
            // to get some accuracy, because we are dividing small
            // pressure difference by a large rpm difference
            pressTemp *= 1000; 
            pressTemp /= rpmTemp;
            // presstemp holds pressure diff per rpm * 100
            // normalise everything
            rpmTemp = rpm - (*(gp + count - 1)).rpm; // rpm > lower point (1dp)
            pressTemp *= rpmTemp;                  // pressure diff to 4 dp
            pressTemp /= 1000;                     //pressure diff to 1 dp 
            maxPressure = (*(gp + count - 1)).mp + (pressTemp);
        }

        // do the comparison
        if (pressure > maxPressure + pressureLimits.mapDeadBand)
        {
            retval =  PRESSURE_ABOVE_MAP;
        }
        else if (pressure > maxPressure - pressureLimits.mapDeadBand)
        {
            retval =  PRESSURE_IN_MAP;
        }
    }
    return (retval);
}

#endif

