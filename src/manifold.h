/* ------------------------------------------------------------
Title:          manifold.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   23/11/2001
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for manifold pressure control block

Changes:

------------------------------------------------------------ */

#ifndef _MANIFOLD_H
#define _MANIFOLD_H

// Return Values for the check routine
#if MAP_VERSION
typedef enum 
{
    PRESSURE_BELOW_MAP = 0,        // below MAP deadband
    PRESSURE_IN_MAP,
    PRESSURE_ABOVE_MAP,
    PRESSURE_IN_WARN,
    PRESSURE_LIMIT_EXCEEDED
} PressureValue;

// THis is just a True/False type of question

void loadManifoldPressureLimits (void);

PressureValue manifoldPressureCheck (void);
#endif

#endif
