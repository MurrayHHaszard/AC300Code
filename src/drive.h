/* ------------------------------------------------------------
Title:          Drive.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for Pitch Motor drive and state functions

Changes:

------------------------------------------------------------ */

#ifndef _DRIVE_H
#define _DRIVE_H

/*
Motor Drive Control
*/

// Auto control states..What the motor drive is
// actually doing at this stage

typedef enum
{
    MD_FEATHER = 0,
    MD_FEATHER_REVERSE,
    MD_IDLE,
    MD_FINER,
    MD_COARSER,
    MD_BETA,
    MD_BETA_HOLD,		// if speed goes over limit when in beta
    MD_BETA_EXIT,
    MD_FINE_BRAKE,
    MD_COARSE_BRAKE,
	MD_REVERSE,		//  MHH:18/07/2025
	MD_REVERSE_REVERSE
} MotorDriveState;



extern MotorDriveState dState;

void Set_dState(MotorDriveState new_dstate,int ifrom);	// MHH: 02/04/2018

void setIdle(int ifrom);

// Initialisation call
void initDriveControl (void);

// repeated call for control
void updateDriveControl (void);

// Overcurrent shutdown..
void shutdownDrive (void);

// state accessor
MotorDriveState driveState (void);

// disable & enable functions, used by auto test
void disableMotorDrive (void);
void enableMotorDrive (void);

// set in feather mode
void setDriveToFeather (void);
// stop driving
void cancelFeatherDrive (void);
//void cancelFeatherDrive (void);
// reverse feather
void reverseFeatherDrive (void);

#if BETA_VERSION
// Set the drive pin to enable relay
void setBetaDriveMode (void);
// release the relay and drive out of beta mode
void clearBetaDriveMode(void);

// Release relay temporarily due to overspeed
void setBetaHoldMode (void);
// Re-engage relay once speed comes back down
void clearBetaHoldMode (void);
// drop the realy (prior to reverse)
void clearBetaDriveModeForReverse (void);
#endif

#endif

