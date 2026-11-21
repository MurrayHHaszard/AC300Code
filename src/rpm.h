/* ------------------------------------------------------------
Title:          rpm.h

Copyright:      Aero Trading Ltd, December 2000

Date created:   30/11/2000
Author:         Peter Bodley

Product:        AC200 pitch Controller
Version:        1.0
Platform:       M16/62
Comment:        Header for RPM measurment code

Changes:

------------------------------------------------------------ */
#ifndef _RPM_H
#define _RPM_H

/*
Propeller speed monitoring
*/

// initialise speed monitoring code
void initSpeed (void);

// accessor functions return non zero if RPM changed
//BYTE magSpeed (ULONG *rpm);
//WORD magSpeed (void);		// MHH:20/11/2020
WORD Get_RPM(void);
BYTE ignSpeed (ULONG *rpm);
extern uint16_t Engine_rpm;
extern int16_t Engine_rpm_change_in_five_ticks;
// Claude 21/11/2026 (10.185). Tacho pulse capture (ac210_rpm.c; ATPCAP / ATPDMP)
void Pcap_start(int n);
int Pcap_count(void);
int Pcap_asked(void);
void Pcap_dump_start(void);
void Pcap_poll(void);
#endif
