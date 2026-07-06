/*
 * port_adc.h
 *
 *  Created on: 19/10/2016
 *      Author: Murray
 */
#include "ac210_global.h"

#ifndef PORT_ADC_H_
#define PORT_ADC_H_

//#define ADC_CHAN_MAX		4
//#define ADC_CHAN_MAX		5		// MHH:27/01/2024. Allow ADC4 on Aux connector P8
#define ADC_CHAN_MAX		6		// MHH:04/02/2026. Allow ADC5 for temperature sensor on AC300UAV5
#define ADC_MOTOR_STATE		0
#define ADC_MOTOR_CURRENT	1
#define ADC_SUPPLY			2		// Battery voltage
#define ADC_DIMMER			3
#define ADC_SLIDER_CONTROL	4		// MHH:02/04/2024. Allow slider control of reverse angle.
#define ADC_TEMPERATURE		5
// Note: AC200 has manifold pressure as last ADC item, but it is never used.

struct ADC_struct
{
	uint32_t cnt;
	uint32_t ErrorCnt;
	uint32_t RawValue[ADC_CHAN_MAX];
	uint32_t TotVal[ADC_CHAN_MAX];
	uint32_t ChanCnt[ADC_CHAN_MAX];
};
extern struct ADC_struct ADC;

int AC210_ADC_Init(void);
uint32_t ADC_GetRawVal(uint8_t chan);

#endif /* PORT_ADC_H_ */
