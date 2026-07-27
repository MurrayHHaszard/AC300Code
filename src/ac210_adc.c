/*
 * @brief ADC example
 * This example show how to  the ADC in 3 mode : Polling, Interrupt and DMA
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "ac210_adc.h"

#include "ac210_timer.h"
#include "analog.h"
#include "board.h"
#include "sstate.h"
#include "drive.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define ADC_IRQn ADC_IRQn
static ADC_CLOCK_SETUP_T ADCSetup;


struct ADC_struct ADC;
//#define ADC_SAMPLE_MULTIPLE	5
//#define ADC_SAMPLE_MULTIPLE	1		// MHH:09/11/2018
//#define ADC_SAMPLE_MULTIPLE	1		// MHH:22/11/2018???
#define ADC_SAMPLE_MULTIPLE	5		// MHH:17/12/2020..Test
#define ADC_SAMPLE_RATE		(1000*ADC_SAMPLE_MULTIPLE)
//#define ADC_SAMPLE_RATE		(200*ADC_SAMPLE_MULTIPLE)
//#define ADC_FILTER_VAL		90
//#define ADC_FILTER_VAL		80	// MHH:06/02/2019
#define ADC_FILTER_VAL		60	// MHH:17/12/2020

#define ADC_STATE_FILTER	80
//#define ADC_FILTER_VAL	80

void setOutput (TestState ts,int ifrom);

//-------------------------------------------------------------------------------------------
int AC210_ADC_Init(void)
{

	/*	Chip_IOCON_PinMux(0, 25, IOCON_ADMODE_EN, IOCON_FUNC1); */
	/*ADC Init */
	Chip_ADC_Init(LPC_ADC, &ADCSetup);
//	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, 500);
	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, ADC_SAMPLE_RATE);
//	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, 10000);
//	for(int i=0;i<4;i++)
	for(int i=0;i<ADC_CHAN_MAX;i++)	// MHH:27/01/2024
	{
		Chip_ADC_EnableChannel(LPC_ADC,ADC_CH0+i, ENABLE);
	}
#ifdef MH_OLD_ENABLE
	Chip_ADC_EnableChannel(LPC_ADC,ADC_CH0, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC,ADC_CH1, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC,ADC_CH2, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC,ADC_CH3, ENABLE);
#endif

// Note: Will need to add ADC_CH3 here if we implement DIMMER

	NVIC_EnableIRQ(ADC_IRQn);
//	Chip_ADC_Int_SetChannelCmd(LPC_ADC,ADC_CH3, ENABLE);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC,ADC_CH4, ENABLE);	// MHH:27/01/2024
	Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------
// Getting problems trying to erase and/or program flash memory sometimes, which seems to be linked to ADC routines.
// Eg physically moving location of ADC routines made it disappear for a while.
// As a workaround, make sure it is turned off when program finishes.
void ADC_DeInit(void)
{
	NVIC_DisableIRQ(ADC_IRQn);
	Chip_ADC_DeInit(LPC_ADC);
}
//---------------------------------------------------------------------------------------------------------------------
// These defs from existing AC200 code
/*
typedef enum
{
    A_DIMMER = 0,
    A_SUPPLY,
    A_MOTOR_CURRENT,
    A_MOTOR_STATE,
    A_MANIFOLD_PRESSURE
} AnalogChannel;
*/
extern uint16_t rawValues[A_MAX];
//extern TestState tState;

// We shouldn't need Manifold pressure, and I cannot find any reference to A_SUPPLY
// In existing code.

//#define ADC_CURR_IX_MAX		20
#define ADC_CURR_IX_MAX		10		// MHH:15/07/2026
const uint8_t ADC_ChanMaxCnt[ADC_CHAN_MAX]={
		2,								// ADC0 = State
		ADC_CURR_IX_MAX,				// ADC1 = Current
		10*ADC_SAMPLE_MULTIPLE,			// ADC2 = Supply (Battery voltage)
		10*ADC_SAMPLE_MULTIPLE,			// ADC3 = Dimmer. Used in Auxiliary port to test when AutoGyro ready.
		10*ADC_SAMPLE_MULTIPLE,			// ADC4 = Aux.P8. MHH:27/01/2024. Also used in Auxiliary port.
		10*ADC_SAMPLE_MULTIPLE,			// ADC5 = Temperature. MHH:04/02/2026.

};
//-----------------------------------------------------------------------------------------------------------------
// Map native ADC raw values to existing code.
// Note: May have to massage values a bit as using different way of collecting ADC data.

//static int Current_flag;
//#define ADC_ZERO_CURRENT_VAL		4
int ADC_raw_current;			// MHH:06/11/2018
int  getCurrentOffset (void);
void ADC_UpdateAC200_RawValues(uint8_t chan)
{
	int rawval;
	int raw_current=0;

	uint16_t ChanMaxCnt = ADC_ChanMaxCnt[chan];
	switch(chan)
	{
	case ADC_MOTOR_STATE:
//		rawValues[A_MOTOR_STATE] = ADC.RawValue[ADC_MOTOR_STATE];
		rawval = ((ADC.RawValue[ADC_MOTOR_STATE])*2)/5;	// Adjust to range AC200 is expecting
		rawValues[A_MOTOR_STATE] = rawval;	// MHH:23/06/15
		break;

	case ADC_MOTOR_CURRENT:
//		if(tState != testIdle) return;	// MHH:07/03/2017. testFeatherReverse set when reversing out of feather. Need to know current
										// otherwise FINE_STOP will be set if thinks current==0.

//		if(tState > testIdle && tState < testFinish) return;	// MHH:19/07/2019???

		rawval = ADC.RawValue[ADC_MOTOR_CURRENT];
		int offset = getCurrentOffset () * 10;


		if(AC210_hardware_version <= 'D')	// MHH: 09/01/2019
		{
			if(rawval <= 50) rawval = 0;		// MHH:27/04/2024
			rawval += (offset/2);
		}

		if(AC210_hardware_version == 'F')	// 24V capable
		{
//			rawval = (rawval - (50 - offset));
			rawval = (rawval - (100 - offset));	// MHH:15/05/2026
		}
		if(AC210_hardware_version == 'G')	// 24V capable
		{
			rawval = (rawval - (20 - offset));
		}
		if(AC210_hardware_version >= 'H')	// 24V capable
		{
			rawval /= ChanMaxCnt;
//			rawval -= 365;
//			rawval -= 5;
//			rawval += offset;
//			if(rawval < 10) rawval = 0;
			ADC_raw_current = rawval;
//			raw_current = (rawval * 18)/10 + offset;	// *1.8
			raw_current = (rawval * 20)/10 + offset;	// MHH:10/07/2023 *2.0
			if(raw_current < 10) raw_current = 0;
//			if(raw_current > 0) raw_current += 20;		// MHH:27/01/2019
			if(raw_current > 0 && raw_current < 20) raw_current = 20;		// MHH:10/07/2023
			// Note: The hub pcb draws about 35 mA when motor not running. May need to account for this...
		}
		else
		{
			if(AC210_hardware_version <= 'D')	// MHH: 09/01/2019
			{
//				if(rawval < 2) rawval = 0;	// MHH:04/02/2019
//				if(rawval < 10) rawval = 0;	// MHH:27/07/2019
				if(rawval < 20) rawval = 0;	// MHH:05/08/2019

//				if(rawval > 0)
//					mh_debug();
//					printf("rawval=%d\r\n",rawval);

			}
			else
			{
				if(rawval < 10) rawval = 0;
			}
			if(rawval > 0)
			{
//				rawval += 100;
				if(rawval < 100) rawval = 100;	// MHH:06/08/2019
			}

			ADC_raw_current = rawval;		// So we can try and work out how to handle Maxon low current
			rawval = (rawval * 63)/10;		// Do adjustment here (*6.3)
			raw_current = rawval / ChanMaxCnt;
		}
		rawValues[A_MOTOR_CURRENT] = raw_current;
		break;

	case ADC_SUPPLY:
		rawValues[A_SUPPLY] = ADC.RawValue[ADC_SUPPLY]/ChanMaxCnt;
		break;

	case ADC_DIMMER:
		rawval = ADC.RawValue[ADC_DIMMER]/ChanMaxCnt;
		if(rawval <= 20) rawval = 0;
		rawValues[A_DIMMER] = rawval;
		break;

	case ADC_SLIDER_CONTROL:
		rawval = ADC.RawValue[ADC_SLIDER_CONTROL]/ChanMaxCnt;
		rawValues[A_SLIDER_CONTROL] = rawval;
		break;

	case ADC_TEMPERATURE:
		rawval = ADC.RawValue[ADC_TEMPERATURE]/ChanMaxCnt;
		rawValues[A_TEMPERATURE] = rawval;
		break;


	}
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t ADC_GetRawVal(uint8_t chan)
{
	return ADC.RawValue[chan];
}
//---------------------------------------------------------------------------------------------------------------------
//#define ADC_FILTER_VAL	80

//#define MH_DEBUG_ADC
#ifdef MH_DEBUG_ADC
#define	CURR_TBL_MAX		25
//static uint32_t Current_raw_max;
static uint16_t Current_tbl[CURR_TBL_MAX];
static int Current_ix;
//static int Curr_bad_count;
#define CMAX2		16
static int Curr_ix2;
static uint16_t Curr_tbl2[CMAX2];
static uint16_t Curr_tbl3[CMAX2];
#endif
//static uint16_t current_last_raw,current_prev_raw;
int adc_good[4];
int adc_bad[4];
//static uint32_t adcval;
//bool Testing_state(void);		// sstate.c

//-------------------------------------------------------------------------
//#define MH_DEBUG_CURRENT
#ifdef MH_DEBUG_CURRENT
char *Fast_itoa(int iv)
{
	static char buff[10];
	int m;

	int v = iv;
	if(v < 0) v = -v;
	char *bp = buff + 9;
	*bp-- = 0;		// trailing null

	for(int i=0;i<8;i++)		// guard against overflow
	{
		m = v%10;
		*bp-- = m + '0';
		v /= 10;
		if(v == 0)
		{
			if(iv < 0)
			{
				*bp-- = '-';	// Minus sign
			}
			return (bp + 1);
		}
	}
	*bp = '*';	// overflow
	return bp;
}
//-------------------------------------------------------------------------
extern TestState tState;
void MH_print_current_details(void)
{
	int tot=0;

    Board_UARTPutChar(':');
	for(int i=0;i<ADC_CURR_IX_MAX;i++)
	{
		int v = ADC_curr_t[i];
		tot+=v;
        Board_UARTPutSTR(Fast_itoa(v));
        Board_UARTPutChar(',');
	}
    Board_UARTPutSTR(Fast_itoa(tot));
    Board_UARTPutSTR("\r\n");
}
#endif

// These tables to "de-spike" values. MHH:22/03/2019
uint16_t ADC_prev_t[ADC_CHAN_MAX];
uint16_t ADC_last_t[ADC_CHAN_MAX];

//uint16_t current_last_raw;
//uint16_t current_prev_raw;
typedef enum
{
    adc_sr_test_idle = 0,
    adc_sr_wait_start,
	adc_sr_testing_state,
	adc_sr_testing_current,
	adc_sr_test_finished
} ADC_SlipringTestState;
ADC_SlipringTestState ADC_slipring_test_state;

int ADC_rpm_count;
int ADC_usecs_per_sector;
uint32_t Prop_read_usecs(void);
extern uint32_t Prop_int_usecs;
extern int Prop_int_rpm;

/*
#define SLIPRING_TEST_STATE		0
#define SLIPRING_TEST_FINE		1
#define SLIPRING_TEST_COARSE	2
*/
//#define MH_SLIPRING_DATA
#ifdef MH_SLIPRING_DATA
int Slipring_test_type;
uint32_t Slipring_test_ticks;
#define ADC_SLIPRING_SECTORS	10
#define ADC_SLIPRING_REVS		2

int ADC_slipring_sector_tbl[ADC_SLIPRING_SECTORS];
BYTE Log_slipring_test_type;
BYTE Log_slipring_tbl[ADC_SLIPRING_SECTORS];
int Log_slipring_tbl_ix = -1;


int ADC_sample_count;

bool ADC_slipring_test_ini(int test_type)
{
	if(Prop_int_rpm < 1500)
	{
		return false;
	}
	ADC_rpm_count = -1;
	ADC_sample_count = 0;
	for(int i=0;i<ADC_SLIPRING_SECTORS;i++) ADC_slipring_sector_tbl[i] = 0;
	ADC_usecs_per_sector = Prop_int_usecs/ADC_SLIPRING_SECTORS;
	Slipring_test_type = test_type;
#ifdef MH_SLIPRING_STATE_LOGIC
	if(Slipring_test_type == SLIPRING_TEST_STATE)
	{
		ADC_slipring_test_state = adc_sr_wait_start;
		setOutput(testCoarseFine);
	}
	else
#endif
	{
		ADC_slipring_test_state = adc_sr_testing_current;
	}
	return true;
}
//----------------------------------------------------------------------------
// This called if any drive pins about to be turned on.
#ifdef MH_SLIPRING_STATE_LOGIC
void ADC_cancel_any_slipring_state_test(void)
{
	if(ADC_slipring_test_state == adc_sr_wait_start || ADC_slipring_test_state == adc_sr_testing_state)
	{
		ADC_slipring_test_state = adc_sr_test_idle;
		setOutput(0);
	}
}
//----------------------------------------------------------------------------
void ADC_slipring_state_add_rawval(uint16_t rawval)
{
	if(ADC_slipring_test_state == adc_sr_testing_state)
	{
		uint32_t usecs = Prop_read_usecs();
	    int sector = usecs / ADC_usecs_per_sector;
	    sector = MIN(sector,(ADC_SLIPRING_SECTORS-1));
		ADC_slipring_sector_tbl[sector] += rawval;		// Note: This could be filtered
		ADC_sample_count++;
	}
}
#endif
void ADC_slipring_current_add_rawval(uint16_t rawval)
{
	if(ADC_slipring_test_state == adc_sr_testing_current)
	{
		uint32_t usecs = Prop_read_usecs();
	    int sector = usecs / ADC_usecs_per_sector;
//	    sector = MIN(sector,(ADC_SLIPRING_SECTORS-1));
	    sector %= ADC_SLIPRING_SECTORS;
		ADC_slipring_sector_tbl[sector] += rawval;		// Note: This could be filtered
		ADC_sample_count++;
	}
}
//---------------------------------------------------------------------------------------------
#define SLIPRING_MIN_STATE_VAL		40		// Arbitrary. May need to change.
//#define SLIPRING_MIN_STATE_VAL		55		// Arbitrary. May need to change.
void ADC_slipring_test_finished(void)
{
/*
	if(ADC_slipring_test_state == adc_sr_testing_state)
	{
		setOutput(0);
	}
	*/
	ADC_slipring_test_state = adc_sr_test_idle;
	int divisor = 180;
	switch(Slipring_test_type)
	{
	case SLIPRING_TEST_STATE:
		setOutput(0);
		break;

	case SLIPRING_TEST_CURRENT:
		divisor = 100;
		break;
	}
	Log_slipring_test_type = Slipring_test_type;
	int min_val = 128;
	for(int i=0;i<ADC_SLIPRING_SECTORS;i++)
	{
		int v = ADC_slipring_sector_tbl[i]/(divisor * ADC_SLIPRING_REVS);
		v = MIN(v,127);		// Will use other bit to identify slipring test type
		Log_slipring_tbl[i] = v;
		min_val = MIN(min_val,v);
	}
	if(min_val < 5)		// Arbitrary value, may need to adjust
	{
		setStateBits (200,S_ERROR_SLIPRING);
	}

#ifdef MH_SLIPRING_STATE_LOGIC
	if(Log_slipring_test_type == SLIPRING_TEST_STATE)
	{
		if(min_val >= SLIPRING_MIN_STATE_VAL)		// Ignore if all sectors look OK.
		{
			return;
		}
	}
#endif
	Log_slipring_tbl_ix = 0;
//#define MH_DISPLAY_SLIPRING_DATA
#ifdef MH_DISPLAY_SLIPRING_DATA
	if(Slipring_test_type != 0)
	{
		PRINTF("\r\n");
		PRINTF("SC:%d\r\n",ADC_sample_count);
		for(int i=0;i<10;i++)
		{
			PRINTF("%d\r\n",ADC_slipring_sector_tbl[i]);
		}
	}
#endif

}
#ifdef MH_SLIPRING_STATE_LOGIC
bool ADC_testing_slipring_idle(void)
{
	return (ADC_slipring_test_state !=  adc_sr_testing_state);
}
#endif

//#ifdef MH_SLIPRING_STATE_LOGIC
void ADC_check_state_test(void)	// Called from Prop interrupt
{
	if(ADC_slipring_test_state == adc_sr_test_idle)
	{
		if(scaledValue (A_MOTOR_CURRENT) >= 1000)
		{
			if(TimeInTicks - Slipring_test_ticks >= 50)
			{
				if(ADC_slipring_test_ini(SLIPRING_TEST_CURRENT))
				{
					Slipring_test_ticks = TimeInTicks;
				}
			}
		}
		return;		// Normal path
	}
	if(ADC_slipring_test_state == adc_sr_wait_start)
	{
//		setOutput(testCoarseFine);	// Note: Test may not work if at Coarse stop
		ADC_rpm_count = 0;
		ADC_slipring_test_state = adc_sr_testing_state;
		return;
	}
	if(ADC_rpm_count++ >= ADC_SLIPRING_REVS)
	{
//		ADC_slipring_test_state = adc_sr_test_finished;
		ADC_slipring_test_finished();
		return;
	}
	if(ADC_slipring_test_state == adc_sr_testing_state)
	{
		if(scaledValue (A_MOTOR_CURRENT) >= 20)
		{
			ADC_slipring_test_state = adc_sr_test_idle;
			setOutput(0);
		}
	}
}
//#endif
//-------------------------------------------------------------------------
#ifdef MH_SLIPRING_DATA
uint8_t ADC_get_slipring_log_data(void)
{
	if(Log_slipring_tbl_ix < 0)		// normal path
	{
		return 0;
	}
	if(Log_slipring_tbl_ix >= ADC_SLIPRING_SECTORS)
	{
		Log_slipring_tbl_ix = -1;
		resetStateBits(200,S_ERROR_SLIPRING);
		return 0;
	}
#ifdef MH_SLIPRING_STATE_LOGIC
	if(Log_slipring_test_type == SLIPRING_TEST_STATE)
	{
		if(scaledValue (A_MOTOR_CURRENT) > 20)
		{
			Log_slipring_tbl_ix = -1;
			return 0;

		}
	}
#endif

	uint8_t b = Log_slipring_tbl[Log_slipring_tbl_ix++];
	b = (b/10) * 10;			// Reduce log usage by taking to closest 10.
	b |= (Log_slipring_test_type << 7);		// Identify test type
	return b;
}
#endif
#endif
//-------------------------------------------------------------------------
//uint16_t ADC_cal_raw_current;
//uint16_t MH_raw_current[32];
//int MH_raw_current_ix;

#define ADC_RAW_SLIDER_MAX		32
uint16_t ADC_raw_slider_control[ADC_RAW_SLIDER_MAX];
uint16_t ADC_raw_slider_ix;

void ADC_IRQHandler(void)
{
	uint8_t chan;
	uint16_t dataADC,ChanMaxCnt;
//	uint16_t rawval;

	/* Interrupt mode: Call the stream interrupt handler */
	NVIC_DisableIRQ(ADC_IRQn);
	ADC.cnt++;
	for(chan=0;chan<ADC_CHAN_MAX;chan++)
	{
		if(Chip_ADC_ReadValue(LPC_ADC,ADC_CH0+chan, &dataADC) == ERROR)
		{
			//			uint32_t adcval = LPC_ADC->DR[chan];
			ADC.ErrorCnt++;		// Just keep track in case of interest
			adc_bad[chan]++;
			continue;

		}
		adc_good[chan]++;
		if(chan == ADC_SLIDER_CONTROL)
		{
			ADC_raw_slider_ix &= (ADC_RAW_SLIDER_MAX - 1);
			ADC_raw_slider_control[ADC_raw_slider_ix++] = dataADC;
		}


		if(chan != ADC_SLIDER_CONTROL)
		{
			if(dataADC >= 4000) continue;		// MHH:19/07/2019
		}

#define MH_NO_CURRENT_SPIKE
#ifdef MH_NO_CURRENT_SPIKE
		uint16_t last_raw = ADC_last_t[chan];
		uint16_t prev_raw = ADC_prev_t[chan];
		if((last_raw > dataADC + 50) && (last_raw > prev_raw + 50))	// Is this an isolated spike?
		{
			//					Curr_bad_count++;
			ADC_last_t[chan] = dataADC;
			continue;
		}
		else
		{
			ADC_prev_t[chan] = last_raw;
			ADC_last_t[chan] = dataADC;
			dataADC = last_raw;
		}

#endif
		ChanMaxCnt = ADC_ChanMaxCnt[chan];
		if(chan == ADC_MOTOR_STATE)		// Special case, need to get value quickly.
		{
			ADC.RawValue[chan] = (ADC.RawValue[chan] * ADC_STATE_FILTER + (dataADC * (100 - ADC_STATE_FILTER)))/100;
			//					ADC.RawValue[chan] = (ADC.RawValue[chan] * ADC_FILTER_VAL + (dataADC * (100 - ADC_FILTER_VAL)))/100;
			ADC_UpdateAC200_RawValues(chan);
#ifdef MH_SLIPRING_STATE_LOGIC
			ADC_slipring_state_add_rawval(rawval);
#endif
		}
		else
		{
#ifdef MH_SLIPRING_DATA
			if(chan == ADC_MOTOR_CURRENT)
			{
				//						ADC_cal_raw_current = rawval;	// Use this value when calibrating
				ADC_slipring_current_add_rawval(rawval);
			}
#endif
			ADC.ChanCnt[chan]++;
			ADC.TotVal[chan] += dataADC;
			if(ADC.ChanCnt[chan] >= ChanMaxCnt) {
				// Weight result = 90% previous plus 10% of average of current batch.

				ADC.RawValue[chan] = (ADC.RawValue[chan] * ADC_FILTER_VAL + (ADC.TotVal[chan] * (100 - ADC_FILTER_VAL)))/100;
//				ADC.RawValue[chan] = ADC.TotVal[chan];

				ADC.ChanCnt[chan] = 0;
				ADC.TotVal[chan]  = 0;
				ADC_UpdateAC200_RawValues(chan);
			}
		}
	}
#ifdef MH_INT_CURRENT
	MH_check_if_interrupt(ADC_SAMPLE_MULTIPLE);		// In ac210_rpm.c
#endif

	NVIC_EnableIRQ(ADC_IRQn);
// Not sure that we need to reset these flags here - could try without.
//	Chip_ADC_Int_SetChannelCmd(LPC_ADC,ADC_CH3, ENABLE);
}
