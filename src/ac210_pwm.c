/*
 * pwm.c - attempt a PWM signal.
 *
 *  Created on: 30/04/2015
 *      Author: Murray
 */
//#include "chip.h"
//#include "ac210_timer.h"
#include "ac210_global.h"
//#include "LPC17xx.h"

//============================================================================
// Following definition copied from "LPC17xx.h"
/*------------- Pulse-Width Modulation (PWM) ---------------------------------*/
typedef struct
{
  __IO uint32_t IR;
  __IO uint32_t TCR;
  __IO uint32_t TC;
  __IO uint32_t PR;
  __IO uint32_t PC;
  __IO uint32_t MCR;
  __IO uint32_t MR0;
  __IO uint32_t MR1;
  __IO uint32_t MR2;
  __IO uint32_t MR3;
  __IO uint32_t CCR;
  __I  uint32_t CR0;
  __I  uint32_t CR1;
  __I  uint32_t CR2;
  __I  uint32_t CR3;
       uint32_t RESERVED0;
  __IO uint32_t MR4;
  __IO uint32_t MR5;
  __IO uint32_t MR6;
  __IO uint32_t PCR;
  __IO uint32_t LER;
       uint32_t RESERVED1[7];
  __IO uint32_t CTCR;
} LPC_PWM_TypeDef;

#define LPC_PWM1              ((LPC_PWM_TypeDef       *) LPC_PWM1_BASE     )
//================================================================================
#define LPC_SC				LPC_SYSCTL
//#define PCLKSEL0			PCLKSEL[0]
//#define PINSEL4			PINSEL[4]
#define	LPC_PINCON			LPC_IOCON
#define DBG					printf

#define PCPWM1 (1 << 6)
#define PCLK_PWM1_BY8 ((1 << 13)|(1 << 12))
#define PCLK_PWM1_BY4 ~(PCLK_PWM1_BY8)
#define PCLK_PWM1 ((0 << 13)|(1 << 12))
#define PCLK_PWM1_BY2 ((1 << 13)|(0 << 12))
#define LER0_EN 1 << 0
#define LER1_EN 1 << 1
#define LER2_EN 1 << 2
#define LER3_EN 1 << 3
#define LER4_EN 1 << 4
#define LER5_EN 1 << 5
#define LER6_EN 1 << 6
#define PWMENA1 1 << 9
#define PWMENA2 1 << 10
#define PWMENA3 1 << 11
#define PWMENA4 1 << 12
#define PWMENA5 1 << 13
#define PWMENA6 1 << 14
#define TCR_CNT_EN 0x00000001
#define TCR_RESET 0x00000002
#define TCR_PWM_EN 0x00000008
//----------------------------------------------------------------------------------
// Note: The PWM routines have been coded so that PWM channels 1 to 3 are set up identically for each
// change of pulse width, however, only one channel is enabled at a time (when motor is being controlled by PWM).
// The PWM channels used are:
//	Channel	Pin		Pin number	J2 pin	Drive
//	PWM1.1	P2.0		75		14		L3
//	PWM1.2	P2.1		74		13		L2
//	PWM1.3	P2.2		73		12		L1

static uint32_t PWM_period_us;
static volatile uint32_t *pPWM_MR[4];
void PWM_SetPeriod_us(uint32_t period_us)
{
	PWM_period_us = period_us;		// will use for pulse width calcs.

 // set PWM cycle
	LPC_PWM1->MR0 = period_us;
// Set PulseWidth to 0 usecs
	LPC_PWM1->MR1 = 0;
	LPC_PWM1->MR2 = 0;
	LPC_PWM1->MR3 = 0;

//Load Shadow register content
	LPC_PWM1->LER = LER0_EN | LER1_EN | LER2_EN| LER3_EN;
	LPC_PWM1->PCR = PWMENA1|PWMENA2|PWMENA3;	// Enable channels 1,2,3
 //Enable PWM Timer
	LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN;
}
//----------------------------------------------------------------------------------
static uint32_t PWM_pulse_width_us;
static bool PWM_active_channel[4];
void PWM_SetPulseWidth_us(int pwm_pulse_us)
{
	uint32_t pulse_width;
// If not set to zero then if cycle has not completed then
// the new settings are ignored.
	LPC_PWM1->LER = 0;
	PWM_pulse_width_us = pwm_pulse_us;
	for(int i=1;i<=3;i++)	// Set active channels to pulse_width, others to zero
	{
		pulse_width = 0;
		if(PWM_active_channel[i])
		{
			pulse_width = pwm_pulse_us;
		}
		*pPWM_MR[i] = pulse_width;
	}
	LPC_PWM1->LER = LER1_EN | LER2_EN | LER3_EN;
}
//----------------------------------------------------------------------------------
//static uint32_t map_pwm_enable[]={0,PWMENA1,PWMENA2,PWMENA3,};
// Using PWMENAn did not work as expected. If not on, pin could still be on (though maybe not pulsing).
// Now setting corresponding match register to zero instead.

void PWM_Enable(int n)
{
	PWM_active_channel[n] = true;
	PWM_SetPulseWidth_us(PWM_pulse_width_us);
}
//---------------------------------------------------------------------------------
void PWM_Disable(int n)
{
	if(n != 0)
	{
		PWM_active_channel[n] = false;
	}
	else
	{
		PWM_active_channel[1] = false;
		PWM_active_channel[2] = false;
		PWM_active_channel[3] = false;
	}
	PWM_SetPulseWidth_us(PWM_pulse_width_us);
}
//----------------------------------------------------------------------------------
void PWM_SetFrequency(uint32_t pwm_freq)
{
	uint32_t period_us = ((1000000+(pwm_freq/2))/pwm_freq);	// round to closest value
	PWM_SetPeriod_us(period_us);
}
//----------------------------------------------------------------------------------
#define PWM_FREQUENCY	21000
//#define PWM_PULSE_WIDTH_US	((1000000+(PWM_FREQUENCY/2))/PWM_FREQUENCY)
void PWM_SetPulsePercent(int percent)
{
	int pulsewidth_us;
//	DPRINTF("pwm_pct=%d\r\n",percent);
	if(percent>100) percent = 100;
	pulsewidth_us = ((PWM_period_us * percent) + 50)/100;
	PWM_SetPulseWidth_us(pulsewidth_us);
}
//----------------------------------------------------------------------------------
// Note that the start_one_shot_ms is a value in ms to give total power. It is done
// by setting a timer interrupt which turns PWM to normal after the "one shot"
// Leave the one shot logic for now.
void p_SetDrivePWM(WORD AC200_pwm,WORD start_one_shot_ms)
{
	static int last_percent=-1;
	int percent;		// Convert from range of 0 to 255 to 0 to 100
	percent = (AC200_pwm * 100 + 254) / 255;
	if(percent != last_percent) {
		last_percent = percent;
//		printf("p_SetDrivePWM:%d pct\n\r",percent);
//		DPRINTF("p_SetDrivePWM:%d pct\n\r",percent);
		if(percent > 100)
		{
			last_percent = percent;		// Breakpoint here.
		}
		PWM_SetPulsePercent(percent);
	}
}
//----------------------------------------------------------------------------------
void p_SetCycleRate(BYTE PWM_CycleRate)
{
	static BYTE save_PWM_CycleRate=255;
	static const uint32_t pwm_freq_tbl[]=
	{
			53600,		// 0 best for Maxon
			31250,		// 1 (to match AC200)
			20830,		// 2 (to match AC200)
	};
	if(PWM_CycleRate == save_PWM_CycleRate) return;	// normal case
	if(PWM_CycleRate > 2) Abort(AC210_SRC_PWM+10,"p_SetCycleRate: > 2");
	save_PWM_CycleRate = PWM_CycleRate;
	uint32_t pwm_freq = pwm_freq_tbl[PWM_CycleRate];
	PWM_SetPulsePercent(0);	// Just to be sure
//	printf("p_SetCycleRate: Setting frequency to: %d\r\n",pwm_freq);	// MHH:09/09/2023
	PWM_SetFrequency(pwm_freq);
}
//----------------------------------------------------------------------------------
void AC210_PWM_Init(void)
{
	//enable PWM1 Power
	LPC_SC ->PCON |= PCPWM1;
 //PWM peripheral clk = PCLK
	LPC_SC ->PCLKSEL[0] &= (PCLK_PWM1_BY4);

//Pin select done in board.c at board ini

    uint32_t PCLK = SystemCoreClock / 4;

    uint32_t prescale = PCLK / 1000000; // default to 1 MHz
    LPC_PWM1->PR = prescale - 1;
	//reset on MR0
	LPC_PWM1->MCR = 1 << 1;

	pPWM_MR[1] = &(LPC_PWM1->MR1);		// Save pointers to match registers so we can access by index later
	pPWM_MR[2] = &(LPC_PWM1->MR2);
	pPWM_MR[3] = &(LPC_PWM1->MR3);

#ifdef RPM_TEST
	int rpm,period_us;

//	PWM_SetPeriod_us(10000);	// 100 Hz = 6000 RPM
//	PWM_SetPulseWidth_us(100);
	for(rpm=6000;rpm>0;rpm-=100) {
		printf("rpm=%d\n\r",rpm);
		period_us = (60000000 + rpm/2)/rpm;
		PWM_SetPeriod_us(period_us);	// 100 Hz = 6000 RPM
		wait_ms(1000);
	}
#else
	int percent = 0;
//	printf("AC210_PWM_Init:Setting frequency to %d\n\r",PWM_FREQUENCY);
//	PWM_SetPeriod_us(PWM_PULSE_WIDTH_US);
	PWM_SetFrequency(PWM_FREQUENCY);		// Start at compatible value for AC200
//	printf("AC210_PWM_Init:Setting pulsewidth to %d%%\n\r",percent);
	PWM_SetPulsePercent(percent);
	PWM_Disable(0);		// Disable all PWM channels
#endif
}
