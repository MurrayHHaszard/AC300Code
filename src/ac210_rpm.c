/*
 * rpm.c, derived from port_rpm.c
 *
 *  Created on: 16/05/2015
 *      Author: Murray
 */

//#include <cr_section_macros.h>
//#include <string.h>
#include <stdlib.h>

#include "ac210_global.h"
#include "ac210_timer.h"

#include "param.h"

//---------------------------------------------------------------------------------------------
// from pinint.c
//#define GPIO_INTERRUPT_PIN     17	/* GPIO pin number mapped to interrupt */
#define GPIO_INTERRUPT_PIN     PINMASK(AC210_J1_MAG_INT_PIND)	/* GPIO pin number mapped to interrupt */
#define GPIO_INTERRUPT_PORT    PINPORT(AC210_J1_MAG_INT_PIND)	/* GPIO port number mapped to interrupt */

/* On the LPC1769, the GPIO interrupts share the EINT3 vector. */
#define GPIO_IRQ_HANDLER  			EINT3_IRQHandler/* GPIO interrupt IRQ function name */
#define GPIO_INTERRUPT_NVIC_NAME    EINT3_IRQn	/* GPIO interrupt NVIC interrupt name */



//int Prop_int_prev=-1;
//int Prop_int_dups;
//int Prop_int_rpm;

//int Prop_int_rpm_prev;	// MHH:07/05/2024
int Prop_rpm_delta_per_tick;
uint32_t Prop_rpm_delta_usecs;
int Prop_rpm_delta_prev;
int Prop_rpm_delta;


//int Prop_filter_rpm;
uint32_t Prop_int_usecs;
uint32_t Prop_int_usecs_prev;
int Prop_int_count;
int Prop_int_last_count;
int Prop_int_count_1;
extern int Board_pin_read(uint8_t port,uint8_t pin);
extern WORD AT_flag_rpm_stop;	// defined in comms.c

TIMER_t Prop_int_timer;
void AC210_InitMagTimerPin(void)
{

/* Configure GPIO interrupt pin as input */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_INTERRUPT_PORT, GPIO_INTERRUPT_PIN);

#ifdef MH_XXX	// MHH:13/05/2024
	Prop_int_prev = Board_pin_read(PIN_DEF2(AC210_J1_MAG_INT_PIND));
#endif
/* Configure the GPIO interrupt */
	Chip_GPIOINT_SetIntFalling(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN);
//	Chip_GPIOINT_SetIntRising(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN);

/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(GPIO_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(GPIO_INTERRUPT_NVIC_NAME);

	Timer_start(&Prop_int_timer);
}
//-----------------------------------------------------------------------------------------------------------------
void Prop_RPM_ISR(void);
//int RPM_count;
//int RPM;
void GPIO_IRQ_HANDLER(void)
{
//	uint32_t pins_rising =  Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, PORT_0);
//	uint32_t pins_falling = Chip_GPIOINT_GetStatusFalling(LPC_GPIOINT, PORT_0);
	Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN);
//	RPM_count++;
#ifdef MH_XXX	// MHH:13/05/2024
	if(TimeInSeconds > 0)
	{
		RPM = (60 * RPM_count)/TimeInSeconds;
	}
#endif
	Prop_RPM_ISR();

}
//-----------------------------------------------------------------------------------------------------------------
//#define PROP_MAV_MAX	10
#ifdef MH_RPM_MAV
#define PROP_MAV_MAX	4	// MHH:16/12/2020
int Prop_mav_ix;
int Prop_mav_t[PROP_MAV_MAX];
#endif
int mh_int_count1;
int mh_int_count2;
int mh_int_count3;

int Engine_rpm_delta_per_tick_1dec;
int Engine_rpm_1dec;
int Engine_rpm_1dec_prev;
int Engine_rpm_1dec_prev1;
int Engine_rpm_delta_1dec;
int Prop_rpm_1dec;
int Prop_rpm_1dec_prev;
int Prop_rpm_1dec_prev1;
int Prop_rpm_delta_1dec;	// MHH:18/11/2024
int Prop_rpm_delta_per_tick_1dec;
#define PU_MAX		32
struct PROP_DEBUG
{
	uint32_t Prop_usecs;
	uint16_t Engine_RPM;
	int16_t Engine_delta;
	uint16_t Prop_rpm;
	int16_t Prop_delta;
} PDebug_t[PU_MAX],*pPDebug;
int PU_ix;
// Note: Changed interrupt initialise so ISR is only called on a falling edge. MHH:12/11/2022
void Prop_RPM_ISR(void)
{
    mh_int_count1++;
	int s1 = Board_pin_read(PIN_DEF2(AC210_J1_MAG_INT_PIND));

#ifdef MH_DEBOUNCE
	if(s1 == Prop_int_prev)  // De-bounce
    {
        Prop_int_dups++;
        return;
    }
    Prop_int_prev = s1;
#endif

    if(s1 != 0)
    {
    	mh_int_count2++;
    	return;
    }

    mh_int_count3++;
    Prop_int_usecs_prev = Prop_int_usecs;
    Prop_int_usecs = Timer_read_us(&Prop_int_timer);
    Timer_reset(&Prop_int_timer);

    Engine_rpm_1dec_prev1 = Engine_rpm_1dec_prev;	// MHH:21/05/2023
    Engine_rpm_1dec_prev = Engine_rpm_1dec;

    Prop_rpm_1dec_prev1 = Prop_rpm_1dec_prev;
    Prop_rpm_1dec_prev = Prop_rpm_1dec;

//    Prop_int_rpm =  (60000000)/Prop_int_usecs;	// No implied decimal place

    uint32_t prop_int_usecs_2revs = Prop_int_usecs + Prop_int_usecs_prev;

    Prop_rpm_1dec = (1200000000)/prop_int_usecs_2revs;		// 1 implied decimal place
    Prop_rpm_delta_1dec = (Prop_rpm_1dec - Prop_rpm_1dec_prev1)/2;	// Average change over the last 2 revs

    int scaling_factor_percent = getParameter(SF_MAG_SPEED);
    Engine_rpm_1dec  = (Prop_rpm_1dec * scaling_factor_percent + 50) / 100;
//    int engine_rpm_delta_1dec = Engine_rpm_1dec - Engine_rpm_1dec_prev;	// Change in rpm since last revolution of prop
    Engine_rpm_delta_1dec = (Engine_rpm_1dec - Engine_rpm_1dec_prev1)/2;	// Average change over the last 2 revs


    PU_ix &= (PU_MAX -1);		// Only works if multiple of 2
    pPDebug = PDebug_t + PU_ix;

    pPDebug->Prop_usecs = Prop_int_usecs;
    pPDebug->Engine_RPM = Engine_rpm_1dec;
    pPDebug->Engine_delta = Engine_rpm_delta_1dec;
    pPDebug->Prop_rpm   = Prop_rpm_1dec;
    pPDebug->Prop_delta = Prop_rpm_delta_1dec;

    PU_ix++;

    /*
     *  Hope to use Prop_rpm_delta_per_tick as the 'D' component of PID control.
     */


    int prop_int_usecs = (int)prop_int_usecs_2revs;
    Prop_rpm_delta_per_tick_1dec = (Prop_rpm_delta_1dec * 40000) / prop_int_usecs;	// Because 40,000 usecs per 2 x tick
    Engine_rpm_delta_per_tick_1dec = (Engine_rpm_delta_1dec * 40000) / prop_int_usecs;	// Because 40,000 usecs per 2 x tick

    Prop_int_count++;
#ifdef MH_RPM_MAV
    Prop_mav_ix %= PROP_MAV_MAX;
    Prop_mav_t[Prop_mav_ix++] = Prop_int_rpm;
#endif
#ifdef MH_SLIPRING_DATA
    ADC_check_state_test();
#endif
}
uint32_t Prop_read_usecs(void)
{
	return Timer_read_us(&Prop_int_timer);
}
//----------------------------------------------------------------------------------------
// Called from void SysTick_Handler() to make sure prop is still turning. If we are getting zero readings in 0.5 secs, then assume stopped.
void MagTimer_Check_RPM(void)
{
//	PRINTF("Prop_int_count:%d\r\n",Prop_int_count);
	if(Prop_int_count == 0)
	{
#ifdef MH_RPM_MAV
		for(int i=0;i<PROP_MAV_MAX;i++) Prop_mav_t[i]=0;	// Zero MAV table
#endif
//		Prop_int_rpm = 0;
//		Prop_int_rpm_prev = 0;
		Engine_rpm_1dec = 0;		// MHH:13/05/2024
		Prop_rpm_1dec = 0;			// MHH:18/05/2024
		Prop_rpm_1dec_prev = 0;
		Prop_rpm_1dec_prev1 = 0;

	}
	Prop_int_count = 0;
}
//----------------------------------------------------------------------------------------
int Prop_rpm;
int MagTimer_count;
uint32_t MagTimer_Get_RPM2(void)
{
	MagTimer_count++;
	if(AT_flag_rpm_stop)
	{
		return 0;
	}

	Prop_rpm = (Engine_rpm_1dec + 5)/10;	// MHH:13/05/2024

#ifdef MH_XXX	// MHH:13/05/2024

//	if(Prop_int_rpm < 600*RPMFACTOR)	// MAV takes too long at low RPM. Note implied decimal place (actually 600 RPM).
	if(Prop_int_rpm < 600)	// MAV takes too long at low RPM. Note implied decimal place (actually 600 RPM).
	{
		Prop_rpm = Prop_int_rpm;
	}
	else
	{
		int tot=0;
		for(int i=0;i<PROP_MAV_MAX;i++) tot += Prop_mav_t[i];	// Sum MAV table
		int mav = (tot + PROP_MAV_MAX/2)/PROP_MAV_MAX;
		Prop_rpm = mav;
	}
#endif
	return Prop_rpm;
}
//----------------------------------------------------------------------------------------
/**
 * @brief	Handle interrupt from GPIO pin or GPIO pin mapped to PININT
 * @return	Nothing
 */
//--------------------------------------------------------------------------------------------
struct
{
//#define MH_DEBUG_RPM
#ifdef MH_DEBUG_RPM
	uint32_t tbl[64];
	uint32_t ix;
#endif
	uint32_t high;
	uint32_t low;
} DebugRPM;


//----------------------------------------------------------------------------------------
// Separate routine so we only calculate when interested
extern WORD AT_flag_rpm_stop;	// defined in comms.c

WORD p_GetAC200_magRPM(WORD ScalingFactorPercent)
{
//	static int cnt;
//	static uint32_t lastRPM;
	uint32_t rpm1,rpm2;
	rpm1 = MagTimer_Get_RPM2();
	rpm2 = rpm1;
	if(ac2_test_flag != AC2_TEST_ON)	// No scale for AC2_TEST
	{
		if(ScalingFactorPercent) {
			rpm2 = (rpm1 * ScalingFactorPercent + 50) / 100;
		}
	}
	else	// Debug
	{
//		PRINTF("Prop_int_rpm=%d\r\n",Prop_int_rpm);
	}
//#define MH_SHOW_RPM
#ifdef MH_SHOW_RPM			// Could make this dynamic depending on debug setting.
	if(++cnt%50 == 0) {
		printf("p_GetAC200_magRPM:rpm1:%d,rpm2:%d,speed_changed:%d\n\r",rpm1,rpm2,speed_changed);
	}
#endif
	return (WORD)rpm2;	// NO dec place.
}
