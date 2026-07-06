/*
===============================================================================
 Name        : ac210.c
 Author      : MHH
 Version     :
 Copyright   : Airmaster Propellers Limited: 2016
 Description : main definition
===============================================================================
*/

#include "board.h"

// TODO: insert other include files here
#include "ac210_global.h"
#include "ac210_timer.h"
#include "ac210_pwm.h"
#include "ac210_adc.h"
#include "ac210_ssp.h"
#include "global.h"
#include "diags.h"

#include "analog.h"

const char *AC210_name = "AC300_V5";

void Board_LED_Toggle(uint8_t LEDNumber);

void Board_ShowSwitches(void);	// MHH:
bool Board_Test_Fine(void);
bool Board_Test_Coarse(void);
void Board_TestMotorState(void);

// TODO: insert other definitions and declarations here
// PRINTF Feather_mode
//-----------------------------------------------------------------------------
void DisplayStateReadingDelay(int delay_secs)
{
	int secs;
	int state_rawval;
	int current_rawval;
	int supply_rawval;
	int dimmer_rawval;

	for(secs=0;secs<delay_secs;secs++) {
		state_rawval = ADC_GetRawVal(ADC_MOTOR_STATE);
		current_rawval = ADC_GetRawVal(ADC_MOTOR_CURRENT);
		supply_rawval = ADC_GetRawVal(ADC_SUPPLY);
		dimmer_rawval = ADC_GetRawVal(ADC_DIMMER);
		DPRINTF("ADC.cnt:%d, state:%d, current:%d, supply:%d, dimmer:%d\n\r",ADC.cnt,state_rawval,current_rawval,supply_rawval,dimmer_rawval);
		ADC.cnt = 0;
		wait_ms(1000);
	}
}
//-----------------------------------------------------------------------------
int ReturnToContinue(void)
{
//  char str1[20];
	int c;

   printf("\r\n<RETURN> to continue");
   AC210_uart3_wait();		// Wait for buffer to be output
   while(true)
   {
	   c = Board_UARTGetChar();
	   if(c == 10 || c == 13) break;
	   if(c == 'x' || c == 'X')
	   {
		   c = 'X';
		   break;
	   }
   }
//   scanf("%s", str1);	// scanf does not seem to accept an empty field for type s
   puts("\r\n");
   return c;
}
//-----------------------------------------------------------------------------
/**
 * @brief	SysTick Interrupt Handler
 * @return	Nothing
 * @note	Systick interrupt handler updates the button status
 */
// Note: Called 100 times a second
volatile uint32_t sysTick;
volatile bool AC210_wait=true;
volatile uint32_t WaitFlags;
bool AC210_watchdog_active = false;
int32_t AC210_watchdog_count;	// Use our own watch dog so we can record reason for reset
uint32_t AC210_usecs_at_tick;
uint32_t AC210_idle_percent2d;
WORD AC210_watchdog_from;
uint16_t gVerbose;
#define FLAG_CHECK_POWER	1
#define FLAG_CHECK_RPM		(1<<1)
#define AC210_HERTZ			50
#define AC210_USECS_PER_TICK	(1000000/AC210_HERTZ)

//-------------------------------------------------------------------------
static void LedsAllOff(void)
{
	for(int i=1;i<=6;i++)
	{
		Board_LED_Set(i,false);
	}
}
//-------------------------------------------------------------------------
static void LedsAllRedOn(void)
{
	for(int i=1;i<=6;i++)
//	for(int i=2;i<=6;i+=2)
	{
		if(i & 1)
			Board_LED_Set(i,false);
		else
			Board_LED_Set(i,true);
	}
}
//-------------------------------------------------------------------------
static void LedsAllGreenOn(void)
{
	for(int i=1;i<=6;i++)
//	for(int i=2;i<=6;i+=2)
	{
		if((i & 1) == 0)
			Board_LED_Set(i,false);
		else
			Board_LED_Set(i,true);
	}
}
//-------------------------------------------------------------------------
static void LedsAllOn(void)
{
	for(int i=1;i<=6;i++)
	{
		Board_LED_Set(i,true);
	}
}
//-------------------------------------------------------------------------
void FlashAllLEDS(int tenths)
{
	int flashes=tenths*2;

	LedsAllOff();
	for(int f=0;f<flashes;f++)
	{
		for(int i=1;i<=6;i++)
		{
			Board_LED_Toggle(i);
		}
	    wait_ms(50);
	}
}
//-------------------------------------------------------------------------
void FlashRedLEDS(int tenths)
{
	int flashes=tenths*2;

	LedsAllOff();
	for(int f=0;f<flashes;f++)
	{
		for(int i=2;i<=6;i+=2)
		{
			Board_LED_Toggle(i);
		}
	    wait_ms(50);
	}
}
//-------------------------------------------------------------------------
// Note: Consider making equivalent for AC200
#define LED_WAIT_MS		5000		// 5 secs
void AC210_test_leds(void)
{
	AC210_watchdog_active = false;				// This could take a while

	PRINTF("Setting all LEDS to GREEN...\r\n");
	LedsAllGreenOn();
	wait_ms(LED_WAIT_MS);		// Wait

	PRINTF("Setting all LEDS to RED...\r\n");
	LedsAllRedOn();
	wait_ms(LED_WAIT_MS);		// Wait

	PRINTF("Setting all LEDS to ORANGE...\r\n");
	LedsAllOn();
	wait_ms(LED_WAIT_MS);		// Wait

	PRINTF("Turning all LEDS off...\r\n");
	LedsAllOff();
	wait_ms(LED_WAIT_MS);		// Wait

	PRINTF("End of test\r\n");
}
//-------------------------------------------------------------------------
void Abort(ULONG err_num,char far *msg)
{
	L2PRINTF("Abort:(%d):%s\n\r",err_num,msg);
//	DPRINTF("Abort:(%d):%s\n\r",err_num,msg);

// Could do something with LED0??

	int rv = DiagsError(err_num+100000);
	AC210_watchdog_active = false;	// Disable watchdog so it does not interfere

// Just put this test in while testing!!!

	if(rv == 0)
	{
		FlashAllLEDS(50);
		AC210_reboot();
//		NVIC_SystemReset();
	}
	FlashAllLEDS(1000);	// Need to know about this
}
//--------------------------------------------------------------------------------------------------------------
// Test if my watchdog overrides this...
int AC210_Force_Hardfault(void)
{
	int *ptr=0;
	int v;
	ptr+=1000000;
	v=*ptr;
	return v;
}
//--------------------------------------------------------------------------------------------------------------
uint8_t AC210_watchdog_reset_count;
void AC210_Watchdog_Reset(void)
{
//#define MH_DISABLE_WATCHDOG
#ifdef MH_DISABLE_WATCHDOG
	printf("\r\nAC210_Watchdog_Reset:DISABLED FOR TESTING!!!\r\n");
	AC210_watchdog_active = false;
	return;
#endif


	if(AC210_watchdog_reset_count++ == 0)
	{
		L2PRINTF("\r\nAC210_Watchdog_Reset:%05d\r\n",AC210_watchdog_from);
		wait_ms(100);	// MHH:04/08/2023
		if(AC210_watchdog_from != 0)	// MHH: 22/08/2018
		{
			DiagsError(AC210_watchdog_from+200000);
		}
	}
	FlashAllLEDS(50);		// Make yellow as well so do not confuse with AC200 open circuit error
//	FlashRedLEDS(50);
	AC210_reboot();
}
//--------------------------------------------------------------------------------------------------------------
void AC210_reboot(void)
{
//	for(;;);

// What do we do if in an Abort loop, eg if logging not working?
// Could test a pin (DIM pin or similar) and if active then put in loop and turn off watchdog.
// This would allow me to load debugger.

	NVIC_SystemReset();
}
//--------------------------------------------------------------------------------------------------------------
void AC210_watchIt(WORD from)	// called from main:watchIt()
{
	AC210_watchdog_active = true;
	AC210_watchdog_from = from;
	AC210_watchdog_count  = AC210_HERTZ * 2;	// 2 seconds
}
//--------------------------------------------------------------------------------------------------------------
void AC210_watchIt2(WORD from)	// called from main:watchIt2()
{
	AC210_watchdog_from = from;
}
//--------------------------------------------------------------------------------------------------------------
static int HardFault_count;
// If we don't have a hardfault handler then default behaviour is an infinite loop.
void HardFault_Handler(void)
{
	if(HardFault_count++ == 0)
	{
// Might benefit from adding more watchdog points
		printf("AC210_Hardfault_Handler:%05d\r\n",AC210_watchdog_from);

		for(int i=0;i<6;i++)
		{
			LedsAllOn();	// Yellow
			wait_ms(500);
			LedsAllRedOn();	// Red
			wait_ms(500);
		}
//		NVIC_DisableIRQ((IRQn_Type)-13);	// Cannot clear hardfault as has a negative interrupt number (-13)
//		NVIC_ClearPendingIRQ((IRQn_Type)(-13));
//		DiagsError(AC210_watchdog_from+300000);	// Doesn't work, probably because i2C interrupt driven.
	}
	AC210_reboot();
//	NVIC_SystemReset();
}
//--------------------------------------------------------------------------------------------------------------
void SysTick_Handler(void)
{
	sysTick++;		// Called 50 times per second
	if(AC210_watchdog_active)
	{
		if(--AC210_watchdog_count <= 0)
		{
			AC210_Watchdog_Reset();
		}
	}
	if(sysTick % 25 == 0) { 		// twice a second
		MagTimer_Check_RPM();		// make sure we are still getting readings from Mag sensor input.
	}
	if(sysTick % 50 == 0)
	{
		RTC_secs++;
	}

	AC210_wait = false;
	AC210_usecs_at_tick = us_ticker_read();	// May be useful understanding what % of CPU is idle
}
//-------------------------------------------------------------------------------------------------------------
// MHH:29/05/2025. Attempt to find CAN receive bug
void Wait_secs_no_watchdog(int secs)
{
	PRINTF("Waiting %d secs...",secs);
	AC210_watchdog_active = false;
	wait_us(secs*1000000);
	watchIt(WD_AC210+10);
//	PRINTF("Finished\r\n");
}
//--------------------------------------------------------------------------------------------------------------
//void AC210_CheckForCanMsg(void);
void ADC_CheckPower(void);
void RPM_Check(void);
// Use wait time to do any checks
void mainTimer (void);		// defined in main.c
void p_Wait(void)
{
	uint32_t working_usecs = us_ticker_read() - AC210_usecs_at_tick;
	uint32_t idle_usecs = AC210_USECS_PER_TICK - working_usecs;
	uint32_t idle_percent2d = (idle_usecs * 10000)/AC210_USECS_PER_TICK;
	AC210_idle_percent2d = (((AC210_idle_percent2d * 9) + idle_percent2d)+5)/10;	// filter


	if(AC210_watchdog_active == false)
	{
		 AC210_watchIt(WD_AC210);				// In case was disabled by any routine
	}

// By knowing how many idle usecs we have before next SysTick interrupt we can decide if we want to
// do anything while waiting. fState Feather_mode fState dState

#ifdef MH_OLD_ERASE
	if((sysTick % 100) == 0)
	{
//		float f_idle_percent = (float)(AC210_idle_percent2d)/100.0;
//		printf("p_Wait:Idle percent: %4.1f\r\n",f_idle_percent);
	}
	AC210_log_check_erase_sector();
#endif

	while(AC210_wait)
	{
#ifdef MH_DEBUG_RPM
		if(WaitFlags) {
			if(WaitFlags & FLAG_CHECK_POWER) {
				WaitFlags &= ~FLAG_CHECK_POWER;
				ADC_CheckPower();
			}
			if(WaitFlags & FLAG_CHECK_RPM) {
				WaitFlags &= ~FLAG_CHECK_RPM;
				RPM_Check();
			}
		}
#endif
	}
	AC210_wait = true;
	mainTimer();		// Set flag for AC200 main
}


//-----------------------------------------------------------------------------
void AC210_eeprom_I2C_Init(void);
void eeprom_Test(void);
void AC210_InitMagTimerPin(void);
uint32_t MagTimer_Get_RPM(void);
void AC210_BoardInit(void);
void AC210_J2_Enable(void);
void AC210_Aux_Serial_Init(void);
void AC210_Aux_UARTPutSTR(char *str);
void AC210_Aux_UARTPutChar(char ch);
void AC210_Test_Fine_Coarse(void);
//-----------------------------------------------------------------------------
void AC210_dummy_str(const unsigned char *str)
{
}
//-----------------------------------------------------------------------------
//#define MH_TEST_DRIVE
#ifdef MH_TEST_DRIVE
#define U1	0x2
#define U2	0x4
#define U3	0x8

#define L1  0x20
#define L2  0x40
#define L3	0x80

static void TestDrive(void)
{
	PWM_SetPulsePercent(90);
    while(true)
    {
//    	Test_Fine_Coarse();
// Note: Colours are from when I had a breadboard test with LEDS instead of using power PCB.
    	printf("U1 + L2 (Fine = Green + Red)\r\n");
    	p_ControlPort = U1 + L2;
    	p_SetDrivePins();
        wait_ms(1000);

//    	Board_Drive_U_On(1);
//      Board_Drive_L_On(2);
//    	Test_Fine_Coarse();
//      wait_ms(1000);

/*
        printf("Braking at 50%%\r\n");
        Board_Drive_U_Off(1);
        Board_Drive_L_On(1);
        wait_ms(500);
        Board_Drive_L_Disable();
*/
//  	Test_Fine_Coarse();
        printf("U2 + L1 (Coarse = Red + Green)\r\n");
    	p_ControlPort = U2 + L1;
    	p_SetDrivePins();
        wait_ms(1000);

//        Board_Drive_U_On(2);
//        Board_Drive_L_On(1);
//    	Test_Fine_Coarse();
//        wait_ms(1000);

/*
        printf("Braking at 50%%\r\n");
        Board_Drive_U_Off(2);
        Board_Drive_L_On(2);
        wait_ms(500);
        Board_Drive_L_Disable();
*/
//    	Test_Fine_Coarse();
        printf("U3 + L1 (Feather = Yellow + Green)\r\n");
    	p_ControlPort = U3 + L1;
    	p_SetDrivePins();
        wait_ms(1000);
//        Board_Drive_U_On(3);
//        Board_Drive_L_On(1);
    	p_ControlPort = 0;
    	p_SetDrivePins();
        wait_ms(1000);
//        Board_Drive_U_Off(3);
//        Board_Drive_L_Off(1);
    }
}
#endif
//------------------------------------------------------------------------------------------------
// Return usecs since last call
uint32_t AC210_usecs_elapsed(void)
{
	static uint32_t usecs_prev;
	uint32_t usecs = us_ticker_read();
	uint32_t usecs_elapsed = usecs - usecs_prev;
	usecs_prev = usecs;
	return usecs_elapsed;
}
//------------------------------------------------------------------------------------------------
uint32_t AC210_ms_elapsed(uint32_t *p_usecs_prev)
{
	uint32_t usecs_prev = *p_usecs_prev;
	uint32_t usecs = us_ticker_read();
	uint32_t usecs_elapsed = usecs - usecs_prev;
	uint32_t ms_elapsed = (usecs_elapsed+500)/1000;
	*p_usecs_prev = usecs;
	return ms_elapsed;
}
//------------------------------------------------------------------------------------------------
void AC210_ms_display(char *desc,uint32_t ms)
{
	char msg[20];
	sprintf(msg,"[%s:%d]\r\n",desc,ms);
	txDebug(msg);
}
// D:%d E:%d
//-----------------------------------------------------------------------------
//E=%d WriteStatsRec: Log_checkpoint: _flash_2: setOutput: ,"startTest:State_StartFastTest
// Set_tState:Drive:1900 cancelTest: setIdle: Set_cState:Hist4:C_BRAKE:'+' State_StartFastTest:
//cancelTest:reset:]set:Set_dState:'X':A:ERR Param Slider
void loadParameters(void);
int main(void) {

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
//    printf("Hello World!\r\n");
#endif
#endif
	AC210_BoardInit();

	SysTick_Config(SystemCoreClock / AC210_HERTZ);	// 50 Hz

	AC210_watchIt(0);	// MHH:21/08/2018

	AC210_SSP_Init();

	AC210_STATE_Init();

	AC210_DRIVE_Init();

//	AC210_Aux_Serial_Init();	// This is currently done in board.c as aux port is debug serial port
	AC210_InitMagTimerPin();
//    wait_ms(500);	// Test if this stops i2c hard fault...
    AC210_eeprom_I2C_Init();	// MHH:12/07/2023. Moved to before AC210_SerialInit.
//    AC210_RTC_init();
    AC210_PWM_Init();
    AC210_ADC_Init();

//    AC210_CAN_Init();			// Now done only if Aux_port CAN option

    AC210_J2_Enable();		// Enable J2

#ifdef MH_TEST_DRIVE
    TestDrive();
#endif

//#define MH_TEST_SLIDER_CONTROL
#ifdef MH_TEST_SLIDER_CONTROL
//    WORD v = 65535;
    AC210_watchdog_active = false;
    while(true)
    {
    	WORD v2 = scaledValue (A_SLIDER_CONTROL);
    	PRINTF("v2 = %d\r\n",v2)
    	wait_ms(500);
    }

#endif

//#define MH_TEST_LEDS_SWITCHES
#ifdef MH_TEST_LEDS_SWITCHES
	printf("Testing switches and LEDS\r\n");
//	for(int i=0;i<20;i++)
	while(true)
	{
    	Board_ShowSwitches();
    	for(int led=1;led<=6;led++)
    	{
    		Board_LED_Set(led, true);
    		wait_ms(500);		// wait half a sec
    	    Board_LED_Set(led, false);
    	}
	}
#endif

//    PRINTF("PCB version: 5%c\r\n",AC210_hardware_version);
    AC210_plog_init();		// MHH:08/08/2023

//    printf(">>Start:\r\n");
    L2PRINTF(">Start:\r\n");
    AC210_main();


//#define MH_TEST_AUX_SERIAL
#ifdef MH_TEST_AUX_SERIAL
	printf("Testing Auxiliary Serial Port...\r\n");
//	AC210_Aux_Serial_Init();
#define MH_TEST_AUX_2
#ifdef MH_TEST_AUX_2
	while(true)
	{
		AC210_Aux_UARTPutChar('A');
	}
#endif
	AC210_Aux_UARTPutSTR("Hello World from Auxiliary Port\r\n");
	while(true);		// Should test input as well...
#endif


//#define MH_TEST_RPM
#ifdef MH_TEST_RPM
	printf("Testing P0.4 rpm sensor\r\n");
	while(true)
	{
		MagTimer_Get_RPM();
		wait_ms(1000);
	}
#endif
//#define MH_TEST_EEPROM
#ifdef MH_TEST_EEPROM
    eeprom_Test();
    while(true);
#endif

//#define MH_TEST_ADC
#ifdef MH_TEST_ADC
   DisplayStateReadingDelay(10000);
#endif

//#define MH_TEST_MOTOR_STATE
#ifdef MH_TEST_MOTOR_STATE
   printf("Board_TestMotorState\r\n");
   while(true)
   {
	   Board_TestMotorState();
	   wait_ms(1000);	// Once a second
   }
#endif

#ifdef MH_TEST_PWM
   while(1) {
    	for(int pchan=1;pchan<=3;pchan++)
        {
           	printf("PWM_chan: %d\r\n",pchan);
            PWM_Enable(pchan);
        	Board_ShowSwitches();
        	for(int led=1;led<=6;led++)
        	{
        		int pcnt = led * 10;
        		PWM_SetPulsePercent(pcnt);
        		Board_LED_Set(led, true);
        		wait_ms(2000);		// wait 2 secs
        	    Board_LED_Set(led, false);
        	}
        }
   }
#endif
    return 0 ;
}
