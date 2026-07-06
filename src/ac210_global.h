/*
 * ac210_global.h
 *
 *  Created on: 15/11/2016
 *      Author: Murray
 */

#ifndef AC210_GLOBAL_H_
#define AC210_GLOBAL_H_

#include "board.h"
#include "global.h"
#include "ac210_pins.h"
#include "ac210_timer.h"

extern volatile uint32_t sysTick;

extern const char *AC210_name;		// defined in AC210.c

void AC210_plog_puts(const char *s);
void AC210_plog_display(int val);
void AC210_plog_init(void);

void AC210_DeIcer_check(void);

//#define MH_INT_CURRENT		// MHH:16/12/2020
//#define MH_INT_CURRENT		// MHH:11/01/2021
void MH_check_if_interrupt(int sample_multiple);

#define SLIPRING_TEST_STATE		0
#define SLIPRING_TEST_CURRENT	1
//#define SLIPRING_TEST_COARSE	2
extern int Slipring_test_type;
extern uint32_t TimeInTicks;
bool ADC_slipring_test_ini(int test_type);
uint8_t ADC_get_slipring_log_data(void);
void ADC_check_state_test(void);
bool ADC_testing_slipring_idle(void);
void ADC_cancel_any_slipring_state_test(void);


void AC210_Aux2_CAN_Init(void);

void AC210_Aux2_Serial_Init(void);
bool AC210_Aux2_Readable(void);
int AC210_Aux2_UARTGetChar(void);
void AC210_Aux2_UARTPutSTR(char *str);

void AC210_check_slave_command(char *ibuff);

void AC210_reboot(void);

void MagTimer_Check_RPM(void);

void AC210_Disable_COM3(void);
void AC210_CAN_Init(int baudrate,int can_mode);

#ifdef MH_XXX		// MHH:08/04/2026
void AC210_RELAY_Set(bool On);
extern bool AC210_relay_on;
#endif

void AC210_RTC_init(void);
ULONG AC210_RTC_secs(void);
WORD AC210_set_rtc(uint32_t utc_secs_since_2000);
extern uint32_t RTC_secs;

uint32_t AC210_usecs_elapsed(void);
uint32_t AC210_ms_elapsed(uint32_t *p_usecs_prev);
void AC210_ms_display(char *desc,uint32_t ms);

void AC210_SIG100_check(void);
void AC210_SIG100_send(void);
void AC210_brushless_pin_init(void);



#ifdef MH_SIG60
void AC210_SIG60_Init(void);
void AC210_SIG60_check(void);
void AC210_SIG60_set_stop(int stop);		// Called from sstate.c
bool AC210_SIG60_Connected(void);
WORD AC210_SIG60_encoder_angle(void);
void AC210_SIG60_send_stops(void);
float AC210_SIG60_angle(void);
bool AC210_SIG60_set_angle(WORD value);
bool AC210_SIG60_motor_active(void);
int AC210_SIG60_input_errors(void);
#endif

float AC210_SIG100_angle(void);

void Aux_ChangeBaud(int baud_rate);
void Aux_putc(char ch);
void Aux_puts(char *string);
bool Aux_readable(void);
int Aux_getc(void);
int Aux_getc_timeout(uint32_t millisecs);

WORD AC210_aux2_get_throttle(void);
void AC210_aux2_check_line(void);

WORD AC210_can_get_throttle(void);
void AC210_display_auto_results(void);
void AC210_save_flash(void);
void AC210_restore_flash(void);
int AC210_hex_display_i2c_buff(void);

void AC210_test_leds(void);

void AC210_logix_show(void);
void AC210_logix_build(void);
void AC210_logix_verify(WORD value);

int AC210_logix_erase(WORD value);

void AC210_logctl_repair(void);

void AC210_logctl_check_run();
void AC210_display_logctl(void);
void AC210_log_init(void);
void AC210_serial_log_init(void);
void AC210_eeprom_write_magic(void);
void AC210_eelog(void);
void AC210_eelog_change_rate(WORD value);
void AC210_eelog_check(WORD value);
void AC210_eelog_write_diags(void);
void AC210_ee_display_ctl(WORD value);
void AC210_ee_display_diags(WORD value);
void AC210_ee_load(WORD value);	// called from comms.c
void AC210_ee_load_range(void);	// called from comms.c
void AC210_eelog_history(WORD value); // called from comms.c
void AC210_repair_diag_rec(WORD value);	// called from comms.c
void AC210_ee_verify(WORD value);	// called from comms.c
void AC210_ee_duplicate(WORD value);	// Used for testing
void AC210_eelog_send_diag_rec(WORD value);	// called from diags.c
int AC210_current_log_filesize(int ifrom);	// called from diags.c
long AC210_log_get_ticks(void);	// called from diags.c
void AC210_log_check_erase_sector(void);

void AC210_ssp_flash_hex(WORD value);	// called from comms.c
int AC210_ssp_reset_flash_check(void);	// called from comms.c and ac210_log.c
int AC210_get_flash_check_page(void);	// called from comms.c

void AC210_PC_ChangeBaud(int baud_rate);	// in AC210_comms.c, called from comms.c

void AC210_uart0_wait(void);
void AC210_uart3_wait(void);

void DebugAbort(char *emsg);
void eeprom_hex(WORD value);
void eeprom_log_reset(WORD val);

int AC210_flash(void);
void AC210_dummy_str(const unsigned char *str);	// Force Version_id to load
void AC210_Switch_Init(void);
void AC210_LED_Init(void);

extern bool AC210_integrated;		// Defined in ac210_digital.c
void AC210_Switch_Test_Integrated(void);
void Set_log_data_enabled(bool bval,int ifrom);

int AC210_SerialInit(void);
void AC210_SerialDeInit(void);

void AC210_main(void);
void p_Wait(void);
void AC210_watchIt(WORD from);	// called from main:watchIt()
void AC210_watchIt2(WORD from);	// called from main:watchIt2()

WORD AC210_board_test(WORD val);
void AC210_RIT_Set(uint32_t usecs);		// Repetitive interrupt timer. Called from comms.c

void Debug_ShowCommand(char *buf);

void AC210_STATE_Init(void);
void p_ZeroStatePins(void);
void p_SetStatePins(int test_state);

void AC210_DRIVE_Init(void);
void p_SetDrivePWM(WORD AC200_pwm,WORD start_one_shot_ms);
void p_SetCycleRate(BYTE PWM_CycleRate);
void p_SetDrivePins(void);
void SetDrivePins(uint32_t drive_pins);
//void Drive_set_speed2(WORD speed,int ifrom);
void Drive_set_speed2(WORD speed);	// MHH:21/01/2026

WORD p_GetAC200_Switches(bool maskmanual);
void p_send(const char *buf,uint32_t numChar);
void p_rxDisableInt(void);
void p_rxEnableInt(void);

void p_eeprom_i2c_pos(uint32_t pos);
int p_eeprom_i2c_read(uint8_t *buff, uint16_t len);
int p_eeprom_i2c_write(const uint8_t *buff, uint16_t len);
void p_setLEDS(WORD state);

//WORD p_GetAC200_magRPM(BYTE *flag_speed_changed,WORD ScalingFactorPercent);
WORD p_GetAC200_magRPM(WORD ScalingFactorPercent);	// MHH:20/11/2020

#define AC210_SRC_ADC		51000
#define AC210_SRC_BOARD		52000
#define AC210_SRC_COMMS		53000
#define AC210_SRC_DIGITAL	54000
#define AC210_SRC_DRIVE		55000
#define AC210_SRC_FLASH		56000
#define AC210_SRC_I2C		57000
#define AC210_SRC_LEDS		58000
#define AC210_SRC_NOTCODED	59000
#define AC210_SRC_PWM		60000
#define AC210_SRC_RPM		61000
#define AC210_SRC_STATE		62000
#define AC210_SRC_TIMER		63000
#define AC210_SRC_US_TICKER	64000
#define AC210_SRC_MAIN		65000
#define AC210_SRC_LOG		66000
#define AC210_SRC_SIG100	67000

#define SRC_ANALOG			WD_ANALOG
#define SRC_DIAGS			WD_DIAGS
#define SRC_RPM				WD_RPM

//void Abort(int err,char *msg);		// Moved to global.h
//int DiagsError(ULONG err);
#ifdef AC210_PORT
extern bool AC210_watchdog_active;
#define AC210_4LEDS
#ifdef AC210_4LEDS
extern bool AC210_4leds;
#endif
int ReturnToContinue(void);
extern char p_Buffer[];
extern int p_BufferLen;
extern void Board_UART0PutSTR(const char *p_buff);

void Aux_check_line(void);	// defined in AC210_comms.c

void PC_putc(char ch);
//void PC_getline();
void PC_flush_output(void);
int PC_getc(void);		// MH:16/03/2023. Maybe we can...Cannot have this because LPC_UART0 special case and all characters sent to rxBuF, terminated by CRLF
void PC_puts(const char *buf);
int PC_ReturnToContinue(void);
void Aux2_puts(char *string);
void Aux2_puts2(const char *string);
int Aux2_getc(void);
void Aux2_uart2_wait(void);


//#define PRINTF(...) {p_BufferLen = sprintf(p_Buffer,__VA_ARGS__);Board_UART0PutSTR(p_Buffer);}
#define PRINTF(...) {sprintf(p_Buffer,__VA_ARGS__);Board_UART0PutSTR(p_Buffer);}	// MHH:10/01/2026
#define PRINTF_FLUSH		{AC210_uart0_wait();}
//#define DPRINTF(...) 		{p_BufferLen = sprintf(p_Buffer,__VA_ARGS__);Aux2_puts(p_Buffer);}
#define DPRINTF(...) 		{sprintf(p_Buffer,__VA_ARGS__);Aux2_puts(p_Buffer);}	// MHH:10/01/2026
//#define DPRINTF(...) 		{printf(__VA_ARGS__);}
#define DPRINTF_FLUSH		{Aux2_uart2_wait();}

//#define LPRINTF(...) {p_BufferLen = sprintf(p_Buffer,__VA_ARGS__);AC210_plog_puts(p_Buffer);}
#define LPRINTF(...) {sprintf(p_Buffer,__VA_ARGS__);AC210_plog_puts(p_Buffer);}		// MHH:10/01/2016
//#define L2PRINTF(...) {p_BufferLen = sprintf(p_Buffer,__VA_ARGS__);Board_UART0PutSTR(p_Buffer);AC210_plog_puts(p_Buffer);}
//#define L2PRINTF(...) {p_BufferLen = sprintf(p_Buffer,__VA_ARGS__);Aux2_puts(p_Buffer);AC210_plog_puts(p_Buffer);}
#define L2PRINTF(...) {sprintf(p_Buffer,__VA_ARGS__);Aux2_puts(p_Buffer);AC210_plog_puts(p_Buffer);}	// MHH:10/01/2026

//#define AUX_PORT_SIG60		3		// May need to move...

#endif
extern uint8_t p_ControlPort;
extern uint16_t gVerbose;
//extern uint8_t p_State_Hi_Output;
//extern uint8_t p_State_Lo_Output;

extern bool Sig100_connected;
extern bool AC210_brushless_board;
extern bool AC210_remote_control_board;
extern bool Hub_pos_error;

void AC210_ssp_write_test(uint32_t sector);
void AC210_ssp_flash_erase(uint32_t sector);

void mh_debug();

extern int CAN_mode;
#define CAN_MODE_ROTAX		1
#define CAN_MODE_RC			2
#define CAN_MODE_SERIAL		3

int iround(float f);

void Wait_secs_no_watchdog(int secs);
void Set_overcurrent_dir(WORD v);


//#define MH_PUTC_POS2

#endif /* AC210_GLOBAL_H_ */
