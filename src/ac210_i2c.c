/* i2c.c - adapted from example in periph_i2c . MHH: 14-May-2015
 * Includes eeprom read/write routines
 *
 */

#include <stdlib.h>
#include <string.h>

#include "board.h"

#include "ac210_log.h"
#include "ac210_rtc.h"

#include "global.h"


//#include <cr_section_macros.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define DEFAULT_I2C          I2C0

// Note: The LPC1769 on board ee-prom is 8k bytes (64k bits) and is accessed through
// I2C1 (pins P0.19 and P0.20)

// It has a page size of 32 bytes. When writing, the first 2 bytes are the address (high order first).

//#define ADDRESS_24LC64		0x50		// LPX1769 on board ee-prom address
//#define PAGESIZE_24LC64		32

#define ADDRESS_M24M01		0x50		// Testing for use on LPC1768 AC200 logic board
#define PAGESIZE_M24M01		256


#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000

static int mode_poll;   /* Poll/Interrupt mode flag */
//static I2C_ID_T i2cDev = DEFAULT_I2C; /* Currently active I2C device */
//static uint8_t buffer[2][256];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
//		Chip_I2C_SlaveStateHandler(id);	// MHH:02/01/2019. After adding RTC was getting some hardware faults via this routine.
	}
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	Board_I2C_Init(id);

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C1_IRQHandler(void)
{
	i2c_state_handling(I2C1);
}

/**
 * @brief	I2C0 Interrupt handler
 * @return	None
 */
void I2C0_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

/**
 * @brief	Main program body
 * @return	int
 */
extern volatile uint32_t sysTick;
#ifdef MH_DELAY_TICKS
void delayTicks(uint32_t ticks)
{
	uint32_t save_ticks = sysTick;
//	printf("Waiting for %d ticks",ticks);
	while(sysTick < save_ticks+ ticks) {};
}
#endif

//=================================================================================
// Note: These routines assume that there is only one eeprom we are accessing, and we
// have initialised its details. Just makes it simpler, otherwise have to remember multiple
// ids, slave addresses and positions
#define EEPROM_MAX_PAGESIZE	256
struct I2C_EEPROM_struct {
	I2C_ID_T i2c_id;
	uint8_t slaveAddr;
	uint16_t pagesize;
	uint32_t pos;
	uint32_t wpos;
	uint8_t buff[EEPROM_MAX_PAGESIZE+2];
} I2C_eeprom;
//------------------------------------------------------------------------------
int eeprom_i2c_init(I2C_ID_T i2c_id, uint8_t slaveAddr,uint16_t pagesize)
{
	uint8_t buff[4];

// Initialise the I2C port

	i2c_app_init(i2c_id, SPEED_100KHZ);

// Now, probe to see if a valid slaveAddr

	if(Chip_I2C_MasterRead(i2c_id, slaveAddr, buff, 1) <= 0) {
		printf("eeprom_i2c_init: Cannot access slaveAddr:%02X", slaveAddr);
		return -1;
	}

	if(pagesize > EEPROM_MAX_PAGESIZE) {
		printf("eeprom_i2c_init: pagesize exceeds maximum of %d\n\r", EEPROM_MAX_PAGESIZE);
		return -1;
	}

// So far so good....

	I2C_eeprom.i2c_id    = i2c_id;
	I2C_eeprom.slaveAddr = slaveAddr;
	I2C_eeprom.pagesize  = pagesize;
	I2C_eeprom.pos = 0;
	return 0;
}
//------------------------------------------------------------------------------
//#define EEPROM_SIZE		(65536*2)		// Must be less than this or will wrap
void p_eeprom_i2c_pos(uint32_t pos)
{
	if(pos > EEPROM_SIZE) pos = EEPROM_SIZE;
	I2C_eeprom.pos = pos;
}
//------------------------------------------------------------------------------
uint32_t AC210_get_eeprom_pos(void)
{
	return I2C_eeprom.pos;
}
//------------------------------------------------------------------------------
#define EEPROM_HIGH_BIT		1		// Held in slave address byte

//#define EEPROM_HIGH_BIT		2		// Held in slave address byte
int eeprom_i2c_read_pos(void)
{
	int tcnt,bytes_written;

	if(I2C_eeprom.pos == I2C_eeprom.wpos) {
		return 0;		// nothing to do
	}
// Note: To address above 64k bytes need to put MSB in slave address B1 (bit 1) position.

	I2C_eeprom.buff[0] = I2C_eeprom.pos >> 8;	// High order pos
	I2C_eeprom.buff[1] = I2C_eeprom.pos & 0xff;	// Low order pos
	I2C_eeprom.slaveAddr &= ~EEPROM_HIGH_BIT;		// Mask out high order bit
	if(I2C_eeprom.pos & 0x10000) I2C_eeprom.slaveAddr |= EEPROM_HIGH_BIT;

	for(tcnt=0;tcnt<20;tcnt++) {
		bytes_written = Chip_I2C_MasterSend(I2C_eeprom.i2c_id, I2C_eeprom.slaveAddr, I2C_eeprom.buff, 2);
		if(bytes_written) break;

//		printf("?");	// MHH:09/08/2023. Commented out.
//		delayTicks(1);
		wait_ms(5);
	}
	if(bytes_written != 2) {
		printf("eeprom_i2c_read_pos: bytes_written != 2\n\r");
		return -1;
	}
	I2C_eeprom.wpos = I2C_eeprom.pos;
	return 0;
}
//------------------------------------------------------------------------------
// The idea is to make the i/o independent of page size, for portability
#define I2C_MAXSIZE_RAW_READ		252			// 256 no good

int p_eeprom_i2c_read(uint8_t *buff, uint16_t len)
{
	int bytes_read;

	if(eeprom_i2c_read_pos()) {	// Make sure positioned correctly
		return -1;
	}

	int bytes_left = len;
	uint8_t *p_dst = buff;
	for(;;)
	{
		int read_len   = MIN(bytes_left,I2C_MAXSIZE_RAW_READ);
		bytes_read = Chip_I2C_MasterRead(I2C_eeprom.i2c_id,I2C_eeprom.slaveAddr , p_dst, read_len);
		if(bytes_read != read_len) {
			printf("eeprom_i2c_read: bytes_read != requested\n\r");
			return -1;
		}
		bytes_left -= read_len;
		if(bytes_left <= 0) break;
		p_dst += read_len;
	}

	I2C_eeprom.pos += len;
	I2C_eeprom.wpos = I2C_eeprom.pos;
	return 0;
}
//------------------------------------------------------------------------------
#ifdef MH_CHECK_OVERLAP
static bool CheckOverlap(int pos1,int len1,int pos2,int len2)
{

	int pos1b = pos1 + len1 -1;
	int pos2b = pos2 + len2 -1;

	if(pos1 >= pos2 && pos1 <= pos2b) return true;
	if(pos1b >= pos2 && pos1b <= pos2b) return true;

	if(pos2 >= pos1 && pos2 <= pos1b) return true;
	if(pos2b >= pos1 && pos2b <= pos1b) return true;

	return false;
}
#endif
//------------------------------------------------------------------------------
// The idea is to make the i/o independent of page size, for portability
int p_eeprom_i2c_write(const uint8_t *buff, uint16_t len)
{
	int bytes_written;
	uint16_t wlen;
	int bytes_left,tcnt;
	int page_boundary,page_bytes_left;
	const uint8_t *bp;


#ifdef MH_CHECK_OVERLAP
	if(CheckOverlap(I2C_eeprom.pos,len,1024,48))
	{
		printf("Overlap!!\r\n");
	}
#endif

	bytes_left = len;
	bp = buff;
	while(bytes_left > 0) {
		I2C_eeprom.buff[0] = I2C_eeprom.pos >> 8;	// High order pos
		I2C_eeprom.buff[1] = I2C_eeprom.pos & 0xff;	// Low order pos
		I2C_eeprom.slaveAddr &= ~EEPROM_HIGH_BIT;		// Mask out high order bit
		if(I2C_eeprom.pos >= EEPROM_SIZE)
		{
			return -2;									// EOF
		}
		uint32_t total_bytes_left = EEPROM_SIZE - I2C_eeprom.pos;

		if(I2C_eeprom.pos & 0x10000)
		{
			I2C_eeprom.slaveAddr |= EEPROM_HIGH_BIT;
		}
#define MH_EEPROM_PAGE_WRITE
#ifdef MH_EEPROM_PAGE_WRITE
		page_boundary = ((I2C_eeprom.pos / I2C_eeprom.pagesize) + 1) * I2C_eeprom.pagesize;
		page_bytes_left = page_boundary - I2C_eeprom.pos;
#endif
		wlen = MIN(bytes_left,len);
		wlen = MIN(wlen,page_bytes_left);
		wlen = MIN(wlen,253);		// Cannot exceed this because (wlen+2) cannot exceed 255. MHH:30/11/2016
		wlen = MIN(wlen,total_bytes_left);		// Otherwise can wrap around.
		memcpy(I2C_eeprom.buff+2,bp,wlen);
		for(tcnt=0;tcnt<20;tcnt++) {
			bytes_written = Chip_I2C_MasterSend(I2C_eeprom.i2c_id, I2C_eeprom.slaveAddr, I2C_eeprom.buff, wlen+2);
			if(bytes_written) break;

//			printf("+");		// MHH:09/08/2023
//			delayTicks(1);
			wait_ms(5);
		}
		if(bytes_written != wlen+2) {
			printf("eeprom_i2c_write: bytes_written:%d != requested:%d\n\r",bytes_written,wlen+2);
			return -1;
		}
		bytes_left -= wlen;
		bp += wlen;
		I2C_eeprom.pos += wlen;
		I2C_eeprom.wpos = I2C_eeprom.pos;
//		printf("eeprom_i2c_write:pos:%d,delaying a second\n\r",I2C_eeprom.pos);
//		delayTicks(100);
	}
	return 0;
}
//------------------------------------------------------------------------------
// Hex dump 256 bytes of eeprom from position value
uint8_t I2C_hex_buff[256];
void eeprom_hex2(int jstart)
{
	PRINTF("  ");

	int jmax=jstart+16;
#ifdef MH_BUFF250
	if(jmax > 250)
	{
		jmax=250;
		for(int i=0;i<6;i++) printf("   ");
	}
#endif

	for(int j=jstart;j<jmax;j++)
	{
		uint8_t c = I2C_hex_buff[j];
		uint8_t c2 = c;
		if(c < 32 || c > '}') c2 = '_';
		PC_putc(c2);
	}
}
//-----------------------------------------------------------------------------------
int AC210_hex_display_i2c_buff(void)
{
	int jstart=0;
	uint8_t c;
	for(int i=0;i<256;i++)
	{
		if(i && (i%16 == 0))
		{
			eeprom_hex2(jstart);
			jstart+=16;
		}
		if(i%16 == 0) PRINTF("\r\n%04x ",i);
		c = I2C_hex_buff[i];
		PRINTF(" %02x",c);
	}
	eeprom_hex2(jstart);
	//	puts("\r\n");
	PRINTF("\r\n");

    if(PC_ReturnToContinue() == 'X') return 1;
    return 0;
}
//-----------------------------------------------------------------------------------
void eeprom_hex(WORD value)
{
	AC210_watchdog_active = false;

	p_eeprom_i2c_pos(value);
	uint32_t pos = value;
	PC_puts("\r\n");
    PRINTF_FLUSH;		// Wait for buffer to be output
    wait_ms(100);			// Wait for '>'
	for(;;)
	{
		PRINTF("\r\nPos:%d\r\n",pos);
		if(p_eeprom_i2c_read(I2C_hex_buff, 256)) {
			DebugAbort("eeprom_i2c_read");
		}
		pos += 256;

		if(AC210_hex_display_i2c_buff()) break;

	}
	AC210_watchdog_active = true;

}
//#define MH_EEPROM_TEST
#ifdef MH_EEPROM_TEST
//------------------------------------------------------------------------------
static uint8_t TestWriteBuffer[128],TestReadBuffer[128];
#ifdef MH_OLD_I2C_TEST
int CheckBuffers(int ibuf)
{
	int i,dcnt;
	uint8_t cw,cr;
	dcnt = 0;
//	istart = ibuf * 100;
	for(i=0;i<100;i++) {
//		cw = (istart+i) & 0xff;	// should be..
		cw = i;
		cr = TestReadBuffer[i];
		if(cr != cw) {
			printf("ibuf:%d,Byte:%d, cw:%d, Cr:%d\n\r",ibuf,i,cw,cr);
			dcnt++;
		}
	}
//	printf("Differences:%d\n\r",dcnt);
	return dcnt;
}
#endif
//------------------------------------------------------------------------------
void eeprom_Test(void)
{
//	int i;
//	int dcnt = 0;
	int pos;

	printf("eeprom_test:\n\r");
//	printf("eeprom_i2c_init..\n\r");
//	eeprom_i2c_init(I2C0, ADDRESS_M24M01,PAGESIZE_M24M01);

#ifdef MH_OLD_I2C_TEST
	for(i=0;i<128;i++) {
		TestWriteBuffer[i] = i;
	}
#endif

	printf("Writing at %d byte page intervals, starting from position 1024\r\n",EEPROM_RESET_SIZE);

	for(pos=EELOG_START_POS;pos<EEPROM_SIZE;pos+=EEPROM_RESET_SIZE)
	{
		p_eeprom_i2c_pos(pos);
		sprintf((char *)TestWriteBuffer,"%6d",pos);
		printf("%6d ",pos);

		if(p_eeprom_i2c_write(TestWriteBuffer, 8)) {
			DebugAbort("eeprom_i2c_write");
		}
	}
	printf("\r\nFinished writing,I2C_eeprom.pos = %d\r\n",I2C_eeprom.pos);
	printf("Now checking\r\n");
	for(pos=EELOG_START_POS;pos<EEPROM_SIZE;pos+=EEPROM_RESET_SIZE)
	{
		p_eeprom_i2c_pos(pos);
		sprintf((char *)TestWriteBuffer,"%6d",pos);
		printf("%6d ",pos);
		if(p_eeprom_i2c_read(TestReadBuffer, 8)) {
			DebugAbort("eeprom_i2c_read");
		}
		if(strcmp((char *)TestWriteBuffer,(char *)TestReadBuffer) != 0)
		{
			printf("\r\nTestReadBuffer:%s\r\n",TestReadBuffer);
		}
	}
	printf("\r\nFinished checking,I2C_eeprom.pos = %d\r\n",I2C_eeprom.pos);


#ifdef MH_OLD_I2C_TEST
	printf("Writing 8000 bytes..\n\r");
	for(i=0;i<80;i++) {
		if(eeprom_i2c_write(TestWriteBuffer, 100)) {
			Abort("eeprom_i2c_write");
		}
	}
	printf("Positioning to zero\n\r");
	eeprom_i2c_pos(0);
	printf("Reading 8000 bytes..\n\r");
	for(i=0;i<80;i++) {
		if(eeprom_i2c_read(TestReadBuffer, 100)) {
			Abort("eeprom_i2c_read");
		}
		dcnt += CheckBuffers(i);
	}
	printf("Total dcnt:%d\n\r",dcnt);
	printf("Indefinite delay...");
	while(1) {
		delayTicks(1000);
	}
#ifdef MH_NO_I2C	// May be useful?
	Chip_I2C_DeInit(I2C0);
	Chip_I2C_DeInit(I2C1);
#endif
#endif
}
#endif
//------------------------------------------------------------------------------
int mh_eeprom_test;
void AC210_eeprom_I2C_Init(void)
{
//	printf("AC210_eeprom_I2C_Init...\n\r");
	eeprom_i2c_init(I2C0, ADDRESS_M24M01,PAGESIZE_M24M01);
#ifdef MH_EEPROM_TEST
	if(mh_eeprom_test)
	{
		eeprom_Test();		// Test!!!!
	}
#endif
}
//=================================================================================
#ifdef MH_RTC
#define ADDRESS_DS3231		 0x68		// 1101000
//#define PAGESIZE_DS3231		 0x13		// Bytes 0 to 12 are valid

#define DS3231_MAX_PAGESIZE		0x13		// Reduce later - want to look at flags initially
uint8_t ds3231_buff_in[DS3231_MAX_PAGESIZE];
//------------------------------------------------------------------------------
int DS3231_i2c_read(int len)
{
	int bytes_read = Chip_I2C_MasterRead(I2C1, ADDRESS_DS3231, ds3231_buff_in, len);
	if(bytes_read != len) {
		printf("DS3231_i2c_init: Cannot access slaveAddr:%02X", ADDRESS_DS3231);
		return -1;
	}
	return 0;
}
//----------------------------------------------------------------------------------
int DS3231_i2c_pos(int pos)
{
	uint8_t buff[4];
	int bytes_written;
	int tcnt;

//	buff[0] = pos >> 8;	// High order pos
//	buff[1] = pos & 0xff;	// Low order pos

	buff[0] = pos;

	for(tcnt=0;tcnt<5;tcnt++) {
//		bytes_written = Chip_I2C_MasterSend(I2C1, ADDRESS_DS3231, buff, 2);
		bytes_written = Chip_I2C_MasterSend(I2C1, ADDRESS_DS3231, buff, 1);
		if(bytes_written) break;

		printf("?");
//		delayTicks(1);
		wait_ms(5);
	}
	if(bytes_written != 1) {
		printf("DS3231_i2c_read_pos: bytes_written != 1\n\r");
		return -1;
	}
	return 0;
}
//------------------------------------------------------------------------------
int DS3231_i2c_init(void)
{
// Note: Could check that both SDA1 and SCL1 pins are being pulled up to see if DS3231 attached.


// Initialise the I2C port

	i2c_app_init(I2C1, SPEED_100KHZ);

// Now, try and read entire buffer

	if(DS3231_i2c_pos(0) == 0)
	{
		return DS3231_i2c_read(DS3231_MAX_PAGESIZE);
	}
	return -1;

/*
	int bytes_read = Chip_I2C_MasterRead(I2C1, ADDRESS_DS3231, ds3231_buff_in, DS3231_MAX_PAGESIZE);
	if(bytes_read != DS3231_MAX_PAGESIZE) {
		printf("DS3231_i2c_init: Cannot access slaveAddr:%02X", ADDRESS_DS3231);
		return -1;
	}
	return 0;
*/
}
//----------------------------------------------------------------------------------
int DS3231_i2c_read_7(void)
{
	if(DS3231_i2c_pos(0) == 0)
	{
		return DS3231_i2c_read(7);
	}
	return -1;
}
//----------------------------------------------------------------------------------
int DS3231_i2c_write(const uint8_t *buff, int pos, uint8_t len)
{
	int bytes_written;
	int tcnt;
	uint8_t buff_out[DS3231_MAX_PAGESIZE+2];

//	buff_out[0] = pos >> 8;	// High order pos
//	buff_out[1] = pos & 0xff;	// Low order pos

	buff_out[0] = pos;

	memcpy(buff_out+1,buff,len);

	for(tcnt=0;tcnt<5;tcnt++) {
		bytes_written = Chip_I2C_MasterSend(I2C1, ADDRESS_DS3231, buff_out, len+1);
		if(bytes_written) break;

		printf("+");
		wait_ms(5);
	}
	if(bytes_written <= 0)
	{
		return -1;
	}
	return 0;
}
#endif
