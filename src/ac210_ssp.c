/*
 * ac210_ssp.c
 *
 *  Created on: 3/01/2017
 *      Author: Murray
 */

//#include "lpc17xx.h"
#include "board.h"
#include "stdio.h"
#include "string.h"

#include "ac210_global.h"
#include "ac210_log.h"
#include "ac210_ssp.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

// FIXME: SSP0 not working on LPCXpresso LPC1769.  There seems to be some sort
//        of contention on the MISO signal.  The contention originates on the
//		  LPCXpresso side of the signal.
//


#define LPC_SC				LPC_SYSCTL
#define PCLKSEL0			PCLKSEL[0]
#define PINSEL0				PINSEL[0]
#define	LPC_PINCON			LPC_IOCON
#define	LPC_GPIO0 			(LPC_GPIO+0)
#define FIOCLR				CLR
#define FIOSET				SET

//#define SS_N_LO	LPC_GPIO0->FIOCLR |= (1<<16);	// GPIO pin P0.16 as a Slave Select (SS) pin is in the low level
//#define SS_N_HI	LPC_GPIO0->FIOSET |= (1<<16);	// GPIO pin P0.16 as a Slave Select (SS) pin is in the low level

//#define SS_N_LO	;	// GPIO pin P0.6 as a Slave Select (SS) pin is in the low level
//#define SS_N_HI	;	// GPIO pin P0.6 as a Slave Select (SS) pin is in the low level
#define SS_N_LO	LPC_GPIO0->FIOCLR |= (1<<6);	// GPIO pin P0.6 as a Slave Select (SS) pin is in the low level
#define SS_N_HI	LPC_GPIO0->FIOSET |= (1<<6);	// GPIO pin P0.6 as a Slave Select (SS) pin is in the low level

#define BUFFER_SIZE                         (0x110)

/* Tx buffer */
static uint8_t Tx_Buf[BUFFER_SIZE];

/* Rx buffer */
static uint8_t Rx_Buf[BUFFER_SIZE];
static Chip_SSP_DATA_SETUP_T xf_setup;

//#define MH_OLD_SSP
#ifdef MH_OLD_SSP


#ifdef MH_GT_8BIT
#define SSP_DATA_BIT_NUM(databits)          (databits + 1)


#define SSP_DATA_BYTES(databits)            (((databits) > SSP_BITS_8) ? 2 : 1)
#define SSP_LO_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? 0xFF : (0xFF >> \
																					  (8 - SSP_DATA_BIT_NUM(databits))))
#define SSP_HI_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? (0xFF >> \
																			   (16 - SSP_DATA_BIT_NUM(databits))) : 0)
#endif

#define SSP_MODE_SEL                        (0x31)
#define SSP_TRANSFER_MODE_SEL               (0x32)
#define SSP_MASTER_MODE_SEL                 (0x31)
#define SSP_SLAVE_MODE_SEL                  (0x32)
#define SSP_POLLING_SEL                     (0x31)
#define SSP_INTERRUPT_SEL                   (0x32)
#define SSP_DMA_SEL                         (0x33)


//static SSP_ConfigFormat ssp_format;
static Chip_SSP_DATA_SETUP_T xf_setup;
static volatile uint8_t  isXferCompleted = 0;
static uint8_t dmaChSSPTx, dmaChSSPRx;
static volatile uint8_t isDmaTxfCompleted = 0;
static volatile uint8_t isDmaRxfCompleted = 0;

#if defined(DEBUG_ENABLE)
static char sspWaitingMenu[] = "SSP Polling: waiting for transfer ...\n\r";
static char sspIntWaitingMenu[]  = "SSP Interrupt: waiting for transfer ...\n\r";
static char sspDMAWaitingMenu[]  = "SSP DMA: waiting for transfer ...\n\r";

static char sspPassedMenu[] = "SSP: Transfer PASSED\n\r";
static char sspFailedMenu[] = "SSP: Transfer FAILED\n\r";

#ifdef MH_OLDWAY
static char sspTransferModeSel[] = "\n\rPress 1-3 or 'q' to quit\n\r"
								   "\t 1: SSP Polling Read Write\n\r"
								   "\t 2: SSP Int Read Write\n\r"
								   "\t 3: SSP DMA Read Write\n\r";

static char helloMenu[] = "Hello NXP Semiconductors \n\r";
static char sspMenu[] = "SSP demo \n\r";
static char sspMainMenu[] = "\t 1: Select SSP Mode (Master/Slave)\n\r"
							"\t 2: Select Transfer Mode\n\r";
static char sspSelectModeMenu[] = "\n\rPress 1-2 to select or 'q' to quit:\n\r"
								  "\t 1: Master \n\r"
								  "\t 2: Slave\n\r";
#endif

#endif /* defined(DEBUG_ENABLE) */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize buffer */
static void Buffer_Init(void)
{
	uint16_t i;
	uint8_t ch = 0;

	for (i = 0; i < BUFFER_SIZE; i++) {
		Tx_Buf[i] = ch++;
		Rx_Buf[i] = 0xAA;
	}
}

/* Verify buffer after transfer */
static uint8_t Buffer_Verify(void)
{
	uint16_t i;
	uint8_t *src_addr = (uint8_t *) &Tx_Buf[0];
	uint8_t *dest_addr = (uint8_t *) &Rx_Buf[0];

	for ( i = 0; i < BUFFER_SIZE; i++ ) {

		if ((*src_addr) != (*dest_addr) ) {
			return 1;
		}
		src_addr++;
		dest_addr++;

#ifdef MH_GT_8BIT
		if (SSP_DATA_BYTES(ssp_format.bits) == 2) {
			if (((*src_addr) & SSP_HI_BYTE_MSK(ssp_format.bits)) !=
				((*dest_addr) & SSP_HI_BYTE_MSK(ssp_format.bits))) {
				return 1;
			}
			src_addr++;
			dest_addr++;
			i++;
		}
#endif
	}
	return 0;
}

/* Select the Transfer mode : Polling, Interrupt or DMA */
static void appSSPTest(int transfer_mode)
{
	dmaChSSPTx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Tx);
	dmaChSSPRx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Rx);

	xf_setup.length = BUFFER_SIZE;
	xf_setup.tx_data = Tx_Buf;
	xf_setup.rx_data = Rx_Buf;


	Buffer_Init();

	switch (transfer_mode) {
	case SSP_POLLING_SEL:	/* SSP Polling Read Write Mode */
		DEBUGOUT(sspWaitingMenu);
		xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

		Chip_SSP_RWFrames_Blocking(LPC_SSP1, &xf_setup);

		if (Buffer_Verify() == 0) {
			DEBUGOUT(sspPassedMenu);
		}
		else {
			DEBUGOUT(sspFailedMenu);
		}
		break;

	case SSP_INTERRUPT_SEL:
		DEBUGOUT(sspIntWaitingMenu);

		isXferCompleted = 0;
		xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

		Chip_SSP_Int_FlushData(LPC_SSP1);/* flush dummy data from SSP FiFO */
		//			if (SSP_DATA_BYTES(ssp_format.bits) == 1) {
		Chip_SSP_Int_RWFrames8Bits(LPC_SSP1, &xf_setup);
		//			}
		//			else {
		//				Chip_SSP_Int_RWFrames16Bits(LPC_SSP1, &xf_setup);
		//			}

		Chip_SSP_Int_Enable(LPC_SSP1);	/* enable interrupt */
		while (!isXferCompleted) {}

		if (Buffer_Verify() == 0) {
			DEBUGOUT(sspPassedMenu);
		}
		else {
			DEBUGOUT(sspFailedMenu);
		}
		break;

	case SSP_DMA_SEL:	/* SSP DMA Read and Write: fixed on 8bits */
		DEBUGOUT(sspDMAWaitingMenu);
		isDmaTxfCompleted = isDmaRxfCompleted = 0;
		Chip_SSP_DMA_Enable(LPC_SSP1);
		/* data Tx_Buf --> SSP */
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChSSPTx,
				(uint32_t) &Tx_Buf[0],
				GPDMA_CONN_SSP1_Tx,
				GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
				BUFFER_SIZE);
		/* data SSP --> Rx_Buf */
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChSSPRx,
				GPDMA_CONN_SSP1_Rx,
				(uint32_t) &Rx_Buf[0],
				GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
				BUFFER_SIZE);

		while (!isDmaTxfCompleted || !isDmaRxfCompleted) {}
		if (Buffer_Verify() == 0) {
			DEBUGOUT(sspPassedMenu);
		}
		else {
			DEBUGOUT(sspFailedMenu);
		}
		Chip_SSP_DMA_Disable(LPC_SSP1);
		break;

		//		case 'q':
		//		case 'Q':
		//			Chip_GPDMA_Stop(LPC_GPDMA, dmaChSSPTx);
		//			Chip_GPDMA_Stop(LPC_GPDMA, dmaChSSPRx);

	default:
		break;
	}
}
/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	SSP interrupt handler sub-routine
 * @return	Nothing
 */
void SSP1_IRQHandler(void)
{
	Chip_SSP_Int_Disable(LPC_SSP1);	/* Disable all interrupt */
//	if (SSP_DATA_BYTES(ssp_format.bits) == 1) {
		Chip_SSP_Int_RWFrames8Bits(LPC_SSP1, &xf_setup);
//	}
#ifdef MH_GT_8BIT
	else {
		Chip_SSP_Int_RWFrames16Bits(LPC_SSP1, &xf_setup);
	}
#endif
	if ((xf_setup.rx_cnt != xf_setup.length) || (xf_setup.tx_cnt != xf_setup.length)) {
		Chip_SSP_Int_Enable(LPC_SSP1);	/* enable all interrupts */
	}
	else {
		isXferCompleted = 1;
	}
}

/**
 * @brief	DMA interrupt handler sub-routine. Set the waiting flag when transfer is successful
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChSSPTx) == SUCCESS) {
		isDmaTxfCompleted = 1;
	}

	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChSSPRx) == SUCCESS) {
		isDmaRxfCompleted = 1;
	}
}
#endif
//-------------------------------------------------------------------------------------------------
static void SSP_RW_Poll_Mode(int length)
{
	//	dmaChSSPTx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Tx);
	//	dmaChSSPRx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Rx);

	xf_setup.length = length;
	xf_setup.tx_data = Tx_Buf;
	xf_setup.rx_data = Rx_Buf;

	//	Buffer_Init();

	xf_setup.rx_cnt = xf_setup.tx_cnt = 0;
	SS_N_LO;				// Set !CS line to LO state
	if(Chip_SSP_RWFrames_Blocking(LPC_SSP1, &xf_setup) == 0)
	{
		DebugAbort("SSP_RW_Poll_Mode: error");
	}
	SS_N_HI;				// Set !CS line to HI state
//	printf("xf_setup.rx_cnt =%d, xf_setup.tx_cnt = %d\r\n",xf_setup.rx_cnt, xf_setup.tx_cnt);
}
//-------------------------------------------------------------------------------------------------
uint8_t SSP_Flash_Read_Status(void)
{
//	printf("SSP_Flash_Read_Status\r\n");
	Tx_Buf[0] = 5;		// Read status command
	Tx_Buf[1] = 0;		// dummy

	SSP_RW_Poll_Mode(2);	// Send and Receive
	uint8_t status = Rx_Buf[1];
	return status;
}
//-------------------------------------------------------------------------------------------------
uint32_t SSP_Flash_Read_ID(void)
{
//	printf("SSP_Flash_Read_ID\r\n");
	Tx_Buf[0] = 0x9F;		// Read ID command
	Tx_Buf[1] = 0;		// dummy

	SSP_RW_Poll_Mode(5);	// Send and Receive
	uint32_t flash_id=0;
	for(int i=1;i<=4;i++)
	{
		flash_id <<= 8;
		uint8_t b=Rx_Buf[i];
		flash_id |= b;
	}
//	printf("flash_id=%04x\r\n",flash_id);
	return flash_id;
}
//-------------------------------------------------------------------------------------------------
void SSP_Flash_Write_Enable(void)
{
	Tx_Buf[0] = 6;		// Write Enable
	SSP_RW_Poll_Mode(1);	// Send and Receive
}
//-------------------------------------------------------------------------------------------------
void SSP_copy_address(uint32_t byte_address)
{
	uint32_t a = byte_address;
	for(int i=3;i>=1;i--)
	{
		Tx_Buf[i] = (uint8_t) (a & 0xff);
		a >>=8;
	}
}
//-------------------------------------------------------------------------------------------------
#define SSP_CMD_PAGE_PROGRAM		0x2
#define SSP_CMD_READ				0x3

#define SSP_FLASH_PAGE_SIZE	256

#ifdef MH_SSP_READ_WRITEBYTE
void SSP_WriteByte(uint32_t byte_address,uint8_t byte_val)
{
	printf("SSP_WriteByte\r\n");

	SSP_Flash_Write_Enable();

	Tx_Buf[0] = SSP_CMD_PAGE_PROGRAM;		// write instruction

	SSP_copy_address(byte_address);
	Tx_Buf[4] = byte_val;
	SSP_RW_Poll_Mode(5);
	for(;;)
	{
		uint8_t sr1 = SSP_Flash_Read_Status();
		if(sr1 == 0) break;
		wait_ms(1);
	}
}
//-------------------------------------------------------------------------------------------------
uint8_t SSP_ReadByte(uint32_t byte_address)
{
	printf("SSP_ReadByte\r\n");

	Tx_Buf[0] = SSP_CMD_READ;		// read instruction

	SSP_copy_address(byte_address);
	SSP_RW_Poll_Mode(5);
	return Rx_Buf[4];	// ???
}
#endif
//-------------------------------------------------------------------------------------------------
static void SSP_Write_Poll_Mode(int length)		// Test write only
{
	//	dmaChSSPTx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Tx);
	//	dmaChSSPRx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Rx);

	SS_N_LO;				// Set !CS line to LO state
	if(Chip_SSP_WriteFrames_Blocking(LPC_SSP1, Tx_Buf, length) == 0)
	{
		DebugAbort("SSP_Write_Poll_Mode: error");
	}
	SS_N_HI;				// Set !CS line to HI state
}
//-------------------------------------------------------------------------------------------------
// Note: Could put a limit on loops here, in case of problem.
static bool SSP_wait_ready;
static void SSP_Flash_Wait_Ready(void)
{
	if(SSP_wait_ready == false) return;
	SSP_wait_ready = false;
	for(int i=0;i<500;i++)	// MHH:20/02/2019. Put limit in as if flash not working didn't exit loop
	{
		uint8_t sr1 = SSP_Flash_Read_Status();
		if(sr1 == 0) return;
		if(sr1 == 2) return;	// Just write enabled
		wait_ms(1);
	}
	DebugAbort("SSP_Flash_Wait_Ready: error");
}
//-------------------------------------------------------------------------------------------------
static bool SSP_Flash_Ready(void)
{
	if(SSP_wait_ready == false) return true;
	uint8_t sr1 = SSP_Flash_Read_Status();
	if(sr1 == 0) return true;
	if(sr1 == 2) return true;	// Just write enabled
	return false;	// not ready
}
//-------------------------------------------------------------------------------------------------
void SSP_WriteBuffer(uint32_t byte_address, uint8_t *pBuffer, uint32_t len)
{
	SSP_Flash_Wait_Ready();
	SSP_Flash_Write_Enable();

	Tx_Buf[0] = SSP_CMD_PAGE_PROGRAM;		// write instruction
	SSP_copy_address(byte_address);
	memcpy(Tx_Buf+4,pBuffer,len);

//	SSP_RW_Poll_Mode(4+len);
	SSP_Write_Poll_Mode(4+len);
	SSP_wait_ready = true;
//	SSP_Flash_Wait_Ready();
}
//-------------------------------------------------------------------------------------------------
#ifdef MH_SSP__WRITE_PAGE
static void SSP_WritePage(uint32_t page,uint32_t len, uint8_t *pBuffer)
{
//	printf("SSP_WritePage\r\n");

	SSP_Flash_Wait_Ready();
	SSP_Flash_Write_Enable();

	Tx_Buf[0] = SSP_CMD_PAGE_PROGRAM;		// write instruction
	uint32_t byte_address = page * SSP_FLASH_PAGE_SIZE;
	SSP_copy_address(byte_address);
	memcpy(Tx_Buf+4,pBuffer,len);

//	SSP_RW_Poll_Mode(4+len);
	SSP_Write_Poll_Mode(4+len);
	SSP_wait_ready = true;
//	SSP_Flash_Wait_Ready();
}
#endif
//-------------------------------------------------------------------------------------------------
static void SSP_ReadPage(uint32_t page,uint32_t len, uint8_t *pBuffer)
{
//	printf("SSP_ReadPage\r\n");

	SSP_Flash_Wait_Ready();

	Tx_Buf[0] = SSP_CMD_READ;		// read instruction
	uint32_t byte_address = page * SSP_FLASH_PAGE_SIZE;
	SSP_copy_address(byte_address);
	SSP_RW_Poll_Mode(4+len);

	memcpy(pBuffer,Rx_Buf+4,len);
}
//-------------------------------------------------------------------------------------------------
int SSP_ReadBuffer(uint32_t byte_address, uint8_t *pBuffer, uint32_t len)
{
	SSP_Flash_Wait_Ready();

	Tx_Buf[0] = SSP_CMD_READ;		// read instruction
	SSP_copy_address(byte_address);
	SSP_RW_Poll_Mode(4+len);		// Could get bytes_read from here
	memcpy(pBuffer,Rx_Buf+4,len);
	return 0;
}
//-------------------------------------------------------------------------------------------------
#define SSP_CMD_SECTOR_ERASE		0xD8
static void SSP_Erase_Sector(uint32_t sector)
{
// The default settings for the S25FL127S are for 64K erase.
// Since the maximum number of programmable bytes is 16Mb, there is a maximum of 256 possible 64K sectors.
// sector 0 to Sector 255.

// Since we are providing a full byte address then the sector number is multiplied by 64K to give 3 byte address.

#define SIXTYFOUR_K		(64 * 1024)

//	printf("SSP_Erase_sector:%d\r\n",sector);

	SSP_Flash_Wait_Ready();
	SSP_Flash_Write_Enable();

	Tx_Buf[0] = SSP_CMD_SECTOR_ERASE;		// Sector Erase instruction
	uint32_t byte_address = sector * SIXTYFOUR_K;
	SSP_copy_address(byte_address);
//	uint32_t t1 = us_ticker_read();
	SSP_RW_Poll_Mode(4);
	SSP_wait_ready = true;
/*
	for(;;)
	{
		uint8_t sr1 = SSP_Flash_Read_Status();
		if(sr1 == 0) break;	// Note: Could also look for error
		wait_ms(1);
	}
	uint32_t t2 = us_ticker_read();
	uint32_t elapsed_us = t2 - t1;
	printf("Erase time:%d (ms)\r\n",elapsed_us/1000);
//	ReturnToContinue();
 *
 */
}
//------------------------------------------------------------------------------
//#define MH_FLASH_TEST
#ifdef MH_FLASH_TEST
static uint8_t TestWriteBuffer[SSP_FLASH_PAGE_SIZE+2],TestReadBuffer[SSP_FLASH_PAGE_SIZE];
void SSP_Flash_Test(void)
{
	AC210_watchdog_active = false;
	printf("SSP_Flash_Test:\n\r");
	printf("Writing at %d byte pages, starting from position %d\r\n",SSP_FLASH_PAGE_SIZE,SIXTYFOUR_K);

	uint32_t n=1;
	uint32_t t1 = us_ticker_read();
	for(int pg=256;pg<512;pg++)
	{
		char *bp = (char *)TestWriteBuffer;
		for(int i=0;i<32;i++)
		{
			sprintf(bp,"%8d",n);
			bp+= 8;
			n++;
		}
		SSP_WritePage(pg,256,TestWriteBuffer);
		if(pg%16 == 0) printf("\r\n");
		printf("%6d",pg);
	}
	uint32_t t2 = us_ticker_read();
	uint32_t elapsed_us = t2 - t1;
	printf("\r\nFinished writing 256 pages, elapsed = %d (ms)\r\n",elapsed_us/1000);
	printf("Now checking\r\n");

	n = 1;
	for(int pg=256;pg<512;pg++)
	{
		char *bp = (char *)TestWriteBuffer;
		for(int i=0;i<32;i++)
		{
			sprintf(bp,"%8d",n);
			bp+= 8;
			n++;
		}
		SSP_ReadPage(pg,256,TestReadBuffer);
		if(pg%16 == 0) printf("\r\n");
		printf("%6d",pg);
		if(memcmp(TestReadBuffer,TestWriteBuffer,256))
		{
			printf("\r\nPage:%d different\r\n",pg);
		}
	}
	uint32_t t3 = us_ticker_read();
	elapsed_us = t3 - t2;
	printf("\r\nFinished reading 256 pages, elapsed = %d (ms)\r\n",elapsed_us/1000);
	AC210_watchdog_active = true;
}
//------------------------------------------------------------------------------
void SSP_Flash_Test2(void)
{
	AC210_watchdog_active = false;
	printf("SSP_Flash_Test2:\n\r");
	printf("Writing 80 byte pages, starting from position %d\r\n",SIXTYFOUR_K);

	uint32_t n=1;
	uint32_t t1 = us_ticker_read();
	uint32_t pos = SIXTYFOUR_K;
	for(int r=0;r<819;r++)
//	for(int r=0;r<5;r++)
	{
		char *bp = (char *)TestWriteBuffer;
		for(int i=0;i<10;i++)
		{
			sprintf(bp,"%8d",n);
			bp+= 8;
			n++;
		}
		AC210_ssp_flash_write(pos,TestWriteBuffer,80);
//		SSP_WriteBuffer(pos,TestWriteBuffer,80);
		if(r%20 == 0) printf("\r\n");
		printf("%6d",r);
		pos += 80;
	}
	uint32_t t2 = us_ticker_read();
	uint32_t elapsed_us = t2 - t1;
	printf("\r\nFinished writing 256 pages, elapsed = %d (ms)\r\n",elapsed_us/1000);
	printf("Now checking\r\n");

	n = 1;
	pos = SIXTYFOUR_K;

	for(int r=0;r<819;r++)
//	for(int r=0;r<5;r++)
	{
		char *bp = (char *)TestWriteBuffer;
		for(int i=0;i<10;i++)
		{
			sprintf(bp,"%8d",n);
			bp+= 8;
			n++;
		}
		AC210_ssp_flash_read(pos,TestReadBuffer,80);
//		SSP_ReadBuffer(pos,TestReadBuffer,80);
		if(r%20 == 0) printf("\r\n");
		printf("%6d",r);
		if(memcmp(TestReadBuffer,TestWriteBuffer,80))
		{
			printf("\r\nBuffer:%d different\r\n",r);
		}
		pos += 80;
	}
	uint32_t t3 = us_ticker_read();
	elapsed_us = t3 - t2;
	printf("\r\nFinished reading 256 pages, elapsed = %d (ms)\r\n",elapsed_us/1000);
}
#endif
//=================================================================================
// Note: These routines assume that there is only one eeprom we are accessing, and we
// have initialised its details. Just makes it simpler, otherwise have to remember multiple
// ids, slave addresses and positions

//------------------------------------------------------------------------------
// The idea is to make the i/o independent of page size, for portability
int AC210_ssp_flash_read(uint32_t byte_address,uint8_t *buff, uint32_t len)
{
	if(byte_address >= FLASH_SIZE)
	{
		return -1;
	}
/*
	if(len > SSP_FLASH_PAGE_SIZE)
	{
		return -2;
	}
*/
	uint32_t pos = byte_address;
	uint8_t *bp = buff;
	uint32_t bytes_left = len;

	while(bytes_left > 0)
	{
		uint32_t rlen = MIN(bytes_left,SSP_FLASH_PAGE_SIZE);
		SSP_ReadBuffer(pos,bp,rlen);
		bytes_left -= rlen;
		pos += rlen;
		bp += rlen;
	}

//	SSP_ReadBuffer(pos,buff,len);
	return 0;
}
#ifndef MH_XXX
int AC210_flash_read_part(uint32_t part_byte_address,uint8_t *buff, uint32_t len)
{
	uint32_t len1 = len;
	uint32_t part_address = part_byte_address % FLASH_PART_SIZE;
	uint32_t last_byte_address = part_address + len - 1;
	uint32_t len2 = 0;
	if(last_byte_address >= FLASH_PART_SIZE)		// Wrap?
	{
		len2 = last_byte_address - FLASH_PART_SIZE + 1;	// Yes
		len1 = len - len2;
	}

	uint32_t raw_address = part_address + FLASH_PART_OFFSET;

	int rv = AC210_ssp_flash_read(raw_address,buff,len1);		// This is normal case
	if(rv < 0) return rv;
	if(len2)
	{
		rv = AC210_ssp_flash_read(FLASH_PART_OFFSET,buff+len1,len2);		// Read from start of partition
	}
	return rv;
}
#else
int AC210_flash_read_part(uint32_t part_byte_address,uint8_t *buff, uint32_t len)
{
	uint32_t part_address = part_byte_address % FLASH_PART_SIZE;
	uint32_t raw_address = part_address + FLASH_PART_OFFSET;

	return  AC210_ssp_flash_read(raw_address,buff,len);
}
#endif
void AC210_flash_write_part(uint32_t part_byte_address,uint8_t *buff, uint32_t len)
{

	uint32_t len1 = len;
	uint32_t part_address = part_byte_address % FLASH_PART_SIZE;
	uint32_t last_byte_address = part_address + len - 1;
	uint32_t len2 = 0;
	if(last_byte_address >= FLASH_PART_SIZE)		// Wrap?
	{
		len2 = last_byte_address - FLASH_PART_SIZE + 1;	// Yes
		len1 = len - len2;
	}
	uint32_t raw_address = part_address + FLASH_PART_OFFSET;

//	AC210_ssp_raw_flash_write(raw_pos,ee_serial_write_buff,data_len);

	AC210_ssp_raw_flash_write(raw_address,buff,len1);		// This is normal case
	if(len2)
	{
		AC210_ssp_raw_flash_write(FLASH_PART_OFFSET,buff+len1,len2);		// Write to start of partition
	}
}
//------------------------------------------------------------------------------
// Somehow diags is getting corrupted during run update, causing (I think)an overwrite of flash.
// To safeguard against this, I'm adding an eeprom field of last page written,
// So that any SSP write must have an address that is greater than that.

// Note: Will need to modify to allow wrap!!!

#ifdef MH_OLD_FLASH_CHECK

int Flash_last_log_page_written=-1;

#define EEPROM_LASTPAGE_POS		512
#define EEPROM_LASTPAGE_SIZE	  4

//------------------------------------------------------------------------------
int AC210_get_flash_check_page(void)
{
	if(Flash_last_log_page_written == -1)
	{
		printf("AC210_SSP_ini_flash_check\r\n");
		int last_page;
		uint8_t* p_buf = (uint8_t *)&last_page;
		p_eeprom_i2c_pos(EEPROM_LASTPAGE_POS);
		if(p_eeprom_i2c_read(p_buf, EEPROM_LASTPAGE_SIZE))
		{
			DebugAbort("..p_eeprom_i2c_read fail");
			return -1;
		}
		if(last_page == -1 || last_page == 0)		// Quick and dirty way to initialise
		{
			last_page = FLASH_PAGE_OFFSET;
			printf("..Setting last flash page to %d\r\n",last_page);
		}
		if(last_page < FLASH_PAGE_OFFSET || last_page > FLASH_MAX_LOG_PAGES)
		{
			DebugAbort("AC210_get_flash_check_page");
			return -1;
		}
		printf("..last flash page written:%d\r\n",last_page);
		Flash_last_log_page_written = last_page;
	}
	return Flash_last_log_page_written;
}
//------------------------------------------------------------------------------
int SSP_check_byte_address(uint32_t byte_address)
{
	if(Flash_last_log_page_written == -1)
	{
		printf("SSP_check_byte_address:Initialising\r\n");
		int last_page;
		uint8_t* p_buf = (uint8_t *)&last_page;
		p_eeprom_i2c_pos(EEPROM_LASTPAGE_POS);
		if(p_eeprom_i2c_read(p_buf, EEPROM_LASTPAGE_SIZE))
		{
			DebugAbort("SSP_check_byte_address:1");
			return -1;
		}
		if(last_page == -1)		// Quick and dirty way to initialise
		{
			printf("..Setting last flash page to zero\r\n");
			last_page = 0;
		}
		if(last_page < 0 || last_page > FLASH_MAX_LOG_PAGES)
		{
			DebugAbort("SSP_check_byte_address:2");
			return -1;
		}
		printf("..last flash page written:%d\r\n",last_page);
		Flash_last_log_page_written = last_page;
	}

	int byte_address_log_page = byte_address/EELOG_PAGESIZE;		// 1024
	if(byte_address_log_page < Flash_last_log_page_written)
	{
		LogData.enabled = false;	// Turn logging off
		DebugAbort("SSP_check_byte_address:Attempt to overwrite");
		return -1;
	}
	return 0;
}
//------------------------------------------------------------------------------
void SSP_update_last_page(uint32_t byte_address)
{
	int byte_address_log_page = byte_address/EELOG_PAGESIZE;		// 1024
	if(byte_address_log_page > Flash_last_log_page_written)
	{
		Flash_last_log_page_written = byte_address_log_page;
		p_eeprom_i2c_pos(EEPROM_LASTPAGE_POS);
		uint8_t* p_buf = (uint8_t *)&Flash_last_log_page_written;
		if(p_eeprom_i2c_write(p_buf, EEPROM_LASTPAGE_SIZE)) {
			LogData.enabled = false;	// Turn logging off
			DebugAbort("SSP_update_last_page:eeprom_i2c_write");
		}
	}
}
#endif
//------------------------------------------------------------------------------
#ifdef MH_OLD_SSP_CHECK
int AC210_ssp_reset_flash_check(void)
{
	Flash_last_log_page_written = -1;		// Force a re-read

	int last_page = FLASH_PAGE_OFFSET;					// Set to 64
	uint8_t* p_buf = (uint8_t *)&last_page;
	p_eeprom_i2c_pos(EEPROM_LASTPAGE_POS);
	if(p_eeprom_i2c_write(p_buf, EEPROM_LASTPAGE_SIZE)) {
		LogData.enabled = false;	// Turn logging off
		DebugAbort("AC210_ssp_flash_ini_check:eeprom_i2c_write");
		return -1;
	}
	return 0;
}
#endif
//------------------------------------------------------------------------------
// The idea is to make the i/o independent of page size, for portability
// This allows access to sector zero

void AC210_ssp_raw_flash_write(uint32_t byte_address,uint8_t *buff, uint32_t len)
{
	uint32_t pos = byte_address;
	uint8_t *bp = buff;
	uint32_t bytes_left = len;

	while(bytes_left > 0)
	{
		uint32_t offset = pos % SSP_FLASH_PAGE_SIZE;
		uint32_t bytes_avail = SSP_FLASH_PAGE_SIZE - offset;
		uint32_t wlen = MIN(bytes_left,bytes_avail);
		SSP_WriteBuffer(pos,bp,wlen);
		bytes_left -= wlen;
		pos += wlen;
		bp += wlen;
	}
}
//------------------------------------------------------------------------------
#ifdef MH_SSP_FLASH_WRITE		// MHH:10/12/2023
static int ssp_error;
//int AC210_ssp_flash_write(uint32_t byte_address,uint8_t *buff, uint32_t len)
int AC210_ssp_flash_write(uint32_t byte_address,uint8_t *buff, uint32_t len)
{
//This is trickier than the read as the write will page wrap


#ifdef MH_OLD_SSP_CHECK
	if(byte_address >= FLASH_SIZE)
	{
		return -1;
	}
	if(SSP_check_byte_address(byte_address))
	{
		return -1;
	}
#else
	if(byte_address + len > FLASH_SIZE)
	{
		ssp_error = 1;
		return -1;
	}
	if(byte_address < FLASH_FILE_OFFSET)
	{
		ssp_error = 2;
		return -1;
	}
#endif

	AC210_ssp_raw_flash_write(byte_address,buff,len);


#ifdef MH_OLD_SSP_CHECK
	SSP_update_last_page(pos);
#endif
	return 0;
}
#endif
uint8_t SSP_test_buffer[10];
void Display_test_buffer(char *mess)
{
	PRINTF("%s\r\n",mess);
	for(int i=0;i<10;i++)
	{
		int c = SSP_test_buffer[i];
		int c2 = c;
		if(c2 <32 || c2 > 126) c2 = '_';
		PRINTF("%d %c <%d>\r\n",i,c2,c);
	}
}
void AC210_ssp_write_test(uint32_t sector)		//MHH:09/12/2023
{
	uint32_t pos = sector * 65536;
	AC210_ssp_flash_read(pos,SSP_test_buffer, 10);
	Display_test_buffer("Before write");
	AC210_ssp_raw_flash_write(pos,(uint8_t *)"ABCDEFGHIK",10);
	AC210_ssp_flash_read(pos,SSP_test_buffer, 10);
	Display_test_buffer("After write");
}
//-------------------------------------------------------------------------------------------------
void AC210_ssp_flash_erase(uint32_t sector)
{
	SSP_Erase_Sector(sector);
}
//-------------------------------------------------------------------------------------------------
// Called from AC210 wait routine. Erase sector if status is ready.
bool AC210_ssp_flash_erase_if_ready(uint32_t sector)
{
	if(SSP_Flash_Ready() == false)
	{
		return false;
	}
	SSP_Erase_Sector(sector);
	return true;
}
//-------------------------------------------------------------------------------------------------
/**
 * @brief	Main routine for SSP example
 * @return	Nothing
 */
int AC210_SSP_Init(void)
{
	/* SSP initialization */
	Board_SSP_Init(LPC_SSP1);
	Chip_SSP_Init(LPC_SSP1);		// Sets Master mode
//	Chip_SSP_SetBitRate(LPC_SSP1, 400000);	// To start with
	Chip_SSP_SetBitRate(LPC_SSP1, 12000000);	// Think 12,000,000 is max for LPC1768

// This already done in Chip_SSP_Init()

//	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
//	ssp_format.bits = SSP_BITS_8;
//	ssp_format.clockMode = SSP_CLOCK_MODE0;
//	Chip_SSP_SetFormat(LPC_SSP1, ssp_format.bits, ssp_format.frameFormat, ssp_format.clockMode);

	Chip_SSP_Enable(LPC_SSP1);

	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, 6, true);			// Set direction
	SS_N_HI;	// Set !CS line to HI state

#ifdef MH_SSP_DMA

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);

	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));	// ??
	NVIC_EnableIRQ(DMA_IRQn);
#endif

	/* Setting SSP interrupt */
//	NVIC_EnableIRQ(SSP1_IRQn);

//	Chip_SSP_SetMaster(LPC_SSP1, 1);	// Already done at Chip_SSP_Init()
//	DEBUGOUT("Master Mode\n\r");


//	appSSPMainMenu();
//	appSSPTest(SSP_DMA_SEL);


	SSP_Flash_Read_Status();
	SSP_Flash_Read_ID();

//#define MH_TEST_SSP_FLASH
#ifdef MH_TEST_SSP_FLASH
	uint32_t erase_sector = 1;

	SSP_Erase_Sector(erase_sector);
	SSP_Flash_Test2();
	ReturnToContinue();
#endif

	return 0;
}

//------------------------------------------------------------------------------
// Hex dump 250 bytes of eeprom from position value
extern uint8_t I2C_hex_buff[256];
#ifdef MH_OLD_HEX2
static void flash_hex2(int jstart)
{
	printf("  ");

	int jmax=jstart+16;
	if(jmax > 256)
	{
		jmax=256;
		for(int i=0;i<6;i++) printf("   ");
	}

	for(int j=jstart;j<jmax;j++)
	{
		uint8_t c = I2C_hex_buff[j];
		uint8_t c2 = c;
		if(c < 32 || c > '}') c2 = '_';
		Board_UARTPutChar(c2);
	}
}
#endif
//-----------------------------------------------------------------------------------
void AC210_ssp_flash_hex(WORD value)
{
	AC210_watchdog_active = false;
	PC_puts("\r\n");
    PRINTF_FLUSH;		// Wait for buffer to be output
    wait_ms(100);			// Wait for '>'

	uint32_t page = value;
	for(;;)
	{
		PRINTF("\r\nPage:%d\r\n",page);
		SSP_ReadPage(page,256, I2C_hex_buff);

		page++;

		if(AC210_hex_display_i2c_buff()) break;

	}
	AC210_watchdog_active = true;
}
#ifdef MH_XXX

	/* DeInitialize SSP peripheral */
	Chip_SSP_DeInit(LPC_SSP1);

	return 0;
}
#endif

#ifdef MH_XXX


void 	sspInit 			(void);							// SSP0 initialization
void 	sspSpeed_10kHz 		(void);							// SSP0 set to    10,000 kHz frequency
void 	sspSpeed_400kHz		(void);							// SSP0 set to   400,000 kHz frequency
void 	sspSpeed_500kHz		(void);							// SSP0 set to   500,000 kHz frequency
void 	sspSpeed_666kHz		(void);							// SSP0 set to   666,666 kHz frequency
void 	sspSpeed_800kHz		(void);							// SSP0 set to   800,000 kHz frequency
void 	sspSpeed_1MHz		(void);							// SSP0 set to       1,0 MHz frequency
void 	sspSpeed_2_5MHz		(void);							// SSP0 set to       2,5 MHz frequency
void 	sspSpeed_5MHz		(void);							// SSP0 set to       5,0 MHz frequency
void 	sspSpeed_10MHz		(void);							// SSP0 set to      10,0 MHz frequency
void 	sspSpeed_25MHz		(void);							// SSP0 set to      25,0 MHz frequency
uint8_t sspWriteOrRead		(uint8_t TransmittedDataByte);	// Write or Read 1 byte data to the SSP0 bus



// SSP1 initialization
void sspInit (void)
{
	// SSP pins settings
//	LPC_SC->PCONP 		|= (1<<21); 		// PCSSP0 The SSP0 interface power/clock control bit
	LPC_SC->PCONP 		|= (1<<10); 		// PCSSP1 The SSP1 interface power/clock control bit

//	LPC_SC->PCLKSEL1 	|= (1<<10); 		// SSP_PCLK = CCLK (PCLK = Peripheral Clock; CCLK = Core Clock)
//	LPC_SC->PCLKSEL1 	&=~(1<<11); 		// SSP_PCLK = CCLK (PCLK = Peripheral Clock; CCLK = Core Clock)
	// 4.7.3 of LPC176x manual
	LPC_SC->PCLKSEL0 	|= (1<<20); 		// SSP_PCLK = CCLK (PCLK = Peripheral Clock; CCLK = Core Clock)
	LPC_SC->PCLKSEL0 	&=~(1<<21); 		// SSP_PCLK = CCLK (PCLK = Peripheral Clock; CCLK = Core Clock)

	// SSP1:
	// p0.6 = SSEL1 (func 2)
	// p0.7 = SCK1  (func=2)
	// p0.8 = MISO1 (func=2)
	// p0.9 = MOSI1 (func=2)

	// Keep Pin 6 as GPIO for now.

	LPC_PINCON->PINSEL0 &= ~(0b11<<(6*2));	// Set func = 0 for p0.6
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, 6, true);			// Set direction

#ifdef MH_SET_SSP_FUNC
	LPC_PINCON->PINSEL0 &= ~(0b11<<(7*2));	// Set func = 0 for p0.7
	LPC_PINCON->PINSEL0 |= (0b10<<(7*2));	// Set func = 2 for p0.7

	LPC_PINCON->PINSEL0 &= ~(0b11<<(8*2));	// Set func = 0 for p0.8
	LPC_PINCON->PINSEL0 |= (0b10<<(8*2));	// Set func = 2 for p0.8

	LPC_PINCON->PINSEL0 &= ~(0b11<<(9*2));	// Set func = 0 for p0.9
	LPC_PINCON->PINSEL0 |= (0b10<<(9*2));	// Set func = 2 for p0.9
#endif

/*
	LPC_PINCON->PINSEL0 &=~(1<<30);			// pin P0.15 as the SCK0 line (SSP Clock) pin in the SSP mode
	LPC_PINCON->PINSEL0 |= (1<<31);			// pin P0.15 as the SCK0 line (SSP Clock) pin in the SSP mode
	LPC_GPIO0->FIODIR 	|= (1<<16); 		// pin P0.16 as the output
	LPC_PINCON->PINSEL1 &=~(1<<0 )|(1<<1 ); // pin P0.16 as the GPIO, manual !SS line (negative Slave Select)
	LPC_PINCON->PINSEL1 &=~(1<<2);			// pin P0.17 as the MISO0 line (Master Input Slave Output)
	LPC_PINCON->PINSEL1 |= (1<<3);			// pin P0.17 as the MISO0 line (Master Input Slave Output)
	LPC_PINCON->PINSEL1 &=~(1<<4);			// pin P0.18 as the MOSI0 line (Master Output Slave Input) of the SSP
	LPC_PINCON->PINSEL1 |= (1<<5); 			// pin P0.18 as the MOSI0 line (Master Output Slave Input) of the SSP
*/

// Not sure about this, may try both ways.

	SS_N_HI;								// pin P0.6 - Slave Select on the high level
	uint32_t CR0 = LPC_SSP1->CR0;
	// bits 3:0 = Data size select. 8 bits = 0b0111
	CR0 &= ~0b1111;		// 4 bits, could also write as 0xf;
	CR0 |= 0b0111;		// select 8 bit transfer

	// bits 5:4 = Frame format. SPI mode = 0

	CR0 &= ~(0b11 << 4);	// Just clear field

	// bit 6 = Clock Out polarity (CPOL). if 0 then clock is low between frames. If 1 then high between frames
	// we will set to zero as per example

	CR0 &= ~(0b1 << 6);		// Set to 0

	// bit 7 = Clock Out Phase (CPHA). If 0 then serial data captured on first transition from inter-frame state of the clock line
	// Use 0 as per example

	CR0 &= ~(0b1 << 7);		// Set to zero

	// bits 15:8, serial clock rate (SCR).

	CR0 &= ~(0xff << 8);	// Clear first
	CR0 |= (124<<8);		// as per example, supposed to be 400 kHz

	LPC_SSP1->CR0 = CR0;	// Update register

/*
	// Set the Control Register 0
	LPC_SSP1->CR0 |= (1<<2)|(1<<1)|(1<<0);	// Data Size Select; 8-bit transfer
	LPC_SSP1->CR0 &=~(1<<3);				// Data Size Select; 8-bit transfer
	LPC_SSP1->CR0 &=~(1<<5)|(1<<4);			// Frame Format; SPI
	LPC_SSP1->CR0 &=~(1<<6);				// Clock Out Polarity; SSP controller maintains the bus clock low between frames
	LPC_SSP1->CR0 &=~(1<<7);				// Clock Out Phase; SSP controller captures serial data on the first clock transition of
											// the frame, that is, the transition away from the inter-frame state of the clock line
	LPC_SSP1->CR0 |= (124<<8);				// Serial Clock Rate; frequency is PCLK / (CPSDVSR × [SCR+1]); 400 kHz
*/

	// Set the Control Register 1

	uint32_t CR1 = LPC_SSP1->CR1;

// Bit 0 = Loop Back Mode. 0 = off

	CR1 &= ~(1<<0);		// LBM=0

// Bit 1 = SSP enable. Only set to 1 if other devices on the same bus.

	CR1 &= ~(1<<1);		// SSP disabled

// Bit 2 = Master/Slave mode. 0 = Master

	CR1 &= ~(1<<2);		// MS=0 (Master)

// Bit 3 = Slave Output Disable (SOD). Not relevant in Master mode.

	CR1 &= ~(1<<3);		// SOD=0

	LPC_SSP1->CR1 = CR1;	// Update register

/*
	LPC_SSP1->CR1 &=~(1<<0);				// Loop Back Mode; During normal operation
	LPC_SSP1->CR1 &=~(1<<2);				// Master/Slave Mode; The SSP controller acts as a master on the bus, driving the
											// SCLK, MOSI, and SSEL lines and receiving the MISO line
*/

	LPC_SSP1->CPSR = 2;						// Clock Prescale Register; This even value between 2 and 254, by which SSP_PCLK is divided
											// to yield the prescaler output clock

	LPC_SSP1->CR1 |= (1<<1);				// SSP Enable; The SSP controller will interact with other devices on the serial bus

//	LPC_GPIO0->FIOSET |= (1<<6);			// set hi
//	wait_ms(100);
//	LPC_GPIO0->FIOCLR |= (1<<6);			// set lo

}
//------------------------------------------------------------------------------------------------------------------------
// Write or ReadS 1 byte data to/from the SSP1 bus
uint8_t sspWriteOrRead (uint8_t TransmittedDataByte)
{
	uint8_t ReceivedDataByte = 0;			// temporary variable

	LPC_SSP1->DR = TransmittedDataByte;		// DR - Data Register
	while((LPC_SSP1->SR & (1 << 2)) == 0); 	// wait until byte is received
	ReceivedDataByte = LPC_SSP1->DR;
	return ReceivedDataByte;
}
//------------------------------------------------------------------------------------------------------------------------
// READ IDENTIFICATION - COMMAND
uint32_t ReadID_CMD (void)
{
	///////////////////////////////////////////////////////////
	////////// READ IDENTIFICATION - COMMAND - BEGIN //////////
	///////////////////////////////////////////////////////////

	uint8_t  Tab [4];		// ReadID table
	uint32_t i 		= 0;	//
	uint32_t ReadID = 0;

//	SS_N_HI;				// Set !CS line to HI state
	SS_N_LO;				// Set !CS line to LO state
	sspWriteOrRead (0x9F);	// Read Identification instruction
	for (i=0; i<4; i++) { Tab [i] = sspWriteOrRead (0xFF); }	// Dummy byte with DataByteValue variable storages data from memory
//	Timer0_MicroSeconds (TimeDelay_BeforeCSHi);
	SS_N_HI;				// Set !CS line to HI state

	ReadID = (Tab [0] << 24) | (Tab [1] << 16) | (Tab [2] << 8) | (Tab [3] << 0);

//	for (i=0; i<4; i++)	{ UART0_SendByte (Tab [i]); }
//	for (i=0; i<4; i++) { Tab [i] = 0x00; }

	return (ReadID);
	/////////////////////////////////////////////////////////
	////////// READ IDENTIFICATION - COMMAND - END //////////
	/////////////////////////////////////////////////////////
}
//------------------------------------------------------------------------------------------------------------------------
// WRITE ENABLE - COMMAND
void WriteEnable_CMD (void)
{
	////////////////////////////////////////////////////////////
	////////////// WRITE ENABLE - COMMAND - BEGIN //////////////
	////////////////////////////////////////////////////////////

//	SS_N_HI;	// Set !CS line to HI state
	SS_N_LO;	// Set !CS line to LO state
	sspWriteOrRead (0x06);	// Write Disable instruction
//	Timer0_MicroSeconds (TimeDelay_BeforeCSHi);
	SS_N_HI;	// Set !CS line to HI state

	////////////////////////////////////////////////////////////
	/////////////// WRITE ENABLE - COMMAND - END ///////////////
	////////////////////////////////////////////////////////////
}
//------------------------------------------------------------------------------------------------------------------------
void WriteByte (uint32_t DataByteAddress, uint8_t DataByteValue)
{
	uint8_t DataByteAddres_Byte2 = 0;	// MSB
	uint8_t DataByteAddres_Byte1 = 0;
	uint8_t DataByteAddres_Byte0 = 0;	// LSB

	DataByteAddres_Byte2 = ( (DataByteAddress & 0x00FF0000) >> 16 );
	DataByteAddres_Byte1 = ( (DataByteAddress & 0x0000FF00) >>  8 );
	DataByteAddres_Byte0 = ( (DataByteAddress & 0x000000FF) >>  0 );

	///////////////////////////////////////////////////////////////////////
	/////////////// PAGE WRITE/BYTE WRITE - COMMAND - BEGIN ///////////////
	///////////////////////////////////////////////////////////////////////

	WriteEnable_CMD ();		// WRITE ENABLE - COMMAND

	// Byte Write Sequence
	SS_N_LO;				// Set !CS line to LO state
	sspWriteOrRead (0x02);	// Page Program write 1 byte command/instruction
	sspWriteOrRead (DataByteAddres_Byte2); 	// ( (Address & 0x00FF0000) >> 16);		// Address bits 23-16
	sspWriteOrRead (DataByteAddres_Byte1);	// ( (Address & 0x0000FF00) >>  8);		// Address bits 15- 8
	sspWriteOrRead (DataByteAddres_Byte0);	// ( (Address & 0x000000FF) >>  0);		// Address bits  7- 0
	sspWriteOrRead (DataByteValue);			// Data byte
//	Timer0_MicroSeconds (TimeDelay_BeforeCSHi);
	SS_N_HI;				// Set !CS line to HI state

	////////////////////////////////////////////////////////////////////////
	//////////////// PAGE PROGRAM/BYTE WRITE - COMMAND - END ///////////////
	////////////////////////////////////////////////////////////////////////
}
//------------------------------------------------------------------------------------------------------------------------
uint8_t ReadByte (uint32_t DataByteAddress)
{
	uint8_t DataByteAddres_Byte2 = 0;	// MSB
	uint8_t DataByteAddres_Byte1 = 0;
	uint8_t DataByteAddres_Byte0 = 0;	// LSB
	uint8_t DataByteValue		 = 0;

	DataByteAddres_Byte2 = ( (DataByteAddress & 0x00FF0000) >> 16 );
	DataByteAddres_Byte1 = ( (DataByteAddress & 0x0000FF00) >>  8 );
	DataByteAddres_Byte0 = ( (DataByteAddress & 0x000000FF) >>  0 );

	/////////////////////////////////////////////////////////////////
	/////////////// READ DATA BYTES - COMMAND - BEGIN ///////////////
	/////////////////////////////////////////////////////////////////

	// Byte Read Sequence
	SS_N_LO;				// Set !CS line to LO state
	sspWriteOrRead (0x03);	// Read Data Bytes write 1 byte command/instruction
	sspWriteOrRead (DataByteAddres_Byte2); 	// ( (Address & 0x00FF0000) >> 16);		// Address bits 23-16
	sspWriteOrRead (DataByteAddres_Byte1);	// ( (Address & 0x0000FF00) >>  8);		// Address bits 15- 8
	sspWriteOrRead (DataByteAddres_Byte0);	// ( (Address & 0x000000FF) >>  0);		// Address bits  7- 0
	DataByteValue = sspWriteOrRead (0xFF);	// Dummy byte with DataByteValue variable storages data from memory
//	Timer0_MicroSeconds (TimeDelay_BeforeCSHi);
	SS_N_HI;				// Set !CS line to HI state

	///////////////////////////////////////////////////////////////
	/////////////// READ DATA BYTES - COMMAND - END ///////////////
	///////////////////////////////////////////////////////////////

	return (DataByteValue);
}
//------------------------------------------------------------------------------------------------------------------------
// READ STATUS REGISTER - COMMAND
uint8_t ReadStatusReguster_CMD (void)
{
	////////////////////////////////////////////////////////////
	////////// READ STATUS REGISTER - COMMAND - BEGIN //////////
	////////////////////////////////////////////////////////////

	uint8_t  StatusRegister_Read				= 0;
	uint8_t  WriteInProgressBit_Read			= 0;	// Zero   bit (LSb) of Status Register
	uint8_t  WriteEanbleLatchBit_Read 			= 0;	// First  bit of Status Register
	uint8_t  BlockProtectBits_Read				= 0;	// Second, Third and Fourth bits of Status Register
	uint8_t  StatusRegisterWriteProtectBit_Read	= 0;	// Seventh bit (MSb) of Status Register

//	SS_N_HI;	// Set !CS line to HI state
	SS_N_LO;	// Set !CS line to LO state
	sspWriteOrRead (0x05);	// Read Status Register instruction
	StatusRegister_Read = sspWriteOrRead (0xFF); 	// Dummy byte with DataByteValue variable storages data from memory
//	Timer0_MicroSeconds (TimeDelay_BeforeCSHi);
	SS_N_HI;	// Set !CS line to HI state

	WriteInProgressBit_Read				= ( (StatusRegister_Read & (0x01)) >> 0 );		// Zero   bit (LSb) of Status Register
	WriteEanbleLatchBit_Read 			= ( (StatusRegister_Read & (0x02)) >> 1 );		// First  bit of Status Register
	BlockProtectBits_Read				= ( (StatusRegister_Read & (0x1C)) >> 2 );		// Second, Third and Fourth bits of Status Register
	StatusRegisterWriteProtectBit_Read	= ( (StatusRegister_Read & (0x80)) >> 7 );		// Seventh bit (MSb) of Status Register

//	UART0_SendByte (StatusRegister_Read);
//	UART0_SendByte (WriteInProgressBit_Read);
//	UART0_SendByte (WriteEanbleLatchBit_Read);
//	UART0_SendByte (BlockProtectBits_Read);
//	UART0_SendByte (StatusRegisterWriteProtectBit_Read);

	return (StatusRegister_Read);
	//////////////////////////////////////////////////////////
	////////// READ STATUS REGISTER - COMMAND - END //////////
	//////////////////////////////////////////////////////////
}
//------------------------------------------------------------------------------------------------------------------------
int AC210_SSP_Init(void)
{
//	sspInit();

	Board_SSP_Init(LPC_SSP1);
	Chip_SSP_Init(LPC_SSP1);		// Sets Master mode
	Chip_SSP_SetBitRate(LPC_SSP1, 400000);	// To start with

// This already done in Chip_SSP_Init()

//	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
//	ssp_format.bits = SSP_BITS_8;
//	ssp_format.clockMode = SSP_CLOCK_MODE0;
//	Chip_SSP_SetFormat(LPC_SSP1, ssp_format.bits, ssp_format.frameFormat, ssp_format.clockMode);

	Chip_SSP_Enable(LPC_SSP1);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, 6, true);			// Set direction
	SS_N_HI;	// Set !CS line to HI state

	uint32_t FlashID 	= 0;
	uint8_t sr = ReadStatusReguster_CMD ();
	WriteEnable_CMD ();		// WRITE ENABLE - COMMAND
	uint8_t sr2 = ReadStatusReguster_CMD ();
	FlashID = ReadID_CMD ();
	WriteByte (10000, 123);
	for(int i=0;i<10;i++)
	{
		uint8_t sr1 = ReadStatusReguster_CMD ();
		printf("(%d) SR1 after write:%02x\r\n",i,sr1);
		if(sr1 == 0) break;
		wait_ms(100);
	}
	uint8_t b = ReadByte(10000);

	return 0;
}
#endif



/**
 * @}
 */
