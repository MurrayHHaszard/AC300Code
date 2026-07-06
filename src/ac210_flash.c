/*
===============================================================================
 Name        : ac210_flash.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/
// Derived from mh_flash_iap.c

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#include "ac210_global.h"

// TODO: insert other include files here

// TODO: insert other definitions and declarations here

/* Last sector address */
#define START_ADDR_LAST_SECTOR  0x00000000
//#define START_ADDR_LAST_SECTOR  0x00078000

/* Size of each sector */
#define SECTOR_SIZE             1024

/* LAST SECTOR */
#define IAP_LAST_SECTOR         0

/* Number of bytes to be written to the last sector */
#define IAP_NUM_BYTES_TO_WRITE  256
//#define IAP_NUM_BYTES_TO_WRITE  32	// Does not work!!

/* Number elements in array */
#define ARRAY_ELEMENTS          (IAP_NUM_BYTES_TO_WRITE / sizeof(uint32_t))

/* Data array to write to flash */
static uint32_t src_iap_array_data[ARRAY_ELEMENTS];

int AC210_flash(void)
{
	uint32_t *p_dword;
	uint8_t ret_code;
	uint32_t part_id;

    DPRINTF("ac210_flash:\r\n");

    // First copy in existing sector zero

    p_dword = (uint32_t *)0;

	for (int i = 0; i < ARRAY_ELEMENTS; i++) {
		src_iap_array_data[i] = *p_dword++;
	}
#ifdef MH_XXX
	for(int i=0;i<8;i++)
	{
		DPRINTF("Address:%8x,value:%8x\r\n",i*4,src_iap_array_data[i]);
	}
#endif
	if(src_iap_array_data[7] == 0)
	{
		DPRINTF("Checksum already zero\r\n");
		return 0;
	}
	src_iap_array_data[7] = 0;		// This is where checksum is kept

	/* Read Part Identification Number*/
	part_id = Chip_IAP_ReadPID();
	DPRINTF("Part ID is: %x\r\n", part_id);
	wait_ms(100);

	/* Disable interrupt mode so it doesn't fire during FLASH updates */
	__disable_irq();

	/* IAP Flash programming */
	/* Prepare to write/erase the last sector */
	ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		printf("Chip_IAP_PreSectorForReadWrite() failed to execute, return code is: %x\r\n", ret_code);
		return 1;
	}

#ifdef MH_ERASE_SECTOR
	/* Erase the last sector */
	ret_code = Chip_IAP_EraseSector(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT("Chip_IAP_EraseSector() failed to execute, return code is: %x\r\n", ret_code);
	}

	/* Prepare to write/erase the last sector */
	ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT("Chip_IAP_PreSectorForReadWrite() failed to execute, return code is: %x\r\n", ret_code);
	}
#endif

	/* Write to the last sector */
	ret_code = Chip_IAP_CopyRamToFlash(START_ADDR_LAST_SECTOR, src_iap_array_data, IAP_NUM_BYTES_TO_WRITE);

	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		printf("Chip_IAP_CopyRamToFlash() failed to execute, return code is: %x\r\n", ret_code);
		return 1;
	}

	/* Re-enable interrupt mode */
	__enable_irq();

	/* Start the signature generator for the last sector */
	Chip_FMC_ComputeSignatureBlocks(START_ADDR_LAST_SECTOR, (SECTOR_SIZE / 16));

	/* Check for signature generation completion */
	while (Chip_FMC_IsSignatureBusy()) {}

	/* Get the generated FLASH signature value */
	printf("Generated signature for the last sector is: %x \r\n", Chip_FMC_GetSignature(0));
    return 0 ;
}
