/*
 * ac210_ssp.h
 *
 *  Created on: 3/01/2017
 *      Author: Murray
 */

#ifndef AC210_SSP_H_
#define AC210_SSP_H_

int AC210_SSP_Init(void);
void AC210_ssp_flash_erase(uint32_t sector);
int AC210_ssp_flash_write(uint32_t pos,uint8_t *buff, uint32_t len);
void AC210_ssp_raw_flash_write(uint32_t byte_address,uint8_t *buff, uint32_t len);
int AC210_ssp_flash_read(uint32_t pos,uint8_t *buff, uint32_t len);
bool AC210_ssp_flash_erase_if_ready(uint32_t sector);
int AC210_flash_read_part(uint32_t part_byte_address,uint8_t *buff, uint32_t len);	// MHH:25/12/2023
void AC210_flash_write_part(uint32_t part_byte_address,uint8_t *buff, uint32_t len);	// MHH:29/12/2023


#endif /* AC210_SSP_H_ */
