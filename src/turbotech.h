/*
 * turbotech.h
 *
 *  Created on: 5/10/2026
 *      Author: OEM
 */

#ifndef TURBOTECH_H_
#define TURBOTECH_H_

struct RC2_STATUS
{
	uint8_t flags;
	uint8_t dir_pin;
	uint16_t hub_voltage;
	uint16_t hub_temperature;
	uint16_t hub_adc_temperature;
	uint16_t hub_adc_last_temperature;
	uint16_t hub_dac0_v;
	int16_t enc_rpm;
	uint16_t last_enc_pos;
};

extern struct RC2_STATUS RC2_status;


#endif /* TURBOTECH_H_ */
