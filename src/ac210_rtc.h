/*
 * ac210_rtc.h
 *
 *  Created on: 29/12/2018
 *      Author: Murray
 */

#ifndef AC210_RTC_H_
#define AC210_RTC_H_

int DS3231_i2c_init(void);
int DS3231_i2c_read(int len);
int DS3231_i2c_read_7(void);
int DS3231_i2c_write(const uint8_t *buff, int pos, uint8_t len);

extern uint8_t ds3231_buff_in[];

#endif /* AC210_RTC_H_ */
