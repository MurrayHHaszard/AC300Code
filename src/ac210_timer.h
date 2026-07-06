/*
 * mhtimer.h
 *
 *  Created on: 20/10/2014
 *      Author: Murray
 */

// function definitions for timer routines.

#ifndef AC210_TIMER_H_
#define AC210_TIMER_H_

#include <stdint.h>

typedef struct
{
	uint32_t	_start;
	uint32_t	_running;
	uint32_t	_time;
} TIMER_t;

void wait_us(uint32_t);
void wait_ms(uint32_t);

void Timer_start(TIMER_t *);
void Timer_stop(TIMER_t *);
uint32_t Timer_read_us(TIMER_t *);
float Timer_read(TIMER_t *);
uint32_t Timer_read_ms(TIMER_t *);
uint32_t Timer_slicetime(TIMER_t *);
void Timer_reset(TIMER_t *);
extern uint32_t us_ticker_read(void);

#endif /* AC210_TIMER_H_ */
