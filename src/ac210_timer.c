// mhtimer.c - attempt to create mbed like timer and wait routines.

//#include "board.h"

//#include <cr_section_macros.h>

// TODO: insert other include files here

//#include <string.h>
#include "ac210_timer.h"

#include "ac210_us_ticker.h"
//#include "mhcan3.h"
// Note: This is not efficient in a multitasking environment. There may be an RTOS call which waits
// more efficiently. Eg we could wait for number of system ticks, then return when remainder is less than a tick

void wait_us(uint32_t usecs)
{
	uint32_t start_usecs = us_ticker_read();
	uint32_t end_usecs   = start_usecs + usecs;
	if(end_usecs < start_usecs) {
		while(us_ticker_read() > start_usecs);	// wait until wrap around
	}
	while(us_ticker_read() < end_usecs);		// loop until end

}
void wait_ms(uint32_t msecs)
{
	wait_us(msecs*1000);
}

void Timer_start(TIMER_t *pTIMER) {
    pTIMER->_start = us_ticker_read();
    pTIMER->_running = 1;
}

void Timer_stop(TIMER_t *pTIMER) {
    pTIMER->_time += Timer_slicetime(pTIMER);
    pTIMER->_running = 0;
}

uint32_t Timer_read_us(TIMER_t *pTIMER) {
    return pTIMER->_time + Timer_slicetime(pTIMER);
}

float Timer_read(TIMER_t *pTIMER) {
    return (float)Timer_read_us(pTIMER) / 1000000.0f;
}

uint32_t Timer_read_ms(TIMER_t *pTIMER) {
    return Timer_read_us(pTIMER) / 1000;
}

uint32_t Timer_slicetime(TIMER_t *pTIMER) {
    if (pTIMER->_running) {
        return us_ticker_read() - pTIMER->_start;
    } else {
        return 0;
    }
}

void Timer_reset(TIMER_t *pTIMER) {
    pTIMER->_start = us_ticker_read();
    pTIMER->_time = 0;
}

