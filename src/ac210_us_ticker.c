/*
 * us_ticker.c
 *
 *  Created on: 20/10/2014
 *      Author: Murray
 */
// Copy of mbed routines.

//#include "us_ticker_api.h"
//#include "LPC17xx.h"		// This header from mbed
//#include "cmsis_nvic.h"		// This header from mbed too

#include "chip.h"

#define LPC_SC				LPC_SYSCTL
#define LPC_TIM_TypeDef		LPC_TIMER_T
#define LPC_TIM3_BASE		LPC_TIMER3_BASE

#define US_TICKER_TIMER      ((LPC_TIM_TypeDef *)LPC_TIM3_BASE)
#define US_TICKER_TIMER_IRQn TIMER3_IRQn

int us_ticker_inited = 0;
void us_ticker_init(void) {
    if (us_ticker_inited) return;
    us_ticker_inited = 1;

    LPC_SC->PCONP |= 1 << 23; // Clock TIMER_3

    US_TICKER_TIMER->CTCR = 0x0; // timer mode
    uint32_t PCLK = SystemCoreClock / 4;

    US_TICKER_TIMER->TCR = 0x2;  // reset

    uint32_t prescale = PCLK / 1000000; // default to 1MHz (1 us ticks)
    US_TICKER_TIMER->PR = prescale - 1;
    US_TICKER_TIMER->TCR = 1; // enable = 1, reset = 0

#ifdef TICKER_INTERRUPT
    NVIC_SetVector(US_TICKER_TIMER_IRQn, (uint32_t)us_ticker_irq_handler);
    NVIC_EnableIRQ(US_TICKER_TIMER_IRQn);
#endif
}

uint32_t us_ticker_read() {
    if (!us_ticker_inited)
        us_ticker_init();

    return US_TICKER_TIMER->TC;
}
#ifdef TICKER_INTERRUPT

void us_ticker_set_interrupt(unsigned int timestamp) {
    // set match value
    US_TICKER_TIMER->MR0 = timestamp;
    // enable match interrupt
    US_TICKER_TIMER->MCR |= 1;
}

void us_ticker_disable_interrupt(void) {
    US_TICKER_TIMER->MCR &= ~1;
}

void us_ticker_clear_interrupt(void) {
    US_TICKER_TIMER->IR = 1;
}
#endif
