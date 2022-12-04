/*
 * timer_counter.c
 *
 *  Created on: Aug 26, 2022
 *      Author: len12
 */

#include <msp430.h>
#include "timer_counter.h"

#define ACLK 0x0100   // Timer ACLK source
#define UP 0x0010     // Timer UP mode

void setup_timer0(void)
{
    TA0CCR0  = 10;                      // 10 * 100us = 1000us = 0.001 second
    TA0CTL   = ACLK + UP;                // Set ACLK, UP mode, ACLK source from VLO = 10 kHz
    TA0CCTL0 = CCIE;                     // Enable interrupt for Timer_0
    _BIS_SR(GIE);
    counter = 0;
}

//******************************************************************************
// Timer0 Interrupt ************************************************************
//******************************************************************************
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_ISR (void)
{
    counter++;
}
