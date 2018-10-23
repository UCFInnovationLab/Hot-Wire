/*
 * tick.c
 *
 *  Created on: May 27, 2013
 *      Author: Collin
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "timer_a.h"

#include "tick.h"

static tick_t tick = 0;

/* tick_init()
 *
 * Initializes the tick hardware components (timer_a)
 *
 * Arguments:
 * NONE
 *
 * Returns:
 * NONE
 *
 */
void tick_init()
{
	tick = 0;

    Timer_A_initUpModeParam initUpParam = { 0 };
        initUpParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                       // Use ACLK (slower clock)
        initUpParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_8;           // Input clock = SMCLK / 8= 1MHz
        initUpParam.timerPeriod = 10000;                                     // Period 1MHz/10000 =100Hz -> WRITTEN IN CCR0
        initUpParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;   // Enable TAR -> 0 interrupt
        initUpParam.captureCompareInterruptEnable_CCR0_CCIE =
                TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;                               // Enable CCR0 compare interrupt
        initUpParam.timerClear = TIMER_A_DO_CLEAR;                                // Clear TAR & clock divider
        initUpParam.startTimer = false;   // Don't start the timer, yet

    Timer_A_initUpMode (TIMER_A2_BASE, &initUpParam);
    Timer_A_clearTimerInterrupt( TIMER_A2_BASE );                                 // Clear TA0IFG
    Timer_A_clearCaptureCompareInterrupt( TIMER_A2_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0                                         // Clear CCR0IFG
    );

    Timer_A_startCounter(
        TIMER_A2_BASE,
        TIMER_A_UP_MODE
    );
}

/* tick_getTime()
 *
 * Get the current time in ticks.
 *
 * Arguments:
 * NONE
 *
 * Returns:
 * The current time in ticks.
 *
 */
tick_t tick_getTime()
{
	tick_t tickValue = 0;

	/* Atomically retrieve the tick counter. */
	//TIMER_A_disableInterrupt(TIMER_A2_BASE);
	tickValue = tick;
	//TIMER_A_enableInterrupt(TIMER_A2_BASE);

	return tickValue;
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
void TIMER2_A0_ISR(void) __attribute__((interrupt(TIMER2_A0_VECTOR)));
#endif
void TIMER2_A0_ISR(void)
{
    tick++;                         //Timer Overflow
}
