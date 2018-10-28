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
#include "timer_b.h"

#include "tick.h"

static tick_t tick = 0;

extern unsigned int  flag;

bool debounce_switch_s1();
bool debounce_switch_s2();
bool debounce_switch_sw1();
bool debounce_switch_sw2();

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

    Timer_B_initUpModeParam initUpParam = { 0 };
        initUpParam.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;                       // Use ACLK (slower clock)
        initUpParam.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_8;           // Input clock = SMCLK / 8= 1MHz
        initUpParam.timerPeriod = 1000;                                     // Period 1MHz/1000 =1000Hz -> WRITTEN IN CCR0
        //initUpParam.timerInterruptEnable_TAIE = TIMER_B_TAIE_INTERRUPT_DISABLE;   // Enable TAR -> 0 interrupt
        initUpParam.captureCompareInterruptEnable_CCR0_CCIE =
                TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE;                               // Enable CCR0 compare interrupt
        initUpParam.timerClear = TIMER_B_DO_CLEAR;                                // Clear TAR & clock divider
        initUpParam.startTimer = false;   // Don't start the timer, yet

    Timer_B_initUpMode (TIMER_B0_BASE, &initUpParam);
    Timer_B_clearTimerInterrupt( TIMER_B0_BASE );                                 // Clear TA0IFG
    Timer_B_clearCaptureCompareInterrupt( TIMER_B0_BASE,
        TIMER_B_CAPTURECOMPARE_REGISTER_0                                         // Clear CCR0IFG
    );

    Timer_B_startCounter(
        TIMER_B0_BASE,
        TIMER_B_UP_MODE
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
	//TIMER_B_disableInterrupt(TIMER_B0_BASE);
	tickValue = tick;
	//TIMER_B_enableInterrupt(TIMER_B0_BASE);

	return tickValue;
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
void TIMER0_B0_ISR(void) __attribute__((interrupt(TIMER0_B0_VECTOR)));
#endif
void TIMER0_B0_ISR(void)
{
    tick++;                         //Timer Overflow
    if (debounce_switch_s2())
        flag |= BIT0;

    if (debounce_switch_s1())
        flag |= BIT1;

    if (debounce_switch_sw1())
        flag |= BITA;

    if (debounce_switch_sw2())
        flag |= BITB;
}
