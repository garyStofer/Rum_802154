/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/
/*
  $Id: sleep.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/
#include "mac.h"
#include "system.h"
#include "sleep.h"
#include "radio.h"
#include "serial.h"
#include "sensors.h"
#include <avr/sleep.h>
#include <avr/wdt.h>


#if ((NODETYPE == ENDDEVICE) && (RUMSLEEP == 1) )

/**
   @defgroup sleep Sleep functions

   Function to support the sleeping of the node.  Currently, this code
   only runs on the end nodes, as part of the sensor application.  It
   can be called at any time to put the node to sleep, however.



   The low-power oscillator must be driven from an on-board 32.768 KHz crystal.

   @{
*/



/**
     Put the node to sleep.  This function must put all the parts of the
   hardware to sleep, and wake them up.

   param:  tenthSeconds The time to sleep in tenths of a second.

   @ingroup sleep
*/
void nodeSleep(u16 tenthSeconds)
{
    // Check to see if we really should sleep
    if (!macConfig.sleeping)
        // Just return, rather than sleeping
        return;

    // ************** Power down the other board peripherals
    Leds_off();

    // ************** Power down the radio
    // wait for radio to be done
    u8 state = BUSY_TX_ARET;
    while (state == BUSY_TX_ARET ||
           state == BUSY_RX_AACK)
        state = radioGetTrxState();

    // Now put radio to sleep
    radioEnterSleepMode();


// TODO: figure out what needs to be put to sleep to minimize current consumption
// ************** Power down the on-chip modules
// PRR = 0xbf; ??? 
// Disable ADC
//        ADCSRA &= ~(1 << ADEN);
// Turn off comparator
//        ACSR |= (1 << 
    
// turn off ports  etc

// Turn off BOD

// This should only be done once -- No need to do it over agin 
/*
        AVR_ENTER_CRITICAL_REGION();
#define BODS  6
#define BODSE 5
        MCUCR  |= (1 << BODSE) | (1<< BODS);
        MCUCR  &= ~(1 << BODSE);
        AVR_LEAVE_CRITICAL_REGION();
    
*/


    // ************** Set the timer to wake up
	// Set TIMER2 Asyncronous Mode.
	ASSR |= (1 << AS2);
	// Set TIMER2 Prescaler to 1024.
	TCCR2B |= ((1 << CS22)|(1 << CS21)|(1 << CS20));
	// Wait for TCNT2 write to finish.
	while(ASSR & (1 << TCR2BUB))
		;


    // Sleep as many times as needed to sleep for the full time
    while (tenthSeconds)
    {
		// Set TIMER2 output compare register from user.
		if (tenthSeconds > 75)
		{
			// Just decrement by the max timeout
			OCR2A = 240; // 7.5 seconds, max timeout
			tenthSeconds -= 75;
		}
		else
		{
			// Can measure the remaining time in one timer cycle

			tenthSeconds = tenthSeconds * 16 / 5;
			if (!tenthSeconds)
				tenthSeconds++;
			OCR2A = tenthSeconds;
			tenthSeconds = 0;
		}
		// Wait for OCR2 write to finish.
		while(ASSR & (1 << OCR2AUB))
			;
		// Reset TIMER2 timer counter value.
		TCNT2 = 0;
		// Wait for TCNT2 write to finish before entering sleep.
		while(ASSR & (1 << TCN2UB))
			;

		// Clear interrupt flag
		TIFR2 |= (1 << OCF2A);
		// Enable TIMER2 output compare interrupt.
		TIMSK2 |= (1 << OCIE2A);


        // ************** Go to sleep
        AVR_ENTER_CRITICAL_REGION();
        set_sleep_mode( SLEEP_MODE_PWR_SAVE);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
        AVR_LEAVE_CRITICAL_REGION();

        wdt_disable();
    }

    // ************** Awake now, wake up everything

 //    PRR = 0x03; ??


    if (SERIAL)
        serial_init(NULL);

    debugMsgStr("\r\nNode slept");
    if ( macConfig.associated)
        HAL_ADC_INIT();

    // Bring SPI port back up (must re-init after sleep)
    radio_spi_init();

    // Wake up radio.
    radioLeaveSleepMode();

    // Set RF212 to 250KB mode.
// TODO: do I need to call this??
    //radioSetup900();

    radioSetTrxState(PLL_ON);
}


/*
  TIMER2 Real Time Clock Sleep ISR function.
*/
ISR(TIMER2_COMPA_vect)
{
    // Disable TIMER2 output compare interrupt.
    TIMSK2 &= ~(1 << OCIE2A);
}

/**
Watchdog timer vector handler.
*/
ISR(WDT_vect)
{
    wdt_disable();
}


#else
// For coord, provide dummy function so that code compiles
void nodeSleep(u16 seconds) {}
#endif


