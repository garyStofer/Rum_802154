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
  $Id: main.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

#include <stdlib.h> 
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <compat/twi.h>


#include "stdbool.h"
#include "radio.h"
#include "mac.h"
#include "mac_event.h"
#include "timer.h"
#include "serial.h"
#include "sensors.h"
#include "mac_start.h"
#include "mac_data.h"
#include "mac_associate.h"
#include "system.h"
#include "hal_avr.h"
#include "EE_prom.h"
#include "avr_sixlowpan.h"
#include "i2c.h"

/* On restart WDT will be enabled with shortest timeout if previously
   enabled. Must turn it off before anything else, attempting to
   disable within the main() function will take too long. */
   
void disable_wdt(void) \
      __attribute__((naked)) \
      __attribute__((section(".init3")));
void disable_wdt(void)
{
	MCUSR = 0;
	wdt_disable();
}

// set PC to here in the debugger to cause a reset/restart
#define SOFT_RESET
#ifdef SOFT_RESET
void
Reset_via_Watchdog( void)
{
    wdt_enable(WDTO_15MS);
    for(;;);
}
#endif
void 
Device_init()
{

    halSetupClock();

	LED_INIT(); 
	
	// Init the button
    BUTTON_SETUP();
	
	// Init the timer system for the MAC
    timerInit();

// init serial
    if (SERIAL)
        serial_init(NULL);

	sei();		// enable global interupts 

	// Init radio -- calls radio_hal_init()
    if ( radioInit() != RADIO_SUCCESS)
	{
		   debugMsgStr("\r\n Wrong BAND or unsupported RF device");
		   // Lock the CPU Nothing else to do when we can't talk to the radio chip
		   Leds_on();
		   while (1);
	}

    if (APP==SENSOR)
    {
        i2c_init(200000UL);		// 200Khz clock

    	if (SENSOR_TYPE==SENSOR_TMP100 )
    		TMP100_init(TMP100_12_BitCONF);

    	if ( SENSOR_TYPE==SENSOR_BARO )
    		HP03_init();

    	if ( SENSOR_TYPE==SENSOR_BMP085 )
    	    		BMP085_init();

     	if ( SENSOR_TYPE==SENSOR_HYG ) // Make PD4 an output
     		DDRD  |=  0x10;


      	Sensor_HW_Init();		// analog sensors
    }



    blink_red(200); // Blip the LED once on powerup

	
}



/**
     Main function of the program.
*/
int main(void)
{
   	Device_init();

	// If the EEPROM is cleared, init it to something useful
	checkEeprom();

	// Init the mac and stuff
#if (NODETYPE == COORD)
	{
		debugMsgStr("\r\nStartup: as Coordinator");
		macFindClearChannel();
	// appClearChanFound() will be called
	}
#else
	// End or Router node, start scanning for a coordinator
	macStartScan();


#	if (IPV6LOWPAN == 1)
		sixlowpan_init();
		sixlowpan_application_init();
#	endif
#endif

#if( NODETYPE == BC_ENDDEVICE )
		// A BC only node has no way of knowing if the network is still alive and on the same channel unless it periodically
		// does a re-sacn of all channels or the channel & PANDID is fixed
	  StartSensorBC_interval();
#endif

     // Main forever loop for the application.
    for(;;)
    {
        // Turn interrupts on if they are off.  They should always be on
        // at this point in the loop, so just turn them on every time, in
        // case interrupts were inadvertently turned off elsewhere.
        sei();
        // Task functions called from main loop.  Either add your own task loop
        // or edit the example appTask().
        appTask();   // This currently only does the debug interface via RS232
        macTask();

    }
    return 0;
}
