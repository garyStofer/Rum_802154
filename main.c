#include <stdlib.h> 
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "stdbool.h"

#include "radio.h"
#include "mac.h"
#include "mac_event.h"
#include "mac_scan.h"
#include "timer.h"
#include "mac_start.h"
#include "mac_data.h"
#include "mac_associate.h"
#include "system.h"
#include "hal_avr.h"
#include "EE_prom.h"
#include "Sensors/measure.h"



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

	LED_INIT(); 	// PD2 and PD3
	MSG_LED_INIT(); //  PD7
	
	// Init the button
    BUTTON_SETUP(); //PD4

    // Also PD5 is used as input in Coord for selection of UART output format -- Human/machine
	// Init the timer system for the MAC
    timerInit();

// init serial
#    if (SERIAL)
        serial_init(NULL);
#endif

	sei();		// enable global interrupts

	// Init radio -- calls radio_hal_init()
    if ( radioInit() != RADIO_SUCCESS)
	{
		   debugMsgStr("\r\n Wrong BAND or unsupported RF device");
		   // Lock the CPU Nothing else to do when we can't talk to the radio chip
		   Leds_on();
		   while (1);
	}

   	// All sensor measurement init stuff
   	Do_SensorInit();

    blink_led0(200); // Blip the LED once on power-up
}


/**
     Main function of the program.
*/
volatile double f = 0.1223;
int main(void)
{

   	Device_init();

	// If the EEPROM is cleared, initialize it to something useful
	checkEEprom();
// EE_SetFrameInterval(1); 	// and make it 0.1 seconds
	// Init the mac and stuff
#if (NODE_TYPE == COORD)
	{
		debugMsgStr("\r\nStartup: as Coordinator");
		DDRD &= ~(1 << PD5); PORTD |= (1 << PD5);  // Make PD5 a pulled up input to switch between human and machine interface on UART port

		// BIND button
		if (BUTTON_PRESSED() )		// This creates a new PAN_ID , and all Children have to bind again
		{
			Led1_on();
			EE_SetPanID(0xffff);
			while (BUTTON_PRESSED())
				;
			Led1_off();
		}

		macFindClearChannel();	// appClearChanFound() will be called which calls  macStartCoord(void);
	}
#else
	debugMsgStr("\r\nStartup: as Device");

	// BIND BUTTON
	if (BUTTON_PRESSED() )
	{
		debugMsgStr("\r\nClearing PAN_ID -- BINDING to new..");
		Led1_on();
		EE_SetPanID(0xffff);			// This binds to a new PAN ID
		EE_SetFrameInterval(5);		// initialize sending of readings every 0.5 Seconds
		while (BUTTON_PRESSED())
			;
		Led1_off();
	}

	// End or Router node, start scanning for a coordinator
	macStartScan();


#endif

#if( NODE_TYPE == BC_ENDDEVICE )
		// A BC only node has no way of knowing if the network is still alive and on the same channel unless it periodically
		// does a re-scan of all channels or the channel & PANDID is fixed
	  StartSensorBC_interval();
#endif

     // Main forever loop for the application.
    for(;;)
    {
        // Turn interrupts on if they are off.  They should always be on
        // at this point in the loop, so just turn them on every time, in
        // case interrupts were inadvertently turned off elsewhere.
        sei();

        appMenu();   // This currently only does the debug interface via RS232
        macTask();

    }
    return 0;
}
