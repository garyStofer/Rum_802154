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
  $Id: avr_timer.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/
#include "mac_event.h"
#include "hal.h"
#include "radio.h"
#include "timer.h"
#include "system.h"
#include <stdlib.h>
#include "data_types.h"



/**
   @addtogroup timer_module
   @{

   This module implements a user-friendly timer interface.  The basic
   functionality is to set alarm events that cause a function to be
   called after a delay.  See macSetAlarm()

   This timer is a general-purpose timing function, to be used by
   application code.

   To use the timer, call timerInit() once at program startup.  To set
   a timer, call the macSetAlarm() function, which will cause a
   callback to occur after the specified time.

   For example, to cause an LED to flash momentarily, one could write:

   @code
   void ledOff(void)
   {
       LED_OFF();  // Macro to turn off LED
   }

   void flashLed(void)
   {
       LED_ON();   // Macro to turn on LED
       macSetAlarm(100, ledOff);  // In 100mS, call ledOff()
   }
   @endcode

   Essentially, the macSetAlarm functions simply set a timer to call
   an arbitrary function after some time delay.  Note that the time
   delay is not a "spin loop", so that other processing can occur
   while the timer is waiting to expire.


   There is a limit to how many pending events can be stored.  Adjust
     TIMER_EVENTS_MAX in necessary.
*/

/**
   Structure to hold active timers.
*/
typedef struct
{
    u16 time;                  ///< Ticks left before timer expires.
    u8 timerID;
    void (*callback) (void);   ///< Function to call when timer expires.
} timerEventT;

/// The maximum number of pending timer events.  Adjust as necessary.
#define TIMER_EVENTS_MAX 10
/// Array of timer event structures.
static timerEventT timerEvents[TIMER_EVENTS_MAX];
static u16 secondTimer = 1000/MS_PER_TICK;
#define max(a, b) ( (a)>(b) ? (a) : (b) )       // Take the max between a and b

#ifdef LONG_TIMER
#define LONG_TIMER_EVENTS_MAX 3
/// Array of long-time timer event structures.
static timerEventT longTimerEvents[LONG_TIMER_EVENTS_MAX];
#endif

/**
      Start the hardware timer running.
*/
/*
static u8 timerRunning=0;
static void 
timerStart(void)
{
    if (!timerRunning)
    {
        // Clear timer counter value.
        TIMER_CLEAR();

        // Enable timer output compare interrupt.
        TIMER_ENABLE();
        timerRunning = 1;
    }
}
*/


/**
     Initialize the MAC timer.  Called at powerup, or when you
   want to clear all timers.  Sets the timeout for each tick of the
   system clock.
 */
void timerInit(void)
{
    TIMER_INIT();

    // Init the PRNG
    if (NODE_TYPE == ROUTER || NODE_TYPE == COORD)
        srand(TCNT(TICKTIMER));

	TIMER_CLEAR();

    // Enable timer output compare interrupt.  -- Timer starts to interrupt as soon as Global interrup is enabled
    TIMER_ENABLE();
}

/**
     Returns a unique timer ID. Used only ti kill a timer prematurely

   returns:  The unique ID.
*/
static u8 
getUniqueTimer_ID(void)
{
    static u8 currentTimerID;
    u8 i;
    bool goodID=false;

    // Start searching from the last ID+1

    for(currentTimerID++; goodID; currentTimerID++ )
    {
        // don't return zero as an ID, macSetAlarm return zero on error
        if (currentTimerID==0)
            continue;

        // See if this ID is used by any active timer
        for (i=0; i<TIMER_EVENTS_MAX; i++)
        {
            if (timerEvents[i].timerID == currentTimerID)	// timer ID in use -- skipp
                continue;
        }

#       if (LONG_TIMER)
        {
            for (i=0; i<LONG_TIMER_EVENTS_MAX; i++)
            {
                if (longTimerEvents[i].timerID == currentTimerID)	// timer ID in use -- skipp
                    continue;
            }
        }
#		endif
        goodID =true;

    }
    return currentTimerID;
}






#define TICKS_PER_MS (u16)(1.0/((float)MS_PER_TICK))

volatile static u16 tickTimer;

u16
macGetTime(void)
{
    u16 localtime;

    AVR_ENTER_CRITICAL_REGION();
    localtime = tickTimer;
    AVR_LEAVE_CRITICAL_REGION();

    return localtime;
}

/**
     Sets a general purpose timer.  Can be called by application.
   The callback will not be called from the timer ISR, so it is
   thread-safe.

   param:  time Number of milliseconds to wait before calling
   the callback function.  This parameter can be 0-65535

   param:  callback Pointer to a function to call back after the
   specified time.  The callback function must take no arguments and
   return nothing.

   returns:  Handle to timer.  Can be used to call macTimerKill().

   NOTE: Timers are oneshot only - no repeat or reload function
         you must call MacSetAlarm repeadedtly for a timed loop
*/
u8
macSetAlarm(u16 time, void(*callback)(void))
{
    u8 i;
    u16 ticks;

    if (!time)
    {
        // Don't delay, just call it
        callback();
        return 0;
    }

    // Store the timer details in the array
    ticks = max(time, 1);  // At least one tick

    // Protect this section from an ISR that will add an alarm
    AVR_ENTER_CRITICAL_REGION();
    // search for free event structure
    for (i=0;i<TIMER_EVENTS_MAX;i++)
        if (!timerEvents[i].time)
            // free, use this one
            break;

    if (i >= TIMER_EVENTS_MAX)
        // Out of timers to use, just quit
        return 0;

    timerEvents[i].time = ticks;
    timerEvents[i].callback = callback;
    // don't return zero as a timer ID
    timerEvents[i].timerID = getUniqueTimer_ID();

    AVR_LEAVE_CRITICAL_REGION();

    return timerEvents[i].timerID;
}


/**
     A long-delay timer function, which can delay for up to 15.7
   hours.

   returns:  Handle to timer.  Can be used to call macTimerKill().
*/
u8
macSetLongAlarm(u16 seconds, void(*callback)(void))
{
    // Only use long alarms with app or 6lowpan
#   if (LONG_TIMER)
    {
        // Find a free timer
        u8 i;
        
        if (!seconds)
        {
            // Just go right now
            callback();
            return 0;
        }
        
        for (i=0;i<LONG_TIMER_EVENTS_MAX;i++)
            if (!longTimerEvents[i].time)
                // free, use this one
                break;
        
        if (i >= LONG_TIMER_EVENTS_MAX)
            // No more timers, just quit
            return 0;
        
        // Store the time and callback into free timer
        longTimerEvents[i].time = seconds;
        longTimerEvents[i].callback = callback;
        longTimerEvents[i].timerID = getUniqueTimer_ID();
		
   /*     started once and for all in Timer init
        // start timer running, since we have at least one timer running
        AVR_ENTER_CRITICAL_REGION();
        timerStart();
        AVR_LEAVE_CRITICAL_REGION();
   */
        return longTimerEvents[i].timerID;
    }
#endif
    return 0;
}

/**
     End the timer specified by ID, before it is expired.
   Calling after expiration will not cause any problem, unless a new
   timer has been set withe the same ID.

   param:  timerID The value returned from macSetAlarm when the alarm
   was set.
*/
void
macTimerKill(u8 timerID)
{
    u8 i;

    // search for timer with timerID
    for (i=0;i<TIMER_EVENTS_MAX;i++)
    {
        if (timerEvents[i].timerID == timerID)
        {
            // kill this timer
            timerEvents[i].time = 0;
            timerEvents[i].timerID = 0;
            return;
        }
    }
    // search for long timer with timerID
#   if (LONG_TIMER)
    {
        for (i=0;i<LONG_TIMER_EVENTS_MAX;i++)
            if (longTimerEvents[i].timerID == timerID)
            {
                // kill this timer
                longTimerEvents[i].time = 0;
                longTimerEvents[i].timerID = 0;
                return;
            }
    }
#	endif
}

/**
     Timer interrupt service routine.
*/
//ISR(TIMER1_COMPA_vect)
ISR(TICKVECT)
{
    u8 i;
    event_object_t event;

    tickTimer++;

    
    // Decrement second timer
    if (LONG_TIMER)
    {
        if (!--secondTimer)
        {
            // Reset one second timer
            secondTimer = 1000/MS_PER_TICK;
            // Handle the one-second timers
            for (i=0;i<LONG_TIMER_EVENTS_MAX;i++)
            {
                if (longTimerEvents[i].time)
                {
                    // This timer is active, check for expiration
                    if (!--longTimerEvents[i].time)
                    {
                    	timerEvents[i].timerID = 0;
                        // Timer expired, queue the associated callback
                        event.event = MAC_EVENT_TIMER;  // Event type, see event_t for details.
                        event.data = (u8*)longTimerEvents[i].callback;
                        mac_put_event(&event);
                    }
                }
            }
        }
    }

    // check for pending events
    for (i=0;i<TIMER_EVENTS_MAX;i++)
    {
        if (timerEvents[i].time)
        {
            // this timer event is live, check for expiration
            if (!--timerEvents[i].time)
            {
            	timerEvents[i].timerID = 0;
                // Timer expired, queue the associated callback
            	event.event = MAC_EVENT_TIMER;  // Event type, see event_t for details.
                event.data = (u8*)timerEvents[i].callback;
                mac_put_event(&event);
            }
        }
    }
}

/** @} */
