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
  $Id: mac.c,v 1.2 2009/05/28 19:45:34 bleverett Exp $
*/
// Includes

#include <stdlib.h>
#include <stddef.h>
#include "hal.h"
#include "mac.h"
#include "radio.h"
#include "mac_scan.h"
#include "timer.h"
#include "mac_associate.h"
#include "EE_prom.h"

// Globals

#if DEBUG
/// Debugging temporary string, only used if DEBUG is non-zero
char debugStr[100];
char dbg_buff[40];
#endif 
/// MAC config struct, contains state info for the MAC
macConfig_t macConfig;

// Macros & Defines
/**
     A global buffer to hold frames.  This area is used by both
   send an receive functions to hold radio frame data.  Sometimes the
   first byte is the frame length byte, and sometimes the first byte
   is the first byte of the frame.
*/
u8 mac_buffer_tx[sizeof(rx_frame_t)];  ///< Global MAC buffer (transmit)
u8 mac_buffer_rx[sizeof(rx_frame_t)];  ///< Global MAC buffer (receive)

// Implementation

/**
   Init the mac, which includes initializing the radio chip.

   param:  Channel Sets the channel to use for the MAC.  Use 0xff for
   non-coordinator nodes.
*/
void macInit(u8 Channel)			// this gets called from multiple places to init/reinit the MAC and radio
{
	macConfig.currentChannel = Channel;
    macConfig.panId = BROADCASTPANID;
    macConfig.shortAddress = BROADCASTADDR;
    macConfig.associated = false;

    macConfig.parentShortAddress = BROADCASTADDR;
    macConfig.lastRoute = BROADCASTADDR;
    macConfig.hopsToCoord = 0;
    macConfig.busy = 0;
    macConfig.sleeping = 1; // Assume sleeping state initially

    // Setup the address of this device by reading the stored address from eeprom.
    halGetMacAddr((u8*)&macConfig.longAddr);

    radioSetup();

#   if (NODETYPE == COORD)
    // Initialize the array of nodes (coordinator only)
    macInitNodes();
#endif

#if (DEMO && !TRACKDEMO)
    debugMsgStr("\r\nMAC & Radio Init in DEMO mode - reduced TX-output, reduced RX-sens, RSSI only for association ");
#endif

}

/**
   Set the radio's operating channel, and saves that channel to the
   MAC's global data structure.

   param:  channel The channel number to use for radio communication.
*/
void
macSetOperatingChannel(u8 channel)
{
    // Set the channel
    macConfig.currentChannel = channel;
    radioSetOperatingChannel(channel);
}

/** @} */
