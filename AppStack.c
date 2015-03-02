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
  $Id: rum_application.c,v 1.5 2009/05/28 22:42:00 bleverett Exp $
*/

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "radio.h"
#include "timer.h"
#include "mac.h"
#include "mac_event.h"
#include "mac_start.h"
#include "mac_data.h"
#include "mac_scan.h"
#include "mac_associate.h"
#include "system.h"
#include "hal.h"
#include "sleep.h"
#include "sensors.h"
#include <avr/pgmspace.h>
#include "serial.h"
#include "EE_prom.h"


#define tuip_init_802154(a, b)
//#include <avr/eeprom.h>
// You can program the EEPROM with macaddress this way, using an ELF file
//u8 EEMEM macaddress[8] = {0xde, 0xed, 0xbe, 0xef, 0x11, 0x22, 0x33, 0x44};
//u8 EEMEM macaddress[8] = {0x01, 0x23, 0x45, 0x56, 0x78, 0x9a, 0xbc, 0xde};
//u8 EEMEM macaddress[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};



#define UDP_PORT_COMMANDS 0xF0B0
#define UDP_PORT_RESPONSE 0xF0B1




/**
   On startup, this application uses a number of callback functions to
   bring up the network.  A coordinator node executes this process:

   - Call macFindClearChannel(), which starts a channel scan to find a
     free channel to use for the new PAN.

   - When the scan is complete, the appClearChanFound() callback is
     called. This then initializes the MAC/radio using the channel
     found.

   When a router or end node starts, it connects to the network using
   these steps:

   - macStartScan() is called, which scans all available channels,
     issuing a beacon request frame on each channel.

   - When the scan is complete, appScanConfirm() is called with an
     argument that specifies whether the scan successfully found a
     network to join.

   - If the scan process found a network, then appAssociate() is
     called, which causes the node to send an association request
     packet to the node found in the channel scan.

   - When an association response packet is received, the
     appAssociateConfirm() function is called by the MAC.  At this
     point, the node is part of the network and can send data packets
     to other nodes.

     app_data Sending and Receiving Data

   To send data to another node, the application can call the
   macDataRequest() function.  The application must know the address
   of the destination node, unless the destination is the coordinator,
   which always uses the short address 0x0000 (See
   DEFAULT_COORD_ADDR).

   When a data packet is received, it is routed on to its
   destination. If the final destination is this node's short
   address, then the MAC will call the appDataIndication()
   function, which can then process the incoming data.
*/


// Application variables
#if (NODE_TYPE != COORD)
static u8 failCount;  ///< Number of transmit-to-parent failures before re-scanning.
#endif

volatile u8 gotPanCoordPing;


// These LED functions are used to provide address callbacks for macSetAlarm.
// Cannot use defines directly in callbacks.
static void
ledoff0(void)
{
    Led0_off();
}
static void
ledoff1(void)
{
 	Led1_off();
}


// This function can be used for diagnostic reasons. Not used in RUM application.
void
blink_led0(u8 ms)
{
    Led0_on();
    macSetAlarm(ms, ledoff0);
}
void
blink_led1(u8 ms)
{
    Led1_on();
    macSetAlarm(ms, ledoff1);
}

/**
   Application callback function, called when the MAC receives a ping request
   packet addressed to this node.

   param:  addr The short address of the node that sent the ping request
*/
void appPingReq(ftPing *pf)
{
    // We got a ping, send a response
    // Blip the LED
	blink_led1(1);

    debugMsgStr("\r\nPing request from node ");
    debugMsgInt(pf->originAddr);
    macPing(PING_RSP_FRAME, pf->originAddr);
}

/**
   Application callback function, called when the MAC receives a ping
   response packet addressed to this node.

   param:  addr The short address of the node that send the ping response
 */
void appPingRsp(ftPing *pf)
{
    debugMsgStr("\r\nPing response from ");
    debugMsgHex(pf->originAddr);
    debugMsgStr(" to Dev "); debugMsgHex(pf->finalDestAddr);
    debugMsgStr("\r\nRemote: LQI=");
    debugMsgInt(pf->lqi );
    debugMsgStr(", RSSI=");
    debugMsgInt(pf->rssi );
    debugMsgStr("\r\nLocal : LQI=");
    debugMsgInt(radioGetSavedLqiValue());
    debugMsgStr(", RSSI=");
    debugMsgInt(radioGetSavedRssiValue());
    debugMsgCrLf();
 
    // turn on the LED
    blink_led1(LED_DELAY);
}


/**
   Application callback function, called when the MAC receives an ACK
   packet from a node that we sent a packet to.
*/
void
appPacketSendSucceed(void)  // This gets called from MAC_EVENT_ACK in mac_Task()
{
    // Reset the failure count on a good packet to parent
    // (could also decrement failCount)

#if (NODE_TYPE != COORD)

	if (macIsScanning())	// nothing to do while scanning
	{
//		debugMsgStr("\r\nACK while scanning");
		return;
	}
//	debugMsgStr("\r\n AppPacketSendSucceeded");
	// figure out which way we were sending when the failure occurred
	if (macConfig.lastDestAddr == macConfig.parentShortAddress)
		failCount = 0;


	sensorPacketSendSucceed();



#endif
}

/**
   Application callback function, called when the MAC fails to send a
   packet due to a channel access failure (air is too busy).
*/
void appPacketSendAccessFail(void)
{
#if (NODE_TYPE != COORD)
	debugMsgStr("\r\n AppPacketSendAccessFail ");

	if (macIsScanning())
	{
		debugMsgStr("while scanning");
		return;
	}


    sensorPacketSendFailed();

#endif
}

/**
   Application callback function, called when the MAC fails to receive
   an ACK packet from a node that we sent a packet to.
*/
void appPacketSendFailed(void)
{
    #if (NODE_TYPE != COORD)
        u8 parentFailed;
		// don't mess with access failures during scanning occupied channels will cause them
		if (macIsScanning())
			return;

		debugMsgStr("\r\n AppPacketSendFail=");
		debugMsgInt(failCount+1);

		parentFailed = (macConfig.lastDestAddr == macConfig.parentShortAddress);
		/* Special code: this prevents a situation where the
		   coordingator goes away, and the nodes take a long time to
		   realize that the network is gone.  Nodes Keep trying to
		   re-associate with each other, but until a router loses a
		   number of packets to its parent, it still thinks it's
		   associated.  This code forces the issue by either verifying
		   that the router is still connected, or forcing a failure.    */
		// Send an empty data packet
		if (parentFailed && NODE_TYPE == ROUTER)
			macDataRequest(macConfig.parentShortAddress, 0, NULL);

		// Don't have a cow until we have missed a few packets in a row
		if (++failCount < 8)
		{
			sensorPacketSendFailed();
			return;
		}

		// A sent packet failed to reach its destination too many times
		// figure out which way we were sending when the failure occurred
		if (parentFailed)
		{
			// re-associate if we were sending upstream
			macConfig.associated = false;

			// It is possible to make the coord/end units re-connect
			// to the coordinator more quickly by not scanning all
			// available channels.  To do this, uncomment the following
			// line.
			// macSetScanChannel(macConfig.currentChannel);


			sensorLostNetwork();

			macSetAlarm((radioRandom(8)+5) *10, macStartScan);
		}
		if (NODE_TYPE == ROUTER &&
			macIsChild(macConfig.lastDestAddr))
		{
			// Drop child from table if the failure was downstream
			macRemoveChild(macConfig.lastDestAddr);
			debugMsgStr("\r\nDropped child bcs packet send failed.");
		}
#endif

}


/**
     Callback function, called when the MAC receives a data packet for
   this node.

   The data is available in mac_buffer_rx, which can be cast to a
   dataFrame_t struct like this:

   @code
   ftData *frame = (ftData *)(mac_buffer+1);
   @endcode
 */
void
appDataIndication(u8 * payload, u8 len, bool Broadcast)
{

#if DEBUG==2
	debugMsgStr("appDataIndication->");
#endif
	blink_led1(LED_DELAY);
#if  DEBUG && SERIAL
// TODO: this should be it's own function dealing with all Broadcast frames
	 if( Broadcast)
	 {
		 debugMsgStr("Broadcst from ");
		 tBC_Data * BC_Frame_data = (tBC_Data *) payload;
		 debugMsgInt(BC_Frame_data->SensorUnitID );
		 debugMsgStr(" Stype ");
		 debugMsgInt(BC_Frame_data->Readings[0].SensorType );
		 debugMsgStr(" ADC0 = ");
		 debugMsgInt(BC_Frame_data->Readings[0].ADC_Value );
	 }
	 else
#endif
	 {
#if ( NODE_TYPE == COORD)	// pass data to application
		 CoordRcvSensorPacket(payload, len);
#else
		 sensorRcvPacket(payload,  len );
#endif
	 }
}
/**
   Callback function, called when the MAC has associated a child of
   this node to the network.

   param:  shortAddress The short address of the new child node that
   has been associated.  The MAC stores this address, so the
   application should not have to.
*/
void
appChildAssociated(u16 shortAddress)
{
    // Blip the LED when we associate a child
    debugMsgStr("\r\n!App Child Associated\n\r");
	blink_led1(LED_DELAY);
}

/**
   Callback function, called when the MAC has associated a new node to
   a network. This is only called for the coordinator.

   param:  shortAddress The short address assigned to the new node.  The
   MAC stores this address, so the application should not have to.
*/
void
appNodeAssociated(u16 shortAddress)
{
	//debugMsgStr("\r\n!App NODE Associated\n\r");
}

# if(NODE_TYPE != COORD)

/**
   Callback function, called when the MAC receives an association
   confirm packet addressed to this node, or the process timed out.

   param:  success True if an association packet was received, or false
   if the association process timed out.
*/

void
appAssociateConfirm(bool associated)
{
	if (associated)
	{
		blink_led1(200);

		// If we are auto-sending data, start that process.
        SensorStartReadings();	// init measurment sensors and start sending data to COORD

	}

}

/**
   Callback function, called when the MAC has completed its channel scan.

   param:  success True if   macScan found a network to connect to,
   or false if no networks were found.
*/
void
appScanConfirm(u8 success)
{
    // Write app code here -- This  node has finished its scan,
    // and has receive a coordinator address.
}
#endif

/**
   Callback function, called when   macFindClearChannel()
   completes.

   u8 channel The clear channel selected for use by this PAN.
*/
void
appClearChanFound(u8 channel)
{
    if (NODE_TYPE == COORD)
    {
        macInit(channel);
        macStartCoord();
        debugMsgStr("\r\nStartup, I am THE coordinator on ch ");
        debugMsgInt(channel);
        debugMsgStr("\r\n");
        macConfig.associated = true;

    }
}

