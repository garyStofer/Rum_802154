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
  $Id: mac_data.c,v 1.5 2009/05/27 18:30:31 mvidales Exp $
*/

#include <stdint.h>
#include <stddef.h>
#include "stdbool.h"
#include <string.h>
#include "mac.h"
#include "radio.h"
#include "mac_data.h"

#include "data_types.h"
#include "mac_scan.h"
#include "mac_route.h"
#include "mac_associate.h"
#include "system.h"
#include "System_config.h"
#include "timer.h"


/**
   @ingroup mac
   @{
   @defgroup mac_data MAC data functions
   @{

   This module includes code for sending and receiving data using the
   MAC.  Data is sent to another node on the network using
   macDataRequest(), and incoming packets are routed to
   macDataIndication(), which calls appDataIndication().

   If 6LoWPAN is being used, do not call macDataRequest() directly,
   but use the   avr6lowpan functions (AVR only).
*/

// Define the length of the header (non-payload part) of a data frame
#define ftDataHeaderSize offsetof(ftData, payload)



// Globals
static u16 pingAddr;  // temp var because ReallyDoPing is being called indirectly from timer function
static u16 pingType;  // temp var  ""

// Stored frames (if NODE_SLEEP is set)
typedef struct {
    u8  len;         // Length of stored frame (zero if this element is unused)
    u16 addr;        // Short address of node to receive frame
    u8  buf[128];    // Buffer to store frame in
} tStoredFrame;

#if ((NODE_TYPE == COORD || NODE_TYPE == ROUTER) && NODE_SLEEP )
static tStoredFrame storedFrames[STORED_FRAMES]= {{0}};
#endif









// Target function for timer, send a frame prepared by macDataRequest()
void
mdr_timer(void)
{
    // Create a struct pointer to the global variable...
    ftData *data_frame = (ftData*)(mac_buffer_tx+1);

    radioSendData(*mac_buffer_tx, (u8*)data_frame);
}
/*
 * Broadcast only nodes do not fully associate with the network. They only scan the channels in serach of a network with PANID
 * as given by the compile define, which can be the broadcast PANID or a specific one.
 * Upon a sucessful search we have a network channel and a network PANID, which is either the defined PANID or any networks PANID
 * if the scan was done with a Broadcast PANID to start with.
 *
 */

void
macData_BC_Request( u8 len, u8 * data)
{
    ftData *data_frame = (ftData*)(mac_buffer_tx+1); // Create a struct pointer to the global variable...

    // if we have not found a channel to broadcast on do nothing
    if (panDescriptor.logicalChannel  == 0xff)
    	return;

    Led1_on();
    debugMsgStr("\r\nmacData_BC_Request->");

    // Build the frame.
    data_frame->fcf=FCF_DATA_NOACK; //FCF Data frame no ACK -- to prevent multiple acks
    data_frame->seq = macConfig.dsn++;
    data_frame->panid = macConfig.panId;
    data_frame->srcAddr = macConfig.shortAddress;
    data_frame->originAddr = macConfig.shortAddress;
    data_frame->finalDestAddr = BROADCASTADDR;
    data_frame->destAddr = BROADCASTADDR;
    data_frame->type =DATA_FRAME;

    u8 i;
    for(i=0; i<len; i++)
        ((u8*)&data_frame->payload)[i] = *data++;

    *mac_buffer_tx = len + ftDataHeaderSize; // save length away

     macConfig.busy = true;  // this locks out debug tasks
     radioSendData(*mac_buffer_tx, (u8*)data_frame);
     Led1_off();
}






/**
      The macsixlowpanDataRequest function is used to send a frame over
    the air to another node.  Any node type can call this function.

    param:  addr Short address of the destination node.

    param:  len The length of the packet in bytes.

    param:  data Pointer to the data to be sent.

    param:  type Type of frame to be sent
*/
static void
macDataRequestInt(u16 addr, u8 len, u8 * data, u8 type)
{
    // Create a struct pointer to the global variable...
    ftData *data_frame = (ftData*)(mac_buffer_tx+1);
    u8 rpSent; // Was a routing packet sent?


    if (addr == macConfig.shortAddress || addr == BROADCASTADDR)
    {
        // Don't send to self
    	debugMsgStr("\r\nSelf addressed or Brodcast: nothing sent!");
        return;
    }
    if (!macConfig.associated)
        // This node has no short address
        return;
#if DEBUG==2
    debugMsgStr("\r\nmacDataRequest->");
#endif
    // Build the frame.
    data_frame->fcf = FCF_DATA;
    data_frame->seq = macConfig.dsn++;
    data_frame->panid = macConfig.panId;
    data_frame->srcAddr = macConfig.shortAddress;
    data_frame->finalDestAddr = addr;
    data_frame->originAddr = macConfig.shortAddress;

    // send a routing packet if necessary
    rpSent = macSendRoutingPacket(addr);

#if (NODE_TYPE == COORD)
    {
        // Find the child node that can route this packet
        u16 child = addr;
        u16 parent = macGetParent(child);
        while (parent != DEFAULT_COORD_ADDR)
        {
            child = parent;
            parent = macGetParent(child);
        }
        // send to child node that can route this packet
        data_frame->destAddr = child;
    }
#   else
    {
        // All data is send to parent, unless this is a wakeup frame
        if (type == WAKE_NODE)
            data_frame->destAddr = addr;
        else
        	data_frame->destAddr = macConfig.parentShortAddress;
    }
#endif
    // Frame type is data
    data_frame->type = type;

    // Set high bit of type if we're sleeping
    if (macConfig.sleeping && NODE_TYPE == ENDDEVICE && NODE_SLEEP)
       	data_frame->type |= 0x80;  // Indicate that this ENDdevice is sleeping

    // Copy the payload data to frame. (note: this creates smaller code than using memcpy!!)
    u8 i;
    for(i=0; i<len; i++)
        ((u8*)&data_frame->payload)[i] = *data++;

    // Check addresses again - addr will be different now
    if (data_frame->destAddr == macConfig.shortAddress ||
        data_frame->destAddr == BROADCASTADDR)
        // Don't send to self
        return;

    // send data to radio, after some time delay
    *mac_buffer_tx = len + ftDataHeaderSize; // save length away
#if (NODE_TYPE == COORD)
    {
        // See if the child is sleeping (only the coord sends directly to a child node)
        if (NODE_SLEEP && macIsChild(addr) && macIsChildSleeping(addr) )
            // Send it later, after child is awake
            macHoldFrame(addr, (u8*)data_frame, (u8)*mac_buffer_tx);
        else
        {
            // Node is not sleeping child, send it now.
#if DEBUG==2
        	debugMsgStr("-- macSetAlarm set");
#endif
        	macSetAlarm(rpSent ? MAC_RP_DELAY : MAC_DATA_DELAY, mdr_timer);
        }
    }
#  else
        macSetAlarm((type==WAKE_NODE || type == DEBUG_FRAME) ? 0 : MAC_DATA_DELAY, mdr_timer);
#endif
    macConfig.busy = true;
}

/**
     The macsixlowpanDataRequest function is used to send a frame over
   the air to another node.  Any node type can call this function.

   param:  addr Short address of the destination node.

   param:  len The length of the packet in bytes.

   param:  data Pointer to the data to be sent.
*/
void
macDataRequest(u16 addr, u8 len, u8 * data)
{
    macDataRequestInt(addr, len, data, DATA_FRAME);
}

/**
      The macWakeRequest function is called by the coordinator to
            send a wakeup packet to a router.

    param:  addr Short address of the parent router of the node to wake
    up.
    param:  child Short address of the child node to wake up.
*/
void
macWakeRequest(u16 addr, u16 child)
{
    if (NODE_TYPE == ROUTER || NODE_TYPE== COORD)
    {
        macDataRequestInt(addr, 2, (u8*)&child, WAKE_NODE);
    }
}

/**
     Send an other-the-air (OTA) debug frame.  This contains a
   string payload that is displayed on the coordintor end.
*/
void
macOtaDebugRequest(u8 *str)
{
#if (NODE_TYPE != COORD)
    {
        macDataRequestInt(DEFAULT_COORD_ADDR, strlen((char *)str)+1, str, DEBUG_FRAME);
    }
#endif
}


/**
   This function is called when the MAC receives a packet that is
   addressed to this node.  The packet is dispatched according to its
   contents.
*/
void
macDataIndication(u16 dest)
{
    // Sort out the different types of data packets.
   ftData *frame = (ftData*)(mac_buffer_rx+1);
   u8 pl_len = *mac_buffer_rx - 16;
#if DEBUG==2
    debugMsgStr("\r\nmacDataIndication<-");
#endif
    switch (frame->type & 0x7f)  // Mask high bit just in case it was somehow missed
    {
		case DATA_FRAME:
			// Data, send it up the chain
			appDataIndication(frame->payload, pl_len, (dest ==  BROADCASTADDR));
			break;

		case DEBUG_FRAME:
			// Frame containing debug message, print it on coord
			if (NODE_TYPE == COORD && OTA_DEBUG && DEBUG)
			{
				debugMsgStr("\r\nNode ");
				debugMsgInt(frame->originAddr);
				debugMsgStr(": ");

				// Remove leading cr/lf's from string
				u8 *p = frame->payload;
				while (*p)
				{
					if (*p > ' ')
						break;
					p++;
				}
				debugMsgStr_d((char *)p);
			}
			break;

		case WAKE_NODE:
			// Wake up the end node.
#if (NODE_TYPE == ROUTER)
			{
				u8 addr = ((ftWake*)frame)->addr;
				// See if this is from parent or child
				if ((((ftWake*)frame)->srcAddr) == macConfig.parentShortAddress)
					// Set the flag to wake up the end node when it sends a packet
					macWakeChildNode(addr);
			}
#endif

#if (NODE_TYPE == ENDDEVICE) /* also include BC_ENDDEVICE ?*/
			{
				// Wake yourself up now
				macConfig.sleeping = false;
				// Send parent a confirmation that we are awake
				macDataRequestInt(macConfig.parentShortAddress,2,
								  (u8*)&macConfig.shortAddress,WAKE_NODE);
				debugMsgStr("\r\nAwake");
			}
#endif
			break;

		case PING_REQ_FRAME:
			// We got a ping request, let the app handle that
			appPingReq((ftPing *) (mac_buffer_rx+1));
			break;

		case PING_RSP_FRAME:
			// We got a ping response, app will handle it
			appPingRsp((ftPing *) (mac_buffer_rx+1));
			break;

#if (NODE_TYPE == ROUTER)
		case DROP_CHILD_FRAME:
			// Coordinator is telling us to drop a child
			macRemoveChild(*(u16*)(&frame->payload));
			break;
#endif
		default:
			break;
    }
}

// Target function to timer, sends ping packet after a delay
// note that arguments are via static vars due to the indirect nature of the call
static void
macDoPingRequest(void)
{
    ftPing frame;

    frame.fcf = FCF_DATA;
    frame.seq = macConfig.dsn++;
    frame.panid = macConfig.panId;
    frame.srcAddr = macConfig.shortAddress;
    frame.originAddr = macConfig.shortAddress;
    frame.finalDestAddr = pingAddr;
    frame.rssi = radioGetSavedRssiValue();
    frame.lqi = radioGetSavedLqiValue();
    frame.type = pingType;

    // Set high bit of type if we're sleeping
    if (macConfig.sleeping && NODE_TYPE == ENDDEVICE && NODE_SLEEP)
       	frame.type |= 0x80;

#if (NODE_TYPE == COORD)
    {
#if DEBUG==2
 debugMsgStr("\r\nin macDoPingRequest");
#endif
    	// Find the top parentpin
        u8 addr = macGetTopParent(pingAddr);
        frame.destAddr = addr;
        // See if the child is sleeping (only the coord sends directly to a child node)
        if (NODE_SLEEP &&
            macIsChild(addr)&&
            macIsChildSleeping(addr))
        {
            // Send it later, after child is awake
#if DEBUG==2
debugMsgStr("\r\ncalling macHoldFrame for ping response");
#endif
            macHoldFrame(addr, (u8*)&frame, sizeof(ftPing));
            // Don't send frame right now
            return;
        }
        else
        {
#if DEBUG==2
debugMsgStr("\r\n child: ");
debugMsgInt(addr);
debugMsgStr(" not sleeping");
#endif
        }

    }
#else
        // End/router nodes
        frame.destAddr = macConfig.parentShortAddress;
#endif
#if DEBUG==2
debugMsgStr(" sending Ping Response now\n\r");
#endif

    radioSendData(sizeof(ftPing), (u8*)&frame);
}

/**
   Send a ping packet (either request or response) to another node.

   param:  pingTypeArg Which type of ping to send, either  PING_REQ_FRAME or   PING_RSP_FRAME.

   param:  addr Short address of node to send ping
*/
void
macPing(u8 pingTypeArg, u16 addr)
{
    if (addr == macConfig.shortAddress)
        // Don't send to self
        return;

    if (!macConfig.associated)
        // Broadcast addr
        return;

    pingAddr = addr;
    pingType = pingTypeArg;

debugMsgStr("\r\nmacPing to: ");
debugMsgInt(pingAddr);
debugMsgStr(" type: ");
debugMsgInt(pingType); // 2 == request 3== response
debugMsgStr("\r\n");

#   if (NODE_TYPE == COORD)
    {

        u8 rpSent;
        rpSent = macSendRoutingPacket(addr); 						// First send a routing packet
#if DEBUG==2
debugMsgStr("\r\nsetting up Alarm for ping response");
#endif
        macSetAlarm(rpSent ? MAC_RP_DELAY : 0, macDoPingRequest);	// Then send the Ping (response)
        macConfig.busy = true;
    }
#   else
    {

        // End/router nodes
    	macDoPingRequest();
        macConfig.busy = true;
    }
#   endif
}


/**
   Save a frame for transmission later.  Used to store frames for
   sleeping children.

   param:  addr Short address of recipient node

   param:  len Length of the frame to store

   param:  buf Pointer to buffer containing the frame to save
*/
void
macHoldFrame(u16 addr, u8 *buf, u8 len)
{
#   if ((NODE_TYPE == ROUTER || NODE_TYPE == COORD) && NODE_SLEEP )
    {
        u8 i,done=0;

        for (i=0;i<STORED_FRAMES;i++)
        {
            if (!storedFrames[i].len)
            {
                if (!done)  // Only store once
                {
#if DEBUG
debugMsgStr("\r\nHolding frame for sleeping child");
#endif
					// This one's free, use it
                    storedFrames[i].len = len;
                    storedFrames[i].addr = addr;
                    memcpy(storedFrames[i].buf, buf, len);
                    // Don't store this frame twice
                    done = 1;
                }
            }
            else
            {
                // This item is used, make sure it isn't for this address,
                // since we only want to store one pending frame for each
                // sleeping node.
                if (storedFrames[i].addr == addr)
                    // Delete this frame
                    storedFrames[i].len = 0;
            }
        }
    }
# endif
}

/**
   Send a frame that has been stored for a child node.  This is meant
   to be called immediately upon receiving a packet from a sleeping child node.
*/
void macSendStoredFrame(u16 addr)
{
#   if ((NODE_TYPE == ROUTER || NODE_TYPE == COORD) && NODE_SLEEP )
    {
        u8 i;

        // See if a frame is stored for this node
        for (i=0;i<STORED_FRAMES;i++)
        {
            if (storedFrames[i].len &&
                storedFrames[i].addr == addr)
            {
#if DEBUG
debugMsgStr("\r\nMac sending postponed frame \r\n");
#endif
                // Send this frame, remove checksum
                radioSendData(storedFrames[i].len, storedFrames[i].buf);
                // Clear out this frame
                storedFrames[i].len = 0;
            }
        }
    }
#   endif
}
/** @} */
/** @} */
