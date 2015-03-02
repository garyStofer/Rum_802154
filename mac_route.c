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

#include "mac.h"
#include "mac_associate.h"
#include "mac_data.h"
#include "timer.h"
#include "radio.h"
#include "hal.h"
#include "mac_route.h"
#include <string.h>

/**
   @addtogroup mac
   @{
 */

/**
    Send routing packet down the chain, removing own address from the
    list of addresses in the route.
*/
void
macForwardRoutingPacket(void)
{
#if (NODE_TYPE == ROUTER)
    {
        u8 payloadLength = ((rx_frame_t*)mac_buffer_rx)->length - sizeof(ftRouting);
        ftRouting *frame = (ftRouting*)(mac_buffer_tx+1);
        u16 *p;  // pointer to list of addresses in incoming routing packet
        u8 hopCount = payloadLength/2; // convert from bytes to words

        // Copy rx to tx, since we are forwarding packet
        macCopyRxToTx();

        // Don't send packets unless we have a valid short address
        if (!macConfig.associated)
            return;

        // set pointer to first address
        p = &frame->shortAddr;

        // set last route address in MAC
        macConfig.lastRoute = p[hopCount-1];

        debugMsgStr("\r\nSet route=");
        debugMsgHex(macConfig.lastRoute);

        if (hopCount > 1)
        {
            // must forward to next address in chain

            // remove last address from list
            hopCount--;

            frame->fcf = FCF_ROUTE;
            frame->seq = macConfig.dsn++;
            frame->destAddr = p[hopCount];
            frame->srcAddr = macConfig.shortAddress;
            frame->cmd = ROUTING_PACKET;

            // send out routing packet
            radioSendData(sizeof(ftRouting) + (hopCount-1)*2, (u8*)frame);
        }
    }
#endif
}


void
macRouteAssociateResponse(void)
{
    // Send the mac packet down the chain
    // The incoming packet is not for this node, therefore
    // the incoming packet is an indirect packet.

#if (NODE_TYPE == ROUTER)
    {
        // find out if we are sending to the endnode.
        ftAssocRespIndirect* inputFrame = (ftAssocRespIndirect*) (mac_buffer_rx+1);

        // Don't send packets unless we have a valid short address
        if (!macConfig.associated)
            return;

        if (inputFrame->parentAddr == macConfig.shortAddress)
        {
            // This frame is being sent from me (a router) to the newly
            // associated node. We have to translate from indirect to
            // direct frame format
            ftAssocRespDirect *outputFrame = (ftAssocRespDirect*) mac_buffer_tx;

            outputFrame->fcf = FCF_ASSOC_RESP_DIRECT;
            outputFrame->seq = macConfig.dsn++;
            outputFrame->panid = macConfig.panId;
            outputFrame->dstAddr = inputFrame->macAddr;
            outputFrame->srcAddr = macConfig.shortAddress;
            outputFrame->cmd = ASSOCIATION_RESPONSE;
            outputFrame->shortAddr = inputFrame->shortAddr;

            // Add this new node to child table
            macAddChild(outputFrame->shortAddr);

            radioSendData(sizeof(ftAssocRespDirect),(u8*)outputFrame);
        }
        else
        {
            // This frame is being forwarded to a downstream router, not an
            // end node.  Just re-use the input packet, and change the
            // addresses for src/dest.
            ftAssocRespIndirect *outputFrame = (ftAssocRespIndirect*) (mac_buffer_tx+1);

            macCopyRxToTx();
            outputFrame->seq = macConfig.dsn++;
            outputFrame->dstAddr = macConfig.lastRoute;
            outputFrame->srcAddr = macConfig.shortAddress;

            // Also, check to see if the parent of the new node is one of my
            // children.  This is needed because the routing packet mechanism
            // doesn't work properly for a three-hop route.
            if (macIsChild(outputFrame->parentAddr))
                outputFrame->dstAddr = outputFrame->parentAddr;

            radioSendData(sizeof(ftAssocRespIndirect),(u8*)outputFrame);
        }
    }
#endif
}

void
macRouteAssociateRequest(void)
{
    // This router node has received an association request packet.  We must
    // Send this packet along, and if the packet came from a MAC address, then
    // we must translate from a direct packet to an indirect packet.
#if (NODE_TYPE == ROUTER)
    {
        if (mac_buffer_rx[2] == (FCF_ASSOC_REQ_DIRECT >> 8)) // Direct from MAC addr?
        {
            // translate from direct to indirect
            ftAssocReqDirect *input = (ftAssocReqDirect*)(mac_buffer_rx+1);
            ftAssocReqIndirect output;

            output.fcf = FCF_ASSOC_RESP_IND;
            output.seq = macConfig.dsn++;
            output.panid = macConfig.panId;
            output.destAddr = macConfig.parentShortAddress; // send to parent
            output.srcAddr = macConfig.shortAddress;
            output.cmd = ASSOCIATION_REQUEST;
            output.parentAddr = input->parentAddr;
            output.macAddr = input->srcAddr;
            output.type = input->type;

            radioSendData(sizeof(ftAssocReqIndirect), (u8*)&output);
            debugMsgStr("\r\nReceived assoc req");
        }
        else
        {
            // input frame is indirect, output frame is indirect, just copy
            ftAssocReqIndirect *input = (ftAssocReqIndirect*)(mac_buffer_rx+1);
            ftAssocReqIndirect *output = (ftAssocReqIndirect*)mac_buffer_tx;

            output->fcf = FCF_ASSOC_RESP_IND;
            output->seq = macConfig.dsn++;
            output->panid = macConfig.panId;
            output->destAddr = macConfig.parentShortAddress;  // send all packets to parent
            output->srcAddr = macConfig.shortAddress;
            output->cmd = ASSOCIATION_REQUEST;
            output->parentAddr = input->parentAddr;
            output->macAddr = input->macAddr;
            output->type = input->type;

            radioSendData(sizeof(ftAssocReqIndirect), (u8*)output);
        }
    }
#endif
}

void
mrd(void)
{
    // Send packet after routing packet was sent
#if (NODE_TYPE == COORD)
    {
        // Packet has already been created, just send it.
        ftData *frame = (ftData *)(mac_buffer_tx+1);
        radioSendData(sizeof(ftData), (u8 *)frame);
    }
#endif
}

/**
    Route a data packet.  There are a few simple rules for routing packets:

   - Routers: if a packet is from my parent, and the destination is my
     child, send the packet to the child.

   - Routers: if a packet is from my parent, and the destination is
     not my child, then send to the last default route.

   - Routers: if a packet is from my child, send the packet to my parent.

   - Coordinator: send the packet down the tree, preceded by a routing
     packet if necessary.
*/
void
macRouteData(void)
{
#if (NODE_TYPE == ROUTER || NODE_TYPE == COORD)
    {
        ftData *frame = (ftData *)(mac_buffer_tx+1);

        // Copy RX to TX buffer
        macCopyRxToTx();

        blink_led0(20);

        // See if this frame is in the child table
        if (macIsChild(frame->finalDestAddr))
        {
            // send frame to child
            frame->seq = macConfig.dsn++;
            frame->destAddr = frame->finalDestAddr;
            frame->srcAddr = macConfig.shortAddress;
            debugMsgStr("\r\nRoute data to child");

            // See if the child is sleeping
            if (NODE_SLEEP && macIsChildSleeping(frame->finalDestAddr))
                // Send it later, after child is awake
                macHoldFrame(frame->finalDestAddr, (u8*)frame, (u8)*mac_buffer_tx - 2);
            else if (frame->destAddr != BROADCASTADDR)
                // Send the frame along (subtract 2 bytes from length for checksum length)
                radioSendData(*mac_buffer_tx - 2, (u8*)frame);
        }
        else // Not child node, send up or down the chain
        {
            if (NODE_TYPE == COORD)
            {
                // Send down the chain
                frame->seq = macConfig.dsn++;
                frame->destAddr = macGetTopParent(frame->finalDestAddr);
                frame->srcAddr = DEFAULT_COORD_ADDR;

                debugMsgStr("\r\nRoute data from Coord");

                if (macSendRoutingPacket(frame->finalDestAddr))
                {
                    macSetAlarm(MAC_RP_DELAY, mrd);
                    macConfig.busy = true;
                    // will send packet later, get out now
                    return;
                }
            }
            else if (NODE_TYPE == ROUTER)
            {
                // See if we should route up or down
                if (frame->srcAddr == macConfig.parentShortAddress)
                {
                    // this frame is from parent, send it down default route
                    frame->seq = macConfig.dsn++;
                    frame->destAddr = macConfig.lastRoute;
                    frame->srcAddr = macConfig.shortAddress;
                    debugMsgStr("Route data down to ");
                    debugMsgHex(macConfig.lastRoute);
                }
                else
                {
                    // this frame is from child, send up the chain
                    frame->seq = macConfig.dsn++;
                    frame->destAddr = macConfig.parentShortAddress;
                    frame->srcAddr = macConfig.shortAddress;
                    debugMsgStr("\r\nRoute data up");
                }
            }
            // Make sure we're not broadcasting frames
            if (frame->destAddr != BROADCASTADDR)
                // Send the frame along (subtract 2 bytes from length for checksum length)
                radioSendData(*mac_buffer_tx - 2, (u8*)frame);
        }

    }
#endif
}

/**
   Create and send a routing packet, returns non-zero if packet was sent.

   param:  shortAddr The short address of the end node, or the short
   address of the parent of the final destination node.

   returns:  Zero if no routing packet was sent (probably because it was not needed).
   returns:  Non-zero if a routing packet was sent.
*/
u8
macSendRoutingPacket(u16 shortAddr)
{
#if (NODE_TYPE == COORD)
    {
        volatile u16 topParent; // Return to caller

        u8 buf[128];
        ftRouting *frame = (ftRouting *)buf;
        u8 hops; // Number of router-to-router hops

        // Calculate the routing portion of the routing packet
        hops = macCreateRoute(shortAddr, frame);

        if (hops < 3)
            // Could not create routing packet, or no packet needed
            return 0;

        frame->fcf = FCF_ROUTE;
        frame->seq = macConfig.dsn++;
        frame->panid = macConfig.panId;
        frame->srcAddr = DEFAULT_COORD_ADDR;
        frame->cmd = ROUTING_PACKET;

        topParent = frame->destAddr;

        // See if we can skip routing because the last packet went there.
        if (macGetLastRoute(topParent) == frame->shortAddr)
            return 0;

        // Save route for this child router
        macSetLastRoute(topParent, frame->shortAddr);

        // And finally send the packet over the air
        radioSendData(sizeof(ftRouting) + (hops-3)*2,(u8*)frame);

        // Set the flag to say we sent a routing packet.
        return 1;
    }
#endif
    return 0;
}

/** @} */

