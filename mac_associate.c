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

  $Id: mac_associate.c,v 1.3.4.2 2009/06/05 02:21:07 mvidales Exp $
*/

#include <string.h>
#include <stdio.h>
#include "mac.h"
#include "radio.h"
#include "timer.h"
#include "mac_associate.h"
#include "system.h"
#include "mac_route.h"
#include "mac_data.h"
#include <avr/pgmspace.h>

/** Number of children for a router */
#define MAXCHILDREN 50

typedef struct {
    u16 childAddr;
    u8  wakeup:1;
    u8  sleeping:1;
} __attribute__((packed)) tChildTableItem;

// Global variables.
#if (NODE_TYPE != COORD)
static u8 associationTimer;
#else
static u16 dropParent, dropChild;
#endif


u8 macIsChild(u16 shortAddr);
u16 addNode(u8 type, u64 *macAddr, u16 parentAddr);
u16 macGetNodeAddr(u64 *macAddr);
void macNotifyDrop(u16 parent, u16 child);

/**
   @ingroup mac
   @{
   @defgroup mac_associate MAC association
   @{

   This module handles the association of nodes to the network. It
   includes code for both the coordinator node and the router/end
   nodes.  See the RUM Application Note (AVR2070) to see a detailed
   description of how the protocol operates.

   There are two basic modes of association - direct and indirect.
   Direct packets are sent to and from the new node being associated
   to the network.  Indirect packets are passed up the chain of
   routers to the coordinator, and back down to the new parent node.
   The parent node must translate between direct and indirect nodes.
   In the case where a new node is associating to the coordinator,
   only direct packets are used.
*/

///   Table of associated nodes, only instantiated for coordinator.
static associatedNodes_t nodes[MAXNODES * (NODE_TYPE == COORD)];



#if (NODE_TYPE ==ROUTER)
static u16 childNodeNdx;     // Used for macFirstChild() and macNextChild()
/** Table of children nodes, only instantiated for router nodes. */
static tChildTableItem childNodes[MAXCHILDREN * (NODE_TYPE == ROUTER)];
#endif


/**
     Associate this node with a network.  The caller must supply
   the channel to use and what short address to associate with.  Most
   likely the scan function macScan() has been called to obtain these
   parameters.

   This function only sends a direct association request, because this
   node does not have a short address yet.

   param:  shortAddr The short address of the prospective parent node.
   param:  channel The channel of the network to join.
*/
void macAssociate(u16 shortAddr, u8 channel)
{
#   if (NODE_TYPE != COORD)
    {
        // This node sends a direct association packet to its new parent
        // The first association request is always a direct request

        // Create a struct pointer to the global variable...
        ftAssocReqDirect* direct_assoc_frame = (ftAssocReqDirect*)(mac_buffer_tx+1);

        // Set the channel.
        macSetOperatingChannel(channel);

        // Create the association command frame.
        direct_assoc_frame->fcf = FCF_ASSOC_REQ_DIRECT;

        // Increment and set the data sequence number.
        direct_assoc_frame->seq = macConfig.dsn++;
        // Complete the addressing fields.
        direct_assoc_frame->panid = macConfig.panId;
        direct_assoc_frame->destAddr= shortAddr;
        memcpy(&(direct_assoc_frame->srcAddr), &(macConfig.longAddr), sizeof macConfig.longAddr);
        direct_assoc_frame->cmd = ASSOCIATION_REQUEST;
        direct_assoc_frame->parentAddr = shortAddr;
        direct_assoc_frame->type = NODE_TYPE;

        // send data to radio.
        radioSendData(sizeof(ftAssocReqDirect), (u8 *)direct_assoc_frame);

        // Set a time out timer for an "association period" of 100 ms. This gets executed if there is no Association response in "Timeout"
        associationTimer = macSetAlarm(ASSOCIATION_TIMEOUT, macAssociationConfirm);
    }
#   endif
}

/**
     This function is called when the MAC receives an association
   response frame.  This function stops the association timeout timer,
   and call the callback function   appAssociateConfirm.
*/
void macAssociationConfirm(void)
{
#   if (NODE_TYPE != COORD) // Coords cannot associate
    {
        // We have finished the association process, kill the association timeout timer...
        macTimerKill(associationTimer);

        if ( macConfig.associated )
        {
        	ftAssocRespDirect * resp = (ftAssocRespDirect*) (mac_buffer_rx+1);

        	macConfig.parentShortAddress = resp->srcAddr;
    		macConfig.shortAddress = resp->shortAddr;
    		radioSetShortAddress(macConfig.shortAddress);

    		debugMsgStr("\r\nAssociated to Dev ");
    		debugMsgHex(macConfig.parentShortAddress);
    		debugMsgStr(" as ");
    		debugMsgHex(macConfig.shortAddress);
        }
        else
        	debugMsgStr("\r\nFailed to associate");


        // Let app know that we got an associate packet
        appAssociateConfirm(macConfig.associated);
    }
#   endif
}

// This function is only a timer callback, from a timer set in macAssociationResponse()
#if (NODE_TYPE == COORD)
static void sendAsResInd(void)
{

    {
        // Packet has already been created, just send it.
        ftAssocRespIndirect *response = (ftAssocRespIndirect*)(mac_buffer_tx+1);
        radioSendData(sizeof(ftAssocRespIndirect), (u8 *)response);
    }

}
#endif
/**
     This function is called when an association request is
   received by this node (and this node is coordinator).  The frame is
   held in the global mac_buffer array.
*/
void
macAssociationResponse(void)
{
    // This code is only run by the coordinator node, as it is the only node
    // that originates the association response.
    // The coordinator must send either a direct or indirect response, depending
    // on which type of request it gets.

#if (NODE_TYPE == COORD)
    {
        u16 shortAddress = DEFAULT_COORD_ADDR;
        u16 childAddr;
        u8 rpSent; // Was a routing packet sent?

        u16 fcf = mac_buffer_rx[1] + mac_buffer_rx[2]*0x100;
        if(fcf == FCF_ASSOC_REQ_IND) // association request indirect
        {
            ftAssocReqIndirect  *request  = (ftAssocReqIndirect *)(mac_buffer_rx+1);
            ftAssocRespIndirect *response = (ftAssocRespIndirect*)(mac_buffer_tx+1);

            // See if this node is already in the table
            DECLARE64(request->macAddr);
            childAddr = macGetNodeAddr(USE64(request->macAddr));
            if (childAddr != BROADCASTADDR && nodes[childAddr].nodeType)
            {
                // This node already has been associated, notify its
                // parent to drop them from the table
                if (nodes[childAddr].parentShortAddress == DEFAULT_COORD_ADDR)
                    // This node was coord's child, remove it
                    macRemoveChild(childAddr);
                else
                {
                    // This node has already been associated, and if it was
                    // previously associated to another router, then have that
                    // router drop this node from its parent table
                    if (request->parentAddr != nodes[childAddr].parentShortAddress)
                        macNotifyDrop(nodes[childAddr].parentShortAddress, childAddr);
                }
            }

            // first, send routing packet if necessary
            rpSent = macSendRoutingPacket(request->parentAddr);

            // Add the new node to the table.
            shortAddress = addNode(request->type, USE64(request->macAddr), request->parentAddr);

            response->shortAddr = shortAddress;

            response->fcf = FCF_ASSOC_RESP_IND;
            response->seq = macConfig.dsn++;
            response->panid   = macConfig.panId;
            response->dstAddr  = request->srcAddr;
            response->srcAddr   = macConfig.shortAddress; // If coord this is the same as macCoordShortAddress
            response->cmd = ASSOCIATION_RESPONSE;
            response->parentAddr = request->parentAddr;
            response->macAddr = request->macAddr; // long address of device that requested association

            // send data to radio, after a delay for routing packet to get out of the way
            macSetAlarm(rpSent ? MAC_RP_DELAY : 0, sendAsResInd);
            macConfig.busy = true;

            // Alert the application that we added a child
            appChildAssociated(shortAddress);
        }
        else if(fcf == FCF_ASSOC_REQ_DIRECT) // association response direct
        {
            volatile ftAssocReqDirect *request = (ftAssocReqDirect *)(mac_buffer_rx+1);
            volatile ftAssocRespDirect *response = (ftAssocRespDirect *)(mac_buffer_tx+1);

            // See if this node is already in the table
            DECLARE64(request->srcAddr);
            childAddr = macGetNodeAddr(USE64(request->srcAddr));


            if (childAddr != BROADCASTADDR)
            {
                // This node already has been associated, notify its
                // parent to drop them from the table
                if (nodes[childAddr].parentShortAddress == DEFAULT_COORD_ADDR)
                    // Remove from Coordinator table
                    macRemoveChild(childAddr);
                else
                    // Notify node's parent to drop
                    macNotifyDrop(nodes[childAddr].parentShortAddress, childAddr);
            }
            // Add node to table, coord is parent
            shortAddress = addNode(request->type,
                                   USE64(request->srcAddr),
                                   DEFAULT_COORD_ADDR);

            response->fcf = FCF_ASSOC_RESP_DIRECT;

            // Increment and set the data sequence number.
            response->seq = macConfig.dsn++;

            // Complete the addressing fields.
            response->panid = macConfig.panId;
            response->dstAddr = request->srcAddr;
            response->srcAddr = macConfig.shortAddress; // If coord this is the same as macCoordShortAddress
            // Generate the payload data and record the node address data in the Pan
            // Coord's address table.
            response->cmd = ASSOCIATION_RESPONSE;
            response->shortAddr =  shortAddress;

            // send data to radio.
            radioSendData(sizeof(ftAssocRespDirect), (u8 *)response);
        }

        debugMsgStr("\r\nMac Associated node: ");
		debugMsgInt(shortAddress);
		debugMsgStr(" Hops = ");
		if (macGetHopCount(shortAddress) ==1)
			debugMsgStr("direct");
		else
		{
			debugMsgInt(macGetHopCount(shortAddress) -1);
		}
		debugMsgStr("\r\n");


        // Let app know that a node was associated
        appNodeAssociated(shortAddress);
    }
#endif
}

/**
   Add a node to the table of nodes, and return a short address for
   the new node.  This code only runs on the coordinator.

   param:  type The node type, see   NODE_TYPE.

   param:  macAddr The long 8-byte MAC address of the new node.

   param:  parentAddr Short address of the parent of the new node.

   returns:  The short address for the new node.
*/
u16
addNode(u8 type, u64 *macAddr, u16 parentAddr)
{
    // Find the node in the table, return it or find new one
#   if (NODE_TYPE == COORD)
    {
        u16 i=0xffff;

        nodes[0].nodeType = COORD;

        // Check to see if the node was previously stored in the table.
        for (i=1;i<MAXNODES;i++)
        {
            if(nodes[i].nodeLongAddress == *macAddr)
                break;
        }

        // If node was not found, pick first empty spot
        if (i >= MAXNODES)
        {
            for (i=1;i<MAXNODES;i++)
                if (!nodes[i].nodeType)
                    break;
        }
        if (i >= MAXNODES)
            // Table is full, sorry
            return 0xffff;

        // Node address not found. index "i" is still set to last empty
        // element in the array. Index "i" becomes the node short address.
        nodes[i].nodeType = type;
        nodes[i].nodeLongAddress = *macAddr;
        nodes[i].parentShortAddress = parentAddr;
        nodes[i].lastRoutedAddress = 0;
        nodes[i].wakeup = 0;

        return i;
    }
#   endif
    return 0;
}

/**
   Create a routing packet.  This function fills in a routing packet
   payload with the routes required. See the protocol documentation
   for details.

   param:  shortAddr The short address of the destination node to route
   a packet to.

   param:  frame Pointer to the routing frame which will be sent.  This
   function fills in the payload of the frame with hop addresses.  If
   frame is NULL, then the routing frame is not used.

   returns:  The number of hops from the coordinator to the node.
*/
u8 macCreateRoute(u16 shortAddr, ftRouting *frame)
{
#    if (NODE_TYPE == COORD)
    {
        u16 node=shortAddr;

        if (shortAddr >= MAXNODES)
            return 1;

        // Is this a valid address?
        if  (!nodes[shortAddr].nodeType)
        {
            debugMsgStr("\r\nBad dest node (RP)");
            // Bad node, don't bother sending anything
            return 0;
        }

        // Is this node a direct child node?
        if (node == DEFAULT_COORD_ADDR ||
            nodes[node].parentShortAddress == DEFAULT_COORD_ADDR)
            // There is one hop from this node to coordinator
            return 1;

        // Erase first hop (from dest end) because it should not end up in
        // the payload table of hops.
        node = nodes[node].parentShortAddress;

        // Count the hops to node
        volatile u8 hops=1;
        u16 *payload=NULL;

        if (frame)
            payload = &frame->shortAddr;

        for(;;)
        {
            if (!nodes[node].nodeType)
                // bad short address, quit
                return 0;

            // Go for another hop
            hops++;

            // see if we're just one hop away
            if (nodes[node].parentShortAddress == DEFAULT_COORD_ADDR)
            {
                // Save highest parent to frame
                if (frame)
                    frame->destAddr = node;
                return hops;
            }

            // store node address in payload
            if (payload)
                *payload++ = node;

            // find node's parent, the new node for next cycle
            node = nodes[node].parentShortAddress;
        }
    }
#endif
    return 0;
}


/**
   Find a node's short address given the node's long MAC address.

   param:  macAddr Pointer to a long (8-byte) MAC address.

   returns:  The short address of the node, or   BROADCASTADDR on
   error.
*/
u16
macGetNodeAddr(u64 *macAddr)
{
#   if (NODE_TYPE == COORD)
    {
        u16 i;

        for (i=1;i<MAXNODES;i++)
            if (nodes[i].nodeType &&
                nodes[i].nodeLongAddress == *macAddr)
                return i;
    }
#   endif
    // Not found, return nonsense address
    return BROADCASTADDR;
}


/**
   Determine if a given node is a child if this node.

   param:  shortAddr The short address of the node to look up.

   returns:  true if shortAddr is a child of this node.
   returns:  false if shortAddr is not a child of this node.
 */
u8
macIsChild(u16 shortAddr)
{
    //returns true if the node with shortAddr is a child of this node

#    if (NODE_TYPE == COORD)
    {
        // Check to see if the node was previously stored in the table.
        if (shortAddr >= MAXNODES)
            return 0;
        if(nodes[shortAddr].nodeType &&
           nodes[shortAddr].parentShortAddress == DEFAULT_COORD_ADDR)
                return true;
    }
#elif (NODE_TYPE == ROUTER)
    {
        // This is a router node, just search child table
        u8 i;

        // Coordinator is not a child of a router ;-)
        if (!shortAddr)
            return false;

        // See if the node is in the table
        for (i=0;i<MAXCHILDREN;i++)
            if (childNodes[i].childAddr == shortAddr)
            // This node already exists, return true
                return true;
    }
#endif
    return false;

}

/**
   Determine if a given node is a sleeping child of this node.  This
   function assumes that addr is a child node of this node.

   param:  shortAddr The short address of the node to look up.

   returns:  true if shortAddr is sleeping.
   returns:  false if shortAddr is not sleeping.
*/
u8
macIsChildSleeping(u16 shortAddr)
{
    //returns true if the node with shortAddr is a child of this node

#   if (NODE_TYPE == COORD && NODE_SLEEP)
    {
        // Check to see if the node was previously stored in the table.
        if (shortAddr >= MAXNODES)
            return 0;
        if(nodes[shortAddr].sleeping)
                return true;
    }
#elif (NODE_TYPE == ROUTER && NODE_SLEEP)
    {
        // This is a router node, just search child table
        u8 i;

        // Coordinator is not a child of a router ;-)
        if (!shortAddr)
            return false;

        // See if the node is in the table
        for (i=0;i<MAXCHILDREN;i++)
            if (childNodes[i].childAddr == shortAddr &&
                childNodes[i].sleeping)
                return true;
    }
#endif
    return false;

}

/**
   Add a newly-associated node to this router's child table.  Only
   directly-connected nodes are added to the table.  This function
   only applies to router nodes.

   param:  shortAddr The short address of the newly associated node to
   add to the table.
*/
void macAddChild(u16 shortAddr)
{
#if (NODE_TYPE == ROUTER)
    {
        u8 i;

        // See if the node is already in the table
        for (i=0;i<MAXCHILDREN;i++)
            if (childNodes[i].childAddr == shortAddr)
            {
                // This node already exists, just turn sleep off
                childNodes[i].wakeup = false;
                childNodes[i].sleeping = false;

                debugMsgStr("\r\nRe-adding child ");
                debugMsgHex(shortAddr);
                return;
            }

        debugMsgStr("\r\nAdding child node ");
        debugMsgHex(shortAddr);

        // Add the node to the first open spot
        for (i=0;i<MAXCHILDREN;i++)
            if (!childNodes[i].childAddr)
            {
                // Found an empty spot, store the address and leave
                childNodes[i].childAddr = shortAddr;
                childNodes[i].wakeup = false;
                childNodes[i].sleeping = false;

                // Notify the application
                appChildAssociated(i);

                return;
            }
    }
#endif
}

/**
   Remove a node from the child table.  This function applies to both
   routers and coordinators.

   param:  shortAddr The short address of the node to remove from the
   table.
*/
void macRemoveChild(u16 shortAddr)
{
#    if (NODE_TYPE == COORD)
    {
        // remove from coord table, clear data
        if (shortAddr >= MAXNODES)
            return;
        nodes[shortAddr].nodeType = 0;
        nodes[shortAddr].parentShortAddress = 0;
        nodes[shortAddr].lastRoutedAddress =0;
    }
#endif
#   if (NODE_TYPE == ROUTER)
    {
        u8 i;

        debugMsgStr("\r\nRemoving child node ");
        debugMsgHex(shortAddr);

        // find the node and clear it
        for (i=0;i<MAXCHILDREN;i++)
            if (childNodes[i].childAddr == shortAddr)
            {
                childNodes[i].childAddr = 0;
                break;
            }
    }
#endif
}

/**
   Find the parent of a given node.

   param:  shortAddr The short address of the child node.

   returns:  The short address of the parent of the given node.
 */
u16 macGetParent(u16 shortAddr)
{
    // Coord only, return short address of the node's parent
#   if (NODE_TYPE == COORD)
    {
        if (shortAddr >= MAXNODES)
            return 0;
        return nodes[shortAddr].parentShortAddress;
    }
#endif
    return 0;
}

/**
   Find the "topmost" parent for a node (the one connected to the
   coord).  This function only runs on the coordinator node.

   param:  addr The short address of the node to find the top parent of.

   returns:  The short address of the node's top parent.  This node is
   the first hop from coordinator to addr.
*/
u16 macGetTopParent(u16 addr)
{
#   if (NODE_TYPE == COORD)
    {
        if (addr >= MAXNODES)
            return 0;
        for(;;)
        {
            if (!nodes[addr].nodeType)
                // This is a bad node, get out
                return 0;
            if (macIsChild(addr))
                return addr;
            // jump up the chain one step.
            addr = nodes[addr].parentShortAddress;
        }
    }
#endif
    return 0;
}

/**
   Find the first child in the network table of a given parent node.
   This function only runs on the coordinator node.

   param:  addr The short address of the parent node.

   returns:  The short address of the first child node of addr in the
   network table.

   returns:  Zero if no child was found.
*/
u16 findFirstChild(u16 addr)
{
    // Return first child of node addr, or 0x0 if none found

#   if (NODE_TYPE == COORD)
    {
        u16 i;
        for (i=1;i<MAXNODES;i++)
        {
            if (nodes[i].nodeType &&
                nodes[i].parentShortAddress == addr)
            {
                // found the first child in the list
                return i;
            }
        }
        // none found, return zero
    }
#endif
    return 0;
}


/**
   Find the next sibling node of a given node.  This function only
   runs on the coordinator node.

   param:  addr The short address of the node to query.

   returns:  The short address of the next sibling of the node given in
   the network table.

   returns:  Zero if no next sibling node was found.
*/
u16
findNextSibling(u16 addr)
{
#   if (NODE_TYPE == COORD)
    {
        if (addr >= MAXNODES)
            return 0;

        // find the next sibling of node addr
        u16 parent = nodes[addr].parentShortAddress;
        u16 i;

        for (i=addr+1;i<MAXNODES;i++)
        {
            if (nodes[i].nodeType &&
                nodes[i].parentShortAddress == parent)
                // Found the next sibling
                return i;
        }
    }
    // none found, return zero
#endif
    return 0;
}

/**
   Print the network tree in a semi-graphical format, to be viewed on
   a terminal screen.  This code only applies if the   DEBUG flag
   has been turned on.
 */
void macPrintTree(void)
{
#if DEBUG
#       if (NODE_TYPE == COORD)
        {
            // debug stuff, print tree of all nodes
            u16 ndx=0;
            u16 newNdx;
            //            u16 *p;
            u8 level=0;
            u8 i;
            associatedNodes_t *node;
            char nodeTypeStr[4] = "CCRE"; // Coord, Router, End


            debugMsgStr("\r\nTable Data:\r\n");
            for(i=1;i<MAXNODES;i++)
            {
                node = &nodes[i];
                // see if this node is used
                if (!node->nodeType)
                    continue;

                // Print addresses
                u32 low = node->nodeLongAddress & 0xffffffff;
                u32 high = node->nodeLongAddress >> 32;
                sprintf(debugStr,strcpy_P(dbg_buff,PSTR("%c%02X %c%02X %02X 0x%08lX%08lX\r\n")),
                        nodeTypeStr[node->nodeType],
                        i,
                        nodeTypeStr[nodes[node->parentShortAddress].nodeType],
                        nodes[i].parentShortAddress,
                        nodes[i].lastRoutedAddress,
                        high,
                        low);
                debugMsgStr_d(debugStr);

            }


            debugMsgStr("\r\nNetwork Tree\r\n");

            for(;;)
            {
                // method:
                // 1. Print current node number and some spaces
                // 2. Find first child, if yes, goto step one
                // 3. If no, find next sibling of node.
                // 4. If no, print CRLF, bunch of spaces, and goto step 1
                // 5. If yes (sibling), go back to parent
                // 6. If parent == coord, quit
                // 7. Goto step 3
                node = &nodes[ndx];

                sprintf(debugStr,strcpy_P(dbg_buff,PSTR("%c%02X ")),nodeTypeStr[node->nodeType], ndx);
                debugMsgStr_d(debugStr);

                if ((newNdx = findFirstChild(ndx)))
                {
                    // Child found, just go again
                    ndx = newNdx;
                    level++;
                    continue;
                }
                else
                {
                fns:
                    // No child found, look for a sibling
                    if ((newNdx = findNextSibling(ndx)))
                    {
                        // Found a sibling, print CRLF and some spaces
                        ndx = newNdx;
                        debugMsgStr("\r\n");
                        for (i=0;i<level;i++)
                            debugMsgStr("    ");
                        continue;
                    }
                    else
                    {
                        // No sibling found, go back to parent and look for more.
                        ndx = nodes[ndx].parentShortAddress;
                        if (!ndx)
                            // parent is coord, we are done
                            break;
                        // Go up one level
                        level--;
                        // and goto the find next sibling part
                        goto fns;
                    }
                }
            }

        }
#		endif
#       if (NODE_TYPE == ROUTER)
        {
            // Print child table

            u16 i;

            debugMsgStr("Child Table for node ");
            sprintf(debugStr,strcpy_P(dbg_buff, PSTR("%04X\r\n")),macConfig.shortAddress);
            debugMsgStr_d(debugStr);

            for(i=0;i<MAXCHILDREN;i++)
                if (childNodes[i].childAddr)
                {
                    sprintf(debugStr,strcpy_P(dbg_buff,PSTR("%04X\r\n")),childNodes[i].childAddr);
                    debugMsgStr_d(debugStr);
                }
            debugMsgCrLf();
        }
#		endif
#endif
}

#if (NODE_TYPE == COORD)
// These two functions are targets to the timer function, called after a time
// to send a Drop Notification packet and a routing packet to a router.
static void mnd1(void)
{

    {
        // Finally, send the packet
        ftDropChild frame;

        debugMsgStr("\r\nDropping child ");
        debugMsgHex(dropChild);
        debugMsgStr(" from parent ");
        debugMsgHex(dropParent);

        frame.fcf =  FCF_DATA;
        frame.seq =           macConfig.dsn++;
        frame.panid =         macConfig.panId;
        frame.destAddr =      macGetTopParent(dropParent);
        frame.srcAddr =       DEFAULT_COORD_ADDR;
        frame.finalDestAddr = dropParent;
        frame.originAddr =    DEFAULT_COORD_ADDR;
        frame.type =          DROP_CHILD_FRAME;
        // pass child address to drop
        frame.childAddr =     dropChild;

        // Send frame over the radio
        if (frame.destAddr != DEFAULT_COORD_ADDR)
            // don't send a message to ourself
            radioSendData(sizeof(ftDropChild) ,(u8*)&frame);
    }

}
#endif
void mnd2(void)
{
#    if (NODE_TYPE == COORD)
    {
        // Send a routing packet and then the drop packet
        u8 rpSent;  // Routing packet sent?

        rpSent = macSendRoutingPacket(dropParent);
        macSetAlarm(rpSent ? MAC_RP_DELAY : 0, mnd1);
    }
#	endif
}

/**
   Sends a frame from the coordinator notifying a router to drop a
   child from its table.  This occurs as a result of a re-association,
   where the new node already exists in the network table, as a child
   of a parent different from its new parent.

   param:  parent Short address of the former parent node.

   param:  child Short address the of the child node to drop from the
   parent's node table.
*/
void macNotifyDrop(u16 parent, u16 child)
{
#    if (NODE_TYPE == COORD)
    {
        // wait a while, then send the routing packet and drop packet.

        // Save params for final drop packet.
        dropParent = parent;
        dropChild = child;
        macSetAlarm(MAC_NOTIFY_DELAY, mnd2);
        macConfig.busy = true;
    }
#	endif

}

/**
   Get the last routed address of one of the router nodes in the
   network.

   param:  parent Short address of the router node being queried.

   returns:  The short address of the last route address for the router
   (parent).
*/
u16 macGetLastRoute(u16 parent)
{
    if (NODE_TYPE == COORD && parent < MAXNODES)
        return nodes[parent].lastRoutedAddress;
    return 0;
}

/**
   Sets the last routed address for a router node.  This code only
   exists on the coordinator node.

   param:  parent Short address of the router node.

   param:  addr Short address of the last routed address for a node.
*/
void macSetLastRoute(u16 parent, u16 addr)
{
    if (NODE_TYPE == COORD && parent < MAXNODES)
        nodes[parent].lastRoutedAddress = addr;
}

/**
   Gets one entry in the table of nodes.  This code only runs on the
   coordinator.

   param:  index The short address of the node to retrieve.

   returns:  Pointer to the   associatedNodes_t structure for the
   given node.
*/
associatedNodes_t *macGetNode(u16 index)
{
    // return pointer to node data.
    if (NODE_TYPE == COORD && index < MAXNODES)
        return &nodes[index];
    return NULL;
}

#   if (NODE_TYPE == COORD)
/**
   Initializes the   nodes array of   associatedNodes_t
   structures.  Really, this only initializes the one entry for the
   coordinator, since we can assume the other entries are all zeroed
   on startup.
*/
void macInitNodes(void)
{

    {
        // Init nodes array - wipe out all of the table
        memset(&nodes, 0, sizeof(nodes));
    }

}
#	endif

/**
   Set a child node's entry to be woken up. This code runs on both the
   coordinator and router:

   - On the coordinator, this function causes a wakeup packet to be
     sent to the sleeping node's parent.

   - On the router parent node, this function causes an entry to be
     set in the child node, so that when a packet is received from the
     sleeping child node, a wakeup packet will immediately be sent.

   See   macChildIsAwake for more information about wakeup.

   param:  addr Short address of child node.
*/
void macWakeChildNode(u16 addr)
{
#   if (NODE_SLEEP)
    {
#        if (NODE_TYPE == COORD)
        {
            // Find parent of the node
            u16 parent = macGetParent(addr);
            if (parent == DEFAULT_COORD_ADDR)
            {
                // this node is parent, set to wake up.
                if (addr < MAXNODES)
                    nodes[addr].wakeup = true;
            }
            else
            {
                // Send a packet to the node's parent router
                macWakeRequest(parent, addr);
            }
        }
#		endif
#       if (NODE_TYPE == ROUTER)
        {
            // Search child nodes for child
            u8 i;
            for (i=0;i<MAXCHILDREN;i++)
            {
                if (childNodes[i].childAddr == addr)
                {
                    // Set this node to wake up
                    childNodes[i].wakeup = true;
                    return;
                }
            }
            // Not found, too bad.
        }
#		endif
    }
#	endif
}

/**
   Clear the wakeup flag for the given child node.  This is called
   when the child wakes up and sends a WAKE_NODE packet back to the
   parent.

   param:  addr The short address of the child node that has woken up.
*/
void macClearChildWakeFlag(u16 addr)
{
#if (NODE_SLEEP)
    {
        // Is the address my child?
        if (!macIsChild(addr))
            return;

#       if (NODE_TYPE == ROUTER)
        {
            u8 i;

            for (i=0;i<MAXCHILDREN;i++)
            {
                if (childNodes[i].childAddr == addr)
                {
                    // Clear the wakeup flag
                    childNodes[i].wakeup = 0;
                    break;
                }
            }
        }
#		endif
#       if (NODE_TYPE == COORD)
        {
            if (addr >= MAXNODES)
                return;
            nodes[addr].wakeup = 0;
        }
#		endif
    }

#	endif
}


/**
   Wake child node by sending it a packet.  This function checks to
   see if the incoming data packet is from a child of this node, and
   if the child node is sleeping, the wakeup packet is sent.

   This function is called on the receipt of every packet.

   param:  frame Pointer to ftData struct, the received packet.
*/
void
macChildIsAwake(ftData *frame)
{
#   if (NODE_SLEEP && ( NODE_TYPE == ROUTER || NODE_TYPE == COORD))
    {
        u16 addr = frame->originAddr;

        // Is the packet from my child?
        if (!macIsChild(addr))
            return;

#        if (NODE_TYPE == ROUTER)
        {
            u8 i;

            for (i=0;i<MAXCHILDREN;i++)
            {
                if (childNodes[i].childAddr == addr)
                {
                    // Set sleeping flag based on whether high bit is set
                    childNodes[i].sleeping = (frame->type & 0x80 ? 1 : 0);

                    // Wakeup a child if necessary
                    if (childNodes[i].wakeup)
                    {
                        /* Wakeup the node, unless this is a wake packet,
                           in which case the node is already awake and
                           simply acknowledging that fact. */
                        if (frame->type == WAKE_NODE)
                            // Clear the wakeup flag, child is awake now
                            childNodes[i].wakeup = 0;
                        else
                            // Child has sent us something, and he must be woken up.
                            // Send a wakeup packet immediately.
                            macWakeRequest(addr, 0);
                        break;
                    }

                    // Send a stored frame if there is one.
                    if (NODE_SLEEP)
                        macSendStoredFrame(addr);
                }
            }
        }
#endif
#        if (NODE_TYPE == COORD)
        {
            if (addr >= MAXNODES)
                return;

            // Set sleeping flag based on whether high bit is set
            nodes[addr].sleeping = (frame->type & 0x80 ? 1 : 0);

            if (nodes[addr].wakeup)
            {
                // Send the end node a wakeup frame
                macWakeRequest(addr, 0);
                // Clear the wakeup flag
                nodes[addr].wakeup = 0;
            }
            // Send a stored frame if there is one.
            if (NODE_SLEEP)
                macSendStoredFrame(addr);
        }
#endif
    }
#endif
    // Reset the sleeping bit in the frame.
    frame->type &= ~0x80;
}

/**
   Returns the short address of the first child node (routers only),
   or zero if none found.
*/
u16 macFirstChild(void)
{
#   if (NODE_TYPE == ROUTER)
    {
        for (childNodeNdx=0;childNodeNdx<MAXCHILDREN;childNodeNdx++)
            if (childNodes[childNodeNdx].childAddr)
                return childNodeNdx;
    }
#	endif
    return 0;
}

/**
   Returns the short address of the next child node (routers only), or
   zero if none found.  Must call macFirstChild() before calling this
   for the first time.
*/
u16 macNextChild(void)
{
#   if (NODE_TYPE == ROUTER)
    {
        for (;childNodeNdx<MAXCHILDREN;childNodeNdx++)
            if (childNodes[childNodeNdx].childAddr)
                return childNodeNdx;
    }
#	endif
    return 0;
}

/**
      This will return the hop count for the associated node requested.

    param:  node_short Short address of node the search for hop count.
*/
u8 macGetHopCount(u16 node_short)
{
    u8 hops = 1;

    for(;;)
    {
        if (!nodes[node_short].nodeType)
            // bad short address, quit
            return 0;

        // see if we're just one hop away
        if (nodes[node_short].parentShortAddress == DEFAULT_COORD_ADDR)
        {
            return hops;
        }

        // find node's parent, the new node for next cycle
        node_short = nodes[node_short].parentShortAddress;

        // Go for another hop
        hops++;
    }
    return 0;
}



/** @} */
/** @} */

