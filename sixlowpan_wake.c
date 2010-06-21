 /* Copyright (c) 2008-2009  ATMEL Corporation
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

  $Id: sixlowpan_wake.c,v 1.3 2009/05/21 23:20:15 mvidales Exp $
*/
#include "mac.h" /* Must include this first to define NODETYPE */
#include "system.h"  // Must get IPV6LOWPAN flag

#if (IPV6LOWPAN )

#if (NODETYPE == COORD)
#include "tuip.h"
#include "interfaces.h"
#include "net/uip.h"
#include "net/uip-nd6.h"
#include "net/uip-netif.h"
#include "net/sicslowpan.h"
#endif // NODETYPE == COORD

#include <string.h>
#include "mac_associate.h" /* Needed for MAXNODES */
#include "mac_data.h"
#include "timer.h"
#include "system.h"
#include "sixlowpan_wake.h"
#include "sleep.h"

/**
 * @file
 *         AVR 6LoWPAN Sleeping-Node Support
 */

/**
 * @addtogroup avr6lowpan
 * @{
 * @defgroup avr6lowpansleeping 6LoWPAN Sleeping End Nodes Support
 * @{
 */

#if APP != SENSOR
u16 frameInterval = 20;
u16 appInterval = 15;
#else
u16 appInterval = 1;
#endif

//#define VISUALIZE_SLEEPING

/**   Visualize the sleep cycle
 *
 * Define VISUALIZE_SLEEPING to make LED 2 (when available) light up
 * whenever the device is awake. This allows you to easily see when the device
 * wakes up in response to a wake-up request.
 */

#ifdef VISUALIZE_SLEEPING
    #define NOT_SLEEPING() LED_ON(2)
    #define SLEEPING()     LED_OFF(2)
#else
    #define NOT_SLEEPING()
    #define SLEEPING()
#endif


#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF          ((struct uip_udp_hdr *)&uip_buf[UIP_LLIPH_LEN])

void macsixlowpanDataRequest(u16 addr, u8 len, u8 * data);
void sixlowpan_application_periodic(void);

static uint8_t wakeSent = 0;

#if ((NODETYPE == COORD) )

typedef struct {
    uip_ipaddr_t destAddr;
    uint16_t     destPort;
} t_wakeup_information;

/**
 *   A list of where each node should send it's wake-up packet.
 */
t_wakeup_information wakeup_information[MAXNODES];


/**
 *   Function called whenever 6LoWPAN Data is received, to check
 *        if this data indicates a node that was asleep is now awake.
 *
 * param:  src The address of the node which received the data.
 *
 *        This function looks for UDP data sent to the IP address & port
 *        where the 'wakeup' command should be sent. If it sees it, the
 *        node is marked as being awake. The data must be in normal IP
 *        format in uip_buf.
 */
void sixlowpanSleep_dataReceived (uint16_t src)
{
    //Only continue the check if we are awaiting wakeup...
    if (wakeup_information[src].destPort)
    {
        if ((HTONS(UIP_UDP_BUF->destport) == wakeup_information[src].destPort) &&
            (uip_ipaddr_cmp(&UIP_IP_BUF->destipaddr, &wakeup_information[src].destAddr)))
            {
                wakeup_information[src].destPort = 0;
            }
    }

    return;
}

/**
 *   Called when the 'wakeup' command has been sent to an end node.
 *        We remember who sent the command, so we can send it back to
 *        the node later.
 *
 * param:  shortaddress The address of the node which is being sent to.
 * param:  ipaddr The IP address which sent the wakeup command.
 * param:  port The UDP port which the node should respond to.
 */
void sixlowpanSleep_wakeupSent(uint16_t shortaddress, uip_ipaddr_t * ipaddr, uint16_t port)
{
    wakeSent = 1;

    //Check this is possible
    if (shortaddress >= MAXNODES)
    {
        return;
    }

    //Store the information...
    wakeup_information[shortaddress].destAddr = *ipaddr;
    wakeup_information[shortaddress].destPort = port;

    return;
}

/**
 *   Called when the 'i am awake, anybody I should talk to?' command is
 * received from an end node.
 *
 * param:  shortaddress The address of the node which sent this request.
 */
void sixlowpanSleep_awakeReceived(uint16_t shortaddress)
{

    /* Check if we are waiting on them */
    if (!wakeup_information[shortaddress].destPort)
    {
        return;
    }

    /* We create a special packet, a UDP packet with the previously
     * stored srcAddr & srcPort. */

    uip_len = 0;

    /** IPv6 Header **/
    UIP_IP_BUF->vtc = 0x60; /* IPv6, 0 traffic class */
    UIP_IP_BUF->flow = 0;   /* 0 flow */
    UIP_IP_BUF->len[0] = UIP_UDPH_LEN + 1;    /* 8 UDP Header Bytes + 1 UDP Payload */
    UIP_IP_BUF->len[1] = 0;
    UIP_IP_BUF->ttl = 64;   /* 64-hop limit */
    UIP_IP_BUF->proto = UIP_PROTO_UDP;

    /* Make packet look like it comes from original server */
    UIP_IP_BUF->srcipaddr = wakeup_information[shortaddress].destAddr;

    /* Destination is based on short address.. */
    //The coordinator has a short address of zero, so we just use it's IPv6 address
    //here. We can then replace the lower two bytes with the short address we want..
    UIP_IP_BUF->destipaddr = uip_netif_physical_if[INTERFACE_802154].addresses[1].ipaddr;
    UIP_IP_BUF->destipaddr.u8[14] = MSB(shortaddress);
    UIP_IP_BUF->destipaddr.u8[15] = LSB(shortaddress);

    /* UDP Header */
    UIP_UDP_BUF->destport = HTONS(WAKEUP_PORT);
    UIP_UDP_BUF->srcport = HTONS(wakeup_information[shortaddress].destPort);
    UIP_UDP_BUF->udplen = HTONS(UIP_UDPH_LEN + 1);

    /* UDP Payload */
    uip_buf[UIP_IPH_LEN + UIP_UDPH_LEN] = 'w';

    /* Checksummer */
    UIP_UDP_BUF->udpchksum = 0;

    /* Calculate UDP checksum. */
    UIP_UDP_BUF->udpchksum = ~(uip_udpchksum());
    if(UIP_UDP_BUF->udpchksum == 0) {
      UIP_UDP_BUF->udpchksum = 0xffff;
    }

    uip_len = UIP_IPH_LEN + UIP_UDPH_LEN + 1;

    /* Send data out */
    tuip_interface_set(INTERFACE_802154);
    tcpip_ipv6_output();
}

/**
 *   Called by RUM when sending a packet failed.
 *
 * If sending a packet failed, this might mean
 * the node was asleep we were trying to send to.
 */
void sixlowpanSleep_packetFailed(void)
{
    if (wakeSent)
        macWakeChildNode(macConfig.lastDestAddr);

    wakeSent = 0;

    return;
}

void sixlowpanSleep_packetSucceed(void)
{
    wakeSent = 0;

    return;
}


#else

void sixlowpanSleep_packetSucceed(void)
{
    //   sixlowpanSleep_checkSleep();
 }

void sixlowpanSleep_packetFailed(void)
{
    //    sixlowpanSleep_checkSleep();

}

#endif //NODETYPE == COORD


#if ((NODETYPE != COORD) |)
static uint8_t perSetup = 0;

/**
 *   Initilize the sleep & timing subroutines.
 *
 *        This function must be called before any of the sleep code will
 *        work. Once started the sleep code takes care of restarting timers
 *        as needed, so no other user action is required.
 */
void sixlowpanSleep_init(void)
{
    NOT_SLEEPING();
    sixlowpanSleep_setupPeriodic();
}

/**
 *   Checks for any pending messages.
 *
 * Sends an "I am awake, anybody I should talk to?" message to the
 * edge router / coordinator.
 */
void sixlowpanSleep_sendAwake(void)
{
    /* We only want to send this once per             *
     * wake cycle, as otherwise we are wasting power! */

    uint8_t frame[1];

    frame[0] = ATMEL_GET_AWAKE;

    if (!wakeSent || !RUMSLEEP)
    {
        macsixlowpanDataRequest(DEFAULT_COORD_ADDR, 1, frame);
        wakeSent = 1;
    }
}


/**
 *   Called when activity occurs on the node.
 *
 *        This is used to reset the 'timeout' counter. Acivity counts
 *        as either data TX'd or RX'd.
 *
 */
void sixlowpanSleep_activity(void)
{
    uint16_t newTimeout;

    newTimeout = (u16)macGetTime() + (u16)SIXLOWPAN_TIMEOUT_MS;

#if (IPV6LOWPAN && RUMSLEEP)
    /* Set a new timeout value */
    macSetTimeout(newTimeout);
#endif
}

/**
 *   Called whenever the node times out.
 *
 * When this function is called the node becomes eligable for sleep.
 *
 */
void sixlowpanSleep_timeout(void)
{
    macConfig.sleeping = true;
}

static uint16_t sleepCount;

/**
 *   Called periodically by the node.
 *
 *        This function is called at a user-specified interval.
 *        Handles sleep timeouts & calling the application periodic
 *        function.
 */
void sixlowpanSleep_periodic(void)
{
    sixlowpanSleep_sendAwake();

    perSetup = 0;

    sleepCount++;

    if ((sleepCount == SIXLOWPAN_PERIODIC_APP_TIME) && SIXLOWPAN_PERIODIC_APP_TIME)
    {
        /* Send the periodic message */
        macSetAlarm(7, sixlowpan_application_periodic);
        sleepCount = 0;
    }

    sixlowpanSleep_checkSleep();
}

/**
 *   Will set the node to sleep if it should be sleeping.
 *
 *        Calling this function causes the node to transmit a 'should
 *        i be awake' message, and if no response to go to sleep if
 *        sleeping is enabled.
 */
void sixlowpanSleep_checkSleep(void)
{
    //Don't do anything when waiting on response!
    //if (macConfig.busy)
    //    return;

    if ((macConfig.sleeping) && (RUMSLEEP))
    {
       /* Check if anyone wants us to wake up... */
       //if (!VLP)
       //macSetAlarm(30, sixlowpanSleep_sendAwake);

       /* Otherwise go to sleep */
       macSetAlarm(50, sixlowpanSleep_sleep);
    }
    else
    {
        /* Setup periodic timer if we should be awake still! */
        sixlowpanSleep_setupPeriodic();
    }
}

/**
 *   Sets up the periodic timer.
 *
 *        When called sets up a timer to call the sixlowpanSleep_periodic()
 *        function at the appropriate time. When the node is awake, this
 *        function will be called every time the sixlowpanSleep_periodic()
 *        function is called.
 */
void sixlowpanSleep_setupPeriodic(void)
{
    //Don't let user call this multiple times! Would set multiple timers...
    if (perSetup)
        return;

    perSetup = 1;


	// Less than 6.5 seconds
	if (SIXLOWPAN_PERIODIC_TIME < 65)
	{
	    if (SIXLOWPAN_PERIODIC_TIME)
	        macSetAlarm(SIXLOWPAN_PERIODIC_TIME * 100, sixlowpanSleep_periodic);
	    else
	        macSetAlarm(5000, sixlowpanSleep_periodic); /* Default of 5 S */
	}
	// More than 65 seconds
	else
	{
		macSetLongAlarm(SIXLOWPAN_PERIODIC_TIME / 10 , sixlowpanSleep_periodic);
	}
}

/**
 *   Puts the node to sleep if it can.
 *
 *        If the node is unable to sleep, instead this function just
 *        sets up the periodic timer to run. If the node can sleep,
 *        the node will go to sleep between the periodic timer firing.
 */
void sixlowpanSleep_sleep(void)
{
    if (NODETYPE == ENDDEVICE)
    {
        // Don't sleep if we've been woken up, restart the
        // periodic timer & perform the 'awake' action
        // Don't wake up though if we are in VLP mode
        if (!macConfig.sleeping)
        {
            sixlowpanSleep_setupPeriodic();

           // if (!VLP)
            sixlowpanSleep_sendAwake();
        }

        //Check if the MAC Is busy - if so don't kill it,
        //let's try again in a bit
        //else if (macConfig.busy)
        //{
        //    macSetAlarm(10, sixlowpanSleep_sleep);
        //}
        else
        {
            //Going to sleep!
            wakeSent = 0;

            SLEEPING();
            if (SIXLOWPAN_PERIODIC_TIME)
                nodeSleep(SIXLOWPAN_PERIODIC_TIME);
            else
                nodeSleep(50); /* Default of 5 S */
            NOT_SLEEPING();

            sixlowpanSleep_periodic();
        }
    }
    else
    {
        sixlowpanSleep_setupPeriodic();
    }

    return;
}

#endif //NODETYPE != COORD

#endif


