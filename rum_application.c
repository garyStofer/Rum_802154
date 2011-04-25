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
#include "sixlowpan_wake.h"
#include <avr/pgmspace.h>
#include "serial.h"
#include "EE_prom.h"


#define tuip_init_802154(a, b)
//#include <avr/eeprom.h>
// You can program the EEPROM with macaddress this way, using an ELF file
//u8 EEMEM macaddress[8] = {0xde, 0xed, 0xbe, 0xef, 0x11, 0x22, 0x33, 0x44};
//u8 EEMEM macaddress[8] = {0x01, 0x23, 0x45, 0x56, 0x78, 0x9a, 0xbc, 0xde};
//u8 EEMEM macaddress[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
#include "avr_sixlowpan.h"
	#if (IPV6LOWPAN == 0)
		#define sixlowpan_hc01_gen_rs(a)
	#endif
void sixlowpan_application_init(void);
static u8 pingTimer;


extern tSerialFrame SerialFrame[];

#define UDP_PORT_COMMANDS 0xF0B0
#define UDP_PORT_RESPONSE 0xF0B1

extern u8 simulateButton;
extern u16 frameInterval;

extern uint8_t destipAddr[];
extern uint8_t googleipAddr[];
extern uint8_t serveripAddr[];

#if (DEBUG && SERIAL)
static u8 streamMode=0;        ///< Are we in streaming mode?
#endif

/**
   @addtogroup app
   @{

   This is a sample application, intended to show how the Route Under
   MAC can be used to create a simple application.

     app_network_start Joining the network

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

#if DEBUG
/**
      This is an array of radio register names for serial output
           used when rf2xx_reg_dump() is called.  See the radio
           datasheet for details.
*/
static const char R_1[]  PROGMEM="TRX_STATUS";
static const char R_2[]  PROGMEM="TRX_STATE";
static const char R_3[]  PROGMEM="TRX_CTRL_0";
static const char R_4[]  PROGMEM="TRX_CTRL_1";
static const char R_5[]  PROGMEM="PHY_TX_PWR";
static const char R_6[]  PROGMEM="PHY_RSSI";
static const char R_7[]  PROGMEM="PHY_ED_LEVEL";
static const char R_8[]  PROGMEM="PHY_CC_CCA";
static const char R_9[]  PROGMEM="CCA_THRES";
static const char R_10[] PROGMEM="TRX_CTRL_2";
static const char R_11[] PROGMEM="IRQ_MASK";
static const char R_12[] PROGMEM="IRQ_STATUS";
static const char R_13[] PROGMEM="VREG_CTRL";
static const char R_14[] PROGMEM="BATMON";
static const char R_15[] PROGMEM="XOSC_CTRL";
static const char R_16[] PROGMEM="RX_SYN";
static const char R_17[] PROGMEM="RF_CTRL_0";
static const char R_18[] PROGMEM="XAH_CTRL_1";
static const char R_19[] PROGMEM="FTN_CTRL";
static const char R_20[] PROGMEM="RF_CTRL_1";
static const char R_21[] PROGMEM="PLL_CF";
static const char R_22[] PROGMEM="PLL_DCU";
static const char R_23[] PROGMEM="PART_NUM";
static const char R_24[] PROGMEM="VERSION_NUM";
static const char R_25[] PROGMEM="MAN_ID_0";
static const char R_26[] PROGMEM="MAN_ID_1";
static const char R_27[] PROGMEM="SHORT_ADDR_0";
static const char R_28[] PROGMEM="SHORT_ADDR_1";
static const char R_29[] PROGMEM="PAN_ID_0";
static const char R_30[] PROGMEM="PAN_ID_1";
static const char R_31[] PROGMEM="IEEE_ADDR_0";
static const char R_32[] PROGMEM="IEEE_ADDR_1";
static const char R_33[] PROGMEM="IEEE_ADDR_2";
static const char R_34[] PROGMEM="IEEE_ADDR_3";
static const char R_35[] PROGMEM="IEEE_ADDR_4";
static const char R_36[] PROGMEM="IEEE_ADDR_5";
static const char R_37[] PROGMEM="IEEE_ADDR_6";
static const char R_38[] PROGMEM="IEEE_ADDR_7";
static const char R_39[] PROGMEM="XAH_CTRL_0";
static const char R_40[] PROGMEM="CSMA_SEED_0";
static const char R_41[] PROGMEM="CSMA_SEED_1";
static const char R_42[] PROGMEM="CSMA_BE";

static char * rf2xx_reg_names[] PROGMEM = {R_1, R_2, R_3, R_4, R_5, R_6, R_7, R_8, R_9, R_10,
							 R_11, R_12, R_13, R_14, R_15, R_16, R_17, R_18, R_19, R_20,
							 R_21, R_22, R_23, R_24, R_25, R_26, R_27, R_28, R_29, R_30,
							 R_31, R_32, R_33, R_34, R_35, R_36, R_37, R_38, R_39, R_40,
							 R_41, R_42 };

/**
      This is an array of radio register values to be used when
           rf2xx_reg_dump() is called.  See the radio datasheet for
           details.
*/
static u8 rf2xx_reg_enum[] PROGMEM =
    {RG_TRX_STATUS, RG_TRX_STATE, RG_TRX_CTRL_0, RG_TRX_CTRL_1, RG_PHY_TX_PWR,
     RG_PHY_RSSI, RG_PHY_ED_LEVEL, RG_PHY_CC_CCA, RG_CCA_THRES, RG_TRX_CTRL_2, RG_IRQ_MASK,
     RG_IRQ_STATUS, RG_VREG_CTRL, RG_BATMON, RG_XOSC_CTRL, RG_RX_SYN, RG_RF_CTRL_0, RG_XAH_CTRL_1,
     RG_FTN_CTRL, RG_RF_CTRL_0, RG_PLL_CF, RG_PLL_DCU, RG_PART_NUM, RG_VERSION_NUM, RG_MAN_ID_0,
     RG_MAN_ID_1, RG_SHORT_ADDR_0, RG_SHORT_ADDR_1, RG_PAN_ID_0, RG_PAN_ID_1,
     RG_IEEE_ADDR_0, RG_IEEE_ADDR_1, RG_IEEE_ADDR_2, RG_IEEE_ADDR_3, RG_IEEE_ADDR_4,
     RG_IEEE_ADDR_5, RG_IEEE_ADDR_6, RG_IEEE_ADDR_7, RG_XAH_CTRL_0, RG_CSMA_SEED_0,
     RG_CSMA_SEED_1, RG_CSMA_BE};
#endif

/**
      Dumps the RF2xx register contents to serial port.

    Note: The serial output will only be available if the   DEBUG
    macro is defined as non-zero.
*/
/*
void test_dump()
{

#if  DEBUG && SERIAL
	int i ;
	volatile int val ;
	for (i = 0; i<10; i++ )
	{
		    val =  pgm_read_byte(&rf2xx_reg_enum[i]);
			serial_puts_P((char*) pgm_read_word(&rf2xx_reg_names[i]));
	     	sprintf(debugStr,strcpy_P(dbg_buff, PSTR(" - %02X ")),  i);
            debugMsgStr_d(debugStr);
			debugMsgCrLf();
	}
#endif
}
*/

// Application variables
static u8 failCount;  ///< Number of transmit-to-parent failures before re-scanning.
volatile u8 gotPanCoordPing;

// Function declarations
void ledcallback(void);
void sixlowpan_button(void);

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
blink_red(u8 ms)
{
    Led0_on();
    macSetAlarm(ms, ledoff0);
}
void
blink_blue(u8 ms)
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
	blink_blue(1);

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
    debugMsgStr("\r\nPing response from "); debugMsgHex(pf->originAddr);
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
    blink_blue(LED_DELAY);
}

/**
   Application callback function.  Called when this node receives a
    ping response via IPv6
*/

/**
   Application callback function, called when the MAC receives an ACK
   packet from a node that we sent a packet to.
*/
void
appPacketSendSucceed(void)  // This gets called from MAC_EVENT_ACK in mac_Task()
{
    // Reset the failure count on a good packet to parent
    // (could also decrement failCount)

#if (NODETYPE != COORD)

	if (macIsScanning())	// nothing to do while scanning
	{
//		debugMsgStr("\r\nACK while scanning");
		return;
	}
//	debugMsgStr("\r\n AppPacketSendSucceeded");
	// figure out which way we were sending when the failure occurred
	if (macConfig.lastDestAddr == macConfig.parentShortAddress)
		failCount = 0;

	// Tell sensor app
#	if (APP == SENSOR )
		sensorPacketSendSucceed();
#endif
	// Tell IPv6 LOWPAN
	if (IPV6LOWPAN)
		sixlowpanSleep_packetSucceed();

#endif
}

/**
   Application callback function, called when the MAC fails to send a
   packet due to a channel access failure (air is too busy).
*/
void appPacketSendAccessFail(void)
{
	debugMsgStr("\r\n AppPacketSendAccessFail ");

	if (macIsScanning())
	{
		debugMsgStr("while scanning");
		return;
	}

    // Tell sensor app
#if (APP == SENSOR)
        sensorPacketSendFailed();
#endif
}

/**
   Application callback function, called when the MAC fails to receive
   an ACK packet from a node that we sent a packet to.
*/
void appPacketSendFailed(void)
{

    //Tell 6LoWPAN
    #if IPV6LOWPAN
        sixlowpanSleep_packetFailed();
    #endif

    u8 parentFailed;
    if (NODETYPE != COORD)
    {
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
        if (parentFailed && NODETYPE == ROUTER)
            macDataRequest(macConfig.parentShortAddress, 0, NULL);

        // Don't have a cow until we have missed a few packets in a row
        if (++failCount < 8)
        {
            // Tell sensor app
#			if (APP == SENSOR)
                sensorPacketSendFailed();
#			endif
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

#if (APP == SENSOR)
                sensorLostNetwork();
#endif
            macSetAlarm((radioRandom(8)+5) *10, macStartScan);
        }
        if (NODETYPE == ROUTER &&
            macIsChild(macConfig.lastDestAddr))
        {
            // Drop child from table if the failure was downstream
            macRemoveChild(macConfig.lastDestAddr);
            debugMsgStr("\r\nDropped child bcs packet send failed.");
        }
    }

}


void
rf2xx_reg_dump(void)
{
#if  DEBUG && SERIAL
    {
        u8 i,j,k,val;

        debugMsgStr("\r\n\r\nREG DUMP\r\n");

        k = sizeof(rf2xx_reg_enum);
        for(i=0;i<k;i++)
        {
            val =  radio_register_read(pgm_read_byte(&rf2xx_reg_enum[i]));

	        serial_puts_P((char*) pgm_read_word(&rf2xx_reg_names[i]));

			sprintf(debugStr,strcpy_P(dbg_buff, PSTR(" 0x%02X ")),  (uint16_t)val);
            debugMsgStr_d(debugStr);

            
			for (j=7;j<8;j--)
                // Print a bit
                debugMsgChr(((val >> j)&1 ? '1' : '0' ));
            debugMsgCrLf();
        }

        debugMsgCrLf();
    }
#endif
}

/**
   Application function, sends a data frame to the coordinator, and
   schedules another call in one second, using the timer function.

   This can be used for testing the network.  To send a real data
   frame, use macDataRequest().
 */
void
appSendDataFrameTest(void)
{
    // send data frames once per second
    if (NODETYPE != COORD)
        macDataRequest(0x00, 4, (u8*) ((NODETYPE == ENDDEVICE) ? "endd" : "rout"));

    // Send another data frame later
    macSetAlarm(1000, appSendDataFrameTest);
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
    // Write app code here -- This node has received a data frame.
    // Example code:
#if DEBUG==2
	debugMsgStr("appDataIndication->");
#endif
	blink_blue(LED_DELAY);
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
	 {
#if (APP == SENSOR  && NODETYPE == COORD)	// pass data to application
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
   // debugMsgStr("\r\n!App Child Associated");
	blink_blue(LED_DELAY);
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
	//debugMsgStr("\r\n!App NODE Associated");
}

# if(NODETYPE != COORD)

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
		blink_blue(200);

		/* 6lowpan association */
		if (IPV6LOWPAN == 1)
			sixlowpan_hc01_gen_rs();

		// If we are auto-sending data, start that process.
		if (APP == SENSOR )
                SensorStartReadings();	// init measurment sensors and start sending data to COORD

	}
	else
	{
		if (VLP)
		{
			// Sleep for 10 seconds, try again
			nodeSleep(100);
			macStartScan();
		}
		else
			// Try again in one second
		macSetAlarm(1000, macStartScan);
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
    if (NODETYPE == COORD)
    {
        macInit(channel);
        macStartCoord();
        debugMsgStr("\r\nStartup, I am THE coordinator on ch ");
        debugMsgInt(channel);
        macConfig.associated = true;

        if (IPV6LOWPAN == 1)
            //Start uIP stack with 802.15.4 interface
            tuip_init_802154(macConfig.panId, macConfig.shortAddress);
    }
}


/**
   Sample application function. Sends a ping packet to the network
   coordinator.
*/
void Ping_Coord_loop(void)
{
// TODO: why set Backoff times to 0
//	radio_register_write(RG_CSMA_BE,0);
    macPing(PING_REQ_FRAME, DEFAULT_COORD_ADDR);
    pingTimer = macSetAlarm(2500, Ping_Coord_loop);

}
/**
      This is used as the allNodes callback for repeating the function operation.
*/
void allNodesCB(void)
{
    allNodes(0,0);
}
/**
   When called, this will either Ping or request data from all associated nodes.
   A reading interval can also be set if desired.
*/
void allNodes(u8 func, u16 val)
{
#   if (NODETYPE == COORD)
    {
        static u8 nodeNdx = 0; // Incremented each time through
        static u8 function;
        static u16 value;
        u8 nodeFound=0;
        associatedNodes_t *node;

        // Check to see if this is first time through
        if (func)
        {
            // Set up for repeated calls to this function
            function = func;
            value = val;
            nodeNdx = 1;
        }

        // See if we are supposed to do something
        if (!function)
            return;

        // Get a node from table
        node = macGetNode(nodeNdx);

        // Do the operation for connected nodes
        if (node->nodeType)
        {
            nodeFound = 1;
            // Do the operation on it
            switch (function)
            {
            case PING_ALL:
                // Send a ping to the node
                debugMsgStr("\r\nPinging node ");
                debugMsgInt(nodeNdx);
                macPing(PING_REQ_FRAME, nodeNdx);
                break;
            case REPORT_ALL:
                // Change report time
#      			if (APP==SENSOR)
                {
                    debugMsgStr("\r\nInterval node ");
                    debugMsgInt(nodeNdx);
                    debugMsgStr(" = ");
                    debugMsgInt(value);
                    sensorRequestReading(nodeNdx, value);
                }
#				endif
                break;
            }
        }

        // Prepare for next node in the list
        nodeNdx++;
        if (nodeNdx < MAXNODES)
            // Let's go again
            macSetAlarm(nodeFound ? 250 : 1, allNodesCB);
        else
            // All done, cancel any furthur action
            function = value = 0;
    }
#endif
}

/** Serial MENU:
   Print prompts for debug mode.  This is in a separate function
   because we want a delay before printing the next prompt, in case
   there is some status data printed from the last command.
 */
void printPrompt(void)
{
#    if (NODETYPE == COORD)
    {
        debugMsgStr("\r\nd=dump t=table i=info p=ping s=stream c=chan");
        debugMsgStr("\r\nr=reading n=name w=wake P=pause H=Hygr ");
    }
#   else
        debugMsgStr("\r\nd=dump t=table i=info p=ping s=stream P=pause: ");
#endif
}


#define sicslowpan_hc01_gen_rs()

/**
   Sample application task loop function.  This function is meant to
   be called periodically.  It uses the serial port and button status
   for input, and implements a terminal interface for debugging use.
*/
void appTask(void)
{
    static u8 state=0;             // Used for button processing

    // perform periodical things.
    // check for button presses on RCB's
    if (BUTTON_PRESSED() || ( simulateButton && IPV6LOWPAN == 1 && NODETYPE != COORD ) )
    {
        if (!state)
        {
            if (pingTimer)
            {
                // stop pinging
                macTimerKill(pingTimer);
                pingTimer = 0;
            }
            else
            {
                // ping the coordinator
                debugMsgStr("\r\nPinging coord\r\n");
                if (IPV6LOWPAN == 1)
                {
                    simulateButton = 0;
                    sixlowpan_button();
                }
                else
                	Ping_Coord_loop();
            }

            state = 1;
        }
    }
    else
        state = 0;

#if (DEBUG && SERIAL)


    	if (serial_ischar() )
        {
            u8 n;
            char ch;
            static u16 addr=0;
         

            if (macConfig.busy)
                // try again when mac is not busy
                return;

            ch = serial_getchar();
            // Quit stream mode on Ctrl-t
            if (ch == 0x14)
                streamMode = 0;
            // In stream mode, send all serial data over the air.
            if (streamMode)
            {
                // Send the chars out over the air to dest
                n = 0;
                for(;;)
                {
                    // Build a string of chars waiting in the queue
                    debugStr[n++] = ch;
                    if (n >= 100)
                        break;
                    if (serial_ischar())
                        ch = serial_getchar();
                    else
                        break;
                }
                // And send it off to destination
                macDataRequest(addr, n, (u8*) debugStr);
            }
            else
            {
                debugMsgCrLf();
                switch (ch)
                {
                case 'd':
                    // reg dump
                    rf2xx_reg_dump();
                    break;
                case 't':
                    // print table
                    macPrintTree();
                    break;
                case 'i':
                    // print info
                    sprintf(debugStr,strcpy_P(dbg_buff, PSTR("\r\nshort = %04X\r\nparent = %04X\r\nroute=%04X\r\n")),
                            macConfig.shortAddress,
                            macConfig.parentShortAddress,
                            macConfig.lastRoute);
                    debugMsgStr_d(debugStr);
                    sprintf(debugStr,strcpy_P(dbg_buff, PSTR("chan = %d\r\n")), macConfig.currentChannel);
                    debugMsgStr_d(debugStr);
                    debugMsgStr("PAN ID = 0x");
                    debugMsgHex(macConfig.panId);
                    debugMsgCrLf();
                    u32 low = macConfig.longAddr;
                    u32 high = macConfig.longAddr >> 32;
                    sprintf(debugStr,strcpy_P(dbg_buff, PSTR("long = 0x%08lX%08lX\r\n")), high, low);
                    debugMsgStr_d(debugStr);

                    sprintf(debugStr,strcpy_P(dbg_buff,PSTR("assoc = %s\r\nHops = %04X\r\n")),
                            macConfig.associated ? "true" : "false",
                            macConfig.hopsToCoord);
                    debugMsgStr_d(debugStr);
                    sprintf(debugStr,strcpy_P(dbg_buff, PSTR("rand = %02X\r\n")), radioRandom(8));
                    debugMsgStr_d(debugStr);
                    // Radio part number
                    u16 pn = radioGetPartnum();
                    switch (pn)
                    {
                    case RF230:
                        pn = 230;
                        break;
                    case RF231:
                        pn = 231;
                        break;
                    case RF212:
                        pn = 212;
                        break;
                    default:
                        // Just report whatever number the radio chip gives.
                        break;
                    }
                    debugMsgStr("Part=RF");
                    sprintf(debugStr,strcpy_P(dbg_buff,PSTR("%u, Rev=%d\r\n")), pn, radio_register_read(RG_VERSION_NUM));
                    debugMsgStr_d(debugStr);
                    
#if (NODETYPE != COORD && APP == SENSOR)
                    {
                        debugMsgStr("Name=");
                        debugMsgStr_d(sensorGetName());
                        debugMsgCrLf();
                    }
#endif
                    // Report compile options
                    sprintf(debugStr,strcpy_P(dbg_buff, PSTR("Sleep = %s\r\n")), RUMSLEEP ? "Yes":"No");
                    debugMsgStr_d(debugStr);
                    debugMsgStr_d("Sleep timed by 32Khz Xtal on Timer2\r\n");
                    sprintf(debugStr,strcpy_P(dbg_buff,PSTR("CPU Freq = %dMHz\r\n")), (int)(F_CPU / 1000000UL));
                    debugMsgStr_d(debugStr);
                    
					if (APP == SENSOR)
                    {
                        sprintf(debugStr,strcpy_P(dbg_buff,PSTR("Interval = %d00 msec\r\n")), halGetFrameInterval() );
                        debugMsgStr_d(debugStr);
                    }
                    sprintf(debugStr,strcpy_P(dbg_buff, PSTR("6LoWPAN = %s\r\n")), IPV6LOWPAN ? "Yes":"No");
                    debugMsgStr_d(debugStr);
                    sprintf(debugStr,strcpy_P(dbg_buff,PSTR("Demo mode = %s\r\n")), DEMO ? "Yes":"No");
					debugMsgStr_d(debugStr);
                    break;

                case 'p':
                    // ping
                    debugMsgStr("\r\nEnter short addr:");
                    serial_gets(debugStr, 50, true);
                    addr = atoi(debugStr);
                    macPing(PING_REQ_FRAME, addr);
                    break;
                case 's':
                    // Send data stream
                    debugMsgStr("\r\nStream mode to addr:");
                    serial_gets(debugStr,50,true);
                    addr = atoi(debugStr);
                    streamMode = 1;
                    break;
                case 'c':
                    // change coordinator channel
#				    if (NODETYPE == COORD)
                    {
                        debugMsgStr("\r\nEnter new chan");
                        serial_gets(debugStr, 50, true);
                        ch = atoi(debugStr);
                        debugMsgStr("\r\nEnter new PANID");
                        serial_gets(debugStr, 50, true);

                        // Re-do the init stuff.
                        macInit(ch);
                        macStartCoord();
                        debugMsgStr("\r\nStartup, I am coordinator.\r\n");
                        macConfig.associated = true;

                        // Set PANID
                        addr = atoi(debugStr);
                        if (addr)
                        {
                            radioSetPanId(addr);
                            macConfig.panId = addr;
                        }
                    }
#endif
                    break;
                case 'r':
                    // Request reading from end node
#                   if (NODETYPE == COORD && APP == SENSOR)
                    {
                        // get address and time
                        debugMsgStr("\r\nRequest data from node:");
                        serial_gets(debugStr,50,true);
                        addr = atoi(debugStr);
                        u16 time;
                        debugMsgStr("\r\nReport time (100mS increments):");
                        serial_gets(debugStr,50,true);
                        time = atoi(debugStr);
                        // Grab pending frames, so we don't trigger sending req frame
                        macTask();
                        sensorRequestReading(addr, time);
                    }
#endif
                    break;
                case 'H':
                    // send an argument
#                   if (NODETYPE == COORD && APP == SENSOR)
                    {
                        // get address of node
                        debugMsgStr("\r\nAddress which node:");
                        serial_gets(debugStr,50,true);
                        addr = atoi(debugStr);

                        short arg_1;
                        short arg_2;
                        debugMsgStr("\r\nHyg Thresh:");
                        serial_gets(debugStr,50,true);
                        arg_1 = atoi(debugStr);

                        debugMsgStr("\r\nHyg Hyst:");
                        serial_gets(debugStr,50,true);
                        arg_2 = atoi(debugStr);

                        // Send argument value and index to node
                        sensorSendArgument(addr, arg_1, arg_2);
                    }
#endif
                    break;
                case 'S':
                    // sleep
#                   if (NODETYPE != COORD)
                    {
                        for(;;)
                        {
                            u8 count;
                            nodeSleep(20);
                            macPing(PING_REQ_FRAME, DEFAULT_COORD_ADDR);
                            // Get the ping response before going back to sleep
                            delay_us(6000);
                            for (count=0;count<100;count++)
                                macTask();
                            // Send out the response string
                            delay_us(1000);
                        }
                    }
#endif
                    break;
                case 'n':
                    // Name a node
#                  if (NODETYPE == COORD && APP == SENSOR)
                    {
                        debugMsgStr("\r\nName which node:");
                        serial_gets(debugStr,50,true);
                        addr = atoi(debugStr);
                        debugMsgStr("\r\nEnter name:");
                        serial_gets(debugStr,50,true);
                       sensorSendSetNodeName(addr, debugStr);
                    }
#endif
                    break;

                case 'w':
                    // wake an end node
#					if (NODETYPE == COORD)
                    {
                        debugMsgStr("\r\nWake which node:");
                        serial_gets(debugStr,50,true);
                        addr = atoi(debugStr);
                        // Must process any rx'd packets before running macWake...
                        macTask();
                        macWakeChildNode(addr);
                    }
#endif
                    break;

                case 'P':
                    // Pause serial display
                    debugMsgStr("\r\nPaused, press 'P' to unpause");
                    serial_toggle_pause();
                    break;

                case 'A':
                    // Do something to all nodes
#                   if (NODETYPE == COORD)
                    {
                        debugMsgStr("\r\nAll nodes - (r)eading, (p)ping:");
                        serial_gets(debugStr,50,true);
                        // Do the function
                        if (*debugStr == 'p')
                            allNodes(PING_ALL,0);
                        if (*debugStr == 'r' && APP == SENSOR)
                        {
                            debugMsgStr("\r\nReport time (100mS intervals):");
                            serial_gets(debugStr,50,true);
                            allNodes(REPORT_ALL, atoi(debugStr));
                        }
                    }
#endif
                    break;
                default:
                    break;
                }
                // Delay a bit to allow for other messages (ping resp) to print.
                macSetAlarm(250,printPrompt);
            }
        }
#endif


}


/** @} */
