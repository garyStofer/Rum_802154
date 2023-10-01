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

/// Support for changing all node function.

#define PING_ALL    1
#define REPORT_ALL  2
#define WAKE_ALL 	3

#if ( DEBUG && SERIAL)
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

static char * rf2xx_reg_names[]  = {R_1, R_2, R_3, R_4, R_5, R_6, R_7, R_8, R_9, R_10,
							 R_11, R_12, R_13, R_14, R_15, R_16, R_17, R_18, R_19, R_20,
							 R_21, R_22, R_23, R_24, R_25, R_26, R_27, R_28, R_29, R_30,
							 R_31, R_32, R_33, R_34, R_35, R_36, R_37, R_38, R_39, R_40,
							 R_41, R_42 };

/**
      This is an array of radio register values to be used when
           rf2xx_reg_dump() is called.  See the radio datasheet for
           details.
*/
const static u8 rf2xx_reg_enum[] PROGMEM =
    {RG_TRX_STATUS, RG_TRX_STATE, RG_TRX_CTRL_0, RG_TRX_CTRL_1, RG_PHY_TX_PWR,
     RG_PHY_RSSI, RG_PHY_ED_LEVEL, RG_PHY_CC_CCA, RG_CCA_THRES, RG_TRX_CTRL_2, RG_IRQ_MASK,
     RG_IRQ_STATUS, RG_VREG_CTRL, RG_BATMON, RG_XOSC_CTRL, RG_RX_SYN, RG_RF_CTRL_0, RG_XAH_CTRL_1,
     RG_FTN_CTRL, RG_RF_CTRL_0, RG_PLL_CF, RG_PLL_DCU, RG_PART_NUM, RG_VERSION_NUM, RG_MAN_ID_0,
     RG_MAN_ID_1, RG_SHORT_ADDR_0, RG_SHORT_ADDR_1, RG_PAN_ID_0, RG_PAN_ID_1,
     RG_IEEE_ADDR_0, RG_IEEE_ADDR_1, RG_IEEE_ADDR_2, RG_IEEE_ADDR_3, RG_IEEE_ADDR_4,
     RG_IEEE_ADDR_5, RG_IEEE_ADDR_6, RG_IEEE_ADDR_7, RG_XAH_CTRL_0, RG_CSMA_SEED_0,
     RG_CSMA_SEED_1, RG_CSMA_BE};

static void allNodesCB(void);


static void
rf2xx_reg_dump(void)
{

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

}


/** Serial MENU:
   Print prompts for debug mode.  This is in a separate function
   because we want a delay before printing the next prompt, in case
   there is some status data printed from the last command.
 */
void printPrompt(void)
{
#    if (NODE_TYPE == COORD)
    {
        debugMsgStr("\r\n(d)ump (t)able (i)nfo (p)ing (c)han");
        debugMsgStr("\r\n(r)eading (n)ame (w)ake (H)ygrometer");
        debugMsgStr("\r\n(S)low all, (F)ast all, (P)ing all, sl(E)ep all, (W)ake all\r\n");
    }
#   else
        debugMsgStr("\r\nd=dump t=table i=info p=ping \n\r");
#endif
}
#endif // DEBUG and SERIAL

#if (NODE_TYPE == COORD && SERIAL)
/**
   When called, this will either Ping or request data from all associated nodes.
   A reading interval can also be set if desired.
*/
static void
allNodes(u8 func, u16 val)
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
			debugMsgStr("\r\nInterval node ");
			debugMsgInt(nodeNdx);
			debugMsgStr(" = ");
			debugMsgInt(value);
			sensorRequestReading(nodeNdx, value);
			break;

		case WAKE_ALL:
			debugMsgStr("\r\nWaking node ");
			debugMsgInt(nodeNdx);
			macTask();
			macWakeChildNode(nodeNdx);
			break;
		}
	}

	// Prepare for next node in the list
	nodeNdx++;
	if (nodeNdx < MAXNODES)
		// Let's go again
		macSetAlarm(nodeFound ? 250 : 1, allNodesCB);
	else
		// All done, cancel any further action
		function = value = 0;
}


/**
      This is used as the allNodes callback for repeating the function operation.
*/
static void
allNodesCB(void)
{
    allNodes(0,0);
}
#endif


void
appMenu(void)
{
	// perform periodical things.

#if (SERIAL)
	if (NODE_TYPE == COORD && 	(PIND & (1 << PD5)))	// Machine interface -- Pull PD5 to gnd for human if
	{
		char ch;

		if (macConfig.busy)   // try again when mac is not busy
			return;

		if (! serial_ischar()) // noting to do
			return;

		ch = serial_getchar();

		switch (ch)
		{
		case 'W':
			allNodes(WAKE_ALL, 0);
			break;

		case 'E':
			allNodes(REPORT_ALL, 0);
			break;

		case 'S':
			allNodes(REPORT_ALL, 300);
			break;
		case 'F':
			allNodes(REPORT_ALL, 10);
			break;


		}
	}
#if (DEBUG )
	else												// Human interface
	{
		char ch;
		static u16 addr = 0;

		if (! serial_ischar()) // noting to do
			return;

		if (macConfig.busy)   // try again when mac is not busy
			return;

		ch = serial_getchar();

		debugMsgCrLf();
		switch (ch)
		{
		case 'd':
			// reg dump
			rf2xx_reg_dump();
			break;

		case 'i':
			// print info
			sprintf(debugStr,
					strcpy_P(dbg_buff, PSTR("\r\nshort = %04X\r\nparent = %04X\r\nroute=%04X\r\n")),
					macConfig.shortAddress, macConfig.parentShortAddress,
					macConfig.lastRoute);
			debugMsgStr_d(debugStr);
			sprintf(debugStr, strcpy_P(dbg_buff, PSTR("chan = %d\r\n")),
					macConfig.currentChannel);
			debugMsgStr_d(debugStr);
			debugMsgStr("PAN ID = 0x");
			debugMsgHex(macConfig.panId);
			debugMsgCrLf();
			u32 low = macConfig.longAddr;
			u32 high = macConfig.longAddr >> 32;
			sprintf(debugStr,
					strcpy_P(dbg_buff, PSTR("long = 0x%08lX%08lX\r\n")), high,
					low);
			debugMsgStr_d(debugStr);

			sprintf(debugStr,
					strcpy_P(dbg_buff, PSTR("assoc = %s\r\nHops = %04X\r\n")),
					macConfig.associated ? "true" : "false",
					macConfig.hopsToCoord);
			debugMsgStr_d(debugStr);
			sprintf(debugStr, strcpy_P(dbg_buff, PSTR("rand = %02X\r\n")),
					radioRandom(8));
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
			sprintf(debugStr, strcpy_P(dbg_buff, PSTR("%u, Rev=%d\r\n")), pn,
					radio_register_read(RG_VERSION_NUM));
			debugMsgStr_d(debugStr);

	#if (NODE_TYPE != COORD )
			{
				debugMsgStr("Name=");
				debugMsgStr_d(sensorGetName());
				debugMsgCrLf();
			}
	#endif
			// Report compile options
			sprintf(debugStr, strcpy_P(dbg_buff, PSTR("Sleep = %s\r\n")),
			NODE_SLEEP ? "Yes" : "No");
			debugMsgStr_d(debugStr);
			debugMsgStr_d("Sleep timed by 32Khz Xtal on Timer2\r\n");
			sprintf(debugStr, strcpy_P(dbg_buff, PSTR("CPU Freq = %dMHz\r\n")),
					(int) (F_CPU / 1000000UL));
			debugMsgStr_d(debugStr);

			sprintf(debugStr,
					strcpy_P(dbg_buff, PSTR("Interval = %d00 msec\r\n")),
					EE_GetFrameInterval());
			debugMsgStr_d(debugStr);

			sprintf(debugStr, strcpy_P(dbg_buff, PSTR("Demo mode = %s\r\n")),
			DEMO ? "Yes" : "No");
			debugMsgStr_d(debugStr);
			break;

		case 'p':
			// ping
			debugMsgStr("\r\nEnter short addr:");
			serial_gets(debugStr, 50, true);
			addr = atoi(debugStr);
			macPing(PING_REQ_FRAME, addr);
			break;


	#if (NODE_TYPE != COORD)
			case 'S': 	// sleep

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

	# if (NODE_TYPE == COORD )
		case 't':
			// print table
			macPrintTree();
			break;

		case 'c':
			// change coordinator channel
			debugMsgStr("\r\nEnter new channel");
			serial_gets(debugStr, 50, true);
			ch = atoi(debugStr);
			debugMsgStr("\r\nEnter new PANID or leave blank for same");
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
			break;

		case 'r':
			// Set reading interval
			// get address and time
			debugMsgStr("\r\nRequest data from node:");
			serial_gets(debugStr, 50, true);
			addr = atoi(debugStr);
			u16 time;
			debugMsgStr("\r\nReading interval time (100mS increments):");
			serial_gets(debugStr, 50, true);
			time = atoi(debugStr);
			// Grab pending frames, so we don't trigger sending req frame
	// ??????
			macTask();
			sensorRequestReading(addr, time);
			break;

		case 'H': // send an argument to SHT11
			// get address of node
			debugMsgStr("\r\nAddress which node:");
			serial_gets(debugStr, 50, true);
			addr = atoi(debugStr);

			short arg_1;
			short arg_2;
			debugMsgStr("\r\nHyg Thresh:");
			serial_gets(debugStr, 50, true);
			arg_1 = atoi(debugStr);

			debugMsgStr("\r\nHyg Hyst:");
			serial_gets(debugStr, 50, true);
			arg_2 = atoi(debugStr);

			// Send argument value and index to node
			sensorSendArgument(addr, arg_1, arg_2);
			break;

		case 'n':    // Name a node
			debugMsgStr("\r\nName which node:");
			serial_gets(debugStr, 50, true);
			addr = atoi(debugStr);
			debugMsgStr("\r\nEnter name:");
			serial_gets(debugStr, 50, true);
			sensorSendNodeName(addr, debugStr);
			break;

			//This takes a end node out of Sleep Mode, i.e. it's always receiving and consumes more power because of that
			// There is currently no way to put the node back to sleep other than resetting the end node.
	// TODO: a modification to the wake call could be done to allow a node to go back to sleep again
		case 'w':         // wake an end node
			debugMsgStr("\r\nWake which node:");
			serial_gets(debugStr, 50, true);
			addr = atoi(debugStr);
			// Must process any rx'd packets before running macWake...
			macTask();
			macWakeChildNode(addr);
			break;

		case 'W':         // Affect all nodes
			debugMsgStr("\r\nWake All nodes");
			allNodes(WAKE_ALL, 0);
			break;

		case 'E':         // Affect all nodes
			debugMsgStr("\r\nSleep All nodes");
			allNodes(REPORT_ALL, 0);
			break;

		case 'S':         // Affect all nodes
			debugMsgStr("\r\nAll nodes Interval 30sec");
			allNodes(REPORT_ALL, 300);
			break;

		case 'M':         // Affect all nodes
			debugMsgStr("\r\nAll nodes Interval 10sec");
			allNodes(REPORT_ALL, 100);
			break;

		case 'F':
			debugMsgStr("\r\nAll nodes Interval 1sec");
			allNodes(REPORT_ALL, 10);
			break;

		case 'P':         // Affect all nodes
			debugMsgStr("\r\nPing All nodes");
			allNodes(PING_ALL, 0);
			break;
	#endif

		default:
			// Delay a bit to allow for other messages (ping resp) to print.
				macSetAlarm(250, printPrompt);
			break;
		}


	}
#endif
#endif	// serial && debug

}
