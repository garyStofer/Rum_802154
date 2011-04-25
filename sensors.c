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
  $Id: sensors.c,v 1.2 2009/05/29 23:22:48 mvidales Exp $
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "radio.h"
#include "mac.h"
#include "mac_event.h"
#include "mac_associate.h"
#include "system.h"
#include "timer.h"
#include "sensors.h"
#include "mac_data.h"
#include "sleep.h"
#include "serial.h"
#include "EE_prom.h"
#include "i2c.h"


#define INTERACTIVE 1
#include <util/twi.h>

#if IPV6LOWPAN
#include "avr_sixlowpan.h"
#endif


/**
     This code demonstrates a simple wireless sensor network, where data
   collected by end nodes and routers is sent to the coordinator.

   This application cooperates with the default   app, and can be
   left out of the compilation if desired by setting the   APP flag
   to zero.

   For demonstration, it is possible to have sensor nodes send random
   data, which saves the effort of connecting real sensors to each
   node.  To use this "random data" mode, set   SENSOR_TYPE to  
   SENSOR_RANDOM_T

     sleeping Sleeping and waking

   The sensor application causes nodes to sleep and wake.  The idea is
   that an end node can sleep for some interval while it is waiting to
   send a reading.  When it is time to send a reading, the unit wakes
   up, captures a reading, transmits the reading to its parent, and
   then goes back to sleep.

   It is necessary to provide a way to wake up the node, so that we
   can change the data interval or to calibrate the node.

   The mechanism for doing this is to use the   ftWake frame as a
   signal to wake up.  When an end node is done tranmitting its data
   frame, it remains awake for a short period (about 50mSec) to be
   able to receive a wakeup frame from its parent.  If no frame is
   received in that time, a timer expires and the node goes back to
   sleep.  If a ftWake frame is received, then the node stays awake
   and stop sending frames.  The node can now be calibrated or a new
   data interval can be set by sending the node an   sftRequestData
   frame.

     sensorcalls Sensor application function calls

   Here is a listing of how the various sensor calls are made to
   implement the various sensor functions.

   @verbatim
 Coordinator                 Frame type over the air             Router/End node

                                      Read Data

 sensorRequestReading()    --->  REQ_READING_FRAME --->        sensorStartReadings()
 sensorRcvReading()        <---    READING_FRAME   <---        sendReading()
  (do something with reading)

                                    Node Name

 sensorSendSetNodeName()   --->    SET_NODE_NAME   --->        sensorSetNodeNameRcv()

                                                                (set node name)
 (to retrieve the node name,
 just get a data packet, which
 contains the node name.)
   @endverbatim

     contdata Sending data continuously

   When the sensorRequestReading() function is called on the
   coordinator, the sensor node is directed to report data
   periodically back to the coordinator.  The time argument to this
   function sets the interval between readings, in tenths of a
   second.  This value is stored in EEPROM, and the unit will resume sending data after a power cycle.

*/



#if ( NODETYPE == BC_ENDDEVICE)
static void
Sensor_BCSendReading( void)
{
	static tBC_Data s_data;

	u16 i;

	s_data.DataType = BC_SENSOR_DATA;

	//Using 12 lower bits from IEEE MAC address for unit ID
	halGetEeprom((u8*)(offsetof(tEepromContents, eepromMacAddress))+6 ,2,(u8*)&i);		// Getting the two LSB's from the IEEE MAC addr
	s_data.SensorUnitID = i;
	s_data.Readings[0].SensorType = Temp;
	s_data.Readings[1].SensorType = None; //  indicates that there are no more readings

	HAL_SELECT_ADC0();
	HAL_START_ADC();
	HAL_WAIT_ADC();

	s_data.Readings[0].ADC_Value = HAL_READ_ADC() ;

	macData_BC_Request( sizeof(s_data),(u8 *) &s_data);

	// restart the timer for the next transmission
	i = halGetFrameInterval();

	if (i <655 )
	  macSetAlarm( i * 100, Sensor_BCSendReading);
	else
	  macSetLongAlarm((i + 5)/10, Sensor_BCSendReading);
}

void StartSensorBC_interval(void)
{

	HAL_ADC_INIT();

	macSetAlarm(2000, Sensor_BCSendReading);// start loop with initial delay simply to get passt the initial association sequence

}
#endif

// Enable this file only if we are compiling in the sensor app.
#if (APP == SENSOR)

static u8 sensorSentReading; // flag - have we sent a reading and are waiting for a response?
static u8 sendReadingTimer;
//Initialize the sensor system by configuring the A/D converter
void Sensor_HW_Init(void)
{
#if (NODETYPE==ROUTER || NODETYPE==ENDDEVICE )
	debugMsgStr("\r\n Sensor HW init()");
	HAL_ADC_INIT();

#endif
}

/**
   Start the sensor-sending-data loop depending wheter or not the node has been previeously
   configured to send data repeadetly.

   This gets called upon completion of Associateion for Sensor nodes
*/
void
SensorStartReadings(void)
{
#if (NODETYPE==ROUTER || NODETYPE==ENDDEVICE )

	// This will start a repeated Sensor reading to be sent to the COORD node
	// based on the Sensor frame interval stored in the EEPROM
	if (halGetFrameInterval())
	{
		 sendReadingTimer = macSetAlarm(10, SensorSendReading);// Start the first reading almost right away
	}

#endif
}

#if (NODETYPE == COORD)
	u16 node_addr;
	char node_name[NAME_LENGTH];
#else
	struct{
			char name[8];  // Node name
		  } __attribute__((packed)) sensorInfo[1];

#endif


/**
   Returns a pointer to the name of this node.

   returns:  a pointer to the name of this node.
*/
#if (NODETYPE != COORD)
char *sensorGetName(void)
{
	halGetSensorName(&sensorInfo->name);
    return sensorInfo->name;
}
#endif

/** @} */
/**
   @name Router/End node sensor functions
   @{
*/

static void
sensorSleep(void)
{
// Only end nodes can sleep
#if(NODETYPE == ENDDEVICE && RUMSLEEP)

#if IPV6LOWPAN /* IPv6LOWPAN Has it's own sleep routines we use... */
        getInterval();
#else
	// go to sleep, then wakeup and send again
	halGetFrameInterval(&frameInterval);  // Just to be sure
	u8 sleepTime = frameInterval;
	if (!sleepTime)
		sleepTime = 1;

	// Don't sleep if we've been woken up.
// TODO: this seems wrong -- What happens to the SendReading -- Returing here breaks the sensor reading loop !!
	if (!macConfig.sleeping)
		return;

	nodeSleep(sleepTime);
	sendReading();

#endif
#endif
}

/**
   Send a single reading to the coordinator and go to sleep if
   required.
*/


// TODO there should be a way to turn the reading loop off again by the Coord for ex, or when battery runs low
void
SensorSendReading(void)
{
#if (NODETYPE == ROUTER || NODETYPE == ENDDEVICE)

	volatile float val;

	u16 i;
	sftSensorReading reading;

	if (! macConfig.associated)		// if not associtaed we don't know how to send
    	return;

	Led1_on();	// this gets cleared by: sensorPacketSendSucceed(),sensorPacketSendFailed() and sensorLostNetwork()

	//Using 12 lower bits from IEEE MAC address for unit ID
	// Getting the two LSB's from the IEEE MAC addr
	halGetEeprom((u8*)(offsetof(tEepromContents, eepromMacAddress))+6 ,2,(u8*)&i);
	reading.SensorUnitID = i;
	reading.Frametype = READING_FRAME;
// TODO: switch below is a constant case -- Will this get optimized out ?? Should be recoded via if, elseif for optimization??
	switch (SENSOR_TYPE)
	{
		case SENSOR_BMP085:
			reading.SensorType=BMP085;
			BMP085_Read(&reading.readings[0] , &reading.readings[1]);
			break;

		case SENSOR_HYG:
			reading.SensorType = Hyg;
			SHT1xRead(&reading.readings[0], &reading.readings[1]);
			break;

		case SENSOR_NET: // RSSI
			reading.SensorType = RSSI;
			reading.readings[0] = radioGetSavedRssiValue();
			break;

		case SENSOR_TMP100:
			reading.SensorType = TMP100;
			reading.readings[0] = TMP100_read();
			break;

		 case SENSOR_BARO:
			reading.SensorType = Baro;
			HP03_Read(&reading.readings[0] ,&reading.readings[1]);
			break;

		case SENSOR_GAS_HC:
			// Turn the heater of the gas sensor on -- Only needed the first time around, but delayed execution
			// in here to leave the heater off unless the device is associted
			DDRD |= 0x80;
			PORTD |= 0x80;
			reading.SensorType = V_Hc;
			HAL_SELECT_ADC6();
			HAL_START_ADC();
			HAL_WAIT_ADC();
			val = (float) HAL_READ_ADC() ;
			val *= 2.5e-3;		// in volts -- using 2.56V internal ref and x1 gain
			val*=2; 			// Voltage divider 2:1
			reading.readings[0]= val;
			break;

		case SENSOR_BAT:
			reading.SensorType = V_bat;
			HAL_SELECT_ADC7();
			HAL_START_ADC();
			HAL_WAIT_ADC();
			val = (float) HAL_READ_ADC() ;
			val *= 2.5e-3;		// in volts -- using 2.56V internal ref and x1 gain
			val *=16;			// Voltage divider 16:1
			reading.readings[0]= val;
			break;

		default:
			reading.readings[0] = reading.readings[1] = 0;
			debugMsgStr("\r\nInvalid Sensor Type requested !!");
			return;
	}



	debugMsgStr("\r\nSending SensorPacket");
	//char str[20];
	//sprintf((char *)str," %1.2f ",(double)val);
	//debugMsgStr_d((char *) str);

	// Send it off
#if IPV6LOWPAN
	//If we have a report time of zero, don't actually send...
	if(frameInterval)		//We flag this data as possibly being sent to a remote address too
		sixlowpan_sensorPerSend(sizeof(sftSensorReading), (u8*)&reading);
#else
	macDataRequest(DEFAULT_COORD_ADDR, sizeof(sftSensorReading), (u8*)&reading);
#endif

	sensorSentReading = true;


#endif
}

/**
   This node has successfully sent a packet, and this function is
   called when that happens.
*/
void
sensorPacketSendSucceed(void)	// This gets called from appPacketSendSucceed() when APP==1
{
	u16 interval;
   // Only handle this event if we are really waiting for a packet send result.
	if (!sensorSentReading)
		return;

	sensorSentReading = false;

	Led1_off();
    debugMsgStr("\r\n SensorPacketSendSucceeded");

#if IPV6LOWPAN //IPv6LOWPAN Handles this itself
    return;
#endif

    if (! (interval = halGetFrameInterval()))	// in case the sending of data was turned of by the coord by setting the interval to 0
    	 return;

// TODO: Not sure what this is for ?
    if (!macConfig.sleeping)
        return;

    // Sleep or wait for the interval, then send again
    if (RUMSLEEP && NODETYPE == ENDDEVICE)
    {
        // Wait for the frame to get out, then go to sleep
        // Also must wait to receive and process a packet from
        // parent.
        macSetAlarm(50 + 150*(BAND == BAND900), sensorSleep);
    }
    else
    {
        // no sleep, just wait for the interval time and send again
        if (interval <= 650)
            // Less than 65 seconds
            sendReadingTimer = macSetAlarm(interval * 100, SensorSendReading);
        else
            // One or more minutes
            sendReadingTimer = macSetLongAlarm((interval+5)/10, SensorSendReading);
    }
}

/**
   This node has failed to send a packet.
*/
void
sensorPacketSendFailed(void)
{
	Led1_off();
    debugMsgStr("\r\n  SensorPacketSendFailed");

#if IPV6LOWPAN //IPv6LOWPAN Handles this itself
    return;
#endif

    // Only handle this event if we are really waiting for a packet send result.
    if (!sensorSentReading)
        return;

    sensorSentReading = 0;

    // Wait a bit and try again.
//    macTimerKill(sendReadingTimer);
    sendReadingTimer = macSetAlarm(RETRY_WAIT_PERIOD, SensorSendReading);
}

/**
   This node has lost its network connection.
*/
void
sensorLostNetwork(void)
{
	Led1_off();
    sensorSentReading = 0;
    // Do nothing more.  The node will try to re-associate, and if
    // it does, it will start sending data again.


  // turn gas sensor heater off
    DDRD |= 0x80;
    PORTD &= 0x0F;  // turn all sensor related controls off -- heater, valves --
}


/**
    Send a sensor reading to coordinator and start timer to continuously send frames.
    This gets invoked by the controller to set the sensor node into reading mode. -- The mode persists through the EEprom
*/
#if (NODETYPE == ROUTER || NODETYPE == ENDDEVICE)
static void
sensorRequestReadings(sftRequestData *req)
{

    {
        // For very low power nodes, don't set interval to < 1sec
        if (VLP && req->time < 10)
        	req->time = 10;

        halSetFrameInterval(req->time);

        // Enable sleep mode so we will sleep between sending data frames.
        if (req->time)
        {
            macConfig.sleeping = true;
            debugMsgStr("\r\nSleeping interval = ");
            debugMsgInt(req->time);
            debugMsgCrLf();
            debugMsgStr("\r\n  Sensor Start Reading loop Requested by Coord  ");
            // And start the sending process.
            SensorStartReadings();
        }
        else
        	debugMsgStr("\r\n  Sensor Reading loop Stopped by Coord");

    }

}
#endif



/**
   Set the name of the sensor name.

   param:  name String that contains the new name.  The max length is
     NAME_LENGTH bytes.
*/
#	if (NODETYPE == ROUTER || NODETYPE == ENDDEVICE)
static void
sensorSetNodeNameRcv(char *name)
{
    // Sensor node, set the name
	strncpy(sensorInfo->name, name, NAME_LENGTH);
    halSaveSensorName(&sensorInfo->name);
}

static void
sensorSetArgument( short val0, short val1)
{
	halSaveSensorArgs(&val0,0);
	halSaveSensorArgs(&val1,1);
}
#endif


/**
   Handle a received packet.  The packet is a data packet
   addressed to this node.  This function applies to the coordinator
   and router/end nodes.

   param: 
   Pointer to the frame payload.  See   ftData.
*/
void
sensorRcvPacket(u8 *frame, u8 len )
{
#if   (NODETYPE == ROUTER || NODETYPE == ENDDEVICE)
		// Sensor frame type is always first byte of frame
debugMsgStr(" -- sensorRcvPacket");
		switch (((sftRequest*)frame)->type)
		{
			case REQ_READING_FRAME:
				// Frame containing a node's sensor reading request
				sensorRequestReadings((sftRequestData *)frame);
				break;

    		case SET_NODE_NAME:
				sensorSetNodeNameRcv(((sftSetName *)frame)->name);
				break;

    		case SET_ARG_FRAME:
    			sensorSetArgument(((sftSetArgs*)frame)->arg_1,((sftSetArgs*)frame)->arg_2);


    			break;

			default:
				break;
			}

#endif
}

//---------------------- Coordinator Functions from here on down -------------------

/**
   Request a reading from a sensor node

   param:  addr Short address of node being requested to send data

   param:  time Time to wait between readings, in 100mSec intervals. If
   time is zero, then only one reading will be sent.
*/
#if (NODETYPE==COORD )
// TODO: there should be a way to turn readings off again by the COORD

void
sensorRequestReading(u16 addr, u16 time)
{

	sftRequestData req;
	req.type = REQ_READING_FRAME;
	req.time = time;

debugMsgStr("\r\nSending Reading-Interval-Request to node:");
debugMsgInt(addr);
debugMsgStr(" to:");
debugMsgInt(time);
debugMsgStr("00 ms");

	if (IPV6LOWPAN)
		sixlowpan_sensorSend(addr, sizeof(sftRequestData), (u8*)&req);
	else
		macDataRequest(addr, sizeof(sftRequestData), (u8*)&req);
}


static void
CoordRcvSensorReading(sftSensorReading *reading)
{
	char str[10];

	if (SERIAL)
	{
		sprintf(str,"Sensor %d :", reading->SensorUnitID);
		serial_puts(str);

		sprintf(str,"%1.2f", (double) reading->readings[0]);

		switch (reading->SensorType)
		{
			case TMP100:
				serial_puts(" TMP100=");
				serial_puts((char *)str);
				serial_puts(" DegC");
				break;
			case Baro:
			case BMP085:
				serial_puts(" TempBaro=");
				serial_puts((char *)str);
				serial_puts(" DegC,");
				serial_puts(" Baro=");
				sprintf(str,"%1.2f", (double) reading->readings[1]);
				serial_puts((char *)str);
				serial_puts(" hPa");
				break;
			case Hyg:
				serial_puts(" TempHyg=");
				serial_puts((char *)str);
				serial_puts(" DegC,");
				serial_puts(" RH=");
				sprintf(str,"%1.2f", (double) reading->readings[1]);
				serial_puts((char *)str);
				serial_puts(" %");
				break;
			case V_bat:
				serial_puts(" V_bat=");
				serial_puts((char *)str);
				serial_puts(" Volt");
				break;
			case V_Hc:
				serial_puts(" GasHC=");
				serial_puts((char *)str);
				serial_puts(" Volt");
				break;
			case RSSI:
				serial_puts(" remRSSI=");
				serial_puts((char *)str);
				break;
			default:
				serial_puts(" UNK");
				serial_puts((char *)str);
				break;
		}


  	    debugMsgStr(", locRSSI=");
		debugMsgInt(radioGetSavedRssiValue());
		debugMsgStr(", locLQI=");
		debugMsgInt(radioGetSavedLqiValue());
		serial_puts("\r\n");


	}


}

void
CoordRcvSensorPacket(u8 *frame, u8 len )
{
#if DEBUG==2
	debugMsgStr(" -- CoordRcvSensorPacket");
#endif
    // Sensor frame type is always first byte of frame

	switch (((sftRequest*)frame)->type)
	{
        case READING_FRAME:
            // Frame containing a node's sensor reading
            CoordRcvSensorReading((sftSensorReading *)frame);
            break;

        default:
            break;
	}
}



/**
   Set the name of the sensor name.

   param:  addr Short address of the node to be named.
   param:  name String that contains the new name.  The max length is
     NAME_LENGTH bytes.
*/
void
sensorSendSetNodeName(u16 addr, char *name)
{
	sftSetName frame;

	frame.type = SET_NODE_NAME;
	strncpy(frame.name, name, NAME_LENGTH);

        // Send frame
#if IPV6LOWPAN
        sixlowpan_sensorSend(addr, sizeof(sftSetName), (u8*)&frame);
#else
        macDataRequest(addr, sizeof(sftSetName), (u8*)&frame);
#endif
}

void
sensorSendArgument( u16 addr, short arg_1, short arg_2)
{
	sftSetArgs frame;

	frame.type = SET_ARG_FRAME;
	frame.arg_1 = arg_1;
	frame.arg_2 = arg_2;


        // Send frame
#if IPV6LOWPAN
        sixlowpan_sensorSend(addr, sizeof(sftSetName), (u8*)&frame);
#else
        macDataRequest(addr, sizeof(sftSetName), (u8*)&frame);
#endif
}

#endif

#else // APP != SENSOR


char *sensorGetName(void) {return NULL;}
void sensorSendSetNodeName(u16 addr, char *name) {}
void sensorSendArgument( u16 addr, u8 arg_c, short arg_v){}

#endif // APP == SENSOR




/** @} */

