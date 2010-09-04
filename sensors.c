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

                                   Calibration

 sensorRequestCalInfo()    ---> CAL_REQ_INFO_FRAME --->        sensorReplyCalInfo()
 sensorRcvCalInfo()        <---   CAL_INFO_FRAME   <---
 (Save cal info)
 sensorSendCalPoint()      --->    CAL_CMD_FRAME   --->        sensorRcvCalCommand()
                                                                (calibrate sensor)
 sensorSendCalPoint()      --->    CAL_CMD_FRAME   --->        sensorRcvCalCommand()
  (if 2-point cal)                                              (calibrate sensor)


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
static u8 sensorType;
static u8 busyCal; // Is the cal process running?  This flag is used to suppress user menus during calibration.
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
		 sensorType = SENSOR_TYPE;
		 sendReadingTimer = macSetAlarm(10, SensorSendReading);// Start the first reading almost right away
	}

#endif
}

#if (NODETYPE == COORD)
	u16 node_addr;
	char node_name[NAME_LENGTH];
	// Cal info that coord has to save to do a cal
	sftCalInfo coordCalInfo[1];
#else
	struct{
			char name[8];  // Node name
		  } __attribute__((packed)) sensorInfo[1];

#if CAL
		static tCalData calData[1];
		static tCalFactors calFactors[1];
#	endif
#endif


/**
   Returns a pointer to the name of this node.

   returns:  a pointer to the name of this node.
*/
#if (NODETYPE != COORD)
char *sensorGetName(void)
{
    return sensorInfo->name;
}
#endif

/**
   Returns true if the node is engaged in a calibration process.  This
   value is used to suppress the main menu operation during the calibration process.

   returns:  true  if the node is engaged (busy) with a calibration
   returns:  false if the node is not busy
*/
u8 sensorCalBusy(void)
{
    return busyCal;
}



/** @} */
/**
   @name Router/End node sensor functions
   @{
*/

/**
   Sensor node sends raw data back to coordinator
*/
#if ((NODETYPE == ROUTER || NODETYPE == ENDDEVICE) && CAL)
static void
sensorReplyRawData(sftRequest *req)
{

    {
        // Send data back
        sftRawData rawData;
        rawData.type = RAW_DATA_FRAME;
        rawData.reading = 0x8000 + macConfig.dsn * 300;
        // send it
#if IPV6LOWPAN
        sixlowpan_sensorReturn(sizeof(sftRawData), (u8*)&rawData);
#else
        macDataRequest(DEFAULT_COORD_ADDR, sizeof(sftRawData), (u8*)&rawData);
#endif

    }

}
#endif
#if ((NODETYPE == ROUTER || NODETYPE == ENDDEVICE) && CAL)
static void
sensorReplyCalInfo(sftRequest *req)
{

    {
        // Return tCalInfo packet
        sftCalInfo calInfo = {
            .type = CAL_INFO_FRAME,
            .calType = CAL_POINTS,
            .addr = macConfig.shortAddress,
            .units = "degF"};

#if IPV6LOWPAN
        sixlowpan_sensorReturn(sizeof(sftCalInfo), (u8*)&calInfo);
#else
        macDataRequest(DEFAULT_COORD_ADDR, sizeof(sftCalInfo), (u8*)&calInfo);
#endif
    }

}
#endif
#if ((NODETYPE == ROUTER || NODETYPE == ENDDEVICE) && CAL)
static void
sensorRcvCalCommand(sftCalCommand *calCommand)
{

    {
        // Received a cal command, save away readings, calc when done
        u8 index = calCommand->index;

        // Begin A/D reading
        HAL_SAMPLE_ADC();

        // Save user-entered reading
        calData->reading[index] = atof((char*)calCommand->reading);

        // Finish A/D reading
        HAL_WAIT_ADC();
        calData->ad[index] = HAL_READ_ADC();


        // If we have one (single-point) or both readings, do the cal
        if (CAL_POINTS == 1)
        {
            // Wipe out cal before we get a current reading, so that
            // old cal doesn't get used by sensorGetReading()
            calFactors->offset = 0;
            // Single-point temperature cal, save the offset
//TODO: this whole thing with CAL is unclear -- Commented out at this point
//            calFactors->b = calData->reading[index] - sensorGetReading();
            // Save new cal factors to EEPROM
            halSaveCalFactors(calFactors);


        }
        if (index)
        {
            // Second of two points, calculate calibration parameters
            double m, b;
            m = calData->reading[0] - calData->reading[1];
            b = (double)calData->ad[0];
            b -= (double)calData->ad[1];
            m /= b;
            b = calData->reading[0];
            b -= (m * (double)calData->ad[0]);

            debugMsgStr("\r\nA1=");
            debugMsgHex(calData->ad[0]);
            debugMsgCrLf();
            debugMsgStr("A2=");
            debugMsgHex(calData->ad[1]);
            debugMsgCrLf();
            debugMsgStr("r1=");
            debugMsgFlt(calData->reading[0]);
            debugMsgCrLf();
            debugMsgStr("r2=");
            debugMsgFlt(calData->reading[1]);
            debugMsgCrLf();

            // Store data in cal factors struct
            calFactors->gain = m;
            calFactors->offset = b;
            // Save new cal factors to EEPROM
            halSaveCalFactors(calFactors);
        }
    }

}
#endif
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

	volatile float val ;
	float dummy;

	u16 i;
	sftSensorReading reading ;

	if (! macConfig.associated)		// if not associtaed we don't know how to send
    	return;

	reading.Frametype = READING_FRAME;
	//Using 12 lower bits from IEEE MAC address for unit ID
	 halGetEeprom((u8*)(offsetof(tEepromContents, eepromMacAddress))+6 ,2,(u8*)&i);		// Getting the two LSB's from the IEEE MAC addr
	 reading.SensorUnitID = i;


	if (SENSOR_TYPE == SENSOR_ALL )
	{
		if (! --sensorType)
			sensorType = SENSOR_ALL-1;
	}
	else
		sensorType = SENSOR_TYPE;

	switch (sensorType)
	{
		case SENSOR_NET: // RSSI
			reading.SensorType = RSSI;
			val = radioGetSavedRssiValue();
			break;

		case SENSOR_TEMP1:
			reading.SensorType = Temp1;
			val = TMP100_read();
			break;

		 case SENSOR_TEMP2:
			reading.SensorType = Temp2;
			HP03_Read(&val , &dummy);
			break;

		 case SENSOR_BARO:
			reading.SensorType = Baro;
			HP03_Read(&dummy , &val);
			break;

		case SENSOR_GAS:
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
			break;

		case SENSOR_BAT:
			reading.SensorType = V_bat;
			HAL_SELECT_ADC7();
			HAL_START_ADC();
			HAL_WAIT_ADC();
			val = (float) HAL_READ_ADC() ;
			val *= 2.5e-3;		// in volts -- using 2.56V internal ref and x1 gain
			val *=16;			// Voltage divider 16:1
			break;

		default:
			val = 0;
	}

	Led1_on();	// this gets cleared by: sensorPacketSendSucceed(),sensorPacketSendFailed() and sensorLostNetwork()

	debugMsgStr("\r\nSending SensorPacket");
	//char str[20];
	//sprintf((char *)str," %1.2f ",(double)val);
	//debugMsgStr_d((char *) str);


	reading.reading = val;
	// Send it off
#	if IPV6LOWPAN
	//If we have a report time of zero, don't actually send...
	if(frameInterval)		//We flag this data as possibly being sent to a remote address too
		sixlowpan_sensorPerSend(sizeof(sftSensorReading), (u8*)&reading);
#	else
	macDataRequest(DEFAULT_COORD_ADDR, sizeof(sftSensorReading), (u8*)&reading);
#	endif

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
    PORTD |= 0x00;
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
        }
        debugMsgStr("\r\n  Sensor Start Reading loop Requested by Coord  ");
        // And start the sending process.
        SensorStartReadings();
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

    {
        // Sensor node, set the name
        strncpy(sensorInfo->name, name, NAME_LENGTH);
    }

}
#endif


/**
   Handle a received packet.  The packet is a data packet
   addressed to this node.  This function applies to the coordinator
   and router/end nodes.

   param: 
   frame Pointer to the frame payload.  See   ftData.
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

			case CAL_REQ_INFO_FRAME:
				// Frame containing a node's calibration information
#if CAL
				sensorReplyCalInfo((sftRequest*)frame);
#endif
				break;

			case REQ_RAW_DATA_FRAME:
				// Request a frame containing a node's raw data (A/D) readings
#if CAL
				sensorReplyRawData((sftRequest*)frame);
#endif
				break;

			case CAL_CMD_FRAME:
				// Frame sent from coordinator commanding a node to perform a calibration
#if CAL
				sensorRcvCalCommand((sftCalCommand*)frame);
#endif
				break;

			case SET_NODE_NAME:
				sensorSetNodeNameRcv(((sftSetName *)frame)->name);
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

    {
        sftRequestData req;
        req.type = REQ_READING_FRAME;
        req.time = time;

        if (IPV6LOWPAN)
            sixlowpan_sensorSend(addr, sizeof(sftRequestData), (u8*)&req);
        else
            macDataRequest(addr, sizeof(sftRequestData), (u8*)&req);
    }

}


static void
CoordRcvSensorReading(sftSensorReading *reading)
{
	char str[10];

	if (SERIAL)
	{
		sprintf(str,"\r\nSensor %d :", reading->SensorUnitID);
		serial_puts(str);

		sprintf(str,"%1.2f", (double) reading->reading);

		switch (reading->SensorType)
		{
			case Temp1:
				serial_puts(" Temp1=");
				serial_puts((char *)str);
				serial_puts(" DegC");
				break;
			case Temp2:
				serial_puts(" Temp2=");
				serial_puts((char *)str);
				serial_puts(" DegC");
				break;
			case Baro:
				serial_puts(" Baro=");
				serial_puts((char *)str);
				serial_puts(" hPa");
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

	}


}
/**
   Callback function to receive calibration information from a sensor
   node.

   param:  calInfo Pointer to tCalInfo structure that contains the node's
   calibration information.
*/
static void sensorCalProcess(void);

static void
sensorRcvCalInfo(sftCalInfo *calInfo)
{

    {
        // Cal info received, perform a calibration.
        *coordCalInfo = *calInfo;
        if (DEBUG)
            // Start the callback function going
            if (INTERACTIVE)
            {
                sensorCalProcess();
            }
    }

}

static void
sensorRcvRawData(sftRawData *rawData)
{
    {
        // Do what you want with the raw data
    }
}
void
CoordRcvSensorPacket(u8 *frame, u8 len )
{

		debugMsgStr(" -- CoordRcvSensorPacket");
       // Sensor frame type is always first byte of frame
        switch (((sftRequest*)frame)->type)
        {
        case READING_FRAME:
            // Frame containing a node's sensor reading
            CoordRcvSensorReading((sftSensorReading *)frame);
            break;
        case CAL_INFO_FRAME:
            // Frame containing a node's calibration information
            sensorRcvCalInfo((sftCalInfo*)frame);

            break;
        case RAW_DATA_FRAME:
            // Frame containing a node's raw data (A/D) readings
            sensorRcvRawData((sftRawData*)frame);
            break;
        default:
            break;
        }


}


/**
   Request an end node to send its calibration information.  When the
   node responds with the info,   sensorRcvCalInfo() is called.

   param:  addr Short address of node
*/
void
sensorRequestCalInfo(u16 addr)
{

    {
        sftRequest req;

        req.type = CAL_REQ_INFO_FRAME;

        // Clear the cal info struct
        coordCalInfo->type = 0;

#if IPV6LOWPAN
        sixlowpan_sensorSend(addr, sizeof(sftRequestData), (u8*)&req);
#else
        macDataRequest(addr, sizeof(sftRequestData), (u8*)&req);
#endif
    }

}

void sensorRequestRawData(u16 addr)
{

    {
        sftRequest req;
        req.type = REQ_RAW_DATA_FRAME;

#if IPV6LOWPAN
        sixlowpan_sensorSend(addr, sizeof(sftRequestData), (u8*)&req);
#else
        macDataRequest(addr, sizeof(sftRequestData), (u8*)&req);
#endif
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

}


// Index is 0 for first point, 1 for second point
void
sensorSendCalPoint(u8 index, char *str)
{

    {
        sftCalCommand calCmd = {
            .type = CAL_CMD_FRAME,
            .index = index};
        strncpy((char*)calCmd.reading, str, 6);
        // Send packet to sensor node
#if IPV6LOWPAN
        sixlowpan_sensorSend(coordCalInfo->addr, sizeof(sftCalCommand), (u8*)&calCmd);
#else
        macDataRequest(coordCalInfo->addr, sizeof(sftCalCommand), (u8*)&calCmd);
#endif
       debugMsgStr("\r\nSent cal data to sensor");
    }

}

/**
   Coord node calibration routine. This must be called repetitively
   because the sensor node must be queried constantly to get current
   A/D readings.
*/
#if (DEBUG && SERIAL)
static void
sensorCalProcess(void)
{

        // Save state of where we're at
        static enum {start, read1, read2} progress;
        char str[12];

        switch (progress)
        {
        case start:
            // Print out intro, start getting A/D readings
            debugMsgStr("\r\nCal node, enter applied reading A [");
//            debugMsgStr((char *)coordCalInfo->units);
            debugMsgStr("]:");
            progress++;
            busyCal = true;
            break;
        case read1:
            // Don't block unless we have at least one serial char
            if (serial_ischar())
            {
                // Get reading, send to sensor node
                serial_gets(str,10,true);
                // Send frame to sensor node with cal data
                sensorSendCalPoint(0, str);

                progress++;

                // Output stuff for next reading
                if (coordCalInfo->calType == 2)
                {
                    debugMsgStr("\r\nCal node, enter applied reading B [");
//                    debugMsgStr((char *)coordCalInfo->units);
                    debugMsgStr("]:");
                }
            }
            break;
        case read2:
            // Get reading two, if there is one
            if (coordCalInfo->calType == 2)
            {
                // Need a second reading
                if (serial_ischar())
                {
                    // Get reading, send to sensor node
                    serial_gets(str,10,true);
                    progress++;  // Default will Start over

                    // Send frame to sensor node with cal data
                    sensorSendCalPoint(1, str);
                }
            }
            else
                // Don't bother with second reading
                progress++;
            break;
        default:
            // Quit, don't call this function again
            // End the cal process, reset progress for next time.
            busyCal = false;
            progress = start;
            // Return so that we don't re-call this function
            return;
        }
        // Thank you, come again!
        macSetAlarm(50, sensorCalProcess);

}
#endif




#endif

#else // APP != SENSOR


char *sensorGetName(void) {return NULL;}
void sensorSendSetNodeName(u16 addr, char *name) {}

#endif // APP == SENSOR




/** @} */

