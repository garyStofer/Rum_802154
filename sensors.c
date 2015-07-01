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
#include "Sensors/measure.h"

/**
     sleeping Sleeping and waking

   The sensor application causes nodes to sleep and wake.  The idea is
   that an end node can sleep for some interval while it is waiting to
   send a reading.  When it is time to send a reading, the unit wakes
   up, captures a reading, transmits the reading to its parent, and
   then goes back to sleep.

   It is necessary to provide a way to wake up the node, so that we
   can change the data interval or to calibrate the node.

   The mechanism for doing this is to use the   ftWake frame as a
   signal to wake up.  When an end node is done transmitting its data
   frame, it remains awake for a short period (about 50mSec) to be
   able to receive a wake-up frame from its parent.  If no frame is
   received in that time, a timer expires and the node goes back to
   sleep.  If a ftWake frame is received, then the node stays awake
   and stop sending frames.  The node can now be calibrated or a new
   data interval can be set by sending the node an   sftRequestData
   frame.

     sensor calls Sensor application function calls

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



#if ( NODE_TYPE == BC_ENDDEVICE)
static void
Sensor_BCSendReading( void)
{
	static tBC_Data s_data;

	u16 i;

	s_data.DataType = BC_SENSOR_DATA;

	//Using 12 lower bits from IEEE MAC address for unit ID
	halGetEeprom((u8*)(offsetof(tEepromContents, MacAddress))+6 ,2,(u8*)&i);		// Getting the two LSB's from the IEEE MAC addr
	s_data.SensorUnitID = i;
	s_data.Readings[0].SensorType = Temp;
	s_data.Readings[1].SensorType = None; //  indicates that there are no more readings

	HAL_SELECT_ADC0();
	HAL_START_ADC();
	HAL_WAIT_ADC();

	s_data.Readings[0].ADC_Value = HAL_READ_ADC() ;

	macData_BC_Request( sizeof(s_data),(u8 *) &s_data);

	// restart the timer for the next transmission
	i = EE_GetFrameInterval();

	if (i <655 )
	  macSetAlarm( i * 100, Sensor_BCSendReading);
	else
	  macSetLongAlarm((i + 5)/10, Sensor_BCSendReading);
}

void StartSensorBC_interval(void)
{

	HAL_ADC_INIT();

	macSetAlarm(2000, Sensor_BCSendReading);// start loop with initial delay simply to get past the initial association sequence

}
#endif

#if (NODE_TYPE == ROUTER || NODE_TYPE == ENDDEVICE)

static u8 SentReading_pendingAck; // flag - have we sent a reading and are waiting for a response?
static u8 sendReadingTimer;
struct{
		char name[8];  // Node name
	  } __attribute__((packed)) sensorInfo[1];


/**
   Start the sensor-sending-data loop depending whether or not the node has been previously
   configured to send data repeatedly.

   This gets called upon completion of Association for Sensor nodes
*/
void
SensorStartReadings(void)
{
	// This will start a repeated Sensor reading to be sent to the COORD node
	// based on the Sensor frame interval stored in the EEPROM
	if (EE_GetFrameInterval())
	{
		 sendReadingTimer = macSetAlarm(10, SensorSendReading);// Start the first reading in 1 second
	}

}


/**
   Returns a pointer to the name of this node.

   returns:  a pointer to the name of this node.
*/

char *sensorGetName(void)
{
	halGetSensorName(&sensorInfo->name);
    return sensorInfo->name;
}


/** @} */
/**
   @name Router/End node sensor functions
   @{
*/
// Only end nodes can sleep
#if( NODE_SLEEP && (NODE_TYPE == ENDDEVICE) )
 void
sensorSleep(void)
{
	// go to sleep, then wake-up and send again

	u16 sleepTime = EE_GetFrameInterval();  // Just to be sure

	if (sleepTime <10)						// Don't sleep less than 1 second
		sleepTime = 10;

	nodeSleep(sleepTime);
	SensorSendReading();


#endif
}

/**
   Send a single reading to the coordinator and go to sleep if
   required.
   This gets called from the mac Alarm function
*/
void
SensorSendReading(void)
{
	u16 i;
	sftSensorReading reading;

	if (! macConfig.associated)		// if not associated we don't know how to send
    	return;

	Led1_on();	// this gets cleared by: sensorPacketSendSucceed(),sensorPacketSendFailed() and sensorLostNetwork()

	//Using 12 lower bits from IEEE MAC address for unit ID
	// Getting the two LSB's from the IEEE MAC addr
	halGetEeprom((u8*)(offsetof(tEepromContents, MacAddress))+6 ,2,(u8*)&i);
	reading.SensorUnitID = i;
	reading.Frametype = READING_FRAME;

	Do_SensorMeasurment( &reading);

	debugMsgStr("\r\nSending SensorPacket");

	// Send it off

	macDataRequest(DEFAULT_COORD_ADDR, sizeof(sftSensorReading), (u8*)&reading);


	SentReading_pendingAck = true;


}

/**
   This node has successfully sent a packet, and this function is
   called when that happens.
*/
void
sensorPacketSendSucceed(void)	// This gets called from appPacketSendSucceed()
{
	u16 interval;
   // Only handle this event if we are really waiting for a packet send result.
	if (!SentReading_pendingAck)
		return;

	SentReading_pendingAck = false;

	Led1_off();
    debugMsgStr("\r\n SensorPacket successfully sent ");


    if (! (interval = EE_GetFrameInterval()))	// in case the sending of data was turned of by the coord by setting the interval to 0
    {
    	debugMsgStr("\r\n Sensor Readings turned off ");
    	return;
    }

    // Sleep or wait for the interval, then send again
    if (macConfig.sleeping)
    {
        // Wait for the frame to get out, then go to sleep
        // Also must wait to receive and process a packet from
        // parent.
        macSetAlarm(100 + 150*(BAND == BAND900), sensorSleep);
        // Stay awake for 250ms then call sensorSleep which eventually calls
        // SensorSendReading again to continue the reading loop
    }
	else	// no sleeping, just wait for the interval time and send again
    {
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


    // Only handle this event if we are really waiting for a packet send result.
    if (!SentReading_pendingAck)
        return;

    SentReading_pendingAck = 0;

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
    SentReading_pendingAck = 0;
    // Do nothing more.  The node will try to re-associate, and if
    // it does, it will start sending data again.
}


/**
    Send a sensor reading to coordinator and start timer to continuously send frames.
    This gets invoked by the controller to set the sensor node into reading mode. -- The mode persists through the EEprom

    Sending a 0 (re)enables the nodes sleeping mode.  Sleeping is initially enabled but can be turned off with a WAKE_NODE command from
    the COORD/ROUTER or with a button press >7.5 Sec while the node is sleeping
*/

 void
sensorSetReadingInterval(sftRequestData *req)
{
	if (req->time != 0)
	{
		// For Sleeping nodes don't repeat faster than 1 second
		if (NODE_SLEEP && req->time < 10)
			req->time = 10;

		EE_SetFrameInterval(req->time);

		// need to terminate the current reading interval, so that the new time takes effect immediately
		SensorSendReading();

	}
	else	//A zero in the reading interval puts a node into the sleeping mode
	{
		if (NODE_SLEEP)
			macConfig.sleeping = true;
	}
}

/**
   Set the name of the sensor name.

   param:  name String that contains the new name.  The max length is
     NAME_LENGTH bytes.
*/

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

		// Sensor frame type is always first byte of frame
debugMsgStr(" -- sensorRcvPacket");
		switch (((sftRequest*)frame)->type)
		{
			case REQ_READING_FRAME:
				// Frame containing a node's sensor reading request
				sensorSetReadingInterval((sftRequestData *)frame);
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
}

#endif
//---------------------- Coordinator Functions from here on down -------------------
//void
//
//{}
/**
   Request a reading from a sensor node

   param:  addr Short address of node being requested to send data

   param:  time Time to wait between readings, in 100mSec intervals. If
   time is zero, then only one reading will be sent.
*/
#if (NODE_TYPE==COORD )

	u16 node_addr;
	char node_name[NAME_LENGTH];

void
Sensor_HW_Init(void)
{}

void
sensorRequestReading(u16 addr, u16 time)
{

	sftRequestData req;
	req.type = REQ_READING_FRAME;
	req.time = time;

debugMsgStr("\r\nSending Reading-Interval-Request to node:");
debugMsgInt(addr);
debugMsgStr(" to:");
debugMsgInt(time/10);
debugMsgStr(" Sec.\r\n");


	macDataRequest(addr, sizeof(sftRequestData), (u8*)&req);
}


static void
CoordRcvSensorReading(sftSensorReading *reading)
{
	char str[10];

#if (SERIAL)
	if (!(PIND & (1 << PD5)))	// Human interface
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
			case SoilM:
				serial_puts(" V_bat=");
				serial_puts((char *)str);
				serial_puts(" V,");
				serial_puts(" VWC=");
				sprintf(str,"%1.2f %%", (double) reading->readings[1]);
				serial_puts((char *)str);
				serial_puts(" Vx=");
				sprintf(str,"%1.2f V", (double) reading->readings[2]);
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
	else // machine interface
	{
		sprintf(str,"SENS:%d", reading->SensorUnitID);	serial_puts(str);
		sprintf(str,":%d", reading->SensorType);	serial_puts(str);
		sprintf(str,":%1.2f", (double) reading->readings[0]);serial_puts(str);
		sprintf(str,":%1.2f", (double) reading->readings[1]);serial_puts(str);
		sprintf(str,":%1.2f", (double) reading->readings[2]);serial_puts(str);
		sprintf(str,":%1.2f", (double) reading->readings[3]);serial_puts(str);
		sprintf(str,":RSSI:%d\r\n", radioGetSavedRssiValue());serial_puts(str);
	}
#endif
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
sensorSendNodeName(u16 addr, char *name)
{
	sftSetName frame;

	frame.type = SET_NODE_NAME;
	strncpy(frame.name, name, NAME_LENGTH);

	// Send frame
	macDataRequest(addr, sizeof(sftSetName), (u8*)&frame);

}

void
sensorSendArgument( u16 addr, short arg_1, short arg_2)
{
	sftSetArgs frame;

	frame.type = SET_ARG_FRAME;
	frame.arg_1 = arg_1;
	frame.arg_2 = arg_2;


    // Send frame

    macDataRequest(addr, sizeof(sftSetName), (u8*)&frame);

}

#endif







/** @} */

