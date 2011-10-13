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
  $Id: sensors.h,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

#ifndef SENSORS_H
#define SENSORS_H
#include "mac.h"

/// Support for changing all node's interval response time.
#define REPORT_ALL    2
// Support for pining all nodes.
#define PING_ALL      1
void allNodes(u8 func, u16 val);
void StartSensorBC_interval(void);
void StartSensorReadings(void);		// Init and start automatic sensor reading transmissions to COORD
void sensorRequestReading(u16 addr, u16 time);
u8 sensorCalBusy(void);

// Numbering must be consecutive from 1
#define SENSOR_NONE		 	0
#define SENSOR_NET       	1  // Network stats , RSSI
#define SENSOR_TMP100	 	2
#define SENSOR_BAT		 	3
#define SENSOR_HP03		 	4
#define SENSOR_SHT11		 	5
#define SENSOR_GAS_HC 	 	6
#define SENSOR_GAS_CO 	 	7
#define SENSOR_BMP085		8

// This define needs to be on the compile cmd line, definig one of the above sensors
#ifndef SENSOR_TYPE
#error "APP==SENSOR but SENSOR_TYPE not defined";
#endif

// Sensor type enum -- not to exceed width of Sensor type field in struct tAD_data & sftSensorReading.
// Match the compile time defines above
typedef enum {None = SENSOR_NONE,
			  RSSI = SENSOR_NET,
			  TMP100 = SENSOR_TMP100,
			  V_bat = SENSOR_BAT,
			  Baro = SENSOR_HP03,
			  Hyg = SENSOR_SHT11,
			  V_Hc = SENSOR_GAS_HC,
			  V_Co2 =SENSOR_GAS_CO,
			 BMP085 = SENSOR_BMP085
}
tSensor_data_type;
// Broadcast data frame type -- do not exceed bitfield lenght below
typedef enum {BC_NONE, BC_SENSOR_DATA,} tBC_DataType;

typedef struct
{
	tSensor_data_type 	SensorType :4 ;		// 16 different sensor types, i.e. temp, baro, angle, etc..
	unsigned short 		ADC_Value  :12;		// with 12 bits resolution each
}__attribute__((packed)) tAD_data;

typedef struct
{
	unsigned short 		Unused 	 :1;	// spare
	tBC_DataType 		DataType :3;	// for different data packages sent  via the BC interface -- for now only sensor AD data
	unsigned short SensorUnitID  :12;	// The unique umber of the broadcasting device -- Could be part of the IEEE address
									    // or possibly we could send the BC frame with a long address ??
// union the folowing should different data types be needed
	tAD_data Readings[10];			// the individual readings the sensor broadcasts
} __attribute__((packed)) tBC_Data;


#if (APP == SENSOR)

#define RETRY_WAIT_PERIOD 50   // Time (mS) to wait after a packet failed to try again.

/*
   data_structures Data Structures

   The structures listed above describe the frames sent between a
   sensor node and the coordinator.  The 'sft' prefix stands for
   "sensor frame type".
*/

/*
   Sensor application frame types
*/
#define SET_NODE_NAME          2   ///< Frame directing the node to change its name string
#define REQ_READING_FRAME      3   ///< Frame requesting that a node begin sending sensor readings
#define READING_FRAME          4   ///< Frame containing a node's sensor reading
#define SET_ARG_FRAME		   5 // send arguments to the sensor node from the coordinator

/** @} */

/// Length of node name string
#define NAME_LENGTH 8
/** Sensor reading packet, sent by sensor node to report its data to
    the coordinator.
*/


typedef struct{
    u8    				Frametype;    		// Frame type, see   READING_FRAME -- NOTE: this is a waste of space when only 8 frame types are known
    tSensor_data_type 	SensorType :4 ;		// the kind of the reading
    unsigned short    	SensorUnitID :12; 	// part of the MAC address for absolute sensor identification
    float 				readings[4];		// A sensor can have 4 independent results
} __attribute__((packed)) sftSensorReading;

/** Sensor calibration request packet, sent by coordinator to ask
    sensor node to reply with its cal info
*/
typedef struct{
    u8    type;          ///< Frame type,
} __attribute__((packed)) sftRequest;

/** Sensor data request packet, sent by coordinator to ask
    sensor node to reply with its computed data
*/
typedef struct{
    u8    type;          ///< Frame type,
    u16   time;          ///< Interval between data frames, in 100mSec intervals.
} __attribute__((packed)) sftRequestData;


/** Change name of node packet, sent to sensor node */
typedef struct{
    u8    type;          ///< Frame type, see   SET_NODE_NAME
    char  name[NAME_LENGTH]; ///< Name of sensor
} __attribute__((packed)) sftSetName;

/** Change argument values in sensor node */
typedef struct{
    u8 type  ;         // Frame type,
    short arg_1;	    // argument index 0-3
    short arg_2; 		// Argument value
} __attribute__((packed)) sftSetArgs;

extern u16 node_addr;

void Sensor_HW_Init( void);
void sensorRcvPacket(u8 *payload, u8 len);
void sensorSendSetNodeName(u16 addr, char *name);
char *sensorGetName(void);
void sensorSendArgument( u16 addr, short arg_1, short arg_2);
void sensorPacketSendSucceed(void);
void sensorPacketSendFailed(void);
void sensorLostNetwork(void);
void sixlowpan_sensorPerSend(u8 len, u8 * data);
void sixlowpan_sensorSend(u16 addr, u8 len, u8 * data);
void sixlowpan_sensorReturn(u8 len, u8 * data);
void SensorStartReadings(void);
void SensorSendReading(void);

#if NODETYPE == COORD
	extern char node_name[NAME_LENGTH];
#endif

#endif	// APP==SENSOR
/** @} */

#endif
