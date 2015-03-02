#ifndef SENSORS_H
#define SENSORS_H
#include "mac.h"




void StartSensorBC_interval(void);
void StartSensorReadings(void);		// Init and start automatic sensor reading transmissions to COORD
void sensorRequestReading(u16 addr, u16 time);
u8 sensorCalBusy(void);

// Numbering must be consecutive from 1
#define SENSOR_NONE		 	0
#define SENSOR_RSSI       	1  // Network stats , RSSI
#define SENSOR_TMP100	 	2
#define SENSOR_BAT		 	3
#define SENSOR_SHT11		4
#define SENSOR_GAS_HC 	 	5
#define SENSOR_GAS_CO 	 	6
#define SENSOR_BMP085		7
#define SENSOR_SOILM		8


#ifndef SENSOR_TYPE
#define SENSOR_TYPE SENSOR_NONE
#endif

// Sensor type enum -- not to exceed width of Sensor type field in struct tAD_data & sftSensorReading.
// Match the compile time defines above
typedef enum {None = SENSOR_NONE,
			  RSSI = SENSOR_RSSI,
			  TMP100 = SENSOR_TMP100,
			  V_bat = SENSOR_BAT,
			  Hyg = SENSOR_SHT11,
			  V_Hc = SENSOR_GAS_HC,
			  V_Co2 =SENSOR_GAS_CO,
			 BMP085 = SENSOR_BMP085,
			 SoilM  = SENSOR_SOILM
}
tSensor_data_type;


// Broadcast data frame type -- do not exceed bit-field length below
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
	unsigned short SensorUnitID  :12;	// The unique number of the broadcasting device -- Could be part of the IEEE address
									    // or possibly we could send the BC frame with a long address ??
// union the following should different data types be needed
	tAD_data Readings[10];			// the individual readings the sensor broadcasts
} __attribute__((packed)) tBC_Data;



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
#define SET_NODE_NAME          2   //< Frame directing the node to change its name string
#define REQ_READING_FRAME      3   //< Frame requesting that a node begin sending sensor readings
#define READING_FRAME          4   //< Frame containing a node's sensor reading
#define SET_ARG_FRAME		   5   // send arguments to the sensor node from the coordinator

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
void sensorSendNodeName(u16 addr, char *name);
char *sensorGetName(void);
void sensorSendArgument( u16 addr, short arg_1, short arg_2);
void sensorPacketSendSucceed(void);
void sensorPacketSendFailed(void);
void sensorLostNetwork(void);
void SensorStartReadings(void);
void SensorSendReading(void);


#if NODE_TYPE == COORD
	void CoordRcvSensorPacket(u8 *frame, u8 len );
	extern char node_name[NAME_LENGTH];
#endif

#endif

