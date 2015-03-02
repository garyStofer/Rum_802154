/***
   Locations in EEPROM of the stored data.  Note that this struct only
   exists in EEPROM, and is never created in RAM.  There are various
   macros that use this structure definition to locate data in EEPROM.

   @ingroup app
*/

typedef struct {
	u16 PanId;					// My networks ID
    u8 MacAddress[8];     		// The node's unique 802.15.4 MAC address
    short args[4];      		// arguments set by coord for various sensor control functions
    u16 SensorframeInterval;    // The number of tenth seconds between data readings (see sensor.h)
    char SensorName[9];			// MAX Interval = 6553.5 seconds == ~1hr 50 minutes
} tEepromContents;

/**
   Macro to retrieve MAC address stored in EEPROM.
 */
#define  halGetMacAddr(p)  halGetEeprom((u8*) offsetof(tEepromContents, MacAddress),          \
                                       sizeof(typeof(((tEepromContents*)0)->MacAddress)),(u8*) p)
#define  halPutMacAddr(p)  halPutEeprom((u8*) offsetof(tEepromContents, MacAddress),          \
                                       sizeof(typeof(((tEepromContents*)0)->MacAddress)), (u8*) p)
#define  halSaveSensorArgs(p,i) halPutEeprom((u8*)offsetof(tEepromContents, args[i]), sizeof(short),(u8*) p)
#define  halGetSensorArgs(p,i)  halGetEeprom((u8*)offsetof(tEepromContents, args[i]), sizeof(short),(u8*) p)
#define  halSaveSensorName(p) halPutEeprom((u8*)offsetof(tEepromContents, SensorName), sizeof(typeof(((tEepromContents*)0)->SensorName)),(u8*) p)
#define  halGetSensorName(p)  halGetEeprom((u8*)offsetof(tEepromContents, SensorName), sizeof(typeof(((tEepromContents*)0)->SensorName)),(u8*) p)

extern u16 EE_GetPanID(void);
extern void EE_SetPanID( u16 panid );
extern void EE_SetFrameInterval(u16 );
extern u16  EE_GetFrameInterval(void);
extern void checkEEprom(void);
extern void Clear_EEprom();
