/***
   Locations in EEPROM of the stored data.  Note that this struct only
   exists in EEPROM, and is never created in RAM.  There are various
   macros that use this structure definition to locate data in EEPROM.

   @ingroup app
*/

typedef struct {
    u8 eepromMacAddress[8];     // The node's unique IEEE MAC address
    short args[4];      		// arguments set by coord for varieous sensor control functions
    u16 SensorframeInterval;    // The number of tenth seconds between data readings (see sensor.h)
    char SensorName[9];
} tEepromContents;

/**
   Macro to retrieve MAC address stored in EEPROM.
 */
#define  halGetMacAddr(p)  halGetEeprom(offsetof(tEepromContents, eepromMacAddress),          \
                                       sizeof(typeof(((tEepromContents*)0)->eepromMacAddress)), \
                                       p)
#define  halPutMacAddr(p)  halPutEeprom(offsetof(tEepromContents, eepromMacAddress),          \
                                       sizeof(typeof(((tEepromContents*)0)->eepromMacAddress)), \
                                       p)
#define  halSaveSensorArgs(p,i) halPutEeprom((u8*)offsetof(tEepromContents, args[i]), sizeof(short),(u8*) p)
#define  halGetSensorArgs(p,i)  halGetEeprom((u8*)offsetof(tEepromContents, args[i]), sizeof(short),(u8*) p)
#define  halSaveSensorName(p) halPutEeprom((u8*)offsetof(tEepromContents, SensorName), sizeof(typeof(((tEepromContents*)0)->SensorName)),(u8*) p)
#define  halGetSensorName(p)  halGetEeprom((u8*)offsetof(tEepromContents, SensorName), sizeof(typeof(((tEepromContents*)0)->SensorName)),(u8*) p)

extern void halSetFrameInterval(u16 );
extern u16  halGetFrameInterval(void);
extern void checkEeprom(void);
