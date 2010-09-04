/**
   Calibration data, stored in EEPROM.  To calculate measured data,
   the following formula is used:

   @code
   reading = m * ADC + b
   @endcode
*/
typedef struct{
    float gain;     ///< The slope of the reading versus ADC input
    float offset;     ///< The offset of the reading
} __attribute__((packed)) tCalFactors;

/**
   Locations in EEPROM of the stored data.  Note that this struct only
   exists in EEPROM, and is never created in RAM.  There are various
   macros that use this structure definition to locate data in EEPROM.

   @ingroup app
*/
typedef struct {
    u8 eepromMacAddress[8];      ///< The node's unique IEEE MAC address
    tCalFactors calFactors;      ///< The node's calibration data (see sensor.h)
    u16 SensorframeInterval;           ///< The number of tenth seconds between data readings (see sensor.h)
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
#define  halSaveCalFactors(p) halPutEeprom((u8*)offsetof(tEepromContents, calFactors), sizeof(tCalFactors),(u8*) p)
#define  halGetCalFactors(p)  halGetEeprom((u8*)offsetof(tEepromContents, calFactors), sizeof(tCalFactors),(u8*) p)


extern void halSetFrameInterval(u16 );
extern u16  halGetFrameInterval(void);
extern void checkEeprom(void);
