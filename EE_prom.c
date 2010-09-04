/*
 * EE_prom.c
 *
 *  Created on: May 27, 2010
 *      Author: Gary
 */
#include <math.h>
#include "hal.h"
#include "radio.h"
#include "EE_prom.h"

/**
   Verifies that the EEPROM contains valid data for the stored MAC
   address.  If the EEPROM is unprogrammed, then a random MAC address
   is written into EEPROM.

   Similarly, the sensor interval time is set to 2 seconds if the
   EEPROM is unprogrammed.
 */

// This gets programmed into the EEPROM if AVRDUDE is configured so that it loads the .eep file from the build.
// Also make sure that the "preseve EEPROM" is not set in the fuse bits, otherwise the programming does not work
// because the EEPROM is beeing restored to the previous values
 u8 EEMEM eeContent[13]= {0xaa,0xFE,0xba,0x11,0x12,0x01,0x00,0,0,0,0,20,0}; // Not very useful for MAC addr since all devices would get the same address

//TODO: To programm individual MAC addresses per board the EEPROM needs to be programmed outside of the program download and the EEPROM preserve bit in
//      the fuse bits set, so that it stays permanently   This will have to be a sperate process and the code below should then only check for an BAD
// MAC address and halt the CPU with the red LED on for example.

static u16 SensorReadInterval=0;

u16
halGetFrameInterval(void)
{
	if (! SensorReadInterval )
	{
		 halGetEeprom((u8*)offsetof(tEepromContents, SensorframeInterval), sizeof(SensorReadInterval), (u8*) &SensorReadInterval);
	}
	return SensorReadInterval;
}

void
halSetFrameInterval( u16 period )
{
	halPutEeprom((u8*)offsetof(tEepromContents, SensorframeInterval), sizeof(SensorReadInterval),  (u8*)&SensorReadInterval);
}

void
checkEeprom(void)
{


        u8 buf[8];
        volatile u16 dataInterval; // aka frameInterval
        tCalFactors calFactors;
        u8 i,bad;
// #define SET_EE_PROM

// this just programms the MAC address to something specific -- Not very useful as all devices would endup with the same MAC addr this way
#ifdef SET_EE_PROM
        tEepromContents eeprom;
        eeprom.eepromMacAddress[0]= 0xca;
        eeprom.eepromMacAddress[1]= 0xfe;
        eeprom.eepromMacAddress[2]= 0xba;
        eeprom.eepromMacAddress[3]= 0xad;
        eeprom.eepromMacAddress[4]= 0x11;
        eeprom.eepromMacAddress[5]= 0x12;
        eeprom.eepromMacAddress[6]= 0x01;
        eeprom.eepromMacAddress[7]= 0x00;
        eeprom.calFactors.gain = 0.25;
        eeprom.calFactors.offset = 0;
        eeprom.SensorframeInterval = 20; // 2 sec
        halPutEeprom(offsetof(tEepromContents, eepromMacAddress), sizeof(eeprom), (u8*) &eeprom);
#endif

        halGetMacAddr(buf);

        for (bad=1,i=0; i<8; i++)
        {
            if (buf[i] != 0xff)
            {
                // Valid (non-fffffff) MAC addr
                bad = 0;
                break;
            }
        }

        // // create a random MAC address and store it
        if (bad)
        {
            radioSetTrxState(RX_ON);
            for (i=0;i<8;i++)
                buf[i] = radioRandom(8);
            halPutMacAddr(buf);
        }

        // Get sensor send interval from  EEPROM
        dataInterval = halGetFrameInterval();
        if (dataInterval == 0xffff)
        {
			dataInterval = 21; 						// set to 2 seconds
			halSetFrameInterval(dataInterval); 	// and make it so
        }


		halGetCalFactors(&calFactors);
        if (isnan(calFactors.offset)     || isnan(calFactors.gain))
        {
        	calFactors.gain = 1;
        	calFactors.offset = 0;
        	halSaveCalFactors(&calFactors);
        }
}



