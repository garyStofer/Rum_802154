/*
 * EE_prom.c
 *
 *  Created on: May 27, 2010
 *      Author: Gary
 */
#include "hal.h"
#include "radio.h"
#include "EE_prom.h"

/**
   Verifies that the EEPROM contains valid data for the stored MAC
   address.  If the EEPROM is unprogrammed, then a random MAC address
   is written into EEPROM.

   Similarly, the sensor interval time is set to 2 seconds if the
   EEPROM is un-programmed.
 */

// This gets programmed into the EEPROM if AVRDUDE is configured so that it loads the .eep file from the build.
// Also make sure that the "preserve EEPROM" is not set in the fuse bits, otherwise the programming does not work
// because the EEPROM is being restored to the previous values
// u8 EEMEM eeContent[15]= {0xff,0xff,0xaa,0xFE,0xba,0x11,0x12,0x01,0x00,0,0,0,0,20,0}; // Not very useful for MAC addr since all devices would get the same address

//TODO: To program discrete MAC addresses per board the EEPROM needs to be programmed outside of the program download and the EEPROM preserve bit in
//      the fuse bits set, so that it stays permanently
// This will have to be a operate process and the code below should then only check for an BAD
// MAC address and halt the CPU with the red LED on for example.

static u16 SensorReadInterval=0;

u16
EE_GetPanID(void)
{
	volatile u16 tmp; // =0x12;

	halGetEeprom((u8*)offsetof(tEepromContents, PanId), sizeof(tmp), (u8*) &tmp);

	return tmp;
}

void
EE_SetPanID( u16 panid )
{
	halPutEeprom((u8*)offsetof(tEepromContents, PanId), sizeof(panid),  (u8*)&panid);
}

u16
EE_GetFrameInterval(void)
{
	if (! SensorReadInterval )
	{
		 halGetEeprom((u8*)offsetof(tEepromContents, SensorframeInterval), sizeof(SensorReadInterval), (u8*) &SensorReadInterval);
	}

	return SensorReadInterval;

}

void
EE_SetFrameInterval( u16 period )
{
	SensorReadInterval = period;
	halPutEeprom((u8*)offsetof(tEepromContents, SensorframeInterval), sizeof(SensorReadInterval),  (u8*)&SensorReadInterval);
}

// Resets the eeprom contents so that the device creates a random MAC address
// and has sane startup values
// can be called from main during startup with a button pressed for example
void
Clear_EEprom()
{

        tEepromContents eeprom;

        eeprom.PanId = 0xffff;				// Any PAN in reach will connect
        eeprom.MacAddress[0]= 0xff;
        eeprom.MacAddress[1]= 0xff;
        eeprom.MacAddress[2]= 0xff;
        eeprom.MacAddress[3]= 0xff;
        eeprom.MacAddress[4]= 0xff;
        eeprom.MacAddress[5]= 0xff;
        eeprom.MacAddress[6]= 0xff;
        eeprom.MacAddress[7]= 0xff;
        eeprom.args[0] = 0;
        eeprom.args[1] = 0;
        eeprom.args[2] = 0;
        eeprom.args[3] = 0;
        eeprom.SensorframeInterval = 5; // 0.5 seconds
        halPutEeprom(0, sizeof(eeprom), (u8*) &eeprom);
}
// to force an EEPROM clear every time the thing boots
// #define CLEAR_EE_PROM 1
void
checkEEprom(void)
{
        u8 buf[8];
        u8 i,bad;

#ifdef CLEAR_EE_PROM
        Clear_EEprom();
#endif

        halGetMacAddr( buf);


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

        // check sensor send interval from  EEPROM not being ffff
        if (EE_GetFrameInterval() == 0xffff)
        {
			EE_SetFrameInterval(5); 	// and make it 0.5 seconds
        }

}



