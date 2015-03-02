/*
 * i2c.c
 *
 *  Created on: Aug 16, 2010
 *      Author: Gary
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <compat/twi.h>
#include "system.h"
#include "i2c.h"

bool i2cBusErr = false;

unsigned char
I2C_xfer( I2C_mode mode, unsigned char data )
{

	switch ( mode )
	{
		case Start:
		case ReStart:
			// Send Start Condition
			TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
			break;

		case AddrTX:
			TWDR  = data & ~TW_READ;
			TWCR = _BV(TWINT) | _BV(TWEN);
			break;

		case AddrRX:
			TWDR  = data | TW_READ;
			TWCR = _BV(TWINT) | _BV(TWEN);
			break;

		case TX:
			 TWDR = data;
			 TWCR = _BV(TWINT) | _BV(TWEN);
			break;

		case RX_ACK:
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA); // Master reciver mode with ACK
			break;

		case RX_NACK:
			TWCR = _BV(TWINT) | _BV(TWEN); 	   // Master Receiver mode, send NAK afer the last byte
			break;

		case Stop:
			 TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */
			 break;

		default:
		case Reset:
			TWCR =0;
			return 0x1;		// error -- wrong mode
	}
	if (mode != Stop )
	{
		while (!(TWCR & (1 << TWINT)))
		 ; /* wait for completion  */
	}
	 // Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
	 return (TWSR & 0xF8);

}

// I2C init just sets the bus clock speed, but does not actually enable the I2C interface, IO lines change

void
i2c_init( unsigned long f_SCK )
{
    // i2C   TWI clock init
    TWSR = 0;		// Prescaler bits off
    TWBR = (F_CPU / f_SCK - 16) / 2;
//  i.e.  TWBR = 32; == 100Kz at 8Mhz fcpu

    I2C_xfer(Reset,0);
}


