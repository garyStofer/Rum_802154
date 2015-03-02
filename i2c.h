/*
 * i2c.h
 *
 *  Created on: Aug 16, 2010
 *      Author: Gary
 */

#ifndef I2C_H_
#define I2C_H_

typedef enum { Start=0,
			   ReStart,
			   AddrTX,
			   AddrRX,
			   TX,
			   RX_ACK,
			   RX_NACK,
			   Stop,
			   Reset
             } I2C_mode ; // I2c operation modes for Master transmit and Master receive modes

extern bool i2cBusErr;

extern unsigned char I2C_xfer( I2C_mode mode, unsigned char data );
extern void i2c_init( unsigned long f_SCK );


#endif /* I2C_H_ */
