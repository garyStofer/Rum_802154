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

extern unsigned char I2C_xfer( I2C_mode mode, unsigned char data );

// defines for TMP_100 device
#define TMP100_I2C_Addr 0x90
#define TMP100_Temp_Reg 0x00
#define TMP100_Ctrl_Reg 0x01
#define TMP100_ThLow_Reg 0x02
#define TMP100_ThHigh_Reg 0x03
// this commands 10 Bit resolution,( 1/4 degrtee per bit),  contineous operation
#define TMP100_9_BitCONF    0x00
#define TMP100_10_BitCONF    0x20
#define TMP100_11_BitCONF    0x40
#define TMP100_12_BitCONF    0x60
extern void i2c_init( unsigned long f_SCK );
extern _Bool TMP100_init(unsigned char resolution );
extern float TMP100_read( void );
extern void HP03_init(void);
extern bool HP03_Read( float *t, float *p);
extern bool SHT1xRead(float *temp, float *Rh);
extern bool BMP085_init(void );
extern bool BMP085_Read(float * t , float *p);


#endif /* I2C_H_ */
