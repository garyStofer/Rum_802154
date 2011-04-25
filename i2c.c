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
#include <math.h>
#include <compat/twi.h>
#include "system.h"
#include "EE_prom.h"
#include "i2c.h"

static bool Tmp100_Present = false;
static bool BMP085_Present = false;
static bool HP03_Present = false;
static bool i2cBusErr = false;

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

#define BMP085_I2C_Addr 0xEE
#define BMP085_EEprom 0xAA
#define BMP085_Ctrl_Reg 0xF4
#define BMP085_ReadADC 0xF6
#define BMP085_ConvTemp 0x2E	// 16 bits, conversion time 4.5ms


//#define BMP085_ConvPress 0x34   // 16 bits 1 internal sample 4.5Ms
#define BMP085_ConvPress 0xF4   // 16 bits 8 internal samples 25.5ms

union {
	unsigned short dat[11];
	struct{
			short   AC1,AC2,AC3;
			unsigned short AC4,AC5,AC6;
			short  	B1,B2;
			short 	MB,MC,MD;
		} Coeff;
}BMP085_Cal;

_Bool
BMP085_init(void )
{
	unsigned char i;

#if (APP == SENSOR )
	if (i2cBusErr)
		return false;

	if ( I2C_xfer(Start,0) != TW_START)
    {
    	I2C_xfer(Reset,0);
    	i2cBusErr = true;
    }
    if (I2C_xfer(AddrTX, BMP085_I2C_Addr)  == TW_MT_SLA_ACK)
    {
    	BMP085_Present = true;
    	I2C_xfer(TX, BMP085_EEprom); 	// internal register address
    	I2C_xfer(ReStart,0);			// restart for read mode next
    	I2C_xfer(AddrRX,BMP085_I2C_Addr);
    	for (i=0; i<11; i++)
		{
			I2C_xfer(RX_ACK,0);
			BMP085_Cal.dat[i]=TWDR<<8;
			I2C_xfer(RX_ACK,0);
			BMP085_Cal.dat[i]+=TWDR;
		}
    	I2C_xfer(RX_NACK,0);
    }
    I2C_xfer(Stop,0);
    return BMP085_Present;
#else
    return  0;
#endif
}
static unsigned short BMP085_getTemp()
{
	unsigned short t;

	I2C_xfer(Start,0);
	I2C_xfer(AddrTX, BMP085_I2C_Addr);
	I2C_xfer(TX, BMP085_Ctrl_Reg); 	// internal register address
	I2C_xfer(TX, BMP085_ConvTemp); 	// internal register address
	I2C_xfer(Stop,0);
	delay_us(5000); 	// Temp conversion is taking 4.5ms
	I2C_xfer(Start,0);
	I2C_xfer(AddrTX, BMP085_I2C_Addr);
	I2C_xfer(TX, BMP085_ReadADC); 	// internal register address
	I2C_xfer(ReStart,0);			// restart for read mode next
	I2C_xfer(AddrRX,BMP085_I2C_Addr);
	I2C_xfer(RX_ACK,0);
	t = TWDR;
	t=t<<8;
	I2C_xfer(RX_NACK,0);
	t+=TWDR;

	I2C_xfer(Stop,0);
	return t;
}


// This call gets the 16Bit or regular Pressure data -- Xlbs is not read
static unsigned long BMP085_getPress()
{
	unsigned short t=0;
	I2C_xfer(Start,0);
	I2C_xfer(AddrTX, BMP085_I2C_Addr);
	I2C_xfer(TX, BMP085_Ctrl_Reg); 	// internal register address
	I2C_xfer(TX, BMP085_ConvPress); 	// internal register address
	I2C_xfer(Stop,0);
	delay_us(30000); 	// Pressure conversion is taking the most 25.5ms (at oss=3)
	I2C_xfer(Start,0);
	I2C_xfer(AddrTX, BMP085_I2C_Addr);
	I2C_xfer(TX, BMP085_ReadADC); 	// internal register address
	I2C_xfer(ReStart,0);			// restart for read mode next
	I2C_xfer(AddrRX,BMP085_I2C_Addr);
	I2C_xfer(RX_ACK,0);
	t = TWDR<<8;
	I2C_xfer(RX_NACK,0);
	t += TWDR;

	I2C_xfer(Stop,0);

	return t;
}

bool BMP085_Read(float *t,float *press)
{
#if (APP==SENSOR)

	long X1,X2,X3;
	long p;
	unsigned long B4,B7;
	long B3,B5,B6;
	long T;
	long Ut,Up;

	*t = *press =0.0;

	if (!BMP085_Present || i2cBusErr)
		return false;

	// Calc true temp
	Ut = BMP085_getTemp();

	X1 =(Ut - BMP085_Cal.Coeff.AC6) * BMP085_Cal.Coeff.AC5 / 32768L;
	X2 = BMP085_Cal.Coeff.MC * 2048L /(X1 + BMP085_Cal.Coeff.MD);
	B5 = X1+X2;
	T = (B5+8)/16;

	Up = BMP085_getPress();

	B6=B5-4000L;
	X1 = (B6*B6)>>12;
	X1 *= BMP085_Cal.Coeff.B2;
	X1 >>= 11;

	X2 = BMP085_Cal.Coeff.AC2*B6;
	X2 >>= 11;

	X3=X1+X2;

	// Changed from original to remove the oversampling stuff -- makes the reaings more jittery
	B3 = ((BMP085_Cal.Coeff.AC1 *4L + X3) +2) >> 2;
	X1 = (BMP085_Cal.Coeff.AC3* B6) >> 13;
	X2 = (BMP085_Cal.Coeff.B1 * ((B6*B6) >> 12) ) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (BMP085_Cal.Coeff.AC4 * (unsigned long) (X3 + 32768L)) >> 15;
	B7 = ((unsigned long)(Up - B3) * 50000L );

	if (B7 < (unsigned long)0x80000000) {
		p = (B7 << 1) / B4;
	}
	else {
		p = (B7 / B4) << 1;
	}

	X1 = p >> 8;
	X1 *= X1;
	X1 = (X1 * 3038L) >> 16;
	X2 = (p * -7357L) >> 16;
	p += ((X1 + X2 + 3791L) >> 4);	// p in Pa

	*press = p/100.0;
	*t = T/10.0;
	#endif
	return true;
}

static u8 tmp100_ctrl;
_Bool
TMP100_init(unsigned char resolution )
{
#if (APP == SENSOR )
	if (i2cBusErr)
		return false;

//	tmp100_ctrl = resolution|0x1;	// the ADC resolution plus shutdown mode for one shot operation
	tmp100_ctrl = resolution ;

	if ( I2C_xfer(Start,0) != TW_START)
    {
    	I2C_xfer(Reset,0);
    	i2cBusErr = true;
    }
    if (I2C_xfer(AddrTX, TMP100_I2C_Addr)  == TW_MT_SLA_ACK)
    {
    	Tmp100_Present = true;
        I2C_xfer(TX, TMP100_Ctrl_Reg); 	// internal register address
        I2C_xfer(TX, tmp100_ctrl);		// ADC resolution and Shutdown mode
    }

    I2C_xfer(Stop,0);
    return Tmp100_Present;
#else
    return  0;
#endif
}
float
TMP100_read( void )
{
#if (APP == SENSOR)

	volatile union {
		unsigned char byte[2];
		short val;
	} reading;


	float result;

	if (!Tmp100_Present)
		return 0.0;

	// read temp
	I2C_xfer(Start,0);
	I2C_xfer(AddrTX, TMP100_I2C_Addr );


	/*
	// to trigger the one shot temp conversion -- Doesn't seem to make a diff
	I2C_xfer(TX, TMP100_Ctrl_Reg); 		// internal register address
    I2C_xfer(TX, tmp100_ctrl |0x80);	// trigger OneShot ADC
    I2C_xfer(ReStart,0);	// restart for read mode next
    I2C_xfer(AddrTX, TMP100_I2C_Addr );
	*/
	I2C_xfer(TX, TMP100_Temp_Reg);
	I2C_xfer(ReStart,0);	// restart for read mode next
	I2C_xfer(AddrRX,TMP100_I2C_Addr);
	I2C_xfer(RX_ACK,0);
	reading.byte[1] = TWDR;
	I2C_xfer(RX_NACK,0);
	reading.byte[0] = TWDR;
	I2C_xfer(Stop,0);
	reading.val >>=4;
	// calc the result in deg C
	result = reading.val* 0.0625;

	return result;
#endif
}

#define HP03_EEPROM_ADDR 0xA0
#define HP03_ADC_ADDR 0xEE
static union {
	u16 vals[9];
	u8  bytes[18];
	struct
	{
		u16   C1,C2,C3,C4,C5,C6,C7;
		u8 A,B,C,D;
	} Coeff;

} HP03_Cal;

// TODO: possibly add ack checking and refine error condition
void HP03_init( )
{
#if (APP ==SENSOR )
	u8 i;
		// initialize the HP03 device by reading the calibration coefficients from it's 24c02 like eeprom memory


		// This enables the 32Khz oscialltor on TOSC1 and TOSC2 -- Used for clock on the HP03S Temp/baro device
		// OSC is otherwise not used unless RUMSLEEP is configured, then the 32Khz based timer is used for sleeping also
	    ASSR |= (1 << AS2);
	    DDRA = 0x80 ; 		// enable PA7 as output -- Contriols XCLR line on HP03S device
	    PORTA =0;			// XCLR Low


		if ( I2C_xfer(Start,0) != TW_START)
	    {
	    	I2C_xfer(Reset,0);
	    	i2cBusErr = true;
	    	return;
	    }
	    if (I2C_xfer(AddrTX, HP03_EEPROM_ADDR )  == TW_MT_SLA_ACK)
	    {
	    	HP03_Present = true;	// The HP03 acknowledged an address cycle on the EEPROM address
	        I2C_xfer(TX, 16); 	// Set word pointer to 16 for read of first coefficient, -- will auto increment
	        I2C_xfer(ReStart,0);	// switch to master read mode
	        I2C_xfer(AddrRX, HP03_EEPROM_ADDR); // readdress in read mode now
	        for (i = 0; i<7; i++)				// Read words
	        {
	        	I2C_xfer(RX_ACK,0);				// read cal coeffs MSB
	        	HP03_Cal.vals[i] = (TWDR<<8);
	        	I2C_xfer(RX_ACK,0);				// read cal coeffs LSB
	        	HP03_Cal.vals[i] += TWDR;
	        }

	        for (i = 14; i<18; i++)						// Read bytes
	     	{
	     	        	I2C_xfer(RX_ACK,0);				// read cal coeffs ABCD
	     	        	HP03_Cal.bytes[i] = TWDR;
	     	}

	        I2C_xfer(RX_NACK,0); // one more to stop properly
	    }
		I2C_xfer(Stop,0);		 // and finish by transition into stop state
#endif
}

bool HP03_Read(float * t , float *p)
{
#if (APP ==SENSOR  )
	u8 i;
	u16 Y;
	union
	{
		u8 bytes[2];
		u16 val;
	} D2, D1; // D2 = raw Temperatur data , D1 = Raw Pressure data

	float X, Z;
	float dUT;
	float temp;
	float Offs;
	float Sens;
	float Pressure;

	if (!HP03_Present || i2cBusErr)
		return false;

	PORTA |= 0x80;			// XCLR high

	// the temperature Reading
	if ( ( I2C_xfer(Start,0)) != TW_START)
	{
	   	I2C_xfer(Reset,0);
	   	i2cBusErr = true;
	   	return false;
	}

	 I2C_xfer(AddrTX, HP03_ADC_ADDR );
	 I2C_xfer(TX, 0xff);
	 I2C_xfer(TX, 0xE8);	// Temperatur reading address
	 I2C_xfer(Stop,0);
// TODO: This should be a MacAlarm kind of delay, so that the unit is notwound up tight waiting for the delay
	 delay_us(40000);		// delay 40 ms for the result to arrive
	 // get result now
	 I2C_xfer(Start,0);
	 I2C_xfer(AddrTX, HP03_ADC_ADDR );
	 I2C_xfer(TX, 0xFD);
	 I2C_xfer(ReStart,0);
	 I2C_xfer(AddrRX, HP03_ADC_ADDR );
	 I2C_xfer(RX_ACK,0);				// read temp data msb
	 D2.bytes[1] = TWDR;
	 I2C_xfer(RX_NACK,0);				// read temp data lsb
	 D2.bytes[0] = TWDR;
	 I2C_xfer(Stop,0);


	 // The Pressure reading
	 I2C_xfer(Start,0);
	 I2C_xfer(AddrTX, HP03_ADC_ADDR );
	 I2C_xfer(TX, 0xff);
	 I2C_xfer(TX, 0xF0);	// Pressure reading address
	 I2C_xfer(Stop,0);
	 delay_us(40000);		// delay 40 ms for the result to arrive
    // get result now
	 I2C_xfer(Start,0);
	 I2C_xfer(AddrTX, HP03_ADC_ADDR );
	 I2C_xfer(TX, 0xFD);
	 I2C_xfer(ReStart,0);
	 I2C_xfer(AddrRX, HP03_ADC_ADDR );
	 I2C_xfer(RX_ACK,0);				// read temp data msb
	 D1.bytes[1] = TWDR;
	 I2C_xfer(RX_NACK,0);				// read temp data lsb
	 D1.bytes[0] = TWDR;
	 I2C_xfer(Stop,0);


	 if ( D2.val < HP03_Cal.Coeff.C5 )
	 	 Z = HP03_Cal.Coeff.B;
	 else
		 Z= HP03_Cal.Coeff.A;

	X = (float)D2.val - HP03_Cal.Coeff.C5;

	//Y =  pow(2,HP03_Cal.Coeff.C);
    for (Y=1,i = 0; i<HP03_Cal.Coeff.C; i++)
		Y*=2;

	// dUT = X-(X*X/16384.0) * Z / Y;
	dUT = X-X*X*Z / ( 16384.0 * Y) ;

	//Y = pow(2,HP03_Cal.Coeff.D;
	 for (Y=1,i = 0; i<HP03_Cal.Coeff.D; i++)
		Y*=2;

	temp = (250.0 + (dUT * HP03_Cal.Coeff.C6/65536.0) - ( dUT /Y ) ) /10;
	*t = temp;

	Offs = (HP03_Cal.Coeff.C2 +  (HP03_Cal.Coeff.C4-1024.0)* dUT/16384.0)* 4;
	Sens = HP03_Cal.Coeff.C1 + (HP03_Cal.Coeff.C3 * dUT / 1024.0);
	X = (Sens * (D1.val-7168.0) / 16384.0) - Offs;
	Pressure = ((X /32.0) + HP03_Cal.Coeff.C7/10.0);	// in hPa
	*p = Pressure;
#endif
	return true;

}

#define SHT_TEMP_CMD 0x03
#define SHT_RH_CMD 0x05
#define SHT_RD_STAT_CMD 0x7
#define SHT_WR_STAT_CMD 0x6
#define SHT_RESET_CMD 0x1E
#define CLK_TOGGLE PINC=0x1
#define DTA_TOGGLE PINC=0x2

// Const for Temp conversion
#define SHT_d1 0.01		// for degC
#define SHT_d2 -39.65 		// at 3.3V for deg C

// Const for RH conversion
#define SHT_c1 -2.0468
#define SHT_c2 0.0367
#define SHT_c3 -1.5955E-6

// Const for RH temp compensation
#define SHT_t1 0.01
#define SHT_t2 0.00008

// The SHT11 device is not an I2C device,

// Reading the SHT1x device PullUp and Tristate port when "driving" a H on the data line
// This is needeed in order to prevent drive contention in the device
// Note: This Device does not conform to the I2C standard of communication and is most likely not compatible
// with other devices on the I2C bus concurrently.
// The code below it bit-banging the SDA and SLK lines
static short SHT1xRead_OC(unsigned char cmd)
{
	  unsigned char i;
	  unsigned char d;
	  short SO =0;
	  // The Transmission start sequence
	  DDRC = 0x1; 	// PC0 = clock output, PC1=Data Output=tri=high
	  PORTC= 0;
	  PORTC= 0;
	  PORTC = 0x1;   // clk=high
	  PORTC = 0x1;   // clk=high
	  DDRC  = 0x3;   // DTA drive low;
	  PORTC = 0x0;   // clk = low;
	  PORTC = 0x0;   // clk = low;
	  PORTC = 0x1;   // clk = high
	  PORTC = 0x1;   // clk = high
	  DDRC  = 0x1;	 // DTA tristate == pulled high;
	  PORTC = 0x1;   // clk = high
	  PORTC = 0x1;   // clk = high
	  PORTC = 0x1;   // clk = high
	  PORTC = 0x1;   // clk = high
	  PORTC = 0;	// Clock low
	  PORTC = 0;	// for extra time
	  DDRC = 0x3;   // DTA drive low;
	  PORTC = 0;	// for extra time

	  i = 7;
	  // Command write sequence
	  do
	  {
		d = (cmd >> i)& 0x1 ;   	// get current bit
		DDRC = (d ? 0x1 : 0x3);		// sets Tristate on dataline for data high via pullup
		PORTC = 0;					// Allow time for pullup
		PORTC = 0;					// "
		PORTC = 0x1;				// SCK high -- Clock data in
		PORTC = 0;					// SCK low
	  } while (i--);

	  // Read the ack pulse from the device to see if it's responding
	  DDRC = 0x1;		// make SDA input
	  PORTC = 0x1;		// SCK high, allow time for pullup to raise data line (for NAK)
	  PORTC = 0x1;		// "
	  PORTC = 0x1;		// "
	  d = PINC;
	  PORTC =0;

	  if (d &0x2)		// Device did not respond -- return without result
		  return 0;

	  delay_us(20000);  // wait for the measure to Start  Measurment conversion take 80ms for 12Bit humidity data and 320Ms for 14Temp data
						 // should not be a dead wait

// TODO this should be better guarded against device failure
	  while ( PINC & 0x2 )	// Wait for the measure to finish, signaled by "Low" on data line
		  ;
	  // Read the 16 Bit of data from the device for the commanded reading -- Provide ACK pulse to device agter first 8 bit
	  for(i = 16; i; )
	  {
		  i--;
		  PORTC=0x1; // toggle high
		  d = ((PINC & 0x2) >>1);	// extract bit
		  PORTC= 0; // toggle Low // This advances to the next bit on Data line

		  SO |=  d<<i;
		  if ( i == 8)
		  {
			  PORTC=0;	// generate the ACK
			  DDRC = 0x3; // make SDA Output -- drive low for ACK
			  PORTC =0x1;// ACK while data low
			  PORTC =0x0;// ACK while data low
			  DDRC = 0x1; // input again
		  }
	  }

	  // Terminate transfer by sending NAK
	  PORTC = 0;	// extra time to allow Pu to rasie dataline
	  PORTC = 0;	// extra time to allow Pu to rasie dataline
	  PORTC =0x1;	// ACK clock  -- Data is high == NAK
	  PORTC =0x0; 	// ACK clock done

return SO;
}
#define RH_THRES 70
#define RH_HYST 5


bool SHT1xRead(float *temp, float *Rh)
{
	short SOt, SOrh;
	float T, RH;
	volatile short thresh, hyst;


	SOt  = SHT1xRead_OC(SHT_TEMP_CMD);
	SOrh = SHT1xRead_OC(SHT_RH_CMD);

	T= SOt*SHT_d1 + SHT_d2;
	RH = SHT_c1 + SHT_c2*SOrh + SHT_c3*SOrh*SOrh;
	RH = (T-25) * (SHT_t1+SHT_t2*SOrh)+ RH;   // TEMP compensation
	*temp = T;
	*Rh = RH;

	halGetSensorArgs(&thresh,0);		// using arg0 from eeprom as threshold
	halGetSensorArgs(&hyst,1);

	if (thresh+hyst > 97)
	{
		thresh = 92;
		hyst=5;
	}

	if (RH < thresh)
		PORTD |= 0x10;					//turn Hydrostat relay on
	else if (RH >(thresh+hyst))
		PORTD &= ~0x10;					// turn Hydrostat relay off

	return 1;
}
void SHT1xInit( )
{
}

