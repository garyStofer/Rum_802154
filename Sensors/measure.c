/*
 * measure.c
 *
 *  Created on: Feb 4, 2015
 *      Author: gary
 */
#include "../Sensors/measure.h"

#include <compat/twi.h>
#include <math.h>

#include "../i2c.h"
#include "../EE_prom.h"
#include "../radio.h"


#if (SENSOR_TYPE == SENSOR_SOILM)
// reading[0] ->: Battery voltage of Sensor in Volts
// reading[1] ->: Vegetronics probe on ADC6 and linearizes the reading for Volumetric Water Content
// reading[2] ->: Soil temperature from thermister on ADC5 in degree C
// reading[3] ->: unused

static void
Measure_SoilM( sftSensorReading *reading)
{
	volatile float val,Vx,VWC;



	//Read the battery voltage
	HAL_SELECT_ADC7();

	HAL_START_ADC();
	HAL_WAIT_ADC();
	val = (float) HAL_READ_ADC() ;
	val *=((26.1 + 4.7)/4.7);			// input Voltage divider
	val *= 2.5e-3;		// in volts -- using 2.56V internal ref and x1 gain
	reading->readings[0]= val;

	// Read the Vegetronix soil moisture probe and compensate reading according
	// to their correction lookup table -- Probe voltage 0 = 3V, Ri 10K

	// Wait 500ms for the Sensor to come alive
	PORTD |=  (1<<PD5);	// Turn Vegetronix power on
	Led1_off();			// turn led off during waiting for the probe to power up to conserve power
	delay_us(500000);  	// Vegetronix takes 400ms to power up.
	Led1_on(); 			// Turn it back on to indicate transmission of readings

	reading->SensorType = SoilM;
	HAL_SELECT_ADC6();

	HAL_START_ADC();
	HAL_WAIT_ADC();

	PORTD &= ~(1<<PD5);			// Turn Vegetronix power off

	val = (float) HAL_READ_ADC() ;
	val *= 2.5e-3;		// in volts -- using 2.56V internal ref and x1 gain
	Vx =val *= ((36.5 + 10.0)/36.5); 		// Voltage divider 10K Output impedance and 36.5K ADC input impedance

	/* for Vegetronix VH400 probe
	 Voltage Range 	Equation
	0 to 1.1V 	VWC= 10*V-1
	1.1V to 1.3V 	VWC= 25*V- 17.5
	1.3V  to 1.82V 	VWC= 48.08*V- 47.5
	1.82V to 2.2V 	VWC= 26.32*V- 7.89
	!!! NOTE that the table doesn't span the entire 0-3V range the probe is outputting
	    Using the last segment of the curve for voltages over 50% gives you a maximum
	    VWC reading of 72.24% even though  the probe is fully immersed in water
	*/

	if ( Vx < 1.1)
		VWC = (Vx*10.0) -1;
	else if ( Vx < 1.3)
		VWC = (Vx*25.0) -17.5;
	else if ( Vx < 1.82 )
		VWC = (Vx*48.08) - 47.5;
	else if ( Vx < 2.2)
		VWC= (Vx*26.32 )- 7.89;
	else VWC = (Vx* (50/0.8)) - 87.5; // added a last section from 50 to 100% linear

	if (VWC > 100.0)
		VWC = 100.0;
	else if ( VWC <0 )
		VWC = 0;

	reading->readings[2]= Vx;
	reading->readings[1]= VWC;



	//Read the Thermister voltage
	/*
	HAL_SELECT_ADC5();
	HAL_START_ADC();
	HAL_WAIT_ADC();
	val = (float) HAL_READ_ADC() ;
	val *=((26.1+4.7)/4.7);			// input Voltage divider
	val *= 2.5e-3;		// in volts -- using 2.56V internal ref and x1 gain

	reading->readings[2]= val;*/

}
#endif

#if ( SENSOR_TYPE == SENSOR_BAT)
static void
Measure_Vbat_highV(  sftSensorReading *reading)
{
	float val;
	reading->SensorType = V_bat;
	HAL_SELECT_ADC7();
	HAL_START_ADC();
	HAL_WAIT_ADC();
	val = (float) HAL_READ_ADC() ;
	val *= 2.56e-3;		// in volts -- using 2.56V internal ref and x1 gain
	val *=16;			// Voltage divider 16:1
	reading->readings[0]= val;
}
#endif

#if ( SENSOR_TYPE ==  SENSOR_GAS_HC)
static void
Measure_HC(  sftSensorReading *reading)
{
	float val;
	// Turn the heater of the gas sensor on -- Only needed the first time around, but delayed execution
	// in here to leave the heater off unless the device is associated
	DDRD |= 0x80;
	PORTD |= 0x80;
	reading->SensorType = V_Hc;
	HAL_SELECT_ADC6();
	HAL_START_ADC();
	HAL_WAIT_ADC();
	val = (float) HAL_READ_ADC() ;
	val *= 2.5e-3;		// in volts -- using 2.56V internal ref and x1 gain
	val*=2; 			// Voltage divider 2:1
	reading->readings[0]= val;
}
#endif

#if (SENSOR_TYPE==SENSOR_TMP100)

// defines for TMP_100 device
#define TMP100_I2C_Addr 0x90
#define TMP100_Temp_Reg 0x00
#define TMP100_Ctrl_Reg 0x01
#define TMP100_ThLow_Reg 0x02
#define TMP100_ThHigh_Reg 0x03
// this commands 10 Bit resolution,( 1/4 degree per bit),  continuous operation
#define TMP100_9_BitCONF    0x00
#define TMP100_10_BitCONF    0x20
#define TMP100_11_BitCONF    0x40
#define TMP100_12_BitCONF    0x60

static bool Tmp100_Present = false;
static u8 tmp100_ctrl;

static _Bool
TMP100_init(void  )
{

	if (i2cBusErr)
		return false;

//	tmp100_ctrl = resolution|0x1;	// the ADC resolution plus shutdown mode for one shot operation
	tmp100_ctrl = TMP100_12_BitCONF ;

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

}
static float
TMP100_read( void )
{


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

}


static void
Measure_TMP100(  sftSensorReading *reading)
{
	reading->SensorType = TMP100;
	reading->readings[0] = TMP100_read();
}
#endif

#if ( SENSOR_TYPE== SENSOR_SHT11)

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

// Reading the SHT1x device: PullUp and Tristate port when "driving" a H on the data line
// This is needeed in order to prevent drive contention in the device
// Note: This Device does not conform to the I2C standard of communication and is most likely not compatible
// with other devices on the I2C bus concurrently.
// The code below it bit-banging the SDA and SLK lines
static short
SHT1xRead_OC(unsigned char cmd)
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
	  // Read the 16 Bit of data from the device for the commanded reading -- Provide ACK pulse to device after first 8 bit
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
	  DDRC = 0x1; 	// input, pull up for a high -- should be input from loop already
	  PORTC = 0;	// extra time to allow Pu to rasie dataline
	  PORTC = 0;	// extra time to allow Pu to rasie dataline
	  PORTC = 0;	// extra time to allow Pu to rasie dataline
	  PORTC =0x1;	// ACK clock  -- Data is high == NAK
	  PORTC =0x1;	// ACK clock  -- Data is high == NAK
	  PORTC =0x0; 	// ACK clock done

return SO;
}
#define RH_THRES 70
#define RH_HYST 5


static bool
SHT1xRead(float *temp, float *Rh)
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
static void
SHT1xInit( void )
{
	DDRD  |=  0x10;
}



static void
Measure_SHT11(  sftSensorReading *reading)
{
	reading->SensorType = Hyg;
	SHT1xRead(&reading->readings[0], &reading->readings[1]);
}
#endif

#if (SENSOR_TYPE == SENSOR_BMP085)


static bool BMP085_Present = false;
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


static _Bool
BMP085_init(void )
{
	unsigned char i;
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

}

static unsigned short
BMP085_getTemp()
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


static bool BMP085_Read(float *t,float *press)
{
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

	return true;
}

static void
Measure_BMP085 ( sftSensorReading *reading)
{
	volatile float val;
	reading->SensorType=BMP085;
	BMP085_Read(&reading->readings[0] , &reading->readings[1]);

	// Now read the battery voltage
	HAL_SELECT_ADC7();
	HAL_START_ADC();
	HAL_WAIT_ADC();
	val = (float) HAL_READ_ADC() ;
	val *=((26.1+6.8)/6.8);			// input Voltage divider
	val *= 2.5e-3;		// in volts -- using 2.56V internal ref and x1 gain
	reading->readings[2]= val;
}
#endif

void
Do_SensorMeasurment( sftSensorReading *reading)
{

	reading->SensorType =None;
	reading->readings[0] = reading->readings[1] = 0;
	reading->readings[2] = reading->readings[3] = 0;

#if (SENSOR_TYPE == SENSOR_BMP085 )
	Measure_BMP085 (reading);
#endif

#if (SENSOR_TYPE == SENSOR_SHT11)
	Measure_SHT11(reading);
#endif

#if (SENSOR_TYPE == SENSOR_RSSI)
	reading->SensorType = RSSI;
	reading->readings[0] = radioGetSavedRssiValue();
#endif

#if (SENSOR_TYPE == SENSOR_TMP100)
	Measure_TMP100( reading);
#endif

#if (SENSOR_TYPE == SENSOR_GAS_HC)
	Measure_HC( reading);
#endif

#if (SENSOR_TYPE == SENSOR_SOILM)
	Measure_SoilM( reading);
#endif

#if (SENSOR_TYPE == SENSOR_BAT)
	Measure_Vbat_highV(reading);
#endif

}

//
void
Do_SensorInit( void )
{

	HAL_ADC_INIT();			// Initialize the ADC function of the chip

#if (SENSOR_TYPE == SENSOR_BMP085)
	i2c_init(200000UL);		// 200Khz clock
	BMP085_init();
#endif

#if (SENSOR_TYPE ==  SENSOR_TMP100)
	i2c_init(200000UL);		// 200Khz clock
	TMP100_init();
#endif


#if (SENSOR_TYPE ==  SENSOR_SHT11) // Make PD4 an output
	i2c_init(200000UL);		// 200Khz clock
	SHT1xInit();
#endif


#if (SENSOR_TYPE == SENSOR_SOILM)
	DDRD  |=  (1<<PD5);			// PD5 controls power to Vegetronix --
	PORTD &= ~(1<<PD5);			// Turn Vegetronix power off
#endif


}
