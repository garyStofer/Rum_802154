
#ifndef HAL_AVR_H
#define HAL_AVR_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "rum_types.h"

#define MINER_A   11
#define MINER_B   12

/**
   The BAND macro is set to one particular band based
   on which platform selected for compilation.  Do not manually set
   this macro.

   The BAND macro is set to one of the following two bands.

*/
#define BAND900    1
#define BAND2400   2

/* Atmel Miner  */
#if PLATFORM==MINER_A

		#if (!defined(__AVR_ATmega324P__) )

                #error "Incorrect MCU for Platform! Check Makefile"
        #endif

		//HAS to be defined to be the actual Osc. freq. of the device, either internal RC or external Xtal
		#define F_OSC 8000000L

        /* ATMEGA324P */
		// These SPI pins and ports are hardwired in the cpu.
		// Must use these pins otherwise SPI will not function
        #   define SPIPORT    B
        #   define SSPIN      (0x04)
        #   define MOSIPIN    (0x05)
        #   define MISOPIN    (0x06)
        #   define SCKPIN     (0x07)

		// These pins and ports can be assigned
		#	define RF_CHIP_DEVSEL	(4)	// must be on same port as SPI i.e. "B"
        #   define RSTPORT    B
        #   define RSTPIN     (0x01)
        #   define SLPTRPORT  B
        #   define SLPTRPIN   (0x03)
        #   define USART      0
  		#   define TICKTIMER  1			// 16 bit counter

 		#   define RADIO_VECT INT2_vect  ///< Radio interrupt vector
            /// Macro to enable the radio interrupt -- make sure the bits match the above int vector
        #   define HAL_ENABLE_RADIO_INTERRUPT( ) EICRA |= 0x30, EIMSK |= 4
            /// Macro to disable the radio interrupt
        #   define HAL_DISABLE_RADIO_INTERRUPT( ) EICRA &= ~0x30, EIMSK &= ~4

        // TODO using AREF Pin as input of VCC for now, but need to change AREF circuit to have cap instead
        // Using Xtal/16 clock
        #   define HAL_ADC_INIT()    ADMUX = 0, ADCSRA = 0xD4, DIDR0=0xff	// Mux and Ref, Enable and start a convsersion, Prescale = /16
        #   define HAL_STOP_ADC()    ADCSRA &= ~0x80						// Disable, ADEN =0
        #   define HAL_START_ADC()   ADCSRA |= (1 << ADSC) | (1 << ADIF) | (1 << ADEN)	//  Set SC,EN and clear IF
        #   define HAL_WAIT_ADC()    while (!(ADCSRA & (1<<ADIF))) {;}; ADCSRA |= (1<<ADIF) // Wait for ADC IF and clear IF afterwards again

        #   define HAL_READ_ADC()    ADC
        #   define HAL_SELECT_ADC0()   ADMUX = (ADMUX & 0xF0)
        #   define HAL_SELECT_ADC1()   ADMUX = (ADMUX & 0xF0) | 0x01
        #   define HAL_SELECT_ADC2()   ADMUX = (ADMUX & 0xF0) | 0x02
        #   define HAL_SELECT_ADC3()   ADMUX = (ADMUX & 0xF0) | 0x03
        #   define HAL_SELECT_ADC4()   ADMUX = (ADMUX & 0xF0) | 0x04
        #   define HAL_SELECT_ADC5()   ADMUX = (ADMUX & 0xF0) | 0x05
        #   define HAL_SELECT_ADC6()   ADMUX = (ADMUX & 0xF0) | 0x06
        #   define HAL_SELECT_ADC7()   ADMUX = (ADMUX & 0xF0) | 0x07

	    #   define BAND BAND900

        #define Leds_init()               (DDRD  |=  0x18)
        #define Leds_on()                 (PORTD |=  0x18)
        #define Leds_off()                (PORTD &= ~0x18)
       
	    #define Led0_on()                 (PORTD |= 0x10)
        #define Led1_on()                 (PORTD |= 0x8)
        
        #define Led0_off()                (PORTD &= ~0x10)
        #define Led1_off()                (PORTD &= ~0x8)

        #define Led0_toggle()             (PIND |= 0x10)
        #define Led1_toggle()             (PIND |= 0x8)
    
        // LED Macros
        #define LED_INIT()                  Leds_init()
        // LED_ON(led), where led doesn't matter - there is only one LED on the board.
        #define LED_ON(led)                 Led##led##_on()
        #define LED_OFF(led)                Led##led##_off()


        // Button macros
        #define BUTTON_SETUP()
        #define BUTTON_PRESSED() 0
//        #define BUTTON_SETUP()          DDRB &= ~(1 << PB0), PORTB |= (1 << PB0)
//        #define BUTTON_PRESSED()        (!(PINB & (1 << PB0)))
#elif PLATFORM==MINER_B
		#if (! (defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644__)))

                #error "Incorrect MCU for Platform! Check Makefile"
        #endif

		//HAS to be defined to be the actual Osc. freq. of the device, either internal RC or external Xtal
		#define F_OSC 8000000L

        /* ATMEGA324P */
		// These SPI pins and ports are hardwired in the cpu.
		// Must use these pins otherwise SPI will not function
		#   define SPIPORT    B
        #   define SSPIN      (4)
        #   define MOSIPIN    (5)
        #   define MISOPIN    (6)
        #   define SCKPIN     (7)

		// These pins and ports can be assigned
		#define RF_CHIP_DEVSEL	(0)	// must be on same port as SPI i.e. "B"

        #   define RSTPORT    B
        #   define RSTPIN     (3)
        #   define SLPTRPORT  B
        #   define SLPTRPIN   (1)

        #   define USART      0
  		#   define TICKTIMER  1			// 16 bit counter

 		#   define RADIO_VECT INT2_vect  ///< Radio interrupt vector
            /// Macro to enable the radio interrupt -- make sure the bits matche the above int vector
        #   define HAL_ENABLE_RADIO_INTERRUPT( ) EICRA |= 0x30, EIMSK |= 4
            /// Macro to disable the radio interrupt
        #   define HAL_DISABLE_RADIO_INTERRUPT( ) EICRA &= ~0x30, EIMSK &= ~4


		// Mux=0 and Ref=internal 2.56V, Enable and start a convsersion, clock Prescale = /16
        #   define HAL_ADC_INIT()    ADMUX = 0xc0, ADCSRA = 0xD4, DIDR0=0xff	// Mux=0 and Ref=internal 2.56V, Enable and start a convsersion, Prescale = /16
        #   define HAL_STOP_ADC()    ADCSRA &= ~0x80						// Disable, ADEN =0
        #   define HAL_START_ADC()   ADCSRA |= (1 << ADSC) | (1 << ADIF) | (1 << ADEN)	//  Set SC,EN and clear IF
        #   define HAL_WAIT_ADC()    while (!(ADCSRA & (1<<ADIF))) {;}; ADCSRA |= (1<<ADIF) // Wait for ADC IF and clear IF afterwards again

        #   define HAL_READ_ADC()      ADC
        #   define HAL_SELECT_ADC0()   ADMUX = (ADMUX & 0xE0)
        #   define HAL_SELECT_ADC1()   ADMUX = (ADMUX & 0xE0) | 0x01
        #   define HAL_SELECT_ADC2()   ADMUX = (ADMUX & 0xE0) | 0x02
        #   define HAL_SELECT_ADC3()   ADMUX = (ADMUX & 0xE0) | 0x03
        #   define HAL_SELECT_ADC4()   ADMUX = (ADMUX & 0xE0) | 0x04
        #   define HAL_SELECT_ADC5()   ADMUX = (ADMUX & 0xE0) | 0x05
        #   define HAL_SELECT_ADC6()   ADMUX = (ADMUX & 0xE0) | 0x06
        #   define HAL_SELECT_ADC7()   ADMUX = (ADMUX & 0xE0) | 0x07
		#   define HAL_SELECT_1_1V()   ADMUX = (ADMUX & 0xE0) | 0x1E
		#	define HAL_SELECT_GND()    ADMUX = (ADMUX & 0xE0) | 0x1F

	    #   define BAND BAND900

        #define Leds_init()               (DDRD  |=  0x0c)
        #define Leds_on()                 (PORTD |=  0x0c)
        #define Leds_off()                (PORTD &= ~0x0c)

	    #define Led0_on()                 (PORTD |= 0x08)
        #define Led1_on()                 (PORTD |= 0x04)

        #define Led0_off()                (PORTD &= ~0x08)
        #define Led1_off()                (PORTD &= ~0x04)

        #define Led0_toggle()             (PIND |= 0x08)
        #define Led1_toggle()             (PIND |= 0x04)

        // LED Macros
        #define LED_INIT()                  Leds_init()
        // LED_ON(led), where led doesn't matter - there is only one LED on the board.
        #define LED_ON(led)                 Led##led##_on()
        #define LED_OFF(led)                Led##led##_off()


        // Button macros
        #define BUTTON_SETUP()
        #define BUTTON_PRESSED() 0
//        #define BUTTON_SETUP()          DDRB &= ~(1 << PB0), PORTB |= (1 << PB0)
//        #define BUTTON_PRESSED()        (!(PINB & (1 << PB0)))
#else
        #error PLATFORM undefined or incorrect value
#endif

#if (F_CPU > F_OSC)
#error "CPU frequency can not be higher than Osc frequency";
#endif

#if BAND == BAND2400
#include "at86rf23x_registermap.h"
#elif BAND == BAND900
#include "at86rf212_registermap.h"
#else
#error "BAND Undefined!"
#endif


/**
    @name Macros used to generate register names

    The various CAT macros (DDR, PORT, and PIN) are used to assign
    port/pin/DDR names to various macro variables.  The variables are
    assigned based on the specific connections made in the hardware.
    For example TCCR(TICKTIMER,A) can be used in place of TCCR0A if
    TICKTIMER is defined as 0. This setup allows changing which
    resources are used on a PC board with minimal changes.
*/

#define CAT(x, y)      x##y                ///< Concatenate two strings
#define CAT2(x, y, z)  x##y##z             ///< Concatenate three strings
#define DDR(x)         CAT(DDR,  x)        ///< Data direction register macro
#define PORT(x)        CAT(PORT, x)
#define PIN(x)         CAT(PIN,  x)
#define UCSR(num, let) CAT2(UCSR,num,let)
#define RXEN(x)        CAT(RXEN,x)
#define TXEN(x)        CAT(TXEN,x)
#define TXC(x)         CAT(TXC,x)
#define RXC(x)         CAT(RXC,x)
#define RXCIE(x)       CAT(RXCIE,x)
#define UCSZ(x,y)      CAT2(UCSZ,x,y)
#define UBRR(x)        CAT(UBRR,x)
#define UDRE(x)        CAT(UDRE,x)
#define UDRIE(x)       CAT(UDRIE,x)
#define UDR(x)         CAT(UDR,x)
#define TCNT(x)        CAT(TCNT,x)
#define TIMSK(x)       CAT(TIMSK,x)
#define TCCR(x,y)      CAT2(TCCR,x,y)
#define COM(x,y)       CAT2(COM,x,y)
#define OCR(x,y)       CAT2(OCR,x,y)
#define CS(x,y)        CAT2(CS,x,y)
#define WGM(x,y)       CAT2(WGM,x,y)
#define OCIE(x,y)      CAT2(OCIE,x,y)
#define COMPVECT(x)    CAT2(TIMER,x,_COMPA_vect)
#define UDREVECT(x)    CAT2(USART,x,_UDRE_vect)
#define RXVECT(x)      CAT2(USART,x,_RX_vect)


/**
   @name Macros to use in source code for platform-specific names

*/
#define SLP_TR                SLPTRPIN            ///< Pin number that corresponds to the SLP_TR pin.
#define DDR_SLP_TR            DDR( SLPTRPORT )    ///< Data Direction Register that corresponds to the port where SLP_TR is connected.
#define PORT_SLP_TR           PORT( SLPTRPORT )   ///< Port (Write Access) where SLP_TR is connected.
#define PIN_SLP_TR            PIN( SLPTRPORT )    ///< Pin (Read Access) where SLP_TR is connected.
#define hal_set_slptr_high( ) ( PORT_SLP_TR |= ( 1 << SLP_TR ) )      /// < This macro pulls the SLP_TR pin high.
#define hal_set_slptr_low( )  ( PORT_SLP_TR &= ~( 1 << SLP_TR ) )     ///< This macro pulls the SLP_TR pin low.
#define hal_get_slptr( ) (    ( PIN_SLP_TR & ( 1 << SLP_TR ) ) >> SLP_TR )  ///< Read current state of the SLP_TR pin (High/Low).
#define RST                   RSTPIN              ///< Pin number that corresponds to the RST pin.
#define DDR_RST               DDR( RSTPORT )      ///< Data Direction Register that corresponds to the port where RST is
#define PORT_RST              PORT( RSTPORT )     ///< Port (Write Access) where RST is connected.
#define PIN_RST               PIN( RSTPORT )      ///< Pin (Read Access) where RST is connected.
#define hal_set_rst_high( )   ( PORT_RST |= ( 1 << RST ) )  ///< This macro pulls the RST pin high.
#define hal_set_rst_low( )    ( PORT_RST &= ~( 1 << RST ) ) ///< This macro pulls the RST pin low.
#define hal_get_rst( )        ( ( PIN_RST & ( 1 << RST )  ) >> RST )  ///< Read current state of the RST pin (High/Low).

#define HAL_PORT_SPI          PORT( SPIPORT )     ///< The SPI module PORT.
#define HAL_DDR_SPI           DDR( SPIPORT )      ///< Data Direction Register for the SPI port.
#define HAL_DD_SS             SSPIN               ///< Data Direction bit for SS.
#define HAL_DD_SCK            SCKPIN              ///< Data Direction bit for SCK.
#define HAL_DD_MOSI           MOSIPIN             ///< Data Direction bit for MOSI.
#define HAL_DD_MISO           MISOPIN             ///< Data Direction bit for MISO.



#define RADIO_DEVSEL_H( ) (HAL_PORT_SPI  |=  ( 1<< RF_CHIP_DEVSEL)) //!< MACRO for pulling SS high.
#define RADIO_DEVSEL_L( )  (HAL_PORT_SPI  &= ~( 1<< RF_CHIP_DEVSEL)) //!< MACRO for pulling SS low.



/** This macro will protect any subsequent code from interrupts. */
#define AVR_ENTER_CRITICAL_REGION( ) {u8 volatile saved_sreg = SREG; cli( )

/** This macro ends a protected block of code and must always be used
    in conjunction with   AVR_ENTER_CRITICAL_REGION.  */
#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

// Debugging macros
#if DEBUG && SERIAL
#include "serial.h"
#include <avr/pgmspace.h>
#define debugMsgChr(c) serial_putchar(c)
#define debugMsgStr_d(s) serial_puts(s)
#define debugMsgStr(s) serial_puts_P(PSTR(s))
#define debugMsgFlt(n) sprintf(debugStr,"%f",n), serial_puts(debugStr)
#define debugMsgInt(i) sprintf(debugStr,"%d",i), serial_puts(debugStr)
#define debugMsgHex(x) sprintf(debugStr,"%x",x), serial_puts(debugStr)
#define debugMsgCrLf() serial_puts_P(PSTR("\r\n"))
#elif DEBUG && OTA_DEBUG
#define otaDebugMsg(s,a) sprintf(debugStr,s,a);   \
    macOtaDebugRequest((u8*)debugStr);
#define otaDebugMsg0(s) sprintf(debugStr,s);   \
    macOtaDebugRequest((u8*)debugStr);
#define debugMsgChr(c) otaDebugMsg("%c", c)
#define debugMsgStr(s) otaDebugMsg("%s", s)
#define debugMsgFlt(n) otaDebugMsg("%f", n)
#define debugMsgInt(i) otaDebugMsg("%d",i)
#define debugMsgHex(x) otaDebugMsg("%x",x)
#define debugMsgCrLf() otaDebugMsg0("\r\n")
#else
#define debugMsgChr(c)
#define debugMsgStr_d(s) 
#define debugMsgStr(s)
#define debugMsgFlt(n)
#define debugMsgInt(i)
#define debugMsgHex(x)
#define debugMsgCrLf()
#endif

// Macros used to ensure compatibility with ARM code
// ARM is a 32-bit machine, so it needs to declare a
// temporary variable, which is guaranted to be on a
// 32-bit boundary.  The AVR should not declare the
// temporary variable because it eats up a lot of flash
// and isn't required for AVR.
#define  DECLARE64(x)
#define  USE64(x) ((u64*)&x)

/// Enable the interrupt from the radio transceiver.
#define radiol_enable_trx_interrupt( ) HAL_ENABLE_RADIO_INTERRUPT( )

/// Disable the interrupt from the radio transceiver.
#define hal_disable_trx_interrupt( ) HAL_DISABLE_RADIO_INTERRUPT( )


/// @name Macros for radio operation.

#define HAL_PLL_LOCK_MASK      ( 0x01 ) //!< Mask for the PLL_LOCK interrupt.
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) //!< Mask for the PLL_UNLOCK interrupt.
#define HAL_RX_START_MASK      ( 0x04 ) //!< Mask for the RX_START interrupt.
#define HAL_TRX_END_MASK       ( 0x08 ) //!< Mask for the TRX_END interrupt.
#define HAL_ED_READY_MASK      ( 0x10 ) //!< Mask for the ED_READY interrupt.
#define HAL_TRX_UR_MASK        ( 0x40 ) //!< Mask for the TRX_UR interrupt.
#define HAL_BAT_LOW_MASK       ( 0x80 ) //!< Mask for the BAT_LOW interrupt.


// Using the delay inline function from AVRLib
#define delay_us(us)  _delay_us(us)
/*============================ TYPDEFS =======================================*/
//! RX_START event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_rx_start_event_handler().
typedef void (*hal_rx_start_isr_event_handler_t)(u32 const isr_timestamp, u8 const frame_length);

//! RRX_END event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_trx_end_event_handler().
typedef void (*hal_trx_end_isr_event_handler_t)(u32 const isr_timestamp);

typedef void (*rx_callback_t) (u16 data);


/*============================ PROTOTYPES ====================================*/
void radio_hal_init( void );
void radio_spi_init(void);

u8   radio_register_read( u8 address );
void radio_register_write( u8 address, u8 value );
u8   radio_subregister_read( u8 address, u8 mask, u8 position );
void radio_subregister_write( u8 address, u8 mask, u8 position,
                            u8 value );
void radio_frame_read(void);
void radio_frame_write( u8 *write_buffer, u8 length );
void radio_sram_read( u8 address, u8 length, u8 *data );
void radio_sram_write( u8 address, u8 length, u8 *data );

void halGetEeprom(u8 *addr, u8 length, u8 *dest);
void halPutEeprom(u8 *addr, u8 length, u8 *src);

bool calibrate_rc_osc(void);
void halSetupClock(void);



#endif

