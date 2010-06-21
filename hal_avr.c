/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/
/*
  $Id: hal_avr.c,v 1.3 2009/05/28 19:45:34 bleverett Exp $
*/

/*============================ INCLUDE =======================================*/
#include <stdlib.h>
#include "mac.h"
#include "radio.h"
/*============================ MACROS ========================================*/

void radioRxStartEvent(u8 const frame_length);

/**
   @ingroup radio
   @{
*/

/*
 * Macros defined for the radio transceiver's access modes.
 *
 * These functions are implemented as macros since they are used very often and
 * we want to remove the function call overhead.
 */
#define HAL_DUMMY_READ         (0x00) //!< Dummy value for the SPI.

#define HAL_TRX_CMD_RW         (0xC0) //!< Register Write (short mode).
#define HAL_TRX_CMD_RR         (0x80) //!< Register Read (short mode).
#define HAL_TRX_CMD_FW         (0x60) //!< Frame Transmit Mode (long mode).
#define HAL_TRX_CMD_FR         (0x20) //!< Frame Receive Mode (long mode).
#define HAL_TRX_CMD_SW         (0x40) //!< SRAM Write.
#define HAL_TRX_CMD_SR         (0x00) //!< SRAM Read.
#define HAL_TRX_CMD_RADDRM     (0x7F) //!< Register Address Mask.

#define HAL_CALCULATED_CRC_OK   (0) //!< CRC calculated over the frame including the CRC field should be 0.
/*============================ TYPDEFS =======================================*/
/*============================ VARIABLES =====================================*/
/*Flag section.*/

/*============================ PROTOTYPES ====================================*/
void radioTrxEndEvent(void);
void macEdCallback(void);

extern u8 rx_mode;

/*============================ IMPLEMENTATION ================================*/


void radio_spi_init(void)
{
    /*SPI Specific Initialization.
     * This code relies upon the reset condition of the port registers affected, i.e. register values of 0.
     *
     */

    //Set SCK SS and MOSI as output.
	//NOTE: SS PIN must be configured as output, otherwise SPI will not function in Master mode
    HAL_DDR_SPI  |= ((1 << HAL_DD_SCK) | (1 << HAL_DD_MOSI) | (1 << SSPIN));
    HAL_PORT_SPI |= (1 << HAL_DD_SCK) |(1 << SSPIN); //Set CLK and SS pin high

    // make RF chip device sel inactive
    HAL_DDR_SPI  |= (1<<RF_CHIP_DEVSEL);
    HAL_PORT_SPI |= (1<<RF_CHIP_DEVSEL);


    //Enable SPI module and master operation.
    SPCR         = (1 << SPE) | (1 << MSTR);
    //Enable doubled SPI speed in master mode.
    SPSR         = (1 << SPI2X);
}

/*! \brief  This function initializes the Hardware Abstraction Layer.
 *
 */
void radio_hal_init(void)
{
    /*IO Specific Initialization.*/
    DDR_SLP_TR |= (1 << SLP_TR); //Enable SLP_TR as output.
    DDR_RST    |= (1 << RST);    //Enable RST as output.

    radio_spi_init();
    radiol_enable_trx_interrupt();    //Enable interrupts from the radio transceiver.
}

/*! \brief  This function reads data from one of the radio transceiver's registers.
 *
 *  \param  address Register address to read from. See datasheet for register
 *                  map.
 *
 *  \see Look at the at86rf23x_registermap.h file for register address definitions.
 *
 *  \returns The actual value of the read register.
 *
 */
u8 radio_register_read(u8 address)
{
    //Add the register read command to the register address.
    //    address &= HAL_TRX_CMD_RADDRM;
    address |= HAL_TRX_CMD_RR;

    u8 register_value = 0;

    AVR_ENTER_CRITICAL_REGION();

    RADIO_DEVSEL_L(); //Start the SPI transaction by pulling the Slave Select low.

    /*Send Register address and read register content.*/
    SPDR = address;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    register_value = SPDR;

    SPDR = register_value;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    register_value = SPDR;

    RADIO_DEVSEL_H(); //End the transaction by pulling the Slave Select High.

    AVR_LEAVE_CRITICAL_REGION();

    return register_value;
}

/*! \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *
 *  \see Look at the at86rf23x_registermap.h file for register address definitions.
 *
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 *
 */
void radio_register_write(u8 address, u8 value)
{
    //Add the Register Write command to the address.
    address = HAL_TRX_CMD_RW | (HAL_TRX_CMD_RADDRM & address);

    AVR_ENTER_CRITICAL_REGION();

    RADIO_DEVSEL_L(); //Start the SPI transaction by pulling the Slave Select low.

    /*Send Register address and write register content.*/
    SPDR = address;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    SPDR;  // Dummy read of SPDR

    SPDR = value;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    SPDR;  // Dummy read of SPDR

    RADIO_DEVSEL_H(); //End the transaction by pulling the Slave Slect High.

    AVR_LEAVE_CRITICAL_REGION();

    // Set the rx_mode variable based on how we set the radio
    if ((address & ~HAL_TRX_CMD_RW) == RG_TRX_STATE)
    {
        // set rx_mode flag based on mode we're changing to
        value &= 0x1f;   // Mask for TRX_STATE register
        if (value == RX_ON ||
            value == RX_AACK_ON)
            rx_mode = true;
        else
            rx_mode = false;
    }
}

/*! \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf23x_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 *
 */
u8 radio_subregister_read(u8 address, u8 mask, u8 position)
{
    //Read current register value and mask out subregister.
    u8 register_value = radio_register_read(address);
    register_value &= mask;
    register_value >>= position; //Align subregister value.

    return register_value;
}

/*! \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf23x_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 *
 */
void radio_subregister_write(u8 address, u8 mask, u8 position,
                            u8 value)
{
    //Read current register value and mask area outside the subregister.
    u8 register_value = radio_register_read(address);
    register_value &= ~mask;

    //Start preparing the new subregister value. shift in place and mask.
    value <<= position;
    value &= mask;

    value |= register_value; //Set the new subregister value.

    //Write the modified register value.
    radio_register_write(address, value);
}

/*! \brief  This function will upload a frame from the radio transceiver's frame
 *          buffer.
 *
 *          If the frame currently available in the radio transceiver's frame buffer
 *          is out of the defined bounds. Then the frame length, lqi value and crc
 *          be set to zero. This is done to indicate an error.
 *
 */
void radio_frame_read(void)
{
    u8 *rx_data=mac_buffer_rx;
    rx_frame_t *rx_frame=(rx_frame_t*)mac_buffer_rx;

    AVR_ENTER_CRITICAL_REGION();

    RADIO_DEVSEL_L();

    /*Send frame read command.*/
    SPDR = HAL_TRX_CMD_FR;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    u8 frame_length = SPDR;

    /*Read frame length.*/
    SPDR = frame_length;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    frame_length = SPDR;

    /*Check for correct frame length.*/
    if ((frame_length >= HAL_MIN_FRAME_LENGTH) && (frame_length <= HAL_MAX_FRAME_LENGTH))
    {
        *rx_data++ = frame_length; //Store frame length.

        /*Upload frame buffer to data pointer. Calculate CRC.*/
        SPDR = frame_length;
        while ((SPSR & (1 << SPIF)) == 0)
            ;

        do
        {
            u8 tempData = SPDR;
            SPDR = 0;       // dummy write

            *rx_data++ = tempData;
            while ((SPSR & (1 << SPIF)) == 0)
                ;
        } while (--frame_length > 0);

        /*Read LQI value for this frame.*/
        rx_frame->lqi = SPDR;
/*
        // get frames ED
        SPDR = 0;
        while ((SPSR & (1 << SPIF)) == 0) {;}
        rx_frame->ED = SPDR;

        // get frame RX_Status
         SPDR = 0;
         while ((SPSR & (1 << SPIF)) == 0) {;}
            rx_frame->Rx_stat = SPDR;
*/
        RADIO_DEVSEL_H();
    }
    else
    {
        RADIO_DEVSEL_H();

        if (rx_frame)
        {
            rx_frame->length = 0;
            rx_frame->lqi    = 0;
            rx_frame->crc    = false;
        }
    }

    AVR_LEAVE_CRITICAL_REGION();
}

/*! \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 *
 */
void radio_frame_write(u8 *write_buffer, u8 length)
{
    length &= HAL_TRX_CMD_RADDRM; //Truncate length to maximum frame length.

    AVR_ENTER_CRITICAL_REGION();

    //Toggle the SLP_TR pin to initiate the frame transmission.
    hal_set_slptr_high();
    hal_set_slptr_low();

    RADIO_DEVSEL_L(); //Initiate the SPI transaction.

    /*SEND FRAME WRITE COMMAND AND FRAME LENGTH.*/
    SPDR = HAL_TRX_CMD_FW;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    SPDR; // Dummy read of SPDR

    SPDR = length;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    SPDR;  // Dummy read of SPDR

    // Download to the Frame Buffer.
    do
    {
        SPDR = *write_buffer++;
        --length;

        while ((SPSR & (1 << SPIF)) == 0)
            ;

        SPDR;  // Dummy read of SPDR
    } while (length > 0);

    RADIO_DEVSEL_H(); //Terminate SPI transaction.

    AVR_LEAVE_CRITICAL_REGION();
}

/*! \brief Read SRAM
 *
 * This function reads from the SRAM of the radio transceiver.
 *
 * \param address Address in the TRX's SRAM where the read burst should start
 * \param length Length of the read burst
 * \param data Pointer to buffer where data is stored.
 *
 */
void radio_sram_read(u8 address, u8 length, u8 *data)
{
    AVR_ENTER_CRITICAL_REGION();

    RADIO_DEVSEL_L(); //Initiate the SPI transaction.

    /*Send SRAM read command.*/
    SPDR = HAL_TRX_CMD_SR;
    while ((SPSR & (1 << SPIF)) == 0)
        ;
    SPDR;  // Dummy read of SPDR

    /*Send address where to start reading.*/
    SPDR = address;
    while ((SPSR & (1 << SPIF)) == 0)
        ;
    SPDR;  // Dummy read of SPDR

    /*Upload the chosen memory area.*/
    do
    {
        SPDR = HAL_DUMMY_READ;
        while ((SPSR & (1 << SPIF)) == 0) {;}
        *data++ = SPDR;
    } while (--length > 0);

    RADIO_DEVSEL_H();

    AVR_LEAVE_CRITICAL_REGION();
}

/*! \brief Write SRAM
 *
 * This function writes into the SRAM of the radio transceiver.
 *
 * \param address Address in the TRX's SRAM where the write burst should start
 * \param length  Length of the write burst
 * \param data    Pointer to an array of bytes that should be written
 *
 */
void radio_sram_write(u8 address, u8 length, u8 *data)
{
    AVR_ENTER_CRITICAL_REGION();

    RADIO_DEVSEL_L();

    /*Send SRAM write command.*/
    SPDR = HAL_TRX_CMD_SW;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    SPDR;  // Dummy read of SPDR

    /*Send address where to start writing to.*/
    SPDR = address;
    while ((SPSR & (1 << SPIF)) == 0)
        ;
    SPDR;  // Dummy read of SPDR

    /*Upload the chosen memory area.*/
    do
    {
        SPDR = *data++;
        while ((SPSR & (1 << SPIF)) == 0)
            ;
        SPDR;  // Dummy read of SPDR
    } while (--length > 0);

    RADIO_DEVSEL_H();

    AVR_LEAVE_CRITICAL_REGION();
}


/*! ISR for the radio IRQ line, triggered by the input capture.
 *  This is the interrupt service routine for timer1.ICIE1 input capture.
 *  It is triggered of a rising edge on the radio transceivers IRQ line.
 */

ISR(RADIO_VECT)
{
    /*Read Interrupt source.*/
    u8 interrupt_source = radio_register_read(RG_IRQ_STATUS);

    if (interrupt_source & HAL_TRX_END_MASK)
        radioTrxEndEvent();

    // Energy detect event
    if (interrupt_source & HAL_ED_READY_MASK)
        macEdCallback();

    /* Handle the incomming interrupt. Prioritized.
       Other Interrupts: 
       HAL_TRX_UR_MASK
       HAL_PLL_UNLOCK_MASK
       HAL_PLL_LOCK_MASK
    */
    if ((interrupt_source & HAL_RX_START_MASK))
    {
        /*Read Frame length and call rx_start callback.*/
        RADIO_DEVSEL_L();

        SPDR = HAL_TRX_CMD_FR;
        while ((SPSR & (1 << SPIF)) == 0) {;}
        SPDR;

        SPDR = 0; // Send dummy byte so we can read one byte back
        while ((SPSR & (1 << SPIF)) == 0) {;}
        u8 frame_length = SPDR;

        RADIO_DEVSEL_H();

        radioRxStartEvent(frame_length);
    }
    else if (interrupt_source & HAL_BAT_LOW_MASK)
    {
        //Disable BAT_LOW interrupt to prevent interrupt storm. The interrupt
        //will continously be signaled when the supply voltage is less than the
        //user defined voltage threshold.
        u8 trx_isr_mask = radio_register_read(RG_IRQ_MASK);
        trx_isr_mask &= ~HAL_BAT_LOW_MASK;
        radio_register_write(RG_IRQ_MASK, trx_isr_mask);
    }
    else
        ; // unknown ISR
}



/**
   General-purpose function to read data out of eeprom

   param:  offset The offset in EEPROM of the start of the data block
   param:  length The length in bytes of the data block
   param:  dest  Pointer to the area in memory to place the data block
*/
void halGetEeprom(u8 *addr, u8 length, u8 *dest)
{
    AVR_ENTER_CRITICAL_REGION();
    eeprom_read_block (dest, addr, length);
    AVR_LEAVE_CRITICAL_REGION();
}

/**
   General-purpose function to write data to eeprom

   param:  offset The offset in EEPROM of the start of the data block
   param:  length The length in bytes of the data block
   param:  src  Pointer to the area in memory which contains the data block
*/
void halPutEeprom(u8 *addr, u8 length, u8 *src)
{
    AVR_ENTER_CRITICAL_REGION();
    eeprom_write_block (src, addr, length);
    AVR_LEAVE_CRITICAL_REGION();
}

/**
   Initialize the AVR clock speed.
*/
void halSetupClock(void)
{

    // Set clock speed based on Xtal and F_CPU define
	
	// It programs the ClockPrescaleRegister to divide Fosc to the value given in the F_CPU define
	// It assumes that Fosc is running at 16Mhz, either by external Xtal osc.

//NOTE on sharing clocks between RF212 and MCU:
/*	Since the AVR can not switch the clock source programmatically we are stuck with the clock source as programmed
 *  via the fuse bits and can not switch in the RF212's clock output after starting up on the RC osc. Dynamic switching
 *  of the MCU's clock source would be necessary since the radio chip disables the clock during reset and sleep cuasing a
 *  deadlock in the CPU.
 *
 *  Using the internal ~8mhz RC oscillator would be ok fine, except that the UART bases the baud rate on the CPU clock
 *  and it might not be precise enough for the serial debug output.
 *
 *  Using an external Xtal on the MCU and feeding the clock to the radio seems like a good choice, except that the radio
 *  requires a 16Mhz clock and a VCC of no more than 3.6V. The  (20mhz) MCU is not guaranteed to run at 16Mhz below 4.5V.
 *  and therefore this solution is not guaranteed to work in all situations, although it seems to work on the proto board.
 *
 *  So, to guarantee proper operation of the MCU AND serial port one will have to use an 8Mhz Xtal on the MCU and
 *  a 16Mhz Xtal on the radio or use the internal 8Mhz RC osc and then calibrate it with the help of the 32Khz timer xtal.
 *
 *  For a final product where the serial port is not used, the xtal on the MCU can be omitted and the internal 8Mhz Osc can
 *  be used.
 *
 */
	AVR_ENTER_CRITICAL_REGION();
    CLKPR = 1 << CLKPCE;  				// Set the change-enable flag
    CLKPR = ((F_OSC/F_CPU)-1);          // Set for divide-by-n,
    AVR_LEAVE_CRITICAL_REGION();
    
}


