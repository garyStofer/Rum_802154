/* Copyright (c) 2008-2009 ATMEL Corporation
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
 * $Id: bootloader.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/
/**
 * @file
 *         AVR Bootloader Function
 */

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/wdt.h>
#include "system.h"
#include "bootloader.h"
#include "radio.h"

#if IPV6LOWPAN

uint32_t bootloaderEndAddress;

/**
 * \brief Copies program from temporary storage to main FLASH
 * \param endAddress Ending address of temporary storage space in FLASH
 */
void boot_copy_program(void)
{
#if FLASHEND > 0x10000
    uint32_t readAddr = BOOTLOADER_INITIAL_ADDR;
#else
    uint16_t readAddr = BOOTLOADER_INITIAL_ADDR;
#endif
    uint32_t writeAddr = 0;

	uint16_t i;

    cli();

    while(readAddr < bootloaderEndAddress)
    {
        eeprom_busy_wait();

        boot_page_erase (writeAddr);
        boot_spm_busy_wait ();      // Wait until the memory is erased.

        //We read from RWW section, so reenable it and wait
        //for it to be ready
        boot_rww_enable ();
        while(boot_rww_busy());

        //Read a dword from memory, program into buffer
        for (i=0; i<SPM_PAGESIZE; i+=2)
        {
#if FLASHEND > 0x10000
            boot_page_fill (writeAddr + i, pgm_read_dword_far(readAddr));
#else
            boot_page_fill (writeAddr + i, pgm_read_dword(readAddr));
#endif
            readAddr += 2;
        }

        boot_page_write (writeAddr);     // Store buffer in flash page.
        boot_spm_busy_wait();       // Wait until the memory is written.

        writeAddr += SPM_PAGESIZE;

    }


    //Reenable RWW section
	boot_rww_enable ();
    
	//Force a reboot
	wdt_enable(WDTO_120MS);

	while(1);

    return;
}

/**
 * \brief Programs a page in FLASH
 * \param page Byte address of the page
 * \param buf Pointer to data to write
 */
void boot_program_page (uint32_t page, uint8_t *buf)
{
        uint16_t i;
        uint8_t sreg;

        // Disable interrupts.

        sreg = SREG;
        cli();

        eeprom_busy_wait();

        boot_page_erase (page);
        boot_spm_busy_wait ();      // Wait until the memory is erased.

        for (i=0; i<SPM_PAGESIZE; i+=2)
        {
            // Set up little-endian word.

            uint16_t w = *buf++;
            w += (*buf++) << 8;

            boot_page_fill (page + i, w);
        }

        boot_page_write (page);     // Store buffer in flash page.
        boot_spm_busy_wait();       // Wait until the memory is written.

        // Reenable RWW-section again. We need this if we want to jump back
        // to the application after bootloading.

        boot_rww_enable ();

        // Re-enable interrupts (if they were ever enabled).

        SREG = sreg;
}

#endif
