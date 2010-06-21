/* Copyright (c) 2008-2009  ATMEL Corporation
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

  $Id: sixlowpan_wake.h,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

#ifndef SIXLOWPAN_WAKE_H
#define SIXLOWPAN_WAKE_H

/**
 * @addtogroup avr6lowpansleeping
 * @{
 */

#include "avr_sixlowpan.h"

extern u16 frameInterval;
extern u16 appInterval;

/**   Time between node waking up and checking for traffic ( tenths of seconds) */
#define SIXLOWPAN_PERIODIC_TIME         frameInterval

/**   Number of SIXLOWPAN_PERIODIC_TIMEs between calling the periodic
 *         application */
#define SIXLOWPAN_PERIODIC_APP_TIME     appInterval


/**   Timeout before node goes to sleep if no activity occurs (mS) */
#define SIXLOWPAN_TIMEOUT_MS            7000

#define WAKEUP_PORT 0xF0B2
#define ATMEL_GET_AWAKE 0x01

#if ((NODETYPE==COORD) && IPV6LOWPAN)

void sixlowpanSleep_dataReceived (uint16_t src);
void sixlowpanSleep_wakeupSent(uint16_t shortaddress, uip_ipaddr_t * ipaddr, uint16_t port);
void sixlowpanSleep_awakeReceived(uint16_t shortaddress);
#endif

void sixlowpanSleep_sendAwake(void);
void sixlowpanSleep_timeout(void);
void sixlowpanSleep_periodic(void);
void sixlowpanSleep_setupPeriodic(void);
void sixlowpanSleep_sleep(void);
void sixlowpanSleep_checkSleep(void);

void sixlowpanSleep_activity(void);
void sixlowpanSleep_packetFailed(void);
void sixlowpanSleep_packetSucceed(void);
void sixlowpanSleep_init(void);


/** @} */

#endif //SIXLOWPAN_WAKE_H
