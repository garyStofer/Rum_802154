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
  $Id: mac_beacon.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

#include <stdio.h>
#include <string.h>
#include "mac.h"
#include "mac_beacon.h"
#include "mac_route.h"
#include "radio.h"
#include "timer.h"


/**
   Create and send a beacon frame.  This is called in response to a
   beacon request frame.
*/
void
sendBeaconFrame(void)
{
#   if (NODETYPE == ROUTER || NODETYPE== COORD)
    {
#if DEBUG==2
    	debugMsgStr("\r\nTX BF");
#endif
    	blink_blue(25);
        ftBeacon* data_frame = (ftBeacon*)(mac_buffer_tx+1);

        data_frame->fcf   = FCF_BEACON;
        data_frame->seq   = macConfig.bsn++;
        data_frame->panid = macConfig.panId;
        data_frame->addr  = macConfig.shortAddress;

        if(NODETYPE == ROUTER)
            data_frame->superFrame = ROUTER_SUPERFRAME;
        else
            data_frame->superFrame = COORD_SUPERFRAME;

        data_frame->netID = 0x06;
        data_frame->hops = macConfig.hopsToCoord;

        // send data to radio.
        radioSendData(sizeof(ftBeacon), (u8 *)data_frame);
    }
#   endif
}
