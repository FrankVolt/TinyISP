/*==============================================================================

  TinyISP 1.0 - Turn an ATmega328[P] Arduino compatible board (like an Uno) or
  a Teensy into an In System Programmer (ISP).  Based on MegaISP and 
  ArduinoISP.

  ----------------------------------------------------------------------------

  Copyright (c) 2012 Rowdy Dog Software
  All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer. 
    
  * Redistributions in binary form must reproduce the above copyright notice, 
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution. 
    
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
  POSSIBILITY OF SUCH DAMAGE.

==============================================================================*/

#ifndef KnockBangReceiver_h
#define KnockBangReceiver_h

#include <inttypes.h>
#include <avr/io.h>


#define MISO_PASTE(lft,rgt)       lft ## rgt
#define MISO_MAKE_REG(lft,rgt)    MISO_PASTE(lft,rgt)

#if defined( __AVR_ATmega328P__ ) || defined( __AVR_ATmega328__ ) || defined( __AVR_ATmega168__ )

  #define MISO_DDR    DDRB
  #define MISO_PORT   PORTB
  #define MISO_PIN    PINB
  #define MISO_BIT    4

  #define MISO_PCICR  PCICR
  #define MISO_PCIFR  PCIFR
  #define MISO_PCR    0

  #define MISO_PCIE   MISO_MAKE_REG(PCIE,MISO_PCR)
  #define MISO_PCMSK  MISO_MAKE_REG(PCMSK,MISO_PCR)
  #define MISO_PCIF   MISO_MAKE_REG(PCIF,MISO_PCR)

  #define KNOCKBANG_RECEIVER_AVAILABLE 1

#elif defined( __AVR_AT90USB646__ )

  #define MISO_DDR    DDRB
  #define MISO_PORT   PORTB
  #define MISO_PIN    PINB
  #define MISO_BIT    3

  #define MISO_PCICR  PCICR
  #define MISO_PCIFR  PCIFR
  #define MISO_PCR    0

  #define MISO_PCIE   MISO_MAKE_REG(PCIE,MISO_PCR)
  #define MISO_PCMSK  MISO_MAKE_REG(PCMSK,MISO_PCR)
  #define MISO_PCIF   MISO_MAKE_REG(PCIF,MISO_PCR)

  #define KNOCKBANG_RECEIVER_AVAILABLE 1

#elif defined( __AVR_AT90USB162__ )

  #define MISO_DDR    DDRB
  #define MISO_PORT   PORTB
  #define MISO_PIN    PINB
  #define MISO_BIT    3

  #define MISO_PCICR  PCICR
  #define MISO_PCIFR  PCIFR
  #define MISO_PCR    0

  #define MISO_PCIE   MISO_MAKE_REG(PCIE,MISO_PCR)
  #define MISO_PCMSK  MISO_MAKE_REG(PCMSK,MISO_PCR)
  #define MISO_PCIF   MISO_MAKE_REG(PCIF,MISO_PCR)

  #define KNOCKBANG_RECEIVER_AVAILABLE 1

#elif defined( __AVR_ATmega32U4__ )

  #define MISO_DDR    DDRB
  #define MISO_PORT   PORTB
  #define MISO_PIN    PINB
  #define MISO_BIT    3

  #define MISO_PCICR  PCICR
  #define MISO_PCIFR  PCIFR
  #define MISO_PCR    0

  #define MISO_PCIE   MISO_MAKE_REG(PCIE,MISO_PCR)
  #define MISO_PCMSK  MISO_MAKE_REG(PCMSK,MISO_PCR)
  #define MISO_PCIF   MISO_MAKE_REG(PCIF,MISO_PCR)

  #define KNOCKBANG_RECEIVER_AVAILABLE 1

#else
  #warning Missing MISO_* definitions.  KnockBang receive functions are not available.
  #define KNOCKBANG_RECEIVER_AVAILABLE 0
#endif


#if KNOCKBANG_RECEIVER_AVAILABLE

static const uint8_t KBR_NO_FAULT       = 0;

void KnockBangReceive_begin( void );

void KnockBangReceive_end( void );

uint8_t KnockBangReceive_available( void );

int KnockBangReceive_read( void );

typedef struct
{
  uint8_t knock_continue;
  uint8_t hello_float;
  uint8_t sample_pad;
  uint8_t sample_point;
  uint8_t init_byte;
  uint8_t ring_space;
  uint8_t ring_head;
  uint8_t overrun;
}
kbr_stats_t;

bool KnockBangReceive_getStats( kbr_stats_t & stats );

uint8_t KnockBangReceive_getLastFault( void );

uint8_t KnockBangReceive_getLastDebug1( void );

#endif // KNOCKBANG_RECEIVER_AVAILABLE


#endif
