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

#ifndef TinyISP_BuildOptions_h
#define TinyISP_BuildOptions_h


/*----------------------------------------------------------------------------*/
/* 
  Take a guess at the board based on the processor.
*/

#if defined( __AVR_ATmega328__ ) || defined( __AVR_ATmega328P__ )

#define BOARD_ARDUINO_UNO                 1
#define __AVR_ATmega328X__                1

#elif defined( __AVR_AT90USB646__ )
// fix? Check for CORE_TEENSY ?

#define BOARD_TEENSY                      1
#define BOARD_TEENSY_1_PLUS               1

#elif defined( __AVR_AT90USB162__ )
// fix? Check for CORE_TEENSY ?

#define BOARD_TEENSY                      1
#define BOARD_TEENSY_1                    1

#elif defined( __AVR_ATmega32U4__ )
// fix? Check for CORE_TEENSY ?

#define BOARD_TEENSY                      1
#define BOARD_TEENSY_2                    1

#else

#error No definition for the selected board.

#endif


/*----------------------------------------------------------------------------*/
/* 
  Baud rate for communications between the host computer and this "programmer".
  19200 makes this sketch a drop-in replacement for ArduinoISP.
*/

// #define PROGRAMMER_BAUD_RATE              250000
#define PROGRAMMER_BAUD_RATE              19200


/*----------------------------------------------------------------------------*/
/*
  LED provides feedback for the human.  The states are...
  
  Heartbeat - LED slowly fades brighter then darker.  Programmer is running.
  
  Error - LED blinks solid once a second.  Currently only STK500 protocol 
      problems are checked so an error indicates a problem between the host
      and the programmer.

  Flash - LED flashes full brightness and then slowly fades darker.  
      Programmer is communicating with the target.

  LED on pin 9 makes this sketch a drop-in-replacement for ArduinoISP but may 
  interfere with other optional features.
*/

#if BOARD_ARDUINO_UNO

// #define LED_PIN                           5
#define LED_PIN                           9

#elif BOARD_TEENSY_1 || BOARD_TEENSY_1_PLUS

#define LED_PIN                           0

#elif BOARD_TEENSY_2

#define LED_PIN                           5

#endif


/*----------------------------------------------------------------------------*/
/*
  SCK rules from the datasheet...

  Low:  > 2 CPU clock cycles for fck < 12 MHz, 3 CPU clock cycles for fck >= 12 MHz
  High: > 2 CPU clock cycles for fck < 12 MHz, 3 CPU clock cycles for fck >= 12 MHz

  fck      CPU time               SCK frequency             Option
  -------  ---------------------  ------------------------  ------
    8 MHz  4 / 1000000 = 0.5 us   1 / 0.5 us =       2 MHz  FAST
    1 MHz  4 / 1000000 = 4 us     1 / 4 us =       250 KHz  NORMAL
    1 MHz  4 / 1000000 = 4 us     (1 / 4 us) / 2 = 125 KHz  SAFE
  128 KHz  4 / 128000 = 31.25 us  1 / 31.25 us =    32 KHz  SLOW

  Note: fck is the clock speed of the target.
*/

#define SLOW                              4
#define SAFE                              3
#define NORMAL                            2
#define FAST                              1

#define PROGRAMMER_SPI_CLOCK              SAFE


/*----------------------------------------------------------------------------*/
/*
  Knock-Bang Relay and Serial Relay options.
*/

#define RELAY_KNOCK_BANG_ENABLED          0

/* 
  If this option is enabled for a board that has only one Hardware Serial port,
  "#include <SoftwareSerial.h>" must be present in the sketch.  If this option
  is not enabled, "#include <SoftwareSerial.h>" cannot be present in the 
  sketch.
*/
#define RELAY_SERIAL_ENABLED              0
#define RELAY_BAUD_RATE                   9600


/*----------------------------------------------------------------------------*/
/*
  By default should the target be held in reset after programming?
*/

#define HOLD_TARGET_IN_RESET_BY_DEFAULT   0


/*----------------------------------------------------------------------------*/
/*
  Output a "tuning signal"?  The tuning signal is a 2 millisecond high pulse.
  For OSCCAL tuning the low part can be any duration.  For "long duration" 
  tuning the low part must be (and is) 48 microseconds.
*/

#define TUNING_SIGNAL_ENABLED             0
#define TUNING_SIGNAL_OUTPUT_LONG_PULSE   1


/*----------------------------------------------------------------------------*/
/*
  If TICK_TOCK_ENABLED is true, TinyISP provides a simple stopwatch.  When the
  pin is toggled by the target, the pin value and millis() are output.
*/

#define TICK_TOCK_ENABLED                 0

#if BOARD_ARDUINO_UNO

#define TICK_TOCK_PIN                     8

#elif BOARD_TEENSY_1_PLUS

#define TICK_TOCK_PIN                     17

#elif BOARD_TEENSY_2

#define TICK_TOCK_PIN                     10

#endif


/*----------------------------------------------------------------------------*/
/*
  Aggregate options based on the individual options selected above
*/

#if (RELAY_SERIAL_ENABLED) && (BOARD_ARDUINO_UNO)
  #define RELAY_SOFT_SERIAL_ENABLED  1
#endif

#if (RELAY_SERIAL_ENABLED) && (BOARD_TEENSY)
  #define RELAY_UART_SERIAL_ENABLED  1
#endif


#endif
