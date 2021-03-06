/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! This file contains the system configuration definition.
//! \brief
//!
//! - Compiler:           IAR EWAVR and GNU GCC for AVR
//! - Supported devices:  AT90USB162, AT90USB82
//!
//! \author               Atmel Corporation: http://www.atmel.com \n
//!                       Support and FAQ: http://support.atmel.no/
//!
//! ***************************************************************************

/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

// Compiler switch (do not change these settings)
#include "lib_mcu/compiler.h"             // Compiler definitions
#ifdef __GNUC__
   #include <avr/io.h>                    // Use AVR-GCC library
#elif __ICCAVR__
   #define ENABLE_BIT_DEFINITIONS
   #include <ioavr.h>                     // Use IAR-AVR library
#else
   #error Current COMPILER not supported
#endif


//! @defgroup global_config Application configuration
//! @{

#include "conf/conf_scheduler.h" //!< Scheduler tasks declaration


#include "lib_board/AVRUSBRF01/AVRUSBRF01_drv.h"


#define BYPASS_USB_AUTOBAUD
//! CPU core frequency in kHz
#define FOSC 16000


// -------- END Generic Configuration -------------------------------------

// UART Sample configuration, if we have one ... __________________________
#define BAUDRATE        38400
#define USE_UART2
#define UART_U2

//#define uart_putchar putchar
#ifndef __GNUC__
   #define uart_usb_putchar putchar
#endif
#define r_uart_ptchar int
#define p_uart_ptchar int

#define NB_MS_BEFORE_FLUSH         50 //ms
#define REPEAT_KEY_PRESSED       100

#define DUMMY GPIOR0
#define SPI_CONFIG      0

#define LED_FLASH_DELAY 20 //ms
#define RF_BUFFER_SIZE  32

#define RF_LEDS_OFF_DELAY     1000
#define RF_LEDS_FLASH_DELAY   5

//! @}

#endif // _CONFIG_H_

