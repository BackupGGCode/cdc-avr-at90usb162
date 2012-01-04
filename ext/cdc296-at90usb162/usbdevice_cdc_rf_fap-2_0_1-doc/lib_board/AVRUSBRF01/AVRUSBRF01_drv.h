/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! AT90USBnRF REFERENCE DESIGN
//! \brief
//! RF transceiver nrf24L01 from Nordic
//! Driver file for basic peripherals
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

// LEDs
// ****
#define   Leds_init()          (DDRD |= 0x03, PORTD &= ~0x03)

#define   Led0_on()            (PORTD |= 0x01)                  // PD0
#define   Led0_off()           (PORTD &= ~0x01)
#define   Led0_toggle()        (PORTD ^= 0x01)

#define   Led1_on()            (PORTD |= 0x02)                  // PD1
#define   Led1_off()           (PORTD &= ~0x02)
#define   Led1_toggle()        (PORTD ^= 0x02)

#define   Leds_set_val(c)      (Leds_off(),PORTD |= (c&0x03)+((c&0x0C)<<2))
#define   Leds_get_val()       ((PORTD&0x30)>>2+(PORTD&0x03))
#define   Leds_off()           (PORTD &= ~0x03)

// PUSHBUTTONS
// ***********
#define   Hwb_button_init()         (DDRD &= ~0x80)
#define   Is_hwb()                  (((PIND&0x80) == 0) ? TRUE : FALSE)   // PD7

// ==================
// RF_Module
// ==================
// board handles Nordic nRF24L01 :
//    -> CE is connecter to PC2
//    -> CS is connected to PB4
//    -> IRQ is connected to PD4
#define   NRF_port_init()        (DDRC |= 0x04, DDRB |= 0x10, DDRD &= 0xFB, \
                                  PORTC &= ~0x04, PORTB |= 0x10, PORTD |= 0x10,\
                                  DDRB |= 0x01)

#define   NRF_CS_PORT           PORTB   // port
#define   NRF_CS_PIN            0x04    // offset

#define   NRF_CE_PORT           PORTC   // port
#define   NRF_CE_PIN            0x02    // offset

#define   NRF_IRQ_PORT          PIND   // port
#define   NRF_IRQ_PIN           0x04    // offset



