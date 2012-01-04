/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! This file provides a minimal funtion set for the SPI
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

/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"
//#include "lib_mem/spi\spi_lib.h"
#include "lib_mcu/spi/spi_lib.h"


/*_____ G L O B A L    D E F I N I T I O N _________________________________*/

/*_____ D E F I N I T I O N ________________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

bit spi_test_hit (void)
{
return Spi_rx_ready();
}


bit spi_init (spi_cf_t config)
{
   Spi_init_bus();
   if(config == SPI_MASTER){Spi_select_master_mode();}
   else                    {Spi_select_slave_mode();}

   Spi_hw_init(SPI_CONFIG);
   Spi_set_doublespeed();/*to delete if wished*/
   Spi_enable();
   return TRUE;
}


char spi_putchar (char ch)
{
   Spi_send_byte(ch);
   while(!Spi_tx_ready());
   return ch;
}



char spi_getchar (void)
{

   register char c;

   while(!Spi_rx_ready());
   c = Spi_get_byte();
   return c;
}

void  SPI_Transmit_Master(char cData)
{
     /* Wait for transmission complete */
   Spi_wait_eot();
     /* Start new transmission */
   Spi_send_byte(cData);

}

char spi_rw(char tx)
{
   Spi_send_byte(tx);
   return Spi_read_data();
}



