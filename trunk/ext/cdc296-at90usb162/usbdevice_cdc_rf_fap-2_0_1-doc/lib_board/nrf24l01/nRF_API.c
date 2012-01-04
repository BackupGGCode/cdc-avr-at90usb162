/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! This file manages the NRF24L01 RF device and provide low level driver access (based on SPI driver).
//! \brief
//!
//!  Original author: Runar Kjellhaug from Nordic RefDesign with Dallas MCU
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

#include "config.h"
#include "nRF_API.h"
#include "lib_mcu/spi/spi_lib.h"


/**
 * NRF_Read
 *
 * @brief Read one byte from nRF24L01 register, 'reg'
 *
 * @param reg, register to read
 *
 * @return reg_val, register value
 */
U8 NRF_Read(U8 reg)
{
   U8 reg_val;

   NRF_select();
   spi_rw(reg);                                    // Select register to read from..
   reg_val = spi_rw(0);                            // ..then read registervalue
   NRF_unselect();                                 // CSN high, terminate SPI communication
   return(reg_val);                                // return register value
}


/**
 * NRF_RW_Reg
 *
 * @brief Writes value 'value' to register 'reg'
 *
 * @param 'reg' register to write value 'value' to
 *
 * @return status byte
 */
U8 NRF_RW_Reg(U8 reg, U8 value)
{
   U8 status;

   NRF_select();           // CSN low, init SPI transaction
   status = spi_rw(reg);   // select register
   spi_rw(value);          // ..and write value to it..
   NRF_unselect();         // CSN high again
   return(status);         // return nRF24L01 status byte
}


/**
 * NRF_write_buf
 *
 * @brief Writes contents of buffer '*pBuf' to nRF24L01, typically used to write TX payload, Rx/Tx address
 *
 * @param 'register 'reg' to write,
 * @param buffer '*pBuf*' contains data to be written
 * @param bytes: number of data byte to write
 *
 * @return nRF24L01 status byte
 */
U8 NRF_write_buf(U8 reg,U8 *pBuf, U8 bytes)
{
   U8 status,byte_ctr;
   volatile U8 tempo;

   NRF_select();                                   // Set CSN low, init SPI tranaction
   status = spi_rw(reg);                           // Select register to write to and read status byte
   for(byte_ctr=0; byte_ctr<bytes; byte_ctr++)     // then write all byte in buffer(*pBuf)
   {  
     Spi_send_byte(*pBuf++);
     Spi_read_data();
     //   spi_rw(*pBuf++);
   }

   NRF_unselect();                                 // Set CSN high again
   NRF_enable();
   for(tempo=0;tempo<FOSC/400;tempo++);     //! FIXE ME !!!!!! 10µs CE pulse required
   NRF_disable();
   return(status);                                 // return nRF24L01 status byte
}

/**
 * NRF_read_buf
 *
 * @brief Reads 'bytes' #of bytes from register 'reg', typically used to read RX payload, Rx/Tx address
 *
 * @param 'register 'reg' to read,
 * @param buffer '*pBuf*' contains data to be stored
 * @param bytes: number of data byte to write
 *
 * @return nRF24L01 status byte
 */
U8 NRF_read_buf(U8 reg, U8 *pBuf, U8 bytes)
{

   U8 status,byte_ctr;

   NRF_select();                                     // Set CSN low, init SPI tranaction
   status = spi_rw(reg);                           // Select register to write to and read status byte

   for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
   {  
      Spi_write_data(0);
      pBuf[byte_ctr]=Spi_read_data();
   }

   NRF_unselect();                                     // Set CSN high again
   return(status);                                 // return nRF24L01 status byte
}
