/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! This file makes the connexion between the Frequency agility Protocol
//! and the NRF driver API.
//!
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

#include "FAPtoNRF_API.h"
#include "config.h"
#include "lib_board/nrf24l01/nRF_API.h"                      //includes the NRF driver API
#include "lib_MCU/compiler.h"                                //includes the compiler definitions
#include "lib_mcu/spi/spi_lib.h"
#include "fap_setup.h"



//---------------------------Other FAPtoNRF_API functions------------------------------


//! @brief This function reads the received data payload of the 24l01 module.
//!
//! @param buf is a pointer to the location were you want to store the received data
//!
//! @return the function returns a byte composed of the receiving pipe(MSB) and the data length (LSB) 
//!
U16 hal_nrf_read_rx_pload(U8 *buf)
{
   U8 length;
   U8 pipe;

   pipe = (NRF_RW_Reg(NOP,0) & 0x0E) >> 1;         //Reads the number of the pipe wich received data from status word.
        
   if (pipe <7)
   {
     length = (NRF_Read(RD_RX_PLOAD_W));      //Reads the length of the data received.
   }
   else
   {
     length = 0;
   }

        NRF_rx_buffer(buf,length);                      //Reads the data payload
        
        return (length | (pipe<<8));
}



//! @brief This function reads the address of a data pipe
//!
//! @param pipe is the number of the pipe
//! @param addr is the read address.
//!
//! @return the function returns the length of the address. 
//!
U8 hal_nrf_get_address(U8 pipe, U8 *addr)
{
  switch(pipe)
  {
    case HAL_NRF_PIPE0:
    case HAL_NRF_PIPE1:
    case HAL_NRF_TX:
      return NRF_read_buf(pipe, addr,5);     //5 bytes is the length of the address

    default:
      *addr = NFR_read_reg((RX_ADDR_P0 + pipe));
      return hal_nrf_get_address_width();
  }
}


//! @brief This function writes the address of the a pipe
//!
//! @param pipe is the number of the pipe
//! @param addr is the address to write.
//!
void hal_nrf_set_address(U8 pipe, U8 *addr)
{
  switch(pipe)
  {
    case HAL_NRF_TX:
    case HAL_NRF_PIPE0:
    case HAL_NRF_PIPE1:
      NRF_write_buf((U8) pipe, addr, 0);
      break;

    case HAL_NRF_PIPE2:
    case HAL_NRF_PIPE3:
    case HAL_NRF_PIPE4:
    case HAL_NRF_PIPE5:
      NFR_write_reg((RX_ADDR_P0 + ((U8) pipe)), *addr); 
      break;

    default:
      break;
  }
}




//! @brief This function opens the selected communication pipe (0-5)
//!
//! @param pipe_num contains the number of the pipe to open
//! @param auto_ack : put 1 if you want Auto-Acknoledgment, 0 otherwise
//!
void NRF_open_pipe(U8 pipe_num, U8 auto_ack)
{
  switch(pipe_num)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      NFR_write_reg(EN_RXADDR,NFR_read_reg(EN_RXADDR) | (1<<pipe_num));

      if(auto_ack)
        NFR_write_reg(EN_AA, NFR_read_reg(EN_AA) | (1<<pipe_num));
      else
        NFR_write_reg(EN_AA, NFR_read_reg(EN_AA) & (0<<pipe_num));
      break;

    case 0xFF: //all pipes
      NFR_write_reg(EN_RXADDR, 0x3F);

      if(auto_ack)
        NFR_write_reg(EN_AA, 0x3F);
      else
        NFR_write_reg(EN_AA, 0);
      break;
      
    default:
      break;
  }
}



//! @brief This function closes the selected communication pipe (0-5)
//!
//! @param pipe_num contains the number of the pipe to close
//!
void NRF_close_pipe(U8 pipe_num)
{
  switch(pipe_num)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      NFR_write_reg(EN_RXADDR, NFR_read_reg(EN_RXADDR) & (0<<pipe_num));
      NFR_write_reg(EN_AA, NFR_read_reg(EN_AA) & (0<<pipe_num));
      break;
    
    case 0xFF:  //all pipes
      NFR_write_reg(EN_RXADDR, 0);
      NFR_write_reg(EN_AA, 0);
      break;
      
    default:
      break;
  }
}


//! @brief This function writes the acknoledge payload
//!
//! @param pipe is the number of the pipe wich will send acknoledge
//! @param tx_pload is a pointer to the location of the ack payload
//! @param length is the length of the ack payload
//!
void hal_nrf_write_ack_pload(U8 pipe, U8 *tx_pload, U8 length)
{
  NRF_select();

  spi_rw(WR_ACK_PLOAD | pipe);
  while(length--)
  {
    spi_rw(*tx_pload++);
  }

  NRF_unselect();
}


//! @brief This function unlocks the 24l01 tranceiver
//!
void hal_nrf_lock_unlock()
{
  NRF_select();

  spi_rw(0x50);             
  spi_rw(0x73);

  NRF_unselect();
}





//! @brief This function modify the fap timer period (only used by fap)
//!
void fap_modify_timer_period(void)
{
  OCR0A = FAP_TIMER_MODIFY_PERIOD / 4;           // /4 because TIMER0 has 4µs time unit.
}
