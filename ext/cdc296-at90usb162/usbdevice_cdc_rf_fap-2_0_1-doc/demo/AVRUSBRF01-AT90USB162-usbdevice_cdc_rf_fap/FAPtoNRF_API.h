/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! This file contains the function declarations
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


#include "lib_board/nrf24l01/nRF_API.h"                      //includes the NRF driver API
#include "lib_MCU/compiler.h"                                //includes the compiler definitions
#include "lib_mcu/spi/spi_lib.h"

//----Definition of variables formats----

#define uint8_t      U8
#define uint16_t     U16  
#define int8_t       S8
#define int16_t      S16
#define hal_nrf_output_power_t  U8

//-----------------Definition of constants-------------------------

#define HAL_NRF_CRC_OFF      0   /**< Disable CRC check          */
#define HAL_NRF_CRC_8BIT     2   /**< Enable 8 Bit CRC check     */
#define HAL_NRF_CRC_16BIT    3   /**< Enable 16 Bit CRC check    */  
#define HAL_NRF_1MBPS        0   /**< Set baud rate to 1M bit/s  */
#define HAL_NRF_2MBPS        1   /**< Set baud rate to 2M bit/s  */
#define HAL_NRF_18DBM        0   /**< Output power set to -18dBm */
#define HAL_NRF_12DBM        1   /**< Output power set to -12dBm */
#define HAL_NRF_6DBM         2   /**< Output power set to -6dBm  */
#define HAL_NRF_0DBM         3   /**< Output power set to 0dBm   */
#define HAL_NRF_PWR_DOWN     0   /**< tranceiver power down      */
#define HAL_NRF_PWR_UP       1   /**< tranceiver power up        */
#define HAL_NRF_PIPE0        0   /**< select pipe 0              */
#define HAL_NRF_PIPE1        1   /**< select pipe 1              */
#define HAL_NRF_PIPE2        2   /**< select pipe 2              */
#define HAL_NRF_PIPE3        3   /**< select pipe 3              */
#define HAL_NRF_PIPE4        4   /**< select pipe 4              */
#define HAL_NRF_PIPE5        5   /**< select pipe 5              */
#define HAL_NRF_TX           6   /**< Refer to TX address        */
#define HAL_NRF_ALL       0xFF   /**< select all pipes           */
#define HAL_NRF_PTX          0   /**< transmitter mode           */ 
#define HAL_NRF_PRX          1   /**< receiver mode              */ 
#define RD_RX_PLOAD_W     0x60  
#define WR_TX_PLOAD       0xA0  
#define WR_ACK_PLOAD      0xA8  
#define WR_NAC_TX_PLOAD   0xB0  
#define DYNPD             0x1C  /**< 'Enable dynamic payload length' register address */ 
#define FEATURE           0x1D  /**<'FEATURE' register address   */ 


// Status bits definition
#define NRF_RX_DR            6  /**< RX data ready               */ 
#define NRF_TX_DS            5  /**< TX data sent                */ 
#define NRF_TX_DR            5  /**< TX data ready               */ 
#define NRF_MAX_RT           4  /**< Max retransmits             */ 






//-------------- Public and usefull NRF driver access for FAP --------------------------


#define CE_LOW()                          (NRF_disable())                                                           /**< Disables the tranceiver */
#define CE_HIGH()                         (NRF_enable())                                                            /**< Enables the tranceiver  */
void hal_nrf_lock_unlock();                                                                                         /**< Unlock the tranceiver   */


#define hal_nrf_write_reg(reg,val)        (NRF_enable(),NRF_disable(),NFR_write_reg(reg,val))                       /**< Writes a value in a register       */
#define hal_nrf_read_reg(reg)             (NRF_enable(),NFR_read_reg(reg))                                          /**< Returns the value of the register  */
U16 hal_nrf_read_rx_pload(U8 *buf);                                                                                 /**< Reads the content of rx payload    */
#define hal_nrf_enable_ack_pl()           (NFR_write_reg(FEATURE, (NFR_read_reg(FEATURE) | 0x02)))                  /**< Enables the payload acknoledgment  */
void hal_nrf_write_ack_pload(U8 pipe, U8 *tx_pload, U8 length);                                                     /**< Writes the payload acknoledgment   */
#define hal_nrf_write_tx_pload(buf,n)     (NRF_tx_buffer(buf,n))                                                    /**< Writes the contents of tx payload  */
#define hal_nrf_flush_rx()                (NRF_flush_rx())                                                          /**< flushs the rx buffer               */
#define hal_nrf_flush_tx()                (NFR_write_reg(FLUSH_TX,0))                                               /**< flushs the tx buffer               */


#define hal_nrf_set_rf_channel(n)         (NRF_set_channel(n))                                                      /**< Sets the frequency channel (0 to 123)                  */
#define hal_nrf_set_output_power(power)   (NFR_write_reg(RF_SETUP, ((NFR_read_reg(RF_SETUP) & 0xF9) | (power<<1)))) /**< Sets the output signal power (0 to 3)                  */
#define hal_nrf_set_power_mode(en)        (NFR_write_reg(CONFIG, ((NFR_read_reg(CONFIG) & 0xFD) | (en<<1))))        /**< Wake(1) or Shutdown(0) the tranceiver                  */
#define hal_nrf_set_operation_mode(rtx)   (NFR_write_reg(CONFIG, ((NFR_read_reg(CONFIG) & 0xFE) | rtx)))            /**< puts the tranceiver into transmitter(0) or receiver(1) */
#define hal_nrf_set_datarate(rate)        (NFR_write_reg(RF_SETUP,(NFR_read_reg(RF_SETUP) & 0xF7) | (rate<<3)))     /**< Sets the transmission baud rate                        */
#define hal_nrf_set_crc_mode(mode)        (NFR_write_reg(CONFIG,(NFR_read_reg(CONFIG) & 0xF3) | (mode<<2)))         /**< Sets the CRC Mode (0:OFF, 1: 8 Bit, 2: 16 Bit)         */


#define hal_nrf_open_pipe(pipe, AutoAck)  (NRF_open_pipe(pipe, AutoAck))                                            /**< Open selected pipe (pipe: 0~5, AutoAck: 0/1) */
#define hal_nrf_close_pipe(pipe)          (NRF_close_pipe(pipe))                                                    /**< Close selected pipe (pipe: 0~5)              */
#define hal_nrf_set_address_width(width)  (NFR_write_reg(SETUP_AW, (width - 2)))
#define hal_nrf_get_address_width()       (NFR_read_reg(SETUP_AW)+ 2)
#define hal_nrf_get_transmit_attempts()   (NFR_read_reg(OBSERVE_TX) & 0x0F)                                         /**< Returns the number of transmit attemps       */
U8 hal_nrf_get_address(U8 pipe, U8 *addr);                                                                          /**< Returns the address of the selected pipe     */
void hal_nrf_set_address(U8 pipe, U8 *addr);                                                                        /**< Sets the address of the selected pipe        */
void fap_modify_timer_period(void);                                                                                 /**< Modify the fap timer isr period              */
void NRF_open_pipe(U8 pipe_num, U8 auto_ack);                                                                       /**< Open pipe (pipe_num: 0~5, auto_ack: 0/1)     */
void NRF_close_pipe(U8 pipe_num);                                                                                   /**< Close pipe(pipe_num: 0~5)                    */


#define hal_nrf_rx_fifo_empty()           (NFR_read_reg(FIFO_STATUS) & 0x01)                                        /**< Returns true if rx fifo is empty             */
#define hal_nrf_tx_fifo_empty()           (NFR_read_reg(FIFO_STATUS) & 0x10)                                        /**< Returns true if tx fifo is empty             */
#define hal_nrf_rx_fifo_full()            (NFR_read_reg(FIFO_STATUS) & 0x02)                                        /**< Returns true if rx fifo is full              */
#define hal_nrf_tx_fifo_full()            (NFR_read_reg(FIFO_STATUS) & 0x20)                                        /**< Returns true if tx fifo is full              */


#define hal_nrf_get_clear_irq_flags()     (NFR_write_reg(STATUS, (0x40|0x20|0x10)) & (0x40|0x20|0x10))              /**< Returns the irq flags and clear them         */


#define hal_nrf_enable_dynamic_pl()       (NFR_write_reg(FEATURE,NFR_read_reg(FEATURE) | 0x04))                     /**< Enable Dynamic Payload Length                */
#define hal_nrf_setup_dyn_pl(n)           (NFR_write_reg(DYNPD,n))                                                  /**< Setup Dynamic Payload Length                 */
#define hal_nrf_set_auto_retr(retr,delay) (NFR_write_reg(SETUP_RETR, (((delay/250)-1)<<4) | retr))                  /**< Setup transmit auto retry with delay         */

