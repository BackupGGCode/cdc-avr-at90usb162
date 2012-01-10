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


// Low level pin definition check (hardware map)
#ifndef  NRF_CS_PORT
#error   NRF_CS_PORT should be defined somewhere (usually target_board_file.h)
#endif

#ifndef  NRF_CS_PIN
#error   NRF_CS_PIN should be defined somewhere (usually target_board_file.h)
#endif

#ifndef  NRF_CE_PORT
#error   NRF_CE_PORT should be defined somewhere (usually target_board_file.h)
#endif

#ifndef  NRF_CE_PIN
#error   NRF_CE_PIN should be defined somewhere (usually target_board_file.h)
#endif

#ifndef  NRF_IRQ_PORT
#error   NRF_IRQ_PORT should be defined somewhere (usually target_board_file.h)
#endif

#ifndef  NRF_IRQ_PIN
#error   NRF_IRQ_PIN should be defined somewhere (usually target_board_file.h)
#endif

#ifndef  NRF_port_init
#error   NRF_port_init() should be defined somewhere (usually target_board_file.h)
#endif


// Pin access
#define   NRF_select()           (NRF_CS_PORT &= ~(1<<NRF_CS_PIN))
#define   NRF_unselect()         (NRF_CS_PORT |= (1<<NRF_CS_PIN))
#define   NRF_disable()          (NRF_CE_PORT &= ~(1<<NRF_CE_PIN))
#define   NRF_enable()           (NRF_CE_PORT |= (1<<NRF_CE_PIN))


// Define nRF24L01 interrupt flag's
#define IDLE            0x00  // Idle, no interrupt pending
#define MAX_RT          0x10  // Max #of TX retrans interrupt
#define TX_DS           0x20  // TX data sent interrupt
#define RX_DR           0x40  // RX data received

#define SPI_CFG         0x40  // SPI Configuration register value
#define SPI_CTR         0x01  // SPI Control register values
#define SPI_CLK         0x00  // SYSCLK/2*(SPI_CLK+1) == > 12MHz / 2 = 6MHz
#define SPI0E           0x02  // SPI Enable in XBR0 register


// SPI(nRF24L01) commands
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NOP             0xFF  // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address

// Status bits definition
#define NRF_RX_DR   6
#define NRF_TX_DR   5
#define NRF_MAX_RT  4



// Public and usefull driver access

#define NFR_read_reg(reg)        (NRF_Read(reg|READ_REG))
#define NFR_write_reg(reg,val)   (NRF_RW_Reg(reg|WRITE_REG,val))

#define NRF_init_tx()            (NRF_disable(),NFR_write_reg(CONFIG,0x0A))  // Prim TX, CRC
#define NRF_init_rx()            (NRF_disable(),NFR_write_reg(CONFIG,0x0B),NRF_enable())  // Prim RX, CRC, PowerUp

#define Is_not_NRF_IRQ()         ((NRF_IRQ_PORT & (1<<NRF_IRQ_PIN))? TRUE : FALSE)
#define Is_NRF_IRQ()             ((NRF_IRQ_PORT & (1<<NRF_IRQ_PIN))? FALSE : TRUE)


#define NRF_tx_buffer(buf,n)     (NRF_write_buf(WR_TX_PLOAD,buf,n))
#define NRF_rx_buffer(buf,n)     (NRF_read_buf(RD_RX_PLOAD,buf,n))

#define NRF_set_payload(pipe,n)  (NFR_write_reg(RX_PW_P##pipe,n))

#define NRF_set_channel(n)       (NFR_write_reg(RF_CH,n))
#define NRF_set_byte_addr(n)     (NFR_write_reg(SETUP_AW,n-2))

#define NRF_ack_RX_DR()          (NFR_write_reg(STATUS,(1<<NRF_RX_DR)))
#define NRF_ack_TX_DR()          (NFR_write_reg(STATUS,(1<<NRF_TX_DR)))
#define NRF_ack_MAX_RT()         (NFR_write_reg(STATUS,(1<<NRF_MAX_RT)))
#define NRF_ack_received()       (NRF_ack_RX_DR())
#define NRF_ack_transmit()       (NRF_ack_TX_DR())
#define NRF_ack_timeout()        (NRF_ack_MAX_RT())

#define NRF_flush_rx()           (NFR_write_reg(FLUSH_RX,0))

#define Is_NRF_RX_DR()           (NFR_read_reg(STATUS)&(1<<NRF_RX_DR) ? TRUE : FALSE)
#define Is_NRF_TX_DR()           (NFR_read_reg(STATUS)&(1<<NRF_TX_DR) ? TRUE : FALSE)
#define Is_NRF_MAX_RT()          (NFR_read_reg(STATUS)&(1<<NRF_MAX_RT) ? TRUE : FALSE)

#define Is_NRF_received()        (Is_NRF_RX_DR())
#define Is_NRF_transmit()        (Is_NRF_TX_DR())
#define Is_NRF_timeout()         (Is_NRF_MAX_RT())

#define NRF_Write_addr(reg,buf,n)   NRF_write_buf(reg+0x20,buf, n)

//_____ D E C L A R A T I O N S ____________________________________________

 U8 SPI_RW(U8 byte);                                // Single SPI read/write
 U8 NRF_Read(U8 reg);                               // Read one byte from nRF24L01
 U8 NRF_RW_Reg(U8 reg, U8 value);                 // Write one byte to register 'reg'
 U8 NRF_read_buf(U8 reg, U8 *pBuf, U8 bytes);   // Read multiply bytes from one register
 U8 NRF_write_buf(U8 reg, U8 *pBuf, U8 bytes);


//_____ S A M P L E   U S A G E with AT90USB128 USB software library _______
#if 0
/*
//_____  I N C L U D E S ___________________________________________________

#include "config.h"
#include "conf_usb.h"
#include "rf_task.h"
#include "lib_mcu/usb/usb_drv.h"
#include "usb_descriptors.h"
#include "modules/usb/device_chap9/usb_standard_request.h"
#include "usb_specific_request.h"
#include "lib_mcu/spi/spi_lib.h"
#include "lib_board/nrf24l01/nRF_API.h"



//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

volatile U8  tmp;
U8 rx_buf[32];
U8 tx_buf[32];

//!
//! @brief This function initializes the target board ressources.
//!
//! @param none
//!
//! @return none
//!
//!/
void rf_task_init(void)
{
   Leds_init();
   NRF_port_init();
   spi_init(SPI_MASTER);
   NRF_init_rx();           // Set nRf primary receiver
   NRF_set_payload(0,8);    // Pipe 0 receiver is 8 bytes
   NRF_enable();            // Enable receiver
}





//! @brief Entry point of the HID generic communication task
//!
//! This function manages IN/OUT repport management and communication with nRF24L01 driver.
//!
//!
//! @param none
//!
//! @return none
void rf_task(void)
{
   U8 i;

   if(Is_device_enumerated())             // Check USB HID is enumerated
   {
      Usb_select_endpoint(EP_HID_OUT);    // Get Data repport from Host
      if(Is_usb_receive_out())            // Something received on the USB?
      {
         for(i=0;i<EP_OUT_LENGTH;i++)
         {
            tx_buf[i]=Usb_read_byte();
         }
         NRF_init_tx();                   // RF is Tx mode
         NRF_tx_buffer(tx_buf, 8);        // Send data to Rf
         while(Is_not_NRF_IRQ());         // Wait event from Rf
         if(Is_NRF_transmit())            // Data sent ?
         {
            NRF_ack_TX_DR();
         }
         else if (Is_NRF_timeout())
         {
            NRF_ack_MAX_RT();
         }
         NRF_init_rx();
         Usb_ack_receive_out();
      }

      if (Is_NRF_IRQ())                            // Something received on Rf ?
      {
         if(Is_NRF_received())
         {
            Led_usb_toggle();
            NRF_rx_buffer(rx_buf,8);               // Get data from Rf
            Usb_select_endpoint(EP_HID_IN);        // Ready to send these information to the host application
            while(Is_usb_write_enabled()==FALSE);  // Wait EP ready
            {
               for(i=0;i<EP_IN_LENGTH;i++)
               {
                  Usb_write_byte(rx_buf[i]);
               }
               Usb_ack_in_ready();                 // Send data over the USB
            }
            NRF_ack_RX_DR();
         }
      }
   }
}
*/
#endif
