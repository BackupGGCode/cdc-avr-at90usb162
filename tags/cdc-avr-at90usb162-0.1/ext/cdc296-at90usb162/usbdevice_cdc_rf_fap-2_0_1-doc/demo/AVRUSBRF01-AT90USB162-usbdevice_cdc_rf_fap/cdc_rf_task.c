/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! This file manages the CDC task.
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

//_____  I N C L U D E S ___________________________________________________

#include "config.h"
#include "conf_usb.h"
#include "cdc_rf_task.h"
#include "lib_mcu\usb\usb_drv.h"
#include "usb_descriptors.h"
#include "modules\usb\device_chap9\usb_standard_request.h"
#include "usb_specific_request.h"
#include "lib_mcu/spi/spi_lib.h"
#include "lib_board/nrf24l01/nRF_API.h"
#include "FAPtoNRF_API.h"
#include "fap.h"
//#include <stdio.h>



//_____ M A C R O S ________________________________________________________





//_____ D E F I N I T I O N S ______________________________________________



//_____ D E C L A R A T I O N S ____________________________________________


volatile U8 cpt_sof;
volatile U8 led_tx_flash=0;
volatile U8 led_rx_flash=0;
volatile U16 cpt_sof_led;

         S_line_coding line_coding;
         
         fap_tx_rx_struct_t buf;
         fap_tx_rx_struct_t* buffer=&buf;
         

           
         U8 tx_counter;
         U8 rx_counter;
         
         U8 fap_Channels[]=FAP_CHANNELS;
         U8 wdp_pairing_address[]=WDP_PAIRING_ADDRESS;
         
         
         U8 HWB_HOLD=0;        
         
         
#define Flash_tx_led()  (led_tx_flash=0,Led1_on())
#define Flash_rx_led()  (led_rx_flash=0,Led0_on())
         

//! @brief This function initializes the hardware ressources required for CDC demo.
//!
void cdc_rf_task_init(void)
{
   Leds_init();
   Hwb_button_init();
     
   //Init nrf24L01 driver
   NRF_port_init();
   spi_init(SPI_MASTER);   

   tx_counter = 0;
   rx_counter = 0;
   Usb_enable_sof_interrupt();
   
   
   // hardware dependant, see fap_macros.h
   FAP_INIT_TIMER();
   FAP_NRF_INIT_IRQ();

   
   //Init FAP protocol             
   fap_init();                       // Frequency Agility Protocol Initialisation
   fap_select_radio_idle_mode(FAP_STANDBY_IDLE);  // Radio in IDLE between transmissions for minimum latency
   fap_goto_idle_mode();
   fap_set_channels(fap_Channels);    // Put the frequency channels table into fap... 
   fap_ch_sync_enable();              // Enables the frequency channels synchroniasation
   fap_rx_data(0x01, 0);              // Put fap into air monitoring (pipe 0 and low latency, infinite timeout)
   
}




//! @brief Entry point of the uart cdc management
//!
//! This function links the uart and the USB bus.
//!
void cdc_rf_task(void)
{
   
   if(Is_device_enumerated()) // Enumeration processs OK ?
   {

      
      // Receive
      if (fap_read_rx_fifo(buffer))             // Something in receive fifo ?
      { 
        
        usb_send_buffer(buffer->pl,buffer->pl_length);
        
        Flash_rx_led();                         // Flashes RX led
      }
      
      


      //transmit
      if(usb_test_hit())
        {
            
          usb_read_buffer(buffer->pl,&buffer->pl_length);

          if (fap_tx_data(buffer,100))           // Send data to tx fifo, with a 100 period timeout
          {     
          while(fap_get_mode()!=FAP_IDLE);
          fap_rx_data(0x01, 0);                  // Put fap into air monitoring again (pipe 0 and low latency, infinite timeout)           
          Flash_tx_led();                    // Flashes TX led
          }
            
        }
         

      
      
      
      if (Is_hwb())                                     //if HWB button pressed
      {
         if (!HWB_HOLD)                                 //if HWB button was not pressed before (once operation)
         {
         //Re-Init FAP protocol (to resynchronize the dongles)            
         fap_init();                                    // Frequency Agility Protocol Initialisation
         fap_select_radio_idle_mode(FAP_STANDBY_IDLE);  // Radio in IDLE between transmissions for minimum latency
         fap_goto_idle_mode();    
         fap_set_channels(fap_Channels);                // Put the frequency channels table into fap... 
         fap_ch_sync_enable();                          // Enables the frequency channels synchroniasation
         buffer->pl_length=1;        
         fap_tx_data(buffer,100);                       //forces the transfer of at least one character (to make sure fap is working)       
         fap_rx_data(0x01, 0);                          // Put fap into air monitoring (pipe 0 and low latency, infinite timeout)
         Flash_rx_led(); 
         Flash_tx_led();
         HWB_HOLD=1;
         }
      }
      else HWB_HOLD=0;
          
 
      
   }
}






//! @brief timer interrupt for the Fap timer isr routine.
#ifdef __GNUC__
 ISR(TIMER0_COMPA_vect)
#else
#pragma vector = TIMER0_COMPA_vect
__interrupt void fap_timer_interrupt()
#endif
{
  fap_timer_isr_function();             //fap isr routine
}



//! @brief NRF interrupt for the Fap NRF isr routine.
#ifdef __GNUC__
 ISR(INT5_vect)
#else
#pragma vector = INT5_vect
__interrupt void fap_nrf_interrupt()
#endif
{
  fap_nrf_isr_function();             // fap isr function called by IRQ 
}






//! @brief sof_action
//!
//! USB Start Of Frame Interrupt user subroutine, executed each 1ms
//!
void sof_action()
{

   cpt_sof++;
   if(led_tx_flash<=LED_FLASH_DELAY)
   {
     led_tx_flash++;
   }
   if(led_tx_flash==LED_FLASH_DELAY)
   {
      Led1_off();     
   }
   
   if(led_rx_flash<=LED_FLASH_DELAY)
   {
     led_rx_flash++;
   }
   if(led_rx_flash==LED_FLASH_DELAY)
   {
      Led0_off();     
   }
   

   if(cpt_sof_led++>=RF_LEDS_OFF_DELAY+RF_LEDS_FLASH_DELAY+1) cpt_sof_led=0;
   if(cpt_sof_led==RF_LEDS_OFF_DELAY)
   {
       Led0_on();
       Led1_on();
   }
   if(cpt_sof_led==RF_LEDS_OFF_DELAY+RF_LEDS_FLASH_DELAY)
   {
       Led0_off();
       Led1_off();
   }
 
}


//! @brief  This function checks if a character has been received on the USB bus.
//!
//! @return  The function returns a TRUE bit if a byte is ready to be read.
//!
//!
bit usb_test_hit(void)
{
  if (!rx_counter)
  {
    Usb_select_endpoint(TX_EP);
    if (Is_usb_receive_out())
    {
      rx_counter = Usb_byte_counter();
      if (!rx_counter)
      {
        Usb_ack_receive_out();
      }
    }
  }
  return (rx_counter!=0);
}

//! @brief  This function reads one byte from the USB bus.
//!
//! If one byte is present in the USB fifo, this byte is returned. If no data
//! is present in the USB fifo, this function waits for USB data.
//!
//! @return  The function returns the received byte.
//!
char usb_getchar(void)
{
  register Uchar data_rx;

  Usb_select_endpoint(TX_EP);
  if (!rx_counter) while (!usb_test_hit());
  data_rx=Usb_read_byte();
  rx_counter--;
  if (!rx_counter) Usb_ack_receive_out();
  return data_rx;
}


//! @brief  This function checks if the USB emission buffer is ready to accept at
//! at least 1 byte.
//! 
//! @return  The function returns TRUE if the firmware can write a new byte to transmit.
//!
bit usb_tx_ready(void)
{
  if (!Is_usb_write_enabled())
  {
    return FALSE;
  }
  return TRUE;
}



/** 
  * @brief This function transmits a ram buffer content to the USB.
  * This function is mode efficient in term of USB bandwith transfer.
  * 
  * @param U8 *buffer : the pointer to the RAM buffer to be sent 
  * @param data_to_send : the number of data to be sent
  */
void usb_send_buffer(U8 *buffer, U8 nb_data)
{
   U8 zlp;
   
   // Compute if zlp required
   if(nb_data%RX_EP_SIZE) 
   { zlp=FALSE;} 
   else { zlp=TRUE; }
   
   Usb_select_endpoint(RX_EP);
   while (nb_data)
   {
      while(Is_usb_write_enabled()==FALSE); // Wait Endpoint ready
      while(Is_usb_write_enabled() && nb_data)
      {
         Usb_write_byte(*buffer);
         buffer++;
         nb_data--;
      }
      Usb_ack_in_ready();
   }
   if(zlp)
   {
      while(Is_usb_write_enabled()==FALSE); // Wait Endpoint ready
      Usb_ack_in_ready();
   }
}



/** 
  * @brief This function receives data from the USB and fills a ram buffer.
  * This function is more efficient in term of USB bandwith transfer.
  * 
  * @param U8 *buffer : the pointer to the RAM buffer to be filled 
  * @param U8 nb_data : the number of received data bytes
  */
void usb_read_buffer(U8 *buffer, U8 *nb_data)
{
   
   *nb_data = rx_counter;                      //store the number of received data bytes
   
   Usb_select_endpoint(TX_EP); 
   
   while (rx_counter)
   {
    
    *buffer=Usb_read_byte();
    buffer++;
    rx_counter--;   
    
   } 
   
   Usb_ack_receive_out(); 
   
}