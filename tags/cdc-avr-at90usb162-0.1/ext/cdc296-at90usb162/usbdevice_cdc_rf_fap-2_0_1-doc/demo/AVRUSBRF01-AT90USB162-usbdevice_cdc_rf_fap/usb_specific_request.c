/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! user call-back functions
//! \brief
//!
//!  This file contains the user call-back functions corresponding to the
//!  application:
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


//_____ I N C L U D E S ____________________________________________________

#include "config.h"
#include "conf_usb.h"
#include "lib_mcu/usb/usb_drv.h"
#include "usb_descriptors.h"
#include "modules/usb/device_chap9/usb_standard_request.h"
#include "usb_specific_request.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N ________________________________________________

//_____ P R I V A T E   D E C L A R A T I O N ______________________________

#ifdef __GNUC__                          // AVRGCC does not support point to PGM space
        extern PGM_VOID_P pbuffer;
#else
        extern U8   code *pbuffer;
#endif
        
extern U8   data_to_transfer;
extern U8   initial_data_to_transfer;
extern S_line_coding   line_coding;


//_____ D E C L A R A T I O N ______________________________________________

//! @breif This function checks the specific request and if known then processes it
//!
//! @param type      corresponding at bmRequestType (see USB specification)
//! @param request   corresponding at bRequest (see USB specification)
//!
//! @return TRUE,  when the request is processed
//! @return FALSE, if the request is'nt know (STALL handshake is managed by the main standard request function).
//!
Bool usb_user_read_request(U8 type, U8 request)
{
   U16 wValue;

   LSB(wValue) = Usb_read_byte();
   MSB(wValue) = Usb_read_byte();

   if( USB_SETUP_SET_CLASS_INTER == type )
   {
      switch( request )
      {
         case SETUP_CDC_SET_LINE_CODING:
         cdc_set_line_coding();
         return TRUE;
         break;
   
         case SETUP_CDC_SET_CONTROL_LINE_STATE:
         cdc_set_control_line_state(); // according cdc spec 1.1 chapter 6.2.14
         return TRUE;
         break;
   
         case SETUP_CDC_SEND_BREAK:
         cdc_send_break(wValue);             // wValue contains break lenght
         return TRUE;
         break;
      }
   }
   if( USB_SETUP_GET_CLASS_INTER == type )
   {
      switch( request )
      {
         case SETUP_CDC_GET_LINE_CODING:
         cdc_get_line_coding();
         return TRUE;
         break;
      }
   }
   return FALSE;  // No supported request
}


//! This function fills the global descriptor
//!
//! @param type      corresponding at MSB of wValue (see USB specification)
//! @param string    corresponding at LSB of wValue (see USB specification)
//!
//! @return FALSE, if the global descriptor no filled
//!
Bool usb_user_get_descriptor(U8 type, U8 string)
{ 
     switch(type)
   {
      case DESCRIPTOR_STRING:
      switch (string)
       {
              case LANG_ID:
              data_to_transfer = sizeof (usb_user_language_id);
              pbuffer = &(usb_user_language_id.bLength);
              return TRUE;
              break;
              case MAN_INDEX:
              data_to_transfer = sizeof (usb_user_manufacturer_string_descriptor);
              pbuffer = &(usb_user_manufacturer_string_descriptor.bLength);
              return TRUE;
              break;
              case PROD_INDEX:
              data_to_transfer = sizeof (usb_user_product_string_descriptor);
              pbuffer = &(usb_user_product_string_descriptor.bLength);
              return TRUE;
              break;
              case SN_INDEX:
                
#if (USE_UNIQUE_SIGNATURE==ENABLE)
              f_get_serial_string=TRUE;
              data_to_transfer = sizeof (usb_user_serial_number)+(UNIQUE_SIGNATURE_LENGTH*4);
              initial_data_to_transfer=data_to_transfer;
              pbuffer = &(usb_user_serial_number.bLength);
#else
              data_to_transfer = sizeof (usb_user_serial_number);
              pbuffer = &(usb_user_serial_number.bLength);
#endif
              return TRUE;
              break;
              default:
              return FALSE;
       }
      default:
      return FALSE;
   }
}


//! @brief This function configures the endpoints
//!
//! @param conf_nb configuration number choosed by USB host
//!
void usb_user_endpoint_init(U8 conf_nb)
{
  usb_configure_endpoint(INT_EP,      \
                         TYPE_INTERRUPT,     \
                         DIRECTION_IN,  \
                         SIZE_32,       \
                         ONE_BANK,     \
                         NYET_ENABLED);

  usb_configure_endpoint(TX_EP,      \
                         TYPE_BULK,  \
                         DIRECTION_OUT,  \
                         SIZE_32,     \
                         ONE_BANK,     \
                         NYET_ENABLED);

  usb_configure_endpoint(RX_EP,      \
                         TYPE_BULK,     \
                         DIRECTION_IN,  \
                         SIZE_32,       \
                         ONE_BANK,     \
                         NYET_ENABLED);

  Usb_reset_endpoint(INT_EP);
  Usb_reset_endpoint(TX_EP);
  Usb_reset_endpoint(RX_EP);


}

//! cdc_get_line_coding.
//!
//! @brief This function manages reception of line coding parameters (baudrate...).
//!
//! @param none
//!
//! @return none
//!
void cdc_get_line_coding(void)
{
     Usb_ack_receive_setup();
     Usb_write_byte(LSB0(line_coding.dwDTERate));
     Usb_write_byte(LSB1(line_coding.dwDTERate));
     Usb_write_byte(LSB2(line_coding.dwDTERate));
     Usb_write_byte(LSB3(line_coding.dwDTERate));
     Usb_write_byte(line_coding.bCharFormat);
     Usb_write_byte(line_coding.bParityType);
     Usb_write_byte(line_coding.bDataBits);

     Usb_send_control_in();
     while(!(Is_usb_read_control_enabled()));
     //Usb_clear_tx_complete();

   while(!Is_usb_receive_out());
   Usb_ack_receive_out();
}


//! cdc_set_line_coding.
//!
//! @brief This function manages reception of line coding parameters (baudrate...).
//!
//! @param none
//!
//! @return none
//!
void cdc_set_line_coding (void)
{
   Usb_ack_receive_setup();
   while (!(Is_usb_receive_out()));
   LSB0(line_coding.dwDTERate) = Usb_read_byte();
   LSB1(line_coding.dwDTERate) = Usb_read_byte();
   LSB2(line_coding.dwDTERate) = Usb_read_byte();
   LSB3(line_coding.dwDTERate) = Usb_read_byte();
   line_coding.bCharFormat = Usb_read_byte();
   line_coding.bParityType = Usb_read_byte();
   line_coding.bDataBits = Usb_read_byte();
     Usb_ack_receive_out();

     Usb_send_control_in();                // send a ZLP for STATUS phase
     while(!(Is_usb_read_control_enabled()));
}


//! @brief This function manages the SET_CONTROL_LINE_LINE_STATE CDC request.
//!
//! @todo Manages here hardware flow control...
//!
void cdc_set_control_line_state (void)
{
   Usb_ack_receive_setup();
    Usb_send_control_in();
   while(!(Is_usb_read_control_enabled()));
}


//! @brief This function manages the SEND_BREAK CDC request.
//!
//! @todo Manages here hardware flow control...
//!
//! @param break lenght
//!
void cdc_send_break(U16 break_duration)
{
   Usb_ack_receive_setup();
   Usb_send_control_in();
   while(!(Is_usb_read_control_enabled()));
}
