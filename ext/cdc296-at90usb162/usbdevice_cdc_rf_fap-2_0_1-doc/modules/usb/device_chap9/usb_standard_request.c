/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! Process USB device enumeration requests.
//! \brief @brief
//!
//!  This file contains the USB endpoint 0 management routines corresponding to
//!  the standard enumeration process (refer to chapter 9 of the USB
//!  specification.
//!  This file calls routines of the usb_specific_request.c file for non-standard
//!  request management.
//!  The enumeration parameters (descriptor tables) are contained in the
//!  usb_descriptors.c file.
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
#include "lib_mcu/pll/pll_drv.h"
#include "usb_specific_request.h"


#if (USE_UNIQUE_SIGNATURE==ENABLE)
#include "lib_mcu/flash/flash_drv.h"
#endif
//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N ________________________________________________

//_____ P R I V A T E   D E C L A R A T I O N ______________________________

static  void    usb_get_descriptor(   void);
static  void    usb_set_address(      void);
static  void    usb_set_configuration(void);
static  void    usb_clear_feature(    void);
static  void    usb_set_feature(      void);
static  void    usb_get_status(       void);
static  void    usb_get_configuration(void);
static  void    usb_get_interface (void);
static  void    usb_set_interface (void);
U8 bin_to_ascii (U8 b);

#ifndef USB_REMOTE_WAKEUP_FEATURE
   #error USB_REMOTE_WAKEUP_FEATURE should be defined as ENABLE or DISABLE in conf_usb.h
#endif


#if (USE_UNIQUE_SIGNATURE==ENABLE)
    U8   f_get_serial_string=FALSE;
    U16 sig_index=UNIQUE_SIGNATURE_ADDRESS;
#endif


  
//_____ D E C L A R A T I O N ______________________________________________

static  bit  zlp;
static  U8   endpoint_status[MAX_EP_NB];
static  U8   device_status=DEVICE_STATUS;

#ifdef __GNUC__                          // AVRGCC does not support point to PGM space
        PGM_VOID_P pbuffer;
        #define Usb_write_PGM_byte(byte) (Usb_write_byte(pgm_read_byte_near((unsigned int)byte))) 
#else
        U8   code *pbuffer;
        #define Usb_write_PGM_byte(byte) (Usb_write_byte(*byte))
#endif
        U8   data_to_transfer;
        U8   initial_data_to_transfer;
        U16  wInterface;

static  U8   bmRequestType;
        U8      remote_wakeup_feature=DISABLE; 
        U8   usb_configuration_nb;
extern  bit     usb_connected;
extern  code    S_usb_device_descriptor             usb_user_device_descriptor;
extern  code    S_usb_user_configuration_descriptor usb_user_configuration_descriptor;

U8      usb_remote_wup_feature;  // Store ENABLED value if a SetFeature(RemoteWakeUp) has been received


//! usb_process_request.
//!
//! @brief This function reads the SETUP request sent to the default control endpoint
//! and calls the appropriate function. When exiting of the usb_read_request
//! function, the device is ready to manage the next request.
//!
//! @param none
//!
//! @return none
//! @note list of supported requests:
//! SETUP_GET_DESCRIPTOR
//! SETUP_GET_CONFIGURATION
//! SETUP_SET_ADDRESS
//! SETUP_SET_CONFIGURATION
//! SETUP_CLEAR_FEATURE
//! SETUP_SET_FEATURE
//! SETUP_GET_STATUS
//!
void usb_process_request(void)
{
   U8  bmRequest;

   Usb_ack_control_out();
   bmRequestType = Usb_read_byte();
   bmRequest     = Usb_read_byte();

   switch (bmRequest)
   {
    case SETUP_GET_DESCRIPTOR:
         if (USB_SETUP_GET_STAND_DEVICE == bmRequestType) { usb_get_descriptor(); }
         else                       { usb_user_read_request(bmRequestType, bmRequest); }
         break;

    case SETUP_GET_CONFIGURATION:
         if (USB_SETUP_GET_STAND_DEVICE == bmRequestType) { usb_get_configuration(); }
         else                       { usb_user_read_request(bmRequestType, bmRequest); }
         break;

    case SETUP_SET_ADDRESS:
         if (USB_SETUP_SET_STAND_DEVICE == bmRequestType) { usb_set_address(); }
         else                       { usb_user_read_request(bmRequestType, bmRequest); }
         break;

    case SETUP_SET_CONFIGURATION:
         if (USB_SETUP_SET_STAND_DEVICE == bmRequestType) { usb_set_configuration(); }
         else                       { usb_user_read_request(bmRequestType, bmRequest); }
         break;

    case SETUP_CLEAR_FEATURE:
         if (USB_SETUP_SET_STAND_ENDPOINT >= bmRequestType) { usb_clear_feature(); }
         else                       { usb_user_read_request(bmRequestType, bmRequest); }
         break;

    case SETUP_SET_FEATURE:
         if (USB_SETUP_SET_STAND_ENDPOINT >= bmRequestType) { usb_set_feature(); }
         else                       { usb_user_read_request(bmRequestType, bmRequest); }
         break;

    case SETUP_GET_STATUS:
         if ((0x7F < bmRequestType) & (0x82 >= bmRequestType))
                                    { usb_get_status(); }
         else                       { usb_user_read_request(bmRequestType, bmRequest); }
         break;

    case SETUP_GET_INTERFACE:
          if (bmRequestType == USB_SETUP_GET_STAND_INTERFACE) { usb_get_interface(); }
          else { usb_user_read_request(bmRequestType, bmRequest); }
          break;


    case SETUP_SET_INTERFACE:
      if (bmRequestType == USB_SETUP_SET_STAND_INTERFACE) {usb_set_interface();}
      break;

    case SETUP_SET_DESCRIPTOR:
    case SETUP_SYNCH_FRAME:
    default: //!< un-supported request => call to user read request
         if(usb_user_read_request(bmRequestType, bmRequest) == FALSE)
         {
            Usb_enable_stall_handshake();
            Usb_ack_receive_setup();
            return;
         }
         break;
  }
}


//! usb_set_address.
//!
//! This function manages the SET ADDRESS request. When complete, the device
//! will filter the requests using the new address.
//!
//! @warning Code:xx bytes (function code length)
//!
//! @param none
//!
//! @return none
//!
void usb_set_address(void)
{
   U8 addr = Usb_read_byte();
   Usb_configure_address(addr);

   Usb_ack_receive_setup();

   Usb_send_control_in();                    //!< send a ZLP for STATUS phase
   while(!Is_usb_in_ready());                //!< waits for status phase done
                                             //!< before using the new address
   Usb_enable_address();
}


//! This function manages the SET CONFIGURATION request. If the selected
//! configuration is valid, this function call the usb_user_endpoint_init()
//! function that will configure the endpoints following the configuration
//! number.
//!
void usb_set_configuration( void )
{
U8 configuration_number;

   configuration_number = Usb_read_byte();

   if (configuration_number <= NB_CONFIGURATION)
   {
      Usb_ack_receive_setup();
      usb_configuration_nb = configuration_number;
   }
   else
   {
      //!< keep that order (set StallRq/clear RxSetup) or a
      //!< OUT request following the SETUP may be acknowledged
      Usb_enable_stall_handshake();
      Usb_ack_receive_setup();
      return;
   }

   Usb_send_control_in();                    //!< send a ZLP for STATUS phase

   usb_user_endpoint_init(usb_configuration_nb);  //!< endpoint configuration
   Usb_set_configuration_action();
}


//! This function manages the GET DESCRIPTOR request. The device descriptor,
//! the configuration descriptor and the device qualifier are supported. All
//! other descriptors must be supported by the usb_user_get_descriptor
//! function.
//! Only 1 configuration is supported.
//!
void usb_get_descriptor(void)
{
U16  wLength;
U8   descriptor_type ;
U8   string_type;
U8   dummy;
U8   nb_byte;
U8   byte_to_send;

   zlp             = FALSE;                  /* no zero length packet */
   string_type     = Usb_read_byte();        /* read LSB of wValue    */
   descriptor_type = Usb_read_byte();        /* read MSB of wValue    */

   switch (descriptor_type)
   {
    case DESCRIPTOR_DEVICE:
      data_to_transfer = Usb_get_dev_desc_length(); //!< sizeof (usb_user_device_descriptor);
      pbuffer          = Usb_get_dev_desc_pointer();
      break;
    case DESCRIPTOR_CONFIGURATION:
      data_to_transfer = Usb_get_conf_desc_length(); //!< sizeof (usb_user_configuration_descriptor);
      pbuffer          = Usb_get_conf_desc_pointer();
      break;
    default:
      if( usb_user_get_descriptor(descriptor_type, string_type)==FALSE )
      {
         Usb_enable_stall_handshake();
         Usb_ack_receive_setup();
         return;
      }
      break;
   }

   dummy = Usb_read_byte();                     //!< don't care of wIndex field
   dummy = Usb_read_byte();
   LSB(wLength) = Usb_read_byte();              //!< read wLength
   MSB(wLength) = Usb_read_byte();
   Usb_ack_receive_setup() ;                  //!< clear the receive setup flag

   if (wLength > data_to_transfer)
   {
      if ((data_to_transfer % EP_CONTROL_LENGTH) == 0) { zlp = TRUE; }
      else { zlp = FALSE; }                   //!< no need of zero length packet
   }
   else
   {
      data_to_transfer = (U8)wLength;         //!< send only requested number of data
   }

   Usb_ack_nak_out();

   while((data_to_transfer != 0) && (!Is_usb_nak_out_sent()))
   {
      while(!Is_usb_read_control_enabled())
      {
        if (Is_usb_nak_out_sent())
          break;    // don't clear the flag now, it will be cleared after
      }
              
      nb_byte=0;
      byte_to_send=0;
      while(data_to_transfer != 0)        //!< Send data until necessary
      {
        
        
        
        if(nb_byte++==EP_CONTROL_LENGTH) //!< Check endpoint 0 size
        {
           break;
        }
         
         
#if (USE_UNIQUE_SIGNATURE==ENABLE)

         if(f_get_serial_string & (data_to_transfer < (initial_data_to_transfer-1)))    //if we are sending the signature characters (third byte and more...)
         {                                                                              //(The first two bytes are the length and the descriptor)
            
             switch (byte_to_send)
             {
               case 0:
                    Usb_write_byte(bin_to_ascii((flash_read_sig(sig_index)>>4) & 0x0F)); //sends the fist part (MSB) of the signature hex number, converted in ascii
                    break;
              
               case 1:
                    Usb_write_byte(0);                                                   //then, sends a null character (Usb_unicode)                    
                    break;
    
               case 2:
                    Usb_write_byte(bin_to_ascii(flash_read_sig(sig_index) & 0x0F));      //sends the second part (LSB) of the signature hex number, converted in ascii 
                    break;
    
               case 3:
                    Usb_write_byte(0);                                                   //then, sends a null character (Usb_unicode)  
                    sig_index++;                                                         //increments the signature address pointer.
                    break;
                    
             }
             byte_to_send=(byte_to_send+1)%4;     
         }
         
         else
            
         {
           Usb_write_PGM_byte(pbuffer++);                                                 //Write a flash byte to USB        
         }        
          
         
#else          
         Usb_write_PGM_byte(pbuffer++);          
#endif

         
         data_to_transfer --;                                                             //decrements the number of bytes to transmit.
      }
      
      
        
      if (Is_usb_nak_out_sent())
        break;
      else
        Usb_send_control_in();
   }
   
   
#if (USE_UNIQUE_SIGNATURE==ENABLE)      
      f_get_serial_string=FALSE;                                                   //end of signature transmission
      sig_index=UNIQUE_SIGNATURE_ADDRESS;                                          //resets the signature address pointer     
#endif   
   

   if((zlp == TRUE) && (!Is_usb_nak_out_sent()))
   {
     while(!Is_usb_read_control_enabled());
     Usb_send_control_in();
   }

   while (!(Is_usb_nak_out_sent()));
   Usb_ack_nak_out();       // clear NAKOUTI
   Usb_ack_control_out();   // clear RXOUTI



}


//! usb_get_configuration.
//!
//! This function manages the GET CONFIGURATION request. The current
//! configuration number is returned.
//!
//! @warning Code:xx bytes (function code length)
//!
//! @param none
//!
//! @return none
//!
void usb_get_configuration(void)
{
   Usb_ack_receive_setup();

   Usb_write_byte(usb_configuration_nb);
   Usb_ack_in_ready();

   while( !Is_usb_receive_out() );
   Usb_ack_receive_out();
}

//! usb_get_status.
//!
//! This function manages the GET STATUS request. The device, interface or
//! endpoint status is returned.
//!
//! @warning Code:xx bytes (function code length)
//!
//! @param none
//!
//! @return none
//!
void usb_get_status(void)
{
U8 wIndex;
U8 dummy;

   dummy    = Usb_read_byte();                 //!< dummy read
   dummy    = Usb_read_byte();                 //!< dummy read
   wIndex = Usb_read_byte();

   switch(bmRequestType)
   {
    case USB_SETUP_GET_STAND_DEVICE:    Usb_ack_receive_setup();
                                   Usb_write_byte(device_status);
                                   break;

    case USB_SETUP_GET_STAND_INTERFACE: Usb_ack_receive_setup();
                                   Usb_write_byte(INTERFACE_STATUS);
                                   break;

    case USB_SETUP_GET_STAND_ENDPOINT:  Usb_ack_receive_setup();
                                   wIndex = wIndex & MSK_EP_DIR;
                                   Usb_write_byte(endpoint_status[wIndex]);
                                   break;
    default:
                                   Usb_enable_stall_handshake();
                                   Usb_ack_receive_setup();
                                   return;
   }

   Usb_write_byte(0x00);
   Usb_send_control_in();

   while( !Is_usb_receive_out() );
   Usb_ack_receive_out();
}


//! usb_set_feature.
//!
//! This function manages the SET FEATURE request. The USB test modes are
//! supported by this function.
//!
//! @warning Code:xx bytes (function code length)
//!
//! @param none
//!
//! @return none
//!
void usb_set_feature(void)
{
U8 wValue;
U8 wIndex;
U8 dummy;

  switch (bmRequestType)
   {
    case USB_SETUP_SET_STAND_DEVICE:
      wValue = Usb_read_byte();
      switch (wValue)
      {
         case USB_REMOTE_WAKEUP:
            if ((wValue == FEATURE_DEVICE_REMOTE_WAKEUP) && (USB_REMOTE_WAKEUP_FEATURE == ENABLED))
            {
                device_status |= USB_STATUS_REMOTEWAKEUP;
                remote_wakeup_feature = ENABLED;
                Usb_ack_receive_setup();
                Usb_send_control_in();
            }
            else
            {
               Usb_enable_stall_handshake();
               Usb_ack_receive_setup();
            }
            break;
            
            
            
         default:
         Usb_enable_stall_handshake();
         Usb_ack_receive_setup();
         break;
      }
      break;

  case USB_SETUP_SET_STAND_INTERFACE:
      //!< keep that order (set StallRq/clear RxSetup) or a
      //!< OUT request following the SETUP may be acknowledged
      Usb_enable_stall_handshake();
      Usb_ack_receive_setup();
    break;

  case USB_SETUP_SET_STAND_ENDPOINT:
      wValue = Usb_read_byte();
      dummy    = Usb_read_byte();                //!< dummy read

      if (wValue == FEATURE_ENDPOINT_HALT)
      {
         wIndex = (Usb_read_byte() & MSK_EP_DIR);

         if (wIndex == EP_CONTROL)
         {
            Usb_enable_stall_handshake();
            Usb_ack_receive_setup();
         }

         Usb_select_endpoint(wIndex);
         if(Is_usb_endpoint_enabled())
         {
            Usb_enable_stall_handshake();
            Usb_select_endpoint(EP_CONTROL);
            endpoint_status[wIndex] = 0x01;
            Usb_ack_receive_setup();
            Usb_send_control_in();
         }
         else
         {
            Usb_select_endpoint(EP_CONTROL);
            Usb_enable_stall_handshake();
            Usb_ack_receive_setup();
         }
      }
      else
      {
         Usb_enable_stall_handshake();
         Usb_ack_receive_setup();
      }
    break;

  default:
    Usb_enable_stall_handshake();
    Usb_ack_receive_setup();
    break;
   }
}


//! usb_clear_feature.
//!
//! This function manages the SET FEATURE request.
//!
//! @warning Code:xx bytes (function code length)
//!
//! @param none
//!
//! @return none
//!
void usb_clear_feature(void)
{
U8 wValue;
U8 wIndex;
U8 dummy;

   if (bmRequestType == USB_SETUP_SET_STAND_DEVICE)
   {
     wValue = Usb_read_byte();
      if ((wValue == FEATURE_DEVICE_REMOTE_WAKEUP) && (USB_REMOTE_WAKEUP_FEATURE == ENABLED))
     {
       device_status &= ~USB_STATUS_REMOTEWAKEUP;
         remote_wakeup_feature = DISABLED;
       Usb_ack_receive_setup();
       Usb_send_control_in();
     }
     else
     {
      Usb_enable_stall_handshake();
      Usb_ack_receive_setup();
     }
      return;
   }
   else if (bmRequestType == USB_SETUP_SET_STAND_INTERFACE)
   {
      //!< keep that order (set StallRq/clear RxSetup) or a
      //!< OUT request following the SETUP may be acknowledged
      Usb_enable_stall_handshake();
      Usb_ack_receive_setup();
      return;
   }
   else if (bmRequestType == USB_SETUP_SET_STAND_ENDPOINT)
   {
      wValue = Usb_read_byte();
      dummy  = Usb_read_byte();                //!< dummy read

      if (wValue == FEATURE_ENDPOINT_HALT)
      {
         wIndex = (Usb_read_byte() & MSK_EP_DIR);

         Usb_select_endpoint(wIndex);
         if(Is_usb_endpoint_enabled())
         {
            if(wIndex != EP_CONTROL)
            {
               Usb_disable_stall_handshake();
               Usb_reset_endpoint(wIndex);
               Usb_reset_data_toggle();
            }
            Usb_select_endpoint(EP_CONTROL);
            endpoint_status[wIndex] = 0x00;
            Usb_ack_receive_setup();
            Usb_send_control_in();
         }
         else
         {
            Usb_select_endpoint(EP_CONTROL);
            Usb_enable_stall_handshake();
            Usb_ack_receive_setup();
            return;
         }
      }
      else
      {
         Usb_enable_stall_handshake();
         Usb_ack_receive_setup();
         return;
      }
   }
}



//! usb_get_interface.
//!
//! TThis function manages the SETUP_GET_INTERFACE request.
//!
//! @warning Code:xx bytes (function code length)
//!
//! @param none
//!
//! @return none
//!
void usb_get_interface (void)
{
   Usb_ack_receive_setup();
   Usb_send_control_in();

   while( !Is_usb_receive_out() );
   Usb_ack_receive_out();
}

//! usb_set_interface.
//!
//! TThis function manages the SETUP_SET_INTERFACE request.
//!
//! @warning Code:xx bytes (function code length)
//!
//! @param none
//!
//! @return none
//!
void usb_set_interface (void)
{
  Usb_ack_receive_setup();
  Usb_send_control_in();                    //!< send a ZLP for STATUS phase
  while(!Is_usb_in_ready());
}

//! usb_generate_remote_wakeup
//!
//! This function manages the remote wake up generation
//!
//! @param none
//!
//! @return none
//!
void usb_generate_remote_wakeup(void)
{
   if(Is_pll_ready()==FALSE)
   {
      Pll_start_auto();
      Wait_pll_ready();
   }
   Usb_unfreeze_clock();
   if (remote_wakeup_feature == ENABLED)
   {
      Usb_initiate_remote_wake_up();
      remote_wakeup_feature = DISABLED;
   }
}  




//! bin_to_ascii
//!
//! This function is used to convert a 4 bit number into an ascii character
//! 5 => '5'       10 => 'A'   
//! 
//! @param binary value to convert
//!
//! @return converted character
//!
U8 bin_to_ascii (U8 b)
{
return ( (b <= 0x09) ? (b+'0') : (b+'A'-10) );
}
