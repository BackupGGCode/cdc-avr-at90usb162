/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! This file manages the USB task either device/host or both.
//! \brief
//!
//!  The USB task selects the correct USB task (usb_device task or usb_host task
//!  to be executed depending on the current mode available.
//!  According to USB_DEVICE_FEATURE and USB_HOST_FEATURE value (located in conf_usb.h file)
//!  The usb_task can be configured to support USB DEVICE mode or USB Host mode or both
//!  for a dual role device application.
//!  This module also contains the general USB interrupt subroutine. This subroutine is used
//!  to detect asynchronous USB events.
//!  Note:
//!    - The usb_task belongs to the scheduler, the usb_device_task and usb_host do not, they are called
//!      from the general usb_task
//!    - See conf_usb.h file for more details about the configuration of this module
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
#include "usb_task.h"
#include "lib_mcu/usb/usb_drv.h"
#if ((USB_DEVICE_FEATURE == ENABLED))
#include "usb_descriptors.h"
#endif
#include "lib_mcu/power/power_drv.h"
#include "lib_mcu/wdt/wdt_drv.h"
#include "lib_mcu/pll/pll_drv.h"
#include "modules/usb/device_chap9/usb_device_task.h"

#ifndef  USE_USB_PADS_REGULATOR
   #error "USE_USB_PADS_REGULATOR" should be defined as ENABLE or DISABLE in conf_usb.h file
#endif

//_____ M A C R O S ________________________________________________________

#ifndef LOG_STR_CODE
#define LOG_STR_CODE(str)
#else
U8 code log_device_disconnect[]="Device Disconnected";
U8 code log_id_change[]="Pin Id Change";
#endif

//_____ D E F I N I T I O N S ______________________________________________

//!
//! Public : U16 g_usb_event
//! usb_connected is used to store USB events detected upon
//! USB general interrupt subroutine
//! Its value is managed by the following macros (See usb_task.h file)
//! Usb_send_event(x)
//! Usb_ack_event(x)
//! Usb_clear_all_event()
//! Is_usb_event(x)
//! Is_not_usb_event(x)
volatile U16 g_usb_event=0;


#if (USB_DEVICE_FEATURE == ENABLED)
//!
//! Public : (bit) usb_connected
//! usb_connected is set to TRUE when VBUS has been detected
//! usb_connected is set to FALSE otherwise
//! Used with USB_DEVICE_FEATURE == ENABLED only
//!/
extern bit   usb_connected;

//!
//! Public : (U8) usb_configuration_nb
//! Store the number of the USB configuration used by the USB device
//! when its value is different from zero, it means the device mode is enumerated
//! Used with USB_DEVICE_FEATURE == ENABLED only
//!/
extern U8    usb_configuration_nb;

//!
//! Public : (U8) remote_wakeup_feature
//! Store a host request for remote wake up (set feature received)
//!/
extern U8 remote_wakeup_feature;

volatile U16 delay_usb;
void usb_delay_ms(U8 ms);
#endif


#if (USB_HOST_FEATURE == ENABLED)
//!
//! Private : (U8) private_sof_counter
//! Incremented  by host SOF interrupt subroutime
//! This counter is used to detect timeout in host requests.
//! It must not be modified by the user application tasks.
//!/
volatile U8 private_sof_counter=0;

   #if (USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE)
extern volatile S_pipe_int   it_pipe_str[MAX_EP_NB];
   #endif

#endif

#if ((USB_DEVICE_FEATURE == ENABLED)&& (USB_HOST_FEATURE == ENABLED))
//!
//! Public : (U8) g_usb_mode
//! Used in dual role application (both device/host) to store
//! the current mode the usb controller is operating
//!/
   U8 g_usb_mode=USB_MODE_UNDEFINED;
   U8 g_old_usb_mode;
#endif

//_____ D E C L A R A T I O N S ____________________________________________

/**
 * @brief This function initializes the USB process.
 *
 *  The function calls the coresponding usb mode initialization function
 *
 *  @param none
 *
 *  @return none
 */
void usb_task_init(void)
{
   #if (USE_USB_PADS_REGULATOR==ENABLE)  // Otherwise assume USB PADs regulator is not used
   Usb_enable_regulator();
   #endif
   usb_device_task_init();
#if (USB_REMOTE_WAKEUP == ENABLED)
   usb_remote_wup_feature = DISABLED;
#endif
}

/**
 *  @brief Entry point of the USB mamnagement
 *
 *  The function calls the coresponding usb management function.
 *
 *  @param none
 *
 *  @return none
*/
void usb_task(void)
{
   usb_device_task();
}

//! @brief USB interrupt subroutine
//!
//! This function is called each time a USB interrupt occurs.
//! The following USB DEVICE events are taken in charge:
//! - Start Of Frame
//! - Suspend
//! - Wake-Up
//! - Resume
//! - Reset
//!
//! For each event, the user can launch an action by completing
//! the associate define (See conf_usb.h file to add action upon events)
//!
//! Note: Only interrupts events that are enabled are processed
//!
//! @param none
//!
//! @return none
#ifdef __GNUC__
 ISR(USB_GEN_vect)
#else
#pragma vector = USB_General_vect
__interrupt void usb_general_interrupt()
#endif
{
  // - Device start of frame received
   if (Is_usb_sof() && Is_sof_interrupt_enabled())
   {
      Usb_ack_sof();
      Usb_sof_action();
   }
  // - Device Suspend event (no more USB activity detected)
   if (Is_usb_suspend() && Is_suspend_interrupt_enabled())
   {
      usb_suspended=TRUE;
      Usb_ack_wake_up();                 // clear wake up to detect next event
      Usb_send_event(EVT_USB_SUSPEND);
      Usb_ack_suspend();
      Usb_enable_wake_up_interrupt();
      Usb_disable_resume_interrupt();
      Usb_freeze_clock();
      Stop_pll();
      Usb_suspend_action();
   }
  // - Wake up event (USB activity detected): Used to resume
   if (Is_usb_wake_up() && Is_wake_up_interrupt_enabled())
   {
      if(Is_pll_ready()==FALSE)
      {
         Pll_start_auto();
         Wait_pll_ready();
      }
      Usb_unfreeze_clock();
      Usb_ack_wake_up();
      if(usb_suspended)
      {
         Usb_enable_resume_interrupt();
         Usb_enable_reset_interrupt();
         while(Is_usb_wake_up())
         {
            Usb_ack_wake_up();
         }
         usb_delay_ms(2);
         if(Is_usb_sof() || Is_usb_resume() || Is_usb_reset() )
         {
            Usb_disable_wake_up_interrupt();
            Usb_wake_up_action();
            Usb_send_event(EVT_USB_WAKE_UP);
            Usb_enable_suspend_interrupt();
            Usb_enable_resume_interrupt();
            Usb_enable_reset_interrupt();
            
         }
         else // Workarround to make the USB enter power down mode again (spurious transcient detected on the USB lines)
         {
            if(Is_usb_wake_up()) return;
            Usb_drive_dp_low();
            Usb_direct_drive_usb_enable();
            Usb_direct_drive_disable();
            Usb_disable_wake_up_interrupt();
         }
      }
   }
  // - Resume state bus detection
   if (Is_usb_resume() && Is_resume_interrupt_enabled())
   {
      usb_suspended = FALSE;
      Usb_disable_wake_up_interrupt();
      Usb_ack_resume();
      Usb_disable_resume_interrupt();
      Usb_resume_action();
      Usb_send_event(EVT_USB_RESUME);
   }
  // - USB bus reset detection
   if (Is_usb_reset()&& Is_reset_interrupt_enabled())
   {
      #if (USB_REMOTE_WAKEUP == ENABLED)
      usb_remote_wup_feature = DISABLED;
      #endif
      Usb_ack_reset();
      usb_init_device();
      Usb_reset_action();
      Usb_send_event(EVT_USB_RESET);
   }

}


void usb_delay_ms(U8 ms)
{
   for(;ms;ms--)
   {
      for(delay_usb=0;delay_usb<FOSC/16;delay_usb++);
   }
}
