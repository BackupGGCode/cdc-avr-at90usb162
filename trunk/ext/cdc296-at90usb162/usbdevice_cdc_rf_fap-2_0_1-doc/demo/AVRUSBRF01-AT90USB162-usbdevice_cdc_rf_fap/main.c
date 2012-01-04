/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! Main for USB application.
//! \brief
//!
//! - Compiler:           IAR EWAVR and GNU GCC for AVR
//! - Supported devices:  AT90USB162, AT90USB82
//!
//! \author               Atmel Corporation: http://www.atmel.com \n
//!                       Support and FAQ: http://support.atmel.no/
//!
//! ***************************************************************************
//!
//! @mainpage AT90USBxxx CDC demonstration
//!
//! @section intro License
//! Use of this program is subject to Atmel's End User License Agreement.
//!
//! Please read file  \ref lic_page for copyright notice.
//!
//! @section install Description
//! This embedded application source code illustrates how to implement a CDC application and wireless transmissions
//! with the AT90USBxxx controller. 
//!
//! This version implements the Frequency Agility Protocol (FAP) by Nordic Semiconductors, 
//! for better robustness against interference from co-existing radio frequency sources.
//!
//! @section sample About the sample application
//! By default this sample code is delivered configured for AVRUSBRF01 hardware with the AT90USB162
//! microcontrolor.
//! define value in config.h file.
//!
//! \image html AVRUSBRF01.jpg
//!
//! Hardware information about the AVRUSBRF01, can be found in <A HREF="../../lib_board/AVRUSBRF01/AVRUSBRF01_sch_layout.pdf">this document</A>
//!
//! This application will enumerates as a CDC (communication device class) virtual COM port. The application
//! can be used as a USB to serial converter.
//! The firmware is operationnal under the following operating systems:
//! - Windows 2000/XP.Vista (<A HREF="../../demo/AVRUSBRF01-AT90USB162-usbdevice_cdc_rf_fap/at90usbxxx_cdc.inf">at90usbxxx_cdc.inf</A> is required when the windows new hardware detection wizard prompts for an .inf driver file).
//! - Linux 2.4 and above
//! - MacOS X 10.4 and above
//! 
//! @section RF_null_modem Wireless Null Modem example
//! Two AVRUSBRF01 and their default CDC_rf_fap firmware can be used to emulate a "Wireless" Null Modem Cable.  
//!
//!-> Press the HWB button on both two AVRUSBRF01 before the first communication to make sure the frequency agility protocol (FAP) is running and synchronized.
//! \image html appli.gif
//!
//! @section src_code About the source code
//! This source code is usable with the following compilers:
//! - IAR Embedded Workbench (4.20a and higher)
//! - AVRGCC (WinAVR 20060421 and higher).
//!
//! Support for other compilers may required modifications or attention for:
//! - compiler.h file 
//! - special registers declaration file
//! - interrupt subroutines declarations
//!
//! @section arch Architecture
//! As illustrated in the figure bellow, the application entry point is located is the main.c file.
//! The main function first performs the initialization of a scheduler module and then runs it in an infinite loop.
//! The scheduler is a simple infinite loop calling all its tasks defined in the conf_scheduler.h file.
//! No real time schedule is performed, when a task ends, the scheduler calls the next task defined in
//! the configuration file (conf_scheduler.h).
//!
//! The sample dual role application is based on two different tasks:
//! - The usb_task  (usb_task.c associated source file), is the task performing the USB low level
//! enumeration process in device mode.
//! - The cdc_rf task performs the loop back application between USB and RF interfaces.
//!
//! \image html arch_full_fap.gif
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
#include "modules/scheduler/scheduler.h"
#include "lib_mcu/wdt/wdt_drv.h"
#include "lib_mcu/power/power_drv.h"
#include "lib_mcu/usb/usb_drv.h"




//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

int main(void)
{
  Usb_enable_regulator();
#ifndef  __GNUC__
   Wdt_off();
#else
   wdt_reset();
   Wdt_clear_flag();
   Wdt_change_enable();
   Wdt_stop();
#endif
   Clear_prescaler();
   scheduler();
   return 1;
}

//! \name Procedure to speed up the startup code
//! This one increment the CPU clock before RAM initialisation
//! @{
#ifdef  __GNUC__
// Locate low level init function before RAM init (init3 section)
// and remove std prologue/epilogue
char __low_level_init(void) __attribute__ ((section (".init3"),naked));
#endif

#ifdef __cplusplus
extern "C" {
#endif
char __low_level_init()
{
  Clear_prescaler();
  return 1;
}
#ifdef __cplusplus
}
#endif
//! @}

