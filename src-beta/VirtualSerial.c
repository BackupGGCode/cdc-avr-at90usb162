/*
             LUFA Library
     Copyright (C) Dean Camera, 2011.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the VirtualSerial demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "VirtualSerial.h"

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,

				.DataINEndpointNumber           = CDC_TX_EPNUM,
				.DataINEndpointSize             = CDC_TXRX_EPSIZE,
				.DataINEndpointDoubleBank       = false,

				.DataOUTEndpointNumber          = CDC_RX_EPNUM,
				.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
				.DataOUTEndpointDoubleBank      = false,

				.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
				.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
				.NotificationEndpointDoubleBank = false,
			},
	};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs
 */
static FILE USBSerialStream;


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	/* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	sei();

	for (;;)
	{
		//SendSpecificString();
		ReadFromStreamAndWriteToPin();
		
		/* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
		CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
	
	/* Pin Out Configuration */
	DDRB = 0xff; // All portB pins are configured as output
	PORTB = 0x00;
}

void ReadFromStreamAndWriteToPin(void){
	char ReportString[4]; // over dimension to allow for NUL
	char* myConstant = "123";
	char* itWorkedStr = "It worked!\r\n";
	char* itDidNotWork = "It did not work!\r\n";
	
	fgets(ReportString,3,&USBSerialStream);
	ReportString[3] = 0; // terminate data to make C string

	if (strncmp(ReportString, myConstant, 3) == 0) {
		PORTB = 0xff;
		fputs(itWorkedStr,&USBSerialStream);
	}		
	else{
		PORTB = 0x00;
		fputs(itDidNotWork,&USBSerialStream);
	}
	/*
	char* ReportString;
	char* BufferString;
	bool talkFlag = false;
	
	// Blink led
	//for (int i=0;i<=5;i++){
		PORTB = 0xff;
		_delay_ms(100);
		PORTB = 0x00;
		_delay_ms(100);
	//}
	
	//ReportString = CDC_Device_ReceiveByte(&USBSerialStream);
	fgets(ReportString,sizeof(ReportString),&USBSerialStream);
	
	if (ReportString != BufferString){
		fputs(ReportString, &USBSerialStream);
		BufferString = ReportString;
	}
	

	/*else{
		if (talkFlag == false){
			ReportString = "No use\n";
			fputs(ReportString,&USBSerialStream);
			talkFlag = true;
		}
	}*/
	
	
	
	/*if (ReportString = "a"){
		PORTB = 0xff;
	}
	else{
		PORTB = 0x00;
	}*/
	
}

/** Checks for changes in the position of the board joystick, sending strings to the host upon each change. */
void SendSpecificString(void)
{
	char*       ReportString  = "data packet\r\n";
	fputs(ReportString, &USBSerialStream);

	/* Alternatively, without the stream: */
	// CDC_Device_SendString(&VirtualSerial_CDC_Interface, ReportString);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

