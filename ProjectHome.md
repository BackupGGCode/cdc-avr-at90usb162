# CDC Tx/Rx Communication Assets for AT90USB162 #

## Purpose ##
The purpose of this project is to implement a Communication Device Class communication protocol for using an Atmel AT90USB162 as a communication device from PC to microchip.

When connected to USB, the AT90USB162 shall enable a COMPORT and the PC shall be able to send and receive data using Hyperterminal or other COMPORT Interface Program.

The AT90USB162 uses the [Minimus AVR](http://minimususb.com/) platform.

New project members are welcome, since they contribute with its development.

## Project Status ##
Active (not completed).

## Development ##
For developing the application you will need [AVR Studio 5](http://www.atmel.com/microsite/avr_studio_5/default.asp?source=redirect) and the source present on this site. Check it out the Featured Wiki DevelopmentBoard.

## Source Architecture ##
The source architecture is:
  * `trunk` : Main development branch
  * `trunk\ext` : Extension codes from other projects.
  * `trunk\src` : Main source code development folder.
  * `branches` : Paralell development branches (if needed)
  * `tags` : Tags to the relevant milestones acomplished


## Based Code ##
There are two main branches this project can use to implement the communication assets. These code can be used to guide the development (or to give good ideas). All imported code (to this project) is converted to AVR Studio 5.

### 1. CDC Application Note 296 from Atmel ###
Details of this app note you can find at [AVR296: AVRUSBRF01 USB RF Dongle user's guide](http://www.atmel.com/dyn/resources/prod_documents/doc7808.pdf) and the original source code you can find at [AVR296.zip](http://www.atmel.com/dyn/resources/prod_documents/AVR296.zip)

The code converted to AVR Studio 5 you can find at `trunk\ext\cdc296-at90usb162`.

### 2. USART Application for AT90USB162 ###
Starting project to implement an USART using the AT90USB162.

You can find the code at `trunk\ext\usart-at90usb162`.

### 3. LUFA/MyUSB 111009 (Lightweight USB Framework for AVRs) ###
Stable release 111009 downloaded from [Four Wallet Cube](http://fourwalledcubicle.com/LUFA.php) (thanks to Dean Camera).

The imported code you can find at `trunk\ext\LUFA111009`.