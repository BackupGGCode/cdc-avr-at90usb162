/* Copyright (c) 2007 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 * $Rev: 1864 $
 *
 */

/** @file
 * Interface functions for the Frequency Agility Protocol.
 *
 * @author Lasse Olsen
 *
 * @defgroup nordic_protocol_fap Frequency Agility Protocol
 * @{ 
 * Implementation of frequency agility protocol for better robustness against
 * interference from co-existing radio frequency sources.  
 */
#ifndef FAP_H__
#define FAP_H__

//#include <stdint.h>
//#include <stdbool.h>
#include "config.h"
#include "fap_setup.h"
#include "fap_macros.h"
//#include "hal_nrf.h"
#include "FAPtoNRF_API.h"

//***************************************************************************** 
// FAP static parameters
//*****************************************************************************

#define FAP_DATARATE HAL_NRF_2MBPS    // 2Mbps datarate 

#define FAP_OUTPUT_POWER HAL_NRF_0DBM // 0 dBm output power 
#define FAP_CRC HAL_NRF_CRC_16BIT     // 16 bit CRC (maximum)
#define FAP_ADDRESS_WIDTH 5           // 5 byte address
#define FAP_LP_RX_LISTEN_PERIODS 1    // Low power RX on time                                       

#if(FAP_MAX_FW_PL_LENGTH > FAP_MAX_ACK_PL_LENGTH)
  #define FAP_MAX_PL_LENGTH FAP_MAX_FW_PL_LENGTH 
#else
  #define FAP_MAX_PL_LENGTH FAP_MAX_ACK_PL_LENGTH 
#endif
         
#if(FAP_MAX_ACK_PL_LENGTH > 15)
  #define FAP_AUTO_RETR_DELAY 500       
#else
  #define FAP_AUTO_RETR_DELAY 250       
#endif  

#define FAP_MAX_TX_PERIOD (130+37+(FAP_MAX_FW_PL_LENGTH*4)+FAP_AUTO_RETR_DELAY)    

//***************************************************************************** 
// Misc. macros
//*****************************************************************************

#define FAP_GET_BIT(a, b) ((a >> b) & 0x01)
#define FAP_CLEAR_BIT(a, b) (a &= ~(1 << b))
#define FAP_SET_BIT(a, b) (a |= (1 << b)) 

//***************************************************************************** 
// Derived parameters
//*****************************************************************************

/**
Specifies the radio wakeup from power down time. This constant is specifies in
number of FAP_RX_PERIODs. 
*/
#define FAP_RADIO_PWR_UP_DELAY ((2000/FAP_RX_PERIOD)+1)   // ~ 2 ms radio startup

/**
Equals the number of transmit periods the transmitter resides at one channel before
channel switch. This period equals the duration between the start of each 
retransmit, and is derived form the retransmit delay and "on air" packet length.
The maximum length of one transmit period in [us] is specified by #FAP_MAX_TX_PERIOD.
*/
#define FAP_TX_SINGLE_CH_REV ((FAP_NRF_AUTO_RETRIES+1)*FAP_AUTO_RETRIES_EXTEND)//[#]  

/*
Equals the maximum time in [us] the transmitter resides on a single channel before 
channel switch. This parameter is derived from the device maximum payload length.
*/  
#define FAP_TX_SINGLE_CH_REV_TIME (FAP_TX_SINGLE_CH_REV*FAP_TX_PERIOD_MAX)//[us]

/*
Equals the number of transmit periods necessary  for a full revolution
through all channels. It is recommended that the transmit timeout period 
specified when using fap_tx_data() is equal or greater than this value.
The maximum length of one transmit period in [us] is specified by #FAP_MAX_TX_PERIOD.
*/
#define FAP_TX_FULL_CH_REV (FAP_TX_SINGLE_CH_REV*FAP_CH_TAB_SIZE)//[#]

/*
Equals the time in [us] necessary for the transmitter to revolve through 
all channels.
*/
#define FAP_TX_FULL_CH_REV_TIME (FAP_TX_FULL_CH_REV*FAP_TX_PERIOD_MAX)//[us]

/**
Equals the number of receive periods the transmitter
resides on a single channel before channel switch. 
The length of one transmit period in [us] is specified by FAP_RX_PERIOD.
*/
#define FAP_RX_SINGLE_CH_REV ((uint16_t)(((FAP_TX_SINGLE_CH_REV*FAP_MAX_TX_PERIOD)/FAP_RX_PERIOD)+1))//[#]

/**
Function used internally by the FAP for a device to synchronize to the host receive channel rotation. 
This function must be customized for the actual MCU implementation. It is required from this 
function that when issued, the FAP protocol timer shall be adjusted to generate an interrupt 
FAP_TIMER_MODIFY_PERIOD microseconds after the point issued. Subsequently this interrupt the 
protocol timer shall retain the original setup (timeout every FAP_RX_PERIOD us).
*/
void fap_modify_timer_period(void);
          
/**
Specifies the time in [us] that equals the time the transmitter
resides on a single channel before channel switch.
Thus, FAP_RX_SINGLE_CH_REV_TIME=FAP_TX_SINGLE_CH_REV_TIME.      
*/
#define FAP_RX_SINGLE_CH_REV_TIME (FAP_RX_SINGLE_CH_REV*FAP_RX_PERIOD) //[us]

/**
Equals the number of receive periods the transmitter
uses on a full revolution through all channels. The length of one transmit 
period in [us] is specified by FAP_RX_PERIOD.     
*/
#define FAP_RX_FULL_CH_REV (FAP_RX_SINGLE_CH_REV*FAP_CH_TAB_SIZE) // [#]

/**
Equals the time in [us] that the transmitter uses on 
a full revolution through all channels. Thus
FAP_RX_FULL_CH_REV=FAP_TX_FULL_CH_REV.    
*/
#define FAP_RX_FULL_CH_REV_TIME (RX_FULL_CH_REV*FAP_RX_PERIOD) // [us]

//***************************************************************************** 
// Typedefs
//*****************************************************************************

/**
Type used for storing payload, payload length and pipe number    
*/
typedef struct
{
  uint8_t pl[FAP_MAX_PL_LENGTH];  
  uint8_t pl_length;
  uint8_t pipe;
} fap_tx_rx_struct_t;

/**
Possible FAP modes. Return values for function fap_get_mode().    
*/
typedef enum
{
  FAP_IDLE,
  FAP_TRANSMITTING,
  FAP_RECEIVING
}fap_modes_t;

/**
Possible RX fifo states. Return values for function fap_get_rx_fifo_status().    
*/
typedef enum
{
  FAP_RX_FIFO_EMPTY,
  FAP_RX_DATA_IN_FIFO,
  FAP_RX_FIFO_FULL,
}fap_rx_status_t;

/**
Possible radio modes during FAP IDLE mode. Input parameters for 
function fap_select_radio_idle_mode().    
*/
typedef enum
{
  FAP_PDOWN_IDLE,
  FAP_STANDBY_IDLE 
}fap_radio_idle_modes_t;

#define FAP_DEVICE0 HAL_NRF_PIPE0
#define FAP_DEVICE1 HAL_NRF_PIPE1
#define FAP_DEVICE2 HAL_NRF_PIPE2
#define FAP_DEVICE3 HAL_NRF_PIPE3
#define FAP_DEVICE4 HAL_NRF_PIPE4
#define FAP_DEVICE5 HAL_NRF_PIPE5
#define FAP_LP_RX_BIT (FAP_DEVICE5+1)

//***************************************************************************** 
// Function prototypes and function parameter defines
//*****************************************************************************

/** @name General functions */
//@{
/**
Initialization function for the frequency agility protocol (FAP). This function
must be called at least once before any of the remaining FAP functions may be used. 
It is recommended that the FAP timer interrupt service routine is not 
enabled until after the FAP is initialized.
*/
void fap_init(void);

/**
Function for setting the explicit receive address for a given pipe.

The radio can monitor up to 6 addresses (pipes) simultaneously. The addresses
for pipe 0 and 1 are specified using 5 bytes, while the address for pipe 2-5 
are specified using a 1 byte address. This 1 byte address specifies 
the LSByte address whilst the remaining address bytes for these 
pipes will equal those for pipe 1.

Note that the FAP must be in IDLE mode in order for this function to 
have any effect.

@param dev specifies the pipe number (0-5) for which to set the address.    

@param adr is a pointer to the address to be used for the given pipe. 
This address is copied and stored in the radio hardware, 
thus the address does not need to reside on this location during receive.   

@return
@retval false if failure due to the FAP not beeing in IDLE prior to function call.
@retval success.
     
@sa fap_rx_data(), fap_tx_data(), fap_goto_idle_mode()
*/
bool fap_set_address(uint8_t dev, uint8_t* adr); 

/**
Function for reading the address set for a given pipe. 

@param dev specifies the pipe number(0-5) for which to get the address.    

@param adr is a pointer to where the address shall be written. For pipe 1-2 5 bytes 
are returned, whilst for the remaining pipes a single byte is returned.
  
@sa  fap_set_address()
*/
void fap_get_address(uint8_t dev, uint8_t* adr);  

/**
Function for setting up the subset of channels to be used by the FAP.

Note that the receiver and the transmitter must be set up using the same subset of channels. 
The number of channels to be used is specified by the constant 
FAP_CH_TAB_SIZE in fap_setup.h. 

It is recommended that the selected channels are distributed over a 
wide frequency range. The possible channel range 
for nRF24L01 is 0 to 123. 

@param channels is a pointer to an array holding the FAP_CH_TAB_SIZE number of
channels to be used.  
*/
void fap_set_channels(uint8_t *channels);
      
/**
Function for selecting in which mode the radio shall reside when
the FAP is in IDLE mode. The nRF radio may reside in two different modes 
when not transmitting or receiving (FAP IDLE), 
which are STANDBY or POWER DOWN, respectively.
In STANDBY the radio consumes more power than in POWER DOWN, but 
the radio start-up time is shorter, which will result in a lower transmit 
latency when starting a new transmit operation. When the radio resides
in power down one must account for an increased latency of typically 1.5 ms. 

The FAP must be in IDLE mode in order for this function to have any effect.

@param mode selects the radio mode.    
Possible arguments are: 
@arg @c FAP_PDOWN_IDLE   
@arg @c FAP_STANDBY_IDLE 

@return 

@retval false if failure due to the FAP not being in IDLE prior to function call. 
@retval true if function executed successfully. 

*/
bool fap_select_radio_idle_mode(fap_radio_idle_modes_t mode); // Power down / standby

/**
Function for forcing the FAP to IDLE mode. 
The FAP have three different operation modes; TRANSMITTING,
RECEIVING or IDLE, respectively. When the FAP is receiving 
or transmitting it may automatically return to IDLE mode after 
a specified timeout period, or in TRANSMIT mode also after transmission
success. 

In addition, this function can be ysed to immediately 
force the FAP to IDLE mode.
*/
void fap_goto_idle_mode(void);

/**
Function for observing the current FAP mode.

@return 
Returns the current the FAP mode.
@retval FAP_IDLE
@retval FAP_TRANSMITTING 
@retval FAP_RECEIVING

@sa fap_rx_data(), fap_tx_data(), fap_goto_idle_mode()
*/
fap_modes_t fap_get_mode(void);  

//@}

/** @name Transmit related functions */
//@{

/**
Function for transmitting data. This function transmits the data provided in
the @b pl field of the input data structure *datainput to the destination given by 
the @b pipe field. The actual TX address to be used is setup using the function 
fap_set_address().

When the data is successfully transmitted the FAP will automatically return to IDLE mode. 
In addition it may be specified a maximum timeout period the FAP shall try to transmit 
data before returning to IDLE mode. 

It is not required for the FAP to be in IDLE mode before issuing this function. It is possible to upload new data to the TX FIFO during an ongoing transmission. The data uploaded to the TX FIFO will immediately be transmitted when the ongoing transmission is completed, thus enabling a streaming type of operation. However, in order to upload TX data during an ongoing transmission, the following criteria must be fulfilled:

-   Channel synchronization must be disabled by fap_ch_sync_enable().
-   The destination for the uploaded data must be the same as for the ongoing transmission.
-   The TX FIFO cannot be full. The TX FIFO can hold up to 3 payloads.
-   The transmit timeout specified must be the same as for the ongoing transmission.  
 
If the FAP is in RECEIVE mode when issuing this function, the FAP will automatically enter 
TRANSMIT mode. After completion the FAP will always go to IDLE mode. 

@param *datainput [in] is a pointer to a structure containing the payload to be transmitted, 
the payload length and the transmit destination. 

@param tx_timeout specifies the maximum period the FAP shall attempt to
transmit before returning to IDLE mode. The maximum timeout period will 
equal transmit_timeout*FAP_MAX_TX_PERIOD [us]. An infinite transmit 
period may be specified by setting this parameter to 0. 

@return
Returns whether the data was successfully handed to the FAP for transmission.    
@retval true if data successfully handed to FAP. The data will be attempted transmitted by the FAP.
@retval false if data were not successfully handed to the FAP due to one or more of the 
required criteria not fulfilled. Any ongoing FAP operation is not affected and the FAP
mode remains the same as before the function was called.    
     
@sa fap_get_mode(), fap_set_address(), fap_tx_success(), fap_goto_idle_mode(void), fap_ch_sync_disable().
*/
bool fap_tx_data(fap_tx_rx_struct_t *datainput, uint16_t tx_timeout);

/**
Function for flushing the hardware transmit FIFO. This yields any pending data in the TX FIFO,
both normal device to host data, and host to device preloaded acknowledge data.

It is recommended that the FAP is in IDLE mode whenever this function is called.

*/
void fap_flush_tx_fifo(void);

/**
Function for writing data to be piggybacked on the next acknowledge packet sent from the host to a device. This function will not initiate a transmission, but only prepare the data to be returned on the next package received from the selected device.  

@param *ackdata [in] is a pointer to the input data structure containing the payload to be transmitted, the payload length and the transmit destination. 
*/
bool fap_write_ack_pload(fap_tx_rx_struct_t* ackdata);

/**
Function returning @b false if TX FIFO full, else @ true.
*/
bool fap_tx_fifo_full(void);

/**
Function for setting the radio output power.

@param power selects the output power.    
Possible arguments are: 
@arg @c HAL_NRF_0DBM for 0 dBm output power
@arg @c HAL_NRF_6DBM for -6 dBm output power
@arg @c HAL_NRF_12DBM for -12 dBm output power
@arg @c HAL_NRF_18DBM for -18 dBm output power
*/
void fap_set_output_power(hal_nrf_output_power_t power);

/**
Function for enabling a new transmission to be started synchronously to the
receiver channel rotation.   

When synchronization is disabled, any new transmission will be started
synchronously with an internal "frequency guess counter". This synchronization must 
be enabled whenever using the function fap_ch_sync_enable().

@sa fap_get_ch_offset(), fap_ch_sync_disable()
*/
void fap_ch_sync_enable(void);

/**
Function for getting the offset by the "guessed" receive channel and the 
previous successful transmit channel.

The basic concept for the FAP is that the receiver continuously monitors 
a subset of channels in a rotating fashion. After a successful transmission the 
transmitting always adjusts an internal "guess counter" to equal this
receive channel rotation. A new transmission is always started using the 
previous successful channel. When awaiting to send new data until this 
function equals 0, the data will be transmitted using the same 
frequency as the receiver most likely monitors, thus minimizing 
the number of retransmits.

Note, when using this functionality, the channel synchronization must be enabled by 
fap_ch_sync_enable().

@sa fap_get_ch_offset(), fap_ch_sync_disable()
*/
uint8_t fap_get_ch_offset(void);

/**
Function for disabling synchronization of transmission frequency to receive 
frequency.

When synchronization is disabled, any new transmission will start
immediately after calling fap_tx_data(). 

@sa fap_get_ch_offset(), fap_ch_sync_enable()
*/
void fap_ch_sync_disable(void);

/**
Function for getting the result of the previous transmit operation.
The return value from this function is only credible when the FAP is 
in IDLE mode 

@return
Result of the previous transmit operation. 

@retval true if previous data were successfully transmitted, or 
if transmission in progress. (FAP mode TRANSMITTING). 
@retval false if previous were not successfully transmitted   
@sa fap_tx_data(), fap_get_mode(), fap_goto_idle_mode(void)
*/
bool fap_tx_success(void); 

/**
Function for providing the number of transmission
attempts needed during the previous transmit operation. 
This function may be used as an indication on
the current radio transmit conditions. In a noisy 
environment one will experience an increase in the 
number of transmission attempts needed for a 
successful packet delivery to the receiver. Note that 
even in an environment without any interfering radio
sources one will experience that the FAP sometimes
uses several transmission attempts. This due to the
frequency altering scheme used at the receiver.      

The returned value from this function is only reliable 
when the FAP is in IDLE mode, and will only yield the
previous transmitted package.

@sa fap_get_ch_switches(), fap_get_ch_offset(), 
fap_ch_sync_disable(), fap_ch_sync_disable()  
*/
uint16_t fap_get_tries(void);

/**
Function for providing the number of 
frequency channel changes needed during the previous 
transmit operation. In an environment without any 
interfering radio sources one will not normally 
experience that the FAP needs to switch transmission 
frequency. A large number of channel switches
will be an indication on extensive radio 
interference.

The returbe value from this function is only reliable 
when the FAP is in IDLE mode.  

@sa fap_get_tries()
*/
uint16_t fap_get_ch_switches(void);

//@}

/** @name Receive related functions */
//@{
/**
Function for setting the FAP in receive mode and start monitoring the air for data. 
Received data will be stored in a FIFO which can be read out by the function
fap_read_rx_fifo(). It may be specified a timeout period after which 
the FAP shall automatically stop receiving and return to IDLE mode.

The FAP offers two different receive modes; one low power mode and one low latency mode.
In low power mode the FAP is monitoring the air for data in only short
slots of time and the receiver is switched off most of the time. 
This leads to a low power consumption at the receive side but a higher latency
experienced at the transmit side as the transmitter has to "struggle"
more to hit the the time slots the receiver is listening. In low
latency mode the receiver is continuously monitoring the air for data, 
which gives a higher power consumption at the receive side but a lower 
experienced latency at the transmit side.
The FAP do not have to be in IDLE mode before using this function. If 
the FAP is in receive mode when this function is used the new 
receive settings will apply immediately.     

@param rx_setup selects receive power mode and pipe(s)/address(es) to 
monitor for incoming data. Bit 0-5 enables/disables receive
pipes/addresses 0-5; 1 enables reception and 0 disables 
reception. Bit 6 selects the receive power mode; 1 sets receiver 
in low power mode and 0 sets the receiver in low latency mode.    

@param receive_timeout specifies the receive timeout period. The timeout period
will be equivalent to receive_timeout*FAP_RX_PERIOD [us]. By setting this 
parameter to 0 the receiver will remain in receive mode until it
is forced to IDLE mode by fap_goto_idle_mode(). If a finite timeout
is specified, it is recommended that the timeout value is set equal or 
grater than FAP_RX_FULL_CH_REV when operating in low latency mode or
 FAP_RX_SINGLE_CH_REV when operating in low power mode.

@sa fap_set_address(), fap_get_mode(), fap_get_rx_fifo_status(), fap_read_rx_fifo(), fap_select_rx_power_mode, 
fap_goto_idle_mode()
*/
void fap_rx_data(uint8_t rx_setup, uint16_t receive_timeout); 

/**
Function for flushing the hardware receive FIFO. 

It is recommended that the FAP is in IDLE mode whenever this function is called.

*/
void fap_flush_rx_fifo(void);

/**
Function for reading received data from receive FIFO.
When the FAP is in receive mode all incoming data will
be stored in a FIFO which can be read using this function.

@param return_struct specifies the destination to where FIFO 
read data shall be copied. This is a structure type fap_tx_rx_struct_t
which contains the field pl for the actual payload data, pl_length
for the received payload length and pipe which tells the pipe
in which the data were received.  

@return 
Information on whether FIFO contained data or not.  
@retval true if unread FIFO data were copied to destination.
@retval false if FIFO was empty and no data were copied.  
@sa fap_rx_data(), fap_get_rx_fifo_status(), 
*/
bool fap_read_rx_fifo(fap_tx_rx_struct_t* return_struct);

//@}

/** @name Interrupt service routine functions */
//@{
/**
Function to be called by the nRF radio interrupt service routine.  
*/
void fap_nrf_isr_function(void);

/**
Function to be called by the FAP timer interrupt service routine.
The FAP requires one dedicated auto reload timer. This function must
be called by the FAP timer interrupt service routine. 
*/
void fap_timer_isr_function(void);
//@}
#endif // FAP_H__
/** @} */

