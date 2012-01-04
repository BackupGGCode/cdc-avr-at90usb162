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
 * 
 * Setup parameters for Nordic Wireless Desktop Protocol (WDP) nRF24LU1 demo 
 *
 * @author Lasse Olsen 
 *
 * @ingroup nordic_protocol_wdp Wireless Desktop Protocol
 * @{
 */

#ifndef WDP_SETUP_H__
#define WDP_SETUP_H__

/** @name Application parameters*/
//@{

/**
Definition of the period the host shall enable pairing after power up.
*/
#define APP_PAIRING_TIMEOUT (5000000/FAP_RX_PERIOD)   // Pairing enabled 5 sec. after host startup

//@}

/** @name Common host/device parameters */
//@{

/** Defines the maximum uplink payload length to be used. The device application
should never exceed this number when writing uplink data using 
wdp_send_data().
*/  
#define WDP_MAX_UL_PL_LENGTH     32

/** Defines the maximum downlink payload length to be used. The host application
should never exceed this number when writing downlink data using 
wdp_write_downlink_data().
*/  
#define WDP_MAX_DL_PL_LENGTH      32 

/**
Definition of the global pairing address to be used for exchanging pairing information.
*/
#define WDP_PAIRING_ADDRESS {12, 15, 65, 32, 26}

/**
Definition of the global subset of channels to be used for exchanging pairing information.
*/
#define WDP_PAIRING_CHANNELS {3, 34, 54, 61, 78} 

/**
Definition of the initial value for the "random" master address generator.
*/
#define WDP_INIT_MASTER_ADDRESS {0, 0, 3, 3} 

/**
Definition of highest frequency channel to be used when generating channel subset.
*/
#define WDP_CH_HIGH 80

/**
Definition of highest frequency channel to be used when generating channel subset.
*/
#define WDP_CH_LOW 2

/**
Definition of the interval between each "keep alive" message for a mouse when 
continuous link activated.
*/
#define WDP_MOUSE_KA_INTERVAL (1000000/FAP_RX_PERIOD)

/**
Definition of the interval between each "keep alive" message for a keyboard when 
a continuous link activated.
*/
#define WDP_KEYBOARD_KA_INTERVAL (1000000/FAP_RX_PERIOD)

/**
Definition of the interval between each "keep alive" message for a remote control when 
continuous link activated.
*/
#define WDP_REMOTE_KA_INTERVAL (1000000/FAP_RX_PERIOD)

//@}

/** @name Device specific parameters */
//@{

/**
Definition of device type. This parameter must be set to equal the actual device implemented.   
*/
#define WDP_DEVICE_TYPE WDP_MOUSE

/**
Definition of the output power to be used by a device when sending a pairing request.
*/
#define WDP_PAIRING_OUTPUT_POWER HAL_NRF_18DBM

/**
Definition of the timeout period for application user data transmission.   
*/
#define WDP_TX_DATA_TIMEOUT (500000/FAP_MAX_TX_PERIOD)

/**
Definition of the timeout period for pairing request transmission.   
*/
#define WDP_TX_PAIRING_TIMEOUT (50000/FAP_MAX_TX_PERIOD)
 
//@}

/** @name Frequency Agility Protocol (FAP) parameters */
//@{

/**
Specifies the number of frequency channels to be used for the frequency agility
protocol. The number of channels should be set as high as possible in order to 
increase the robustness against co-existing radio sources. However, a large number 
of frequency channels will increase the average transmission latency. 
*/                
#define FAP_CH_TAB_SIZE 5   

/**
Specifies the Frequency channels to be used for the frequency agility
protocol. The number of channels should be the same as the FAP_CH_TAB_SIZE
defined above.
*/                

#define FAP_CHANNELS {24, 48, 72, 96, 120}

/**
Specifies the time in [us] the receiver shall listen on each channel during receive 
channel rotation.  
*/
#define FAP_RX_PERIOD 800

/**
Specifies the number of auto retransmit attempts to be used for the nRF radio.

*/
#define FAP_NRF_AUTO_RETRIES 11     

/**
Specifies a multiplication factor in order to extend the number of retransmits set up
by FAP_NRF_AUTO_RETRIES. The nRF radio has a limited number of auto retransmit attempts. 
The total number of transmit attempts on a single channel before channel switch will equal 
(FAP_NRF_AUTO_RETRIES+1)*FAP_AUTO_RETRIES_EXTEND.
*/
#define FAP_AUTO_RETRIES_EXTEND 2                

/**
Specifies the maximum payload length ever to be used for device to host transmission. 
*/
#define FAP_MAX_FW_PL_LENGTH WDP_MAX_UL_PL_LENGTH+1        // Assumes 1 byte reserved for protocol purposes

/**
Specifies the maximum payload length ever to be used for the ACK payload. This constant
is used to decide whether 250 or 500 us retransmit delay is to be used,
as a long ACK payload requires longer retransmit delay. 
*/
#define FAP_MAX_ACK_PL_LENGTH WDP_MAX_DL_PL_LENGTH         // [bytes]

/**
Specifies the number of #FAP_RX_PERIOD the receive channel rotation shall be halted
after data is received. It is recommended that this is set > 0 to ensure that
acknowledge transmission is completed before channel rotation continues. 
*/
#define FAP_RX_CH_HOLD_PERIODS 5

/**
Specifies the value to be used when adjusting the FAP timer after every successful
transmission. When using the synchronization mechanisms offered by the FAP, this value
must be tuned so that the the average number of retransmit equal ~0. (During good radio
conditions). A sennsible value using as a starting point for this value is given
as #FAP_RX_PERIOD - #FAP_MAX_PL_LENGTH. This parameter can only be optimized for one payload length.  

*/
//#define FAP_TIMER_MODIFY_PERIOD (FAP_RX_PERIOD - 395)
#define FAP_TIMER_MODIFY_PERIOD (FAP_RX_PERIOD - 150)


/**
Specifies the size of the software (SW) portion of the FAP receive FIFO. In receive mode, 
the received data is placed in a FIFO consisting of the hardware (HW) FIFO provided 
in the nRF radio in addition to a FIFO contained in the FAP SW. 
This constant specifies the number of FIFO levels for the FW FIFO. 
Thus, the total receive FIFO size will equal the number of
HW FIFO levels + FAP_FIFO_SIZE. The Nordic nRF24L01 provides 
a 3 level deep HW FIFO.
*/
//#define FAP_FIFO_SIZE 3                       // xxx

/**
Speceifies the duration between each "receive burst" when using the
receive low power mode. This constants is specified in units of 
FAP_RX_PERIODs which in turn are specified in [us]. Thus, the duration
between each "receive burst" will equal 
FAP_LP_RX_POLL_PERIOD*FAP_RX_PERIOD [us]. 
*/
#define FAP_LP_RX_POLL_PERIOD (500000/FAP_RX_PERIOD)

//@}

#endif // FAP_SETUP_H__
/** @} */ 
