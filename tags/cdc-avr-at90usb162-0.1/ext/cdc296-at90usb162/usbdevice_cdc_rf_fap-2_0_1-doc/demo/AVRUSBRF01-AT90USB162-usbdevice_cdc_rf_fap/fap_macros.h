/* Copyright (c) 2006 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 */

/** @file
 * Hardware dependent macros for FAP
 * @defgroup nordic_protocol_fap_macro Frequency Agility Protocol HW macros
 * @{
 * @ingroup nordic_protocol_fap
 * Hardware dependent macros for FAP. These macros enables and disables
 * interrupts the FAP depends on. They must be implemented according to
 * the hardware being used.
 */

#ifndef FAP_MACROS_H__
#define FAP_MACROS_H__



// Implementation for nRF24L01

#define FAP_INIT_TIMER()        (TCCR0A = 0x02, TCCR0B = 0x03, OCR0A  = 200) // Timer 0 CTC Mode with prescaler set to Clk/64 => 4µs time unit
#define FAP_NRF_INIT_IRQ()      (EICRB = 0x08)           // Set interrupt activation on falling edge
                         
 
#define FAP_NRF_IRQ_ENABLE()    (EIMSK = EIMSK | 0x20)   //enables external NRF interrupt
#define FAP_NRF_IRQ_DISABLE()   (EIMSK = EIMSK & 0xDF)   //disables external NRF interrupt
#define FAP_TIMER_IRQ_ENABLE()  (TIMSK0 = 0x02)          //enables the Fap timer IRQ
#define FAP_TIMER_IRQ_DISABLE() (TIMSK0 = 0x00)          //disables the Fap timer IRQ




#endif // FAP_MACROS_H__

/** @} */
