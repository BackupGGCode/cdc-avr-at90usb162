/*This file has been prepared for Doxygen automatic documentation generation.*/
//! \file *********************************************************************
//! This file containsSPI lib header file.
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

#ifndef _SPI_LIB_H_
#define _SPI_LIB_H_

/*_____ I N C L U D E - F I L E S ____________________________________________*/
#include "lib_mcu/spi/spi_drv.h"

/*_____ C O N F I G U R A T I O N _________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/


//#ifndef SPI_CONFIG
//#error You must enter SPI_CONFIG in config.h
//#define SPI_CONFIG
//#endif

/**
 * @brief This enumeration allows to define a MASTER or SLAVE configuration
 **/
typedef enum {SPI_MASTER, SPI_SLAVE} spi_cf_t;

/*_____ D E C L A R A T I O N ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ P R O T O T Y P E S ____________________________________________________________*/

/**
 * @brief This function configures the SPI controller:
 * -# MASTER or SLAVE
 * -# bit timing
 * -# enable the controller
 *
 * @param configuration of the node (MASTER or SLAVE).
 * @param configuration of mode (SPI_MASTER_MODE_0...SPI_MASTER_MODE_3 or SPI_SLAVE_MODE_0...SPI_SLAVE_MODE_3).
 *
 * @return status of the init:
 * -# TRUE
 * -# FALSE
 *
 * @pre before calling this function some declaration must be define in config.h:\n
 * - SPI_CONFIG select the prescaler, CPHA leading, CPOL LOW, LSB first.
 */
bit     spi_init        (spi_cf_t config);

/**
 * @brief This function sends a byte on the SPI
 *
 * @param character to send on the SPI.
 *
 * @return character sent
 *
 */
char    spi_putchar        (char uc_wr_byte);

/**
 * @brief This function checks if a bytes has been received on the SPI
 *
 * @return TRUE if byte received
 *
 */
bit     spi_test_hit       (void);

/**
 * @brief This function reads a byte on the SPI
 *
 * @return character read
 *
 */
char    spi_getchar        (void);

//***************************************************************************
//  @fn SPI_Transmit_Master
//!
//! SPI Make the transmission possible
//!
//! @warning See SPI section in datasheet
//!
//! @param (char cData)
//!
//! @return nothing.
//!
//***************************************************************************
void SPI_Transmit_Master(char cData);

//***************************************************************************
//  @fn spi_rw
//!
//! @param (char tx), the data byte to send
//!
//! @return (char) the received byte.
//!
//***************************************************************************
char spi_rw(char tx);

#endif /* _SPI_LIB_H_ */
