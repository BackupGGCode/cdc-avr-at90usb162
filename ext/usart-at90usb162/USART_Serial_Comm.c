/*
 *  USART_Serial_Comm: communication program for AT90USB162
 *  Copyright (C) 2012 P. Rego <soong162@gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <avr/delay.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void USART_Init (){
	// Turn on the Tx and Rx circuitry
    UCSR1B |= (1 << RXEN1) | (1 << TXEN1);

	// Use 8-bit character sizes
	UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);

	// Set the baud rate
	UBRR1H = (BAUD_PRESCALE >> 8);
	UBRR1L = BAUD_PRESCALE;
}

void USART_Transmit (unsigned char data){
	// Wait for empty transmit buffer from USART
	while ( !(UCSR1A & (1 << UDRE1)));

	// Transmit the data
	UDR1 = data;
}

unsigned char USART_Receive (){
	// Wait for data to be received
	while ( !(UCSR1A & (1 << RXC1)));

	// Get and return received data from buffer
	return UDR1;
}

int main(void)
{
	USART_Init();
	char DataTransfer;
	while (1){
		DataTransfer = USART_Receive();
		USART_Transmit(DataTransfer);
	}
}
